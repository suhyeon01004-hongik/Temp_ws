# MORAI path tracking

`morai_path_tracking` consumes a map-frame local path and localization odometry,
then publishes an autonomous `morai_udp_bridge/ActuatorCommand`. It deliberately
has no dependency on `vehicle_control`, `/vehicle/status`, or a direct speed topic.

## Controller model

`base_link` is the rear axle center: x points forward and y points left. For a
vehicle pose `(x_v, y_v, yaw)` and map point `(x_m, y_m)`, the controller uses

```
dx = x_m - x_v; dy = y_m - y_v
x_body = cos(yaw) * dx + sin(yaw) * dy
y_body = -sin(yaw) * dx + cos(yaw) * dy
```

Pure Pursuit selects a forward target at dynamic lookahead
`clamp(base + gain * abs(speed), min, max)` and applies the bicycle-model
steering angle `atan2(2 * wheelbase * target_y, target_x^2 + target_y^2)`,
clamped to the physical steering limit. The longitudinal controller is a PID
using derivative of measured speed: positive output becomes `accel`, negative
output becomes `brake`, so they are mutually exclusive.

The expected simulator setup has GPS and IMU noise disabled. Speed is read only
from `/localization/odometry.twist.twist.linear.x`.

## Interfaces

| Direction | Topic | Type | Units |
| --- | --- | --- | --- |
| Input | `/local_path` | `nav_msgs/Path` | map-frame metres |
| Input | `/localization/odometry` | `nav_msgs/Odometry` | map-frame pose; linear x in m/s |
| Output | `/control/actuator_command` | `morai_udp_bridge/ActuatorCommand` | accel/brake `[0,1]`; steering radians |

The node publishes at 30 Hz by default. Default target speed is **3.0 m/s**.

## Configuration

All private parameters are required and type-checked at startup. Defaults are in
`config/molit_2026_pure_pursuit.yaml`.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `local_path_topic` | `/local_path` | `nav_msgs/Path` input |
| `odometry_topic` | `/localization/odometry` | `nav_msgs/Odometry` input |
| `command_topic` | `/control/actuator_command` | actuator command output |
| `expected_frame_id` | `map` | required frame for both input headers |
| `control_rate_hz` | `30.0` | WallTimer publication rate |
| `path_timeout_sec` | `0.25` | path receipt and ROS-stamp age limit |
| `odometry_timeout_sec` | `0.25` | odometry receipt and ROS-stamp age limit |
| `maximum_input_skew_sec` | `0.10` | maximum path/odometry stamp separation |
| `safe_brake_command` | `0.50` | brake output in the safe state |
| `wheelbase_m` | `3.0` | rear-axle-to-front-axle distance |
| `lookahead_base_m` | `3.0` | zero-speed lookahead contribution |
| `lookahead_speed_gain_sec` | `0.5` | lookahead gain applied to speed |
| `lookahead_min_m` | `3.0` | lower lookahead clamp |
| `lookahead_max_m` | `6.0` | upper lookahead clamp |
| `minimum_target_distance_m` | `0.5` | shortest valid forward target |
| `maximum_steering_angle_deg` | `40.0` | physical steering magnitude limit |
| `target_speed_mps` | `3.0` | commanded longitudinal speed |
| `speed_kp` | `0.35` | speed PID proportional gain |
| `speed_ki` | `0.08` | speed PID integral gain |
| `speed_kd` | `0.02` | speed PID derivative-on-measurement gain |
| `speed_integral_limit` | `2.0` | absolute PID integral clamp |
| `speed_error_deadband_mps` | `0.05` | no-control speed-error band |
| `maximum_accel_command` | `0.40` | acceleration output cap |
| `maximum_brake_command` | `0.60` | PID braking output cap |

Start with the supplied low gains and `3.0 m/s`; validate steering sign and
speed response in MORAI before increasing speed or gains.

## Safety behavior

The controller resets PID and publishes `(accel=0, brake=0.5, steering=0)` for
missing, stale, frame-invalid, non-finite, or excessively skewed input; invalid
quaternion/timer `dt`; or no valid Pure Pursuit target. Future-stamped input is
also rejected. It never retains a steering command through a fault.

## Run and test

```bash
roslaunch morai_path_tracking pure_pursuit.launch
roslaunch morai_path_tracking pure_pursuit.launch config:=/path/to/controller.yaml

catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Do not run a separate manual controller or other sender against MORAI while the
autonomous controller is active.
