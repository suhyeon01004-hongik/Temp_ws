# Pure Pursuit Path Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS1 Pure Pursuit path follower that estimates speed from GPS position and IMU yaw, runs longitudinal PID, publishes actuator commands, and sends them to MORAI through `morai_udp_bridge`.

**Architecture:** `morai_localization` owns the filtered velocity estimate and publishes it in `/localization/odometry`. A new `morai_path_tracking` package consumes odometry and `/local_path`, computes steering plus accel/brake, and publishes the stamped `morai_udp_bridge/ActuatorCommand` contract. A separate control sender inside `morai_udp_bridge` validates, watches, serializes, and transmits the command without depending on `vehicle_control`.

**Tech Stack:** Ubuntu 20.04, ROS1 Noetic, catkin, roscpp, C++14, nav_msgs, geometry_msgs, std_msgs, message_generation, gtest, rostest, XML launch, YAML.

## Global Constraints

- Work only on branch `feat/pure-pursuit-controller`.
- Do not modify or depend on `vehicle_control`.
- Assume MORAI GPS and IMU sensor noise is disabled.
- Use GPS position delta plus IMU yaw; do not integrate IMU linear acceleration.
- Use `base_link` at the rear axle center, with `x` forward and `y` left.
- Use wheelbase `3.0 m` and clamp physical steering to `±40 deg`.
- Keep target speed and all tuning/safety thresholds in YAML, with initial target speed `3.0 m/s`.
- Keep accel and brake in `[0,1]` and mutually exclusive.
- Keep the existing `/local_path` and `/localization/odometry` interfaces.
- Update every changed package README and the root README.
- Preserve all existing sensor, localization, path, visualization, bringup, and manual-control tests.
- Implement with TDD and make one focused commit per task.

---

## File Map

### Velocity estimation

- `src/morai_localization/include/morai_localization/velocity_estimator.hpp`:
  dependency-free estimator API and result/config types.
- `src/morai_localization/src/velocity_estimator.cpp`: GPS delta, IMU-yaw
  projection, validity gates, and optional first-order filter.
- `src/morai_localization/src/localization_fusion_node.cpp`: ROS adapter that
  publishes odometry only when the estimator returns a valid velocity.
- `src/morai_localization/test/test_velocity_estimator.cpp`: estimator behavior.

### Control bridge

- `src/morai_udp_bridge/msg/ActuatorCommand.msg`: stamped controller-to-bridge
  interface.
- `src/morai_udp_bridge/include/morai_udp_bridge/control_protocol.hpp` and
  `src/control_protocol.cpp`: command validation and exact MORAI packet encoding.
- `src/morai_udp_bridge/include/morai_udp_bridge/udp_sender.hpp` and
  `src/udp_sender.cpp`: UDP datagram transport.
- `src/morai_udp_bridge/include/morai_udp_bridge/control_watchdog.hpp` and
  `src/control_watchdog.cpp`: stale-command selection.
- `src/morai_udp_bridge/src/control_sender_node.cpp`: ROS subscriber and 50 Hz
  wire sender.

### Path tracking

- `src/morai_path_tracking/include/morai_path_tracking/pure_pursuit.hpp` and
  `src/pure_pursuit.cpp`: lookahead selection and bicycle steering.
- `src/morai_path_tracking/include/morai_path_tracking/pid_controller.hpp` and
  `src/pid_controller.cpp`: longitudinal PID and accel/brake split.
- `src/morai_path_tracking/src/pure_pursuit_controller_node.cpp`: message
  validation, frame transform, freshness gates, timer, and command publishing.
- `src/morai_path_tracking/config/molit_2026_pure_pursuit.yaml`: all controller
  tunables.
- `src/morai_path_tracking/launch/pure_pursuit.launch`: standalone controller.

### Composition and documentation

- `src/morai_bringup/launch/molit_2026_autonomous.launch`: autonomous stack.
- Changed package READMEs and root `README.md`: contracts, commands, tuning,
  safety, and the no-concurrent-manual-sender warning.

---

### Task 1: Extract and validate the localization velocity estimator

**Files:**
- Create: `src/morai_localization/include/morai_localization/velocity_estimator.hpp`
- Create: `src/morai_localization/src/velocity_estimator.cpp`
- Create: `src/morai_localization/test/test_velocity_estimator.cpp`
- Modify: `src/morai_localization/src/localization_fusion_node.cpp`
- Modify: `src/morai_localization/config/molit_2026_kcity.yaml`
- Modify: `src/morai_localization/CMakeLists.txt`
- Modify: `src/morai_localization/README.md`

**Interfaces:**
- Consumes: consecutive GPS local points `(x_m, y_m, stamp_sec)` and synchronized
  IMU yaw `yaw_rad`.
- Produces:

```cpp
namespace morai_localization {

struct VelocityEstimatorConfig {
  double minimum_dt_sec{0.005};
  double maximum_dt_sec{0.25};
  double maximum_speed_mps{50.0};
  double filter_time_constant_sec{0.10};
};

struct VelocityEstimate {
  bool valid{false};
  double longitudinal_mps{0.0};
  double lateral_mps{0.0};
};

class VelocityEstimator {
 public:
  explicit VelocityEstimator(const VelocityEstimatorConfig& config);
  VelocityEstimate update(double x_m, double y_m, double stamp_sec,
                          double yaw_rad);
  void reset();
};

}  // namespace morai_localization
```

- Publishes valid `longitudinal_mps` and `lateral_mps` as
  `/localization/odometry.twist.twist.linear.{x,y}`.

- [ ] **Step 1: Add failing estimator tests**

Create tests with these exact cases:

```cpp
TEST(VelocityEstimator, NeedsTwoSamples) {
  VelocityEstimator estimator(VelocityEstimatorConfig{});
  EXPECT_FALSE(estimator.update(0.0, 0.0, 1.0, 0.0).valid);
  const auto estimate = estimator.update(1.0, 0.0, 1.5, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
  EXPECT_NEAR(0.0, estimate.lateral_mps, 1.0e-9);
}

TEST(VelocityEstimator, ProjectsMapVelocityWithImuYaw) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.0;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, M_PI_2);
  const auto estimate = estimator.update(0.0, 1.0, 1.5, M_PI_2);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(2.0, estimate.longitudinal_mps, 1.0e-9);
  EXPECT_NEAR(0.0, estimate.lateral_mps, 1.0e-9);
}

TEST(VelocityEstimator, AppliesTimeConstantFilter) {
  VelocityEstimatorConfig config;
  config.filter_time_constant_sec = 0.1;
  VelocityEstimator estimator(config);
  estimator.update(0.0, 0.0, 1.0, 0.0);
  ASSERT_TRUE(estimator.update(0.1, 0.0, 1.1, 0.0).valid);
  const auto estimate = estimator.update(0.3, 0.0, 1.2, 0.0);
  ASSERT_TRUE(estimate.valid);
  EXPECT_NEAR(1.5, estimate.longitudinal_mps, 1.0e-9);
}
```

Also cover signed reverse speed, `tau=0`, non-finite input, time reversal,
`dt < minimum`, `dt > maximum`, maximum-speed rejection, and `reset()`.

- [ ] **Step 2: Register the missing test target and verify failure**

Add a temporary `catkin_add_gtest(test_velocity_estimator
test/test_velocity_estimator.cpp)` target linked to a not-yet-created
`morai_velocity_estimator` library.

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make run_tests_morai_localization_gtest_test_velocity_estimator
```

Expected: configure or link failure because `velocity_estimator.cpp` and its
public symbols do not exist.

- [ ] **Step 3: Implement the estimator core**

Implement these rules:

```cpp
const double dt = stamp_sec - previous_stamp_sec_;
if (!std::isfinite(dt) || dt < config_.minimum_dt_sec ||
    dt > config_.maximum_dt_sec) {
  setBaseline(x_m, y_m, stamp_sec);
  clearFilter();
  return {};
}

const double vx_map = (x_m - previous_x_m_) / dt;
const double vy_map = (y_m - previous_y_m_) / dt;
const double cosine = std::cos(yaw_rad);
const double sine = std::sin(yaw_rad);
const double raw_longitudinal = cosine * vx_map + sine * vy_map;
const double raw_lateral = -sine * vx_map + cosine * vy_map;

if (std::hypot(vx_map, vy_map) > config_.maximum_speed_mps) {
  setBaseline(x_m, y_m, stamp_sec);
  clearFilter();
  return {};
}

const double alpha =
    config_.filter_time_constant_sec == 0.0
        ? 1.0
        : dt / (config_.filter_time_constant_sec + dt);
```

Initialize the filter from the first valid raw velocity, update the baseline
after every finite sample, and throw `std::invalid_argument` for invalid config
values.

- [ ] **Step 4: Run estimator tests**

Run:

```bash
catkin_make run_tests_morai_localization_gtest_test_velocity_estimator
catkin_test_results build/test_results/morai_localization --verbose
```

Expected: all velocity estimator tests pass.

- [ ] **Step 5: Integrate the estimator into localization**

Replace `updateRawVelocity()` and the raw velocity members in
`localization_fusion_node.cpp` with a `std::unique_ptr<VelocityEstimator>`.
Load these exact private parameters:

```cpp
private_node_.param("minimum_velocity_dt_sec", config.minimum_dt_sec, 0.005);
private_node_.param("maximum_velocity_dt_sec", config.maximum_dt_sec, 0.25);
private_node_.param("maximum_velocity_mps", config.maximum_speed_mps, 50.0);
private_node_.param("velocity_filter_time_constant_sec",
                    config.filter_time_constant_sec, 0.10);
```

In `publishIfReady()`, after yaw is known, call:

```cpp
const VelocityEstimate velocity = velocity_estimator_->update(
    gps_.point.x, gps_.point.y, gps_.header.stamp.toSec(), yaw);
```

Always publish pose and TF. Publish odometry only when `velocity.valid` is true,
setting linear x/y from the estimate and angular z from the IMU.

- [ ] **Step 6: Update localization YAML and README**

Set:

```yaml
minimum_velocity_dt_sec: 0.005
maximum_velocity_dt_sec: 0.25
maximum_velocity_mps: 50.0
velocity_filter_time_constant_sec: 0.10
```

Document the equations, GPS/IMU roles, LPF disable value `0`, validity/reset
behavior, odometry starting on the second valid point, and the noise-off
assumption.

- [ ] **Step 7: Run all localization tests and commit**

Run:

```bash
catkin_make run_tests_morai_localization
catkin_test_results build/test_results/morai_localization --verbose
```

Expected: existing UTM/IMU tests and new estimator tests pass.

Commit:

```bash
git add src/morai_localization
git commit -m "feat(localization): estimate filtered vehicle velocity"
```

---

### Task 2: Add the neutral actuator message and MORAI control protocol

**Files:**
- Create: `src/morai_udp_bridge/msg/ActuatorCommand.msg`
- Create: `src/morai_udp_bridge/include/morai_udp_bridge/control_protocol.hpp`
- Create: `src/morai_udp_bridge/src/control_protocol.cpp`
- Create: `src/morai_udp_bridge/include/morai_udp_bridge/udp_sender.hpp`
- Create: `src/morai_udp_bridge/src/udp_sender.cpp`
- Create: `src/morai_udp_bridge/test/test_control_protocol.cpp`
- Create: `src/morai_udp_bridge/test/test_udp_sender.cpp`
- Modify: `src/morai_udp_bridge/package.xml`
- Modify: `src/morai_udp_bridge/CMakeLists.txt`

**Interfaces:**
- Produces ROS message:

```text
std_msgs/Header header
float32 accel
float32 brake
float32 steering_angle_rad
```

- Produces C++ protocol API:

```cpp
struct ControlInput {
  float accel{0.0F};
  float brake{0.0F};
  float steering_angle_rad{0.0F};
};

struct ControlProtocolConfig {
  float maximum_steering_angle_rad{0.6981317008F};
  float steering_sign{1.0F};
  std::uint8_t drive_gear{4U};
};

using MoraiControlPacket = std::array<std::uint8_t, 55U>;

bool isValidControlInput(const ControlInput& input,
                         const ControlProtocolConfig& config,
                         std::string* error);
MoraiControlPacket encodeMoraiControlPacket(
    const ControlInput& input, const ControlProtocolConfig& config);
```

- Produces UDP API:

```cpp
class UdpSender {
 public:
  UdpSender(const std::string& destination_ip, std::uint16_t destination_port);
  ~UdpSender();
  void send(const std::uint8_t* data, std::size_t size);
};
```

- [ ] **Step 1: Add the message and failing protocol tests**

Create `ActuatorCommand.msg` exactly as shown above. Add protocol tests asserting:

```cpp
TEST(ControlProtocol, ConvertsPhysicalSteeringAngleToNormalizedPacketValue) {
  ControlProtocolConfig config;
  ControlInput input{0.5F, 0.0F, config.maximum_steering_angle_rad};
  const auto packet = encodeMoraiControlPacket(input, config);
  EXPECT_FLOAT_EQ(1.0F, readFloat(packet.data() + 49U));
}

TEST(ControlProtocol, RejectsSimultaneousAccelAndBrake) {
  std::string error;
  EXPECT_FALSE(isValidControlInput(
      ControlInput{0.1F, 0.1F, 0.0F}, ControlProtocolConfig{}, &error));
  EXPECT_EQ("accel and brake must be mutually exclusive", error);
}
```

Also assert the complete 55-byte fixture, `-40 deg`, steering sign reversal,
invalid gear config, NaN/Inf, pedal range, and steering-angle range.

- [ ] **Step 2: Register generation/library/tests and verify failure**

Update `package.xml` with `message_generation`, `message_runtime`, and
`std_msgs`. Update CMake with:

```cmake
add_message_files(FILES ActuatorCommand.msg)
generate_messages(DEPENDENCIES std_msgs)

add_library(morai_udp_bridge_control_protocol src/control_protocol.cpp)
add_library(morai_udp_bridge_udp_sender src/udp_sender.cpp)

catkin_add_gtest(test_control_protocol test/test_control_protocol.cpp)
catkin_add_gtest(test_udp_sender test/test_udp_sender.cpp)
```

Expose both libraries and `message_runtime` through `catkin_package`, link the
tests to their matching libraries, add `${PROJECT_NAME}_EXPORTED_TARGETS` and
`${catkin_EXPORTED_TARGETS}` dependencies to targets that use the generated
message, and install the new headers/libraries.

Run:

```bash
catkin_make run_tests_morai_udp_bridge_gtest_test_control_protocol
```

Expected: build failure because the protocol implementation is absent.

- [ ] **Step 3: Implement strict protocol validation and encoding**

Reuse the verified packet offsets without linking or including anything from
`vehicle_control`:

```cpp
packet[30U] = 2U;  // external control
packet[31U] = config.drive_gear;
packet[32U] = 1U;  // accel/brake longitudinal mode
writeFloat(0.0F, 33U, &packet);
writeFloat(0.0F, 37U, &packet);
writeFloat(input.accel, 41U, &packet);
writeFloat(input.brake, 45U, &packet);
const float normalized =
    config.steering_sign * input.steering_angle_rad /
    config.maximum_steering_angle_rad;
writeFloat(normalized, 49U, &packet);
```

`encodeMoraiControlPacket()` must throw `std::invalid_argument` when validation
fails; it must not silently clamp an invalid ROS command.

- [ ] **Step 4: Implement and test UDP transport**

Follow the existing bridge transport error style: validate IP/port, create an
IPv4 datagram socket, convert the destination with `inet_pton`, send the entire
datagram with `sendto`, retry only on `EINTR`, and throw `std::runtime_error` on
short/failed sends.

In `test_udp_sender.cpp`, bind a loopback UDP receiver to an ephemeral port,
send `{0x10, 0x20, 0x30}`, poll with a finite timeout, and assert the exact
three received bytes.

- [ ] **Step 5: Run bridge protocol/transport tests and commit**

Run:

```bash
catkin_make run_tests_morai_udp_bridge_gtest_test_control_protocol
catkin_make run_tests_morai_udp_bridge_gtest_test_udp_sender
catkin_test_results build/test_results/morai_udp_bridge --verbose
```

Expected: all existing sensor protocol tests and new control/UDP tests pass.

Commit:

```bash
git add src/morai_udp_bridge
git commit -m "feat(udp_bridge): add autonomous actuator protocol"
```

---

### Task 3: Add the control sender watchdog, node, launch, and docs

**Files:**
- Create: `src/morai_udp_bridge/include/morai_udp_bridge/control_watchdog.hpp`
- Create: `src/morai_udp_bridge/src/control_watchdog.cpp`
- Create: `src/morai_udp_bridge/src/control_sender_node.cpp`
- Create: `src/morai_udp_bridge/test/test_control_watchdog.cpp`
- Create: `src/morai_udp_bridge/config/molit_2026_control.yaml`
- Create: `src/morai_udp_bridge/launch/control_sender.launch`
- Modify: `src/morai_udp_bridge/CMakeLists.txt`
- Modify: `src/morai_udp_bridge/README.md`

**Interfaces:**
- Consumes: `/control/actuator_command` as
  `morai_udp_bridge/ActuatorCommand`.
- Produces: MORAI control datagrams to `destination_ip:destination_port`.
- Produces watchdog API:

```cpp
class ControlWatchdog {
 public:
  ControlWatchdog(double timeout_sec, float safe_brake);
  ControlInput select(const ControlInput& latest, bool has_command,
                      double receipt_age_sec) const;
};
```

- [ ] **Step 1: Write failing watchdog tests**

Cover no command, fresh command, age exactly at timeout, stale command,
non-finite age, and safe-brake config:

```cpp
TEST(ControlWatchdog, ReplacesStaleCommandWithSafeBrake) {
  ControlWatchdog watchdog(0.25, 0.5F);
  const ControlInput selected =
      watchdog.select(ControlInput{0.4F, 0.0F, 0.2F}, true, 0.251);
  EXPECT_FLOAT_EQ(0.0F, selected.accel);
  EXPECT_FLOAT_EQ(0.5F, selected.brake);
  EXPECT_FLOAT_EQ(0.0F, selected.steering_angle_rad);
}
```

- [ ] **Step 2: Verify watchdog test failure, implement, and pass**

Run before implementation:

```bash
catkin_make run_tests_morai_udp_bridge_gtest_test_control_watchdog
```

Expected: build/link failure.

Implement constructor validation and `receipt_age_sec < timeout_sec` selection.
Run the same command again and expect PASS.

- [ ] **Step 3: Implement the thin ROS control sender**

Load these private parameters and reject invalid values during construction:

```cpp
command_topic = "/control/actuator_command";
destination_ip = "127.0.0.1";
destination_port = 9093;
send_rate_hz = 50.0;
command_timeout_sec = 0.25;
safe_brake_command = 0.50;
maximum_steering_angle_deg = 40.0;
steering_sign = 1.0;
drive_gear = 4;
```

On a command callback:

1. Convert the message to `ControlInput`.
2. Validate it with `isValidControlInput`.
3. If invalid, log a throttled warning and do not refresh the watchdog.
4. If valid, store it and `ros::WallTime::now()`.

On a `ros::WallTimer` tick:

1. Compute receipt age.
2. Select latest or safe command through `ControlWatchdog`.
3. Encode with `encodeMoraiControlPacket`.
4. Send with `UdpSender`.
5. Throttle transport errors to one log per second.

- [ ] **Step 4: Add YAML and standalone launch**

Create:

```yaml
control_sender_node:
  command_topic: /control/actuator_command
  destination_ip: 127.0.0.1
  destination_port: 9093
  send_rate_hz: 50.0
  command_timeout_sec: 0.25
  safe_brake_command: 0.50
  maximum_steering_angle_deg: 40.0
  steering_sign: 1.0
  drive_gear: 4
```

The launch must accept `config` as an argument, load it with `rosparam`, and
start only `control_sender_node`.

- [ ] **Step 5: Install the control sender artifacts**

Install `control_sender_node`, the watchdog library, `launch/`, `config/`, and
the new public headers using `${CATKIN_PACKAGE_*_DESTINATION}`. Add generated
message dependencies to the node target:

```cmake
add_dependencies(control_sender_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
```

- [ ] **Step 6: Update bridge README**

Change the package description from sensor-only to MORAI wire bridge. Document:

- unchanged sensor receiver responsibilities;
- `/control/actuator_command` fields and units;
- 50 Hz sender and 0.25 s watchdog;
- `40 deg -> normalized ±1` conversion and `steering_sign`;
- MORAI port `9093`;
- prohibition on running the autonomous and manual UDP senders together;
- standalone launch and `rostopic` verification commands.

- [ ] **Step 7: Run all bridge tests and commit**

Run:

```bash
catkin_make run_tests_morai_udp_bridge
catkin_test_results build/test_results/morai_udp_bridge --verbose
```

Commit:

```bash
git add src/morai_udp_bridge
git commit -m "feat(udp_bridge): send watched autonomous controls"
```

---

### Task 4: Build Pure Pursuit and PID cores in the new package

**Files:**
- Create: `src/morai_path_tracking/package.xml`
- Create: `src/morai_path_tracking/CMakeLists.txt`
- Create: `src/morai_path_tracking/include/morai_path_tracking/pure_pursuit.hpp`
- Create: `src/morai_path_tracking/src/pure_pursuit.cpp`
- Create: `src/morai_path_tracking/include/morai_path_tracking/pid_controller.hpp`
- Create: `src/morai_path_tracking/src/pid_controller.cpp`
- Create: `src/morai_path_tracking/test/test_pure_pursuit.cpp`
- Create: `src/morai_path_tracking/test/test_pid_controller.cpp`

**Interfaces:**
- Pure Pursuit:

```cpp
struct Point2d {
  double x{0.0};
  double y{0.0};
};

struct PurePursuitConfig {
  double wheelbase_m{3.0};
  double lookahead_base_m{3.0};
  double lookahead_speed_gain_sec{0.5};
  double lookahead_min_m{3.0};
  double lookahead_max_m{6.0};
  double minimum_target_distance_m{0.5};
  double maximum_steering_angle_rad{0.6981317008};
};

struct PurePursuitResult {
  bool valid{false};
  Point2d target;
  double lookahead_m{0.0};
  double steering_angle_rad{0.0};
};

PurePursuitResult computePurePursuit(
    const std::vector<Point2d>& path_in_vehicle_frame,
    double longitudinal_speed_mps, const PurePursuitConfig& config);
```

- PID:

```cpp
struct PidConfig {
  double kp{0.35};
  double ki{0.08};
  double kd{0.02};
  double integral_limit{2.0};
  double error_deadband_mps{0.05};
  double maximum_accel{0.40};
  double maximum_brake{0.60};
};

struct LongitudinalCommand {
  double accel{0.0};
  double brake{0.0};
};

class LongitudinalPid {
 public:
  explicit LongitudinalPid(const PidConfig& config);
  LongitudinalCommand update(double target_speed_mps,
                             double measured_speed_mps, double dt_sec);
  void reset();
};
```

- [ ] **Step 1: Scaffold only the manifest/build and failing core tests**

Declare `catkin`, `roscpp`, `nav_msgs`, `morai_udp_bridge`, `tf2`, and
`tf2_geometry_msgs`; add `gtest` as a test dependency. Build
`morai_pure_pursuit` and `morai_longitudinal_pid` libraries.

Write Pure Pursuit tests:

```cpp
TEST(PurePursuit, StraightPathProducesZeroSteering) {
  const std::vector<Point2d> path{{-1.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}};
  const auto result = computePurePursuit(path, 2.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(4.0, result.lookahead_m, 1.0e-9);
  EXPECT_NEAR(4.0, result.target.x, 1.0e-9);
  EXPECT_NEAR(0.0, result.steering_angle_rad, 1.0e-9);
}

TEST(PurePursuit, LeftTargetProducesPositiveSteering) {
  const std::vector<Point2d> path{{0.0, 0.0}, {4.0, 2.0}, {8.0, 4.0}};
  const auto result = computePurePursuit(path, 0.0, PurePursuitConfig{});
  ASSERT_TRUE(result.valid);
  EXPECT_GT(result.steering_angle_rad, 0.0);
}
```

Also cover right sign, first point behind, interpolation, short-path fallback,
no forward target, non-finite points, dynamic lookahead clamp, and `±40 deg`
steering clamp.

Write PID tests:

```cpp
TEST(LongitudinalPid, ProducesAccelBelowTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 0.0, 0.1);
  EXPECT_GT(command.accel, 0.0);
  EXPECT_DOUBLE_EQ(0.0, command.brake);
}

TEST(LongitudinalPid, ProducesBrakeAboveTarget) {
  LongitudinalPid pid(PidConfig{});
  const auto command = pid.update(3.0, 4.0, 0.1);
  EXPECT_DOUBLE_EQ(0.0, command.accel);
  EXPECT_GT(command.brake, 0.0);
}
```

Also cover output clamps, mutual exclusion, deadband, derivative on measurement,
integral clamp, conditional anti-windup, reset, invalid `dt`, and non-finite
inputs.

- [ ] **Step 2: Run core tests and verify failure**

Run:

```bash
catkin_make run_tests_morai_path_tracking_gtest_test_pure_pursuit
catkin_make run_tests_morai_path_tracking_gtest_test_pid_controller
```

Expected: missing-symbol/link failures.

- [ ] **Step 3: Implement lookahead and bicycle steering**

Validate every config field. Compute:

```cpp
const double lookahead = std::max(
    config.lookahead_min_m,
    std::min(config.lookahead_max_m,
             config.lookahead_base_m +
                 config.lookahead_speed_gain_sec *
                     std::abs(longitudinal_speed_mps)));
```

For each ordered segment, solve its intersection with the lookahead circle,
choose the first solution on the segment with `x > 0`, and linearly interpolate
the target. If there is no intersection, use the farthest finite forward point
only when its distance is at least `minimum_target_distance_m`.

Compute and clamp:

```cpp
const double distance_squared =
    target.x * target.x + target.y * target.y;
const double raw =
    std::atan2(2.0 * config.wheelbase_m * target.y, distance_squared);
result.steering_angle_rad = std::max(
    -config.maximum_steering_angle_rad,
    std::min(config.maximum_steering_angle_rad, raw));
```

- [ ] **Step 4: Implement PID with derivative-on-measurement**

Use:

```cpp
const double error = target_speed_mps - measured_speed_mps;
const double effective_error =
    std::abs(error) <= config_.error_deadband_mps ? 0.0 : error;
const double derivative =
    has_previous_measurement_
        ? (measured_speed_mps - previous_measurement_mps_) / dt_sec
        : 0.0;
const double candidate_integral = clamp(
    integral_ + effective_error * dt_sec,
    -config_.integral_limit, config_.integral_limit);
const double candidate_output =
    config_.kp * effective_error +
    config_.ki * candidate_integral -
    config_.kd * derivative;
```

Commit the candidate integral unless output is saturated and the current error
would drive it farther into saturation. Split positive effort to accel and
negative effort to brake. Throw on invalid update inputs so the ROS adapter can
publish a safe command and reset.

- [ ] **Step 5: Run core tests and commit**

Run:

```bash
catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Commit:

```bash
git add src/morai_path_tracking
git commit -m "feat(path_tracking): add pure pursuit and speed PID cores"
```

---

### Task 5: Implement the path-tracking ROS node and standalone launch

**Files:**
- Create: `src/morai_path_tracking/src/pure_pursuit_controller_node.cpp`
- Create: `src/morai_path_tracking/config/molit_2026_pure_pursuit.yaml`
- Create: `src/morai_path_tracking/launch/pure_pursuit.launch`
- Create: `src/morai_path_tracking/test/pure_pursuit_controller.test`
- Create: `src/morai_path_tracking/test/test_pure_pursuit_controller.py`
- Create: `src/morai_path_tracking/README.md`
- Modify: `src/morai_path_tracking/CMakeLists.txt`
- Modify: `src/morai_path_tracking/package.xml`

**Interfaces:**
- Consumes `/local_path` (`nav_msgs/Path`) and `/localization/odometry`
  (`nav_msgs/Odometry`).
- Publishes `/control/actuator_command`
  (`morai_udp_bridge/ActuatorCommand`) at 30 Hz.
- Does not consume `/vehicle/status` or any `vehicle_control` message.

- [ ] **Step 1: Write the failing ROS node test**

Create a rostest that starts the controller with test config. In Python:

1. Subscribe to `/control/actuator_command`.
2. Before inputs, assert a received command has accel `0`, brake `0.5`,
   steering `0`.
3. Publish map-frame odometry at `(0,0)`, yaw `0`, speed `0`.
4. Publish a fresh straight map-frame path with points `x=0..9`.
5. Assert a later command has accel `>0`, brake `0`, and steering near `0`.
6. Publish a left-curving path and assert positive steering.
7. Stop refreshing input for `>0.25 s` and assert safe brake returns.

Use finite waits with explicit assertion messages; never sleep without a
deadline loop.

- [ ] **Step 2: Register the node/rostest and verify failure**

Add the executable and:

```cmake
add_rostest(test/pure_pursuit_controller.test)
catkin_install_python(
  PROGRAMS test/test_pure_pursuit_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

Run:

```bash
catkin_make run_tests_morai_path_tracking_rostest_test_pure_pursuit_controller
```

Expected: test fails because the controller node is not implemented.

- [ ] **Step 3: Implement input validation and map-to-vehicle transform**

Store each latest message plus `ros::WallTime::now()` receipt time. A valid
control cycle requires:

```cpp
has_path && has_odometry
path.header.frame_id == expected_frame_id
odometry.header.frame_id == expected_frame_id
!path.header.stamp.isZero()
!odometry.header.stamp.isZero()
path receipt age <= path_timeout_sec
odometry receipt age <= odometry_timeout_sec
ROS path stamp age <= path_timeout_sec
ROS odometry stamp age <= odometry_timeout_sec
abs(path.header.stamp - odometry.header.stamp) <= maximum_input_skew_sec
finite odometry pose, yaw, speed, and every used path point
```

Transform each map point:

```cpp
const double dx = point.x - vehicle_x;
const double dy = point.y - vehicle_y;
const double x_body = std::cos(yaw) * dx + std::sin(yaw) * dy;
const double y_body = -std::sin(yaw) * dx + std::cos(yaw) * dy;
```

- [ ] **Step 4: Implement the 30 Hz control timer**

Use a `ros::WallTimer` and monotonic wall `dt`.

For a valid cycle:

```cpp
const PurePursuitResult lateral =
    computePurePursuit(vehicle_path, speed_mps, pure_pursuit_config_);
const LongitudinalCommand longitudinal =
    pid_.update(target_speed_mps_, speed_mps, dt_sec);

morai_udp_bridge::ActuatorCommand output;
output.header.stamp = ros::Time::now();
output.accel = static_cast<float>(longitudinal.accel);
output.brake = static_cast<float>(longitudinal.brake);
output.steering_angle_rad =
    static_cast<float>(lateral.steering_angle_rad);
publisher_.publish(output);
```

For missing/stale/invalid input, invalid timer `dt`, or invalid Pure Pursuit
result, call `pid_.reset()` and publish accel `0`, brake
`safe_brake_command`, steering `0`.

- [ ] **Step 5: Add complete YAML and launch**

Create YAML:

```yaml
pure_pursuit_controller_node:
  local_path_topic: /local_path
  odometry_topic: /localization/odometry
  command_topic: /control/actuator_command
  expected_frame_id: map
  control_rate_hz: 30.0
  path_timeout_sec: 0.25
  odometry_timeout_sec: 0.25
  maximum_input_skew_sec: 0.10
  safe_brake_command: 0.50
  wheelbase_m: 3.0
  lookahead_base_m: 3.0
  lookahead_speed_gain_sec: 0.5
  lookahead_min_m: 3.0
  lookahead_max_m: 6.0
  minimum_target_distance_m: 0.5
  maximum_steering_angle_deg: 40.0
  target_speed_mps: 3.0
  speed_kp: 0.35
  speed_ki: 0.08
  speed_kd: 0.02
  speed_integral_limit: 2.0
  speed_error_deadband_mps: 0.05
  maximum_accel_command: 0.40
  maximum_brake_command: 0.60
```

The launch accepts a `config` argument, loads it, and starts only the path
tracking node.

- [ ] **Step 6: Install package artifacts**

Install both core libraries, `pure_pursuit_controller_node`, public headers,
`config/`, and `launch/`. Ensure the node waits for the bridge message:

```cmake
add_dependencies(pure_pursuit_controller_node ${catkin_EXPORTED_TARGETS})
```

- [ ] **Step 7: Write the new package README**

Document algorithm equations, rear-axle frame, topics and units, every YAML
parameter, default `3.0 m/s`, initial-gain tuning warning, safe-state
conditions, standalone launch, test commands, and the noise-off assumption.

- [ ] **Step 8: Run package tests and commit**

Run:

```bash
catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Commit:

```bash
git add src/morai_path_tracking
git commit -m "feat(path_tracking): publish safe autonomous commands"
```

---

### Task 6: Compose autonomous bringup and update workspace documentation

**Files:**
- Create: `src/morai_bringup/launch/molit_2026_autonomous.launch`
- Modify: `src/morai_bringup/package.xml`
- Modify: `src/morai_bringup/test/test_launch_composition.py`
- Modify: `src/morai_bringup/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes package launch files from sensors, localization, path manager,
  `morai_path_tracking`, and `morai_udp_bridge`.
- Produces one autonomous entry point that never includes `vehicle_control`.

- [ ] **Step 1: Add a failing launch-composition test**

Parse `molit_2026_autonomous.launch` and require these includes:

```python
expected = {
    "$(find morai_bringup)/launch/molit_2026_stack.launch",
    "$(find morai_path_tracking)/launch/pure_pursuit.launch",
    "$(find morai_udp_bridge)/launch/control_sender.launch",
}
self.assertEqual(self.include_files(root), expected)
self.assertNotIn("vehicle_control", ET.tostring(root, encoding="unicode"))
```

Also assert launch arguments forward the localization, path, controller, and
control-sender config paths.

- [ ] **Step 2: Run the static test and verify failure**

Run:

```bash
catkin_make run_tests_morai_bringup
```

Expected: failure because `molit_2026_autonomous.launch` does not exist.

- [ ] **Step 3: Implement autonomous composition**

Create launch arguments with defaults:

```text
controller_config =
  $(find morai_path_tracking)/config/molit_2026_pure_pursuit.yaml
control_sender_config =
  $(find morai_udp_bridge)/config/molit_2026_control.yaml
```

Forward the existing stack arguments and include the controller and control
sender exactly once. Do not include RViz or any `vehicle_control` launch.
Add `morai_path_tracking` as an exec dependency in bringup `package.xml`.

- [ ] **Step 4: Update bringup and root READMEs**

Document:

```bash
roslaunch morai_bringup molit_2026_autonomous.launch use_lidar:=false
```

Add `/control/actuator_command` to the topic tables. Update the architecture
flow to show localization velocity -> Pure Pursuit/PID -> UDP bridge -> MORAI.
State that:

- sensor noise is assumed off;
- target speed defaults to `3.0 m/s`;
- gains/config paths are tunable;
- manual and autonomous UDP senders must never run together;
- `vehicle_control` remains independent and unchanged.

- [ ] **Step 5: Run bringup/static tests and commit**

Run:

```bash
catkin_make run_tests_morai_bringup
catkin_test_results build/test_results/morai_bringup --verbose
```

Commit:

```bash
git add src/morai_bringup README.md
git commit -m "feat(bringup): compose autonomous path tracking stack"
```

---

### Task 7: Full verification and handoff

**Files:**
- Verify: all files changed by Tasks 1–6
- Modify only if verification exposes a concrete defect or documentation mismatch.

**Interfaces:**
- Consumes: the complete autonomous stack.
- Produces: a clean, tested feature branch ready for MORAI runtime tuning.

- [ ] **Step 1: Check dependency and formatting integrity**

Run:

```bash
git diff main...HEAD --check
source /opt/ros/noetic/setup.bash
rosdep check --from-paths src --ignore-src
```

Expected: no whitespace errors and all declared dependencies resolve. If
`ros-noetic-ackermann-msgs` appears, remove it; the design uses the custom
`ActuatorCommand`.

- [ ] **Step 2: Cleanly build the full workspace**

Run:

```bash
catkin_make
```

Expected: all packages and generated messages compile with no new warnings from
the changed targets.

- [ ] **Step 3: Run the complete test suite**

Run:

```bash
catkin_make run_tests
catkin_test_results build/test_results --verbose
```

Expected: zero failed tests.

- [ ] **Step 4: Verify installed artifacts**

Run:

```bash
catkin_make install
source install/setup.bash
rospack find morai_path_tracking
roslaunch --files morai_path_tracking pure_pursuit.launch
roslaunch --files morai_udp_bridge control_sender.launch
roslaunch --files morai_bringup molit_2026_autonomous.launch
```

Expected: package and all three launch files resolve from the install space.

- [ ] **Step 5: Perform a no-simulator ROS smoke test**

In terminal A, start the controller:

```bash
roslaunch morai_path_tracking pure_pursuit.launch
```

In terminal B, source the same workspace and echo one output:

```bash
rostopic echo -n 1 /control/actuator_command
```

Expected before inputs:

```text
accel: 0.0
brake: 0.5
steering_angle_rad: 0.0
```

Do not start the UDP control sender during this smoke test unless MORAI is
configured to receive the command.

- [ ] **Step 6: Review documentation against actual defaults**

Compare every YAML key/default against the five updated READMEs and root topic
table. Correct only concrete discrepancies, rerun the relevant tests, and
commit any corrections:

```bash
git add README.md src/*/README.md src/*/config src/*/launch
git commit -m "docs: align autonomous controller guidance"
```

Skip this commit when no corrections are needed.

- [ ] **Step 7: Record MORAI runtime validation checklist**

In the final handoff, report these still-required simulator checks without
claiming they ran:

1. Positive steering turns left; otherwise set `steering_sign: -1.0`.
2. Vehicle accelerates gently from rest with `maximum_accel_command: 0.40`.
3. Estimated `/localization/odometry.twist.twist.linear.x` tracks visible motion.
4. Speed converges near `3.0 m/s` without oscillation before raising the target.
5. Stopping path/localization input produces brake `0.5` within `0.25 s`.
6. Only the autonomous sender owns MORAI UDP port `9093`.
