# IMM Hybrid Path Tracking Design

## Goal

Add a third lateral-control mode, `hybrid`, to `morai_path_tracking`.
The new controller shall combine the existing Pure Pursuit and Stanley
controllers without merging their implementations or changing their standalone
behavior. It shall use measured lateral vehicle dynamics to continuously blend
the two requested steering angles and shall be tuned by repeated MORAI rosbag
comparison until it tracks the supplied local path more accurately than either
standalone controller under matched conditions.

The existing `pure_pursuit` and `stanley` modes remain selectable and retain
their current algorithms, coordinate conventions, steering limits, diagnostics,
and configuration parameters.

## Evidence and Chosen Method

The selected method follows the structure in Hojin Jung, “Model-Based Hybrid
Control of Pure Pursuit and Stanley Methods for Vehicle Path Tracking,”
Sensors 25(20), 6491, 2025:

- Run Pure Pursuit and Stanley in parallel.
- Treat each steering command as the input of an otherwise identical linear
  bicycle model.
- Use an Interacting Multiple Model filter to compare the two model predictions
  with measured sideslip angle and yaw rate.
- Use the posterior model probabilities as continuous steering weights.

The paper reported lower RMS tracking error than either standalone method in
MORAI. Its required feedback variables are available in this workspace:

- longitudinal speed from Competition Vehicle Status;
- lateral velocity and yaw rate from synchronized localization odometry;
- vehicle yaw and local path from the existing synchronized path/odometry
  input;
- a verified 3.0 m wheelbase and rear-axle `base_link` origin from
  `ioniq5_description`.

The paper's exact parameter values are initial values, not assumed final IONIQ 5
truth. Model parameters and covariance values therefore remain configurable and
are validated using recorded simulator behavior.

## Alternatives Rejected

### Fixed steering blend

`delta = w * delta_stanley + (1 - w) * delta_pure_pursuit` with constant `w`
is simple but cannot adapt between straight running, corner entry, large
cross-track recovery, and corner exit.

### Threshold switching

Selecting one controller using a curvature or error threshold creates a
steering discontinuity and requires hysteresis or timers. This conflicts with
the observed high-speed steering-oscillation problem.

### Rule-based continuous scheduling

A smooth function of speed, curvature, and cross-track error is a viable
fallback and useful as a baseline. It is not the primary design because its
weights describe tuning preferences rather than which controller currently
matches the measured vehicle dynamics.

## Package Structure

The existing source organization remains:

```text
src/morai_path_tracking/src/
  common/
  controllers/
    lateral/
      pure_pursuit.cpp
      stanley_controller.cpp
      hybrid_controller.cpp
    longitudinal/
  nodes/
  planning/
```

New public interfaces are placed in:

```text
src/morai_path_tracking/include/morai_path_tracking/
  hybrid_controller.hpp
  imm_two_model_filter.hpp
```

New implementations are:

```text
src/morai_path_tracking/src/controllers/lateral/
  hybrid_controller.cpp
  imm_two_model_filter.cpp
```

The IMM filter is separated from path geometry so its state transition,
covariance update, likelihood, normalization, and reset behavior can be tested
without ROS or either path-tracking controller.

## Coordinate and Measurement Model

`base_link` is the rear-axle center, with `x` forward and `y` left. The dynamic
bicycle-model state is:

```text
x = [beta_cg, yaw_rate]
```

Localization supplies body-frame velocity at the rear-axle origin. The
center-of-gravity lateral velocity and measured sideslip are:

```text
vy_cg = vy_rear + rear_axle_to_cg * measured_yaw_rate
beta_cg = atan2(vy_cg, max(abs(vx), minimum_model_speed))
```

The controller uses Competition Vehicle Status `velocity_x_mps` for `vx`, so
longitudinal feedback remains compliant with the competition network rule.
Localization longitudinal velocity is not substituted for Competition Status.

Below `minimum_model_speed`, sideslip is poorly conditioned. The hybrid
controller then uses the configured prior weights and yaw-rate measurement
without updating from sideslip.

## Parallel Controller Evaluation

Every hybrid control cycle computes:

```text
PurePursuitResult pure_pursuit
StanleyResult stanley
```

using the same transformed local path, speed, preview curvature, measured yaw
rate, control interval, and current command state used by the standalone modes.

The hybrid controller consumes:

- `pure_pursuit.steering_angle_rad`;
- `stanley.requested_steering_angle_rad`.

The requested Stanley angle is used so the final blend is saturated and
rate-limited exactly once. The standalone Stanley mode continues using its own
existing limited output. Neither standalone calculation formula is moved into
the hybrid implementation.

If either candidate is invalid, the controller rejects the cycle and invokes
the existing safe brake behavior. It does not silently operate as a different
controller mode.

## Bicycle Models

Both IMM branches use the same continuous linear bicycle model:

```text
beta_dot = A00 * beta + A01 * r + B0 * delta
r_dot    = A10 * beta + A11 * r + B1 * delta
```

with:

```text
A00 = -2(Cf + Cr) / (m vx)
A01 = -1 - 2(Cf lf - Cr lr) / (m vx^2)
A10 = -2(Cf lf - Cr lr) / Iz
A11 = -2(Cf lf^2 + Cr lr^2) / (Iz vx)
B0  =  2Cf / (m vx)
B1  =  2Cf lf / Iz
```

The Pure Pursuit branch uses `delta_pure_pursuit`; the Stanley branch uses
`delta_stanley`. Forward Euler discretization is sufficient at the existing
30 Hz control rate, with the already enforced control-interval bounds.

Initial values follow the paper where IONIQ 5-specific data is unavailable:

```text
mass = 2000 kg
yaw inertia = 4000 kg m^2
front/rear cornering stiffness = 60000 N/rad
front/rear axle-to-CG distance = 1.5 m
Q = diag(0.1, 0.01)
R = diag(0.001, 0.001)
```

All values are YAML parameters. Wheelbase consistency requires
`lf + lr == wheelbase` within a small startup tolerance.

## IMM Update

The two branches retain independent state estimates, covariance matrices, and
posterior probabilities.

For every valid cycle:

1. Apply the configurable two-by-two Markov transition matrix to the previous
   probabilities.
2. Mix the prior state and covariance for each destination model.
3. Predict each branch using its candidate steering input.
4. Update each branch using measured `[beta_cg, yaw_rate]`.
5. Compute the Gaussian innovation likelihood with a numerically stable
   two-dimensional covariance inverse and determinant.
6. Normalize the likelihood-weighted model probabilities.
7. Clamp the Stanley probability to configurable lower and upper bounds, then
   renormalize the Pure Pursuit probability as its complement.

The transition prior is speed-dependent as in the source paper, but its four
base values and speed scaling are configurable. A curvature-based hard override
is not used initially because the current vehicle showed persistent lateral
offset on nearly straight paths and still benefits from some Stanley
cross-track correction.

If likelihood normalization becomes non-finite or both likelihoods underflow,
the filter retains the predicted prior probabilities for that cycle. Invalid
matrix or configuration values reject startup.

## Steering Blend and Output Shaping

The desired hybrid steering is:

```text
delta_requested =
    probability_pure_pursuit * delta_pure_pursuit
  + probability_stanley * delta_stanley
```

It is then:

1. clamped to the common 40 degree physical limit;
2. passed through a hybrid-owned steering-rate limit;
3. clamped once more to the physical limit.

The initial hybrid steering-rate limit is the current Stanley value of
60 deg/s. It remains configurable. A temporal low-pass filter is not included
initially because previous bags measured approximately 0.22 s command-to-yaw
response lag; additional phase lag must be justified by later evidence.

The IMM states reset whenever the controller enters the existing safe state,
the selected controller changes after node restart, or a time discontinuity is
rejected. The previous steering command also resets using the current safe-state
policy.

## ROS Integration

`lateral_controller` accepts:

```text
pure_pursuit
stanley
hybrid
```

The node passes synchronized path/odometry data and Competition Status speed to
the hybrid controller. The existing longitudinal curvature speed planner and
PID are unchanged.

In hybrid mode:

- `/control/lookahead_point` publishes the Pure Pursuit lookahead target;
- the Stanley front-axle projection is added to controller diagnostics;
- visualization publishes both points with distinct namespaces/colors and no
  connecting line, avoiding overlap with the local path.

`ControllerStatus` gains:

```text
pure_pursuit_steering_angle_rad
stanley_steering_angle_rad
hybrid_pure_pursuit_probability
hybrid_stanley_probability
measured_sideslip_angle_rad
pure_pursuit_innovation_norm
stanley_innovation_norm
stanley_projection_point_base
```

Existing fields retain their meaning. `steering_angle_rad` is always the final
published steering command.

## Configuration

The YAML adds a dedicated `hybrid_*` section represented as flat ROS private
parameters, consistent with the current file:

```text
hybrid_mass_kg
hybrid_yaw_inertia_kgm2
hybrid_front_cornering_stiffness_n_per_rad
hybrid_rear_cornering_stiffness_n_per_rad
hybrid_front_axle_to_cg_m
hybrid_rear_axle_to_cg_m
hybrid_process_noise_beta
hybrid_process_noise_yaw_rate
hybrid_measurement_noise_beta
hybrid_measurement_noise_yaw_rate
hybrid_initial_pure_pursuit_probability
hybrid_initial_stanley_probability
hybrid_stanley_probability_min
hybrid_stanley_probability_max
hybrid_transition_pp_to_pp
hybrid_transition_pp_to_stanley
hybrid_transition_stanley_to_pp
hybrid_transition_stanley_to_stanley
hybrid_transition_speed_gain
hybrid_minimum_model_speed_mps
hybrid_maximum_steering_rate_deg_per_sec
```

All parameters are required and validated at node startup. The initial selected
mode becomes `hybrid` after implementation, while standalone modes remain one
config edit away.

## Testing

### Unit tests

- Existing Pure Pursuit tests pass without expectation changes.
- Existing Stanley tests pass without expectation changes caused by hybrid
  integration.
- Bicycle-model prediction has hand-derived straight and constant-turn cases.
- IMM probabilities remain finite, non-negative, and sum to one.
- A measurement closer to the Pure Pursuit prediction increases its posterior
  probability.
- A measurement closer to the Stanley prediction increases its posterior
  probability.
- Equal candidates preserve the prior probabilities.
- Low-speed operation does not divide by zero.
- Singular/invalid covariance and invalid configuration are rejected.
- Hybrid steering equals the probability-weighted candidate steering before
  output limiting.
- Saturation and rate limiting occur once after blending.
- Invalid candidate results invoke the existing safe state.

### ROS integration tests

- All three `lateral_controller` values start and publish their selected mode.
- Hybrid status exposes both candidate angles, probabilities, sideslip, and
  innovations.
- Probabilities sum to one at runtime.
- Synchronized-input and timeout safety behavior remains unchanged.
- Pure Pursuit and Stanley standalone outputs match their pre-hybrid regression
  fixtures.

## Runtime Validation and Tuning

Use the same running MORAI route and record at least two comparable bags for
each mode:

```text
pure_pursuit
stanley
hybrid
```

Record:

```text
/control/controller_status
/control/actuator_command
/localization/odometry
/local_path
/vehicle/competition_status
```

Compare:

- overall, straight, curved, and at-least-50-km/h cross-track error RMS;
- 95th percentile and maximum absolute cross-track error;
- heading-error RMS;
- steering-angle RMS;
- steering-rate RMS and 95th percentile;
- curve-entry and curve-exit transient error;
- probability occupancy and transition count;
- candidate innovation norms.

The first tuning order is:

1. verify measurement signs and CG velocity transformation;
2. tune model/covariance values so probability follows the lower innovation;
3. tune probability transition persistence;
4. tune probability floors and ceilings;
5. tune the existing Pure Pursuit and Stanley parameters only if matched
   standalone bags show a remaining deficiency;
6. change hybrid steering-rate limiting only if command chatter remains after
   probability tuning.

Only one parameter group changes per recorded comparison.

Hybrid validation passes only when, across at least two comparable runs:

- hybrid curved and overall cross-track RMS are lower than the better
  standalone controller, with a target improvement of at least 5%;
- high-speed steering-rate RMS is no worse than the better standalone
  controller by more than 10%;
- no safe-state brake pulses, non-finite probabilities, or direct
  accel/brake reversals occur;
- maximum steering remains within 40 degrees.

If IMM cannot beat the better standalone controller after model/covariance and
probability tuning, retain the same external `HybridController` interface and
replace only its weighting strategy with a short-horizon kinematic predictive
cost blend. The standalone controller files still remain unchanged.

## Documentation and Repository Policy

Update the package README, bringup configuration guide, visualization README,
launch/config contract tests, and run instructions. Explain every `hybrid_*`
parameter and the status diagnostics.

Rosbags and exploratory plots remain outside the repository in `/tmp`.
Keep only one final comparison plot under a documented analysis directory if it
is useful to the user. Remove interrupted experimental files and stale analysis
PNGs from the workspace after final verification.

Do not commit, push, merge, or delete the downloaded Stanley ZIP. The user
will test the visible workspace before deciding how to integrate the branch.
