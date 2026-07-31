# IMM Pure Pursuit–Stanley Hybrid Path Tracking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an IMM-based `hybrid` lateral controller that continuously blends the existing Pure Pursuit and Stanley requests and demonstrably reduces path-tracking error in repeated MORAI runs.

**Architecture:** Keep the standalone controller implementations unchanged. A ROS-independent two-model IMM compares linear bicycle-model predictions driven by the two candidate steering requests against measured CG sideslip and yaw rate; a separate `HybridController` owns that filter, blends the candidates using its posterior probabilities, and applies final saturation/rate limiting once.

**Tech Stack:** ROS Noetic, C++14, catkin, GoogleTest, rostest, rosbag, Python 3 bag analysis

## Global Constraints

- Keep `vehicle_control` independent and unchanged.
- Preserve selectable standalone `pure_pursuit` and `stanley` behavior.
- Keep the verified rear-axle `base_link`, x-forward/y-left coordinates, and 3.0 m wheelbase.
- Use Competition Vehicle Status `velocity_x_mps` as longitudinal feedback.
- Maximum front-wheel steering magnitude is 40 degrees.
- Keep lidar and the default autonomous launch composition enabled.
- Put every tunable hybrid/model/covariance value in `config/molit_2026_path_tracking.yaml`.
- Do not commit, push, merge, or delete the downloaded Stanley ZIP.
- Keep exploratory bags and plots outside the repository in `/tmp`.

---

### Task 1: Restore the standalone-controller regression baseline

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/stanley_controller.hpp`
- Modify: `src/morai_path_tracking/src/controllers/lateral/stanley_controller.cpp`
- Modify: `src/morai_path_tracking/msg/ControllerStatus.msg`
- Modify: `src/morai_path_tracking/test/test_stanley_controller.cpp`
- Replace: `src/morai_path_tracking/test/test_hybrid_controller.cpp`
- Modify: `src/morai_path_tracking/CMakeLists.txt`

**Interfaces:**
- Consumes: the last verified nonlinear yaw-damped Stanley configuration
- Produces: unchanged standalone Stanley output and a clean failing placeholder for the approved IMM hybrid API

- [ ] **Step 1: Remove only the interrupted speed-dependent Stanley-rate experiment**

  Remove `high_speed_steering_rate_rad_per_sec`,
  `steering_rate_transition_*`, and
  `applied_steering_rate_limit_rad_per_sec` from Stanley, its status message,
  and its tests. Keep curvature feedforward, spatial heading smoothing,
  nonlinear yaw damping, 40-degree saturation, and the existing 60 deg/s
  standalone rate limit.

- [ ] **Step 2: Run the focused Stanley test**

  Run:
  `cmake --build build --target run_tests_morai_path_tracking_gtest_test_stanley_controller -j2`

  Expected: all standalone Stanley tests pass.

- [ ] **Step 3: Replace the stale heuristic-hybrid test registration**

  Remove the incomplete smoothstep/CTE hybrid test target. The approved
  hybrid tests are introduced after the IMM public interface is specified in
  Tasks 2 and 3.

- [ ] **Step 4: Verify Pure Pursuit and Stanley regression suites**

  Run:
  `cmake --build build --target run_tests_morai_path_tracking_gtest_test_pure_pursuit run_tests_morai_path_tracking_gtest_test_stanley_controller -j2`

  Expected: both targets pass without changing existing controller formulas.

### Task 2: Implement the two-model IMM core with TDD

**Files:**
- Create: `src/morai_path_tracking/include/morai_path_tracking/imm_two_model_filter.hpp`
- Create: `src/morai_path_tracking/src/controllers/lateral/imm_two_model_filter.cpp`
- Create: `src/morai_path_tracking/test/test_imm_two_model_filter.cpp`
- Modify: `src/morai_path_tracking/CMakeLists.txt`

**Interfaces:**
- Consumes:
  `ImmConfig`, `longitudinal_speed_mps`, `measured_sideslip_rad`,
  `measured_yaw_rate_radps`, two steering requests, and `dt_sec`
- Produces:
  `ImmResult ImmTwoModelFilter::update(...)` with two probabilities,
  innovation norms, model states, and a finite/error status; plus
  `void reset()`

- [ ] **Step 1: Write failing probability and prediction tests**

  Add hand-derived tests proving:

  - zero state/zero steering remains zero;
  - equal candidate inputs preserve equal probabilities;
  - measurement at the PP prediction raises PP probability;
  - measurement at the Stanley prediction raises Stanley probability;
  - probabilities remain finite, non-negative, and sum to one;
  - reset restores configured initial probabilities;
  - invalid mass, inertia, stiffness, axle distances, Q/R, transition rows,
    speed, measurement, and `dt` are rejected.

- [ ] **Step 2: Run the new target and observe RED**

  Run:
  `cmake --build build --target run_tests_morai_path_tracking_gtest_test_imm_two_model_filter -j2`

  Expected: compilation fails because `imm_two_model_filter.hpp` and its API
  do not exist.

- [ ] **Step 3: Implement fixed-size two-state mathematics**

  Implement explicit two-element vectors and 2x2 matrices for:

  - probability/state/covariance interaction;
  - linear bicycle-model Euler prediction;
  - Kalman measurement update with `H=I`;
  - positive-definite 2x2 inverse/determinant checks;
  - Gaussian log likelihood and max-log normalization;
  - configurable Stanley probability floor/ceiling;
  - prior fallback when likelihood normalization is non-finite.

  Do not add Eigen or a new runtime dependency.

- [ ] **Step 4: Run the focused IMM tests and observe GREEN**

  Run the target from Step 2 and confirm all tests pass without warnings.

- [ ] **Step 5: Run the mutation check**

  Confirm tests would fail for swapped candidate inputs, a wrong yaw-input
  sign, probability normalization omission, a non-positive covariance
  determinant, or a reset that retains prior state.

### Task 3: Implement the HybridController composition with TDD

**Files:**
- Create: `src/morai_path_tracking/include/morai_path_tracking/hybrid_controller.hpp`
- Create: `src/morai_path_tracking/src/controllers/lateral/hybrid_controller.cpp`
- Create: `src/morai_path_tracking/test/test_hybrid_controller.cpp`
- Modify: `src/morai_path_tracking/CMakeLists.txt`

**Interfaces:**
- Consumes:
  vehicle-frame path, Competition `vx`, localization rear-axle `vy`,
  measured yaw rate, preview curvature, previous final steering, and `dt`
- Produces:
  `HybridResult HybridController::calculate(...)` containing both native
  controller results, measured CG sideslip, both probabilities/innovations,
  blended request, applied final command, and both visualization points

- [ ] **Step 1: Write failing composition tests**

  Add tests proving:

  - `vy_cg = vy_rear + lr * yaw_rate`;
  - `beta_cg = atan2(vy_cg, max(abs(vx), minimum_model_speed))`;
  - blended request equals
    `p_pp * delta_pp + p_stanley * delta_stanley`;
  - Stanley `requested_steering_angle_rad`, not its already-rate-limited
    output, is blended;
  - 40-degree saturation and hybrid rate limiting happen exactly once after
    blending;
  - PP lookahead and Stanley projection remain separately accessible;
  - either invalid candidate invalidates the hybrid cycle;
  - low-speed and reverse/invalid numeric inputs are handled according to the
    design;
  - `reset()` resets IMM state.

- [ ] **Step 2: Run the Hybrid target and observe RED**

  Run:
  `cmake --build build --target run_tests_morai_path_tracking_gtest_test_hybrid_controller -j2`

  Expected: compilation fails because `HybridController` is absent.

- [ ] **Step 3: Implement minimal composition**

  Construct and invoke the existing `computePurePursuit` and
  `StanleyController::calculate`, update the IMM, blend candidate requests,
  saturate/rate-limit once, and expose diagnostics. Do not duplicate or move
  either standalone controller formula.

- [ ] **Step 4: Run Hybrid and standalone controller tests**

  Expected: Hybrid tests pass, and standalone tests retain the Task 1 output.

### Task 4: Integrate hybrid mode into the ROS node and config

**Files:**
- Modify: `src/morai_path_tracking/src/nodes/path_tracking_controller_node.cpp`
- Modify: `src/morai_path_tracking/msg/ControllerStatus.msg`
- Modify: `src/morai_path_tracking/config/molit_2026_path_tracking.yaml`
- Modify: `src/morai_path_tracking/test/test_config_contract.py`
- Modify: `src/morai_path_tracking/test/stanley_controller.test`
- Modify: `src/morai_path_tracking/test/pure_pursuit_controller.test`
- Modify: `src/morai_path_tracking/test/localization_generation_safety.test`
- Modify: `src/morai_path_tracking/test/test_stanley_controller_node.py`
- Modify: `src/morai_path_tracking/test/test_pure_pursuit_controller.py`

**Interfaces:**
- Consumes: `lateral_controller: hybrid`, synchronized odometry lateral
  velocity/yaw rate, and required flat `hybrid_*` private parameters
- Produces: final actuator command and exact hybrid diagnostics on
  `/control/controller_status`

- [ ] **Step 1: Add failing config and ROS integration expectations**

  Require all approved `hybrid_*` keys, accept exactly the three lateral modes,
  assert hybrid probabilities sum to one, assert candidate angles and
  sideslip are finite, and assert standalone mode fixtures are unchanged.

- [ ] **Step 2: Run focused tests and observe RED**

  Run:
  `python3 src/morai_path_tracking/test/test_config_contract.py`
  and the relevant `rostest` targets.

- [ ] **Step 3: Load and validate hybrid parameters**

  Convert steering degrees to radians, ensure `lf + lr` matches the 3.0 m
  wheelbase, validate probability/transition/covariance ranges, instantiate
  `HybridController`, and change the production YAML default to `hybrid`.

- [ ] **Step 4: Pass rear-axle lateral velocity through input validation**

  Continue using Competition Status for `vx`. Read only
  `odometry.twist.twist.linear.y` and `angular.z` for lateral dynamics.

- [ ] **Step 5: Publish and reset hybrid diagnostics/state**

  Add candidate angles, model probabilities, measured sideslip, innovation
  norms, and Stanley projection to `ControllerStatus`. Call
  `hybrid_controller_.reset()` in every existing safe-state reset path.

- [ ] **Step 6: Run focused config and rostests until GREEN**

  Confirm no mode produces simultaneous accel/brake or changes existing
  timeout/synchronization behavior.

### Task 5: Visualize both hybrid tracking references

**Files:**
- Modify: `src/morai_path_tracking/src/nodes/path_tracking_controller_node.cpp`
- Modify: `src/morai_visualization/src/path_visualizer_node.cpp`
- Modify: `src/morai_visualization/config/path_visualizer.yaml`
- Modify: `src/morai_visualization/rviz/path.rviz`
- Modify: `src/morai_visualization/rviz/path_lidar.rviz`
- Modify: `src/morai_visualization/test/test_path_visualizer.py`
- Modify: `src/morai_visualization/test/test_visualization_assets.py`

**Interfaces:**
- Consumes: PP lookahead and Stanley projection point topics
- Produces: two distinct point-only RViz markers with no connecting line

- [ ] **Step 1: Add failing marker/topic assertions**

  Assert distinct namespaces, colors, point-only marker types, valid
  `base_link` frames, and both RViz displays.

- [ ] **Step 2: Run visualization tests and observe RED**

  Run the package’s focused Python/rostest targets.

- [ ] **Step 3: Publish and render both points**

  Preserve `/control/lookahead_point` for PP LD and add a dedicated Stanley
  projection topic. In standalone modes publish only the relevant point; in
  hybrid publish both.

- [ ] **Step 4: Run visualization tests until GREEN**

### Task 6: Full build and matched live controller baselines

**Files:**
- No repository output files; bags and plots go under `/tmp`

**Interfaces:**
- Consumes: the running MORAI route and all three controller modes
- Produces: comparable baseline metrics and plots

- [ ] **Step 1: Build and run the complete workspace tests**

  Run `catkin_make -j2`, package tests, workspace tests, and
  `catkin_test_results --verbose`. Stop on any failure.

- [ ] **Step 2: Restart the default autonomous launch**

  Keep lidar enabled. Verify one controller sender, `ACTIVE` status, 30 Hz
  controller output, finite probabilities, and no safe-brake pulses.

- [ ] **Step 3: Record at least two Pure Pursuit bags**

  Record controller status, odometry, local path, actuator command, and
  Competition Status for at least 90 seconds per run.

- [ ] **Step 4: Record at least two standalone Stanley bags**

  Use the same route and recording topics/duration.

- [ ] **Step 5: Record at least two Hybrid bags**

  Use the same route and recording topics/duration.

- [ ] **Step 6: Compare matched route bins**

  Report overall/straight/curved/≥50-km/h CTE RMS and p95/max, heading RMS,
  steering RMS/rate RMS/p95, curve transients, safe states, pedal reversals,
  probability occupancy/transitions, and innovation norms.

### Task 7: Iterative model/probability tuning

**Files:**
- Modify: `src/morai_path_tracking/config/molit_2026_path_tracking.yaml`
- Update tests only when a deliberate final default changes

**Interfaces:**
- Consumes: Task 6 bags/metrics
- Produces: a hybrid default that beats the better standalone mode

- [ ] **Step 1: Verify signs and model response**

  Confirm positive left steering predicts positive yaw, the CG sideslip
  transform is correct, and the lower innovation receives higher probability.

- [ ] **Step 2: Tune one parameter group per comparison**

  Tune in order: model/Q/R, transition persistence, probability floor/ceiling,
  then final rate limit. Rebuild/restart only when code changes; restart the
  node for config changes.

- [ ] **Step 3: Re-record after every accepted parameter change**

  Reject any change that improves only one short segment while materially
  worsening overall or curve CTE, steering stability, or safe behavior.

- [ ] **Step 4: Apply the predictive fallback only if IMM fails**

  If model/probability tuning cannot beat the better standalone controller,
  retain `HybridController`’s external interface and replace only its weighting
  implementation with a short-horizon kinematic predictive cost blend, again
  using RED/GREEN tests and matched bags.

- [ ] **Step 5: Satisfy acceptance criteria**

  Across at least two comparable runs, require lower overall and curved CTE RMS
  than the better standalone controller (target ≥5% improvement), high-speed
  steering-rate RMS within 10% of the better standalone, no safe-state brake
  pulses/non-finite probabilities/direct pedal reversals, and steering within
  40 degrees.

### Task 8: Documentation, cleanup, and final verification

**Files:**
- Modify: `src/morai_path_tracking/README.md`
- Modify: `src/morai_bringup/README.md`
- Modify: `src/morai_bringup/docs/CONFIGURATION_GUIDE_KO.md`
- Modify: `src/morai_visualization/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: final verified implementation/config/metrics
- Produces: user-testable workspace with current instructions

- [ ] **Step 1: Document architecture and every parameter/diagnostic**

  Include the IMM formula, CG velocity transform, tuning direction, three
  selectable modes, source tree, run commands, rosbag topics, and actual final
  metrics.

- [ ] **Step 2: Remove superseded analysis artifacts**

  Remove stale repository-root PNGs and interrupted heuristic-hybrid artifacts.
  Keep bags in `/tmp` and preserve the downloaded Stanley ZIP.

- [ ] **Step 3: Run final verification**

  Run the complete build/tests, `catkin_test_results --verbose`,
  `git diff --check`, launch composition checks, 30 Hz topic checks, and one
  final runtime status sample.

- [ ] **Step 4: Hand off without committing**

  Report exact changed files, final config values, measured comparisons,
  remaining limitations, and user test commands. Do not commit, push, or merge.
