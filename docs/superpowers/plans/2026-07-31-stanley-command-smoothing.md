# Stanley Tracking and Command Smoothing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Smooth Stanley reference heading and longitudinal commands while correcting the measured right-side tracking bias.

**Architecture:** Keep the verified rear-axle `base_link` and front-axle Stanley projection. Smooth only the spatial path tangent, filter the curvature planner's raw target before its slew limiter, and rate-limit the PID's signed actuator effort. Expose each new behavior through required ROS parameters and status diagnostics.

**Tech Stack:** ROS Noetic, catkin, C++14, gtest, rostest, Python unittest/YAML contract tests

## Global Constraints

- Work in the visible `/home/suhyeon/catkin_ws` workspace on `feat/controller`.
- Do not create a worktree or hide files from the user's workspace.
- Do not modify or depend on `vehicle_control`.
- Keep the 40 deg physical steering limit and UDP bridge command boundary.
- Do not commit, merge, or push before the user completes live testing.

---

### Task 1: Spatially Smooth the Stanley Reference Heading

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/stanley_controller.hpp`
- Modify: `src/morai_path_tracking/src/controllers/lateral/stanley_controller.cpp`
- Modify: `src/morai_path_tracking/test/test_stanley_controller.cpp`

**Interfaces:**
- Consumes: `StanleyController::calculate(path, speed, previous_steering, dt)`
- Produces: `StanleyConfig::heading_window_m` and a spatially smoothed `StanleyResult::heading_error_rad`

- [ ] **Step 1: Write failing Stanley tests**

Add tests that configure `heading_window_m = 4.0`, feed a mostly straight path
with a short zigzag segment at the front axle, and assert that heading error
stays near zero while the tracking target remains the front-axle projection.
Add invalid zero/non-finite window tests.

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_stanley_controller
```

Expected: compilation fails because `StanleyConfig::heading_window_m` does not
exist.

- [ ] **Step 3: Implement spatial heading calculation**

Add `double heading_window_m{4.0};` to `StanleyConfig`. Track cumulative arc
length, retain the nearest front-axle projection, interpolate points on either
side of its arc position, and calculate heading from their chord. At path ends,
shift the available window; reject a path only when no finite nonzero chord is
available.

- [ ] **Step 4: Run the Stanley tests and confirm GREEN**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_stanley_controller
```

Expected: all Stanley unit tests pass.

### Task 2: Filter Curvature-Based Target Speed

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/curvature_speed_planner.hpp`
- Modify: `src/morai_path_tracking/src/planning/curvature_speed_planner.cpp`
- Modify: `src/morai_path_tracking/test/test_curvature_speed_planner.cpp`

**Interfaces:**
- Consumes: existing raw curvature speed target and `dt_sec`
- Produces: `target_speed_filter_time_constant_sec`,
  `CurvatureSpeedPlan::filtered_target_speed_mps`, and the existing final
  `target_speed_mps`

- [ ] **Step 1: Write failing target-filter tests**

Add a test that initializes on a straight 10 m/s target, switches to a tight
curve with a 0.5 s filter, and asserts the filtered target is strictly between
the raw curve target and 10 m/s. Add reset, disabled-filter, and invalid negative
or non-finite time-constant cases.

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_curvature_speed_planner
```

Expected: compilation fails because the filter fields do not exist.

- [ ] **Step 3: Implement filter-before-slew behavior**

Add the config and result fields, initialize from the first raw target, apply
`1 - exp(-dt/tau)` when enabled, feed the filtered value into the existing
acceleration/deceleration slew limiter, and clear both filter and slew state in
`reset()`.

- [ ] **Step 4: Run planner tests and confirm GREEN**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_curvature_speed_planner
```

Expected: all curvature planner tests pass.

### Task 3: Rate-Limit Normal PID Actuator Effort

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/pid_controller.hpp`
- Modify: `src/morai_path_tracking/src/controllers/longitudinal/pid_controller.cpp`
- Modify: `src/morai_path_tracking/test/test_pid_controller.cpp`

**Interfaces:**
- Consumes: the PID's saturated signed effort and `dt_sec`
- Produces: `PidConfig::command_rate_limit_per_sec`; returned accel/brake remain
  mutually exclusive

- [ ] **Step 1: Write failing effort-rate tests**

Add tests with `command_rate_limit_per_sec = 1.0` that assert a 0.1 s update can
change signed effort by at most 0.1, that accel-to-brake reversal first reaches
zero, and that `reset()` restores the zero starting effort. Add invalid negative
and non-finite rate tests.

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_pid_controller
```

Expected: compilation fails because `command_rate_limit_per_sec` does not exist.

- [ ] **Step 3: Implement signed effort rate limiting**

After amplitude saturation, clamp the signed output around the previous signed
output by `rate * dt`. Convert nonnegative output to accel and negative output
to brake. Treat `0.0` as disabled and clear previous effort in `reset()`.

- [ ] **Step 4: Run PID tests and confirm GREEN**

Run:

```bash
catkin_make --pkg morai_path_tracking tests
./devel/lib/morai_path_tracking/test_pid_controller
```

Expected: all PID tests pass and existing unlimited default behavior remains.

### Task 4: Wire Parameters and Diagnostics Through the ROS Node

**Files:**
- Modify: `src/morai_path_tracking/src/nodes/path_tracking_controller_node.cpp`
- Modify: `src/morai_path_tracking/msg/ControllerStatus.msg`
- Modify: `src/morai_path_tracking/config/molit_2026_path_tracking.yaml`
- Modify: `src/morai_path_tracking/test/test_config_contract.py`
- Modify: `src/morai_path_tracking/test/stanley_controller.test`

**Interfaces:**
- Consumes: required private ROS parameters
- Produces: `raw_target_speed_mps`, `filtered_target_speed_mps`, and final
  `target_speed_mps` on `/control/controller_status`

- [ ] **Step 1: Extend config contract and launch fixture first**

Require `stanley_heading_window_m`,
`target_speed_filter_time_constant_sec`, and
`longitudinal_command_rate_limit_per_sec`. Assert defaults 4.0, 0.35, and 2.0.
Add them to the Stanley rostest launch fixture.

- [ ] **Step 2: Run config test and confirm RED**

Run:

```bash
python3 src/morai_path_tracking/test/test_config_contract.py
```

Expected: failure because runtime YAML lacks the three keys.

- [ ] **Step 3: Wire runtime parameters and status**

Load and validate all three parameters, assign them to the core configs, add
raw/filtered fields to `ControllerStatus.msg`, populate them in
`publishControllerStatus`, and set the runtime defaults in YAML. Keep
`speed_filter_time_constant_sec: 0.0` and the 90 deg/s Stanley steering rate.

- [ ] **Step 4: Run config and ROS node tests**

Run:

```bash
python3 src/morai_path_tracking/test/test_config_contract.py
catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Expected: config contract and controller node tests pass.

### Task 5: Document and Verify the Integrated Behavior

**Files:**
- Modify: `src/morai_path_tracking/README.md`
- Modify: `README.md` only if its package summary needs the new diagnostics

**Interfaces:**
- Consumes: final parameter names and behavior
- Produces: operator-facing tuning and bag comparison instructions

- [ ] **Step 1: Update documentation**

Document the verified rear/front axle coordinates, spatial heading window,
gain 2.0 rationale, target filter ordering, signed pedal effort limiter,
disabled measured-speed and steering temporal filters, status fields, and
which parameters to tune after replay.

- [ ] **Step 2: Build and run the complete relevant test set**

Run:

```bash
catkin_make
catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

Expected: build succeeds and all `morai_path_tracking` tests pass.

- [ ] **Step 3: Review the diff without committing**

Run:

```bash
git diff --check
git status --short --branch
git diff -- src/morai_path_tracking docs/superpowers
```

Expected: no whitespace errors, changes remain visible and uncommitted on
`feat/controller`.

### Task 6: Add Curvature Feedforward and Yaw-Rate Damping

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/stanley_controller.hpp`
- Modify: `src/morai_path_tracking/src/controllers/lateral/stanley_controller.cpp`
- Modify: `src/morai_path_tracking/src/nodes/path_tracking_controller_node.cpp`
- Modify: `src/morai_path_tracking/msg/ControllerStatus.msg`
- Modify: runtime config, tests, and package README

- [x] Record two live bags and confirm high-speed oscillation and curve-exit
  residual steering/yaw rate.
- [x] Add failing tests for signed forward curvature, steady-curve
  feedforward, straight-path yaw damping, invalid runtime yaw rate, and new
  configuration ranges.
- [x] Implement the damped/feedforward Stanley formula using localization
  odometry `angular.z`.
- [x] Expose reference curvature, reference/measured yaw rate, and yaw-rate
  error in `ControllerStatus`.
- [x] Set initial data-informed gains and update config contract/fixtures.
- [ ] Run the full package test suite and compare a new live bag after the
  controller node is restarted.
