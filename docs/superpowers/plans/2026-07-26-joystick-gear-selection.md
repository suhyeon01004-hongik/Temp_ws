# Joystick Gear Selection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Select MORAI P/R/N/D gears with CYVOX MX face buttons, allowing changes only with fresh low-speed odometry.

**Architecture:** A pure C++ `GearSelector` owns the latched gear, button rising-edge detection, and speed gate. `joystick_teleop_node` supplies Joy buttons and `/localization/odometry` speed, while `morai_udp_sender_node` forwards `VehicleCommand.gear` to the existing MORAI packet encoder.

**Tech Stack:** ROS1 Noetic, catkin, C++14, `sensor_msgs/Joy`, `nav_msgs/Odometry`, gtest, rostest.

## Global Constraints

- Buttons default to `A(0)=D`, `B(1)=N`, `X(2)=R`, `Y(3)=P`.
- A gear request is accepted only at or below `0.5 m/s`.
- Missing, non-finite, or stale speed feedback rejects gear changes.
- Only a newly pressed button requests a change; holding a button must not cause a delayed change.
- Multiple gear buttons pressed together reject the request.
- Mapping, speed threshold, odometry topic, timeout, and initial gear are YAML parameters.

---

### Task 1: Pure gear selector

**Files:**
- Create: `src/vehicle_control/include/vehicle_control/gear_selector.hpp`
- Create: `src/vehicle_control/src/gear_selector.cpp`
- Create: `src/vehicle_control/test/test_gear_selector.cpp`
- Modify: `src/vehicle_control/CMakeLists.txt`

**Interfaces:**
- Consumes: `std::vector<std::int32_t> buttons`, `bool speed_valid`, `double speed_mps`.
- Produces: `GearSelectionResult GearSelector::update(...)` containing the latched gear and a status enum.

- [ ] **Step 1: Write failing selector tests**

Cover literal mappings `0→4`, `1→3`, `2→2`, `3→1`; low-speed acceptance; high-speed and invalid-speed rejection; held-button behavior; simultaneous buttons; duplicate/negative config validation.

- [ ] **Step 2: Run RED**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make run_tests_vehicle_control_gtest_test_gear_selector
```

Expected: compilation fails because `gear_selector.hpp` does not exist.

- [ ] **Step 3: Implement the minimal selector**

Define:

```cpp
enum class GearSelectionStatus {
  kNoRequest,
  kChanged,
  kSpeedUnavailable,
  kTooFast,
  kAmbiguousButtons,
  kInvalidButtonMessage,
};

struct GearSelectorConfig {
  int drive_button{0};
  int neutral_button{1};
  int reverse_button{2};
  int park_button{3};
  std::uint8_t initial_gear{4U};
  double maximum_change_speed_mps{0.5};
};

struct GearSelectionResult {
  std::uint8_t gear;
  GearSelectionStatus status;
};
```

Track the four previous button states and update the latched gear only for one
new rising edge with valid speed at or below the threshold.

- [ ] **Step 4: Run GREEN**

Run the selector target and confirm every selector test passes.

### Task 2: ROS odometry and command integration

**Files:**
- Modify: `src/vehicle_control/src/joystick_teleop_node.cpp`
- Modify: `src/vehicle_control/src/morai_udp_sender_node.cpp`
- Modify: `src/vehicle_control/config/cyvox_mx.yaml`
- Modify: `src/vehicle_control/package.xml`
- Modify: `src/vehicle_control/CMakeLists.txt`
- Modify: `src/vehicle_control/test/cyvox_mapping.test`
- Modify: `src/vehicle_control/test/test_cyvox_mapping.py`
- Modify: `src/vehicle_control/test/test_morai_ctrl_packet.cpp`

**Interfaces:**
- Consumes: `/localization/odometry` (`nav_msgs/Odometry`) and `/joy`.
- Produces: `/vehicle/manual_command` with the selected `gear`, then MORAI byte 31 with that value.

- [ ] **Step 1: Write failing integration and packet tests**

Add an odometry publisher to the rostest. At zero speed, publish a released Joy
frame followed by `X=1` and expect `gear=2`. At `1.0 m/s`, press `Y` and expect
the previous gear to remain unchanged. Add a packet assertion that reverse
produces byte `31 == 2`.

- [ ] **Step 2: Run RED**

Run the rostest and packet test. Expected: gear remains fixed at Drive and the
reverse packet behavior is not exercised through the node.

- [ ] **Step 3: Connect selector, odometry, and UDP**

Load these private parameters:

```yaml
odometry_topic: /localization/odometry
odometry_timeout: 0.5
maximum_gear_change_speed_mps: 0.5
initial_gear: 4
drive_button: 0
neutral_button: 1
reverse_button: 2
park_button: 3
```

Subscribe to odometry, compute `hypot(linear.x, linear.y)`, and use wall time to
mark feedback stale. Put `GearSelector::gear` into `VehicleCommand.gear`.
Change `morai_udp_sender_node` to construct `ControlCommand` with
`message->gear` instead of fixed `4U`.

- [ ] **Step 4: Run GREEN**

Run the selector, packet, and rostest targets and confirm they pass.

### Task 3: Documentation and full verification

**Files:**
- Modify: `src/vehicle_control/README.md`
- Modify: `docs/superpowers/specs/2026-07-26-joystick-control-design.md`

- [ ] **Step 1: Document controls and parameters**

Document the face-button mapping, latched behavior, low-speed requirement,
stale odometry rejection, and each YAML parameter in Korean.

- [ ] **Step 2: Build and run the complete workspace test suite**

```bash
source /opt/ros/noetic/setup.bash
catkin_make install
catkin_make run_tests
catkin_test_results
```

Expected: install succeeds and the final summary reports zero errors and zero
failures.

- [ ] **Step 3: Check the diff**

Run `git diff --check` and confirm there is no whitespace error or unrelated
file change.
