# Separate MORAI Reset Key Hold Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Hold MORAI's `i` reset key for 120 ms after a 250 ms Manual-mode settle while keeping every `q` mode-toggle hold at 10 ms.

**Architecture:** Add one independent reset-key timing field to `MoraiResetOptions`, load it from the reset node's private ROS parameters, and use it only when building the `i` key event. Existing mode timing and UDP port configuration remain unchanged.

**Tech Stack:** ROS Noetic, C++14, GoogleTest, catkin

## Global Constraints

- Modify only `src/vehicle_control`.
- Keep `key_hold` at `0.01` seconds for `q`.
- Set `reset_key_hold` to `0.12` seconds for `i`.
- Set `mode_settle` to `0.25` seconds before `i`.
- Do not change MORAI UDP ports.

---

### Task 1: Separate the reset-key hold duration

**Files:**
- Modify: `src/vehicle_control/test/test_reset_command.cpp`
- Modify: `src/vehicle_control/include/vehicle_control/reset_command.hpp`
- Modify: `src/vehicle_control/src/reset_command.cpp`
- Modify: `src/vehicle_control/src/morai_sim_reset_node.cpp`
- Modify: `src/vehicle_control/config/cyvox_mx.yaml`
- Modify: `src/vehicle_control/README.md`

**Interfaces:**
- Consumes: `MoraiResetOptions` and `buildMoraiResetCommand(const MoraiResetOptions&)`
- Produces: `MoraiResetOptions::reset_key_hold_seconds` and ROS parameter `reset_key_hold`

- [ ] **Step 1: Write the failing test**

Change the expected `i` key sleep from `"0.010"` to `"0.120"`, expect a
`"0.2500"` Manual settle, and add:

```cpp
TEST(ResetCommandTest, RejectsNegativeResetKeyHold) {
  MoraiResetOptions options;
  options.reset_key_hold_seconds = -0.1;
  EXPECT_THROW(buildMoraiResetCommand(options), std::invalid_argument);
}
```

- [ ] **Step 2: Run the focused test and confirm RED**

Run:

```bash
catkin_make run_tests_vehicle_control_gtest_test_reset_command
catkin_test_results build/test_results/vehicle_control
```

Expected: failure because the generated `i` hold remains `"0.010"` or the new field is unavailable.

- [ ] **Step 3: Implement the minimal separation**

Add `reset_key_hold_seconds{0.12}` to `MoraiResetOptions`, validate and format it in `buildMoraiResetCommand`, and pass it only to the `appendHeldKey` call for `reset_key`. Load `reset_key_hold` in `morai_sim_reset_node`, set it to `0.12` with `mode_settle: 0.25` in `cyvox_mx.yaml`, and document both values in the timing table.

- [ ] **Step 4: Run focused and full verification**

Run:

```bash
catkin_make run_tests_vehicle_control_gtest_test_reset_command
catkin_make run_tests_vehicle_control
catkin_test_results build/test_results/vehicle_control
catkin_make install --pkg vehicle_control
git diff --check
```

Expected: all `vehicle_control` tests pass, install succeeds, and no whitespace errors are reported.
