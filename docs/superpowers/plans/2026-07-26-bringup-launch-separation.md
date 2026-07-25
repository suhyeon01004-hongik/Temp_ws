# Bringup Launch Separation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split sensor, localization, and path-manager bringup entrypoints and compose them through one full-stack launch.

**Architecture:** Component packages keep their implementation launch files. `morai_bringup` supplies competition-specific wrappers and one unconditional composition launch, while visualization remains independent.

**Tech Stack:** ROS Noetic, roslaunch XML, catkin, Python 3 unittest

## Global Constraints

- Sensor bringup must not know localization or path-manager configuration.
- Localization and path-manager bringup launch files live in `morai_bringup`.
- The full stack includes sensors, localization, and path manager without feature flags.
- Existing package-owned implementation launch files remain reusable.

---

### Task 1: Lock the launch boundaries with tests

**Files:**
- Modify: `src/morai_bringup/test/test_launch_defaults.py`

**Interfaces:**
- Consumes: launch XML files
- Produces: structural regression coverage for all four bringup entrypoints

- [ ] Replace the old default-boolean test with assertions for separated includes.
- [ ] Run the test and confirm it fails because wrappers/full stack do not exist and the sensor launch still owns localization/path.

### Task 2: Split and compose bringup launch files

**Files:**
- Modify: `src/morai_bringup/launch/molit_2026_sensors.launch`
- Create: `src/morai_bringup/launch/molit_2026_localization.launch`
- Create: `src/morai_bringup/launch/molit_2026_path_manager.launch`
- Create: `src/morai_bringup/launch/molit_2026_stack.launch`
- Modify: `src/morai_bringup/launch/gps_localization_path_test.launch`

**Interfaces:**
- Sensors produces raw ROS sensor topics and static sensor TF.
- Localization consumes GPS/IMU and produces `/localization/pose` plus TF.
- Path manager consumes localization and produces `/global_path` and `/local_path`.

- [ ] Remove localization/path arguments and includes from the sensor launch.
- [ ] Add thin localization and path-manager wrappers.
- [ ] Add the full stack composition launch.
- [ ] Recompose the test launch through all three wrappers.
- [ ] Run the focused test and confirm all assertions pass.

### Task 3: Update usage documentation and verify

**Files:**
- Modify: `README.md`
- Modify: package READMEs and Korean configuration guides that show old feature flags.

- [ ] Replace old boolean-option commands with individual/full-stack commands.
- [ ] Run `roslaunch --nodes` for the three individual launches and the full stack.
- [ ] Run the full build and test suite.
- [ ] Run `catkin_make install` and inspect installed launch files.
