# 3시간 경로 추종 추가 튜닝 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 재현 가능한 rosbag 분석과 단일 변수 MORAI 실험으로 hybrid 추종
성능을 개선하고 결과를 표·CSV·PNG·Markdown으로 남긴다.

**Architecture:** rosbag 분석은 ROS 메시지 로딩과 순수 metric 계산/plotting을
분리한다. 제어기는 기존 구조를 유지하고, 데이터가 구조적 한계를 입증할 때만
hybrid 조합부를 TDD로 변경한다.

**Tech Stack:** ROS Noetic, rosbag, Python 3, numpy, matplotlib, C++14,
catkin/rostest/gtest.

## Global Constraints

- Branch: `feat/controller`; no commit, push, or merge before user confirmation.
- Do not modify `vehicle_control`.
- Keep standalone Pure Pursuit and Stanley implementations unchanged.
- Hard constraints: zero wheel-line contacts and measured speed below 60 km/h.
- Simulator window stays at 512×288 except brief inspection.
- Every controller behavior change follows red-green TDD.

---

### Task 1: Reproducible bag-analysis tool

**Files:**
- Create: `src/morai_path_tracking/scripts/analyze_tracking_bag.py`
- Create: `src/morai_path_tracking/test/test_tracking_bag_analysis.py`
- Modify: `src/morai_path_tracking/CMakeLists.txt`

**Interfaces:**
- Consumes: `nav_msgs/Odometry`, `nav_msgs/Path`,
  `morai_path_tracking/ControllerStatus` samples.
- Produces: `analyze_samples(samples, lane_half_width_m, vehicle_width_m,
  wheelbase_m) -> dict` and CLI CSV/PNG artifacts.

- [ ] **Step 1: Write failing pure-metric tests**

Create literal straight and offset trajectories. Assert rear-center error,
wheel outer offset, minimum clearance, contact count, max speed, and straight
brake count with hand-calculated values.

- [ ] **Step 2: Verify RED**

Run:
`python3 -m unittest src/morai_path_tracking/test/test_tracking_bag_analysis.py`

Expected: import failure because `analyze_tracking_bag` does not exist.

- [ ] **Step 3: Implement the minimal metric module and CLI**

Implement path projection, vehicle corner/wheel transforms, status aggregation,
CSV writing, and four matplotlib figures. Keep rosbag imports inside the CLI
loading boundary so pure tests do not need ROS master.

- [ ] **Step 4: Verify GREEN**

Run the unit test and then analyze
`/tmp/hybrid_cte_recovery_050_01.bag`; expected contact count is 0 and measured
speed remains below 60 km/h.

- [ ] **Step 5: Register the script**

Add `catkin_install_python(PROGRAMS scripts/analyze_tracking_bag.py ...)` and
rerun the package build.

### Task 2: Baseline report

**Files:**
- Create: `docs/path_tracking_tuning/2026-07-31/README.md`
- Create: `docs/path_tracking_tuning/2026-07-31/metrics.csv`
- Create: `docs/path_tracking_tuning/2026-07-31/trajectory_comparison.png`
- Create: `docs/path_tracking_tuning/2026-07-31/tracking_timeseries.png`
- Create: `docs/path_tracking_tuning/2026-07-31/steering_timeseries.png`
- Create: `docs/path_tracking_tuning/2026-07-31/longitudinal_timeseries.png`

**Interfaces:**
- Consumes: Task 1 CLI and all retained `/tmp/hybrid_*.bag` records.
- Produces: visible baseline comparison and SHA-256 manifest.

- [ ] **Step 1: Analyze retained bags**

Run the CLI over 0.50 m, 0.60 m, and lateral-acceleration experiments with
human-readable experiment labels.

- [ ] **Step 2: Check metric parity**

Compare generated metrics with previously hand-checked values: the two 0.50 m
runs must have zero contacts and worst minimum clearance 0.039 m.

- [ ] **Step 3: Write the baseline table and interpretation**

Document why 0.50 m beats 0.60 m and why 1.6 m/s² lateral acceleration was
rejected. Record lane and vehicle assumptions beside the table.

### Task 3: Parameter-only MORAI experiments

**Files:**
- Modify only when adopted:
  `src/morai_path_tracking/config/molit_2026_path_tracking.yaml`
- Update: `docs/path_tracking_tuning/2026-07-31/metrics.csv`
- Update: `docs/path_tracking_tuning/2026-07-31/README.md`

**Interfaces:**
- Consumes: current final config and Task 1 analyzer.
- Produces: ranked candidate runs and one adopted configuration.

- [ ] **Step 1: Reconfirm current 0.50 m baseline**

Reset in Manual mode, verify initial pose within 1.0 m, run 130 seconds, record
all six controller/path/status topics, and analyze it.

- [ ] **Step 2: Sweep CTE recovery threshold**

Run 0.45 and 0.55 m one at a time. Reject any contact immediately; repeat only
a candidate whose worst clearance and CTE improve over the baseline.

- [ ] **Step 3: Sweep hybrid PP CTE correction**

With the best recovery threshold fixed, run gains 0.45 and 0.55 one at a time.
Use the same rejection and repeatability rule.

- [ ] **Step 4: Check steering-rate trade-off**

Test 55 deg/s only if the best candidate's p95 steering rate exceeds the
baseline by more than 5 deg/s. Keep 60 deg/s otherwise.

- [ ] **Step 5: Recheck longitudinal metrics**

Confirm both final runs stay below 60 km/h and contain zero straight brake
samples. Do not change PID unless both final runs violate the same metric.

### Task 4: Evidence-gated controller change

**Files:**
- Test if needed: `src/morai_path_tracking/test/test_hybrid_controller.cpp`
- Modify if needed:
  `src/morai_path_tracking/include/morai_path_tracking/hybrid_controller.hpp`
- Modify if needed:
  `src/morai_path_tracking/src/controllers/lateral/hybrid_controller.cpp`

**Interfaces:**
- Consumes: two matching rosbag snapshots that demonstrate the same structural
  hybrid failure.
- Produces: a minimal hybrid-only correction protected by a regression test.

- [ ] **Step 1: State one root-cause hypothesis**

Write the exact time, CTE, heading, PP/Stanley candidates, effective weights,
and yaw-rate evidence in the report.

- [ ] **Step 2: Add and run the failing regression test**

Use the recorded vehicle-frame path and literal expected correction direction.
Run the single gtest and confirm it fails for the stated missing behavior.

- [ ] **Step 3: Implement the smallest hybrid-only fix**

Do not change standalone PP or Stanley. Add no more than one configurable
behavior in the hybrid combiner.

- [ ] **Step 4: Run green tests and two full MORAI laps**

Require the regression test, all hybrid tests, and two contact-free full laps.
Revert the code change if either lap is worse than the parameter-only best.

### Task 5: Final verification and handoff

**Files:**
- Update: `src/morai_path_tracking/README.md`
- Update: `src/morai_bringup/docs/CONFIGURATION_GUIDE_KO.md`
- Update: `docs/path_tracking_tuning/2026-07-31/README.md`

**Interfaces:**
- Consumes: final repeated runs and package tests.
- Produces: final reproducible tuning handoff.

- [ ] **Step 1: Generate final plots and comparison table**

Include all accepted/rejected candidates and clearly mark the selected config.

- [ ] **Step 2: Run fresh verification**

Run:
`catkin_make run_tests_morai_path_tracking -j2`
and:
`catkin_test_results build/test_results/morai_path_tracking`

Expected: zero errors and failures.

- [ ] **Step 3: Verify runtime state**

Stop controller/sender, reset the vehicle, keep Launcher/Simulator running, and
return the Simulator window to 512×288.

- [ ] **Step 4: Report without committing**

Provide clickable workspace links, repeated-run metrics, rejected hypotheses,
test counts, branch name, and dirty-worktree status.
