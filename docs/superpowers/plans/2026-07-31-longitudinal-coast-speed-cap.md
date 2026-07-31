# Longitudinal Coast and Speed-Cap Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 작은 과속에는 coast를 사용하고 60 km/h 전 hard brake를 적용하면서 고곡률 바깥 밀림을 줄인다.

**Architecture:** 기존 `LongitudinalPid`가 속도 오차 구간과 hard-speed guard를 소유한다. 곡률 속도 계획은 기존 연속식을 유지하고 config gain만 조정한다. ROS 노드는 상태 진단과 파라미터 변환만 담당한다.

**Tech Stack:** ROS Noetic, C++14, catkin, gtest, rostest, rosbag

## Global Constraints

- `vehicle_control`은 수정하거나 의존하지 않는다.
- accel과 brake는 동시에 양수가 될 수 없다.
- 측정 속도는 `/vehicle/competition_status.velocity_x_mps`만 사용한다.
- 독립 Pure Pursuit와 Stanley 계산식은 변경하지 않는다.
- 커밋·push·merge하지 않는다.

---

### Task 1: PID coast and hard-speed guard

**Files:**
- Modify: `src/morai_path_tracking/include/morai_path_tracking/pid_controller.hpp`
- Modify: `src/morai_path_tracking/src/controllers/longitudinal/pid_controller.cpp`
- Test: `src/morai_path_tracking/test/test_pid_controller.cpp`

- [ ] Coast band, brake threshold, hard-speed guard의 실패 테스트를 작성한다.
- [ ] focused gtest를 실행해 새 필드/동작 부재로 실패하는지 확인한다.
- [ ] 최소 구현과 파라미터 검증을 추가한다.
- [ ] focused gtest를 다시 실행해 통과시킨다.

### Task 2: ROS configuration and diagnostics

**Files:**
- Modify: `src/morai_path_tracking/src/nodes/path_tracking_controller_node.cpp`
- Modify: `src/morai_path_tracking/msg/ControllerStatus.msg`
- Modify: `src/morai_path_tracking/config/molit_2026_path_tracking.yaml`
- Modify: `src/morai_path_tracking/test/test_config_contract.py`
- Modify: `src/morai_path_tracking/test/*.test`

- [ ] 모든 새 파라미터를 required config로 연결한다.
- [ ] longitudinal state와 overshoot를 상태 메시지에 연결한다.
- [ ] config contract와 ROS 통합 테스트를 실행한다.

### Task 3: Runtime tuning and verification

**Files:**
- Modify: `src/morai_path_tracking/README.md`
- Modify: `src/morai_bringup/docs/CONFIGURATION_GUIDE_KO.md`

- [ ] 매 주행 전 Manual 모드, `i`, 시작 위치 오차 1 m 미만을 확인한다.
- [ ] 전체 구간 rosbag을 두 번 기록한다.
- [ ] 60 km/h 상한, 직선 brake event, wheel envelope, CTE를 비교한다.
- [ ] 합격값을 README와 config 설명에 반영하고 전체 테스트를 실행한다.
