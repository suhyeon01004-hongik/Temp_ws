# Bringup Visualization Entrypoints Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 통합 시험 launch를 제거하고 `morai_bringup/launch/visualization` 아래에
path, LiDAR, path+LiDAR 시각화 진입점을 제공한다.

**Architecture:** 새 launch는 `morai_visualization`의 기존 기능 launch를
그대로 include하는 얇은 wrapper다. 센서·localization·path 실행은
`molit_2026_stack.launch`, 시각화는 하위 wrapper가 각각 담당한다.

**Tech Stack:** ROS1 Noetic, roslaunch XML, Python unittest/nosetests, catkin

## Global Constraints

- `gps_localization_path_test.launch`를 제거한다.
- 시각화 wrapper는 기능 노드를 중복 구현하지 않는다.
- 시각화는 기능 stack을 자동으로 실행하지 않는다.
- 기존 사용 안내는 새 실행 명령으로 모두 교체한다.

---

### Task 1: Launch 구성 교체

**Files:**
- Modify: `src/morai_bringup/test/test_launch_composition.py`
- Delete: `src/morai_bringup/launch/gps_localization_path_test.launch`
- Create: `src/morai_bringup/launch/visualization/path.launch`
- Create: `src/morai_bringup/launch/visualization/lidar.launch`
- Create: `src/morai_bringup/launch/visualization/path_lidar.launch`

**Interfaces:**
- Consumes: `morai_visualization/launch/{path,lidar,path_lidar}.launch`
- Produces: `roslaunch morai_bringup <name>.launch`

- [x] **Step 1: 새 wrapper와 기존 통합 launch 제거를 요구하는 테스트 작성**
- [x] **Step 2: 테스트가 누락된 wrapper와 남아 있는 통합 launch 때문에 실패하는지 확인**
- [x] **Step 3: 세 wrapper를 추가하고 통합 시험 launch를 삭제**
- [x] **Step 4: 구조 테스트 통과 확인**

### Task 2: 문서와 install 검증

**Files:**
- Modify: `README.md`
- Modify: `src/morai_bringup/README.md`
- Modify: `src/morai_bringup/docs/CONFIGURATION_GUIDE_KO.md`
- Modify: `src/morai_localization/README.md`
- Modify: `src/morai_path_manager/README.md`
- Modify: `src/morai_udp_bridge/README.md`
- Modify: `src/morai_visualization/README.md`

**Interfaces:**
- Consumes: 새 bringup launch 명령
- Produces: 기능과 시각화를 별도로 실행하는 한글 안내

- [x] **Step 1: 삭제된 launch 참조를 모두 새 명령으로 교체**
- [x] **Step 2: build와 전체 catkin 테스트 실행**
- [x] **Step 3: install 후 세 wrapper의 `roslaunch --nodes` 결과 확인**
- [x] **Step 4: `git diff --check`와 삭제된 launch 참조 검색**
