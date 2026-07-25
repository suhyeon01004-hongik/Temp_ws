# MORAI Default Control Port Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `vehicle_control`이 MORAI Cmd Control 기본 수신 포트 `9093`으로 명령을 보내도록 모든 활성 기본값과 사용자 문서를 통일한다.

**Architecture:** 실제 launch는 `config/cyvox_mx.yaml`의 값을 로드하고, YAML 없이 송신 노드를 직접 실행할 때는 C++ 내부 기본값을 사용한다. 두 값을 모두 `9093`으로 맞추고 설치 공간을 재생성해 어느 컴퓨터에서도 같은 기본 동작을 보장한다.

**Tech Stack:** ROS Noetic, catkin install space, C++14, YAML, Markdown

## Global Constraints

- MORAI Cmd Control의 Host PORT 기본값은 정확히 `9093`이다.
- `destination_ip` 기본값 `127.0.0.1`과 Destination PORT 설명은 변경하지 않는다.
- UDP 송신 테스트에서 사용하는 임의 포트 `9095`는 MORAI 실행 포트와 충돌하지 않도록 유지한다.
- 사용자가 실행 중인 ROS 노드나 MORAI 시뮬레이터는 조작하지 않는다.

---

### Task 1: 기본 포트와 안내 문서 통일

**Files:**
- Modify: `src/vehicle_control/config/cyvox_mx.yaml`
- Modify: `src/vehicle_control/src/morai_udp_sender_node.cpp`
- Modify: `src/vehicle_control/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: MORAI Cmd Control Host PORT `9093`
- Produces: `destination_port=9093`인 launch 파라미터와 C++ fallback

- [ ] **Step 1: 현재 설정이 새 요구사항을 만족하지 않는지 확인**

Run:

```bash
python3 - <<'PY'
from pathlib import Path

config = Path("src/vehicle_control/config/cyvox_mx.yaml").read_text()
node = Path("src/vehicle_control/src/morai_udp_sender_node.cpp").read_text()
assert "destination_port: 9093" in config
assert 'private_node_.param("destination_port", destination_port, 9093);' in node
PY
```

Expected: FAIL because both defaults are still `9095`.

- [ ] **Step 2: 실행 기본값을 `9093`으로 변경**

In `src/vehicle_control/config/cyvox_mx.yaml`:

```yaml
morai_udp_sender_node:
  destination_port: 9093
```

In `src/vehicle_control/src/morai_udp_sender_node.cpp`:

```cpp
int destination_port = 9093;
private_node_.param("destination_port", destination_port, 9093);
```

- [ ] **Step 3: 사용자 문서를 `9093`으로 변경**

`src/vehicle_control/README.md`와 루트 `README.md`의 MORAI Cmd Control 수신 포트
설명 및 파라미터 표를 `9093`으로 바꾼다. UDP 단위 테스트의 `9095`는 수정하지
않는다.

- [ ] **Step 4: 기본값 검사를 다시 실행**

Run the Step 1 command again.

Expected: PASS.

- [ ] **Step 5: 빌드와 전체 테스트 실행**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin build --install
catkin test
catkin_test_results
```

Expected: build exit code `0`, test failures `0`.

- [ ] **Step 6: 설치 공간과 launch 파라미터 확인**

Run:

```bash
rg -n "9093" install/share/vehicle_control/config/cyvox_mx.yaml \
  install/share/vehicle_control/README.md
roslaunch --dump-params vehicle_control cyvox_morai.launch |
  rg "destination_port: 9093"
```

Expected: 설치된 설정과 launch dump에서 모두 `9093` 확인.

- [ ] **Step 7: 구현 커밋**

```bash
git add README.md src/vehicle_control/config/cyvox_mx.yaml \
  src/vehicle_control/src/morai_udp_sender_node.cpp \
  src/vehicle_control/README.md
git commit -m "fix: use MORAI default control port"
```
