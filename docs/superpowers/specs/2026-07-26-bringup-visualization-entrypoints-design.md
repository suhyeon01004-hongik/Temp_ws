# Bringup 시각화 진입점 정리 설계

## 목표

- 역할이 겹치는 `gps_localization_path_test.launch`를 제거한다.
- 기능 실행과 시각화 실행을 분리한다.
- 팀원이 `morai_bringup`만 보고 센서, localization, path manager와 시각화를
  각각 실행할 수 있게 한다.

## 구조

기능 launch는 현재처럼 `morai_bringup/launch` 바로 아래에 둔다.

```text
launch/
├─ molit_2026_sensors.launch
├─ molit_2026_localization.launch
├─ molit_2026_path_manager.launch
├─ molit_2026_stack.launch
└─ visualization/
   ├─ path.launch
   ├─ lidar.launch
   └─ path_lidar.launch
```

`visualization` 하위 launch는 시각화 기능을 다시 구현하지 않는다. 각각
`morai_visualization` 패키지의 같은 이름 launch를 include하고 필요한 인자를
그대로 전달하는 bringup 진입점이다.

## 실행 방법

```bash
# 센서 + localization + path manager
roslaunch morai_bringup molit_2026_stack.launch

# 경로만 시각화
roslaunch morai_bringup path.launch

# LiDAR만 시각화
roslaunch morai_bringup lidar.launch

# 경로와 LiDAR를 함께 시각화
roslaunch morai_bringup path_lidar.launch
```

시각화 launch는 센서나 localization을 자동으로 실행하지 않는다. 입력 데이터가
필요하면 별도 터미널에서 필요한 기능 bringup 또는 전체 stack을 먼저 실행한다.
ROS1 `roslaunch`는 패키지 내부를 재귀 검색하므로 파일은 `visualization/`
폴더에 두되 실행할 때는 하위 폴더 경로 없이 파일명만 전달한다.

## 제거와 문서 갱신

- `morai_bringup/launch/gps_localization_path_test.launch`를 삭제한다.
- 기존 통합 시험 launch를 안내하던 README와 설정 가이드를 기능 stack 실행과
  별도 시각화 실행으로 교체한다.
- launch 구성 테스트는 삭제된 파일이 존재하지 않는지, 세 wrapper가 올바른
  `morai_visualization` launch만 include하는지 검사한다.

## 검증

- 구조 테스트를 먼저 실패시키고 wrapper 및 삭제를 구현한 뒤 통과시킨다.
- `roslaunch morai_bringup <name>.launch --nodes`로 각 wrapper의
  실제 노드 구성을 확인한다.
- 전체 catkin 테스트와 install을 실행하고 install 공간에도 하위 launch 폴더가
  설치되는지 확인한다.
