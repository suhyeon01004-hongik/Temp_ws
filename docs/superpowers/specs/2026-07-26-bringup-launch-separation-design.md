# Bringup launch 책임 분리 설계

## 목표

`morai_bringup`에 센서, localization, path manager의 대회 실행 진입점을 각각
두고, 전체 실행 launch가 세 진입점을 조합한다. 개별 launch는 다른 계층의
설정이나 on/off 옵션을 알지 않는다.

## 파일과 책임

- `molit_2026_sensors.launch`: description, UDP bridge, Velodyne driver
- `molit_2026_localization.launch`: `morai_localization/localization.launch`
- `molit_2026_path_manager.launch`: `morai_path_manager/route_path_publisher.launch`
- `molit_2026_stack.launch`: 위 세 launch를 무조건 조합하는 운영 진입점
- `gps_localization_path_test.launch`: 세 bringup과 path RViz를 조합하는 시험 진입점

센서 launch에서는 `use_gps_localization`, `use_path_manager`,
`localization_config`, `route_path_config`, `global_path_file`을 제거한다.
전체 stack에는 기능 on/off 옵션을 두지 않는다. 필요한 일부 기능만 실행할 때는
해당 개별 launch를 직접 선택한다. 시각화는 계속 `morai_visualization`에서
별도로 실행한다.

## 검증

- 센서 launch에 localization/path 인자와 include가 없는지 검사
- localization/path bringup이 각 소유 패키지 launch만 포함하는지 검사
- 전체 stack과 test launch가 세 bringup을 포함하는지 검사
- `roslaunch --nodes`로 개별/전체 노드 구성을 확인
- 전체 빌드, 테스트와 install 반영 확인
