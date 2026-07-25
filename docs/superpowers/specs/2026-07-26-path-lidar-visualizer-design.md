# 경로·LiDAR 통합 시각화 설계

## 목표

MORAI 차량 로컬 원점과 ROS TF를 동일하게 정의하고, RViz 한 화면에서
전역경로·local path·차량·현재 위치·yaw·LiDAR point cloud를 확인한다.

## 좌표계 계약

- `base_link`: MORAI 차량 로컬 원점인 뒷차축 허브 중앙
- `base_footprint`: `base_link`의 지면 투영점
- `map -> base_footprint`: GPS XY와 IMU yaw로 localization이 발행
- `base_footprint -> base_link`: 뒷차축 반지름만큼 위로 이동하는 고정 TF
- 센서 장착 TF: 기존처럼 `base_link` 기준
- ROS 축: `x` 전방, `y` 좌측, `z` 위

GPS `(0, 0, 1.2)`는 뒷차축 바로 위, LiDAR `(1.5, 0, 1.25)`는 뒷차축에서
전방 1.5 m 위치다.

## 차량 디버그 모델

MORAI가 공개한 IONIQ 5 길이·폭·wheelbase·overhang은 유지한다. MORAI 문서의
`height=2.434 m`는 실제 센서 장착 화면과 맞지 않으므로 동역학 제원으로
보존하고 RViz 형상에는 별도 시각화 제원을 사용한다.

- 차체 X 범위: `-rear_overhang`부터 `wheelbase + front_overhang`
- 차체 중심 X: `1.5275 m`
- RViz 차체 높이: `1.605 m`
- 뒷차축 높이/휠 반지름: `0.37 m`
- 차체 Z 범위: 지면 `-0.37 m`부터 지붕 `1.235 m`
- 앞·뒤 바퀴를 표시해 뒷차축 원점을 눈으로 확인 가능하게 구성

## 경로 마커

현재 빨간 구는 GPS가 아니라 차량 원점이다. namespace를
`current_position`에서 `vehicle_origin`으로 바꾸고 크기를 줄인다.
마커와 heading 화살표는 `base_link` 기준으로 발행해 TF와 정확히 일치시킨다.
`REAR AXLE` 라벨로 의미를 명확히 한다.

최근접 경로 계산과 좌표 텍스트는 기존 `/localization/pose`를 그대로 사용한다.

## RViz 구성

- 기존 `path.rviz`: 경로 중심의 가벼운 화면으로 유지
- 기존 `lidar.rviz`: localization 없이 센서만 점검하는 `base_link` 고정 화면
- 신규 `path_lidar.rviz`: Fixed Frame `map`, Target Frame `base_link`
- 신규 `path_lidar.launch`: 기존 path marker 노드를 재사용하고 통합 RViz만 실행

`path_lidar.rviz`의 기본 화면은 `path.rviz`와 완전히 같은
`rviz/TopDownOrtho`다. Grid, 경로, marker, RobotModel, TF, 확대율과
카메라 Target도 path-only 화면과 동일하게 유지하고 `/lidar3D`
PointCloud2 display만 추가한다.

LiDAR를 재가공하거나 누적하는 새 노드는 만들지 않는다. `/lidar3D`
`sensor_msgs/PointCloud2`를 `map -> base_footprint -> base_link -> lidar_link`
TF로 RViz가 직접 변환한다.

## LiDAR 표시 수명

실측 `/lidar3D` 주기는 약 `6.2 Hz`이고 메시지 간격은 보통 `0.16 s`,
관측 최대값은 `0.318 s`다. 기존 `Decay Time=0.15 s`는 다음 cloud가 오기
전에 현재 cloud를 지워 깜빡임을 만든다.

통합 및 LiDAR-only profile의 `Decay Time`은 `0.0 s`로 둔다. RViz에서 0은
최신 cloud 하나를 다음 입력까지 유지하므로 입력 주기가 변해도 빈 화면 구간이
생기지 않고 여러 과거 scan도 누적하지 않는다. 주기 측정 republisher나 custom
RViz plugin은 추가하지 않는다.

사용자는 목적에 따라 다음 세 화면을 선택한다.

- `path.launch`: 기존 완전 탑뷰 경로 화면
- `lidar.launch`: LiDAR와 센서 TF만 보는 3D 화면
- `path_lidar.launch`: 기존 경로 탑뷰에 LiDAR만 추가한 화면

Path 및 Path+LiDAR 화면에서는 지도 원점부터 차량까지 이어지는 TF 연결선이
경로처럼 보이지 않도록 TF display를 기본 비활성화한다. LiDAR-only 화면은 센서
장착 검증 목적이므로 TF display를 유지한다. PointCloud는 Grid와 구분되는
연두색 FlatColor `(120, 255, 120)`로 표시한다.

## 검증

- xacro 결과에서 차체·바퀴·`base_footprint -> base_link` 위치 확인
- 실행 중 path marker의 frame과 namespace 확인
- 통합 RViz profile에 경로, RobotModel, TF, `/lidar3D`가 모두 포함되는지 확인
- 통합 profile의 현재 view가 path-only와 같은 `rviz/TopDownOrtho`인지 확인
- 두 LiDAR profile의 `Decay Time`이 `0.0`인지 확인
- 패키지 빌드와 전체 테스트 실행
