# RViz 시각화

`morai_visualization`은 실행 기능과 분리된 디버그 marker 및 RViz profile
패키지다. 다른 패키지의 토픽을 구독해 보여 줄 뿐 localization, 경로 생성,
센서 수신이나 LiDAR decoding은 수행하지 않는다.

path와 LiDAR 단독 화면은 분리해 유지하고, 두 입력을 함께 확인하는 조합
launch/profile만 별도로 제공한다.

팀 공통 실행은 `morai_bringup/launch/visualization`의 wrapper를 사용한다.
아래 `morai_visualization` 직접 실행 명령은 패키지 자체를 개발할 때도 사용할
수 있다.

## 제공하는 화면

| launch | RViz 기준 | 표시 내용 |
| --- | --- | --- |
| `path.launch` | Fixed `map`, Target `base_link` | 전역경로, 시작점, local path, 뒷차축 원점·heading, 차량 모델, 최근접점 |
| `lidar.launch` | Fixed/Target `base_link` | 차량 모델, 센서 TF, `/lidar3D` point cloud |
| `path_lidar.launch` | Fixed `map`, Target `base_link` | 기존 path 완전 탑뷰에 `/lidar3D`만 추가 |

### Path 표시 의미

| 표시 | marker namespace | 의미 |
| --- | --- | --- |
| 청록색 선 | `global_path` | 전체 전역경로 |
| 초록색 구와 `START` | `global_path_start` | 전역경로 파일의 첫 점 |
| 주황색 선 | `local_path` | 현재 위치부터 전방 20 pose |
| 작은 빨간색 구와 `REAR AXLE` | `vehicle_origin` | `base_link`, 즉 뒷차축 중앙 |
| 빨간색 화살표 | `vehicle_heading` | `base_link`의 전방 `+x` |
| 노란색 구 | `nearest_waypoint` | 전역경로 최근접점 |
| 분홍색 선 | `nearest_waypoint` | 최종 위치와 최근접점 사이 오차 |

차량에 붙은 두 marker는 `base_link` 좌표로 발행한다. localization의
`map -> base_footprint -> base_link` TF를 따라가므로 RobotModel, 원점과
heading이 서로 어긋나지 않는다. 흰색 이동 궤적은 발행하지 않는다.
TF 부모-자식 연결선은 경로와 혼동되지 않도록 path 화면에서 기본으로 숨긴다.
좌표를 점검할 때만 RViz의 `Coordinate Frames`를 켠다.

## Path 시각화 실행

모든 표시를 보려면 `/global_path`, `/local_path`, `/localization/pose`와
`map -> base_link` TF가 필요하다. 입력이 일부 없더라도 visualizer는 실행되며
수신한 항목만 표시한다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup path.launch
```

marker만 실행하고 RViz는 띄우지 않을 때:

```bash
roslaunch morai_bringup path.launch start_rviz:=false
```

launch 인자:

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `config` | `config/path_visualizer.yaml` | marker 설정 YAML |
| `start_rviz` | `true` | RViz 실행 여부 |
| `publish_map_frame_anchor` | `true` | RViz가 `map` TF root를 인식하도록 보조 static TF 발행 |
| `rviz_config` | `rviz/path.rviz` | RViz profile 선택 |

`path_visualization_anchor`는 RViz가 `map`을 알도록 만드는 identity child일
뿐 차량 위치나 heading을 만들어 내지 않는다.

## Path marker 설정

`config/path_visualizer.yaml`:

| 파라미터 | 현재값 | 설명 |
| --- | --- | --- |
| `global_path_topic` | `/global_path` | 전역경로 입력 |
| `local_path_topic` | `/local_path` | local path 입력 |
| `localization_topic` | `/localization/pose` | 최종 위치와 yaw 입력 |
| `marker_topic` | `/visualization/path` | MarkerArray 출력 |
| `frame_id` | `map` | 입력·marker 기준 frame |
| `base_frame_id` | `base_link` | 차량 원점·heading marker 기준 frame |
| `global_line_width` | `0.15` | 전역경로 선 두께(m) |
| `local_line_width` | `0.35` | local path 선 두께(m) |
| `vehicle_origin_diameter` | `0.30` | 뒷차축 원점 구 지름(m) |
| `vehicle_origin_label_height` | `0.35` | `REAR AXLE` 글자 높이(m) |
| `nearest_point_diameter` | `0.7` | 최근접점 구 지름(m) |
| `show_global_path_start` | `true` | 전역경로 시작점 표시 여부 |
| `global_start_diameter` | `1.4` | 시작점 구 지름(m) |
| `global_start_text_height` | `1.0` | `START` 글자 높이(m) |

marker 출력은 `/visualization/path`의
`visualization_msgs/MarkerArray` 하나로 묶인다. 기능별로 namespace가 나뉘어
있어 RViz의 MarkerArray 항목에서 표시를 개별로 켜고 끌 수 있다.

## LiDAR 시각화 실행

LiDAR driver와 `robot_state_publisher`는 bringup에서 먼저 실행되어야 한다.

```bash
roslaunch morai_bringup lidar.launch
```

이 launch는 계산 노드를 추가하지 않고 `rviz/lidar.rviz`만 연다.

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `start_rviz` | `true` | RViz 실행 여부 |
| `lidar_topic` | `/lidar3D` | PointCloud2 입력 토픽 |
| `rviz_config` | `rviz/lidar.rviz` | LiDAR RViz profile |

LiDAR 단독 화면의 Fixed Frame은 `base_link`지만 Grid는 지면 frame인
`base_footprint`에 그린다.

## Path + LiDAR 통합 화면

센서 bridge, localization, path manager와 차량 description 전체를 먼저 실행한다.

```bash
roslaunch morai_bringup molit_2026_stack.launch
```

다른 터미널에서 통합 RViz를 연다.

```bash
roslaunch morai_bringup path_lidar.launch
```

이 launch는 새 LiDAR 연산 노드를 만들지 않는다. 기존 path visualizer를
재사용하고 `/lidar3D`를 같은 RViz에 추가한다. `lidar_topic:=...`으로 다른
PointCloud2 토픽도 선택할 수 있다.

화면 구성과 카메라는 `path.launch`의 완전 탑뷰와 동일하고 PointCloud2만
추가된다. LiDAR `Decay Time`은 `0.0`이므로 발행 주기가 변해도 최신 cloud가
다음 입력까지 유지되고, 과거 여러 cloud가 누적되지는 않는다. 점 색은 Grid와
구분되는 부드러운 연두색 FlatColor `(120, 220, 150)`, Alpha `0.45`를 사용한다.

## 현재 제한

path 화면의 위치와 yaw는 GPS+IMU localization이 모두 정상일 때만 갱신된다.
noise를 켠 뒤 direct localization 출력이 흔들리는 문제는 visualization에서
숨기지 않고 localization filter 계층에서 해결한다.

## 확인과 문제 진단

```bash
rostopic echo -n 1 /visualization/path
rostopic echo -n 1 /global_path
rostopic echo -n 1 /local_path
rostopic echo -n 1 /localization/pose
rosrun tf tf_echo map base_link
rostopic hz /lidar3D
```

- RViz `Global Status: Error`이면 Fixed Frame과 실제 TF 존재 여부를 확인한다.
- 경로만 보이고 차량 위치가 없으면 localization 토픽이 들어오지 않은 것이다.
- LiDAR만 안 보이면 `/lidar3D`의 `header.frame_id`가 `lidar_link`인지 확인한다.
- LiDAR가 깜빡이면 RViz PointCloud2의 `Decay Time`이 `0.0`인지 확인한다.
- GPS 원점은 차체 중앙이 아니라 흰 차체 뒤쪽의 뒷바퀴축 사이에 보여야 정상이다.
- 전역경로 시작점은 localization 입력 없이도 `/global_path`가 오면 표시된다.
- 빌드 후에도 새 marker가 안 보이면 기존 launch를 종료하고 install 환경을
  다시 source한 뒤 재실행한다.
