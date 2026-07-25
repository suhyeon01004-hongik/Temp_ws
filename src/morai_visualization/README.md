# RViz 시각화

`morai_visualization`은 실행 기능과 분리된 디버그 marker 및 RViz profile
패키지다. 다른 패키지의 토픽을 구독해 보여 줄 뿐 localization, 경로 생성,
센서 수신이나 LiDAR decoding은 수행하지 않는다.

path와 LiDAR 시각화를 별도 launch/profile로 나눠 두었으므로 한쪽 표시를
고칠 때 다른 기능 패키지에 시각화 코드가 섞이지 않는다.

## 제공하는 화면

| launch | RViz 기준 | 표시 내용 |
| --- | --- | --- |
| `path.launch` | Fixed `map`, Target `base_link` | 전역경로, 시작점, local path, 최종 위치·yaw, 차량 모델, 최근접점 |
| `lidar.launch` | `base_link` | 차량 RobotModel, 센서 TF, `/lidar3D` point cloud |

### Path 표시 의미

| 표시 | marker namespace | 의미 |
| --- | --- | --- |
| 청록색 선 | `global_path` | 전체 전역경로 |
| 초록색 구와 `START` | `global_path_start` | 전역경로 파일의 첫 점 |
| 주황색 선 | `local_path` | 현재 위치부터 전방 20 pose |
| 빨간색 구 | `current_position` | 최종 localization 위치 |
| 빨간색 화살표 | `vehicle_heading` | IMU에서 얻은 최종 yaw |
| 노란색 구 | `nearest_waypoint` | 전역경로 최근접점 |
| 분홍색 선 | `nearest_waypoint` | 최종 위치와 최근접점 사이 오차 |

흰색 이동 궤적은 발행하지 않는다. localization의
`map -> base_footprint -> base_link` TF를 RViz 카메라가 따라가므로 차량 위치는
화면 중앙에 유지되고 IMU yaw에 따라 map이 상대적으로 회전한다.

## Path 시각화 실행

모든 표시를 보려면 `/global_path`, `/local_path`, `/localization/pose`와
`map -> base_link` TF가 필요하다. 입력이 일부 없더라도 visualizer는 실행되며
수신한 항목만 표시한다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_visualization path.launch
```

marker만 실행하고 RViz는 띄우지 않을 때:

```bash
roslaunch morai_visualization path.launch start_rviz:=false
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
| `global_line_width` | `0.15` | 전역경로 선 두께(m) |
| `local_line_width` | `0.35` | local path 선 두께(m) |
| `current_position_diameter` | `1.0` | 현재 위치 구 지름(m) |
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
roslaunch morai_visualization lidar.launch
```

이 launch는 계산 노드를 추가하지 않고 `rviz/lidar.rviz`만 연다.

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `start_rviz` | `true` | RViz 실행 여부 |
| `rviz_config` | `rviz/lidar.rviz` | LiDAR RViz profile |

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
- 전역경로 시작점은 localization 입력 없이도 `/global_path`가 오면 표시된다.
- 빌드 후에도 새 marker가 안 보이면 기존 launch를 종료하고 install 환경을
  다시 source한 뒤 재실행한다.
