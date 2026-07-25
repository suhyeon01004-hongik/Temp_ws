# 통합 실행 구성

`morai_bringup`은 차량 description, MORAI UDP 센서 수신, Velodyne LiDAR,
GPS localization과 path manager의 **실행 조합**을 관리한다. 센서 파싱,
좌표 변환, 경로 계산 같은 기능은 구현하지 않고 각 기능 패키지의 launch와
config를 연결하는 역할만 맡는다.

상세한 설정 변경 절차는
[`docs/CONFIGURATION_GUIDE_KO.md`](docs/CONFIGURATION_GUIDE_KO.md)를 참고한다.

## 실행 구성

| launch | 용도 | 기본 실행 항목 |
| --- | --- | --- |
| `molit_2026_sensors.launch` | 센서 계층 | description, UDP bridge, LiDAR |
| `molit_2026_localization.launch` | localization 계층 | GPS projector, IMU adapter, direct fusion |
| `molit_2026_path_manager.launch` | 경로 계층 | global/local path publisher |
| `molit_2026_stack.launch` | 운영 전체 stack | 위 세 계층 전부 |
| `visualization/path.launch` | 경로 시각화 | path marker, path RViz |
| `visualization/lidar.launch` | LiDAR 시각화 | LiDAR RViz |
| `visualization/path_lidar.launch` | 통합 시각화 | path marker, path+LiDAR RViz |

## MORAI에서 먼저 불러올 파일

| 종류 | 저장소 기준 파일 | MORAI `SaveFile` 대상 |
| --- | --- | --- |
| 센서 preset | [`0725demo.json`](../ioniq5_description/config/morai_presets/0725demo.json) | `Sensor/25.S4.MolitComp03/` |
| 빈 시험 시나리오 | [`2026_molit_path_start_empty.json`](config/morai_scenarios/R_KR_PR_K-city_2025/2026_molit_path_start_empty.json) | `Scenario/R_KR_PR_K-city_2025/` |

빈 시험 시나리오는 전역경로 시작점에 정지한 Ego 차량만 포함하며 NPC, 보행자,
오브젝트와 자동 주행 waypoint는 포함하지 않는다.

센서만 실행:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup molit_2026_sensors.launch
```

localization만 실행:

```bash
roslaunch morai_bringup molit_2026_localization.launch
```

path manager만 실행:

```bash
roslaunch morai_bringup molit_2026_path_manager.launch
```

센서부터 path manager까지 전체 실행:

```bash
roslaunch morai_bringup molit_2026_stack.launch
```

시각화는 기능 stack과 별도 터미널에서 실행한다.

```bash
roslaunch morai_bringup path.launch
roslaunch morai_bringup lidar.launch
roslaunch morai_bringup path_lidar.launch
```

`visualization/` 아래 launch는 `morai_visualization`의 기능을 다시 구현하지
않고 팀 공통 실행 진입점만 제공한다. 시각화 launch는 센서나 localization을
자동으로 실행하지 않는다. ROS1은 패키지 내부의 launch 파일을 재귀 검색하므로
실행 명령에는 `visualization/` 폴더명을 쓰지 않는다.

## 주요 launch 인자

### 센서 launch

| 인자 | 기본값 | 역할 |
| --- | --- | --- |
| `publish_description` | `true` | 차량 URDF와 센서 고정 TF 발행 |
| `use_lidar` | `true` | 공식 Velodyne VLP-16 driver 실행 |
| `bridge_config` | `molit_2026.yaml` | UDP 스트림 설정 선택 |
| `vehicle_config` | IONIQ 5 기본 YAML | 차량 제원 설정 선택 |
| `sensor_mount_config` | 대회 센서 YAML | 센서 frame·장착값 선택 |
| `lidar_device_ip` | 빈 문자열 | 허용할 LiDAR 송신 IP |
| `lidar_port` | `2368` | LiDAR UDP 포트 |
| `lidar_hz` | `15.0` | RPM 계산용 LiDAR 주기 |
| `lidar_frame` | `lidar_link` | PointCloud2 frame |
| `lidar_cut_angle` | `0.0` | 매 회전의 PointCloud 절단 방향(rad) |
| `lidar_fixed_frame` | 빈 문자열 | 주행 왜곡 보정에 사용할 고정 frame |
| `lidar_target_frame` | 빈 문자열 | 보정 후 PointCloud 출력 frame |

### Localization과 path launch

| launch | 인자 | 역할 |
| --- | --- | --- |
| `molit_2026_localization.launch` | `config` | localization YAML 선택 |
| `molit_2026_path_manager.launch` | `config` | local path 정책 YAML 선택 |
| `molit_2026_path_manager.launch` | `path_file` | 전역경로 파일 선택 |

`molit_2026_stack.launch`는 위 인자를 각각 `localization_config`,
`route_path_config`, `global_path_file` 이름으로 받아 해당 계층에 전달한다.
센서 launch와 전체 stack 모두 `lidar_fixed_frame`,
`lidar_target_frame`의 기본값은 빈 문자열이다. 따라서 LiDAR가 시작될 때
localization TF가 아직 없어도 nodelet manager가 종료되지 않는다.

LiDAR 없이 실행하려면:

```bash
roslaunch morai_bringup molit_2026_sensors.launch use_lidar:=false
```

센서 launch에는 localization이나 path manager의 on/off 옵션을 두지 않는다.
필요한 계층의 launch만 선택하거나 전체가 필요하면 stack launch를 사용한다.

## 설정의 소유 패키지

| 변경 대상 | 수정할 파일 | 소유 패키지 |
| --- | --- | --- |
| 차량 제원·센서 장착 TF | `vehicle_specs.yaml`, `molit_2026_sensor_mounts.yaml` | `ioniq5_description` |
| MORAI UDP IP·port·topic | `molit_2026.yaml` | `morai_udp_bridge` |
| GPS CRS·map offset, yaw 보정·동기화 | `molit_2026_kcity.yaml` | `morai_localization` |
| 전역경로·local path 20 pose 정책 | `molit_2026_kcity_route_path.yaml` | `morai_path_manager` |
| marker topic·색상 크기 | `path_visualizer.yaml` | `morai_visualization` |
| MORAI 시험 시나리오 | `config/morai_scenarios/**/*.json` | `morai_bringup` |
| 어떤 노드를 함께 실행할지 | `launch/*.launch` | `morai_bringup` |

같은 값을 bringup에 다시 적어 두지 않는다. 노드 동작값은 소유 패키지의 config,
조합과 파일 선택만 bringup launch에서 관리한다.

## 기본 토픽 흐름

```text
MORAI GPS UDP :9301
  -> /sensors/gps/fix
  -> /localization/gps/local_point
                                  ─┐
                                   ├-> /localization/pose -> /local_path

MORAI IMU UDP :9303
  -> /sensors/imu/data
  -> /localization/imu/data      ─┘

/localization/pose
  -> /localization/odometry
  -> map -> base_footprint -> base_link

공식 경로 파일
  -> /global_path

/global_path + /local_path + 최종 pose
  -> /visualization/path
```

LiDAR는 MORAI native Velodyne UDP를 공식 driver가 받아 `/lidar3D`로 발행한다.
좌표 TF는 `map -> base_footprint -> base_link -> sensor_link` 순서다.
`base_link`는 뒷차축 중앙, `base_footprint`는 그 지면 투영점이며 둘의 높이
차이는 현재 `0.37 m`다.

LiDAR PointCloud는 `lidar_cut_angle:=0.0`을 기준으로 매번 같은 방위에서
한 회전을 끝낸다. 이 값은 패킷 개수로만 스캔을 자를 때 발생하던 회전하는
경계와 중복 구간을 방지한다.

Velodyne의 `fixed_frame` 기반 주행 왜곡 보정은 현재 기본 비활성화 상태다.
설치된 driver는 시작 시 `map -> lidar_link`가 아직 연결되지 않으면
`tf2::ConnectivityException`으로 nodelet manager 전체를 종료할 수 있다.
향후 TF 준비 상태를 확인한 뒤 transform nodelet을 시작하는 별도 실행 절차를
추가한 후 활성화한다.

## 팀원이 변경할 때 주의할 점

- MORAI Sensor Edit의 Destination Port와 UDP YAML의 port는 같아야 한다.
- 센서 장착값은 MORAI `0725demo.json`과 description YAML을 함께 바꾼다.
- 현재 GPS는 `[0.0, 0.0, 1.2]`, 30 Hz, UDP `9301`, noise off가 기준이다.
- 현재 IMU는 `[1.5, 0.0, 0.5]`, 50 Hz, UDP `9303`, noise off가 기준이다.
- 현재 localization은 필터 없이 GPS X/Y와 IMU yaw를 직접 결합한다. noise를
  켤 때는 fusion 구현을 필터 기반으로 교체해야 한다.
- map이나 시나리오가 바뀌면 localization offset과 전역경로 파일을 한 쌍으로
  검증한다.
- path manager만 실행하면 `/global_path`는 즉시 나오지만,
  `/localization/pose`가 들어오기 전에는 `/local_path`가 나오지 않는다.
- 이미 실행 중인 launch는 새 install 내용을 자동으로 읽지 않으므로 빌드 후
  기존 프로세스를 종료하고 다시 실행한다.

## 점검

```bash
rostopic list
rostopic echo -n 1 /sensors/gps/fix
rostopic echo -n 1 /sensors/imu/data
rostopic echo -n 1 /localization/gps/local_point
rostopic echo -n 1 /localization/imu/data
rostopic echo -n 1 /localization/pose
rostopic echo -n 1 /localization/odometry
rosrun tf tf_echo map base_link
rostopic echo -n 1 /global_path
rostopic echo -n 1 /local_path
rostopic echo /diagnostics
```

`/local_path`의 현재 계약과 제어팀 인수인계 내용은
[`morai_path_manager/docs/CONTROL_PATH_INTERFACE_KO.md`](../morai_path_manager/docs/CONTROL_PATH_INTERFACE_KO.md)에
정리되어 있다.
