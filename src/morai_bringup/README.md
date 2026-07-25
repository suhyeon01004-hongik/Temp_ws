# 통합 실행 구성

`morai_bringup`은 차량 description, MORAI UDP 센서 수신, Velodyne LiDAR,
GPS localization과 path manager를 필요한 조합으로 실행한다. 센서 파싱,
좌표 변환, 경로 계산 같은 기능은 구현하지 않고 각 패키지의 launch와 config를
연결하는 역할만 맡는다.

상세한 설정 변경 절차는
[`docs/CONFIGURATION_GUIDE_KO.md`](docs/CONFIGURATION_GUIDE_KO.md)를 참고한다.

## 실행 구성

| launch | 용도 | 기본 실행 항목 |
| --- | --- | --- |
| `molit_2026_sensors.launch` | 전체 센서/주행 스택 구성 | description, UDP bridge, LiDAR |
| `gps_localization_path_test.launch` | GPS+IMU부터 경로·RViz까지 통합 점검 | GPS+IMU bridge, direct localization, path manager, path visualizer, RViz |

## MORAI에서 먼저 불러올 파일

| 종류 | 저장소 기준 파일 | MORAI `SaveFile` 대상 |
| --- | --- | --- |
| 센서 preset | [`0725demo.json`](../ioniq5_description/config/morai_presets/0725demo.json) | `Sensor/25.S4.MolitComp03/` |
| 빈 시험 시나리오 | [`2026_molit_path_start_empty.json`](config/morai_scenarios/R_KR_PR_K-city_2025/2026_molit_path_start_empty.json) | `Scenario/R_KR_PR_K-city_2025/` |

빈 시험 시나리오는 전역경로 시작점에 정지한 Ego 차량만 포함하며 NPC, 보행자,
오브젝트와 자동 주행 waypoint는 포함하지 않는다.

전체 센서 기본 실행:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup molit_2026_sensors.launch
```

GPS+IMU localization과 경로를 함께 사용할 때:

```bash
roslaunch morai_bringup molit_2026_sensors.launch \
  use_gps_localization:=true
```

GPS+IMU/localization/path/RViz만 점검할 때:

```bash
roslaunch morai_bringup gps_localization_path_test.launch
```

`gps_localization_path_test.launch`는 운영 기능의 임시 복사본이 아니라 여러
패키지를 한 번에 확인하기 위한 통합 시험 launch다. localization만 독립
확인하려면 `morai_localization` launch를 직접 실행해야 한다.

## 주요 launch 인자

| 인자 | 기본값 | 역할 |
| --- | --- | --- |
| `publish_description` | `true` | 차량 URDF와 센서 고정 TF 발행 |
| `use_lidar` | `true` | 공식 Velodyne VLP-16 driver 실행 |
| `use_gps_localization` | `false` | GPS projector, IMU adapter와 direct fusion 실행 |
| `use_path_manager` | `use_gps_localization` 값 | 전역/local path publisher 실행 |
| `bridge_config` | `molit_2026.yaml` | UDP 스트림 설정 선택 |
| `vehicle_config` | IONIQ 5 기본 YAML | 차량 제원 설정 선택 |
| `sensor_mount_config` | 대회 센서 YAML | 센서 frame·장착값 선택 |
| `localization_config` | K-City 기본 YAML | 좌표 투영 설정 선택 |
| `route_path_config` | K-City 경로 YAML | local path 정책 선택 |
| `global_path_file` | 공식 대회 경로 | 전역경로 파일 선택 |
| `lidar_device_ip` | 빈 문자열 | 허용할 LiDAR 송신 IP |
| `lidar_port` | `2368` | LiDAR UDP 포트 |
| `lidar_hz` | `15.0` | RPM 계산용 LiDAR 주기 |
| `lidar_frame` | `lidar_link` | PointCloud2 frame |

LiDAR 없이 실행하려면:

```bash
roslaunch morai_bringup molit_2026_sensors.launch use_lidar:=false
```

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

## 팀원이 변경할 때 주의할 점

- MORAI Sensor Edit의 Destination Port와 UDP YAML의 port는 같아야 한다.
- 센서 장착값은 MORAI `0725demo.json`과 description YAML을 함께 바꾼다.
- 현재 GPS는 `[0.0, 0.0, 1.2]`, 30 Hz, UDP `9301`, noise off가 기준이다.
- 현재 IMU는 `[1.5, 0.0, 0.5]`, 50 Hz, UDP `9303`, noise off가 기준이다.
- 현재 localization은 필터 없이 GPS X/Y와 IMU yaw를 직접 결합한다. noise를
  켤 때는 fusion 구현을 필터 기반으로 교체해야 한다.
- map이나 시나리오가 바뀌면 localization offset과 전역경로 파일을 한 쌍으로
  검증한다.
- `use_path_manager`만 켤 수는 있지만 localization 입력 토픽이 없으면
  `/global_path`만 나오고 `/local_path`는 나오지 않는다.
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
