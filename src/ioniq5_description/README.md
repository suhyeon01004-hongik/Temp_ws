# IONIQ 5 차량·센서 모델

`ioniq5_description`은 차량 제원, 센서 장착 위치와 센서 frame을 한곳에서
관리하고 `robot_description`과 고정 TF를 발행하는 패키지다. 통신, localization,
경로 생성은 이 패키지에서 수행하지 않는다.

## 좌표계 기준

MORAI IONIQ 5의 차량-local 축을 그대로 사용한다.

- 원점: 차량 뒷바퀴 축 중앙
- `x`: 차량 전방
- `y`: 차량 좌측
- `z`: 위쪽
- ROS 기준 frame: `base_link`
- 지면 기준 보조 frame: `base_footprint`

센서 위치는 모두 `base_link` 기준이다. 현재 단순 차체 box는 센서와 TF를
확인하기 위한 RViz 형상이며 정밀 외장이나 동역학 모델이 아니다.

## 현재 센서 TF

| 센서 | frame | `xyz` (m) | 상태 |
| --- | --- | --- | --- |
| GPS | `gps_link` | `[0.0, 0.0, 1.2]` | `0725demo.json`과 일치 |
| LiDAR | `lidar_link` | `[1.5, 0.0, 1.25]` | `0725demo.json`과 일치 |
| 전방 카메라 | `camera_front_link` | `[1.9, 0.0, 1.2]` | 규정값 반영 |
| 좌측 카메라 | `camera_left_link` | `[1.15, 0.65, 1.2]` | 규정값 반영 |
| 우측 카메라 | `camera_right_link` | `[1.15, -0.65, 1.2]` | 규정값 반영 |
| IMU | `imu_link` | `[1.5, 0.0, 0.5]` | `0725demo.json`과 일치 |

카메라는 각 `camera_*_link` 아래에 ROS optical-axis 규약을 따르는
`camera_*_optical_frame`도 발행한다. `transform_verified: false`인 센서는
잘못된 0점 TF가 생기지 않도록 URDF에 추가하지 않는다.

## 주로 수정하는 파일

| 파일 | 수정 대상 |
| --- | --- |
| `config/vehicle_specs.yaml` | 차량 길이·폭·높이, wheelbase, 조향 한계, 기준 frame |
| `config/molit_2026_sensor_mounts.yaml` | 센서 `frame_id`, `xyz_m`, `rpy_deg`, 카메라 해상도·FOV |
| `config/morai_presets/0725demo.json` | MORAI에서 실제로 불러오는 현재 센서 preset |
| `urdf/ioniq5_base.xacro` | RViz용 차체 형상 |
| `urdf/molit_sensors.xacro` | 센서 link와 fixed joint 생성 방식 |

현재 MORAI load 파일은 `config/morai_presets/0725demo.json`이다.
`0716demo.json`과 `2026_molit_comp_cam_set.json`은 이전 원본 비교용이므로 현재
설정을 바꾸기 위해 수정하지 않는다.

IMU는 뒷차축 원점에서 전방 `1.5 m`, 차량 중심선, 높이 `0.5 m`의 실내
저위치에 둔다. 앞·뒤 차축 사이 중앙에 가까워 회전에 의한 선가속도 영향을
줄이고 차체에 단단히 고정할 수 있는 초기 기준점이다. 현재 preset에서는
50 Hz, UDP Destination Port `9303`, 모든 IMU noise가 비활성화되어 있다.
GPS는 30 Hz, UDP Destination Port `9301`이며 noise가 비활성화되어 있다.

## 실행

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
roslaunch ioniq5_description description.launch
```

launch 인자는 다음과 같다.

| 인자 | 기본값 | 역할 |
| --- | --- | --- |
| `vehicle_config` | `config/vehicle_specs.yaml` | 차량 제원 YAML 선택 |
| `sensor_config` | `config/molit_2026_sensor_mounts.yaml` | 센서 장착 YAML 선택 |

이 패키지는 별도 센서 토픽을 발행하지 않는다. `robot_state_publisher`가
`/robot_description`을 읽고 `/tf_static`에 고정 센서 TF를 발행한다.

## 센서 장착값 변경 절차

1. MORAI Sensor Edit에서 장착 위치와 UDP Destination을 변경한다.
2. 실제 load preset인 `0725demo.json`을 갱신한다.
3. 같은 `xyz/rpy`를 `molit_2026_sensor_mounts.yaml`에 반영한다.
4. `transform_verified`를 실제 확인 결과에 맞게 설정한다.
5. RViz의 RobotModel/TF와 MORAI 외장 표시가 일치하는지 확인한다.

MORAI preset과 YAML 중 한쪽만 바꾸면 시뮬레이터 데이터와 ROS TF가 서로 다른
센서 위치를 나타내므로 반드시 함께 변경해야 한다.

## 확인 명령

```bash
rosparam get /robot_description
rosrun tf tf_echo base_link gps_link
rosrun tf tf_echo base_link imu_link
rosrun tf tf_echo base_link lidar_link
rosrun rviz rviz
```

GPS, IMU와 LiDAR의 현재 기대값은 각각 `(0, 0, 1.2)`,
`(1.5, 0, 0.5)`, `(1.5, 0, 1.25)`다.
