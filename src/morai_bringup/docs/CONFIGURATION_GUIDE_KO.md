# MORAI 센서 스택 설정 변경 가이드

이 문서는 `catkin_ws`의 MORAI 센서 설정을 안전하게 변경하는 방법을 설명한다.
처음 설정을 수정하는 사람도 따라 할 수 있도록 MORAI 설정과 ROS 설정의 관계,
수정 순서, 확인 명령을 함께 적었다.

## 1. 먼저 알아야 할 전체 구조

센서 하나의 설정은 한 파일에만 들어 있지 않다. MORAI와 ROS가 서로 다른 역할을
담당하기 때문에 같은 센서의 설정이 다음 위치에 나뉜다.

```text
MORAI Sensor preset JSON
  ├─ 센서 개수와 종류
  ├─ 센서 장착 위치와 회전
  ├─ 카메라 해상도와 FOV
  └─ UDP 목적지 IP와 포트
                │
                ├─ 장착값 ──> ioniq5_description/config/
                │              molit_2026_sensor_mounts.yaml
                │
                └─ 포트 ─────> morai_udp_bridge/config/
                               molit_2026.yaml

morai_bringup/launch/
  ├─ molit_2026_sensors.launch       : description, UDP bridge, Velodyne
  ├─ molit_2026_localization.launch  : GPS+IMU localization
  ├─ molit_2026_path_manager.launch  : global/local path
  └─ molit_2026_stack.launch         : 위 세 계층의 전체 조합
```

설정 변경 시 가장 중요한 원칙은 다음 두 가지다.

1. MORAI의 `destinationPort`와 ROS 수신 `port`는 반드시 같아야 한다.
2. MORAI의 센서 `pos/rot`와 ROS의 `xyz_m/rpy_deg`는 반드시 같아야 한다.

한쪽만 수정하면 패킷을 받지 못하거나 RViz의 센서 위치가 실제 시뮬레이터와
달라진다.

## 2. 현재 기준 설정

현재 기준 MORAI preset은 다음 파일이다.

```text
ioniq5_description/config/morai_presets/0725demo.json
```

현재 센서 구성은 다음과 같다.

| 센서 | 위치 `(x,y,z)` m | 회전 `(roll,pitch,yaw)` deg | UDP 목적지 포트 |
|---|---|---|---:|
| Front camera | `(1.9, 0.0, 1.2)` | `(0, 2, 0)` | 9291 |
| Left camera | `(1.15, 0.65, 1.2)` | `(0, 10, 70)` | 9293 |
| Right camera | `(1.15, -0.65, 1.2)` | `(0, 10, 290)` | 9295 |
| VLP-16 LiDAR | `(1.5, 0.0, 1.25)` | `(0, 0, 0)` | 2368 |
| GPS | `(0.0, 0.0, 1.2)` | `(0, 0, 0)` | 9301 |
| IMU | `(1.5, 0.0, 0.5)` | `(0, 0, 0)` | 9303 |

차량 로컬 좌표축은 다음과 같다.

```text
x: 차량 전방
y: 차량 좌측
z: 위쪽
```

각도 단위는 degree이며 YAML의 순서는 roll, pitch, yaw이다.

## 3. MORAI preset 수정 방법

### 3.1 기준 파일

워크스페이스에서 먼저 이 파일을 수정한다.

```text
~/catkin_ws/src/ioniq5_description/config/morai_presets/0725demo.json
```

다운로드 폴더의 원본을 직접 수정하지 않는다. 다운로드 파일은 전달받은 원본을
다시 확인할 수 있도록 보존한다.

### 3.2 UDP 통신 모드

Camera와 LiDAR의 통신 방식은 `commType`으로 지정한다. 이 워크스페이스는 직접
만든 UDP bridge와 공식 Velodyne driver를 사용하므로 다음 값을 사용한다.

```json
"commType": 1
```

ROS 직접 발행용 Topic 필드는 비운다.

```json
"Topic": ""
```

`commType`을 ROS 직접 통신 모드로 바꾸면 이 워크스페이스의 UDP bridge에 카메라
패킷이 도착하지 않을 수 있다.

### 3.3 UDP IP 설정

MORAI와 ROS가 같은 PC에서 실행되면 다음 값을 사용할 수 있다.

```json
"destinationIP": "127.0.0.1"
```

MORAI PC와 ROS PC가 다르면 `destinationIP`를 ROS PC의 IP로 바꾼다. ROS PC에서
다음 명령으로 IP를 확인할 수 있다.

```bash
hostname -I
```

예를 들어 ROS PC가 `192.168.0.20`이라면 다음과 같이 설정한다.

```json
"destinationIP": "192.168.0.20"
```

`hostIP`와 `hostPort`는 MORAI 측 송신 설정이다. ROS bridge가 실제로 수신하는
값은 `destinationIP`와 `destinationPort`다.

### 3.4 MORAI 설치 폴더로 동기화

워크스페이스 preset을 수정한 뒤 MORAI가 읽는 위치로 복사한다.

```bash
cp ~/catkin_ws/src/ioniq5_description/config/morai_presets/0725demo.json \
  ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/0725demo.json
```

복사가 정확했는지 확인한다.

```bash
sha256sum \
  ~/catkin_ws/src/ioniq5_description/config/morai_presets/0725demo.json \
  ~/MoraiLauncher_Lin/MoraiLauncher_Lin_Data/SaveFile/Sensor/25.S4.MolitComp03/0725demo.json
```

두 줄 앞의 해시가 같으면 동일한 파일이다. 이후 MORAI Sensor Edit에서
`0725demo.json`을 불러온다.

기존 `SensorInfo_2023_Hyundai_Ioniq5.json`을 무조건 덮어쓰지 않는다. MORAI가
자동 저장한 현재 설정일 수 있으므로 필요할 때 먼저 백업한다.

## 4. UDP 포트 변경 방법

MORAI preset과 ROS bridge를 함께 수정해야 한다.

### 4.1 MORAI에서 변경

`0725demo.json`에서 해당 센서의 값을 찾는다.

```json
"udpConfig": {
  "destinationPort": 9291
}
```

### 4.2 ROS bridge에서 변경

다음 파일을 수정한다.

```text
morai_udp_bridge/config/molit_2026.yaml
```

전방 카메라 예시는 다음과 같다.

```yaml
cameras:
  front:
    port: 9291
```

항상 다음 관계가 성립해야 한다.

```text
MORAI JSON destinationPort == ROS YAML port
```

현재 포트 계약은 다음과 같다.

| 센서 | MORAI destinationPort | ROS 설정 위치 |
|---|---:|---|
| Front camera | 9291 | `cameras.front.port` |
| Left camera | 9293 | `cameras.left.port` |
| Right camera | 9295 | `cameras.right.port` |
| GPS | 9301 | `gps.port` |
| IMU | 9303 | `imu.port` |
| LiDAR | 2368 | launch의 `lidar_port` |

LiDAR 포트는 다음 launch 파일에서 관리한다.

```text
morai_bringup/launch/molit_2026_sensors.launch
```

```xml
<arg name="lidar_port" default="2368"/>
```

파일을 수정하지 않고 실행할 때만 바꿀 수도 있다.

```bash
roslaunch morai_bringup molit_2026_sensors.launch lidar_port:=2368
```

## 5. ROS 토픽 이름 변경 방법

### 5.1 Camera, GPS, IMU

다음 파일을 수정한다.

```text
morai_udp_bridge/config/molit_2026.yaml
```

카메라 예시:

```yaml
cameras:
  front:
    topic: /image/front/compressed
    camera_info_topic: /image/front/camera_info
```

현재 주요 토픽은 다음과 같다.

| 토픽 | 메시지 타입 |
|---|---|
| `/image/front/compressed` | `sensor_msgs/CompressedImage` |
| `/image/left/compressed` | `sensor_msgs/CompressedImage` |
| `/image/right/compressed` | `sensor_msgs/CompressedImage` |
| `/sensors/gps/fix` | `sensor_msgs/NavSatFix` |
| `/sensors/imu/data` | `sensor_msgs/Imu` |

카메라 영상 토픽을 바꾸면 해당 CameraInfo 토픽도 같은 namespace로 바꾸는 편이
좋다.

### 5.2 LiDAR

LiDAR PointCloud 이름은 다음 launch 파일의 remap으로 정한다.

```text
morai_bringup/launch/molit_2026_sensors.launch
```

```xml
<remap from="velodyne_points" to="/lidar3D"/>
```

### 5.3 `/Ego_topic`

`/Ego_topic`은 현재 센서 UDP bridge가 만들지 않는다. MORAI 차량 상태 송신이나
별도 상태 bridge가 `morai_msgs/EgoVehicleStatus` 타입으로 제공해야 한다.
따라서 `/Ego_topic`이 없다고 Camera/LiDAR UDP 설정을 변경하면 안 된다.

## 6. 센서 장착 위치와 TF 변경 방법

ROS TF의 단일 원본은 다음 파일이다.

```text
ioniq5_description/config/molit_2026_sensor_mounts.yaml
```

MORAI JSON과 YAML의 대응 관계는 다음과 같다.

```text
JSON pos.x/y/z             -> YAML xyz_m
JSON rot.roll/pitch/yaw    -> YAML rpy_deg
```

LiDAR 예시:

```yaml
lidar:
  transform_verified: true
  frame_id: lidar_link
  xyz_m: [1.5, 0.0, 1.25]
  rpy_deg: [0.0, 0.0, 0.0]
```

장착값을 모르는 센서는 0점 TF를 임의로 만들지 않는다.

```yaml
transform_verified: false
```

실제 JSON 또는 Sensor Edit 화면에서 값을 확인한 뒤에만 값을 채우고 `true`로
바꾼다.

```yaml
transform_verified: true
```

카메라는 `frame_id`와 `optical_frame_id`를 모두 가진다. 이미지 메시지에는
optical frame이 들어가고 URDF가 camera link에서 optical frame으로의 표준 회전을
제공한다.

TF 확인:

```bash
rosrun tf tf_echo base_link lidar_link
```

RViz에서는 Fixed Frame을 `base_link`로 설정하고 RobotModel, TF, PointCloud2를
추가한다.

## 7. 카메라 해상도와 FOV 변경 방법

MORAI preset과 description YAML을 함께 수정한다.

MORAI JSON:

```json
"cameraResWidth": 1280,
"cameraResHeight": 720,
"cameraFOV": 90.0
```

description YAML:

```yaml
width: 1280
height: 720
horizontal_fov_deg: 90.0
```

bridge는 이 YAML을 사용해 `sensor_msgs/CameraInfo`의 pinhole intrinsic을 만든다.
두 파일의 해상도 또는 FOV가 다르면 이미지와 CameraInfo가 불일치한다.

현재 구현은 왜곡계수를 모두 0으로 발행한다. 실제 렌즈 왜곡 모델이 필요해지면
JSON 값만 바꾸는 것으로 끝나지 않고 CameraInfo 생성 코드도 수정해야 한다.

## 8. 센서 주기 변경 방법

MORAI JSON의 `sensorPeriod`는 초 단위다.

```text
0.05초 = 20 Hz
0.10초 = 10 Hz
약 0.0333333초 = 30 Hz
```

ROS bridge의 `max_hz`는 대회 규정 또는 운영상 허용할 최대 발행 주기다.

```yaml
stale_timeout: 0.5
max_hz: 30.0
```

- `stale_timeout`: 마지막 패킷 이후 몇 초가 지나면 stale로 판단할지 결정
- `max_hz`: 실제 발행률이 이 값보다 높으면 diagnostics 경고 발생

대회 규정상 최대 Data Rate/Frame Rate는 다음과 같다.

| 센서 | 규정상 최대 | 현재 활성 preset |
|---|---:|---:|
| Camera | 30 Hz | 30 Hz (`sensorPeriod` 약 0.0333333초) |
| GPS | 30 Hz | 30 Hz (`sensorPeriod` 약 0.0333333초) |
| IMU | 50 Hz | 50 Hz (`sensorPeriod` 약 0.02초, noise off) |
| VLP-16 LiDAR | 15 Hz | 15 Hz (`sensorPeriod` 약 0.0666667초) |

규정은 LiDAR에 대해 10 Hz 이하를 권장하지만 최대 허용값은 15 Hz다. 현재는 최대
주기 시험을 위해 15 Hz로 설정했다. 대역폭이나 packet loss가 발생하면 10 Hz로
낮춰 비교한다.

`sensorPeriod`를 늘려 센서가 느려졌다면 `stale_timeout`도 검토한다. 예를 들어
1 Hz 센서에 stale timeout 0.5초를 사용하면 정상 동작 중에도 stale 경고가 반복될
수 있다.

## 9. LiDAR 설정 변경 방법

LiDAR 기본 실행값은 다음 파일에 있다.

```text
morai_bringup/launch/molit_2026_sensors.launch
```

```xml
<arg name="use_lidar" default="true"/>
<arg name="lidar_device_ip" default=""/>
<arg name="lidar_port" default="2368"/>
<arg name="lidar_hz" default="15.0"/>
<arg name="lidar_frame" default="lidar_link"/>
<arg name="lidar_cut_angle" default="0.0"/>
<arg name="lidar_fixed_frame" default=""/>
<arg name="lidar_target_frame" default=""/>
```

- `use_lidar`: Velodyne driver 실행 여부
- `lidar_device_ip`: 특정 MORAI 송신 IP만 허용할 때 지정
- `lidar_port`: MORAI LiDAR destination port
- `lidar_hz`: LiDAR 회전 주기
- `lidar_frame`: PointCloud2의 frame ID
- `lidar_cut_angle`: PointCloud 한 회전을 끝내는 고정 방위(rad)
- `lidar_fixed_frame`: 패킷별 주행 왜곡 보정의 고정 frame
- `lidar_target_frame`: 보정된 PointCloud의 출력 frame

실행할 때 임시 변경:

```bash
roslaunch morai_bringup molit_2026_sensors.launch \
  lidar_device_ip:=192.168.0.10 \
  lidar_port:=2368 \
  lidar_hz:=15.0
```

기본 `lidar_cut_angle:=0.0`은 PointCloud 경계를 매번 같은 방위에 고정한다.
센서 단독 launch는 localization에 의존하지 않도록 `lidar_fixed_frame`과
`lidar_target_frame`을 비워 둔다. 전체 stack도 두 값이 빈 문자열인 안전한
기본값을 사용한다.

현재 설치된 Velodyne transform nodelet은 시작 시
`map -> base_footprint -> base_link -> lidar_link` 연결이 아직 만들어지지
않았을 때 `tf2::ConnectivityException`으로 manager 전체를 종료할 수 있다.
따라서 `lidar_fixed_frame:=map`, `lidar_target_frame:=lidar_link`를 launch
시작과 동시에 지정하지 않는다. TF 연결을 확인한 뒤 transform nodelet을
활성화하는 준비 절차가 구현되기 전까지 두 값은 비워 둔다.

LiDAR 없이 실행:

```bash
roslaunch morai_bringup molit_2026_sensors.launch use_lidar:=false
```

`use_lidar: true`와 `lidar.transform_verified: true`는 의미가 다르다.

- `use_lidar`: UDP packet을 받아 PointCloud를 만들 것인지 결정
- `transform_verified`: `base_link -> lidar_link` TF를 발행할지 결정

## 10. GPS 지도 설정 변경 방법

파일:

```text
morai_localization/config/molit_2026_kcity.yaml
```

현재 설정:

```yaml
config_verified: true
map_id: R-KR_PG_K-City_2025
crs: EPSG:32652
utm_zone: 52
utm_northern: true
east_offset: 302595.0
north_offset: 4124145.0
```

변환식:

```text
local x = UTM easting  - east_offset
local y = UTM northing - north_offset
```

새 맵을 받으면 맵 원본을 다음 구조로 보존한다.

```text
morai_path_manager/map/새_맵_ID/
├─ global_info.json
├─ node_set.json
├─ link_set.json
└─ global_path.txt
```

`global_info.json`에서 다음 값을 찾는다.

```json
"local_origin_in_global": [EAST, NORTH, Z]
```

다음처럼 옮긴다.

```text
EAST  -> east_offset
NORTH -> north_offset
```

확인 전에는 반드시 다음 상태를 유지한다.

```yaml
config_verified: false
```

좌표계와 원점을 공식 자료로 확인한 뒤에만 `true`로 바꾼다. 현재 대회 설정은
검증 완료 상태다. localization만 확인할 때는 전용 bringup을 실행한다.

```bash
roslaunch morai_bringup molit_2026_localization.launch
```

`config_verified`는 설정을 신뢰할 수 있는지 나타낸다. 센서만 확인할 때는
`molit_2026_sensors.launch`, 전체 계층은 `molit_2026_stack.launch`를 사용한다.

## 11. 차량 제원 변경 방법

파일:

```text
ioniq5_description/config/vehicle_specs.yaml
```

```yaml
dimensions:
  length_m: 4.635
  width_m: 1.892
  height_m: 2.434
  wheelbase_m: 3.000

visualization:
  body_height_m: 1.605
  rear_axle_height_m: 0.370
  wheel_radius_m: 0.370

steering:
  minimum_turning_radius_m: 5.87
  maximum_wheel_angle_deg: 40.0
```

차량 모델이 바뀔 때 수정한다. `dimensions`는 차량 제원과 향후 제어 geometry,
`visualization`은 RViz 좌표 확인용 단순 형상의 기준이다. 현재 `base_link`는
MORAI IONIQ 5의 rear axle 중앙 원점이고 `base_footprint`는 그 원점의 지면
투영점이다. 두 frame의 z 간격은 `rear_axle_height_m`이다. 차종이나 MORAI 차량
모델을 바꾸면 같은 원점 계약이 유지되는지 다시 확인한다.

## 12. Competition Status와 자율 제어 설정

Competition Vehicle Status는 Sensor preset이 아니라 MORAI Network Settings에서
추가한다.

```text
Publisher: CompetitioninfoPublisher / Competition Vehicle Status
Destination IP: ROS PC 주소
Destination Port: 9094
```

ROS 수신 설정은 다음 파일이다.

```text
morai_udp_bridge/config/molit_2026_vehicle_status.yaml
```

주요 변경값은 `listen_port`, `status_topic`, `stale_timeout_sec`,
`allowed_source_ip`다. 제어기 PID feedback은
`/vehicle/competition_status.velocity_x_mps`만 사용한다.

제어기 설정은 다음 파일에서 바꾼다.

```text
morai_path_tracking/config/molit_2026_path_tracking.yaml
```

| 목적 | 파라미터 |
| --- | --- |
| 횡제어기 선택 | `lateral_controller`: `pure_pursuit`, `stanley`, `hybrid` |
| Stanley 튜닝 | `stanley_gain`, `stanley_softening_speed_mps`, `stanley_heading_error_gain`, 곡률 feedforward/yaw-rate damping/조향 변화율 |
| Hybrid 튜닝 | IMM 확률·전이, PP 횡오차 보정, candidate conflict 및 `hybrid_cross_track_recovery_full_scale_m` |
| 직선 최고 목표 속도 | `target_speed_kph` (km/h) |
| 곡선 속도 제한 | `minimum_curve_speed_kph`, `maximum_lateral_acceleration_mps2`, `curvature_speed_reduction_gain_m` |
| 곡률 추정 | `curvature_preview_distance_m`, `curvature_sample_spacing_m`, `curvature_epsilon_m_inv` |
| 커브 접근 감속 | `curve_approach_deceleration_mps2` |
| 목표 속도 변화율 | `target_speed_acceleration_limit_mps2`, `curve_target_speed_acceleration_limit_mps2`, `target_speed_deceleration_limit_mps2` |
| PID 튜닝 | `speed_kp`, `speed_ki`, `speed_kd`, `speed_accel_feedforward_gain_per_mps`, 적분/deadband/output limit |
| 타력·제동 경계 | `speed_coast_overspeed_kph`, `speed_brake_overspeed_kph`, `hard_brake_activation_speed_kph`, `minimum_hard_brake_command` |
| 속도 필터 | `speed_filter_time_constant_sec`, 기본 `0.0` |
| LD 조정 | `lookahead_base_m`, `lookahead_speed_gain_sec`, `lookahead_curvature_gain_m`, `lookahead_curvature_preview_distance_m`, min/max |
| 입력 freshness | path/odometry/status timeout |
| 안전 제동 | `safe_brake_command` |
| 출력 관측 | `controller_status_topic`, `lookahead_point_topic` |

Config의 목표·최소 곡선 속도만 km/h를 사용한다. 제어기 내부와
`ControllerStatus`의 `configured_target_speed_mps`,
`curvature_speed_limit_mps`, `target_speed_mps`는 m/s다. 전방 preview
경로를 2 m 간격으로 재샘플링하고 세 점 곡률과 커브까지 거리를 계산한다.
커브 지점 속도는 `v_lateral =
sqrt(maximum_lateral_acceleration_mps2 / curvature)`를 먼저 구한 뒤
`v_lateral / (1 + curvature_speed_reduction_gain_m * curvature)`로
연속 보정한다. 현재 허용속도는
`sqrt(curve_speed^2 + 2 * curve_approach_deceleration_mps2 * distance)`다.
LD는 별도의 8 m 근거리 곡률만 사용한다.

기본 `lateral_controller`는 `hybrid`다. standalone 비교 시
`pure_pursuit` 또는 `stanley`로 바꾸고 노드를 재시작한다. Stanley 기본
gain `2.0`은 58 km/h에서 횡오차 1 m일 때 약 6.3 deg의 보정 조향을 만든다.
softening/minimum speed는 각각 2.0/1.0 m/s이고 heading gain은 0.6,
조향 변화율은 60 deg/s다. 30 Hz에서는 조향이 주기당 최대 2 deg 변한다.
Hybrid는 PP 기하 조향과 Stanley 횡오차 복귀를 IMM 확률로 혼합한다.
두 후보 부호가 급커브에서 충돌할 때 PP 우선 guard를 적용하고, PP가 올바른
CTE 복귀 방향에서 Stanley보다 강한 경우 CTE 0.25–0.50 m 사이에서 PP
비중을 연속적으로 높인다.

현재 기본값은 직선 `58 km/h`, 곡선 최저 `12 km/h`, 허용 횡가속도
`1.8 m/s²`, 추가 감속 gain `5.0 m`다. 반경 50/25/15/10 m 곡선의 제한
속도는 각각 약 31.0/20.1/14.0/12.0 km/h다. 속도 곡률 preview는 45 m이고
path manager는 거리 기준 100 m local path를 제공한다. LD는 4–16 m 범위이며
58 km/h 직선에서는 상한 16 m다.

목표속도 상승/하강률은 2.0/5.0 m/s²이며, 전방 곡률이 속도를 제한하는 동안
재가속은 0.2 m/s²로 제한한다. 첫 고속 PID 값은
`kp=0.18`, `ki=0.02`, `kd=0.0`, 적분 한계 `1.0`, deadband `0.10 m/s`다.
Competition Status가 제어 주기와 다른 간격으로 들어올 때 발생하는 D항
펄스를 피하기 위해 기본 D항은 끈다. 목표보다 deadband 이상 빠르면
feedforward를 차단하고 과속 중 accel 출력을 금지한다.
목표속도 비례 accel feedforward gain은 `0.008`이며 58 km/h에서 약 0.129를
더한다. accel/brake 상한은 0.40/0.60을 유지한다. 목표보다 0.2 km/h
초과하면 먼저 타력주행하고, 1.8 km/h 이상 초과할 때 정상 PID 제동을
허용한다. 실차 속도가 59 km/h에 도달하면 목표속도와 무관하게 최소 0.25의
독립 안전제동을 즉시 적용한다.

커브 접근 감속 능력 가정은 기본 `2.0 m/s²`다. 이 값은 brake 크기가 아니며
감속이 이르면 올리고 늦으면 내린다. 낮출수록 감속 시작이 앞당겨진다.
완만한 커브가 너무 느리면 `maximum_lateral_acceleration_mps2`를 올린다.
완만한 커브는 유지하면서 고곡률에서 더 줄이려면
`curvature_speed_reduction_gain_m`을 올리고, 과하면 내린다. `0.0`이면
추가 감속 보정을 사용하지 않는다.
직선에서 목표속도보다 낮게 정착하면 feedforward gain을 올리고 오버슈트가
크면 내린다. 상태 토픽의
`speed_limiting_curve_distance_m`, `preview_curvature_m_inv`,
`lookahead_curvature_m_inv`로 각각 감속 거리, 속도 곡률, LD 곡률을 확인한다.
`speed_overshoot_mps`와 `longitudinal_state`로 타력/제동 상태를 확인한다.
Stanley/Hybrid 모드에서는 `lateral_controller`, `cross_track_error_m`,
`heading_error_rad`와 hybrid 확률·effective weight·conflict/recovery 필드도
함께 확인한다. Stanley에는 Pure Pursuit식 LD가 없다.
대신 `/control/lookahead_point`에는 전륜 중심에서 경로로 내린 최근접
투영점을 발행하며 RViz에는 연결선 없이 연두색 점 하나로 표시한다.
`ControllerStatus.lookahead_distance_m`은 Stanley 모드에서 0이다.

UDP 송신과 optional gear 변경은
`morai_udp_bridge/config/molit_2026_control.yaml`에서 설정한다. 대회 서버가
기어를 관리하므로 `gear_command_enabled` 기본값은 `false`다. 활성화할 경우
정지 속도, Status timeout, accel/brake interlock 파라미터를 함께 검토한다.

추종점 시각화 크기와 lifetime은
`morai_visualization/config/path_visualizer.yaml`의
`lookahead_point_diameter`, `lookahead_marker_lifetime_sec`에서 바꾼다.
연결선은 표시하지 않는다.

## 13. 실행 방법

새 터미널마다 환경을 불러온다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

센서와 UDP bridge만 실행:

```bash
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

자율 경로 추종 전체 기본 실행:

```bash
roslaunch morai_bringup molit_2026_autonomous.launch
```

## 14. 설정 변경 후 확인 순서

### 14.1 빌드

YAML과 launch 값만 바꾼 경우 launch 재시작만으로 반영되는 경우가 많다. 하지만
패키지 구조, CMake, C++ 코드를 수정했거나 확실히 검증하려면 다시 빌드한다.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 14.2 테스트

```bash
cd ~/catkin_ws
catkin_make run_tests
catkin_test_results build/test_results --verbose
```

### 14.3 토픽 확인

```bash
rostopic list
rostopic hz /image/front/compressed
rostopic hz /image/left/compressed
rostopic hz /image/right/compressed
rostopic hz /lidar3D
rostopic hz /sensors/gps/fix
rostopic hz /sensors/imu/data
rostopic echo /vehicle/competition_status
rostopic echo /control/controller_status
```

GPS local projection을 활성화했다면:

```bash
rostopic echo /localization/gps/local_point
rostopic echo /localization/imu/data
rostopic echo /localization/pose
rostopic echo /localization/odometry
rosrun tf tf_echo map base_link
```

### 14.4 Diagnostics 확인

```bash
rostopic echo /diagnostics
```

주요 메시지:

- `waiting for UDP packets`: 노드는 실행됐지만 패킷이 도착하지 않음
- `UDP stream is stale`: 전에는 받았지만 현재 패킷이 끊김
- `kernel receive buffer is smaller than requested`: OS UDP 버퍼가 설정값보다 작음
- `publish rate exceeds configured competition limit`: 설정한 최대 주기 초과

`kernel receive buffer is smaller than requested`가 나오면 먼저 현재 상한을 확인한다.

```bash
sysctl net.core.rmem_max
```

이 워크스페이스는 각 센서 socket에 4 MiB(`4194304`)를 요청한다. Ubuntu 기본값인
`212992`는 Camera UDP burst에 너무 작을 수 있다. 패키지에 제공된 운영 설정을
다음과 같이 설치한다.

```bash
sudo install -m 0644 \
  "$(rospack find morai_bringup)/config/99-morai-udp.conf" \
  /etc/sysctl.d/99-morai-udp.conf
sudo sysctl --system
sysctl net.core.rmem_max
```

마지막 값이 `8388608`인지 확인한 뒤 bringup을 재시작한다. 이 변경은 시스템 전체의
UDP receive-buffer 최대 허용값만 높이며, 각 socket은 여전히 bridge 설정의
`receive_buffer_bytes`만큼만 요청한다.

### 14.5 영상과 PointCloud 확인

```bash
rqt_image_view
rviz
```

RViz 권장 설정:

```text
Fixed Frame: base_link
RobotModel: 추가
TF: 추가
PointCloud2 Topic: /lidar3D
```

경로·차량·LiDAR를 함께 보려면 다음 전용 profile을 사용한다.

```bash
roslaunch morai_bringup path_lidar.launch
```

이 profile은 기존 path 완전 탑뷰와 같은 화면에 PointCloud2만 추가한다.
좌표를 `map`에 고정하고 카메라 Target을 `base_link`로 두므로 차량을 중심에
유지한 채 전역/local 경로와 LiDAR를 함께 표시한다. PointCloud2의
`Decay Time=0.0`은 최신 cloud를 다음 입력까지 유지해 낮거나 변동하는 발행
주기에서도 빈 화면 구간이 생기지 않도록 한다.

## 15. 문제가 생겼을 때 확인할 것

### Camera가 나오지 않음

1. MORAI preset의 `commType`이 1인지 확인
2. MORAI `destinationIP`가 ROS PC인지 확인
3. JSON `destinationPort`와 YAML `port`가 같은지 확인
4. MORAI에서 수정된 `0725demo.json`을 실제로 불러왔는지 확인
5. `/diagnostics`의 수신 datagram 수 확인

### LiDAR가 나오지 않음

1. `use_lidar`가 true인지 확인
2. Velodyne 패키지 확인

   ```bash
   rospack find velodyne_driver
   rospack find velodyne_pointcloud
   ```

3. MORAI destination port와 `lidar_port` 확인
4. 별도 PC라면 `lidar_device_ip`와 방화벽 확인
5. `/sensors/lidar/packets`와 `/lidar3D`를 각각 확인

Velodyne 시작 로그의 다음 경고는 설치된 `velodyne_pointcloud 1.7.0`이 VLS-128이
아닌 모델에 공통으로 출력하는 메시지다.

```text
No Azimuth Cache configured for model VLP16
```

VLP-16 자체의 calibration 실패를 뜻하지 않는다. 함께 `Number of lasers: 16`이
출력되고 `/sensors/lidar/packets`, `/lidar3D`가 정상 발행되면 이 경고만으로 설정을
바꾸거나 `/opt/ros`의 드라이버를 수정하지 않는다.

### 토픽은 있지만 TF 오류가 발생함

1. 메시지의 `frame_id` 확인
2. `molit_2026_sensor_mounts.yaml`의 `frame_id` 확인
3. `transform_verified` 확인
4. MORAI JSON과 YAML의 장착값 비교

### 최종 localization pose가 나오지 않음

1. `/sensors/gps/fix`가 발행되는지 확인
2. `/sensors/imu/data`와 `/localization/imu/data`가 발행되는지 확인
3. GPS status가 `NO_FIX`인지 확인
4. `rosnode list`에서 `/gps_utm_projector`, `/imu_orientation_adapter`,
   `/localization_fusion` 노드가 실행 중인지 확인
5. `config_verified: true`인지 확인
6. GPS/IMU frame과 timestamp 차이가 localization config 허용값 이내인지 확인

### Competition Status가 나오지 않음

1. MORAI Network Settings에 Competition Vehicle Status를 추가했는지 확인
2. Destination IP와 `listen_port: 9094`가 일치하는지 확인
3. `ss -lunp | grep :9094`로 receiver bind 확인
4. `/diagnostics`의 `competition_vehicle_status` parse error 확인
5. `vehicle_control` status receiver를 동시에 실행하지 않았는지 확인

## 16. 변경 전 체크리스트

- 전달받은 원본 파일을 별도 보존했는가?
- MORAI JSON과 ROS YAML을 함께 수정했는가?
- UDP destination IP가 ROS PC를 가리키는가?
- MORAI destination port와 ROS 수신 port가 같은가?
- Competition Vehicle Status `9094`가 연결되어 있는가?
- 센서 장착 위치와 TF 값이 같은가?
- 확인되지 않은 TF를 `true`로 바꾸지 않았는가?
- 새 맵의 CRS와 원점을 공식 `global_info.json`으로 확인했는가?
- 변경 후 launch를 재시작했는가?
- `/diagnostics`, 토픽 주기, RViz를 확인했는가?

이 체크리스트를 통과하면 MORAI 설정, UDP 수신, ROS 토픽, TF, 지도 좌표 사이의
불일치로 발생하는 대부분의 문제를 예방할 수 있다.
