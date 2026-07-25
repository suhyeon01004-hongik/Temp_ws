# 2026 MORAI autonomous-driving workspace

원본 전달 파일 검토 결과와 규정 대응표는 [REVIEW.md](REVIEW.md)에 정리했다.

현재 workspace는 책임에 따라 다음 네 패키지로 분리한다.

```text
catkin_ws/src/
├── morai_udp_bridge/       # MORAI UDP 수신/패킷 검증/원시 ROS 센서 메시지
├── ioniq5_description/     # 규정 차량 스펙, 센서 장착 config, URDF/TF
├── morai_localization/     # GPS WGS84 → UTM → scenario-local 좌표 변환
└── morai_bringup/          # 위 패키지들의 launch 조합
```

데이터 흐름은 다음과 같다.

```text
MORAI UDP
   │
   ▼
morai_udp_bridge ──► /sensors/*  (raw ROS contract)
                            │
                            ├──► perception/localization algorithms
                            └──► morai_localization ──► /localization/*

ioniq5_description ──► robot_description + static vehicle/sensor TF
morai_bringup ────────► composition only
```

통신 패키지는 시나리오 map offset이나 차량 형상을 알지 않는다. description은 UDP
socket이나 센서 데이터를 처리하지 않으며 센서 모델/frame의 원본만 제공한다.
bringup이 이를 브리지에 주입하고, localization은 raw GPS topic만 구독한다.

## 빌드

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 현재 확정/미확정 경계

- 확정: 규정상 IONIQ 5 치수/조향 제원, 고정 카메라 3대 위치·자세·FOV
- 미확정: GPS/IMU/LiDAR 장착 위치, 차량-local 원점의 차체 기준 위치,
  최종 시나리오 UTM offset
- 외부 의존성 미설치: `ros-noetic-velodyne`

미확정 값은 0으로 가정해 TF를 발행하지 않으며, localization config도 검증 전에는
노드가 시작되지 않도록 구성했다.
