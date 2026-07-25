# 차량 제어

`vehicle_control`은 Joytron CYVOX MX 입력을 차량 공통 명령으로 바꾸고 MORAI
`Ego Ctrl Cmd` UDP 패킷으로 전송한다. 조이스틱 매핑과 MORAI 통신을 분리했기
때문에 이후 자율주행 제어기도 차량 명령 인터페이스에 연결할 수 있다.

```text
CYVOX MX → /joy ───────────────┐
                               ├→ joystick_teleop_node
/localization/odometry ────────┘   → /vehicle/manual_command
                                   → morai_udp_sender_node → MORAI UDP
```

## 준비

ROS 조이스틱 드라이버를 설치하고 워크스페이스를 빌드한다.

```bash
sudo apt install ros-noetic-joy
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make install
source ~/catkin_ws/install/setup.bash
```

현재 CYVOX는 Linux에서 `Microsoft X-Box 360 pad`로 인식되며 다음 장치 경로를
사용한다.

```text
/dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick
```

다른 CYVOX를 사용하면 `config/cyvox_mx.yaml`의 `joy_node/dev`를 해당 장치의
`/dev/input/by-id/*-joystick` 경로로 바꾼다.

## MORAI 설정

1. Driving Info의 Ego Controller를 `AV-ExternalCtrl`로 선택한다.
2. `Edit → Network Settings → Ego Network → Cmd Control`을 UDP로 설정한다.
3. 같은 PC라면 IP를 `127.0.0.1`, MORAI가 명령을 받는 port를 `9093`으로 맞춘다.
4. 다른 PC라면 YAML의 `destination_ip`를 MORAI PC 주소로 바꾼다.

MORAI의 Cmd Control 수신 port와 YAML의 `destination_port`는 반드시 같아야
한다.

## 실행과 확인

```bash
roslaunch vehicle_control cyvox_morai.launch
```

다른 터미널에서 입력과 변환 결과를 확인한다.

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/install/setup.bash
rostopic echo /joy
rostopic echo /vehicle/manual_command
```

기본 조작은 다음과 같다.

| 입력 | 동작 |
| --- | --- |
| 왼쪽 스틱 좌우 | 조향 |
| RT | 가속 |
| LT | 브레이크 |
| RT와 LT를 모두 놓음 | 가속·브레이크 0, 타력 주행 |
| A | D(주행) |
| B | N(중립) |
| X | R(후진) |
| Y | P(주차) |

기어는 버튼을 누른 순간 선택되고 다음 선택까지 유지된다. 기본 설정에서는
`/localization/odometry` 속도가 `0.5 m/s` 이하일 때만 변경된다. 속도 토픽이
없거나 0.5초 이상 갱신되지 않았으면 변경을 거부한다. 주행 중 누른 버튼을 계속
누르고 있어도 감속 후 뒤늦게 변경되지 않으므로, 정차 후 버튼을 다시 눌러야 한다.

조이스틱 명령이 0.25초 이상 끊기면 마지막 가속을 유지하지 않고 가속 0, 조향 0,
브레이크 0.5를 전송한다. 이때 마지막으로 선택한 기어는 유지한다.

## 주요 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | Linux 조이스틱 원시 축·버튼 |
| `/localization/odometry` | `nav_msgs/Odometry` | 기어 변경 판단용 차량 속도 |
| `/vehicle/manual_command` | `vehicle_control/VehicleCommand` | 정규화된 차량 명령 |

## 설정

주요 값은 [`config/cyvox_mx.yaml`](config/cyvox_mx.yaml)에서 수정한다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `steering_axis` | `0` | 왼쪽 스틱 X축 |
| `brake_axis` | `2` | LT축 |
| `accel_axis` | `5` | RT축 |
| `default_trig_val` | `true` | 시작할 때 LT·RT의 실제 초기값 사용 |
| `brake_inverted` | `true` | CYVOX LT의 `+1 → -1` 방향을 브레이크 `0 → 1`로 변환 |
| `accel_inverted` | `true` | CYVOX RT의 `+1 → -1` 방향을 가속 `0 → 1`로 변환 |
| `steering_inverted` | `false` | 좌우 방향 반전 |
| `steering_deadzone` | `0.05` | 중앙 유격 제거 |
| `drive_button` | `0` | D 선택(A) |
| `neutral_button` | `1` | N 선택(B) |
| `reverse_button` | `2` | R 선택(X) |
| `park_button` | `3` | P 선택(Y) |
| `initial_gear` | `4` | 시작 기어(D) |
| `maximum_gear_change_speed_mps` | `0.5` | 기어 변경 허용 최고 속도 |
| `odometry_topic` | `/localization/odometry` | 속도 입력 토픽 |
| `odometry_timeout` | `0.5` | 속도 입력 유효 시간 |
| `destination_ip` | `127.0.0.1` | MORAI PC 주소 |
| `destination_port` | `9093` | MORAI Cmd Control 수신 port |
| `send_rate` | `50.0` | UDP 명령 송신 주기 |
| `command_timeout` | `0.25` | 연결 끊김 판단 시간 |
| `safe_brake` | `0.5` | 연결 끊김 시 브레이크 |

차량이 반대 방향으로 조향되면 `steering_inverted`만 `true`로 바꾼다. 평상시
감속감을 바꾸기 위해 `safe_brake`를 수정하면 안 된다. 이 값은 조이스틱 연결이
끊겼을 때만 사용한다.

## 파일 책임

```text
joy_mapper.*             CYVOX 축을 차량 명령으로 변환
gear_selector.*           버튼·속도로 P/R/N/D 선택
command_watchdog.*       오래된 명령을 안전 명령으로 교체
morai_ctrl_packet.*      대회용 MORAI 55바이트 패킷 직렬화
udp_sender.*             UDP datagram 송신
joystick_teleop_node.cpp ROS Joy 입출력
morai_udp_sender_node.cpp ROS 명령과 UDP 연결
```

매핑·안전 정책·패킷·UDP 변경 시 해당 gtest를 함께 수정하고 실행한다.
