# 차량 제어

`vehicle_control`은 Joytron CYVOX MX로 MORAI 차량을 조작하는 독립 ROS1
패키지다. 기어 변경 판단에 필요한 속도도 MORAI UDP에서 직접 받으므로
localization이나 다른 팀 패키지가 없어도 동작한다.

```text
CYVOX MX → /joy → joystick_teleop_node → /vehicle/manual_command
                                                   ↓
                                      morai_udp_sender_node → MORAI:9093

MORAI → UDP:9094 → morai_vehicle_status_udp_node → /vehicle/status
                                                        ↓
                                             저속 기어 변경 판단

Home 버튼 → /vehicle/reset_request → morai_sim_reset_node
                                      → Simulator 창에 i 입력
```

## 준비

```bash
sudo apt install ros-noetic-joy xdotool
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make install
source ~/catkin_ws/install/setup.bash
```

기본 장치 경로는 아래와 같다.

```text
/dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick
```

다른 CYVOX를 연결하면 `config/cyvox_mx.yaml`의 `joy_node/dev`를 실제
`/dev/input/by-id/*-joystick` 경로로 바꾼다.

## MORAI 설정

1. Driving Info의 Ego Controller를 `AV-ExternalCtrl`로 선택한다.
2. `Edit → Network Settings → Ego Network → Cmd Control`을 UDP로 설정한다.
3. 같은 PC에서는 Host IP와 Destination IP를 `127.0.0.1`로 설정한다.
4. Host Port는 `9093`, Destination Port는 `9094`로 설정하고 Connect한다.

- `9093`: MORAI가 조이스틱 차량 명령을 받는 포트
- `9094`: 이 패키지가 MORAI Ego Vehicle Status를 받는 포트

다른 PC의 MORAI를 제어할 때는 `destination_ip`를 MORAI PC 주소로 바꾸고,
MORAI의 Destination IP는 ROS PC 주소로 지정한다.

## 실행

```bash
roslaunch vehicle_control cyvox_morai.launch
```

기본 조작은 다음과 같다.

| 입력 | 동작 |
| --- | --- |
| 왼쪽 스틱 좌우 | 조향 |
| RT | 가속 |
| LT | 브레이크 |
| RT·LT를 모두 놓음 | 가속·브레이크 0, 타력 주행 |
| A | D(주행) |
| B | N(중립) |
| X | R(후진) |
| Y | P(주차) |
| Home/Guide | MORAI 차량 초기 위치로 재설정 |

기어는 MORAI 속도가 `0.5 m/s` 이하일 때만 바뀐다. 상태 UDP가 없거나 0.5초
이상 끊기면 안전을 위해 변경을 거부한다. 주행 중 눌렀던 버튼은 정차 후 다시
눌러야 한다.

조이스틱 명령이 0.25초 이상 끊기면 가속 0, 조향 0, 브레이크 0.5를 전송하며
마지막 기어는 유지한다.

## 확인

```bash
rostopic hz /joy
rostopic echo /vehicle/status
rostopic echo /vehicle/manual_command
```

Home 버튼 초기화가 안 되면 다음을 확인한다.

```bash
which xdotool
echo "$XDG_SESSION_TYPE"
xdotool search --onlyvisible --name Simulator
```

초기화 기능은 X11의 MORAI `Simulator` 창에 키보드 `i`를 보내는 방식이다.
`xdotool`이 없거나 Wayland 세션이면 차량 조작 노드는 계속 동작하지만 초기화만
실패한다.

## 주요 토픽

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/joy` | `sensor_msgs/Joy` | 조이스틱 원시 입력 |
| `/vehicle/status` | `vehicle_control/VehicleStatus` | MORAI 기어·속도 |
| `/vehicle/manual_command` | `vehicle_control/VehicleCommand` | 차량 제어 명령 |
| `/vehicle/reset_request` | `std_msgs/Empty` | MORAI 초기화 요청 |

## 주요 파라미터

설정 파일은 [`config/cyvox_mx.yaml`](config/cyvox_mx.yaml)이다.

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `steering_axis` | `0` | 왼쪽 스틱 X축 |
| `brake_axis` / `accel_axis` | `2` / `5` | LT / RT축 |
| `drive_button` | `0` | D 선택(A) |
| `neutral_button` | `1` | N 선택(B) |
| `reverse_button` | `2` | R 선택(X) |
| `park_button` | `3` | P 선택(Y) |
| `reset_button` | `8` | 초기화(Home/Guide) |
| `maximum_gear_change_speed_mps` | `0.5` | 기어 변경 허용 속도 |
| `status_timeout` | `0.5` | MORAI 상태 유효 시간 |
| `destination_port` | `9093` | MORAI 명령 수신 포트 |
| `listen_port` | `9094` | MORAI 상태 수신 포트 |
| `command_timeout` | `0.25` | 조이스틱 연결 끊김 판단 |
| `safe_brake` | `0.5` | 연결 끊김 시 브레이크 |
| `window_name` / `reset_key` | `Simulator` / `i` | 초기화 대상 창과 키 |

## 코드 책임

```text
joy_mapper.*                    축 입력 변환
gear_selector.*                 저속 P/R/N/D 선택
button_edge.*                   Home 버튼 1회 입력 판별
command_watchdog.*              끊긴 명령을 안전 명령으로 교체
morai_ctrl_packet.*             차량 명령 UDP 직렬화
morai_vehicle_status_packet.*   MORAI 상태 UDP 역직렬화
udp_sender.* / udp_receiver.*   UDP 송수신
reset_command.*                 xdotool 안전 실행
joystick_teleop_node.cpp        Joy·상태를 차량 명령으로 변환
morai_udp_sender_node.cpp       차량 명령을 MORAI로 송신
morai_vehicle_status_udp_node.cpp MORAI 상태를 ROS로 발행
morai_sim_reset_node.cpp        Home 요청을 MORAI i 키로 전달
```
