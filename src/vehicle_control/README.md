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
                                      → q → i → q → q 입력
          └→ morai_udp_sender_node가 초기화 중 제어 송신 일시 중지
```

## 준비

```bash
sudo apt install ros-noetic-joy xdotool
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make install
source ~/catkin_ws/install/setup.bash
```

한 PC에 게임패드 한 대를 연결하는 환경을 기준으로 기본 장치 경로는 아래와
같다.

```text
/dev/input/js0
```

따라서 같은 모델의 다른 CYVOX를 연결해도 장치 시리얼과 관계없이 인식한다.
게임패드를 여러 대 연결하면 `/dev/input/js0`, `/dev/input/js1` 순서가 달라질
수 있으므로 그때만 `config/cyvox_mx.yaml`의 `joy_node/dev`를 원하는 장치로
바꾼다.

## MORAI 설정

1. Driving Info의 Ego Controller를 `AV-ExternalCtrl`로 선택한다.
2. `Edit → Network Settings → Ego Network → Cmd Control`을 UDP로 설정한다.
   - Host IP / Destination IP: `127.0.0.1`
   - Host Port: `9093`
   - Destination Port: `9094`
3. `Publisher, Subscriber, Service`에서 `MoraiInfoPublisher`
   (`Ego Vehicle Status`)를 UDP로 설정한다.
   - Host IP / Destination IP: `127.0.0.1`
   - Host Port: `9097`
   - Destination Port: `9094`
   - Frequency: `50 Hz`
4. 두 항목을 Connect한다.

- `9093`: MORAI가 조이스틱 차량 명령을 받는 포트
- `9097`: MORAI 상태 Publisher가 사용하는 로컬 송신 포트
- `9094`: 이 패키지가 MORAI 차량 상태를 받는 포트

현재 사용 중인 MORAI 네트워크 저장 파일도 위 포트로 설정했다. 기존에
시뮬레이터를 실행 중이었다면 해당 프리셋을 다시 Load하거나 시뮬레이터를
재시작해야 변경된 포트가 적용된다.

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

기어는 RT를 놓고 LT 브레이크를 50% 이상 밟은 상태에서만 바뀐다. 속도 정보가
있으면 실제 속도가 `0.5 m/s` 이하인지도 함께 확인한다. `/vehicle/status`와
`/localization/odometry`가 모두 없거나 끊겨도 브레이크 인터록으로 기어를
변경할 수 있다. 거부된 버튼은 놓았다가 다시 눌러야 한다.

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

초기화 기능은 X11의 MORAI `Simulator` 창을 잠시 `Manual-Keyboard`로 바꾼 뒤
`i`를 누르고 다시 `AV-ExternalCtrl`로 복귀시키는 방식이다. 이 과정에서 외부
제어 UDP는 0.9초 동안 중지된다. `xdotool`이 없거나 Wayland 세션이면 차량 조작
노드는 계속 동작하지만 초기화만 실패한다.

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
| `steering_deadzone` | `0.05` | 중앙 조향 무시 구간 |
| `steering_scale` | `1.0` | 최대 조향 출력 비율 |
| `steering_expo` | `0.7` | 중앙 조향을 완만하게 만드는 곡선 강도 |
| `drive_button` | `0` | D 선택(A) |
| `neutral_button` | `1` | N 선택(B) |
| `reverse_button` | `2` | R 선택(X) |
| `park_button` | `3` | P 선택(Y) |
| `reset_button` | `8` | 초기화(Home/Guide) |
| `maximum_gear_change_speed_mps` | `0.5` | 기어 변경 허용 속도 |
| `minimum_brake_for_gear_change` | `0.5` | 기어 변경에 필요한 최소 브레이크 |
| `maximum_accel_for_gear_change` | `0.05` | 기어 변경에 허용되는 최대 가속 입력 |
| `allow_brake_interlock_without_status` | `true` | 속도 정보가 없어도 브레이크 인터록 허용 |
| `status_timeout` | `0.5` | MORAI 상태 유효 시간 |
| `odometry_topic` | `/localization/odometry` | 상태 UDP가 없을 때 사용할 속도 |
| `odometry_timeout` | `0.5` | odometry 속도 유효 시간 |
| `destination_port` | `9093` | MORAI 명령 수신 포트 |
| `listen_port` | `9094` | MORAI 상태 수신 포트 |
| `command_timeout` | `0.25` | 조이스틱 연결 끊김 판단 |
| `safe_brake` | `0.5` | 연결 끊김 시 브레이크 |
| `reset_pause_duration` | `0.9` | 초기화 중 제어 UDP 정지 시간 |
| `window_name` / `reset_key` | `Simulator` / `i` | 초기화 대상 창과 키 |
| `control_toggle_key` | `q` | MORAI 제어 모드 전환 키 |
| `key_hold` | `0.01` | MORAI 모드 전환(`q`) 입력 유지 시간 |
| `reset_key_hold` | `0.12` | MORAI 초기화(`i`) 입력 유지 시간 |
| `mode_settle` | `0.25` | 리셋 전 Manual 모드 안정화 시간 |
| `builtin_settle` | `0.0001` | 리셋 후 Built-In 모드 체류 시간 |
| `reset_settle` | `0.1` | 초기화 입력 후 모드 복귀 전 대기시간 |

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
