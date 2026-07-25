# 조이스틱 차량 제어 설계

## 목표

Joytron CYVOX MX 조이스틱으로 MORAI Ego 차량을 수동 조작한다. 조이스틱 입력
처리와 MORAI UDP 송신을 분리해, 이후 자율주행 제어기도 같은 차량 명령
인터페이스를 사용할 수 있게 한다.

## 범위

- 새 ROS1 Noetic 패키지 이름은 `vehicle_control`로 한다.
- MORAI의 `AV-ExternalCtrl`과 `Ego Ctrl Cmd` UDP 통신을 사용한다.
- 첫 버전은 전진 기어 `D`만 지원한다.
- 제어권 중재, 자율주행 제어기, 차량 상태 수신, 진동 출력은 포함하지 않는다.

## 구성

```text
CYVOX MX
  → joy/joy_node
  → /joy (sensor_msgs/Joy)
  → joystick_teleop_node
  → /vehicle/manual_command (vehicle_control/VehicleCommand)
  → morai_udp_sender_node
  → MORAI Ego Ctrl Cmd UDP
```

### `joystick_teleop_node`

`sensor_msgs/Joy`를 차량에 독립적인 `VehicleCommand`로 변환한다.

- 왼쪽 스틱 좌우: 조향 `[-1, 1]`
- RT: 가속 `[0, 1]`
- LT: 브레이크 `[0, 1]`
- 가속 트리거를 놓으면 `accel=0`, `brake=0`으로 타력 주행한다.
- 스틱 deadzone 적용 후 조향 범위를 다시 정규화한다.
- 축 번호와 축 반전은 YAML 파라미터로 바꿀 수 있다.

현재 CYVOX MX는 Linux에서 `Microsoft X-Box 360 pad`로 인식된다. 확인된 축은
왼쪽 스틱 X `0`, LT `2`, RT `5`이며 LT와 RT의 원시 초기값은 `-1`이다. 트리거
출력은 `(axis + 1) / 2`로 변환한다.

### `VehicleCommand`

메시지는 다음 필드만 가진다.

- `std_msgs/Header header`
- `float32 accel`
- `float32 brake`
- `float32 steering`
- `uint8 GEAR_PARK=1`
- `uint8 GEAR_REVERSE=2`
- `uint8 GEAR_NEUTRAL=3`
- `uint8 GEAR_DRIVE=4`
- `uint8 GEAR_LOW=5`
- `uint8 gear`

값의 범위는 가속·브레이크 `[0, 1]`, 조향 `[-1, 1]`이다. 기어 값은 MORAI
규격과 동일하게 `P=1`, `R=2`, `N=3`, `D=4`, `L=5`를 사용한다.

### `morai_udp_sender_node`

`VehicleCommand`를 대회용 MORAI `MolitComp03`에서 실제로 수신하는
`#MoraiCtrlCmd$` 55바이트 UDP 패킷으로 직렬화한다. `data_length`는 `23`이며
rear steering 필드는 포함하지 않는다.

- CtrlMode: `2` (`AutoMode`)
- Gear: `VehicleCommand.gear`의 `P=1`, `R=2`, `N=3`, `D=4`
- longCmdType: `1` (`Throttle`)
- velocity, acceleration: `0`
- front steering: 정규화된 `[-1, 1]`
- 기본 송신 주기: `50 Hz`

수신한 명령이 `0.25초` 동안 갱신되지 않으면 조이스틱 연결이 끊긴 것으로
판단한다. 이때 가속을 `0`, 조향을 `0`, 브레이크를 `0.5`로 바꿔 계속 송신한다.
마지막으로 선택한 기어는 유지한다. 제한 시간과 안전 브레이크 값은 YAML에서
수정할 수 있다.

## 설정

`config/cyvox_mx.yaml`에서 다음 값을 관리한다.

- 조이스틱 장치 경로
- 조향·LT·RT 축 번호와 반전
- 조향 deadzone
- P/R/N/D 버튼 번호와 초기 기어
- odometry 속도 토픽, 유효 시간, 기어 변경 허용 최고 속도
- 명령 제한 시간과 안전 브레이크 값
- MORAI 목적지 IP와 Cmd Control 포트
- UDP 송신 주기

MORAI 목적지는 같은 PC에서 실행하는 환경과 MORAI Cmd Control 기본 설정을
기준으로 `127.0.0.1:9093`을 기본값으로 한다. YAML을 불러오지 않고 UDP 송신
노드를 직접 실행해도 같은 포트를 사용하도록 코드 내부 기본값도 `9093`으로
통일한다. `joy_node`의 자동 반복은 `20 Hz`, deadzone은 `0.05`로 설정한다.

현재 노트북에서는 조이스틱 번호가 바뀌어도 같은 장치를 선택하도록
`/dev/input/by-id/usb-ShanWan_Xbox360_For_Windows_10F36D6-joystick`을 기본값으로
사용한다. 다른 CYVOX를 연결하면 이 경로만 바꾸면 된다.

## 실행

`launch/cyvox_morai.launch`가 `joy_node`, `joystick_teleop_node`,
`morai_udp_sender_node`를 함께 실행한다. MORAI에서는 차량 제어 모드를
`AV-ExternalCtrl`로 선택하고, Network Settings의 Cmd Control 포트를 YAML과
같게 설정해야 한다.

## 오류 처리

- 조이스틱 메시지의 축 수가 부족하면 차량 명령을 만들지 않고 오류를 제한적으로
  출력한다.
- 여러 기어 버튼을 함께 누르거나 저속 조건을 만족하지 않으면 현재 기어를
  유지한다.
- odometry 속도가 없거나 오래됐으면 기어 변경을 거부한다.
- NaN과 범위를 벗어난 입력은 허용 범위로 제한한다.
- UDP 주소나 포트가 잘못되면 송신 노드는 시작 단계에서 실패 이유를 출력한다.
- 조이스틱 메시지가 끊기면 마지막 가속 명령을 유지하지 않고 안전 명령으로
  전환한다.

## 시험

- 놓은 트리거 값 `1`이 `0`으로 변환되는지 시험한다.
- 끝까지 누른 트리거 값 `-1`이 `1`로 변환되는지 시험한다.
- 조향 deadzone, 반전, 범위 제한을 시험한다.
- 얼굴 버튼 기어 매핑, 상승 에지, 저속 제한과 stale odometry를 시험한다.
- MORAI UDP 패킷이 정확히 55바이트이고 header, tail, 제어 필드가 규격과
  일치하는지 시험한다.
- 제한 시간이 지나면 안전 명령이 선택되는지 시험한다.
- catkin 빌드 후 실제 `/joy`, `/vehicle/manual_command`, UDP 송신을 확인한다.

## 완료 조건

- CYVOX를 조작하면 ROS 차량 명령 토픽에 가속·브레이크·조향 값이 나타난다.
- 가속 트리거를 놓았을 때 자동 브레이크 없이 타력 주행 명령이 발행된다.
- 조이스틱 연결이 끊기면 설정된 안전 브레이크가 발행된다.
- MORAI `AV-ExternalCtrl`에서 차량이 조이스틱 명령에 반응한다.
- 축, 기어 버튼, 속도 제한과 네트워크 설정을 코드 수정 없이 YAML에서 변경할 수
  있다.
