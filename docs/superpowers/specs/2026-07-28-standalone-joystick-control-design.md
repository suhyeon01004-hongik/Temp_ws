# 독립형 조이스틱 제어 설계

## 목표

`vehicle_control` 패키지가 localization을 포함한 다른 팀 패키지 없이
Joytron CYVOX MX와 MORAI SIM만으로 조향, 가속, 제동, 안전한 기어 변경 및
초기 위치 복귀를 수행한다.

## 구조

```text
CYVOX MX
  -> joy_node
  -> joystick_teleop_node
     -> /vehicle/manual_command -> morai_udp_sender_node -> MORAI Cmd Control
     -> /vehicle/reset_request  -> morai_sim_reset_node   -> Simulator 창의 I 키

MORAI Ego Vehicle Status UDP
  -> morai_vehicle_status_udp_node
  -> /vehicle/status
  -> joystick_teleop_node
```

모든 사용자 작성 노드와 메시지는 `vehicle_control` 패키지 안에 둔다.
다른 ROS 패키지가 발행하는 pose, odometry 또는 차량 상태에 의존하지 않는다.

## MORAI 차량 상태

`morai_vehicle_status_udp_node`가 `#MoraiInfo$` 패킷을 직접 수신한다.
현재 MORAI 패킷과 구형 패킷의 속도 오프셋을 각각 검증하여 signed speed와
gear를 패키지 자체 `VehicleStatus` 메시지로 발행한다.

기어 변경은 다음 조건을 모두 만족할 때만 허용한다.

- 상태 패킷이 설정된 timeout 이내에 수신됨
- signed speed 절댓값이 `maximum_gear_change_speed_mps` 이하
- A/B/X/Y 중 하나가 새로 눌림

상태가 없거나 오래되면 기존 기어를 유지하고 경고를 출력한다.

## 초기화 버튼

Xbox 360 호환 CYVOX의 Home/Guide 버튼 기본 인덱스를 `8`로 둔다. 버튼의
rising edge에서만 `/vehicle/reset_request`를 한 번 발행한다.

MORAI 공개 `EgoCtrlCmd` UDP에는 초기 위치 복귀 필드가 없으므로 Ubuntu 20.04
X11 환경에서 `xdotool`을 사용해 설정된 `Simulator` 창을 활성화하고 `I` 키를
보낸다. 명령은 shell 문자열이 아니라 argv로 실행하여 설정값이 shell 명령으로
해석되지 않게 한다. 창이나 `xdotool`이 없으면 오류를 출력하되 차량 제어 노드는
계속 동작한다.

## 설정

기본값은 다음과 같다.

- 상태 수신 주소: `0.0.0.0`
- 상태 수신 port: `9094`
- 상태 timeout: `0.5 s`
- 기어 변경 허용 속도: `0.5 m/s`
- 초기화 버튼: `8`
- MORAI 창 이름: `Simulator`
- 초기화 키: `i`

MORAI Cmd Control은 기존 `9093`을 유지한다. Ego Vehicle Status Publisher의
Destination Port를 `9094`로 설정한다.

## 오류 처리와 검증

- 잘못된 header, 길이, 비정상 속도 패킷은 폐기한다.
- UDP bind 실패는 상태 노드를 종료해 중복 port 설정을 드러낸다.
- 상태 timeout 동안 기어 변경을 거부한다.
- Home 버튼을 누르고 있는 동안 초기화 명령을 반복하지 않는다.
- 패킷 파서, UDP loopback, 상태 기반 기어 선택, reset edge 및 명령 구성을
  자동 테스트한다.
