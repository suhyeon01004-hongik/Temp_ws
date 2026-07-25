# 조이스틱 기어 선택 설계

## 목표

CYVOX MX의 얼굴 버튼으로 MORAI 차량 기어를 선택한다.

- `A(0)`: Drive
- `B(1)`: Neutral
- `X(2)`: Reverse
- `Y(3)`: Park

버튼 번호는 `vehicle_control/config/cyvox_mx.yaml`에서 변경할 수 있다.

## 동작

현재 기어는 다음 기어 버튼을 누를 때까지 유지한다. 버튼을 새로 누른 순간에만
요청을 처리하며, 둘 이상의 기어 버튼을 동시에 누르면 변경하지 않는다.

`/localization/odometry`의 선속도 크기가 기본 `0.5 m/s` 이하일 때만 기어를
변경한다. 속도 메시지가 없거나 설정된 제한 시간보다 오래됐으면 변경하지
않는다. 속도 기준, 토픽, 제한 시간과 초기 기어는 YAML 파라미터로 관리한다.

## 구조

순수 C++ `GearSelector`가 버튼 상승 에지, 속도 유효성, 저속 조건과 기어 상태를
관리한다. `joystick_teleop_node`는 odometry와 Joy를 받아 선택된 기어를
`VehicleCommand`에 넣는다. `morai_udp_sender_node`는 메시지의 기어를 그대로
MORAI UDP 패킷에 기록한다.

## 오류 처리와 시험

- 잘못되거나 중복된 버튼 번호는 시작 단계에서 거부한다.
- NaN 속도, 오래된 속도, 다중 버튼 입력에서는 현재 기어를 유지한다.
- 단위 시험으로 버튼 매핑, 상승 에지, 저속 제한, stale 속도와 다중 입력을
  검증한다.
- ROS 통합 시험으로 odometry와 Joy 입력이 올바른 `VehicleCommand.gear`를
  만드는지 확인한다.
- 패킷 시험으로 선택된 기어가 MORAI 기어 바이트에 반영되는지 확인한다.
