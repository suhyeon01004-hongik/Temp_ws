# Pure Pursuit 경로 추종 제어기 설계

## 목표

`/local_path`를 추종하는 ROS1 Pure Pursuit 제어기를 추가한다. 제어기는
localization에서 추정한 차량 속도로 종방향 PID를 수행하고, 정규화된 accel,
brake와 실제 앞바퀴 조향각을 하나의 ROS 명령으로 발행한다.
`morai_udp_bridge`는 이 명령을 검증하고 MORAI UDP 패킷으로 변환한다.

본 기능은 `vehicle_control`의 코드, 메시지, 노드와 launch를 사용하지 않는다.

## 현재 시스템에서 사용하는 사실

- ROS1 Noetic, catkin, C++14를 사용한다.
- `base_link`와 `base_footprint`의 평면 원점은 뒷차축 중앙이다.
- 차량 축은 `x` 전방, `y` 좌측, `z` 위다.
- wheelbase는 `3.000 m`다.
- 최대 조향각은 `40 deg`다.
- `minimum_turning_radius_m`와 `40 deg` 사이의 기존 제원 불일치는 이번
  구현에서 최소 회전반경을 사용하지 않고 사용자 확인값인 `40 deg`를 조향
  clamp로 사용해 해소한다.
- `/local_path`는 `map` frame의 `nav_msgs/Path`이며 기본 설정에서 약
  `9.5 m` 전방을 담는다.
- `/local_path`의 pose orientation은 경로 접선이며 차량 자세가 아니다.
- `/localization/odometry`의 pose는 뒷차축 중앙의 위치와 IMU yaw를 나타낸다.
- 센서 노이즈 규정이 확정되지 않았으므로 현재 단계는 GPS와 IMU noise가 없는
  MORAI 설정을 기준으로 한다.

## 범위

### 포함

- `morai_localization`의 위치 차분 기반 속도 추정 정식화
- GPS 위치 변화와 IMU yaw를 결합한 차량 전진 속도 계산
- 작은 지연의 선택 가능한 1차 저역통과 필터
- Pure Pursuit 횡방향 제어
- localization 추정속도를 사용하는 종방향 PID
- accel, brake, steering angle 명령 발행
- MORAI control UDP 직렬화, 송신, watchdog
- 자율주행 전용 launch와 YAML 설정
- 변경되는 모든 패키지 README와 루트 README 갱신
- 코어 알고리즘 단위 테스트와 launch/config 정적 테스트

### 제외

- `vehicle_control` 수정 또는 재사용
- 조이스틱과 자율주행 사이의 command mux
- MORAI 차량 상태 UDP에서 직접 속도를 읽는 기능
- IMU 가속도 적분, EKF/UKF, wheel odometry
- 장애물 회피, 곡률 기반 속도 계획, 정지선과 mission 종료 처리
- 후진 경로 추종과 자동 기어 변경

수동 `vehicle_control` UDP sender와 자율주행 UDP sender는 동시에 실행하지
않는다. 두 sender가 같은 MORAI 명령 포트로 패킷을 보내면 제어권이 충돌한다.

## 패키지 책임

### `morai_localization`

차량 상태 추정의 소유자다. GPS local XY의 시간 차분으로 map-frame 속도를
구하고 IMU yaw로 차량 frame에 투영한다. 유효한 추정속도를 기존
`/localization/odometry.twist.twist`에 넣는다.

속도 계산은 ROS node에서 분리한 `VelocityEstimator` C++ 코어로 구현한다.
`localization_fusion_node`는 입력 검증, ROS 메시지와 TF 발행만 담당한다.

### `morai_path_tracking`

새 패키지다. `/local_path`와 `/localization/odometry`만 구독한다.
odometry pose로 경로점을 차량 frame으로 변환하고 Pure Pursuit 조향각을
계산한다. odometry의 전진 속도와 설정된 목표속도로 PID를 수행해 accel과
brake를 계산한다.

이 패키지는 socket, UDP packet layout, MORAI 차량 상태와
`vehicle_control`을 알지 못한다.

### `morai_udp_bridge`

MORAI wire protocol의 소유자다. 새 `ActuatorCommand` 메시지를 공개 인터페이스로
제공하고, 별도 control sender node가 메시지를 검증해 MORAI packet으로
직렬화한다. 명령 수신이 끊기면 watchdog이 안전 제동 packet을 보낸다.

기존 Camera/GPS/IMU 수신 node와 설정은 그대로 유지한다. control sender는
별도 executable과 별도 launch/config로 추가한다.

### `morai_bringup`

센서, localization, path manager, path tracking controller와 control UDP
sender를 조합하는 자율주행 전용 launch를 제공한다. 기존 sensor/localization/
path launch의 책임은 바꾸지 않는다.

## 데이터 흐름

```text
MORAI GPS + IMU
    -> morai_udp_bridge sensor receiver
    -> morai_localization
       - map pose
       - GPS delta / dt
       - IMU yaw projection
       - optional low-pass filter
    -> /localization/odometry

/localization/pose
    -> morai_path_manager
    -> /local_path

/local_path + /localization/odometry
    -> morai_path_tracking
       - Pure Pursuit
       - longitudinal PID
    -> /control/actuator_command

/control/actuator_command
    -> morai_udp_bridge control sender
    -> MORAI #MoraiCtrlCmd$ UDP packet
```

## ROS 인터페이스

### 제어기 입력

| 토픽 | 타입 | 단위/frame | 의미 |
| --- | --- | --- | --- |
| `/local_path` | `nav_msgs/Path` | `map`, m | 전방 부분경로 |
| `/localization/odometry` | `nav_msgs/Odometry` | `map -> base_link`, m/s | 차량 pose와 추정속도 |

제어기는 path pose orientation을 사용하지 않는다. 차량 yaw는 odometry pose의
quaternion에서 얻는다.

### 제어기 출력

`morai_udp_bridge/ActuatorCommand`를 새로 정의한다.

```text
std_msgs/Header header
float32 accel
float32 brake
float32 steering_angle_rad
```

- `accel`: `[0.0, 1.0]`
- `brake`: `[0.0, 1.0]`
- `steering_angle_rad`: 양수는 좌회전, 음수는 우회전
- 정상 제어에서는 accel과 brake 중 하나만 0보다 클 수 있다.
- 기본 토픽은 `/control/actuator_command`다.

메시지는 `morai_udp_bridge`가 소유한다. 이는 UDP bridge가 받아들일 수 있는
공개 입력 계약이며, `morai_path_tracking`은 이 메시지에만 의존한다.
`vehicle_control` 메시지 의존성은 생기지 않는다.

## 속도 추정

### 계산

연속 GPS local point `(x, y, t)`에 대해 다음 map-frame 속도를 계산한다.

```text
vx_map = (x_now - x_prev) / dt
vy_map = (y_now - y_prev) / dt
```

동기화된 IMU yaw를 `psi`라 할 때 차량 frame 속도는 다음과 같다.

```text
vx_body =  cos(psi) * vx_map + sin(psi) * vy_map
vy_body = -sin(psi) * vx_map + cos(psi) * vy_map
```

PID는 signed `vx_body`를 사용한다. GPS 안테나 XY가 뒷차축 중앙과 일치하므로
현재 sensor setup에서는 lever-arm 보정이 필요 없다.

IMU linear acceleration은 적분하지 않는다. noise-off 조건에서도 가속도 적분은
중력, 장착점 회전 운동과 bias 처리라는 별도 문제를 만들며, 현재 GPS 30 Hz
경로 추종에는 필요하지 않다. IMU는 yaw를 제공해 GPS 속도 벡터를 차량 전진
방향으로 투영하는 데 사용한다.

### 필터

GPS 좌표의 유한 정밀도와 차분 증폭을 완화하기 위해 다음 1차 필터를 사용한다.

```text
alpha = dt / (tau + dt)
v_filtered = v_previous + alpha * (v_raw - v_previous)
```

기본 `tau`는 `0.10 s`다. 첫 유효 표본은 0부터 ramp하지 않고 raw 값으로
필터를 초기화한다. `tau: 0.0`으로 설정하면 필터를 완전히 끌 수 있다.
필터는 차량 frame의 `vx_body`, `vy_body`에 각각 적용하며 PID는 필터링된
`vx_body`만 사용한다.

### 유효성

- 첫 GPS 표본은 baseline만 만들며 유효 속도를 출력하지 않는다.
- `dt`가 설정 범위를 벗어나면 estimator를 재초기화한다.
- raw 속도의 절댓값이 설정된 최대치를 넘으면 해당 추정을 폐기하고
  baseline을 현재 점으로 옮긴다.
- GPS 또는 IMU freshness/synchronization 검사가 실패하면 기존 localization과
  같이 새 pose와 odometry를 발행하지 않는다.
- pose와 TF는 첫 유효 GPS/IMU 조합부터 발행한다.
- odometry는 속도 추정이 유효한 두 번째 위치 표본부터 발행한다.
- 시간 역행이나 큰 시간 공백 뒤에는 PID가 오래된 속도를 사용하지 않도록
  odometry 발행을 한 주기 이상 중단하고 estimator를 다시 초기화한다.

### localization 설정 기본값

기존 `molit_2026_kcity.yaml`에 다음 값을 둔다.

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `minimum_velocity_dt_sec` | `0.005` | 허용 최소 위치 표본 간격 |
| `maximum_velocity_dt_sec` | `0.25` | 허용 최대 위치 표본 간격 |
| `maximum_velocity_mps` | `50.0` | raw 속도 물리 상한 |
| `velocity_filter_time_constant_sec` | `0.10` | LPF 시정수, `0`이면 비활성 |

## Pure Pursuit 횡방향 제어

### 목표점 선택

1. odometry pose를 이용해 모든 local path 점을 `base_link` 평면 frame으로
   변환한다.
2. 차량 뒤쪽의 점만으로 이루어진 segment는 건너뛴다.
3. 차량 원점에서 lookahead 원과 처음 교차하는 전방 path segment를 찾고
   교차점을 선형 보간한다.
4. 교차점이 없지만 전방 path가 존재하면 마지막 전방점을 사용한다.
5. 사용 가능한 전방점이 없거나 마지막 점도 최소 목표거리보다 가까우면
   정상 조향을 만들지 않고 안전 명령으로 전환한다.

lookahead는 현재 속도에 따라 다음처럼 계산한다.

```text
lookahead = clamp(
    lookahead_base_m + lookahead_speed_gain_sec * abs(vx_body),
    lookahead_min_m,
    lookahead_max_m)
```

기본 목표속도 `3.0 m/s`에서는 lookahead가 `4.5 m`이며 현재 약 `9.5 m`
local path 안에 충분히 들어온다.

### 조향식

차량 frame 목표점이 `(x_target, y_target)`이고 실제 목표거리 제곱이
`Ld_squared`일 때 다음 bicycle model 식을 사용한다.

```text
steering = atan2(2 * wheelbase * y_target, Ld_squared)
```

양수 `y_target`은 좌측이므로 양수 조향각을 만든다. 결과는
`[-40 deg, +40 deg]`로 제한한다.

### 횡방향 설정 기본값

| 파라미터 | 기본값 |
| --- | ---: |
| `wheelbase_m` | `3.0` |
| `lookahead_base_m` | `3.0` |
| `lookahead_speed_gain_sec` | `0.5` |
| `lookahead_min_m` | `3.0` |
| `lookahead_max_m` | `6.0` |
| `minimum_target_distance_m` | `0.5` |
| `maximum_steering_angle_deg` | `40.0` |

## 종방향 PID

### 계산

목표속도는 우선 고정 YAML 파라미터로 둔다. 초기값은 `3.0 m/s`
(`10.8 km/h`)다.

```text
error = target_speed - estimated_speed
u = kp * error + ki * integral - kd * measured_speed_derivative
```

derivative는 error가 아니라 측정속도에 적용해 향후 목표속도가 바뀔 때
derivative kick을 피한다. integral은 설정 범위로 제한하고 output saturation
중 error가 saturation을 더 키우는 방향이면 적분하지 않는 conditional
anti-windup을 적용한다.

```text
u > 0  -> accel = min(u, maximum_accel_command), brake = 0
u < 0  -> accel = 0, brake = min(-u, maximum_brake_command)
```

작은 speed error deadband에서는 P와 D 입력을 0으로 취급하되 이미 쌓인
integral은 anti-windup 규칙에 따라 천천히 해소한다. 입력이 stale하거나
시간이 역행하면 PID 상태를 즉시 reset한다.

초기 gain은 실제 MORAI 폐루프 시험 전의 보수적인 시작값이며 모두 YAML에서
바꿀 수 있다.

| 파라미터 | 기본값 |
| --- | ---: |
| `target_speed_mps` | `3.0` |
| `speed_kp` | `0.35` |
| `speed_ki` | `0.08` |
| `speed_kd` | `0.02` |
| `speed_integral_limit` | `2.0` |
| `speed_error_deadband_mps` | `0.05` |
| `maximum_accel_command` | `0.40` |
| `maximum_brake_command` | `0.60` |

## 제어 주기와 입력 동기화

- controller timer 기본 주기는 `30 Hz`다.
- path와 odometry callback은 최신 유효 메시지만 저장한다.
- freshness는 ROS header stamp와 callback 수신 wall time을 함께 검사한다.
- path와 odometry stamp 차이는 기본 `0.10 s` 이하여야 한다.
- path와 odometry의 frame은 모두 `map`이어야 한다.
- timer의 monotonic wall-time `dt`가 비정상이면 PID를 reset한다.

| 파라미터 | 기본값 |
| --- | ---: |
| `local_path_topic` | `/local_path` |
| `odometry_topic` | `/localization/odometry` |
| `command_topic` | `/control/actuator_command` |
| `expected_frame_id` | `map` |
| `control_rate_hz` | `30.0` |
| `path_timeout_sec` | `0.25` |
| `odometry_timeout_sec` | `0.25` |
| `maximum_input_skew_sec` | `0.10` |
| `safe_brake_command` | `0.50` |

## 안전 동작

다음 중 하나라도 만족하면 controller는 accel `0`, brake `0.5`,
steering `0`의 stamped 안전 명령을 계속 발행하고 PID를 reset한다.

- path 또는 odometry를 아직 받지 못함
- path 또는 odometry가 timeout
- path/odometry stamp skew 초과
- frame 불일치
- 빈 path, 비유한 pose, 비유한 속도
- forward lookahead target 없음
- timer `dt` 오류

UDP sender는 controller와 독립적인 2차 안전 계층이다.

- 명령의 세 값 중 하나라도 NaN/Inf면 전체 명령을 거부한다.
- accel/brake 범위가 `[0,1]` 밖이면 전체 명령을 거부한다.
- steering angle이 설정 최대각 밖이면 전체 명령을 거부한다.
- accel과 brake가 동시에 양수면 전체 명령을 거부한다.
- 유효 명령을 기본 `0.25 s` 동안 받지 못하면 accel `0`, brake `0.5`,
  steering `0`, Drive gear의 packet을 `50 Hz`로 보낸다.

## MORAI UDP 변환

control sender는 현재 저장소에서 검증된 55-byte `#MoraiCtrlCmd$` layout을
`morai_udp_bridge` 안에 독립적으로 구현한다.

- control mode: external control
- longitudinal command type: accel/brake
- gear: Drive
- accel/brake: controller의 `[0,1]` 값
- MORAI steering:

```text
normalized_steering =
    steering_sign * steering_angle_rad / radians(maximum_steering_angle_deg)
```

기본 `steering_sign`은 `1.0`이고 `maximum_steering_angle_deg`는 `40.0`이다.
실제 MORAI 좌/우 부호 확인 결과가 반대이면 YAML의 sign만 `-1.0`으로 바꾼다.

| 파라미터 | 기본값 |
| --- | ---: |
| `command_topic` | `/control/actuator_command` |
| `destination_ip` | `127.0.0.1` |
| `destination_port` | `9093` |
| `send_rate_hz` | `50.0` |
| `command_timeout_sec` | `0.25` |
| `safe_brake_command` | `0.50` |
| `maximum_steering_angle_deg` | `40.0` |
| `steering_sign` | `1.0` |
| `drive_gear` | `4` |

## 파일 구성

### `morai_localization`

```text
include/morai_localization/velocity_estimator.hpp
src/velocity_estimator.cpp
src/localization_fusion_node.cpp
test/test_velocity_estimator.cpp
config/molit_2026_kcity.yaml
README.md
```

### `morai_path_tracking`

```text
CMakeLists.txt
package.xml
include/morai_path_tracking/pure_pursuit.hpp
include/morai_path_tracking/pid_controller.hpp
src/pure_pursuit.cpp
src/pid_controller.cpp
src/pure_pursuit_controller_node.cpp
test/test_pure_pursuit.cpp
test/test_pid_controller.cpp
config/molit_2026_pure_pursuit.yaml
launch/pure_pursuit.launch
README.md
```

### `morai_udp_bridge`

```text
msg/ActuatorCommand.msg
include/morai_udp_bridge/control_protocol.hpp
include/morai_udp_bridge/udp_sender.hpp
src/control_protocol.cpp
src/udp_sender.cpp
src/control_sender_node.cpp
test/test_control_protocol.cpp
test/test_udp_sender.cpp
config/molit_2026_control.yaml
launch/control_sender.launch
README.md
```

### `morai_bringup`과 루트

```text
src/morai_bringup/launch/molit_2026_autonomous.launch
src/morai_bringup/test/test_launch_composition.py
src/morai_bringup/README.md
README.md
```

CMake와 package manifest는 실제 생성되는 libraries, messages, nodes, tests와
runtime dependency를 모두 명시하고 install space에서도 config와 launch가
동작하도록 설치한다.

## 시험

### 속도 추정 단위 테스트

- 첫 표본은 invalid이고 두 번째 정상 표본부터 속도가 유효함
- 직선 등속 운동의 body `vx` 계산
- map Y 방향 운동과 yaw `90 deg` 조합의 body `vx` 계산
- 후진 운동의 signed 음수 속도
- LPF 초기화와 시정수 기반 갱신
- `tau=0` 필터 비활성
- 너무 작거나 큰 `dt`, 시간 역행 reset
- 비유한 입력과 최대 속도 초과 거부

### Pure Pursuit 단위 테스트

- 직선 경로의 0 조향
- 좌/우 경로의 조향 부호
- lookahead 교차점 선형 보간
- 첫 경로점이 차량 뒤에 있는 경우
- 짧은 경로에서 마지막 전방점 fallback
- 전방점 없음과 비유한 path 거부
- `40 deg` 조향 clamp

### PID 단위 테스트

- 목표속도 미만에서 accel
- 목표속도 초과에서 brake
- accel/brake 상호 배타성
- integral limit와 conditional anti-windup
- derivative-on-measurement
- deadband
- reset 후 내부 상태 제거
- 비정상 `dt`와 비유한 입력 거부

### UDP bridge 단위 테스트

- 정확한 55-byte MORAI control packet
- accel/brake와 steering angle 정규화
- `40 deg`가 정규화 steering `1.0`이 됨
- steering sign 반전
- NaN/Inf, 범위 초과, 동시 accel/brake 거부
- watchdog safe packet
- localhost UDP 송신

### 통합/정적 테스트

- 새 패키지와 변경 패키지의 manifest/CMake dependency
- config와 launch 파일 설치
- 자율주행 bringup이 sensor, localization, path manager, controller,
  control sender를 한 번씩 포함
- 자율주행 bringup이 `vehicle_control`을 포함하지 않음
- 기존 bringup launch 구성이 바뀌지 않음

검증 명령은 전체 workspace build, 전체 tests, test result 확인 순서로 수행한다.
MORAI 실행 시험에서는 직진 조향 부호, 저속 가속, 목표속도 수렴,
path/localization 중단 시 안전 제동을 순서대로 확인한다.

## README 갱신

- 루트 README: 자율주행 data flow, 새 토픽, 실행법, 제한사항
- `morai_localization/README.md`: 속도 추정식, 필터, validity, config
- `morai_path_tracking/README.md`: 알고리즘, 입출력, 파라미터, 튜닝, 안전 동작
- `morai_udp_bridge/README.md`: sensor와 control 양방향 책임, packet, watchdog
- `morai_bringup/README.md`: 자율주행 launch와 수동 제어 동시 실행 금지

문서에는 noise-off 가정, 초기 PID gain이 MORAI 시험용 시작값이라는 점,
센서 노이즈 규정 확정 시 estimator 재검토가 필요하다는 점을 명시한다.
