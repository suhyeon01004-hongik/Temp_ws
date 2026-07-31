# Stanley Tracking and Command Smoothing Design

## Goal

실주행 기록에서 확인된 우측 편향, 좌우 조향 진동, 곡률 목표속도 요동,
엑셀·브레이크의 잦은 전환을 줄인다. 대회 IONIQ 5 좌표계와 기존 UDP
인터페이스는 변경하지 않는다.

## Evidence

- `ioniq5_description/config/vehicle_specs.yaml`과 URDF에서 `base_link`는
  후륜 차축 중심이고 차량 좌표는 x 전방, y 좌측이다.
- 축간거리는 3.0 m이므로 Stanley 제어점 `(3.0, 0.0)`은 앞차축 중심과
  일치한다.
- 기록된 19.8초 주행에서 차량은 항상 경로 오른쪽에 있었고 후륜 CTE는
  평균 +0.45 m였다.
- 양의 조향 명령 뒤 약 0.2초 후 양의 yaw가 발생했으므로 조향 부호와 UDP
  변환 부호는 정상이다.
- 조향과 heading error의 상관계수는 0.973이었다. 공식 경로의 짧은 단일
  선분 heading은 인접 구간에서 보통 4~6 deg, 일부에서 11.17 deg까지
  변했다.
- Competition Status 측정 속도는 매끄러웠지만 곡률 기반 목표속도와
  조향 명령은 반복해서 방향이 바뀌었다.

## Coordinate and Stanley Model

map 경로점은 rear-axle `base_link` pose를 기준으로 다음처럼 차량 좌표로
변환한다.

```text
x_body =  cos(yaw) * dx + sin(yaw) * dy
y_body = -sin(yaw) * dx + cos(yaw) * dy
```

Stanley 최근접점 검색은 앞차축 `(wheelbase_m, 0)`을 각 경로 선분에
투영해서 수행한다. 투영점 자체는 시각화와 CTE 기준점으로 유지한다.
경로 heading만 투영점 앞뒤를 합쳐 총 `stanley_heading_window_m` 길이의
공간 chord로 계산한다. 공간 평활화는 시간 LPF와 달리 차량 응답에 추가
시간 지연을 만들지 않는다.

```text
steering = smoothed_heading_error
         + atan2(stanley_gain * cross_track_error,
                 max(abs(speed), minimum_control_speed) + softening_speed)
```

기본 `stanley_heading_window_m`은 4.0 m, `stanley_gain`은 2.0으로 한다.
초기 공간 평활화 단계에서는 40 deg 물리 제한과 90 deg/s 변화율 제한을
유지하며 원인을 분리한다. 아래 동역학 감쇠 확장에서는 기록 결과에 따라
변화율 제한을 60 deg/s로 낮춘다.

## Damped Curvature-Feedforward Stanley Extension

공간 heading 평활화 이후 수집한 두 기록에서도 고속 진동은 남았다.
30초 기록에서 50 km/h 이상 steering rate RMS는 약 40 deg/s였고,
조향-차량 yaw 응답 지연은 약 0.23초였다. 45초 기록의 강한 커브 탈출
시점에는 경로 곡률이 0에 가까워진 뒤에도 조향 약 12 deg와 yaw rate
약 0.38 rad/s가 남았다. 경로 곡률은 안정적인 동안 heading feedback과
실제 yaw rate가 진동했으므로, 추가 시간 LPF 대신 다음 제어식을 사용한다.

```text
kappa_ref = signed_curvature(path at target, target + 4 m, target + 8 m)
r_ref = abs(speed) * kappa_ref

steering = curvature_feedforward_gain * atan(wheelbase * kappa_ref)
         + heading_error_gain * smoothed_heading_error
         + atan2(stanley_gain * cross_track_error,
                 max(abs(speed), minimum_control_speed) + softening_speed)
         - yaw_rate_damping_gain * (measured_yaw_rate - r_ref)
```

곡률 선행항은 커브 정상상태에 필요한 조향을 오차 없이 공급한다. yaw-rate
오차 감쇠는 목표보다 큰 회전만 억제하므로 정상 커브 회전 자체와 싸우지
않고, 커브 탈출 시 남은 회전을 직접 줄인다. localization odometry의
`angular.z`는 pose yaw와 동일한 `yaw_sign`이 적용되므로 좌회전 양수인
차량 좌표 부호가 일치한다.

두 bag에 맞춘 지연 1차 yaw 모델 탐색을 근거로 초기값은 feedforward 1.0,
heading gain 0.7, yaw-rate damping 0.1 s, 조향 변화율 60 deg/s로 한다.
모델은 실차 폐루프를 완전히 재현하지 않으므로 최종 이득은 재주행 기록으로
확정한다.

## Speed Target Smoothing

곡률 프로파일에서 계산한 raw 목표속도 뒤에 1차 저역통과 필터를 두고,
그 뒤에 기존 비대칭 가속·감속 변화율 제한을 적용한다.

```text
alpha = 1 - exp(-dt / target_speed_filter_time_constant_sec)
filtered = filtered + alpha * (raw - filtered)
target = slew_limit(filtered, previous_target, accel_limit, decel_limit)
```

기본 시정수는 0.35초다. 55 km/h에서 약 5.3 m에 해당하며 45 m 곡률
preview 안에서 충분히 작다. `0.0`은 필터 비활성으로 둔다. 초기화 또는
안전상태 복귀 후 첫 목표는 현재 raw 목표로 바로 초기화한다.

Competition Status 속도는 현재 기록상 노이즈 문제가 없으므로
`speed_filter_time_constant_sec: 0.0`을 유지한다. 피드백 속도 필터를
추가하면 종방향 응답 지연만 증가한다.

## Longitudinal Command Smoothing

PID의 포화된 accel/brake 출력을 signed effort로 표현한다.

```text
signed_effort = accel - brake
```

정상 제어 중 signed effort 변화량을
`longitudinal_command_rate_limit_per_sec * dt`로 제한한다. 따라서 accel에서
brake로 전환할 때 반드시 0에 가까운 coast 구간을 지나며 두 페달은 계속
상호 배타적이다. 기본값은 2.0/s이고 `0.0`은 제한 비활성이다.

입력 timeout이나 잘못된 frame에서 사용하는 `safe_brake_command`는 PID를
거치지 않으므로 변화율 제한 없이 즉시 발행한다.

## Diagnostics and Configuration

`ControllerStatus`에 곡률 계산 직후의 `raw_target_speed_mps`와 LPF 뒤의
`filtered_target_speed_mps`를 추가한다. 기존 `target_speed_mps`는 최종
slew-limited PID 목표를 유지한다.

새 설정:

- `stanley_heading_window_m: 4.0`
- `target_speed_filter_time_constant_sec: 0.35`
- `longitudinal_command_rate_limit_per_sec: 2.0`

측정 속도 LPF와 조향 시간 LPF는 기본 비활성이다. 특히 조향 시간 LPF는
이미 확인된 약 0.2초 차량 응답 지연을 더 키울 수 있어 적용하지 않는다.

## Verification

- 단위 테스트로 앞차축 좌표, CTE 부호, 공간 heading 평활화, 짧은 경로
  fallback, 설정 검증을 확인한다.
- 목표속도 LPF가 갑작스러운 raw 목표 변화에서 중간값을 만들고 reset 후
  새 경로 raw 값으로 초기화되는지 확인한다.
- PID signed effort가 설정된 변화율을 넘지 않고 accel/brake 전환 시
  coast를 통과하며 reset 시 상태가 지워지는지 확인한다.
- config 계약, ROS node 테스트, 전체 패키지 빌드와 테스트를 수행한다.
- 실주행 후 동일 토픽 bag으로 CTE 평균, 조향 방향 전환 횟수, raw/filtered/
  final 목표속도, accel/brake 전환 횟수를 다시 비교한다.

## Constraints

- `vehicle_control` 패키지와 연결하지 않는다.
- 제어기는 계속 `/control/actuator_command`만 발행하고 실제 UDP 변환은
  `morai_udp_bridge`가 담당한다.
- 브랜치는 `feat/controller`를 유지한다.
- 사용자 실주행 확인 전에는 commit, merge, push를 하지 않는다.
