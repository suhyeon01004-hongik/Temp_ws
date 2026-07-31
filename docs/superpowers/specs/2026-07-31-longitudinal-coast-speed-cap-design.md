# Longitudinal coast and speed-cap design

## Goal

직선에서는 60 km/h를 넘지 않는 범위에서 가능한 높은 속도를 유지하고,
작은 목표속도 초과에는 브레이크 대신 타력주행을 사용한다. 곡률 구간은
속도 증가보다 모든 바퀴의 차선선 여유를 우선한다.

## Controller behavior

- `measured_speed - target_speed < 0.05 m/s`: 기존 PID와 속도
  feedforward를 사용한다.
- `0.05 <= measured_speed - target_speed < 0.50 m/s`: 적분값을
  unwind하고 signed effort 목표를 0으로 두어 rate limiter를 거쳐 coast한다.
- `measured_speed - target_speed >= 0.50 m/s`: 기존 PID brake를 사용한다.
- `measured_speed >= 59.0 km/h`: 목표속도와 무관하게 최소 0.25 brake를
  즉시 적용한다.
- 직선 설정 목표는 58.0 km/h이며, 고곡률 제한은 기존 연속식의
  `curvature_speed_reduction_gain_m`을 조정한다.

`ControllerStatus.longitudinal_state`는 `ACCEL`, `COAST`, `BRAKE`,
`HARD_SPEED_BRAKE` 중 하나이고 `speed_overshoot_mps`를 함께 발행한다.

## Verification

- PID 단위 테스트로 coast band, brake threshold, hard-speed brake,
  파라미터 범위를 검증한다.
- 동일 초기화 전체 주행을 두 번 기록한다.
- 두 주행 모두 활성 상태, 비정상 값 0, 측정 속도 60 km/h 미만이어야 한다.
- 직선 brake 샘플과 brake event 수가 기존보다 감소해야 한다.
- IONIQ 5 전·후륜 외곽 기준 3.0 m 차선 가정에서 접촉 샘플 0을 목표로 한다.
