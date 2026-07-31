# 3시간 경로 추종 추가 튜닝 설계

## 목적

현재 `hybrid` 제어기의 두 회 연속 바퀴 접촉 0 결과를 기준선으로 삼아 약
3시간 동안 반복 주행한다. 3.0 m 차선 가정에서 접촉 0과 60 km/h 미만을
하드 제약으로 유지하면서 차선 여유, CTE, 조향 진동, 주행시간 순으로
개선한다. 모든 실험은 재현 가능한 표·CSV·PNG 그래프와 함께 워크스페이스에
남긴다.

## 고정 제약

- 작업 브랜치는 `feat/controller`를 유지한다.
- 사용자가 확인하기 전에는 커밋, push, merge를 하지 않는다.
- `vehicle_control` 패키지는 읽거나 수정하지 않는다.
- Pure Pursuit와 Stanley standalone 계산부는 보존한다.
- 기본 센서와 LiDAR 구성은 변경하지 않는다.
- 목표속도는 58 km/h, 독립 안전제동은 59 km/h, 물리 제한은 60 km/h다.
- 차선 평가는 경로 중심 좌우 1.5 m, 차량 바깥 폭 1.892 m, 휠베이스
  3.0 m를 보수적 기준으로 사용한다.
- 시뮬레이터 창은 확인할 때만 최대화하고 평상시 512×288로 유지한다.
- 주행 전 제어 노드를 끄고 Manual 모드에서 `i`로 위치를 초기화하며,
  시작 위치 오차 1.0 m 이하를 확인한다.

## 비교 우선순위

실험 후보는 아래 사전식 우선순위로 비교한다.

1. `wheel_line_contact_samples == 0`
2. `wheel_min_lane_clearance_m` 최대화
3. CTE max, p95, RMS 최소화
4. steering-rate p95와 RMS 최소화
5. 직선 brake 샘플 0 유지
6. 최고속도 60 km/h 미만에서 완주시간 최소화

접촉이 한 샘플이라도 생긴 후보는 다른 지표가 좋아도 폐기한다. 무접촉
후보끼리는 최소 차선 여유를 먼저 비교하고, 차이가 1 cm 이내일 때만 CTE와
조향률을 사용한다.

## 실험 방식

첫 단계는 현재 `hybrid_cross_track_recovery_full_scale_m=0.50`을 변경하지
않고 반복 주행하여 시뮬레이션 분산을 측정하는 것이다. 이후 한 번에 하나의
파라미터만 변경한다.

1. CTE 복귀 전환: 0.45, 0.50, 0.55 m
2. hybrid PP CTE 보정 gain: 0.45, 0.50, 0.55
3. hybrid 조향 변화율: 55, 60 deg/s
4. 필요할 때만 heading/yaw-rate 항을 분석해 코드 변경 여부 결정
5. 횡제어가 안정된 뒤 PID coast/brake 경계의 직선 재검증

각 후보는 최소 한 번 완주한다. 기준선보다 우수한 후보만 두 번째 완주로
재현성을 확인한다. 세 번 연속 가설이 실패하면 파라미터 탐색을 중단하고
현재 hybrid 구조의 오차 복귀/헤딩 제어 결합을 다시 검토한다.

## 분석 산출물

`morai_path_tracking/scripts/analyze_tracking_bag.py`가 rosbag 한 개 또는 여러
개를 입력받아 다음을 생성한다.

- 실험별 한 행을 갖는 `metrics.csv`
- 경로와 rear-axle 궤적, 차선 경계를 나타낸 `trajectory_comparison.png`
- CTE, 바퀴 여유, 속도/목표속도를 나타낸 `tracking_timeseries.png`
- 조향 후보·최종 조향·조향률을 나타낸 `steering_timeseries.png`
- accel/brake/longitudinal state를 나타낸 `longitudinal_timeseries.png`

최종 산출물은 `docs/path_tracking_tuning/2026-07-31/`에 둔다.
`README.md`에는 실험 조건, 전체 비교표, 채택·폐기 근거, 최종 파라미터,
재현 명령, 차선폭 가정의 한계를 기록한다. 원본 bag은 용량 때문에 `/tmp`에
두되 보고서 manifest에 절대경로, 파일 크기, SHA-256을 기록한다.

## 코드 변경 기준

그래프와 rosbag이 동일 위치에서 같은 원인의 오차 증가를 두 번 이상
보일 때만 제어 코드를 바꾼다. 코드 변경 전에는 해당 rosbag 경로를 고정한
최소 회귀 테스트를 추가하고 실패를 확인한다. 구현은 standalone 제어기를
수정하지 않고 hybrid 조합부 또는 독립된 작은 정책 객체에 한정한다.

## 검증

- 분석 도구의 순수 계산 함수는 합성 궤적을 사용하는 단위 테스트로
  수치와 차선 접촉 판정을 검증한다.
- 모든 제어 코드 변경은 red-green TDD를 따른다.
- 최종 후보는 최소 두 번 연속 완주한다.
- 최종 완주마다 ACTIVE 외 상태, 비유한 값, 60 km/h 초과, 직선 brake,
  바퀴 접촉을 검사한다.
- 종료 직전에 `catkin_make run_tests_morai_path_tracking -j2`와
  `catkin_test_results`를 새로 실행한다.

## 자체 검토

설계에는 미정 파라미터나 구현 위치가 없다. 분석 산출물과 제어 튜닝은
하나의 실험 루프에 속하며 독립 하위 시스템으로 분해할 필요가 없다.
차선폭 3.0 m는 대회 확정값이 아니라 현재 비교를 위한 보수적 가정임을
모든 보고서에 명시한다.
