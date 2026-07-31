# 2026-07-31 MORAI 경로 추종 튜닝 결과

## 결론

최종 설정은 5회 연속 평균 129.7초 전체구간 주행에서 바퀴 차선 접촉
0회를 기록했다. 3.0 m 차선, IONIQ 5 외측 타이어 폭 1.892 m, 축거
3.0 m 가정에서 최악 바퀴 여유는 0.081 m였고 최고속도는
58.31 km/h였다. 직선으로 분류된 3,959개 제어 샘플에서는 brake 명령이
한 번도 나오지 않았다.

최종 변경값은 다음과 같다.

| 파라미터 | 이전 | 최종 | 선택 근거 |
| --- | ---: | ---: | --- |
| `hybrid_pure_pursuit_cross_track_correction_gain` | 0.50 | 0.60 | CTE RMS/p95 감소 |
| `hybrid_cross_track_recovery_full_scale_m` | 0.50 | 0.55 | 큰 heading 오차에서 PP 100% 고착 감소 |
| heading 억제 시작/완료 | 없음 | 15.0/17.5 deg | 전륜 CTE가 heading으로 커진 구간만 제한 |
| heading 최대 억제율 | 없음 | 0.30 | 큰 heading 오차에서 최종 PP 비중을 최대 70%로 제한 |
| `hybrid_steering_return_rate_multiplier` | 1.0 | 2.0 | 커브 탈출 시 조향 명령이 이전 큰 조향각에 남는 지연 감소 |
| `curve_approach_deceleration_mps2` | 2.0 | 1.0 | 2 m 앞 고곡률점 진입 목표를 약 15.9→14.4 km/h로 낮춤 |
| `target_speed_kph` | 58.0 | 58.0 | 58.5는 세 번째 반복에서 접촉 발생 |

## 최종 비교

각 그룹은 동일 초기 위치에서 표에 표시된 횟수만큼 전체 위험구간을
반복했다. `safe`는 바퀴 접촉 샘플이 0인 주행 수다. 여유는 경로 중심에서
좌우 1.5 m에 가상 차선선을 만든 뒤 네 바퀴 외측점의 최악값으로 계산했다.

| 설정 | safe | 접촉 합계 | 최악 여유 | CTE RMS 평균 | CTE p95 평균 | CTE 최악 | 조향률 RMS 평균 | 조향률 p95 최악 | 최고속도 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 최초 기준 | 3/3 | 0 | 0.010 m | 0.151 m | 0.326 m | 0.601 m | 18.00 deg/s | 54.87 deg/s | 58.31 km/h |
| 보정 0.60, heading cap 없음 | 2/3 | 1 | -0.009 m | 0.145 m | 0.307 m | 0.606 m | 18.26 deg/s | 53.64 deg/s | 58.32 km/h |
| recovery weight만 heading cap | 9/10 | 1 | -0.021 m | 0.147 m | 0.319 m | 0.616 m | 18.38 deg/s | 55.36 deg/s | 58.32 km/h |
| 최종 PP 비중 cap, 복귀율 1배 | 5/6 | 1 | -0.017 m | 0.146 m | 0.311 m | 0.632 m | 18.41 deg/s | 59.39 deg/s | 58.33 km/h |
| 복귀율 2배, 감속 2.0 | 0/1 | 1 | -0.010 m | 0.147 m | 0.316 m | 0.624 m | 19.70 deg/s | 51.86 deg/s | 58.31 km/h |
| **최종: 복귀율 2배, 감속 1.0** | **5/5** | **0** | **0.081 m** | **0.137 m** | **0.289 m** | **0.535 m** | **19.95 deg/s** | **57.58 deg/s** | **58.31 km/h** |
| 58.5 km/h 시험 | 2/3 | 1 | -0.006 m | 0.147 m | 0.319 m | 0.602 m | 18.22 deg/s | 54.99 deg/s | 58.83 km/h |

최초 기준 대비 최종 설정은 최악 바퀴 여유를 0.010 m에서 0.081 m로
7.1 cm 늘렸다. CTE RMS 평균은 0.151→0.137 m, p95 평균은
0.326→0.289 m, 최악값은 0.601→0.535 m로 모두 감소했다. 조향 복귀율을
높여 조향률 RMS는 18.00→19.95 deg/s로 증가했지만 직접
accel↔brake 전환은 평균 18.0→16.0회로 줄었고 차선 안전 여유가 크게
개선됐다.

## 종방향 결과

최종 5회 평균 제어 샘플은 `ACCEL 2829.8`, `COAST 597.2`,
`BRAKE 465.4`, `HARD_SPEED_BRAKE 0`이었다. 최초 기준의
`COAST 431.3`, `BRAKE 423.3`보다 타력주행은 평균 165.9샘플 늘었다.
브레이크 증가는 고곡률 진입 목표를 더 일찍 낮춘 결과이며, 최종 5회 모두
직선 brake는 0회였다. 따라서 직선의 불필요한 제동 없이 위험 커브에서만
제동량을 추가한 상태다.

58.5 km/h 목표는 실측 최고 58.83 km/h로 60 km/h 미만이었지만 세 번째
반복에서 바퀴 접촉이 발생했다. 따라서 최고속도보다 차선 안전을 우선해
58.0 km/h 목표와 59.0 km/h 독립 hard brake 경계를 유지했다.

## 탈락 실험

| 실험 | 결과 | 판단 |
| --- | --- | --- |
| 곡률 감속 gain 5.0→6.0 | 접촉 1회, 여유 -0.003 m | 탈락 |
| hybrid 조향률 60→55 deg/s | 두 번째 주행 접촉 1회, -0.007 m | 탈락 |
| 최소 LD 4.0→3.5 m | 접촉 1회, -0.001 m | 탈락 |
| heading 완전 억제 7.5→15 deg | 두 번째 주행 접촉 1회, -0.010 m | 범위가 넓어 탈락 |
| heading 완전 억제 15→17.5 deg | 접촉은 없으나 여유 0.024 m, CTE p95 0.330 m | 과도한 Stanley 전환 |
| recovery weight만 최대 30% 억제 | 10번째 주행 접촉 1회, -0.021 m | IMM PP 비중이 높으면 cap이 무효라 탈락 |
| 최종 PP 비중 cap + 복귀율 1배 | 6번째 주행 접촉 1회, -0.017 m | 헤어핀 탈출 조향 복귀 지연 |
| 조향 복귀율 2배만 적용 | 접촉 1회, -0.010 m | 횡제어 수정만으로 여유 부족 |
| 목표속도 58.0→58.5 km/h | 세 번째 주행 접촉 1회, -0.006 m | 탈락 |

## 산출물

- [요약 그래프](final_comparison/summary_metrics.png)
- [경로 및 궤적 비교](final_comparison/trajectory_comparison.png)
- [CTE·바퀴 여유·속도 시계열](final_comparison/tracking_timeseries.png)
- [조향 후보·최종 명령·조향률](final_comparison/steering_timeseries.png)
- [IMM·effective weight·heading cap](final_comparison/hybrid_weights_timeseries.png)
- [엑셀·브레이크·종방향 상태](final_comparison/longitudinal_timeseries.png)
- [개별 주행 수치 CSV](final_comparison/metrics.csv)
- [반복 주행 집계 CSV](final_comparison/aggregate_metrics.csv)
- [전체 상세 수치 YAML](final_comparison/metrics.yaml)
- [원본 bag 경로·크기·SHA256](final_comparison/bag_manifest.yaml)
- [최종 5회 전용 요약 그래프](final_repeated/summary_metrics.png)
- [최종 5회 전용 경로 비교](final_repeated/trajectory_comparison.png)
- [최종 5회 전용 시계열](final_repeated/tracking_timeseries.png)
- [최종 5회 전용 수치 CSV](final_repeated/metrics.csv)
- [최종 5회 전용 bag SHA256](final_repeated/bag_manifest.yaml)

그래프와 CSV는 다음 명령 형식으로 재생성할 수 있다.

```bash
source /opt/ros/noetic/setup.bash
source /home/suhyeon/catkin_ws/devel/setup.bash
rosrun morai_path_tracking analyze_tracking_bag.py \
  run_01=/tmp/example_01.bag \
  run_02=/tmp/example_02.bag \
  --output-dir /home/suhyeon/catkin_ws/docs/path_tracking_tuning/example
```

## 계산 가정과 한계

- `base_link`는 `ioniq5_description` 기준 뒷차축 중심이다.
- 네 바퀴 외측점은 `x=0/3.0 m`, `y=±0.946 m`로 계산한다.
- 경로 중심 좌우 1.5 m를 차선선으로 가정한다.
- MORAI 무노이즈 조건에서 Competition Vehicle Status의 x속도를 사용한다.
- 이 검사는 타이어 외측점의 평면 기하 검사이며 실제 대회 차선폭이 바뀌면
  `--lane-half-width-m`부터 다시 설정해야 한다.
- rosbag 원본은 용량 때문에 `/tmp`에 유지하며, 추적 가능하도록 manifest에
  절대경로와 SHA256을 기록했다.
