# MORAI path tracking

`morai_path_tracking`은 map-frame local path와 localization pose, 대회용
Competition Vehicle Status의 x축 속도를 받아 자율
`morai_udp_bridge/ActuatorCommand`를 발행한다. `vehicle_control` 및 수동
`/vehicle/status`와는 완전히 독립적이다. 횡제어는 config에서 Pure Pursuit
Pure Pursuit, Stanley 또는 IMM hybrid를 선택하고, 곡률 속도 계획과 종방향
PID는 세 모드가 공유한다.

## Controller model

`base_link` is the rear axle center: x points forward and y points left. For a
vehicle pose `(x_v, y_v, yaw)` and map point `(x_m, y_m)`, the controller uses

```
dx = x_m - x_v; dy = y_m - y_v
x_body = cos(yaw) * dx + sin(yaw) * dy
y_body = -sin(yaw) * dx + cos(yaw) * dy
```

가장 가까운 local-path 점부터 전방 경로를 일정 간격으로 재샘플링하고,
연속 세 점의 외접원 곡률과 전방거리를 함께 계산한다. 각 커브의 지점
제한속도 `v_curve`와 현재 허용속도 `v_now`는 다음과 같다.

```
v_lateral = sqrt(maximum_lateral_acceleration / curvature)
v_curve = clamp(v_lateral / (1 + curvature_speed_reduction_gain * curvature),
                minimum_curve_speed, configured_target)
v_now = sqrt(v_curve^2 + 2 * curve_approach_deceleration * curve_distance)
```

Preview 안의 `v_now` 중 최솟값을 raw 목표로 사용하므로 멀리 있는 커브가
보이더라도 커브 지점 제한속도로 즉시 떨어지지 않는다. 최종 PID 목표는
raw 목표속도 LPF와 가속·감속 변화율 제한을 차례로 거친다. 추가 감속 gain은
임계값 없이 연속으로 적용되며 곡률이 클수록 제한속도를 더 많이 낮춘다.

Pure Pursuit LD는 speed preview 곡률과 분리된 차량 앞 근거리 곡률만 사용해
`clamp((base + speed_gain * abs(speed)) /
(1 + curvature_gain * curvature), min, max)`로 계산한다. 선택한 target에는
bicycle-model 조향각
`atan2(2 * wheelbase * target_y, target_x^2 + target_y^2)`을 적용하고 물리
조향 한계로 제한한다. 종방향 PID는 측정 속도의 미분과 목표속도 비례 accel
feedforward를 사용한다. 합산 출력의 양수는 `accel`, 음수는 `brake`가 되어
두 페달이 동시에 출력되지 않는다. 정상 PID 출력은 signed effort
`accel - brake` 기준 변화율 제한을 거치므로 가속에서 제동으로 전환할 때
coast를 통과한다. 입력 오류의 안전 브레이크는 이 제한을 우회한다.

Stanley는 후륜 중심에서 축간거리만큼 앞선 전륜 중심을 경로 선분에
투영한다. `ioniq5_description`의 `base_link`는 후륜 차축 중심이고 축간거리가
3.0 m이므로 Stanley 제어점 `(3.0, 0.0)`은 실제 앞차축 중심과 일치한다.
가장 가까운 투영점은 횡오차와 시각화 기준으로 그대로 유지하되, 경로
heading은 투영점 앞뒤 총 `stanley_heading_window_m` 길이의 chord로 계산해
짧은 경로 선분의 방향 튐을 공간적으로 평활화한다.
Pure Pursuit처럼 전방 LD point를 고르는 알고리즘은 아니며, 이 전륜 투영점이
Stanley의 추종 기준점이다.

```text
reference_curvature = signed_curvature(path at 0 m, 4 m, 8 m)
reference_yaw_rate = abs(speed) * reference_curvature

steering = curvature_feedforward_gain
             * atan(wheelbase * reference_curvature)
         + heading_error_gain * spatially_smoothed_heading_error
         + atan2(stanley_gain * cross_track_error,
                 max(abs(speed), minimum_control_speed) + softening_speed)
         - yaw_rate_damping_gain
             * (measured_yaw_rate - reference_yaw_rate)
```

곡률 feedforward가 정상 커브에 필요한 조향을 횡·헤딩 오차가 커지기 전에
공급하고, yaw-rate 오차 항은 차량이 목표보다 더 빨리 회전할 때만 반대
조향을 더한다. 따라서 커브 탈출 시 경로 곡률은 0으로 내려갔지만 차량
yaw rate가 남은 상황에서 기존 heading 항의 과조향을 감쇠한다.
최종 Stanley 조향은 공용 40 deg 물리 한계와 설정 가능한 조향 변화율로
제한한다. `lateral_controller` 선택은 시작할 때 읽으므로 변경 후 노드를
재시작해야 한다. 기본 `hybrid`는 PP와 Stanley 요청을 IMM 확률로 혼합한다.
두 후보가 급커브에서 반대 방향이면 PP 우선 guard를 적용한다. 두 후보가
같은 복귀 방향이어도 Stanley가 약해 큰 횡오차가 계속 증가할 때는 PP 비중을
연속적으로 높인다. 이 보호 로직은 hybrid에만 적용되고 standalone 구현은
변경하지 않는다.

조향 명령에는 별도의 시간 LPF를 적용하지 않는다. 기록에서 차량 조향 응답이
명령보다 약 0.2초 늦었으므로 시간 필터를 추가하면 위상 지연이 더 커질 수
있다. 경로 heading은 공간적으로 평활화하고, 남은 동역학 진동은 yaw-rate
오차 감쇠로 처리한다.

현재 시뮬레이터는 GPS와 IMU noise가 꺼진 상태를 전제로 한다. PID 속도는
`/vehicle/competition_status.velocity_x_mps`만 사용한다.
`/localization/odometry.twist`는 속도 fallback으로 사용하지 않는다.
기본 속도 필터는 비활성이고 필요할 때만 config의 시정수로 활성화한다.

## Interfaces

| Direction | Topic | Type | Units |
| --- | --- | --- | --- |
| Input | `/local_path` | `nav_msgs/Path` | map-frame metres |
| Input | `/localization/odometry` | `nav_msgs/Odometry` | map-frame 위치·자세 |
| Input | `/vehicle/competition_status` | `morai_udp_bridge/CompetitionVehicleStatus` | vehicle x속도 m/s, gear, control mode |
| Output | `/control/actuator_command` | `morai_udp_bridge/ActuatorCommand` | accel/brake `[0,1]`; steering radians |
| Output | `/control/controller_status` | `morai_path_tracking/ControllerStatus` | 선택 횡제어기, 횡/헤딩 오차, 곡률제한·목표속도, LD, 명령, 안전 상태 |
| Output | `/control/lookahead_point` | `geometry_msgs/PointStamped` | `base_link` 기준 추종점: Pure Pursuit LD 또는 Stanley 전륜 투영점 |

노드는 기본 30 Hz로 발행한다. Config 기본 직선 목표는 대회 제한보다
2 km/h 낮은 **58.0 km/h**다. 실제 속도가 59.0 km/h 이상이면 정상 PID와
명령 변화율 제한보다 우선하는 최소 `brake=0.25` 안전제동을 적용한다.

## Configuration

All private parameters are required and type-checked at startup. Defaults are in
`config/molit_2026_path_tracking.yaml`.

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `local_path_topic` | `/local_path` | `nav_msgs/Path` input |
| `odometry_topic` | `/localization/odometry` | `nav_msgs/Odometry` input |
| `vehicle_status_topic` | `/vehicle/competition_status` | Competition Vehicle Status input |
| `command_topic` | `/control/actuator_command` | actuator command output |
| `controller_status_topic` | `/control/controller_status` | 제어 진단 출력 |
| `lookahead_point_topic` | `/control/lookahead_point` | 선택 횡제어기의 추종점 출력 |
| `expected_frame_id` | `map` | path/odometry frame |
| `expected_velocity_frame_id` | `base_link` | Competition x속도 및 LD frame |
| `control_rate_hz` | `30.0` | WallTimer publication rate |
| `path_timeout_sec` | `0.25` | path receipt and ROS-stamp age limit |
| `odometry_timeout_sec` | `0.25` | odometry receipt and ROS-stamp age limit |
| `vehicle_status_timeout_sec` | `0.25` | Competition Status receipt age limit |
| `maximum_input_skew_sec` | `0.0` | 동기화할 path/odometry source stamp 최대 차이 |
| `input_sync_queue_size` | `10` | path/odometry message-filter queue |
| `minimum_control_dt_sec` | `0.005` | inclusive lower bound for each WallTimer control interval |
| `maximum_control_dt_sec` | `0.10` | inclusive upper bound for each WallTimer control interval |
| `safe_brake_command` | `0.50` | brake output in the safe state |
| `wheelbase_m` | `3.0` | rear-axle-to-front-axle distance |
| `lookahead_base_m` | `4.0` | zero-speed lookahead contribution |
| `lookahead_speed_gain_sec` | `0.75` | lookahead gain applied to speed |
| `lookahead_curvature_gain_m` | `8.0` | 곡률이 커질 때 LD를 줄이는 gain, 0이면 비활성 |
| `lookahead_min_m` | `4.0` | lower lookahead clamp |
| `lookahead_max_m` | `16.0` | upper lookahead clamp |
| `minimum_target_distance_m` | `0.5` | shortest valid forward target |
| `maximum_steering_angle_deg` | `40.0` | physical steering magnitude limit |
| `lateral_controller` | `hybrid` | `pure_pursuit`, `stanley`, `hybrid`; 변경 후 재시작 |
| `stanley_gain` | `2.0` | 횡오차 보정 gain |
| `stanley_softening_speed_mps` | `2.0` | 저속 Stanley 분모에 더하는 속도 |
| `stanley_minimum_control_speed_mps` | `1.0` | Stanley 계산에 사용하는 최소 속도 |
| `stanley_heading_window_m` | `4.0` | 앞차축 투영점 기준 공간 heading chord의 전체 길이 |
| `stanley_heading_error_gain` | `0.6` | 지연된 heading feedback 비중 |
| `stanley_curvature_feedforward_gain` | `1.0` | bicycle-model 곡률 선행 조향 gain |
| `stanley_curvature_preview_distance_m` | `8.0` | signed reference curvature를 계산할 전방 거리 |
| `stanley_yaw_rate_damping_gain_sec` | `0.1` | 목표 대비 과도한 회전을 감쇠하는 yaw-rate 오차 gain |
| `stanley_maximum_steering_rate_deg_per_sec` | `60.0` | Stanley 명령 조향 변화율 한계 |
| `hybrid_pure_pursuit_cross_track_correction_gain` | `0.60` | hybrid PP 후보에 더하는 Stanley CTE 항 비중 |
| `hybrid_candidate_conflict_cross_track_threshold_m` | `0.45` | 후보 부호 충돌 시 CTE 기반 PP guard 임계 |
| `hybrid_cross_track_recovery_full_scale_m` | `0.55` | 절반부터 PP 비중을 부드럽게 높여 전량 적용하는 CTE |
| `hybrid_cross_track_recovery_heading_error_suppression_start_deg` | `15.0` | 큰 heading 오차에서 PP 강제 복귀 감쇠 시작 |
| `hybrid_cross_track_recovery_heading_error_suppression_full_deg` | `17.5` | heading 기반 감쇠가 최대가 되는 오차 |
| `hybrid_cross_track_recovery_heading_error_maximum_suppression_ratio` | `0.30` | 큰 heading 오차에서 최종 PP 비중을 제한하는 최대 비율 |
| `hybrid_maximum_steering_rate_deg_per_sec` | `60.0` | hybrid 진입 방향 기본 조향 변화율 한계 |
| `hybrid_steering_return_rate_multiplier` | `2.0` | 같은 방향에서 조향 절대값을 줄이는 커브 탈출 복귀율 배수 |
| `target_speed_kph` | `58.0` | 운전자가 설정하는 직선 최고 목표 속도, km/h |
| `minimum_curve_speed_kph` | `12.0` | 곡률 제한 후 적용할 최소 목표 속도, km/h |
| `maximum_lateral_acceleration_mps2` | `1.8` | 곡률 제한 속도 계산의 허용 횡가속도 |
| `curvature_speed_reduction_gain_m` | `5.0` | 곡률이 커질수록 제한속도를 추가로 낮추는 연속 gain, 0이면 비활성 |
| `curvature_preview_distance_m` | `45.0` | 차량 최근접 경로점부터 곡률을 탐색할 거리 |
| `lookahead_curvature_preview_distance_m` | `8.0` | LD에 반영할 차량 앞 근거리 곡률 범위 |
| `curvature_sample_spacing_m` | `2.0` | 곡률 계산용 전방 경로 재샘플 간격 |
| `curve_approach_deceleration_mps2` | `1.0` | 커브 거리 기반 현재 허용속도 계산의 감속 능력 가정 |
| `curvature_epsilon_m_inv` | `0.001` | 이 값 이하를 직선으로 처리하는 곡률 기준 |
| `target_speed_acceleration_limit_mps2` | `2.0` | 최종 목표 속도의 최대 상승률 |
| `curve_target_speed_acceleration_limit_mps2` | `0.2` | preview 안에 제한 곡선이 남아 있을 때 목표속도 재상승률 |
| `target_speed_deceleration_limit_mps2` | `5.0` | 최종 목표 속도의 최대 하강률 |
| `target_speed_filter_time_constant_sec` | `0.35` | 곡률 raw 목표속도 LPF 시정수, 0이면 비활성 |
| `speed_filter_time_constant_sec` | `0.0` | Competition 속도 LPF 시정수, 0이면 비활성 |
| `speed_kp` | `0.18` | speed PID proportional gain |
| `speed_ki` | `0.02` | speed PID integral gain |
| `speed_kd` | `0.0` | speed PID derivative-on-measurement gain, 기본 비활성 |
| `speed_integral_limit` | `1.0` | absolute PID integral clamp |
| `speed_integral_unwind_rate_per_sec` | `0.5` | positive integral unwind rate while speed error is in the deadband |
| `speed_error_deadband_mps` | `0.10` | P/D speed-error suppression band |
| `speed_accel_feedforward_gain_per_mps` | `0.008` | 목표속도 1 m/s당 기본 accel |
| `speed_coast_overspeed_kph` | `0.2` | 목표 초과 후 타력주행을 시작하는 속도차 |
| `speed_brake_overspeed_kph` | `1.8` | 정상 PID 제동을 시작하는 목표 초과 속도차 |
| `hard_brake_activation_speed_kph` | `59.0` | 독립 최고속도 안전제동 경계 |
| `minimum_hard_brake_command` | `0.25` | 최고속도 guard의 최소 브레이크 |
| `maximum_accel_command` | `0.40` | acceleration output cap |
| `maximum_brake_command` | `0.60` | PID braking output cap |
| `longitudinal_command_rate_limit_per_sec` | `2.0` | 정상 PID signed effort의 초당 최대 변화, 0이면 비활성 |

`target_speed_kph`는 config에서만 km/h다. `configured_target_speed_mps`,
`raw_target_speed_mps`, `filtered_target_speed_mps`,
`curvature_speed_limit_mps`, `target_speed_mps`를 포함한 상태 토픽과 모든
내부 계산은 m/s를 사용한다. `raw`는 곡률 계산 직후, `filtered`는 1차 LPF
직후, `target`은 최종 가감속 변화율 제한 뒤의 PID 입력이다. 기본 곡률
제한의 반경별 목표 속도는 다음과 같다.

| 곡선 반경 | 목표 속도 |
| ---: | ---: |
| 100 m | 약 46.0 km/h |
| 50 m | 약 31.0 km/h |
| 25 m | 약 20.1 km/h |
| 15 m | 약 14.0 km/h |
| 10 m 이하 | 12 km/h 하한 |

58 km/h 직선 LD는 최대 clamp인 16 m다. 기본 local path는 100 m 거리 기준이며,
45 m speed preview와 16 m 최대 LD가 모두 이 범위 안에 들어온다. LD는 이
45 m 곡률이 아니라 `lookahead_curvature_preview_distance_m` 기본 8 m 안의
곡률만 사용한다.

58 km/h에서 feedforward 기본 출력은 약 0.129다. 완전한 직선에서 속도가
목표보다 낮게 정착하면 `speed_accel_feedforward_gain_per_mps`를 올리고,
목표 부근에서 계속 가속하거나 오버슈트가 크면 내린다. 커브 감속이 여전히
이르면 `curve_approach_deceleration_mps2`를 올리고, 늦으면 내린다. 완만한
커브 제한속도 자체는 `maximum_lateral_acceleration_mps2`를 올리면 빨라지고
내리면 보수적으로 바뀐다. 완만한 커브 속도는 최대한 유지하면서 고곡률
감속만 더 강하게 하려면 `curvature_speed_reduction_gain_m`을 올리고,
고곡률 감속이 과하면 내린다. `0.0`은 추가 감속을 끈다.

기본 `speed_kp=0.18`은 곡률 목표가 내려갈 때 brake 응답을 강화한 시작값이다.
Competition Status의 불규칙한 표본 간격이 D항 펄스를 만들 수 있어
`speed_kd=0.0`을 기본으로 둔다. 목표속도보다 deadband 이상 빠르면
작게 초과하면 feedforward와 적분을 풀고 `COAST`를 유지한다. 설정한
`speed_brake_overspeed_kph` 이상 초과할 때만 정상 PID 브레이크를 허용하며,
D항을 다시 켜더라도 초과속도에서 accel은 출력하지 않는다. 실제
차량에서 목표 부근 진동이 크면 `speed_kp` 또는 feedforward gain을 낮춘다.
곡률에 따라 목표 자체가 톱니처럼 변하면
`target_speed_filter_time_constant_sec`를 올리고, 커브 감속 시작이
지나치게 늦어지면 내린다. 기본 0.35초는 58 km/h에서 약 5.6 m의 진행
거리에 해당하며 45 m preview보다 충분히 짧다.

`curve_approach_deceleration_mps2`는 직접 brake 명령을 정하는 값이 아니라
앞으로 가능한 감속 능력의 가정이다. 값을 낮출수록 감속을 더 일찍 시작하고,
높일수록 늦게 시작한다. 기본 `1.0`은 2 m 앞 고곡률 제한점의 허용속도를
기존 `2.0`보다 약 1.5 km/h 낮춰 헤어핀 외측 바퀴 여유를 확보한다.

`ControllerStatus.preview_curvature_m_inv`는 속도를 제한한 곡률,
`speed_limiting_curve_distance_m`는 그 지점까지 거리(`-1`이면 없음),
`lookahead_curvature_m_inv`는 LD에 사용한 근거리 곡률이다.
`raw_target_speed_mps`, `filtered_target_speed_mps`,
`target_speed_mps`를 함께 보면 곡률 계산, LPF, 변화율 제한을 분리해서
확인할 수 있다.
`longitudinal_state`는 `ACCEL`, `COAST`, `BRAKE`,
`HARD_SPEED_BRAKE` 중 하나이고 `speed_overshoot_mps`는 목표 초과량이다.
hybrid의 두 확률과 두 effective weight,
`hybrid_candidate_conflict_guard_active`,
`hybrid_cross_track_recovery_active/weight`는 IMM 판단과 CTE 복귀
override를 분리해 보여준다.
`hybrid_cross_track_recovery_heading_suppression_active/weight`는 큰
heading 오차 때문에 전륜 Stanley CTE가 커질 때 PP 비중 상한을 얼마나
낮췄는지 보여준다. 기본값은 15~17.5도 사이에서 연속 전환하며 최종
PP 비중을 최대 70%로 제한한다. `hybrid_steering_return_rate_multiplier`는
후보 조향과 이전 조향의 부호가 같고 절대값이 감소할 때만 적용하므로,
반대 방향 전환이나 코너 진입 조향률은 높이지 않는다.
`lateral_controller`, `cross_track_error_m`, `heading_error_rad`로 현재
횡제어 모드와 Stanley 오차를 확인할 수 있다.
`reference_curvature_m_inv`, `reference_yaw_rate_radps`,
`measured_yaw_rate_radps`, `yaw_rate_error_radps`를 함께 보면 곡률 선행항과
회전 감쇠항을 분리해 확인할 수 있다. Pure Pursuit 모드에서는 Stanley 전용
필드를 0으로 발행한다. `/control/lookahead_point`와
`lookahead_point_base`는 Pure Pursuit에서는 LD target, Stanley에서는
전륜 중심에서 경로로 내린 최근접 투영점이다. Stanley에는 LD가 없으므로
`lookahead_distance_m`은 0이다.

현재 Stanley 기본값은 두 번의 30~45초 실주행 기록에서 확인된 고속
좌우 진동과 커브 탈출 과조향을 줄이기 위한 초기값이다. 기존 제어에서는
초기 55 km/h 시험에서 steering rate RMS가 약 40 deg/s였고, 커브 종료 뒤에도
조향 약 12 deg와 yaw rate 약 0.38 rad/s가 남았다. 8 m 곡률 feedforward,
0.6 heading gain, 0.1 s yaw-rate 감쇠를 적용하고 30 Hz 주기당 최대 조향
변화를 2 deg로 제한한다.
고속 횡오차 복귀가 여전히 약하면 `stanley_gain`을 조금씩 올리고, 공간
heading이 경로 굴곡을 지나치게 둔화하면 `stanley_heading_window_m`을
줄인다. 저속 조향이 민감하면 `stanley_softening_speed_mps`를 올린다.
커브 정상상태 조향이 부족할 때는 `stanley_curvature_feedforward_gain`,
탈출 시 회전이 남으면 `stanley_yaw_rate_damping_gain_sec`를 먼저 조정한다.
코너 진입 조향 변화가 제한기에 오래 걸릴 때만
`stanley_maximum_steering_rate_deg_per_sec`를 올린다.

`longitudinal_command_rate_limit_per_sec=2.0`이면 정상 제어 명령은 30 Hz
한 주기당 약 0.067 이하로 변한다. 값이 너무 작으면 실제 커브 제동이
늦어질 수 있고, 너무 크면 accel/brake 전환이 다시 거칠어진다. timeout 등
안전상태의 `safe_brake_command`는 이 값과 무관하게 즉시 적용된다.

2026-07-31 MORAI K-City 평균 129.7초 반복 주행에서 최종 hybrid 설정은
5회 연속 바퀴 접촉 0회, 최악 최소 여유 0.081 m, 최대 CTE 0.535 m,
CTE RMS 평균 0.137 m, 최고 58.31 km/h를 기록했다. 차량 폭 1.892 m를
바퀴 바깥 한계로 사용하고 경로 중심 기준 좌우 1.5 m 차선선을 가정했다.
총 3,959개 직선 샘플의 brake 명령은 0회였다. 58.5 km/h 목표는
세 번째 반복에서 접촉 1회가
발생해 채택하지 않았다. 상세 표·그래프·bag SHA256은
`docs/path_tracking_tuning/2026-07-31/README.md`에 있다.

## Safety behavior

Controller는 path와 odometry를 source stamp 기준으로 먼저 동기화한 뒤 하나의
pair로 저장한다. 새 path만 먼저 들어온 callback 구간에는 직전 정상 pair를
timeout까지 유지하므로 정상 프레임 사이에 `brake=0.5`가 끼지 않는다.

동기화된 pair 또는 Competition Status가 없거나 stale인 경우, frame/stamp/값이
잘못된 경우, timer `dt`가 범위를 벗어나거나 유효한 횡제어 target이 없는 경우에는
PID와 목표 속도 변화율 상태를 reset하고
`(accel=0, brake=0.5, steering=0)`을 발행한다. 정확한 원인은
`/control/controller_status.state`에서 확인한다.

## Run and test

```bash
roslaunch morai_path_tracking path_tracking.launch
roslaunch morai_path_tracking path_tracking.launch config:=/path/to/controller.yaml

catkin_make run_tests_morai_path_tracking
catkin_test_results build/test_results/morai_path_tracking --verbose
```

분석기 인자는 ROS remap 문법이 아니라 `LABEL=/absolute/path.bag` 형식이다.
예시는 다음과 같다.

```bash
rosrun morai_path_tracking analyze_tracking_bag.py \
  run_01=/tmp/run_01.bag run_02=/tmp/run_02.bag \
  --output-dir /tmp/path_tracking_report
```

launch와 config 파일명은 기존 호환성을 위해 유지하지만, 실제 횡제어기는
YAML의 `lateral_controller`가 결정한다.

Do not run a separate manual controller or other sender against MORAI while the
autonomous controller is active.
