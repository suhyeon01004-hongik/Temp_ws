# Localization

`morai_localization`은 GPS와 IMU를 `map` 기준 차량 pose로 변환한다. UDP 수신,
센서 장착 TF, 경로 선택과 RViz 표시는 다른 패키지가 담당한다.

현재 대회 시뮬레이터의 sensor noise를 모두 끈 상태를 위한 **직접 결합** 구현이다.
평균, 저역통과 필터, 보간, EKF와 UKF는 사용하지 않는다.

## 노드 구성

GPS, IMU와 최종 결합을 서로 다른 노드로 유지한다.

```text
/sensors/gps/fix
  -> gps_utm_projector_node
  -> /localization/gps/local_point

/sensors/imu/data
  -> imu_orientation_adapter_node
  -> /localization/imu/data

GPS local point + normalized IMU
  -> localization_fusion_node
  -> /localization/pose
  -> /localization/odometry
  -> map -> base_footprint -> base_link
```

- GPS projector는 WGS84 위·경도를 K-City local XY로 바꾼다.
- IMU adapter는 quaternion과 운동값의 유효성을 검사하고 quaternion을
  단위 길이로 정규화한다.
- fusion 노드는 GPS의 X/Y와 IMU quaternion에서 구한 yaw를 그대로 결합한다.
- GPS가 `(0, 0, 1.2)`로 뒷차축 중앙의 수직 위에 있으므로 현재 XY lever arm
  보정은 필요 없다.
- 평면 경로 추종용 pose이므로 최종 orientation에는 IMU roll/pitch가 아니라
  yaw만 넣는다.

`base_link`는 URDF에서 이미 `base_footprint`의 자식이다. 따라서 localization은
`map -> base_link`를 직접 발행하지 않고 `map -> base_footprint`를 발행한다.
두 frame의 현재 고정 변환이 identity라 차량 기준점의 위치와 yaw는 같다.

## 입출력

| 구분 | 토픽 | 타입 | frame |
| --- | --- | --- | --- |
| 입력 | `/sensors/gps/fix` | `sensor_msgs/NavSatFix` | `gps_link` |
| 중간 출력 | `/localization/gps/local_point` | `geometry_msgs/PointStamped` | `map` |
| 입력 | `/sensors/imu/data` | `sensor_msgs/Imu` | `imu_link` |
| 중간 출력 | `/localization/imu/data` | `sensor_msgs/Imu` | `imu_link` |
| 최종 출력 | `/localization/pose` | `geometry_msgs/PoseStamped` | `map` |
| 최종 출력 | `/localization/odometry` | `nav_msgs/Odometry` | `map` → `base_link` |
| TF | `map -> base_footprint` | dynamic TF | 차량 위치와 yaw |

`/localization/odometry`의 선속도는 연속 GPS XY의 단순 유한차분이며 필터링하지
않는다. 각속도 Z는 IMU 값을 그대로 사용한다. 첫 GPS 점 또는 잘못된 시간 간격
에서는 선속도가 0으로 남는다.

## yaw 처리

MORAI IMU의 X/Y/Z는 차량 전방/왼쪽/위쪽이고 장착 회전도 현재 `(0, 0, 0)`이다.
정규화한 IMU quaternion에서 yaw를 추출해 다음 식으로 최종 yaw를 만든다.

```text
final_yaw = yaw_sign * imu_yaw + yaw_offset_deg
```

현재 기본값은 `yaw_sign: 1.0`, `yaw_offset_deg: 0.0`이다. 이는 필터 파라미터가
아니라 MORAI와 지도 heading 규약을 맞추기 위한 보정값이다. 차량을 알려진 지도
방향으로 세운 뒤 RViz 화살표가 같은 방향인지 확인하고, 차이가 검증된 경우에만
수정한다.

## 동기화와 안전 게이트

fusion은 최신 GPS와 IMU를 사용하지만 다음 조건이면 출력을 갱신하지 않는다.

- 두 관측 timestamp 차이가 `max_sensor_skew_sec`보다 큼
- GPS age가 `max_gps_age_sec`보다 큼
- IMU age가 `max_imu_age_sec`보다 큼
- frame이 기대값과 다르거나 NaN/Inf가 포함됨

이 검사는 오래된 센서를 섞지 않기 위한 입력 거부 조건이며 필터링이 아니다.
센서가 다시 정상적으로 들어오면 별도 reset 없이 출력을 재개한다.

## 설정

기본 파일은 `config/molit_2026_kcity.yaml`이다.

| 주요 파라미터 | 현재값 | 설명 |
| --- | --- | --- |
| `utm_zone` | `52` | WGS84 UTM zone |
| `east_offset` | `302595.0` | K-City local 원점 Easting |
| `north_offset` | `4124145.0` | K-City local 원점 Northing |
| `imu_input_topic` | `/sensors/imu/data` | raw IMU |
| `imu_output_topic` | `/localization/imu/data` | 정규화 IMU |
| `pose_topic` | `/localization/pose` | 최종 pose |
| `odometry_topic` | `/localization/odometry` | 최종 odometry |
| `tf_child_frame_id` | `base_footprint` | localization이 발행할 TF child |
| `yaw_sign` | `1.0` | IMU yaw 부호, `1.0` 또는 `-1.0` |
| `yaw_offset_deg` | `0.0` | 지도 heading 보정각 |
| `max_sensor_skew_sec` | `0.2` | GPS/IMU timestamp 최대 차이 |
| `max_gps_age_sec` | `1.0` | GPS 최대 age |
| `max_imu_age_sec` | `0.25` | IMU 최대 age |

map이 바뀌면 CRS, UTM zone, offset과 전역경로를 함께 검증해야 한다.

## 실행

GPS projector만 실행:

```bash
roslaunch morai_localization gps_utm_projector.launch
```

GPS projector, IMU adapter와 최종 localization 실행:

```bash
roslaunch morai_localization localization.launch
```

이 launch들은 UDP bridge를 실행하지 않는다. 실제 MORAI 입력을 포함한 통합
시험은 다음을 사용한다.

```bash
roslaunch morai_bringup gps_localization_path_test.launch
```

이 통합 launch의 bridge profile은 GPS `9301`과 IMU `9303`을 함께 수신한다.

## 확인

```bash
rostopic hz /sensors/gps/fix
rostopic hz /sensors/imu/data
rostopic echo -n 1 /localization/imu/data
rostopic echo -n 1 /localization/pose
rostopic echo -n 1 /localization/odometry
rosrun tf tf_echo map base_link
```

RViz:

```bash
roslaunch morai_visualization path.launch
```

빨간 구가 최종 위치, 빨간 화살표가 IMU yaw다. RViz Target Frame은
`base_link`라 차량을 화면 중앙에 두고 차량 진행방향에 맞춰 map을 볼 수 있다.

## noise를 켤 때

대회용 GPS/IMU noise를 활성화하면 현재 direct fusion을 그대로 사용하지 않는다.
토픽 계약은 유지하면서 `localization_fusion_node`만 EKF/UKF 또는
`robot_localization` 기반 구현으로 교체하는 것이 원칙이다.
