# 2026 MORAI sensor bridge review

## 검토 기준

- 원본: `~/다운로드/morai_udp_bridge-20260718T093137Z-1-001.zip`
- SHA-256: `28dfaf78c0b82ff9bed284e1f0663de691d18d503a5633bf2468caf2e3b5e8ba`
- 규정집: v1.0, 2026-07-11 제정, 2026-07-15 최종 갱신
- 대회 SIM: `25.S4.MolitComp03`
- 대회 map: `R-KR_PG_K-City_2025`

원문:

- [2026 대회 규정집](https://morai.atlassian.net/wiki/external/ZThjYjQwNDNiM2ZkNDBjODk2MDk4MmM5MjA3Y2ZmODM)
- [MORAI 센서 UDP 프로토콜](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-35)
- [MORAI 센서 좌표계](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-8)
- [MORAI 맵 좌표계](https://help-morai-sim.scrollhelp.site/ko/morai-sim-drive/24.R2/-9)
- [MORAI 공식 센서 예제](https://github.com/MORAI-Autonomous/MORAI-SensorExample)

## 규정 대응

| 항목 | 규정 | 구현 상태 |
|---|---|---|
| GPS | 1대, UDP, 최대 30 Hz | NMEA0183 checksum/GGA 검증 후 raw `NavSatFix` 발행 |
| IMU | 1대, UDP, 최대 50 Hz | 공식 107-byte와 예제 115-byte 형식을 엄격 검증 |
| 3D LiDAR | 1대, VLP-16, 최대 15 Hz, 10 Hz 이하 권장 | native UDP를 공식 ROS Velodyne driver에 연결; 의존성 설치 필요 |
| Camera | 최대 4대, UDP, 최대 30 Hz | 고정 3대의 분할 JPEG 수신/재조립과 `CameraInfo` 발행 |
| 고정 카메라 | 규정 위치·자세·FOV 변경 불가 | description YAML/URDF를 단일 원본으로 구성 |
| GPS blackout | 모든 GPS 정보 blackout | no-fix는 NaN/`STATUS_NO_FIX`, 단절은 stale 진단; 과거값 재발행 없음 |

규정의 주기 상한은 diagnostics에서 감시한다. 실제 SIM UI의 data rate 자체는
시뮬레이터 설정이므로 팀 운영 체크리스트에서도 별도로 확인해야 한다.

## 원본 코드에서 보완한 항목

- 한 파일에 있던 UDP socket, 진단, ROS publisher, UTM 변환을 각각 transport,
  stream adapter, localization 패키지로 분리했다.
- 차량 스펙, 센서 모델/frame/장착값을 `ioniq5_description`으로 옮기고 통신
  YAML의 하드웨어 값 중복을 제거했다.
- Camera tail을 `AI`/`EI`로 제한하고 0-byte/비정상 size를 거부한다.
- IMU를 정확히 107/115 byte, data size 80, zero auxiliary bytes, 정확한 CRLF
  tail 조건으로 검증하여 trailing garbage를 허용하지 않는다.
- NMEA checksum과 위도/경도 hemisphere를 엄격 검증한다.
- 중복 bind endpoint, 포트 범위, timeout/rate/buffer 값 검증을 추가했다.
- 허용하지 않은 송신 IP의 폐기 수와 실제 kernel UDP receive buffer를 진단한다.
- 규정 최대 주기를 diagnostics에서 경고하도록 했다.

## 의도적으로 확정하지 않은 값

- 규정에 없는 GPS/IMU/LiDAR 장착 위치
- MORAI Sensor Edit 차량-local 원점과 차체 중심/axle 사이의 관계
- 최종 시나리오 JSON의 UTM east/north offset
- 규정에 없는 mass, inertia, wheel track, 실제 카메라 distortion calibration

이 값들은 0으로 추정해 TF나 localization 결과를 발행하지 않는다. 카메라
`CameraInfo`는 규정 해상도/FOV로 만든 이상적 pinhole 모델이며, 최종 SIM 영상으로
projection을 한 번 더 검증해야 한다.

## 검증 결과

- catkin 전체 빌드 성공
- parser/좌표 변환 테스트 19개, 실패 0
- ROS runtime에서 Camera 3대, GPS, IMU UDP 수신 및 raw topic 발행 확인
- 규정 카메라 xyz/rpy 및 optical frame 정적 TF 확인
- 미확정 GPS/IMU/LiDAR TF가 URDF에 생성되지 않음을 확인
- `config_verified: false`일 때 UTM projector가 시작을 거부함을 확인

최종 승인을 위해 남은 시험은 `25.S4.MolitComp03`에서 캡처한 실제 datagram
fixture, 장시간 packet-loss/burst 시험, SIM UI의 센서 원점 및 최종 map offset
확인이다.
