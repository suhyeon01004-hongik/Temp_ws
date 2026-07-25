# README 온보딩 문서 개편 설계

## 목적

새 팀원이 저장소를 처음 받은 뒤 전체 구조를 이해하고, MORAI 설정을 맞추고,
빌드·실행·검증하며, 수정할 설정의 소유 패키지를 찾을 수 있게 한다.

## 범위

- 루트 `README.md`를 현재 6개 패키지 기준의 온보딩 허브로 개편한다.
- 6개 패키지 README를 실제 `package.xml`, launch, YAML, 소스의 토픽 및
  파라미터와 대조한다.
- 틀리거나 오래된 설명, 누락된 실행 조건과 변경 절차를 수정한다.
- 실행 코드, launch, YAML, 센서 preset과 경로 데이터는 변경하지 않는다.

## 루트 README 구성

1. 프로젝트 목적과 현재 구현 범위
2. Mermaid 전체 아키텍처 및 데이터 흐름
3. 패키지별 책임과 의존 관계
4. 좌표계와 TF 구조
5. 저장소 준비, 의존성, install 기반 빌드
6. MORAI 센서 preset과 빈 시험 시나리오
7. 전체 센서 실행과 GPS+IMU localization/path 시험 실행
8. 핵심 토픽 계약
9. 설정값별 소유 패키지와 함께 수정할 파일
10. 기능별 검증 순서와 대표적인 문제 진단
11. 패키지 README 및 상세 문서 링크

루트 README는 빠른 온보딩에 필요한 현재값과 진입점을 제공하고, 전체 파라미터
목록이나 내부 구현은 패키지 README로 연결한다.

## 패키지 README 기준

각 패키지 README는 가능한 범위에서 다음 순서를 따른다.

1. 역할과 담당하지 않는 범위
2. 노드 또는 데이터 흐름
3. 입력·출력 토픽과 frame
4. 주요 config 및 수정 위치
5. 실행 방법
6. 검증과 문제 진단
7. 변경 시 함께 맞출 파일
8. 현재 제한과 향후 교체 지점

패키지 고유 정보는 해당 README에만 상세히 기록한다. 루트와 패키지 문서에
현재값이 모두 필요한 경우 패키지 README를 상세 기준으로 삼는다.

## 문서가 반영할 현재 아키텍처

```text
MORAI Camera/GPS/IMU UDP
  -> morai_udp_bridge
  -> /image/*, /sensors/gps/fix, /sensors/imu/data

MORAI VLP-16 UDP
  -> 공식 Velodyne driver
  -> /sensors/lidar/packets -> /lidar3D

GPS + IMU
  -> morai_localization
  -> /localization/pose, /localization/odometry, map TF

전역경로 + localization pose
  -> morai_path_manager
  -> /global_path, 전방 20 pose /local_path

description/TF + localization/path/LiDAR topic
  -> morai_visualization
  -> RViz

morai_bringup
  -> 위 패키지 launch와 config 조합
```

차량 제어기는 아직 구현되지 않았으며 `/local_path`는 향후 제어기 또는 다른
경로 소비자가 사용할 인터페이스로 설명한다.

## 검증

- 모든 Markdown 상대 링크가 실제 파일을 가리키는지 확인한다.
- README에 적힌 패키지명, launch, config, 토픽, frame과 현재값을 저장소에서
  기계적으로 대조한다.
- 오래된 “4개 패키지”, “GPS-only localization”, “미확정 센서 장착값” 표현이
  남지 않았는지 검색한다.
- 문서 변경 후 `catkin_make install`을 실행해 문서 변경이 빌드를 깨지 않는지
  확인한다.
- diff에 README와 이 명세 외 실행 코드·설정 변경이 없는지 확인한다.

## 완료 조건

- 신규 팀원이 루트 README만으로 권장 실행 명령까지 도달할 수 있다.
- 어느 설정을 어느 패키지에서 바꿔야 하는지 찾을 수 있다.
- 세부 동작은 각 패키지 README로 이어지며 서로 모순되지 않는다.
- 변경 내용을 `main`에 커밋하고 기존 `origin/main`에 반영한다.
