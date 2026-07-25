# README 온보딩 문서 개편 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 신규 팀원이 현재 MORAI ROS1 워크스페이스의 구조를 빠르게 이해하고 빌드·실행·검증·설정 변경 위치를 찾을 수 있도록 README를 최신화한다.

**Architecture:** 루트 README는 전체 흐름과 빠른 시작을 제공하는 온보딩 허브로 사용한다. 세부 토픽, 파라미터와 변경 절차는 책임을 소유한 6개 패키지 README에서 관리하며 코드·launch·YAML을 사실의 기준으로 삼는다.

**Tech Stack:** Markdown, Mermaid, ROS1 Noetic, catkin, Git

## Global Constraints

- 모든 사용자 대상 설명은 한글로 작성한다.
- 결론 우선, 짧은 문단, 표와 코드 블록을 사용해 읽기 쉽게 만든다.
- 루트와 패키지 README 사이의 설명과 파라미터 중복을 최소화한다.
- 실행 코드, launch, YAML, URDF, 센서 preset과 경로 파일은 변경하지 않는다.
- 현재 install 기반 실행 방식과 6개 패키지 구조를 기준으로 한다.

---

### Task 1: 루트 README를 온보딩 허브로 개편

**Files:**
- Modify: `README.md`

**Interfaces:**
- Consumes: 6개 패키지 README, `morai_bringup` launch, 현재 센서 preset과 시험 시나리오
- Produces: 신규 팀원이 사용할 전체 구조·빠른 시작·설정 탐색 진입점

- [ ] **Step 1: 오래된 설명을 제거하고 현재 범위를 명시**

현재 4개 패키지와 미확정 센서값 설명을 제거하고 다음을 명시한다.

- 6개 패키지 구성
- Camera/GPS/IMU UDP bridge와 공식 Velodyne driver
- GPS+IMU direct localization
- 전역경로 및 전방 20 pose local path
- path/LiDAR RViz 시각화
- 차량 제어기는 아직 미구현

- [ ] **Step 2: Mermaid 아키텍처와 TF 구조 작성**

MORAI 입력부터 ROS 토픽, localization, path manager와 visualization까지 흐름을
한 개 Mermaid 그래프로 작성한다. TF는
`map -> base_footprint -> base_link -> sensor_link`로 별도 표기한다.

- [ ] **Step 3: 빠른 시작과 MORAI 준비 항목 작성**

다음 명령을 install 기준 권장 절차로 제공한다.

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make install
source ~/catkin_ws/install/setup.bash
roslaunch morai_bringup gps_localization_path_test.launch
```

MORAI에서 `0725demo.json` 센서 preset과
`2026_molit_path_start_empty.json` 시나리오를 불러오는 위치를 연결한다.

- [ ] **Step 4: 패키지·핵심 토픽·설정 소유권 표 작성**

루트에는 대표 토픽과 수정할 패키지만 적고 전체 파라미터 표는 반복하지 않는다.

- [ ] **Step 5: 루트 README diff 검토**

Run: `git diff --check -- README.md`

Expected: 출력 없음.

- [ ] **Step 6: 루트 README 커밋**

```bash
git add README.md
git commit -m "docs: rewrite workspace onboarding guide"
```

### Task 2: 6개 패키지 README를 구현과 대조해 보정

**Files:**
- Modify if needed: `src/ioniq5_description/README.md`
- Modify if needed: `src/morai_udp_bridge/README.md`
- Modify if needed: `src/morai_localization/README.md`
- Modify if needed: `src/morai_path_manager/README.md`
- Modify if needed: `src/morai_visualization/README.md`
- Modify if needed: `src/morai_bringup/README.md`

**Interfaces:**
- Consumes: 각 패키지의 `package.xml`, `launch/`, `config/`, 구현 소스
- Produces: 패키지 책임·토픽·설정·실행·검증의 상세 기준 문서

- [ ] **Step 1: 패키지별 사실 대조**

각 README의 패키지명, launch 파일, config 파일, 노드, 토픽, frame, port,
센서 위치와 주기를 `rg`로 실제 파일에서 확인한다.

- [ ] **Step 2: 틀린 설명과 중요한 누락만 수정**

기존 README를 불필요하게 확장하지 않는다. 다음 공통 순서를 우선한다.

1. 역할과 비담당 범위
2. 데이터 흐름과 입출력
3. 설정 및 실행
4. 검증과 변경 주의사항

- [ ] **Step 3: 문서 간 중복과 모순 검색**

Run:

```bash
rg -n '네 패키지|4개 패키지|미확정.*GPS|GPS.*단독.*localization|devel/setup.bash' \
  README.md src/*/README.md
```

Expected: 현재 구조와 모순되는 표현 없음. 단독 시험을 정확히 설명하는
`gps_utm_projector` 문맥은 허용한다.

- [ ] **Step 4: 패키지 README diff 검토**

Run: `git diff --check -- src/*/README.md`

Expected: 출력 없음.

- [ ] **Step 5: 패키지 README 커밋**

변경된 README 경로만 명시적으로 stage한다.

```bash
git add src/ioniq5_description/README.md \
  src/morai_udp_bridge/README.md \
  src/morai_localization/README.md \
  src/morai_path_manager/README.md \
  src/morai_visualization/README.md \
  src/morai_bringup/README.md
git commit -m "docs: align package guides with current workspace"
```

변경이 필요 없는 README는 commit 대상에서 제외한다.

### Task 3: 문서·빌드 검증과 GitHub 반영

**Files:**
- Verify: `README.md`
- Verify: `src/*/README.md`
- Verify: 저장소 내 Markdown 상대 링크 대상

**Interfaces:**
- Consumes: Task 1과 Task 2의 문서 변경
- Produces: 검증된 `main` 및 `origin/main`

- [ ] **Step 1: Markdown 상대 링크 검사**

Python 표준 라이브러리로 README의 로컬 Markdown 링크를 추출해 각 README 기준
상대 경로에 대상 파일이 존재하는지 검사한다. HTTP 링크와 문서 내부 anchor는
제외한다.

- [ ] **Step 2: 빌드 및 설치 확인**

Run:

```bash
source /opt/ros/noetic/setup.bash
catkin_make install
```

Expected: exit code 0.

- [ ] **Step 3: 변경 범위와 상태 확인**

Run:

```bash
git diff --check
git status --short --branch
git log --oneline --decorate -5
```

Expected: README와 설계·계획 문서만 변경 이력에 포함되고 작업 트리는 깨끗함.

- [ ] **Step 4: GitHub에 push**

VS Code의 `Sync Changes` 또는 `Push`로 `main`을 `origin/main`에 반영한다.
브라우저 인증이 필요하면 VS Code 내장 GitHub 인증을 사용한다.

- [ ] **Step 5: 원격 추적 상태 확인**

Run: `git status --short --branch`

Expected: `## main...origin/main`이며 ahead/behind 표시 없음.
