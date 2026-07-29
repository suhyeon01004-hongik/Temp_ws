# NVIDIA Dynamic Boost 및 Performance 프로필 자동 적용 설계

## 목표

ThinkPad P1 Gen 5에서 NVIDIA Dynamic Boost를 Linux 부팅 시 자동 활성화하고,
ThinkPad ACPI 플랫폼 프로필을 부팅 및 절전 복귀 후 `performance`로 자동 복원한다.

## 확인된 환경

- GPU: NVIDIA RTX A2000 8GB Laptop GPU
- NVIDIA 드라이버: 570.181
- 기본 GPU 전력 제한: 35W
- 드라이버/펌웨어가 보고한 최대 전력 제한: 60W
- `/proc/driver/nvidia/.../power`: `Notebook Dynamic Boost: Supported`
- Lenovo 공식 사양의 해당 GPU TGP: 35W, Dynamic Boost 2.0 지원
- 현재 ACPI 플랫폼 프로필: `balanced`
- 선택 가능한 프로필: `low-power balanced performance`
- MORAI 부하에서 GPU는 약 35W 전력 제한에 도달
- MORAI 부하에서 CPU는 약 97°C까지 상승하고 열 스로틀링 발생

## 선택한 방식

1. 설치된 NVIDIA 570 패키지가 제공하는 공식 D-Bus 정책과
   `nvidia-powerd.service` 템플릿을 시스템 위치에 설치한다.
2. `nvidia-powerd.service`를 enable/start하여 부팅 시 자동 실행한다.
3. 별도 oneshot systemd 서비스가
   `/sys/firmware/acpi/platform_profile`에 `performance`를 기록하도록 한다.
4. system-sleep hook으로 절전 복귀 후에도 `performance`를 복원한다.
5. GPU 전력 제한을 `nvidia-smi -pl 60`으로 직접 고정하지 않는다.
   Dynamic Boost가 SBIOS의 전체 전력·열 예산 안에서 CPU와 GPU 전력을
   동적으로 배분하도록 맡긴다.

## TGP 해석

현재 장치가 보고하는 절대 범위는 기본 35W에서 최대 60W이므로 숫자상 상한은
`+25W`이다. 그러나 60W는 Dynamic Boost의 보장 지속 전력이 아니라
드라이버가 보고한 허용 상한이다. Lenovo가 명시한 제품 TGP는 35W이며,
실제 추가 전력은 AC 연결 상태, CPU 전력, 온도, 펌웨어 정책에 따라 달라진다.
현재 CPU 열 스로틀링을 고려하면 MORAI에서 60W가 지속될 가능성은 낮다.

## 검증

- `systemctl is-enabled/is-active nvidia-powerd.service`
- `systemctl is-enabled/is-active platform-performance.service`
- `cat /sys/firmware/acpi/platform_profile`
- MORAI 실행 중 `nvidia-smi`로 current power limit, power draw, 온도 관찰
- `sensors`와 CPU throttle counter로 CPU 열 스로틀링 재확인

성공 기준은 서비스 두 개가 활성화되고 프로필이 `performance`이며,
MORAI 부하에서 시스템이 오류 없이 동작하는 것이다. Dynamic Boost 성공은
GPU가 반드시 60W에 도달하는 것으로 판단하지 않고, 부하에 따라 current power
limit이 35W보다 올라가는지와 프레임타임이 개선되는지를 함께 본다.

## 롤백

- `nvidia-powerd.service`를 disable/stop하고 설치한 unit 및 D-Bus 정책을 제거
- `platform-performance.service`를 disable/stop하고 sleep hook을 제거
- ACPI 플랫폼 프로필을 `balanced`로 복원
- `systemctl daemon-reload` 실행
