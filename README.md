# Control

## 프로젝트 소개

`Control`은 ROS 2 Humble 기반 차량 제어 소프트웨어와 STM32G4 펌웨어를 함께
관리하는 저장소입니다. 공통 ROS 2 메시지 계약은 별도 저장소
`kaiev26_msgs`에 있으며, 두 저장소를 같은 ROS 2 워크스페이스의 `src/` 아래에
배치해 함께 빌드할 수 있습니다.

이 문서는 프로젝트의 입구와 문서 지도 역할을 합니다. 펌웨어 통신 규약과 안전
동작의 상세 내용은 아래의 기준 문서를 따릅니다.

## 시스템 구성

실차 구성은 세 개의 제어 주체로 나뉩니다.

- STM_A: 가속 페달·모드·기어 입력과 EZkontrol 구동 모터 제어
- STM_B: 세 물리 E-stop 입력, 브레이크 서보, 최종 브레이크 요청 결정
- Host / Control PC: 상위 ROS 2 제어 명령 전달, KEYA 조향 모터 제어, 상태 통합

두 STM은 각각 LPUART1 115200 baud로 별도의 micro-ROS Agent에 연결되어 ROS 2
graph에 참여합니다. 실차 없이 이 구성을 시험할 때는 STM_TEST 보드가 STM_A/B의
물리 입출력과 EZkontrol·KEYA CAN을 대신합니다.

전체 데이터 흐름과 책임 경계는
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)를 참고하십시오.

## 저장소 구조

```text
Control/
├── docs/                            # 시스템·인터페이스·안전 기준 문서
├── firmware/
│   ├── STM_A/                       # 구동 및 차량 입력 펌웨어
│   ├── STM_B/                       # E-stop 및 브레이크 펌웨어
│   ├── STM_TEST/                    # STM_A/B 입출력과 EZ/KEYA HIL emulator
│   ├── micro_ros_agent/             # vendored micro-ROS Agent
│   └── tools/                       # micro-ROS 라이브러리 재생성 스크립트
├── software/
│   └── src/
│       ├── lateral_ctrl/            # KEYA 조향 액추에이터 제어
│       ├── command_governor/        # /planning/command → /vehicle/command 단일 발행자
│       ├── vehicle_state_publisher/ # status 3종을 /vehicle/state로 통합
│       ├── control_launch/          # 실차 구동 통합 launch (vehicle.launch.py)
│       ├── kaiev26_hil/             # Web/PyQt HIL, Virtual LCD, scenario, vcan SIL
│       ├── vehicle_test_scenarios/  # /planning/command 실차 시험 시나리오
│       ├── control_gui_common/      # HIL/dashboard 공용 Qt runtime helper
│       ├── visualization/           # read-only GUI, diagnostics, Foxglove bridge
│       └── gazebo/                  # PID·SILS 시뮬레이션 자원
├── host/                            # 차량 PC 호스트 설정 (can0 systemd-networkd)
└── archive/                         # 과거 구현 보관소, 빌드 대상 아님
```

## 주요 구성요소

### firmware

| 구성요소 | 현재 역할 |
|---|---|
| `firmware/STM_A` | `/vehicle/command`의 속도·브레이크 명령, 물리 입력, EZkontrol CAN, `/drive/status` |
| `firmware/STM_B` | `/vehicle/command`의 브레이크 명령, 물리 E-stop, 브레이크 PWM, `/brake/status`, CAN dashboard 송신 |
| `firmware/UNO_R4_DASHBOARD` | STM_B dashboard CAN 수신, LCD 렌더링, Web 가상 LCD용 USB 적용 상태 보고 |
| `firmware/STM_TEST` | mode·gear·E-stop·pedal 주입, brake PWM 관측, EZ/KEYA CAN emulator와 Plant |
| `firmware/micro_ros_agent` | vendored micro-ROS Agent. 각 STM의 serial transport를 ROS 2 graph에 연결 |

### software/src

| 구성요소 | 현재 역할 |
|---|---|
| `lateral_ctrl` | `/vehicle/command`와 STM 상태를 사용한 KEYA 조향 모터 제어 및 `/steering/status` |
| `command_governor` | `/planning/command`를 정합성 게이트 후 `/vehicle/command`로 재발행하는 유일한 발행자. 현재 패스스루이며 형상(속도 상한·램프·축간 협조)은 이후 단계 |
| `vehicle_state_publisher` | 액추에이터 status 3종을 `/vehicle/state`로 통합 |
| `control_launch` | 실차 구동 스택 통합 `vehicle.launch.py` (Agent 2개 + 거버너 + 조향 + 상태 통합) |
| `kaiev26_hil` | STM_TEST bridge, React/PyQt HIL, UNO Virtual LCD, scenario, software-only `vcan` simulator |
| `vehicle_test_scenarios` | production Planning 대신 `/planning/command`를 발행하는 실차 직진·조향·속도·제동 시험 시나리오 |
| `control_gui_common` | HIL GUI와 vehicle dashboard가 공유하는 PyQt5/Qt runtime helper |
| `visualization` | read-only 차량 상태 GUI, `/diagnostics` 요약과 Foxglove bridge launch |
| `gazebo` | PID 및 SILS 관련 시뮬레이션 자원 |

## Web HIL

브라우저의 전역 **Control Dock**로 Mode/Gear/Pedal/E-stop을 조작하면서
구동·조향·브레이크 관측 페이지를 전환할 수 있습니다. 벤치 개요, CAN 진단·수신 주기,
시나리오 결과와 UNO의 **가상 LCD**도 제공합니다. 화면의 속도는 km/h,
조향은 degree이며 ROS의 m/s·rad 계약은 유지합니다. 기존 PyQt GUI도 유지합니다.

Web 의존성 설치, 프론트엔드·공통 LCD renderer 빌드, 전체 벤치 실행은
[Web Console 운용 문서](software/src/kaiev26_hil/WEB_CONSOLE.md)를 따릅니다.
기존 HIL launch와 Web 전체 벤치를 중복 실행하지 않습니다.

## 빠른 빌드

권장 워크스페이스 배치는 다음과 같습니다.

```text
~/KAI_ws/
└── src/
    ├── Control/
    └── kaiev26_msgs/
```

ROS 2 패키지는 워크스페이스 루트에서 빌드합니다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/KAI_ws/install/setup.bash
```

STM 펌웨어 빌드와 보드별 micro-ROS Agent 실행 절차는
[`firmware/README.md`](firmware/README.md)를 따릅니다.

실차 구동 스택은 `control_launch`의 통합 launch로 실행합니다. **처음부터
끝까지의 절차는 [실차 운용 절차](docs/VEHICLE_RUNBOOK.md)를 따라 하십시오.**
Planning은
포함되지 않으며(별도 실행, `/planning/command` 발행), Planning이 없으면
안전한 대기 상태(STM_A HALTED, 조향 WAIT_CMD)로 섭니다. 조향은 기본
armed이므로 전원 인가 전에 배선과 E-stop을 먼저 확인하십시오. 관찰
GUI(Foxglove·dashboard)는 별도로 켭니다. `/diagnostics` 요약은 구동 스택에
포함되어 있습니다. production Planning 없이 실차 명령 경로를 시험할 때는
`vehicle_test_scenarios`를 사용합니다.

```bash
ros2 launch control_launch vehicle.launch.py            # 실차 구동 스택
ros2 launch visualization foxglove_bridge.launch.py     # 관찰 (선택, 별도)
```

can0는 launch 전에 올라와 있어야 합니다. `host/systemd/`의 networkd 설정을
호스트에 1회 설치하면 부팅·USB 재연결 시 자동 준비됩니다
([HIL runbook 10.1](docs/HIL_RUNBOOK.md#101-socketcan-준비)).

실차 장치(`/dev/stm_a`, `/dev/stm_b`)에서의 최종 확인은 아직이며, HIL
bench에서 동형 구조로 검증했습니다. 그 외 문서화된 실행 절차는 HIL
bench([HIL runbook](docs/HIL_RUNBOOK.md)), 조향 단독
실행([lateral_ctrl](software/src/lateral_ctrl/README.md)), 거버너 단독
실행([command_governor](software/src/command_governor/README.md))입니다.
과거의 종방향 패키지 골격 `longitudinal_ctrl`은 노드가 채워진 적 없이
`command_governor`로 대체되었습니다. 종방향 하드 한도는 계속 STM_A 펌웨어에
있습니다.

## 문서

### 기준 문서

시스템 전체에 적용되는 계약과 설계 기준입니다. 코드와 어긋나면 이 문서를 함께
갱신합니다.

| 문서 | 내용 |
|---|---|
| [Vehicle runbook](docs/VEHICLE_RUNBOOK.md) | 실차 운용 절차 — 구동·확인·정지와 보드 교체 |
| [Architecture](docs/ARCHITECTURE.md) | 전체 시스템 구성, 역할과 데이터 흐름 |
| [Interfaces](docs/INTERFACES.md) | ROS 2, STM 간 UART, EZkontrol·KEYA CAN 계약 |
| [Safety](docs/SAFETY.md) | E-stop, timeout, fail-safe와 복구 동작 |
| [Hardware documents](docs/hardware/README.md) | 구동·조향 하드웨어 제조사 매뉴얼과 통신 프로토콜 |

### 운용 절차

| 문서 | 내용 |
|---|---|
| [Firmware](firmware/README.md) | 공통 STM 빌드, 고정 장치 이름과 micro-ROS Agent 실행 |
| [HIL runbook](docs/HIL_RUNBOOK.md) | SSH/X11부터 Agent·GUI·시험·종료까지 전체 HIL 운용 절차 |

### 구성요소별 상세

| 문서 | 내용 |
|---|---|
| [STM_A](firmware/STM_A/README.md) | STM_A 내부 구조 및 동작 |
| [STM_B](firmware/STM_B/README.md) | STM_B 내부 구조 및 동작 |
| [STM_TEST](firmware/STM_TEST/README.md) | HIL emulator의 pin map, UART protocol과 Plant |
| [lateral_ctrl](software/src/lateral_ctrl/README.md) | KEYA 조향 패키지 사용법과 상태 머신 |
| [kaiev26_hil](software/src/kaiev26_hil/README.md) | STM_TEST bridge, GUI, protocol과 scenario 상세 |
| [vehicle_test_scenarios](software/src/vehicle_test_scenarios/README.md) | 실차 직진·조향 sweep·속도 step/ramp·제동 시험 입력 |
| [control_gui_common](software/src/control_gui_common/README.md) | HIL GUI와 dashboard 공용 Qt runtime 처리 |
| [visualization](software/src/visualization/README.md) | 관찰용 GUI와 Foxglove 사용법 |

## 개발 규칙

### 파일 배치

- ROS 2 패키지는 `software/src/` 아래에 둡니다.
- Gazebo 코드는 `software/src/gazebo/` 아래에 둡니다.
- 전체 제어 launch는 `software/src/control_launch/launch/` 아래에 둡니다.
- 패키지 단독 시험 launch는 해당 패키지의 `launch/` 아래에 둡니다.
- 주기·timeout·핀·통신 ID·안전 조건을 바꿀 때는 위 기준 문서를 함께 갱신합니다.

### 커밋과 브랜치

- 개인 작업은 각자 브랜치에서 진행합니다.
- `build/`, `install/`, `log/`, rosbag, MCAP, 분석 CSV와 plot 산출물은
  커밋하지 않습니다.
- 커밋 전에 `git status`와 `git diff`로 변경 범위를 확인하고 빌드 결과를
  확인합니다.
- 다른 작업자의 branch를 덮어쓰거나 force push해야 할 때는 먼저 합의합니다.

### PR 작성 형식

PR 본문은 다음 형식을 사용합니다.

```markdown
## **브랜치**

## **주요 변경**

-
-

## **확인한 부분**

-

## **확인 필요한 부분**

-
```
