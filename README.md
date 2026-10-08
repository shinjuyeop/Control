# Control

## 프로젝트 소개

`Control`은 ROS 2 Humble 기반 차량 제어 소프트웨어와 STM32G4·UNO R4·RP2350 펌웨어를 함께
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

Arduino UNO R4 Minima는 STM_B의 Dashboard CAN을 받아 LCD를 표시하고 local touch
navigation을 처리하는 독립 HMI입니다. 차량 command consumer나 safety controller는
아니며, HIL에서만 USB Virtual LCD/Touch에 연결됩니다.
XIAO RP2350은 INA228 기반 LV 배터리 계측·Ah/Wh 적산·SOC 저장을 담당합니다.
STM_B의 UART4로 상태를 전달하며, STM_B가 ROS와 UNO 계기판으로 중계합니다.

두 STM은 각각 LPUART1 460800 baud로 별도의 micro-ROS Agent에 연결되어 ROS 2
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
│   ├── UNO_R4_DASHBOARD/            # Dashboard CAN·LCD·FT6336U touch HMI
│   ├── XIAO_RP2350_LV_GAUGE/        # INA228 LV 배터리 적산계·정비 TUI
│   ├── common/                      # STM 공통 주기·성능 계측
│   ├── KEYA/                        # 조향 드라이버 설정·프로토콜 자료
│   ├── micro_ros_agent/             # vendored micro-ROS Agent
│   └── tools/                       # micro-ROS 라이브러리 재생성 스크립트
├── software/
│   └── src/
│       ├── mpc_common/              # 두 MPC가 공유하는 C++/OSQP QP 계산
│       ├── steering_mpc/            # 식별 기반 C++ 조향 토크 MPC
│       ├── path_mpc/                # 식별 기반 C++ 경로 MPC
│       ├── longitudinal_mpc/         # 식별 기반 C++ 상전류 MPC와 분석 scripts
│       ├── lateral_ctrl/            # KEYA 조향 액추에이터 제어
│       ├── command_governor/        # /planning/command → /vehicle/command 단일 발행자
│       ├── vehicle_state_publisher/ # 차량 상태·wheel twist·diagnostics 발행
│       ├── control_launch/          # 실차 구동 통합 launch (vehicle.launch.py)
│       ├── control_tuning/          # 공통 제어 파라미터 조회·편집·저장 TUI
│       ├── control_tools/           # 시험·측정·독립 MCAP 녹화·분석
│       ├── vehicle_test_scenarios/  # /planning/command 실차 시험 시나리오
│       ├── vehicle_console/         # 통합 Console: Vehicle 관측 / HIL 시험 / 공용 Web
│       └── foxglove/                # Foxglove bridge와 engineering layout
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
| `firmware/XIAO_RP2350_LV_GAUGE` | INA228 측정·Ah/Wh·SOC·저장, STM_B UART4 및 USB 정비 |
| `firmware/STM_TEST` | mode·gear·E-stop·pedal 주입, brake PWM 관측, EZ/KEYA CAN emulator와 Plant |
| `firmware/common` | STM 공통 주기 실행과 runtime 성능 계측 |
| `firmware/KEYA` | 조향 드라이버 설정 도구와 프로토콜 분석 자료 |
| `firmware/micro_ros_agent` | vendored micro-ROS Agent. 각 STM의 serial transport를 ROS 2 graph에 연결 |

### software/src

| 구성요소 | 현재 역할 |
|---|---|
| `lateral_ctrl` | `/vehicle/command`와 STM 상태를 사용한 KEYA 조향 모터 제어 및 `/steering/status` |
| `command_governor` | `/planning/command`를 정합성 게이트 후 `/vehicle/command`로 재발행하는 유일한 발행자. 현재 패스스루이며 형상(속도 상한·램프·축간 협조)은 이후 단계 |
| `vehicle_state_publisher` | status 3종을 `/vehicle/state`, `/vehicle/wheel_twist`, `/diagnostics`로 변환 |
| `control_launch` | 실차 구동 스택 통합 `vehicle.launch.py` (Agent 2개 + 거버너 + 조향 + 상태 통합) |
| [`mpc_common`](software/src/mpc_common/README.md) | 두 MPC가 공유하는 C++/OSQP QP 계산 |
| [`steering_mpc`](software/src/steering_mpc/README.md) | 조향 토크 MPC와 실차 bag 식별·비교 |
| [`path_mpc`](software/src/path_mpc/README.md) | 경로 MPC와 폐루프 응답 식별·경로 평가 |
| `longitudinal_mpc` | PC 목표속도→상전류 MPC, STM_A C PI 참조, MCAP 식별·비교 |
| `control_tuning` | P·D 명령 전류 단위 편집, 출력·운용·timeout 설정, 저장 후 구동 스택 재시작으로 적용 |
| `control_tools` | 조향 벤치·측정, 독립 MCAP 녹화, 공통 기록 설정·파라미터 수집과 분석 |
| `vehicle_test_scenarios` | production Planning 대신 `/planning/command`를 발행하는 실차 입력 응답·K-City 경로 추종 TUI 및 headless 시험 시나리오 |
| `vehicle_console` | KAI Vehicle Console: 단일 frontend·공용 Web/ROS/runtime, Vehicle read-only 및 HIL backend |
| `foxglove` | Foxglove bridge와 engineering layout |

## 공통 제어 설정

`ros2 run control_tuning control_tuning_tui`에서 공통 파라미터를 편집합니다.
`S`는 선택한 프리셋을 저장하고, 해당 프리셋으로 구동 스택을 다시 실행할 때 반영합니다.
TUI는 현재값을 조회하며 실행 중인 제어값을 직접 변경하지 않습니다. 시나리오의 속도·진폭·주기는 기존 시나리오 TUI에서 수정합니다.
[Control Tuning 사용법](software/src/control_tuning/README.md)을 참고하십시오.

Control Tuning은 상단에서 주행·공중 프리셋을 먼저 선택한 뒤 편집합니다.
`S`로 조건별 YAML에 프리셋을 저장하고, `P`로 다른 프리셋을 선택합니다.
시작 설정과 프리셋은 `software/src/control_tuning/config/`에 저장해 Git으로 함께 관리합니다.
기본 구동 launch는 주행 프리셋을 사용하며, 다른 조건은 `tuning_preset:=airborne`으로 선택합니다. 실제 시험 조건은 시나리오 TUI 오른쪽의
`시험 조건`에서 Enter → ↑↓ → Enter로 별도로 선택합니다. 녹화에는 실행 노드에서
조회한 실제 제어 파라미터와 시험 중 변경 이벤트, 토크 상세 토픽도 함께 남깁니다.

## KAI Vehicle Console

HIL profile의 전역 **Control Dock**로 Mode/Gear/Pedal/E-stop을 조작하면서
구동·조향·브레이크 관측 페이지를 전환할 수 있습니다. 벤치 개요, CAN 진단·수신 주기,
시나리오 결과와 UNO의 **가상 LCD**도 제공합니다. 화면의 속도는 km/h,
조향은 degree이며 ROS의 m/s·rad 계약은 유지합니다.

`vehicle_console` 한 패키지 안에서 frontend와 공용 계층을 HIL과 Vehicle이 공유합니다. **Vehicle · READ ONLY**는 제어기나 HIL을
초기화하지 않고 차량 토픽만 구독하며, 서버에서 명령 입력을 거부합니다.
`ros2 launch vehicle_console vehicle.launch.py`로 시작합니다(기본 8766).
HIL은 `ros2 launch vehicle_console hil.launch.py`를 사용합니다(8767).
[Console 설치·운용](software/src/vehicle_console/README.md),
[Qt 대체 및 archive 판단](software/src/vehicle_console/CONSOLE_PARITY.md)을 참고하십시오.

Web 의존성 설치, 프론트엔드·공통 LCD renderer 빌드, 전체 벤치 실행은
[Web Console 운용 문서](software/src/vehicle_console/HIL_WEB.md)를 따릅니다.
기존 HIL launch와 Web 전체 벤치를 중복 실행하지 않습니다.

## 빠른 빌드

권장 워크스페이스 배치는 다음과 같습니다.

```text
~/KAI_ws/
└── src/
    ├── Control/
    └── kaiev26_msgs/
```

ROS 2 패키지는 워크스페이스 루트에서 빌드합니다. 위 트리는 Control과 메시지의
기본 배치입니다. 실차 통합 launch에는 별도 `kaiev26_description`과 그 의존성이
필요하고, 센서 드라이버와 TUI 경로 시험의 `kaiev26_decision`,
`kaiev26_motion_control`, `eskf_localization`도 해당 시험에 맞게 준비해야 합니다.
`archive/`는 기본 colcon 탐색에서 제외됩니다.

```bash
cd ~/KAI_ws
si
colcon build --symlink-install
si
```

STM 펌웨어 빌드와 보드별 micro-ROS Agent 실행 절차는
[`firmware/README.md`](firmware/README.md)를 따릅니다.

실차 구동 스택은 `control_launch`의 통합 launch로 실행합니다. **처음부터
끝까지의 절차는 [실차 운용 절차](docs/VEHICLE_RUNBOOK.md)를 따라 하십시오.**
Planning은
포함되지 않으며(별도 실행, `/planning/command` 발행), Planning이 없으면
AUTO에서는 명령 timeout으로 STM_A가 HALTED, 조향이 WAIT_CMD로 대기합니다.
MANUAL의 물리 페달 경로는 별도입니다. 조향은 기본
armed이므로 전원 인가 전에 배선과 E-stop을 먼저 확인하십시오.
관측 화면(Foxglove·Vehicle Console)은 별도로 켭니다. `/diagnostics` 요약은 구동 스택에
포함되어 있습니다. production Planning 없이 실차 명령 경로를 시험할 때는
`vehicle_test_scenarios`를 사용합니다.

```bash
ros2 launch control_launch vehicle.launch.py            # 실차 구동 스택
ros2 launch foxglove foxglove_bridge.launch.py          # 관찰 (선택, 별도)
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

## 시험 기본값과 이전 폴더

실차 시나리오 TUI와 headless YAML의 기본 속도는 모두 `0 km/h`, 조향 중심은
`0 deg`입니다. 조향 진폭은 별도 설정이므로 속도·중심이 0이어도 횡방향 시험은
조향 입력을 생성할 수 있습니다. 시험 속도·진폭을 확인한 뒤 Run을 시작합니다.
TUI는 `ros2 run vehicle_test_scenarios vehicle_test_scenario_runner`로 실행합니다.

| 이전 구성 | 현재 위치 또는 역할 |
|---|---|
| `can_lab` | `archive/teamkai_can_lab.py` |
| `visualization`의 Qt dashboard | `vehicle_console` Vehicle 화면으로 대체; 보관본 정리 |
| 상태·진단 / Web 관측 / Foxglove | `vehicle_state_publisher` / `vehicle_console` / `foxglove` |
| `kaiev26_hil` | `vehicle_console.hil`로 통합; 독립 패키지 보관본 정리 |
| PID/Gazebo 실험 | `archive/software/gazebo/` |
| `longitudinal_ctrl` | `archive/longitudinal_ctrl/`; 현재 명령 경로는 `command_governor` |

올해 대체 완료된 보관본과 복구 기준은 [archive 정리 기록](archive/README.md#2026년-대체-완료-보관본-정리)에 있습니다.

`git pull`은 ignored 캐시나 예전 `build/`, `install/`을 자동 정리하지 않습니다.
패키지 목록은 `colcon list`로, 실제 실행 대상은 `ros2 pkg prefix <package>`로
확인하고 차량 데스크탑에서 `cd ~/KAI_ws` 후 `si`로 현재 workspace 환경을 불러옵니다.
과거 중첩 workspace의 setup 파일을 섞어 읽으면 옛 패키지가 다시 선택될 수 있습니다.

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
| [시험·분석 기록](docs/reports/README.md) | 날짜별 시험 계획과 실측·구현 보고서 |
| [Hardware documents](docs/hardware/README.md) | 구동·조향 하드웨어 제조사 매뉴얼과 통신 프로토콜 |

### 운용 절차

| 문서 | 내용 |
|---|---|
| [Firmware](firmware/README.md) | 공통 STM 빌드, 고정 장치 이름과 micro-ROS Agent 실행 |
| [HIL runbook](docs/HIL_RUNBOOK.md) | Web 접속부터 Agent·시험·종료까지 전체 HIL 운용 절차 |

### 구성요소별 상세

| 문서 | 내용 |
|---|---|
| [control_launch](software/src/control_launch/README.md) | 실차 통합 launch, 재시작, 조이스틱 텔레옵 |
| [control_tuning](software/src/control_tuning/README.md) | 공통 파라미터 TUI, A 단위 P·D, 저장 파일·적용·재시작 |
| [Torque control](software/src/lateral_ctrl/TORQUE_CONTROL.md) | 현재 YAML과 코드 기본값, 조향 fault 복구 |
| [control_tools](software/src/control_tools/README.md) | 시험·측정·녹화·분석 도구 |
| [Logging](software/src/control_tools/LOGGING.md) | 독립 MCAP 기록과 TUI 기록의 구분, CSV 변환 |
| [UNO dashboard](firmware/UNO_R4_DASHBOARD/README.md) | native·AUTO CAN 표시와 touch |
| [LV 배터리 구성](docs/rp2350/lv_battery_gauge.md) | INA228·RP2350·STM_B·ROS·계기판 연결과 계측 경계 |
| [RP2350 펌웨어](firmware/XIAO_RP2350_LV_GAUGE/README.md) | 빌드·업로드·적산·영속 저장·정비 TUI |
| [Vehicle wiring](docs/vehicle-wiring/README.md) | 배선도 편집 |
| [STM_A](firmware/STM_A/README.md) | STM_A 내부 구조 및 동작 |
| [STM_B](firmware/STM_B/README.md) | STM_B 내부 구조 및 동작 |
| [STM_TEST](firmware/STM_TEST/README.md) | HIL emulator의 pin map, UART protocol과 Plant |
| [lateral_ctrl](software/src/lateral_ctrl/README.md) | KEYA 조향 패키지 사용법과 상태 머신 |
| [vehicle_test_scenarios](software/src/vehicle_test_scenarios/README.md) | 실차 직진·조향 sweep·속도 step/ramp·제동 시험 입력 |
| [vehicle_console](software/src/vehicle_console/README.md) | 통합 Console, Vehicle READ ONLY, HIL, 원격 관측 |
| [HIL backend](software/src/vehicle_console/HIL_BACKEND.md) | bridge, protocol, scenario와 vcan 상세 |
| [vehicle_state_publisher](software/src/vehicle_state_publisher/README.md) | 차량 상태·wheel twist·diagnostics 발행 |
| [foxglove](software/src/foxglove/README.md) | Foxglove bridge와 engineering layout |

## 개발 규칙

### 파일 배치

- ROS 2 패키지는 `software/src/` 아래에 둡니다.
- 구형 PID/Gazebo 실험 코드와 모델은 [`archive/software/gazebo`](archive/software/gazebo/README.md)에 보관하며 빌드에서 제외합니다.
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

AUTO 종방향 Speed/Torque PI 및 first-test calibration은 [구현·실측 보고서](docs/reports/2026-09-21_longitudinal_torque_pi.md)를 따릅니다.
식별 플랜트 feedforward 구조와 MANUAL 페달 envelope 공유는
[feedforward 기록](docs/reports/2026-09-29_longitudinal_feedforward.md)에서 시작했고,
EZkontrol 사인파(FOC) 기준 재구성(ABI v4: 회전 중 최소 전류의 방형파/사인파 값, 역회전 디바운스,
붕괴 감지)과 사인파 시험 계획은 [재구성 보고서](docs/reports/2026-10-01_ezkontrol_sine_rework.md)를 따릅니다.
회생제동(ABI v5: 회생 한도·속도/전압/충전전류 보호, 기본 꺼짐의 속도 PI 자동 회생), 기존 프리셋과 거버너 가·감속 slew 유지,
사인파 붕괴 감지 조정과 공중 회생 시험(R1·R2) 계획은 [회생 보고서](docs/reports/2026-10-02_regen_governor_rework.md)를 따릅니다.
공중 PI 자동 회생·feedforward 튜닝과 공중 프리셋 최종값은 [RBS 튜닝 보고서](docs/reports/2026-10-08_rbs_tuning_report.md)를 따릅니다.
