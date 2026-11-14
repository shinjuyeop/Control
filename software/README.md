# Control software

[프로젝트 개요](../README.md) · [문서 지도](../docs/README.md) · [펌웨어](../firmware/README.md)

Host의 ROS 2 패키지는 `src/`에 있습니다. 상위 명령을 검증·전달하고 조향을 제어하며,
MCU 상태를 통합합니다. 시험 입력, 제어 설정, 관측 화면과 MCAP 분석 도구도 함께 관리합니다.

## 명령과 상태 경로

```text
Planning / 실차 시험 입력 / HIL 입력 중 하나
  └─ /planning/command
      └─ command_governor: 입력 검사, 선택형 속도 slew
          └─ /vehicle/command
              ├─ STM_A: 구동 목표와 Host brake
              ├─ STM_B: Host brake
              └─ lateral_ctrl: 조향 목표와 Host brake

/drive/status + /brake/status + /steering/status
  └─ vehicle_state_publisher → /vehicle/state, /vehicle/wheel_twist, /diagnostics
```

`ActuatorCommand`의 속도는 m/s, 조향각은 rad입니다. Console·TUI의 km/h·degree 표시와
구분합니다. 물리 mode·gear는 STM_A 입력이며 Host 명령에 포함되지 않습니다.
세 status의 합성과 각 제어기의 freshness 판정은 별도입니다.
[인터페이스 계약](../docs/INTERFACES.md)과 [안전 조건](../docs/SAFETY.md)을 함께 확인합니다.

## 구동 스택과 제어기

| 구성요소 | 책임과 읽을 내용 |
|---|---|
| [command_governor](src/command_governor/README.md) | 입력 timestamp·유한값 검사, 속도 명령 변화율 제한, `/vehicle/command` 발행. 입력 단절을 재발행으로 가리지 않음 |
| [lateral_ctrl](src/lateral_ctrl/README.md) | KEYA SocketCAN 소유, 조향 상태 머신과 Enable/Disable. [Torque 제어](src/lateral_ctrl/TORQUE_CONTROL.md)에서 보정각 PD·출력 한도·복구 조건 확인 |
| [STM_A 종방향 제어](../firmware/STM_A/README.md) | MCU의 Speed/Torque PI·feedforward·회생·외부 전류 경로. 최종 전류/RPM 한도와 구동 차단은 펌웨어가 적용 |
| [vehicle_state_publisher](src/vehicle_state_publisher/README.md) | 구동·브레이크·조향 상태, wheel twist, diagnostics와 AUTO 계기판 요약 발행 |
| [control_launch](src/control_launch/README.md) | Agent 2개, 차량 TF, Governor, 조향, 상태·진단과 STM_A 설정 적용을 묶는 실차 launch |

기본 차량 launch는 `longitudinal_controller:=firmware`, `torque_controller:=pd`를 사용합니다.
종방향 `pi`/`mpc` 선택 시 Host가 `/vehicle/drive_current_command`를 발행하고 STM_A가
외부 전류 모드로 수용합니다. 이 경로는 설정 readback 성공 후 시작합니다.
`sysid`에서는 시험 러너가 전류를 발행하므로 다른 전류 발행자와 함께 실행하지 않습니다.

### MPC와 모델 식별

종방향·조향·경로 MPC는 현재 실차 운용에 사용하지 않습니다.
실차 기록을 이용한 모델 식별과 Gazebo SITL 비교를 위한 구현입니다.

| 패키지 | 구현 범위 |
|---|---|
| [steering_mpc](src/steering_mpc/README.md) | 실차 bag의 토크 입력→보정 조향각 모델 식별, `lateral_ctrl` 안에서 선택하는 C++ 조향 MPC |
| [path_mpc](src/path_mpc/README.md) | 경로·차량 상태에서 목표 조향각 계산. 실행과 속도·제동 명령 결합은 별도 Decision 패키지가 담당 |
| [longitudinal_mpc](src/longitudinal_mpc/README.md) | 목표속도→상전류 MPC, STM_A C PI를 Host에서 재사용하는 비교 모드, MCAP 식별·평가 |
| [mpc_common](src/mpc_common/README.md) | `steering_mpc`와 `path_mpc`가 공유하는 OSQP QP 계산·제약·workspace 재사용 |

모델 식별, 시뮬레이션 제어 비교와 실차 폐루프 검증을 구분합니다.
[2026-09-24 보고서](../docs/reports/2026-09-24_lateral_mpc.md)는 실차 bag 예측과 Gazebo 비교를
기록하며, 실차 MPC 주행 결과는 포함하지 않습니다. 식별·튜닝·평가 데이터의 경계는
[SYSID → MPC 안내](src/mpc_common/docs/sysid_to_mpc.md)를 따릅니다.

## 시험·설정·관측 도구

| 패키지 | 역할 |
|---|---|
| [control_tuning](src/control_tuning/README.md) | 현재값 조회, 주행·공중 프리셋 편집·저장. `S`는 파일 저장이며 같은 프리셋으로 구동 스택을 재시작해야 적용 |
| [vehicle_test_scenarios](src/vehicle_test_scenarios/README.md) | 상위 Planning을 대신하는 실차 Step/Sine/Ramp·제동·식별·경로 시험 입력 |
| [control_tools](src/control_tools/README.md) | 독립 MCAP 녹화, 조향 측정, 종방향·조향 응답 분석. [기록 형식과 파라미터 수집](src/control_tools/LOGGING.md) |
| [vehicle_console](src/vehicle_console/README.md) | Vehicle 관측 backend와 HIL 시험 backend가 공유하는 Web 화면. Vehicle은 차량 명령을 발행하지 않음 |
| [foxglove](src/foxglove/README.md) | ROS graph 상세 관측용 bridge와 engineering layout |

실차 구동 `control_launch/vehicle.launch.py`와 관측 전용
`vehicle_console/vehicle.launch.py`는 역할이 다릅니다. HIL은
`vehicle_console/hil.launch.py`의 별도 backend이며, 전체 벤치에는 `start_bench:=true`가 필요합니다.
[HIL Web](src/vehicle_console/HIL_WEB.md)과 [backend 구조](src/vehicle_console/HIL_BACKEND.md)를 참고합니다.

시나리오의 기본 속도와 조향 중심은 0이지만 조향 진폭은 별도 값입니다.
시험 조건 표시(`driving`/`airborne`/`stationary`)는 기록용 정보이며 제어 프리셋을 바꾸지 않습니다.
분석에서는 저장 YAML만으로 실제 적용값을 추정하지 않고 Run의 파라미터 snapshot과 STM_A readback을 확인합니다.

## 환경과 빌드

기준 환경은 Ubuntu 22.04 / ROS 2 Humble입니다. 다음은 기본 워크스페이스 배치이며,
선택한 실행 구성의 외부 패키지와 각 패키지 README의 의존성을 먼저 준비해야 합니다.

```text
~/KAI_ws/src/
├── Control/
└── kaiev26_msgs/
```

| 실행 구성 | 추가 준비 |
|---|---|
| 공통 빌드·MCU 통신 | 별도 `kaiev26_msgs`, vendored micro-ROS Agent, MCU와 Host의 메시지 버전 일치 |
| 실차 통합 launch | `kaiev26_description`과 그 의존성, serial 장치, SocketCAN. [실차 runbook](../docs/VEHICLE_RUNBOOK.md) |
| Web·HIL 벤치 | Web Python 환경·Node.js frontend 빌드, STM_A/B/TEST 및 벤치 배선. [HIL runbook](../docs/HIL_RUNBOOK.md) |
| MPC·식별·경로 시험 | OSQP/Eigen/yaml-cpp 및 분석 의존성, 식별 모델·원본 bag. 시험에 따라 별도 Decision·Motion Control·localization·Simulation 패키지 |

의존성 준비 후 **워크스페이스 루트**에서 빌드합니다. 아래 예시는 Bash 기준이며,
기존 문서의 환경별 별칭 `si` 대신 명시적으로 ROS 환경을 불러옵니다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.bash
colcon list
colcon build --symlink-install
source install/setup.bash
```

STM ARM 빌드는 [Firmware](../firmware/README.md#stm-빌드)를 따릅니다.
Web frontend·native LCD renderer는 별도 빌드가 필요하며 colcon만으로 모두 준비되지는 않습니다.
실제 실행·관측·종료 순서는 [실차](../docs/VEHICLE_RUNBOOK.md) 또는 [HIL](../docs/HIL_RUNBOOK.md) 절차를 사용합니다.

## 검증 기록과 이전 구현

소프트웨어 검사 결과는 해당 시점의 [Console 검증 기록](src/vehicle_console/CONSOLE_VALIDATION.md)에,
차량 응답과 시험 계획은 [보고서 색인](../docs/reports/README.md)에 구분해 연결했습니다.
현재 checkout의 빌드 성공이나 실차 성능을 과거 검사 결과로 대신하지 않습니다.

구형 `longitudinal_ctrl`, Qt 화면과 독립 `kaiev26_hil`의 전환·복구 기준은
[문서 지도](../docs/README.md#이전-구현)와 [archive 기록](../archive/README.md)을 따릅니다.
`archive/`는 기본 colcon 탐색에서 제외됩니다. `git pull`은 예전 `build/`·`install/`을 정리하지
않으므로 `colcon list`와 `ros2 pkg prefix <package>`로 실제 선택된 패키지를 확인합니다.
