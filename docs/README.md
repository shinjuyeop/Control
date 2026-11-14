# Control 문서 지도

[프로젝트 개요](../README.md) · [Software](../software/README.md) · [Firmware](../firmware/README.md)

시스템 구조를 처음 볼 때는 아키텍처와 안전 동작을 먼저 읽고, 관심 있는 구성요소의 README와
시험 기록으로 이어가면 됩니다. 실행 절차와 날짜별 실험 결과는 별도 문서에서 관리합니다.

## 설계와 구현 읽기

| 알고 싶은 내용 | 시작 문서 |
|---|---|
| Host·STM_A·STM_B의 책임과 데이터 흐름 | [Architecture](ARCHITECTURE.md) |
| 오래된 명령·통신 단절·E-stop과 복구 조건 | [Safety](SAFETY.md) · [Command Governor](../software/src/command_governor/README.md) |
| ROS 메시지, 단위, UART frame과 CAN 계약 | [Interfaces](INTERFACES.md) |
| 구동·조향 제어기와 시험·관측 패키지 | [Software](../software/README.md) |
| 보드별 역할, FreeRTOS Task와 코드 진입점 | [Firmware](../firmware/README.md) |
| 제어 설정의 저장·적용·readback | [Control Tuning](../software/src/control_tuning/README.md) · [파라미터 연결표](CONTROL_TUNING_PARAMETERS.md) |

`ARCHITECTURE.md`·`INTERFACES.md`·`SAFETY.md`는 현재 시스템의 기준 문서입니다.
패키지별 README는 구성요소의 구현·실행을 설명하고, `reports/`는 해당 시점의 분석과 판단을 보존합니다.
과거 보고서의 설정값을 현재 프리셋이나 실장치 적용값으로 간주하지 않습니다.

## 검증 근거 읽기

| 구분 | 기록과 확인 범위 |
|---|---|
| 보드 HIL | [HIL runbook의 시험 기록](HIL_RUNBOOK.md#15-진행-상태와-남은-작업): 실제 MCU·입출력·CAN emulator 연결과 고장 주입 |
| 소프트웨어·Web | [Console 검증](../software/src/vehicle_console/CONSOLE_VALIDATION.md) · [Web HIL 검증](../software/src/vehicle_console/HIL_WEB.md#검증): 테스트와 당시 live 관측의 구분 |
| MCU 주기·통신 비용 | [Runtime metrics](../firmware/common/README.md): 계측 정의, baud 변경 전후 관측과 미측정 범위 |
| 제어 응답·모델·시험 계획 | [시험·분석 기록](reports/README.md): 공중 무부하·지상 주행·Gazebo 결과와 후속 계획 |

보고서의 숫자는 시험 날짜, 입력, 적용 파라미터와 함께 읽습니다. 원본 MCAP·분석 산출물 일부는
Git에 없으며, 재분석에는 보고서가 지정한 자료가 필요합니다. 새 실행 결과 없이 기존 기록의
검증 범위를 넓히거나 현재 버전의 시험 통과로 바꾸지 않습니다.

## 실행과 하드웨어

| 목적 | 문서 |
|---|---|
| 실차 환경 준비·구동·관측·정지 | [Vehicle runbook](VEHICLE_RUNBOOK.md) |
| 벤치 준비·Web 시험·종료 | [HIL runbook](HIL_RUNBOOK.md) |
| ROS 패키지 의존성과 빌드 | [Software 환경과 빌드](../software/README.md#환경과-빌드) |
| STM 빌드·고정 장치 이름·Agent·보드 교체 | [Firmware](../firmware/README.md) |
| 제조사 매뉴얼·통신 프로토콜 | [Hardware documents](hardware/README.md) · [EZkontrol 설정 메모](hardware/EZKONTROL_SETTINGS_NOTES.md) |
| 배선도 확인·편집 | [Vehicle wiring](vehicle-wiring/README.md) |
| LV 배터리 계측·ROS·계기판 연결 | [LV 배터리 구성](rp2350/lv_battery_gauge.md) · [RP2350 펌웨어](../firmware/XIAO_RP2350_LV_GAUGE/README.md) |
| 계기판 CAN·LCD·touch | [UNO R4 Dashboard](../firmware/UNO_R4_DASHBOARD/README.md) |
| 조향 드라이버 설정·프로토콜 | [KEYA](../firmware/KEYA/README.md) |

## 이전 구현

| 이전 구성 | 현재 위치 또는 역할 |
|---|---|
| `can_lab` | [보관된 CAN 도구](../archive/teamkai_can_lab.py) |
| `visualization` Qt dashboard | [Vehicle Console](../software/src/vehicle_console/README.md), [전환 판단](../software/src/vehicle_console/CONSOLE_PARITY.md) |
| 독립 `kaiev26_hil` | [vehicle_console.hil](../software/src/vehicle_console/HIL_BACKEND.md)로 통합 |
| PID/Gazebo 실험 | [archive/software/gazebo](../archive/software/gazebo/README.md), active build에서 제외 |
| `longitudinal_ctrl` 골격 | [archive/longitudinal_ctrl](../archive/longitudinal_ctrl/), 현재 명령 전달은 Governor, 종방향 제어는 STM_A 또는 선택한 Host 제어기 |

대체 완료된 보관본의 정리 시점과 Git 복구 기준은
[archive 정리 기록](../archive/README.md#2026년-대체-완료-보관본-정리)에 있습니다.

## 문서와 개발 규칙

- ROS 2 패키지는 `software/src/`, 전체 제어 launch는 `software/src/control_launch/launch/`에 둡니다.
- 패키지 단독 시험 launch는 해당 패키지의 `launch/`에 둡니다.
- 주기·timeout·핀·통신 ID·안전 조건을 바꾸면 관련 기준 문서도 함께 갱신합니다.
- 개인 작업은 각자 브랜치에서 진행하며 기존 작업자의 변경과 Git 이력을 보존합니다.
- `build/`, `install/`, `log/`, rosbag, MCAP, 분석 CSV·plot 산출물은 커밋하지 않습니다.
- 커밋 전에 `git status`·`git diff`로 변경 범위를 확인하고 해당 변경에 필요한 빌드·검사를 수행합니다.

<details>
<summary>기존 PR 작성 형식</summary>

```markdown
## **브랜치**

## **주요 변경**

-

## **확인한 부분**

-

## **확인 필요한 부분**

-
```

</details>
