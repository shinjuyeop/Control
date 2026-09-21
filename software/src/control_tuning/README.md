# Control Tuning

실차와 시나리오에서 공통으로 사용하는 Host 조향·Governor 및 STM_A 종방향 설정 TUI다.
현재 적용값 / 편집값 / 선택한 프리셋 저장값을 비교한다. 시나리오의 속도·진폭·주기는
`vehicle_test_scenario_runner`에서 별도로 편집한다.

```bash
cd ~/KAI_ws
si
ros2 run control_tuning control_tuning_tui
```

코드 변경 뒤에는 `colcon build --symlink-install --packages-select control_tuning lateral_ctrl command_governor control_launch` 후 다시 `si` 한다.
Python 의존성은 rich/PyYAML, MCAP 분석은 ROS의 rosbag2_py/numpy다.
`--offline`은 파일 편집·저장만, `--preview`는 첫 화면 출력만 수행한다.
80열×24행 이상, 104열에서 저장값 열, 132열에서 native 이름 열을 표시한다.

## 분류와 적용

| 분류 | 항목 | 적용 |
|---|---|---|
| 1 조향응답 | P/D, 목표 도달 판정, 조향속도 필터 | A / S |
| 2 조향출력 | 최대·지속 전류, 고출력 시간·회복, 한계각 여유 | A / S |
| 3 구동응답 | speed/torque, 속도 Kp/Ki, deadband/hysteresis | MANUAL 정지·무전류에서 A / S |
| 4 구동출력 | 최대 전류, 적분 상한, 전류 상승·하강·재인가, RPM 상한 | MANUAL 정지·무전류에서 A / S |
| 5 명령검사 | 상위 주행 명령 유효시간 (Governor) | S 후 재시작 |
| 6 운용설정 | 조향 제어 방식, 조향 허용 최소 차속 | S 후 조향 재시작 |
| 7 수신제한 | 조향 명령·CAN·피드백·모드·차속 수신 제한 | S 후 재시작 |

프리셋 선택 후 위 7개 분류가 항상 한 행에 표시된다. ←→/Tab으로 이동하거나 1–7로
바로 선택한다. 선택된 분류는 대괄호와 강조색으로 표시한다. 전체 이름이 들어가도록
최소 터미널 폭은 80열이다.

하드웨어 연결, 조향 보정식, 제어 주기와 차량 운동 추정은 편집하지 않는다.
Governor에는 새 ramp를 추가하지 않았다. Decision이 기존 accel/decel ramp를 담당한다.
입력 허용 범위는 코드 계약이며 검증된 실차 tuning range가 아니다.

## 조작

| 키 | 동작 |
|---|---|
| ←→ / Tab, 1–7 | 프리셋/분류 이동 |
| ↑↓ / 휠 | 항목 선택, 검토 목록 이동 |
| Enter, +/− | 직접 입력/선택 전환, 증감 |
| S | 선택한 프리셋 YAML 저장 |
| A | 구동응답·구동출력에서는 STM_A 전체 config; 나머지 분류에서는 기존 조향 live 튜닝 적용 |
| P / 첫 항목에서 ↑ | 프리셋 선택으로 복귀 |
| L / B / X | 현재 적용값 / 기본값 / 프리셋 저장값을 편집란으로 가져오기 |
| R / F | 실행값 조회 / 파일 다시 읽기 |
| ? / Q / Esc | 도움말 / 종료 / 취소 |

편집·S 저장은 곧바로 차량을 변경하지 않는다. A→Enter는 선택한 제어기의 실행값을 바꾼다.
조향과 구동을 하나의 atomic transaction처럼 적용하지 않는다.
현재와 다음 실행 모두 바꾸려면 A와 S를 각각 사용한다. 명령검사·운용설정·수신제한은 재시작 항목이다.

## 두 프리셋과 schema v2

`airborne`과 `driving`만 유지한다. 접지 정지는 별도 calibration이 아니라 시나리오의
`stationary` 시험 조건이다. 선택한 시험 조건은 녹화 annotation이며 제어값을 바꾸지 않는다.

v2는 `lateral`, `longitudinal`, `governor`, `timeouts`로 구분한다.
기존 lateral 값은 그대로 이관했다. 종방향 값은 **MCAP 기반 first-test candidate**다.

| 값 | airborne | driving |
|---|---:|---:|
| mode | torque | torque |
| Kp / Ki | 8 / 1 | 180 / 12 |
| max / integrator A | 8 / 3 | 480 / 120 |
| slew up / down A/s | 40 / 160 | 2400 / 4800 |
| deadband / hysteresis m/s | 0.05 / 0.12 | 0.05 / 0.12 |
| reapply band A / slew A/s | 2 / 10 | 20 / 200 |
| RPM ceiling | 300 | 4500 |

[실측 근거·단위·airborne/A-B 절차](../../../docs/reports/2026-09-21_longitudinal_torque_pi.md)를 따른다.
STM_A boot는 **speed/DEFAULT**다. Speed fallback은 기존 480 A current-limit 경로이므로
airborne Torque의 8 A/RPM300 설정이 Speed에도 적용된다고 가정하지 않는다.

```bash
cd ~/KAI_ws
si
ros2 launch control_launch vehicle.launch.py tuning_preset:=airborne
```

옵션 없는 launch는 driving이다. 마지막 TUI 선택을 자동으로 다음 launch 기본값으로 기억하지 않는다.

```text
config/
├── control_tuning.yaml          # 기존 공통 설정 / 사용자 경로 기준
└── control_tuning.presets/
    ├── driving.yaml
    └── airborne.yaml
```

`--config bench.yaml`은 옆의 `bench.presets/`를 사용한다. launch의 `control_tuning_file`
또는 양쪽의 `KAI_CONTROL_TUNING_FILE`을 같은 경로로 설정한다. 기본 경로는 소스 checkout의
`config/control_tuning.yaml`이고 소스 없는 설치는 share config를 쓴다. 저장 시 lock,
fsync/atomic replace, `.yaml.bak`과 외부 변경 검사를 사용한다. TUI는 Git push를 하지 않는다.

**v1 flat values/옛 통합 preset은 자동 적용하지 않는다.** checked-in v2 파일을 양식으로
기존 values를 해당 section에 옮기고 longitudinal mode/모든 값을 명시한다.
사용자 조향값을 덮어쓰거나 종방향 값을 숨은 fallback으로 채우지 않는다.
선택한 프리셋이 없거나 잘못되면 launch가 실패하며 다른 condition을 대신 쓰지 않는다.
공통 `control_tuning.yaml`은 기반 설정이며 S는 이를 바꾸지 않는다.

기본 YAML → 공통 설정 → 선택한 프리셋 → 명시한 launch 인자 순서다.
`actuation_mode:=position`은 KEYA 조향 인자이며 종방향 mode와 별개다.
`lateral_ctrl` launch는 조향, `command_governor` launch는 Governor,
`control_launch vehicle.launch.py`는 두 노드와 STM_A calibration helper를 포함한다.

## Runtime apply/readback

조향 튜닝은 기존 루트 namespace의 steering node 한 개를 대상으로 한다. Torque mode,
KEYA 전류 환산 기준, 최근 0.5 s 내 차량·조향 상태, MANUAL, |speed|≤0.05 m/s,
조향 비활성을 확인하고 `SetParametersAtomically` 후 실제 값을 재조회한다.
KEYA 내부 0020=torque 2/position 3은 별도 설정이다. steering fault 정책은 바꾸지 않는다.

구동 튜닝은 STM_A가 직접 MANUAL, fresh Part I/II <200 ms, |speed|≤0.05 m/s,
pedal=0, |commanded current|<0.1 A, |actual phase current|≤1 A를 검사한다.
mode 전환도 같은 gate다. request/revision 및 전체 config를 다음 Control cycle에서
atomic commit한다. TUI는 `/drive/longitudinal_status`의 source/revision/실제 float32 값을
비교하고 성공을 표시한다. unsafe는 거절하고 timeout은 적용 여부 미확인으로 표시한다.
자동 재전송은 없다. [Wire 계약](../../../docs/INTERFACES.md#longitudinal-calibration-v1).

launch의 one-shot helper는 15 s 안에 safe stop/readback을 기다려 한 번 적용한다.
실패하면 오류를 표시하고 종료하므로 실제 source/mode를 확인하고 MANUAL 정지에서 A로 적용한다.
Agent reconnect는 RAM config를 유지하고 MCU reboot는 DEFAULT로 복원한다.
매 튜닝 flash나 새 RTOS task가 필요하지 않다. TUI/helper는 movement command를 발행하지 않는다.

## 검증과 MCAP 비교

```bash
cd ~/KAI_ws
si
python3 -m pytest src/Control/software/src/control_tuning/test -q
python3 -m control_tuning.longitudinal_analysis <bag-directory> --start 0 --end 10.5 --target 6
```

Tests는 별도 ROS domain/localhost에서 실행한다. `CONTROL_TUNING_LAUNCH_TESTS=1`은
빌드한 실제 조향/Governor 노드를 CAN/output disabled로 실행해 save/apply/restart를 검사한다.
분석은 원본 MCAP을 변경하지 않는다. start/end는 첫 positive AUTO command 기준 초다.
Control recorder는 calibration readback/PI 상태를 포함한다. 별도 Decision 자체 recorder는
아직 새 topic을 포함하지 않으므로 A/B에서는 Control recorder를 사용한다.
