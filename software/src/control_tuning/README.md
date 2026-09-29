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
Python 의존성은 rich/PyYAML이다. MCAP 분석은 [control_tools](../control_tools/README.md)에서 수행한다.
`--offline`은 파일 편집·저장만, `--preview`는 첫 화면 출력만 수행한다.
80열×24행 이상, 104열에서 저장값 열, 132열에서 native 이름 열을 표시한다.

## 분류와 적용

| 분류 | 항목 | 적용 |
|---|---|---|
| 1 조향응답 | 조향 제어 방식, P/D, 목표 도달 판정, 조향속도 필터 | S 저장 후 재시작 |
| 2 조향출력 | 최대·지속 전류, 고출력 시간·회복, 한계각 여유, 조향 허용 최소 차속 | S 저장 후 재시작 |
| 3 구동응답 | speed/torque, 속도 Kp/Ki, deadband/hysteresis | S 저장 후 재시작 |
| 4 구동출력 | 최대 전류, 적분 상한, 전류 상승·하강·재인가, RPM 상한 | S 저장 후 재시작 |
| 5 수신제한 | 상위 명령 유효시간과 조향 명령·CAN·피드백·모드·차속 수신 제한 | S 저장 후 재시작 |

프리셋 선택 후 5개 분류가 터미널 폭에 맞춰 간격을 두고 한 행에 표시된다.
←→/Tab으로 이동하거나 1–5로 바로 선택한다. 선택된 분류는 대괄호와 강조색으로 표시한다.
최소 터미널 폭은 80열이다.

하드웨어 연결, 조향 보정식, 제어 주기와 차량 운동 추정은 편집하지 않는다.
Governor에는 새 ramp를 추가하지 않았다. Decision이 기존 accel/decel ramp를 담당한다.
입력 허용 범위는 코드 계약이며 검증된 실차 tuning range가 아니다.

## 조작

| 키 | 동작 |
|---|---|
| ←→ / Tab, 1–5 | 프리셋/분류 이동 |
| ↑↓ / 휠 | 항목 선택 |
| Enter, +/− | 직접 입력/선택 전환, 증감 |
| S | 선택한 프리셋 YAML 저장 |
| P / 첫 항목에서 ↑ | 프리셋 선택으로 복귀 |
| L / B / X | 현재 적용값 / 기본값 / 프리셋 저장값을 편집란으로 가져오기 |
| R / F | 실행값 조회 / 파일 다시 읽기 |
| ? / Q / Esc | 도움말 / 종료 / 취소 |

하단 키 안내는 `←→ 분류  ↑↓ 항목  Enter 수정  S 저장  P 프리셋  ? 도움말  Q 종료` 한 줄이다.
추가 조회·복원 키는 `?`에서 확인한다.

**S는 파일 저장만 수행한다.** TUI는 현재값 조회용이며 실행 중 적용 기능은 없다.
저장 후 MANUAL 정지 상태에서 기존 구동 스택을 종료하고 같은 프리셋으로 재시작한다.
TUI만 다시 여는 것으로는 제어값이 적용되지 않는다.

## 두 프리셋과 schema v3

`airborne`과 `driving`만 유지한다. 접지 정지는 별도 calibration이 아니라 시나리오의
`stationary` 시험 조건이다. 선택한 시험 조건은 녹화 annotation이며 제어값을 바꾸지 않는다.

v3는 `steering`, `drive`, `governor`, `timeouts`로 구분한다. 설정키는 `steering_`, `drive_`, `governor_`로 담당 계통을 표시한다.
기존 lateral 값은 그대로 이관했다. 종방향 값은 **MCAP 기반 first-test candidate**다.

| 값 | airborne | driving |
|---|---:|---:|
| mode | torque | torque |
| Kp / Ki | 15 / 2 | 180 / 12 |
| max / integrator A | 40 / 6 | 480 / 120 |
| slew up / down A/s | 120 / 400 | 2400 / 4800 |
| deadband / hysteresis m/s | 0.05 / 0.12 | 0.05 / 0.12 |
| reapply band A / slew A/s | 5 / 40 | 20 / 200 |
| RPM ceiling | 4500 | 4500 |

[실측 근거·단위·airborne/A-B 절차](../../../docs/reports/2026-09-21_longitudinal_torque_pi.md)를 따른다.
STM_A boot는 **speed/DEFAULT**다. `drive_current_limit_a`/`drive_motor_speed_limit_rpm`은
Speed/Torque 공통 envelope다. airborne는 두 AUTO mode 모두 40 A/4500 rpm,
driving은 480 A/4500 rpm이다. Speed에서는 current-limit field와 signed target RPM 범위,
Torque에서는 PI 전류 상한과 EZ RPM ceiling으로 쓰며 최종 물리 forward/reverse clamp는 별도다.
PI gain·적분·deadband/coast·slew/reapply는 Torque 전용이다. 40 A는 공중 40 km/h
시험용 초기 전류 한도다. 현재 타이어 지름·감속비 환산에서 40 km/h는 약 2077 motor rpm으로,
4500 rpm 상한보다 낮다. 도달 시간과 실제 전류는 아직 검증되지 않았다.
MANUAL pedal은 기존 경로를 쓴다.
이 공통 envelope 동작에는 수정된 STM_A firmware가 필요하다. 그 이후 값 조정은 저장·구동 스택 재시작으로 반영한다.

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

**v2 프리셋은 그대로 읽을 수 있고 S 저장 시 v3 이름으로 전환한다.** 읽기만으로 원본 파일을 바꾸지 않는다.
새·옛 이름의 값이 충돌하거나 관리 대상 항목이 누락되면 오류로 표시하며 default를 채우지 않는다.
v1 flat preset은 종방향 값까지 명시해 이관해야 한다. 공통 `control_tuning.yaml`의 native ROS override 형식은 유지한다.

[전체 새 이름·단위·ROS/MCU 연결·MCAP 호환표](../../../docs/CONTROL_TUNING_PARAMETERS.md)를 따른다.
TUI 상세의 **설정키**는 새 이름이고 **ROS/STM_A 연결**은 기존 통신 계약이다.
제어기별 native 이름은 변환 계층이 처리하므로 이미 실행 중인 노드에서도 현재값을 읽을 수 있다.

기본 YAML → 공통 설정 → 선택한 프리셋 → 명시한 launch 인자 순서다.
`actuation_mode:=position`은 KEYA 조향 인자이며 종방향 mode와 별개다.
`lateral_ctrl` launch는 조향, `command_governor` launch는 Governor,
`control_launch vehicle.launch.py`는 두 노드와 STM_A calibration helper를 포함한다.

## 현재값 조회와 다음 실행 반영

TUI는 조향/Governor의 `GetParameters`와 STM_A의 `/drive/longitudinal_status`를 읽는다.
STM_A 설정 송신 publisher나 조향 parameter 변경 client를 만들지 않는다.
현재값, 편집값과 저장값은 별도로 유지하며 오래된 readback을 현재값으로 표시하지 않는다.

구동 스택을 재시작하면 조향/Governor는 저장한 프리셋을 읽는다.
STM_A calibration은 launch의 one-shot helper가 별도로 적용한다.
STM_A가 MANUAL, fresh Part I <500 ms·Part II <1 s, |speed|≤0.05 m/s, pedal=0,
|commanded current|<0.1 A, |actual phase current|≤1 A를 검사하고 revision과 함께 atomic commit한다.
helper가 request/revision/full config readback을 확인하며 unsafe는 거절한다.
[Wire 계약](../../../docs/INTERFACES.md#longitudinal-calibration-v1)은 그대로다.

helper는 15 s 안에 safe stop/readback을 기다려 한 번 적용한다. 실패하면 오류를 표시하고
종료하므로 실제 source/mode와 정지 상태를 확인하고 선택한 프리셋으로 구동 스택을 재시작한다.
자동 재전송은 없다. Agent reconnect는 RAM config를 유지하고 MCU reboot는 DEFAULT로 복원한다.
매 튜닝 flash나 새 RTOS task가 필요하지 않다. KEYA 내부 0020 설정은 별도다.

## 검증과 MCAP 비교

```bash
cd ~/KAI_ws
si
python3 -m pytest src/Control/software/src/control_tuning/test -q
ros2 run control_tools longitudinal_analysis <bag-directory> --start 0 --end 10.5 --target 6
```

Tests는 별도 ROS domain/localhost에서 실행한다. `CONTROL_TUNING_LAUNCH_TESTS=1`은
빌드한 실제 조향/Governor 노드를 CAN/output disabled로 실행해 저장·실행값 유지·재시작 반영을 검사한다.
분석은 원본 MCAP을 변경하지 않는다. start/end는 첫 positive AUTO command 기준 초다.
Control recorder는 calibration readback/PI 상태를 포함한다. 별도 Decision 자체 recorder는
아직 새 topic을 포함하지 않으므로 A/B에서는 Control recorder를 사용한다.
