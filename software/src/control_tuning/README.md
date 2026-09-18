# Control Tuning

실차 주행과 시나리오 시험에서 공통으로 사용하는 Host 제어 설정을 편집하는 TUI입니다.
`현재 적용값 / 편집값 / 프리셋 저장값`을 나란히 비교합니다. 시나리오의 속도·중심각·진폭·주기는
기존 `vehicle_test_scenario_runner`에서 편집합니다.

## 실행

차량 데스크탑의 기존 `si` 별칭을 사용합니다.

```bash
cd ~/KAI_ws
si
ros2 run control_tuning control_tuning_tui
```

처음 추가하거나 코드를 변경했을 때는 먼저 빌드합니다.

```bash
cd ~/KAI_ws
si
colcon build --symlink-install --packages-select control_tuning lateral_ctrl command_governor control_launch
si
```

Python 의존성은 `rich`, `PyYAML`입니다. ROS 의존성과 함께 `rosdep`으로 설치하거나
Humble의 Python 환경에서 `/usr/bin/python3 -m pip install --user rich PyYAML`로 준비합니다.
최소 68열 × 24행이며 80열 터미널에서도 항목·단위·편집값·현재값을 정렬해 표시합니다.
선택 항목의 ROS 이름과 허용 범위는 표 아래에 표시하며 긴 설명과 단위 변환은 `?`로 확인합니다.
104열 이상에서는 프리셋 저장값 열, 132열 이상에서는 ROS 파라미터 열도 표시합니다.
`--offline`은 ROS 없이 편집·저장하고,
`--preview`는 화면만 출력하고 종료합니다. 변경된 행만 다시 그려 전체 화면 깜빡임을 줄입니다.

## 편집 항목

| 분류 | 항목 | 반영 |
|---|---|---|
| 조향 응답 | P 각도 오차 보정, D 조향 움직임 억제, 목표 도달 판정 오차·속도, 필터 주파수 | A 적용 / S 저장 |
| 출력 제한·보호 | 최대·지속 전류 명령, 고출력 허용·재사용 시간, 한계 보호 여유각, 과속 차단 기준 | A 적용 / S 저장 |
| 운용 설정 | Torque/Position, 조향 허용 최소 차속 | S 저장 후 조향 노드 재시작 |
| 수신 시간 제한 | 명령, CAN, 위치/토크 피드백, 모드, 차속, 상위 명령 유효시간 | S 저장 후 해당 노드 재시작 |

하드웨어 연결, 제어 주기, 캘리브레이션, 차량 운동 추정, 펌웨어 설정은 편집하지 않습니다.
화면의 입력 범위는 제어기 코드의 유효성 조건이며 실차에서 검증된 튜닝 권장 범위가 아닙니다.
P는 현재 최대 출력과 정착 오차에 따른 상한을 표시하며 D는 고정 상한이 없습니다.

## 조작

| 키 | 동작 |
|---|---|
| ←→ / Tab | 상단에서 프리셋 선택, 편집 중에는 분류 선택 |
| 1–4 | 선택한 프리셋의 파라미터 분류 선택 |
| ↑↓ / 마우스 휠 | 항목 선택, 검토 목록 스크롤 |
| Enter | 숫자 직접 입력 / 제어 방식 변경; 검토 화면에서 실행 |
| + / − | 항목별 간격으로 조절 |
| S | 선택한 프리셋을 조건별 YAML에 바로 저장 |
| A | 적용할 조향 튜닝 검토 후 실행 노드에 일괄 적용 |
| P / 첫 항목에서 ↑ | 상단 프리셋 선택으로 이동; ←→ 선택, Enter 불러오기 |
| Esc | 입력·검토 취소 |
| L / B / X | 선택 항목에 적용 중 값 / 기본 YAML 값 / 저장값 가져오기 |
| R / F | 노드 재조회 / 파일 다시 읽기 |
| ? / Q | 상세 설명·추가 단축키 / 종료; 저장하지 않은 수정이 있으면 폐기 확인 |

편집은 즉시 차량에 반영되지 않습니다. `S`는 선택한 프리셋 파일만 저장하고,
`A → Enter`는 실행 중 조향 튜닝만 변경합니다. 현재 적용과 다음 실행 모두에 유지하려면
둘 다 사용합니다. 수정한 항목은 `*`로 표시합니다. 운용·수신 시간 항목의 현재값과
선택한 프리셋 저장값이 다르면 상단에 그 개수를 표시합니다. 다른 프리셋으로 실행 중일 수도
있으므로, 재시작할 때 사용할 `tuning_preset`을 확인합니다.

## 시험 조건별 프리셋

`주행 / 공중 / 접지 정지`마다 **제어 설정 20개**를 별도 YAML로 보관합니다.
세 파일이 저장돼 있으며 조건별 검증값은 아닙니다.
시나리오의 진폭·주기·목표속도는 포함하지 않습니다.

공중 프리셋은 무부하 초기 시험 후보로 P=1 A/deg, D=0.04 A/(deg/s),
최대/지속 명령=2/1 A, 고출력 허용/회복=200/2000 ms로 낮췄습니다.
이는 검증 완료값이 아니며 접지 정지·주행 프리셋은 기존 초기값을 유지합니다.
launch의 `tuning_preset`은 조향 노드에 읽기 전용으로 공개됩니다.
시나리오 TUI는 이를 시험 조건 기본값으로 한 번 가져오며, 운영자가 바꾼 조건은 유지합니다.

TUI에서는 프리셋을 먼저 선택합니다. 선택 전에는 분류와 파라미터를 표시하지 않습니다.

1. 상단 `주행 / 공중 / 접지 정지`에서 ←→로 고르고 `Enter`를 누릅니다.
2. 바로 `조향 응답` 편집 화면이 열립니다. `1–4`로 분류를 이동합니다.
3. 편집 후 `S`로 해당 조건의 프리셋을 저장합니다.
4. 구동 스택을 해당 프리셋으로 실행합니다. 이미 실행 중이면 Torque 튜닝은 MANUAL·정지
   상태에서 `A → Enter`로 적용할 수 있고, 운용·수신 시간 변경은 재시작해야 합니다.

```bash
# 기본: 주행 프리셋. 매번 TUI에서 선택할 필요가 없습니다.
ros2 launch control_launch vehicle.launch.py

# 공중 / 접지 정지
ros2 launch control_launch vehicle.launch.py tuning_preset:=airborne
ros2 launch control_launch vehicle.launch.py tuning_preset:=ground_stationary
```

마지막으로 편집·저장한 조건을 launch의 기본값으로 기억하지 않습니다.
공중 프리셋을 저장해도 옵션 없는 실행은 계속 주행 프리셋을 사용합니다.

`P` 또는 첫 파라미터에서 ↑를 누르면 프리셋 선택으로 돌아갑니다. 저장하지 않은 수정이
있을 때만 폐기를 확인하며 `Esc`로 취소할 수 있습니다. 사용자 지정 경로에 아직 프리셋이
없으면 기존 YAML과 공통 설정을 읽어 편집을 시작하고, `S`를 누를 때 파일을 생성합니다.
실행할 프리셋 파일이 없거나 잘못된 경우 launch는 오류를 내며 다른 조건으로 대체하지 않습니다.

P·D·전류 한도는 명령 전류 단위로 함께 저장합니다. 최대 전류를 수정해도 P는 유지합니다.
전류 환산 기준이 다른 프리셋은 불러오지 않습니다.

```text
Control/software/src/control_tuning/config/
├── control_tuning.yaml                    # 기존 공통 설정 / 사용자 지정 경로 기준
└── control_tuning.presets/
    ├── driving.yaml                       # S: 주행 저장, 기본 launch
    ├── airborne.yaml                      # S: 공중 저장
    └── ground_stationary.yaml             # S: 접지 정지 저장
```

`--config bench.yaml`이면 옆의 `bench.presets/`를 사용합니다.
`KAI_CONTROL_TUNING_FILE`로 경로를 바꿔도 같은 규칙입니다. YAML은 Git으로 관리하며
`.yaml.bak`, `.yaml.lock`, 저장 중 임시 파일은 `config/.gitignore`로 제외합니다.
기존 `control_tuning.presets.yaml`도 읽지만 개별 파일이 있으면 우선합니다.
기존 프리셋을 불러와 `S`로 저장하면 해당 조건만 개별 파일에 저장합니다.

시나리오 TUI에서는 오른쪽 `시험 조건`에서 Enter → ↑↓ → Enter로 실제 조건을 별도로
선택합니다. 기록용 선택이므로 제어값은 바뀌지 않습니다. 시나리오 `R`도 제어 튜닝을
적용하지 않습니다. 녹화 시 실행 노드의 실제 파라미터, `/parameter_events`, 토크 상세
토픽을 함께 기록합니다. [시나리오 녹화 계약](../vehicle_test_scenarios/README.md)을 참고하세요.

## 전류 단위와 기존 파라미터

전류는 KEYA **명령 환산값**이며 실측 전류나 모터 정격이 아닙니다.
`keya_protection_current_a`가 장치 Parameter 0003과 일치한다는 기존 전제를 사용합니다.
현재 기본값 20 A에서 `1 raw = 20 × 0.8 / 10000 = 0.0016 A`입니다.

| 화면 이름 / 편집 모델 | 현재 기본값 | 저장되는 ROS 파라미터 |
|---|---:|---|
| P 각도 오차 보정 / `p_gain_a_per_deg` | 32 A/deg | `torque_full_scale_error_deg` = 최대 명령 A / P |
| D 조향 움직임 억제 / `d_gain_a_per_deg_s` | 약 0.0418879 A/(deg/s) | `torque_kd` = D / 0.0016 × 180/π |
| 최대 전류 명령 / `peak_current_command_a` | 16 A | `torque_peak_limit_raw` = A / 0.0016 |
| 지속 전류 명령 / `continuous_current_command_a` | 8 A | `torque_sustained_limit_raw` = A / 0.0016 |
| 조향 허용 최소 차속 / `minimum_vehicle_speed_kph` | 0 km/h | `minimum_steering_speed_kph` |

오차 0.1°이면 P항 3.2 A, 조향속도 60°/s이면 반대 방향 D항 약 2.51 A입니다.
합산 후 출력 제한·정착·보호 조건이 적용됩니다. 최대 명령을 변경해도 편집한 P는
유지하도록 포화 오차를 함께 환산합니다. 정수 raw 반올림 결과를 검토·저장값에 반영합니다.
각도는 앞바퀴 중앙 등가각입니다. ROS 명령 계약은 기존 `steering_target_rad`를 유지합니다.
기존 ROS 파라미터명은 호환성을 위해 유지하며 선택 항목의 설명에 항상 표시합니다.
P·D와 전류 한도는 편집값을 기존 ROS 단위로 환산한 값도 표시합니다.

## 저장 파일과 launch

기본 위치는 **Control 저장소 안의 `software/src/control_tuning/config/control_tuning.yaml`**입니다.
일반적인 배치에서는 `~/KAI_ws/src/Control/software/src/control_tuning/config/control_tuning.yaml`이며,
현재 작업 디렉터리나 사용자 홈 이름에 의존하지 않습니다. TUI와 세 launch가 같은 경로를
사용합니다. 소스 실행·symlink 빌드·일반 colcon 빌드 모두 해당 workspace의 소스 설정을
우선하므로 저장 뒤 launch에 반영하기 위해 다시 빌드할 필요는 없습니다. Control 소스 없이
패키지만 설치한 환경에서는 설치된 `share/control_tuning/config/`를 사용합니다.

프리셋에는 편집 항목 20개를 물리 단위로 저장하고 launch에서 ROS 단위로 환산합니다.
기존 공통 `control_tuning.yaml`은 호환용 기반 설정으로 읽으며 TUI의 `S`는 수정하지 않습니다.
임시 파일·fsync·atomic replace를 사용하고 이전 저장 파일은 `.yaml.bak`에 보관합니다.
파일 잠금과 변경 검사를 통해 다른 TUI나 편집기의 변경을 덮어쓰지 않고 재읽기를 요청합니다.
저장하면 Git 변경사항으로 나타나므로 설정과 코드를 함께 커밋·푸시할 수 있습니다.
TUI가 Git 커밋이나 푸시를 자동으로 실행하지는 않습니다.

이전 기본 경로였던 `~/.config/kaiev26/`는 더 이상 자동으로 읽지 않으며 `XDG_CONFIG_HOME`도
사용하지 않습니다. 다른 PC에 기존 설정이 있다면 시작 설정과 프리셋을 이 `config/`로 옮기거나
아래의 명시적 경로 설정을 사용합니다. 기존 통합 프리셋도 옮긴 뒤 불러올 수 있습니다.

다음 launch는 `tuning_preset`으로 선택한 프리셋을 읽으며 기본값은 모두 `driving`입니다.

- `control_launch vehicle.launch.py`: 조향 + 거버너
- `lateral_ctrl steering_actuator.launch.py`: 조향
- `command_governor command_governor.launch.py`: 거버너

우선순위는 **기본 YAML → 기존 공통 설정 → 선택한 프리셋 → 명시한 launch 인자**입니다.
예를 들어 `actuation_mode:=position`이면 저장된 torque보다 우선합니다.
선택한 프리셋이 없으면 launch가 실패합니다. 이 launch들을 포함하는 HIL에도 같은 규칙이
적용됩니다. HIL의 명시적 Position 인자는 저장된 제어 방식보다 우선합니다.
`ros2 run lateral_ctrl steering_actuator_node` 등 직접 실행은 이 파일을 자동으로 읽지 않습니다.

파일을 분리하려면 TUI의 `--config`와 launch의 `control_tuning_file:=...`에 같은 경로를
지정하거나, 두 터미널에 같은 `KAI_CONTROL_TUNING_FILE` 환경변수를 설정합니다.

```bash
# TUI에서 주행 프리셋을 선택하고 S로 저장한 뒤 종료
ros2 run control_tuning control_tuning_tui --config /tmp/bench-tuning.yaml
ros2 launch control_launch vehicle.launch.py control_tuning_file:=/tmp/bench-tuning.yaml
```

기본 YAML을 별도로 지정한 운용에서는 TUI에도 `--steering-config`, `--governor-config`,
`--calibration`으로 대응하는 파일을 지정하십시오. `?`는 실제 저장 경로를 표시합니다.
저장된 운용·timeout을 반영하려면 기존 launch를 완전히 종료하고 다시 실행합니다.

## 실행 중 적용

루트 namespace의 조향 노드 하나를 대상으로 합니다. 현재 Torque 모드이고 전류 환산
기준이 일치해야 하며, 최근 0.5초 내 차량·조향 상태, MANUAL 모드, 유효 차속
0.18 km/h 이하, 조향 비활성 상태를 확인합니다. 적용 직전 설정·상태를 재조회하고
변경된 설정이 발견되면 다시 검토하도록 합니다.

11개 torque 파라미터를 `SetParametersAtomically`로 전달하고 재조회로 적용값을 확인합니다.
이 작업에 대한 조향 노드의 기존 동작은 출력 Disable 후 새 명령 대기입니다.
TUI는 주행 명령, enable 토글, fault 해제를 보내지 않습니다. 응답이 불확실하면 자동 재적용하지
않고 다시 조회합니다. 이 UI의 상태 검사는 액추에이터 내부의 안전 판정을 대체하지 않습니다.

제어 방식은 Host만 바뀝니다. KEYA 내부 0020은 torque=2 / position=3을 별도로 맞춰야 합니다.
시나리오와 실차 모두 같은 조향·거버너 설정을 사용합니다.

## 검증

```bash
cd ~/KAI_ws
si
cd ~/KAI_ws/src/Control/software/src/control_tuning
/usr/bin/python3 -m pytest test -q
```

단위 변환·유효성, 저장 충돌·복원, 편집·화면 갱신과 ROS 적용 절차를 검사합니다.
화면 테스트는 `pyte`를 사용합니다. 테스트가 실차의 CAN이나 주행 명령을 사용하지 않습니다.
실제 C++ 노드·launch의 저장값 로딩, 적용, 재시작, CLI 우선순위 검증은 위 네 패키지를
빌드한 뒤 `CONTROL_TUNING_LAUNCH_TESTS=1 /usr/bin/python3 -m pytest test -q`로 실행합니다.
이 통합 테스트는 별도 ROS domain에서 CAN과 액추에이터 출력을 비활성화합니다.
