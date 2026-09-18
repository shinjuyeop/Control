# vehicle_test_scenarios

시나리오별 속도·중심각·진폭·주기는 이 패키지의 TUI에서 편집합니다.
공통 조향 P·D, 출력 한도, 운용 설정과 Host timeout은 별도
[Control Tuning TUI](../control_tuning/README.md)에서 편집·저장·적용합니다.

## 현장 실차 시험 TUI

ROS 환경과 workspace overlay를 source한 터미널에서 다음 명령 하나로 실행한다.

```bash
cd ~/KAI_ws
si
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

직접 실행하면 TUI가 열린다. 기존 YAML launch는 명시적인
`vehicle_test_scenario_node` headless entrypoint를 사용한다. 이전처럼
`vehicle_test_scenario_runner --ros-args ...`를 호출해도 headless 호환 경로로
동작한다. 차량 TF, STM A/B Agent,
`command_governor`, 조향, 차량 상태, GNSS와 IMU 드라이버는 별도로 관리한다.
TUI는 이 프로세스들을 임의로 재시작하지 않고 실제 토픽과 ROS graph를 바탕으로
GO/NO-GO 원인을 표시한다.

시험 메뉴는 확장 가능한 카탈로그로 구성한다.

| 상위 분류 | 하위 분류 | 시험 |
| --- | --- | --- |
| 입력 응답 시험 | 종방향 입력 | 일정 속도 직진, 속도 Step, 속도 Ramp, 제동 입력 |
| 입력 응답 시험 | 횡방향 입력 | 조향 양방향 Step, 고정 주기 Sine, Chirp 주파수 Sweep |
| 경로 추종 시험 | — | K-City 예선 경로, K-City 본선 경로 |

조향 Square는 일반 TUI 목록에서 제외했다. 기존 `steering_square` headless 파라미터와
`auto_lateral_steering_square.launch.py`는 기존 입력 형식 호환을 위해 유지한다. 기본 속도·중심각은 이 경로도 0이다.

화살표로 시험과 파라미터를 이동하며 Enter로 선택·수정한다.

| 키 | 동작 |
| --- | --- |
| `↑` / `↓` | 시험·파라미터 이동; 선택형 항목 편집 중에는 값 전환 |
| `←` / `→` | 시험 목록과 파라미터 영역 전환 |
| `Enter` | 시험 선택, 파라미터 편집·확정 |
| `Esc` | 편집 취소 또는 시험 목록으로 이동 |
| `R` | 현재 설정으로 Run 요청, 녹화 ON이면 MCAP 시작 |
| `S` / `Space` | 정지·브레이크 요청 |
| `Q` | 안전 정리 후 종료 |

선택한 시험에서 실제 사용하는 파라미터만 표시한다. 완성된 설정은 `R` 시점에
한 번에 고정하며 실행·저장 중에는 수정하지 못한다. TUI를 열기만 해서는 명령이나
녹화를 시작하지 않는다.

모든 시험의 기본 목표속도와 조향 중심각은 `0 km/h`, `0 deg`다.
속도 Step/Ramp의 낮은·높은 속도와 경로 시험의 최고 목표속도도 0에서 시작한다.
시험할 속도와 중심각은 실행 전에 입력한다. 조향 진폭·주기 등은 기존 기본값을 유지한다.

파라미터 아래의 `MCAP 녹화`와 `시험 조건`에서 **Enter로 편집 → ↑↓로 선택 → Enter로 확정**한다.
`Esc`는 변경을 취소한다. 녹화는 ON/OFF(기본 ON), 조건은 주행/공중/접지 정지 순서다.
녹화 선택은 시나리오를 이동해도 유지되며 실행 중에는 변경할 수 없다.
시험 조건도 모든 시나리오에서 공유하며 `R`을 누를 때 고정한다. 실행 중인 조향 노드의
읽기 전용 `tuning_preset`을 조회해 기본 선택으로 사용하며, 사용자가 바꾼 조건은 유지한다.
이전 버전 노드 등에서 프리셋 정보를 읽지 못하면 직접 선택한다.
기록용 표시이므로 차속·조향 설정·차량 모드를 자동으로 바꾸지 않는다.
조건별 **제어** 프리셋은 [Control Tuning](../control_tuning/README.md) 상단에서 먼저 선택하고
편집하며 `S`로 조건별 YAML에 저장한다. 기본 구동 launch는 주행 프리셋을 사용하고
공중·접지 정지는 `tuning_preset:=airborne` / `ground_stationary`로 실행한다. 자동 선택된 시험 조건이
실제 환경과 일치하는지 확인한 후 실행한다. 실제 파라미터 기록에는 launch 프리셋 이름도 포함된다.
TUI는 alternate screen을 사용하므로 실행 중에는 이전 셸 명령과 로그가 보이지 않고,
종료하면 원래 화면이 복원된다. 상단 상태, 모든 NO-GO 항목, 전체 시험·파라미터,
하단 키 안내의 패널 순서는 고정하며, NO-GO 정렬과 경고·파라미터에 따라 줄 수는 바뀐다. 터미널보다 길면 마우스 휠 또는
`PgUp/PgDn`으로 TUI 전체를 위아래로 스크롤한다.
사용자 정지·오류 안내는 상단 `VEHICLE STATE · 시험 상태`에 표시하며 하단에는 키 안내만 둔다.

화면은 변경된 줄만 한 번에 출력하며 주기적으로 전체 화면을 지우지 않는다.
스크롤·터미널 크기 변경 시에도 현재 표시 영역을 다시 계산하고 남은 줄을 정리한다.

`R`은 GO 조건을 확인하고, 녹화 ON이면 MCAP recorder 연결과 실제 제어 파라미터
기록을 마친 뒤 AUTO를 기다린다. 파라미터 조회가 실패하면 시험 시작을 중단한다.
OFF이면 파일과 recorder를 만들지 않고 시험 준비 후 AUTO를 기다린다.
AUTO 확인부터 5초를 카운트다운하고 입력을 시작한다. 카운트다운·시험 중 MANUAL
전환, 필수 통신 단절, 녹화 실패에는 Run을 끝내며 필요한 경우 브레이크를 요청한다.
조향 하드웨어 고장 비트뿐 아니라 `/steering/status`의 `STATE_FAULT` 또는
`REASON_MOTOR_FAULT`도 시작 금지·실행 중단 조건이다. Torque의 각도·과속 등
소프트웨어 안전 정지이면 `조향 제어 안전 정지`를 표시하고, 실행 중에는 목표속도 0과
브레이크를 요청한다. 조향 노드의 fault 잠금을 자동으로 해제하지 않는다.
조향 입력 시험은 실제 차속이 목표의 ±0.5 km/h 범위를 1초 유지한 뒤 시작한다.
15초 안에 정속 판정이 되지 않으면 경고를 남기고 시험 입력을 계속한다.

K-City 경로 시험은 Decision의 `kcity_quali_route.yaml` 또는
`kcity_final_route.yaml`을 사용한다. Run 동안 GNSS-only 위치추정과 경로 추종
프로세스를 TUI가 소유하고 `/sysid/*` 전용 토픽으로 연결한다. GNSS Fix·속도·TF와
경로 입력이 끊기면 정지한다. 신호·장애물 판단은 포함하지 않는다.

녹화 ON일 때 MCAP과 실행 설정·로그는 기존 workspace의 `records/`에 저장한다.

```text
~/KAI_ws/records/
  <시간>_<scenario>_v<최고속도>/
    rosbag/
    scenario.yaml
    control_parameters.yaml
    console.log
```

외장 SSD 마운트는 Run 조건이 아니다. 녹화는 `R`부터 시작해 정상 완료 후
2초까지 포함한다. OFF이면 후행 녹화 대기 없이 시험을 정리한다. 경로 시험의
위치추정·추종 프로세스는 녹화 여부와 관계없이 Run 동안 관리한다. 기본
제어·상태·GNSS·IMU·TF, 압축 카메라, Ouster packet·metadata를 기록한다.

`scenario.yaml`에는 시험 조건, 입력 설정, 종료 사유, 기록 토픽별 메시지 수와
제어 파라미터 조회 결과를 저장한다. `control_parameters.yaml`은 **실행 중인**
`/steering_actuator_node`, `/command_governor`에서 List/GetParameters로 읽은 전체 값이다.
YAML 기본값으로 누락을 채우지 않는다. 토크 P full-scale·D, 전류 raw 한도, 허용/회복 시간,
필터, 조향 속도 제한, 토크 방향, 운용 방식, timeout, 캘리브레이션과 전류 환산 기준을 포함한다.
P·D·전류 한도의 화면 단위 환산값도 함께 기록한다.

조회는 recorder가 필수 토픽과 `/parameter_events`에 연결된 뒤 수행한다. 파일 저장 성공
후에만 AUTO 대기로 넘어간다. 필수 파라미터 누락·서비스 시간 초과·중복 노드는 실패로 기록하고
시험 시작을 허용하지 않는다. 각 노드의 조회 시각은 ROS 시각과 UTC 시각으로 남긴다.
이 스냅샷은 recorder 준비 완료 시점의 값이며, 이후 변경은 `/parameter_events`로 확인한다.

같은 내용의 JSON은 `/sysid/control_parameters`에도 한 번 발행해 **MCAP 안에도** 보관한다.
`/parameter_events`와 토크 P/D, 합산·최종 raw, 명령 전류, 출력 한도, 포화·고출력 제한·목표 도달
여부, 필터 전후 각속도, `torque_safety` 등 상세 토픽을 기존 recorder에서 함께 기록한다.
별도 `mcap_logging.launch.py`를 동시에 실행할 필요가 없다. Position 모드처럼 발행되지 않는
상세 토픽에는 데이터가 없으며, 시험 중 변경이 없으면 parameter event가 없을 수도 있다.
MCAP OFF에서는 파라미터 조회·파일 저장도 수행하지 않는다.

현재 TUI의 조향 software fault 감지, 조향 설정 대기 중 mode·통신 검사,
headless의 mode freshness에는
[알려진 제한](../../../docs/SAFETY.md#실차-시나리오-실행기의-현재-제한)이 있다.
RUNNING 표시는 모든 액추에이터가 정상 출력 중이라는 보장이 아니다.

## Headless ROS 시나리오 노드

실차 `vehicle.launch.py`를 유지한 채 production Planning 자리에 통제된 시험
명령을 넣는 시나리오 패키지다.

```text
vehicle_test_scenario_node
  -> /planning/command
  -> command_governor
  -> /vehicle/command
  -> STM_A / STM_B / lateral_ctrl
```

`/vehicle/command`로 직접 발행하지 않으며, 실제 Planning이나 다른 시험 발행자와
동시에 실행하지 않는다. 기본값은 `enabled=false`다.

## 시나리오

| `scenario` | 동작 |
| --- | --- |
| `straight` | `cruise_speed_kph`로 일정 속도 직진 |
| `steering_step` | 중앙→좌→중앙→우 순서의 정확한 조향 step |
| `speed_step` | `speed_low_kph`와 `speed_high_kph`를 일정 간격으로 전환 |
| `speed_ramp` | low→high→low 속도를 선형 ramp로 반복 |
| `brake_test` | 일정 속도 명령 후 지정 시간에 brake를 걸고 계속 유지 |
| `steering_sweep` | 일정 속도에서 중앙±진폭을 sine 파형으로 왕복 |
| `steering_chirp` | 일정 속도에서 시작→종료 주파수로 sine 주파수를 선형 변화 |
| `steering_square` | 중앙+진폭과 중앙−진폭을 반주기씩 유지하는 square 파형 |

기본값은 `straight`, `0 km/h`, 조향 모터 Disable이다.
`straight`, `speed_step`, `speed_ramp`, `brake_test`는 중앙각과 관계없이
조향 모터를 Disable한다. 대기·카운트다운·완료 후에도 동일하다.
`steering_step`, `steering_sweep`, `steering_chirp`, `steering_square`는 기존
중앙각·진폭을 사용한다.
조향 노드가 모터 설정을 수락한 뒤 20 Hz로 fresh한 `ActuatorCommand`를 발행한다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `enabled` | `false` | 실제 명령 발행 허용 |
| `scenario` | `straight` | 실행할 시나리오 |
| `cruise_speed_kph` | `0.0` | 직진·sweep·brake 사전 주행 속도 |
| `speed_low_kph` / `speed_high_kph` | `0.0` / `0.0` | step/ramp 양 끝 속도 |
| `speed_step_interval_sec` | `2.0` | step 전환 간격 |
| `speed_ramp_duration_sec` | `5.0` | low에서 high까지 걸리는 시간 |
| `steering_center_deg` | `0.0` | 조향 step/sweep/square 전용 중앙 기준 절대 조향각. 직진 시험에서는 무시 |
| `steering_amplitude_deg` | `10.0` | sweep·square 진폭 |
| `steering_step_interval_sec` | `5.0` | step의 center/좌/center/우 각 phase 시간 |
| `steering_period_sec` | `4.0` | sweep·square 한 주기. square는 반주기씩 좌·우 유지 |
| `chirp_start_hz` / `chirp_end_hz` | `0.2` / `1.0` | chirp 시작·종료 주파수 |
| `chirp_duration_sec` | `10.0` | 한 번의 chirp 입력 시간 |
| `brake_delay_sec` | `3.0` | brake test에서 제동 전 주행 시간 |
| `initial_hold_sec` | `0.0` | 반복 시나리오 시작 전 초기 setpoint 유지 시간 |
| `scenario_repetitions` | `0` | 0이면 무한 반복. 유한 반복 완료 후 speed target 0·brake false 유지 |
| `wait_for_auto_mode` | `false` | true이면 MANUAL에서 zero-speed baseline, AUTO 감지 후 timeline 준비 |
| `auto_start_delay_sec` | `0.0` | AUTO 감지 후 speed target 0·brake false countdown |
| `scenario_status_topic` | `/sysid/scinario` | 현재 시나리오 label `std_msgs/msg/String` 토픽 |
| `scenario_label` | 빈 문자열 | 빈 값이면 scenario 이름, 고정 launch는 run label 사용 |
| `brake_override` | `false` | 모든 시나리오를 즉시 speed/steer 0 + brake로 덮어씀 |

목표속도는 0 이상의 유한한 값이며 고정된 최대 한도는 없다.
`speed_low_kph`는 `speed_high_kph` 이하여야 한다.
조향 시험의 중앙 절댓값과 조향 진폭의 합은 `28 deg` 이하로 제한한다.
시나리오나 파라미터를 실행 중 바꾸면 경과 시간을 0으로 되돌려 새 시나리오를
처음부터 시작한다.
`scenario_label`이 빈 값이면 시나리오 변경에 맞춰 기록용 label도 갱신한다.
명시적으로 지정한 `scenario_label`은 시나리오가 바뀌어도 유지한다.

두 시험 실행기는 `/steering_actuator_node`의 런타임 파라미터
`steering_enabled`를 시나리오에 맞게 설정한다. 이 기능을 지원하는 조향 노드와
함께 빌드·재시작해야 하며, 서비스 미수신·설정 거부 시 주행 입력을 시작하지 않는다.
Disable 중에도 조향 피드백은 계속 기록한다. 메시지의 `steering_target_rad=0`은
사용하지 않는 자리값이며, 모터에 0° 유지를 명령하는 의미가 아니다.
시험 노드를 종료해도 마지막 모터 설정은 유지된다. 일반 Planning으로 복귀할 때는
MANUAL 정차 상태에서 조향 노드를 재시작하거나 `steering_enabled=true`로 복원한다.

## 실행

```bash
cd ~/KAI_ws
si
colcon build --packages-up-to vehicle_test_scenarios --symlink-install
si

ros2 run vehicle_test_scenarios vehicle_test_scenario_node
```

다른 터미널에서 `/planning/command` publisher가 이 노드 하나인지 확인한 후
시나리오를 고르고 마지막에 활성화한다.

```bash
ros2 topic info /planning/command       # Publisher count: 1이어야 함
ros2 param set /vehicle_test_scenario_runner scenario straight
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner enabled true
```

### 속도 step / ramp

```bash
ros2 param set /vehicle_test_scenario_runner speed_low_kph 0.0
ros2 param set /vehicle_test_scenario_runner speed_high_kph 3.0
ros2 param set /vehicle_test_scenario_runner scenario speed_step

# 또는
ros2 param set /vehicle_test_scenario_runner scenario speed_ramp
```

### 제동 시험

```bash
ros2 param set /vehicle_test_scenario_runner scenario brake_test
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner brake_delay_sec 3.0
```

### 조향 sweep/slalom

```bash
ros2 param set /vehicle_test_scenario_runner scenario steering_sweep
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 10.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 4.0
```

`initial_hold_sec`를 함께 설정하면 그 시간 동안은 조향 중앙에서 직진하다가
이후 sweep이 시작된다 (예: 5초 직진 후 슬라럼).

```bash
ros2 param set /vehicle_test_scenario_runner initial_hold_sec 5.0
```

### 조향 square

`steering_period_sec`의 절반 동안 `center+amplitude`, 나머지 절반 동안
`center-amplitude`를 유지한다. center phase 없이 `t=0`에서 바로 좌측 진폭으로
step한다.

```bash
ros2 param set /vehicle_test_scenario_runner scenario steering_square
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.6
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 1.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 8.0
```

시험을 중단할 때는 물리 E-stop/MANUAL을 우선하고, 소프트웨어에서는 다음처럼
brake 명령을 계속 발행할 수 있다.

```bash
ros2 param set /vehicle_test_scenario_runner brake_override true
ros2 param set /vehicle_test_scenario_runner enabled false  # MANUAL 전환 후
```

이 runner는 경로 추종 제어기가 아니라 통제된 step/ramp 입력원이다. 실차 시험은
구동륜 격리, E-stop 준비, 진행 방향 확인 후 낮은 값부터 시작한다.

## 2026-08-25 AUTO sysid 고정 launch

다음 launch들은 명령행 인자를 받지 않는다. 각 파일이 실차 `vehicle.launch.py`,
고정 YAML runner, MCAP logger를 한 번에 시작한다. MCAP은 launch 즉시 기록된다.
`/drive/status`가 MANUAL에서 AUTO로 바뀌면 10초 동안 `/planning/command`에
`speed_target_mps=0.0`, `brake_engage=false`를 발행하며 countdown을
로그에 표시한다. countdown 뒤 시나리오 `t=0`이 시작된다. 정해진 반복이 끝나면
다시 speed target 0·brake false를 유지한다. 종방향 launch는 전 구간 조향 Disable,
횡방향 launch는 대기·완료 시 중앙 조향을 유지한다.

```bash
ros2 launch vehicle_test_scenarios auto_longitudinal_speed_step.launch.py
ros2 launch vehicle_test_scenarios auto_lateral_steering_step.launch.py
ros2 launch vehicle_test_scenarios auto_lateral_steering_sine.launch.py
ros2 launch vehicle_test_scenarios auto_lateral_steering_square.launch.py
```

| Launch | 고정 조건 | MCAP session prefix |
| --- | --- | --- |
| `auto_longitudinal_speed_step.launch.py` | 10 s zero baseline 후 low↔high, 기본 모두 0 km/h, 각 5 s, 4 cycle | `A02_speed_step_10_20` |
| `auto_lateral_steering_step.launch.py` | 기본 0 km/h, center/+1/center/-1°, 각 5 s, 3 cycle | `A11_steering_step_center_pm1` |
| `auto_lateral_steering_sine.launch.py` | 기본 0 km/h, center±1°, 8 s, 6 cycle | `A12_steering_sine_center_pm1_8s` |
| `auto_lateral_steering_square.launch.py` | 기본 0 km/h, center±1° square, 각 4 s 유지, 6 cycle | `A13_steering_square_center_pm1_8s` |

모든 YAML의 목표속도 기본값은 0이며 세 횡방향 YAML의 `steering_center_deg`도 0이다.
실행 전에 source YAML에서 시험할 속도와 중심각을 설정하고 다시 빌드한다.
종방향 YAML에는 센터각 설정이 필요 없다. 기존 고정 label은 식별자 호환을 위해 유지하며,
`A02_speed_step_10_20`의 숫자는 현재 설정속도를 나타내지 않는다.
각 run은 실차 제어컴의
실행 중인 workspace의 `records/` 아래 별도 MCAP으로 저장된다. logger는
`/planning/command`와 `/vehicle/command`를 모두 기록한다.
`/sysid/scinario`에는 각 YAML의 고정 `std_msgs/msg/String` label이 20 Hz로 기록된다.
