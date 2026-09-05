# vehicle_test_scenarios

## 현장 SYSID TUI

이 패키지 디렉터리에서 다음 명령 하나로 실행한다.

    python3 auto_sysid.py

차량 TF, STM A/B Agent, 명령 중계, 조향, 차량 상태, GNSS와 IMU 드라이버는
별도로 실행한다. TUI는 ROS 환경과 작업공간 overlay를 읽어 외부 드라이버와
토픽 수신 상태를 표시한다. 드라이버 실행·재시작·종료는 외부에서 관리한다.
필수 조건은 한 화면에서 GO/NO-GO와 원인을 확인할 수 있다.

메뉴는 1–4로 시나리오 선택, V로 최고 목표속도 입력, R로 Run,
S 또는 Space로 Stop, Q로 종료한다. 사용자가 정하는 수치는 최고 목표속도뿐이며
기본값은 5 km/h이며 고정된 최대 한도는 없다. 0보다 큰 유한한 값을 입력한다.
실제 차속과 순간적인 초과 응답은 기록한다.

| 번호 | 입력 |
| --- | --- |
| 1 | 짧은 가속·유지·0속도·추가 상승·부분 하강 조합, 20초 |
| 2 | 3초 상승·3초 유지·4초 하강·0속도 3초, 13초 |
| 3 | 정속 준비 후 중앙/좌/중앙/우/중앙 각 1초, 4초 감속·0속도 3초 |
| 4 | 정속 준비 후 좌우 조향 0.2→1.0 Hz를 10초, 4초 감속·0속도 3초 |

시나리오 1·2는 대기·카운트다운·시험·완료 후 모두 조향 모터를 Disable하고
센터각을 사용하지 않는다. 화면에는 목표각 대신 `모터 DISABLE 요청`을 표시한다.
시나리오 3·4의 중앙 조향은 2026-09-05 직선 기록에서 선택한 임시값 −0.56°,
좌우 진폭은 앞바퀴 등가각 ±1°다. 설정과 실제 실행 구간은 기록 폴더에 저장한다.
조향 입력은 실제 차속이 목표의 ±0.5 km/h 범위를 1초 유지한 뒤 시작한다.
15초 안에 정속에 도달하지 못하면 4초간 감속하여 종료한다.

Run은 MCAP 연결 후 AUTO를 기다린다. AUTO 확인부터 5초를 카운트다운하고
시험을 시작한다. 이미 AUTO인 경우도 Run을 누른 뒤 5초를 센다.
카운트다운·시험 중 MANUAL 전환은 해당 Run을 종료한다.
다시 AUTO로 올리는 것만으로 시험을 재시작하지 않는다.

정상 완료 후에는 목표속도 0을 유지한다. 조향은 1·2에서 Disable,
3·4에서 중앙 조향을 유지한다.
Stop, 활성 시험의 필수 통신 단절, 녹화 실패에는 브레이크를 요청한다.
Q/Ctrl+C 종료 시에는 명령을 발행했던 경우 정지 요청을 먼저 전달한 뒤
진행 중인 녹화를 저장하고 종료한다.
터미널 연결이 끊어지는 경우에도 같은 정리 절차를 수행한다.

MCAP과 실행 설정·로그는 다음 위치에 저장된다.

    /home/kai/KAI_ws/records/<YYYYMMDD_HHMMSS_microseconds>_scenario_<번호>_v<최고속도>/
      rosbag/
      scenario.yaml
      console.log

녹화는 Run부터 시작해 정상 완료 후 2초까지 포함한다. 모든 토픽은 같은
목록에 수신 상태를 표시한다. 필수 수신 조건에 포함되지 않은 토픽은
데이터가 없으면 `없음 · 실행 가능`으로 표시한다. 화면의 MANUAL,
조향 비활성, 입력 대기는 상황에 따라 정상 상태다. 센서/제어 상태의 실제
수신과 유효성, 명령 발행자 충돌을 별도로 확인한다.

## 기존 ROS 시나리오 노드

실차 `vehicle.launch.py`를 유지한 채 production Planning 자리에 통제된 시험
명령을 넣는 시나리오 패키지다.

```text
vehicle_test_scenario_runner
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

기본값은 `straight`, `3 km/h`, 조향 모터 Disable이다.
`straight`, `speed_step`, `speed_ramp`, `brake_test`는 중앙각과 관계없이
조향 모터를 Disable한다. 대기·카운트다운·완료 후에도 동일하다.
`steering_step`, `steering_sweep`는 기존 중앙각·진폭을 사용한다.
조향 노드가 모터 설정을 수락한 뒤 20 Hz로 fresh한 `ActuatorCommand`를 발행한다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `enabled` | `false` | 실제 명령 발행 허용 |
| `scenario` | `straight` | 실행할 시나리오 |
| `cruise_speed_kph` | `3.0` | 직진·sweep·brake 사전 주행 속도 |
| `speed_low_kph` / `speed_high_kph` | `0.0` / `3.0` | step/ramp 양 끝 속도 |
| `speed_step_interval_sec` | `2.0` | step 전환 간격 |
| `speed_ramp_duration_sec` | `5.0` | low에서 high까지 걸리는 시간 |
| `steering_center_deg` | `0.0` | 조향 step/sweep 전용 중앙 기준 절대 조향각. 직진 시험에서는 무시 |
| `steering_amplitude_deg` | `10.0` | sweep 진폭 |
| `steering_step_interval_sec` | `5.0` | step의 center/좌/center/우 각 phase 시간 |
| `steering_period_sec` | `4.0` | sweep 한 주기 |
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

두 시험 실행기는 `/steering_actuator_node`의 런타임 파라미터
`steering_enabled`를 시나리오에 맞게 설정한다. 이 기능을 지원하는 조향 노드와
함께 빌드·재시작해야 하며, 서비스 미수신·설정 거부 시 주행 입력을 시작하지 않는다.
Disable 중에도 조향 피드백은 계속 기록한다. 메시지의 `steering_target_rad=0`은
사용하지 않는 자리값이며, 모터에 0° 유지를 명령하는 의미가 아니다.
시험 노드를 종료해도 마지막 모터 설정은 유지된다. 일반 Planning으로 복귀할 때는
MANUAL 정차 상태에서 조향 노드를 재시작하거나 `steering_enabled=true`로 복원한다.

## 실행

```bash
cd "${KAIEV26_WS:?set KAIEV26_WS to the ROS 2 workspace root}"
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.zsh
colcon build --packages-up-to vehicle_test_scenarios --symlink-install
source install/setup.zsh

ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
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

시험을 중단할 때는 물리 E-stop/MANUAL을 우선하고, 소프트웨어에서는 다음처럼
brake 명령을 계속 발행할 수 있다.

```bash
ros2 param set /vehicle_test_scenario_runner brake_override true
ros2 param set /vehicle_test_scenario_runner enabled false  # MANUAL 전환 후
```

이 runner는 경로 추종 제어기가 아니라 통제된 step/ramp 입력원이다. 실차 시험은
구동륜 격리, E-stop 준비, 진행 방향 확인 후 낮은 값부터 시작한다.

## 2026-08-25 AUTO sysid 고정 launch

다음 세 launch는 명령행 인자를 받지 않는다. 각 파일이 실차 `vehicle.launch.py`,
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
```

| Launch | 고정 조건 | MCAP session prefix |
| --- | --- | --- |
| `auto_longitudinal_speed_step.launch.py` | 10 s zero baseline 후 0→1, 1↔2 m/s, 각 5 s, 4 cycle | `A02_speed_step_10_20` |
| `auto_lateral_steering_step.launch.py` | 1 m/s, center/+1/center/-1°, 각 5 s, 3 cycle | `A11_steering_step_center_pm1` |
| `auto_lateral_steering_sine.launch.py` | 1 m/s, center±1°, 8 s, 6 cycle | `A12_steering_sine_center_pm1_8s` |

두 횡방향 YAML의 `steering_center_deg: -2.0`은 2026-08-24 실차 데이터에서 얻은 얼라인먼트
변경 전 직진 추정값이다. 얼라인먼트를 바꿨다면 실행 전에 source YAML의 이 값만 새
직진각으로 수정하고 다시 빌드한다. 종방향 YAML에는 센터각 설정이 필요 없다.
각 run은 실차 제어컴의
`/home/kai/KAI_ws/records/` 아래 별도 MCAP으로 저장된다. logger는
`/planning/command`와 `/vehicle/command`를 모두 기록한다.
`/sysid/scinario`에는 각 YAML의 고정 `std_msgs/msg/String` label이 20 Hz로 기록된다.
