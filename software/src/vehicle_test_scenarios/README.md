# vehicle_test_scenarios

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
| `steering_sweep` | 일정 속도에서 중앙±진폭을 sine 파형으로 왕복 |
| `speed_step` | `speed_low_kph`와 `speed_high_kph`를 일정 간격으로 전환 |
| `speed_ramp` | low→high→low 속도를 선형 ramp로 반복 |
| `brake_test` | 일정 속도 명령 후 지정 시간에 brake를 걸고 계속 유지 |

기본값은 `straight`, `3 km/h`, 중앙 조향 `0 deg`다. 모든 시나리오는 20 Hz로
fresh한 `ActuatorCommand`를 계속 발행한다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `enabled` | `false` | 실제 명령 발행 허용 |
| `scenario` | `straight` | 실행할 시나리오 |
| `cruise_speed_kph` | `3.0` | 직진·sweep·brake 사전 주행 속도 |
| `speed_low_kph` / `speed_high_kph` | `0.0` / `3.0` | step/ramp 양 끝 속도 |
| `speed_step_interval_sec` | `2.0` | step 전환 간격 |
| `speed_ramp_duration_sec` | `5.0` | low에서 high까지 걸리는 시간 |
| `steering_center_deg` | `0.0` | 중앙 기준 절대 조향각 |
| `steering_amplitude_deg` | `10.0` | sweep 진폭 |
| `steering_period_sec` | `4.0` | sweep 한 주기 |
| `brake_delay_sec` | `3.0` | brake test에서 제동 전 주행 시간 |
| `brake_override` | `false` | 모든 시나리오를 즉시 speed/steer 0 + brake로 덮어씀 |

속도는 `0..10 km/h`, 중앙 절댓값과 조향 진폭의 합은 `28 deg` 이하로 제한한다.
시나리오나 파라미터를 실행 중 바꾸면 경과 시간을 0으로 되돌려 새 시나리오를
처음부터 시작한다.

## 실행

```bash
cd ~/KAI_ws
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

### 조향 sweep/slalom

```bash
ros2 param set /vehicle_test_scenario_runner scenario steering_sweep
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 10.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 4.0
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

시험을 중단할 때는 물리 E-stop/MANUAL을 우선하고, 소프트웨어에서는 다음처럼
brake 명령을 계속 발행할 수 있다.

```bash
ros2 param set /vehicle_test_scenario_runner brake_override true
ros2 param set /vehicle_test_scenario_runner enabled false  # MANUAL 전환 후
```

이 runner는 경로 추종 제어기가 아니라 통제된 step/ramp 입력원이다. 실차 시험은
구동륜 격리, E-stop 준비, 진행 방향 확인 후 낮은 값부터 시작한다.
