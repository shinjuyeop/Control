# control_launch 조이스틱 텔레옵

게임패드로 목표 조향각과 속도를 조작해 `/planning/command`를 발행합니다.

```bash
ros2 launch control_launch joystick_teleop.launch.py
```

기존 `control_launch vehicle.launch.py`로 실행한 실차 Control과 함께 사용합니다.
텔레옵 실행 중에는 Decision의 명령 발행 노드를 실행하지 않습니다.

```text
게임패드 → joy_node → /joy → control_launch의 joystick_teleop_node → /planning/command
  → command_governor → /vehicle/command → 실차 Control
```

## Gazebo sitl_can에서 사용

기존 `sitl_can`의 vCAN 실행 환경을 준비한 뒤
`Simulation/kaiev26_gazebo_bringup/config/simulation.yaml`을 다음과 같이 설정합니다.

```yaml
common:
  headless: false
driver:
  input: emulated
  initial_mode: autonomous
```

기존 설정 파일에서 이 세 항목을 변경합니다. 운전 모드는 에뮬레이터가
제공하고 패드는 `control_launch`의 텔레옵 노드가 읽습니다. 각 터미널에서 실행합니다.

```bash
ros2 launch kaiev26_gazebo_bringup sitl_can.launch.py
ros2 launch control_launch joystick_teleop.launch.py use_sim_time:=true
```

## 조작

| 입력 | 목표값 |
|---|---|
| 왼쪽 스틱 좌우 | 스틱 값 × 런치 최대 조향각, 기본 −30°..+30° |
| 오른쪽 스틱 좌우 | 스틱 값 × 동적 최대 조향각 |
| RT | 누른 비율 × 런치 최대속도, 기본 0..20 km/h |
| LT | 누른 비율 × 동적 최대속도 |
| 십자키 ← / → | 동적 최대 조향각 감소 / 증가 |
| 십자키 ↓ / ↑ | 동적 최대속도 감소 / 증가 |

입력은 선형으로 매핑합니다. 오른쪽 스틱을 움직이면 오른쪽 입력이 우선하고,
중앙으로 놓으면 현재 왼쪽 스틱 입력을 사용합니다. LT를 누르면 LT 입력이 우선하고,
놓으면 현재 RT 입력을 사용합니다. 동적 최대값이 0이어도 이 우선순위는 같습니다.
두 스틱이 중앙이고 두 트리거를 놓으면 목표 조향각과 속도는 모두 0입니다.

십자키는 다른 조작과 관계없이 언제든 사용할 수 있습니다. 한 번 누를 때 한 단계
변경되며 길게 누르면 그 값이 유지됩니다. 방향을 놓았다가 다시 누르거나 반대 방향으로
바꾸면 다음 단계가 반영됩니다. 변경은 현재 명령에 즉시 적용됩니다.
시작과 설정 변경 시 터미널에 오른쪽 스틱과 LT의 범위를 표시합니다.

## 런치 파라미터

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `max_steering_deg` | 30.0 | 왼쪽 스틱의 한쪽 최대 조향각, 중앙 등가 Ackermann 각도 |
| `max_speed_kph` | 20.0 | RT를 끝까지 눌렀을 때의 목표속도 |
| `steering_step_deg` | 1.0 | 십자키 한 번당 조향 범위 변화량 |
| `speed_step_kph` | 1.0 | 십자키 한 번당 속도 범위 변화량 |

동적 범위는 실행 시 해당 런치 최대값으로 초기화하고 이후 독립적으로 증감합니다.
최솟값은 0이며 런치 최대값보다 크게 설정할 수도 있습니다. 재실행하면 초기화합니다.
최대값은 0 이상, 증감 단위는 양수로 설정합니다. 런치 설정을 바꾸려면 재실행합니다.

```bash
ros2 launch control_launch joystick_teleop.launch.py \
  max_steering_deg:=30.0 max_speed_kph:=20.0 \
  steering_step_deg:=1.0 speed_step_kph:=1.0
```

기존 가제보 패드 배치를 사용합니다. 축 인덱스는 `left_steering_axis=0`,
`right_steering_axis=3`, `lt_axis=2`, `rt_axis=5`, `dpad_horizontal_axis=6`,
`dpad_vertical_axis=7`이며 모두 런치 파라미터입니다. 스틱 왼쪽과 십자키 왼쪽·위쪽이
양수이고 트리거는 놓으면 +1, 끝까지 누르면 −1인 배치입니다.

`joy_node`의 기본값은 `device_id=0`, `deadzone=0.05`, `autorepeat_rate=60.0`,
`coalesce_interval_ms=1`입니다. 스틱 중앙 판정은 `joy_node`에서 처리한 0을 사용합니다.
`joy_topic`, `command_topic`, `use_sim_time`도 런치에서 설정할 수 있습니다.

## ROS 인터페이스

- 입력: `/joy` — `sensor_msgs/msg/Joy`
- 출력: `/planning/command` — `kaiev26_msgs/msg/ActuatorCommand`
- `steering_target_rad`: 좌회전 양수, rad 단위
- `speed_target_mps`: m/s 단위
- `brake_engage`: false

Joy 메시지를 받을 때마다 현재 ROS 시각을 기록해 명령을 발행합니다. 유지 중 입력의
반복 발행은 `joy_node`의 60 Hz autorepeat를 사용합니다. 두 토픽 모두 Reliable,
Keep Last 10 QoS를 사용합니다.

## 토크 조향 시험 스택

```bash
ros2 launch control_launch torque_test.launch.py [torque_peak_limit_raw:=2000] [torque_sustained_limit_raw:=1000]
```

`vehicle.launch.py`를 `actuation_mode=torque`, `steering_enabled=false`, bring-up 토크
한계로 띄운다. 다음은 `ros2 run vehicle_test_scenarios vehicle_test_scenario_runner`(TUI)다.
절차는 [`docs/Experiment_RUNBOOK.md`](../../../docs/Experiment_RUNBOOK.md) T3를 따른다.
`vehicle.launch.py`에도 같은 두 인자가 있으며 빈 값이면 `steering_actuator.yaml`을 따른다.
