# control_launch

실차 구동 스택과 조이스틱 시험 명령 launch를 제공합니다. 워크스페이스 루트에서
빌드하고 차량 데스크탑에서 `cd ~/KAI_ws` 후 `si`로 환경을 불러와 사용합니다.
차량 TF를 위해 별도 `kaiev26_description` 패키지도 필요합니다.

## 실차 통합 실행

```bash
cd ~/KAI_ws
si
ros2 launch control_launch vehicle.launch.py
```

`can0`는 먼저 250 kbit/s로 준비되어 있어야 합니다. 이 launch는 차량 TF,
STM_A/B Agent, `command_governor`, `steering_actuator_node`, `longitudinal_actuator_node`,
차량 상태·wheel twist, `diagnostics_publisher`, `auto_dashboard_summary`를 실행합니다.
센서 드라이버, Planning, 시나리오 TUI와 관측 화면은 별도로 실행합니다.
`/planning/command` 명령원은 하나만 실행하며 `/vehicle/command`는 governor가 발행합니다.

| 인자 | 기본값 | 의미 |
|---|---|---|
| `stm_a_port`, `stm_b_port` | `/dev/stm_a`, `/dev/stm_b` | 각 Agent의 serial 장치 |
| `agent_baud` | `460800` | micro-ROS serial baud |
| `agents` | `true` | Agent 2개 실행 |
| `steering` | `true` | 조향 노드 실행 |
| `can_interface` | `can0` | 조향 CAN 인터페이스 |
| `actuation_mode` | 빈 값 | 선택한 프리셋 사용; `position`/`torque`로 override 가능 |
| `steering_enabled` | 빈 값 | 조향 설정 기본값 사용, 현재 `true` |
| `longitudinal` | `true` | 종방향 torque 노드 실행 |
| `longitudinal_torque` | 빈 값 | YAML 기본(`false`) 사용. `true`면 시작부터 AUTO에서 EZkontrol torque control. 실행 중 `ros2 param set /longitudinal_actuator_node torque_control_enabled true` |
| `longitudinal_speed_source` | 빈 값 | YAML 기본(`drive_status`). `gnss`는 `/gnss/fix_velocity` |
| `tuning_preset` | `driving` | `driving`(주행) / `airborne`(공중) / `ground_stationary`(접지 정지) |
| `control_tuning_file` | 제어 설정 경로 | 기본 `Control/software/src/control_tuning/config/control_tuning.yaml`; `KAI_CONTROL_TUNING_FILE`로 변경 가능 |

공통 파라미터는 `ros2 run control_tuning control_tuning_tui`로 수정합니다.
조향·거버너 모두 **기본 YAML → 기존 공통 설정 → 선택한 프리셋 → 명시적 launch 인자** 순서로 읽습니다.
옵션 없이 실행하면 항상 주행 프리셋을 읽습니다. 선택한 프리셋이 없거나 잘못되면 오류를 냅니다.
TUI에서 `S`로 저장한 값은 기존 launch를 완전히 종료하고 같은 프리셋으로 다시 실행할 때 반영됩니다. Torque 튜닝 11개는 MANUAL·정지 상태에서 `A`로
실행 중 적용할 수도 있습니다. [TUI 사용법](../control_tuning/README.md)을 참고하십시오.

현재 조향 YAML은 하드웨어 출력을 허용하고 최소 차속을 `0 km/h`로 설정합니다.
AUTO·fresh 상태·새 명령 등 조향 조건은 계속 적용됩니다. KEYA 내부 mode는 Host가
설정하지 않으며 torque 운용 시 Parameter 0020=2를 별도로 맞춰야 합니다.

## 종방향 torque control

`longitudinal_actuator_node`는 기본으로 `torque_mode=false`만 발행해 STM_A가 기존
EZkontrol speed control을 쓰게 둡니다. `longitudinal_torque:=true`로 시작하거나
실행 중 파라미터를 켜면 AUTO·전진·fresh 명령·fault 없음 조건에서 상전류 명령으로
바뀌고, 조건이 깨지면 즉시 speed control로 돌아갑니다. STM_A 펌웨어가
`/drive/torque_command`를 구독하는 버전이어야 하며, 예전 펌웨어에서는 켜도 변화가
없습니다. [longitudinal_ctrl](../longitudinal_ctrl/README.md)과
[시험 케이스](../../../docs/LONGITUDINAL_TORQUE_CONTROL_20260920.md)를 참고하십시오.

## 조향 정지와 자동 복구

실제 조향 변화율은 D항과 settle 판정에 사용하며 속도 자체로 정지하지 않습니다.
invalid/stale feedback이나 control stall은 `FEEDBACK_LOST`로 zero→Disable합니다.
정상 feedback과 차단 뒤 받은 새 command가 확인되면 노드 재시작이나 parameter toggle
없이 같은 제어주기에서 재활성화할 수 있습니다. `FAULT`는 bit0(Disable)을 제외한
KEYA 실제 fault bit에만 사용합니다.

`ros2 launch vehicle_console vehicle.launch.py`는 **관측 화면**입니다.
이 화면을 재시작해도 조향 노드는 재시작되지 않습니다. KEYA 자체 hardware fault도
Host 재시작만으로 해제된다고 보장하지 않습니다.
[조향 복구 계약](../lateral_ctrl/TORQUE_CONTROL.md),
[실차 운용 절차](../../../docs/VEHICLE_RUNBOOK.md)를 참고하십시오.

## 조이스틱 텔레옵

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
