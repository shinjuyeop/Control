# lateral_ctrl

TeamKAI-DL 차량의 KEYA 조향 액추에이터를 제어하는 ROS 2 Humble 패키지입니다.
`steering_actuator_node` 하나만 SocketCAN 인터페이스를 열고 조향 모터를 제어해야
합니다.

## 실차 실행 전 주의

`steering_actuator.launch.py`는 `hardware_can_enabled=true`와
`actuator_output_enabled=true`가 기본값입니다. launch를 실행하면 노드가 `can0`을
열고 KEYA 모터에 Disable 및 제어 프레임을 보낼 수 있습니다.

실행 전에 다음을 확인하십시오.

- 바퀴를 공중에 띄우고 조향 기구 주변을 비웁니다.
- 물리 E-stop을 즉시 누를 수 있게 준비합니다.
- CAN 인터페이스, 250 kbit/s 설정, 모터 node ID와 기계적 중앙을 확인합니다.
- 동일한 KEYA 모터를 제어하는 다른 노드를 종료합니다.

## 출력 모드 3가지

`hardware_can_enabled`와 `actuator_output_enabled`는 중복이 아니라 아래 세 가지
상태를 만듭니다. 조향이 움직이지 않을 때 가장 먼저 확인할 표입니다.

| `hardware_can_enabled` | `actuator_output_enabled` | 동작 | 용도 |
| --- | --- | --- | --- |
| `false` | 무관 | CAN 소켓을 열지 않음 | CAN 어댑터가 없는 PC, CI |
| `true` | `false` | 읽기 전용 관측 — 피드백만 수신, 명령 송신 없음 | 벤치·실차 진단 |
| `true` | `true` | 구동 | **launch 기본값** |

노드는 시작할 때 현재 모드를 로그로 알립니다. 구동 중이면
`HARDWARE OUTPUT ARMED ...`, 꺼져 있으면 무엇을 켜야 하는지 함께 출력합니다.

읽기 전용 관측으로 실행하려면 다음처럼 합니다.

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py \
  actuator_output_enabled:=false
```

CAN 소켓조차 열지 않으려면 `hardware_can_enabled:=false`를 함께 줍니다.

`ros2 run lateral_ctrl steering_actuator_node`로 params file 없이 직접 실행할 때만
두 파라미터의 코드 기본값인 `false`가 적용됩니다. 두 값은 노드 생성 시 한 번만 읽으며 런타임 arm/disarm
기능은 없습니다. `node_id`는 `0..255`만 허용하며 범위를 벗어나면 KEYA 드라이버를
생성하지 않고 노드가 종료됩니다.

## 차량 제어 계약

모든 액추에이터는 다음 하나의 명령을 구독합니다.

| 토픽 | 타입 | 발행 주체 | 조향 노드가 사용하는 필드 |
| --- | --- | --- | --- |
| `/vehicle/command` | `kaiev26_msgs/msg/ActuatorCommand` | `command_governor` (판단 계층은 `/planning/command`로 발행) | `steering_target_rad`, `brake_engage` |

`ActuatorCommand`는 `header`, `steering_target_rad`, `speed_target_mps`,
`brake_engage`만 가집니다. 조향각은 rad, 차량 좌회전 양수입니다. 기어와
enable 필드는 명령에 없습니다.

모드와 안전 상태는 STM이 발행하는 진단 메시지에서 받습니다.

| 토픽 | 타입 | 발행 주체 | 사용 내용 |
| --- | --- | --- | --- |
| `/drive/status` | `kaiev26_msgs/msg/DriveActuatorStatus` | STM_A | `mode`: `0=MANUAL`, `1=AUTO` |
| `/brake/status` | `kaiev26_msgs/msg/BrakeActuatorStatus` | STM_B | `stop_reason_bits`의 물리 E-stop bit |
| `/steering/test/estop` | `std_msgs/msg/UInt8` | 벤치 노드 | 시험 입력이 허용된 경우에만 E-stop으로 사용 |

별도 AEB 메시지나 `/safety/aeb_state`는 사용하지 않습니다. 판단 노드가 요구한
제동은 `/vehicle/command.brake_engage`로 전달됩니다. 이 필드는 AEB라는 원인을
ABI에 넣지 않은 최종 브레이크 명령입니다.

조향 노드는 다음 토픽을 발행합니다.

| 토픽 | 타입 | 내용 |
| --- | --- | --- |
| `/steering/status` | `kaiev26_msgs/msg/SteeringActuatorStatus` | 상태, 비활성 원인, timeout, 목표·실제각과 모터 진단 |
| `/steering/angle` | `std_msgs/msg/Float32` | 중앙 등가 실측 조향각, degree·좌회전 양수 |
| `/steering/enable_commanded` | `std_msgs/msg/Bool` | 노드가 KEYA에 명령한 Enable 상태 |
| `/steering/enabled` | `std_msgs/msg/Bool` | KEYA 피드백으로 확인한 Enable 상태 |

## 상태 머신

| 값 | 상태 | 모터 | 의미 |
| ---: | --- | --- | --- |
| 0 | `INIT` | Disable | 초기화 및 유효 피드백 대기 |
| 1 | `DISABLED` | Disable | MANUAL, 모드 미수신·timeout 또는 invalid mode |
| 2 | `WAIT_CMD` | Disable | AUTO와 안전 조건은 정상이지만 새 명령 대기 |
| 3 | `ACTIVE` | Enable | 새 조향 명령 실행 |
| 4 | `ESTOP` | Disable | STM_B가 보고한 물리 E-stop 또는 허용된 시험 E-stop |
| 5 | `BRAKE` | Disable | `/vehicle/command.brake_engage=true` |
| 6 | `CAN_LOST` | Disable | KEYA CAN 통신 timeout |
| 7 | `FEEDBACK_LOST` | Disable | position feedback timeout |
| 8 | `FAULT` | Disable | KEYA fault |

모터가 Enable될 수 있는 상태는 `ACTIVE`뿐입니다. 안전 입력, MANUAL 전환, CAN
손실, 모터 fault 또는 명령 timeout이 발생하면 노드는 과거 target과 pending
command를 폐기하고 모터를 Disable합니다. 조건이 복구되어도 과거 목표를 되살리지
않고, 복구 경계 이후 들어온 새 `/vehicle/command`만 받습니다.

기본 control tick은 20 ms, 즉 50 Hz입니다. `ACTIVE`에서는 rate limit을 적용한
position 명령을 매 tick 보냅니다. `/vehicle/command`의 기본 timeout은 200 ms이므로
실제 발행자는 이보다 충분히 빠르게 명령을 계속 보내야 합니다.
header stamp가 0이거나 timeout보다 오래됐거나 `steering_target_rad`가 nonfinite인
명령도 즉시 모터를 Disable하고 과거 target을 폐기합니다. 공개 상태에는 기존
`REASON_STEER_COMMAND_TIMEOUT`으로 보고하며, 이후 새롭고 유효한 명령을 기다립니다.
조향 노드는 어떤 `/vehicle/command`에서도 `brake_engage=true`를 즉시 적용하지만,
fresh한 header stamp를 가진 명령의 `brake_engage=false`만 기존 command brake를
해제합니다. 따라서 0 stamp나 오래된 명령은 걸려 있는 command brake를 해제하지
못합니다. fresh한 명령이 `brake_engage=false`와 nonfinite 조향값을 함께 보내면
조향 노드는 command brake는 해제하되 조향 명령은 거부하고 Disable 상태를
유지합니다.

## 빌드와 테스트

`kaiev26_msgs`가 같은 ROS 작업공간에 있어야 합니다. 상위 저장소에서 Control과
메시지 저장소가 나란히 있는 경우:

```bash
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths kaiev26_msgs Control/software/src \
  --packages-select kaiev26_msgs lateral_ctrl

source install/setup.bash

colcon test --packages-select lateral_ctrl
colcon test-result --verbose
```

Control 저장소만 따로 빌드한다면 먼저 현재 계약의 `kaiev26_msgs`를 빌드하고 그
`install/setup.bash`를 source해야 합니다.

## 실차 실행

CAN 인터페이스를 준비합니다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
ip -details link show can0
```

ROS 환경을 source한 뒤 실행합니다.

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py
```

다른 터미널에서 실제 연결과 소유권을 확인합니다.

```bash
ros2 topic info -v /vehicle/command
ros2 topic info -v /drive/status
ros2 topic info -v /brake/status
ros2 topic echo /steering/status
ros2 topic echo /steering/enable_commanded
candump -L can0
```

정상 통합에서는 `/vehicle/command`의 발행자가 `command_governor` 하나여야
합니다. Planning 또는 시험 명령원(실차는 `vehicle_test_scenario_runner`, HIL은
`hil_command_generator`)은 `/planning/command`로 내고 거버너가 재발행합니다.
AUTO와 유효한 KEYA feedback만 들어온 직후에는 `WAIT_CMD`이고, 그 이후 새
명령을 받아야 `ACTIVE`가 됩니다.

## 순수 조향 벤치

STM_A/B 없이 조향만 단독으로 돌릴 때 씁니다. 벤치 노드가 `/drive/status`를 직접
발행하므로 HIL bench나 실차 없이도 `ACTIVE`까지 갈 수 있고, 각도 시퀀스를 자동으로
순회하므로 캘리브레이션처럼 반복 스윕이 필요한 작업에 적합합니다.

> **HIL bench와 동시에 실행하지 마십시오.** HIL 스택의 `command_governor`와
> 이 노드가 모두 `/vehicle/command`를 발행하므로 명령이 서로 덮어써집니다.
> 이 노드는 governor를 우회하는 조향 단독 벤치 전용입니다. HIL bench에서
> 조향을 시험할 때는 `hil_command_generator`(governor 경유)만 사용합니다.
> 확인은 다음과 같이 합니다.
>
> ```bash
> ros2 topic info -v /vehicle/command   # publisher가 하나인지
> ```

다음 launch는 actuator 노드와 벤치 명령 노드를 함께 실행합니다.

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

기본 벤치 노드는 다음을 발행합니다.

| 토픽 | 내용 | 주기 |
| --- | --- | ---: |
| `/drive/status` | 시험용 AUTO 상태 | 100 ms |
| `/vehicle/command` | 절대 중앙 등가각 `+5/-5 deg`, 속도 0, 브레이크 해제 | 20 ms |
| `/steering/test/estop` | 시험 E-stop 해제 | 50 ms |

`steer_sequence_deg`는 중앙 등가각 degree·좌회전 양수입니다. 벤치 노드는 단위만
canonical radian으로 바꾸어 발행합니다. 명령은 현재각 offset이 아닌 절대
중앙 등가각입니다. 바퀴가 중앙에서 벗어난 상태에서는 첫 명령이 큰 이동을 만들 수
있습니다.

시험 E-stop은 다음처럼 조작합니다.

```bash
ros2 param set /steering_bench_test_node test_estop_active true
ros2 param set /steering_bench_test_node test_estop_active false
```

STM_A와 STM_B의 실제 모드·E-stop을 사용하고 벤치 노드만 조향 명령을 발행하게
하려면 다음처럼 실행합니다.

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false \
  safety_injection_enabled:=false
```

이 혼합 벤치에서는 STM_A가 `/drive/status`, STM_B가 `/brake/status`, 벤치 노드가
`/vehicle/command`를 각각 발행합니다. 시험 E-stop publisher는 생성되지 않습니다.

## 주요 파라미터

| 파라미터 | 기본값 | 의미 |
| --- | ---: | --- |
| `control_period_ms` | 20 | 중앙 상태 머신과 KEYA position 명령 주기 |
| `can_timeout_ms` | 500 | KEYA CAN 통신 timeout |
| `feedback_timeout_ms` | 500 | position feedback timeout |
| `mode_timeout_ms` | 1500 | `/drive/status` timeout |
| `steer_command_timeout_ms` | 200 | `ACTIVE`에서 `/vehicle/command` timeout |
| `command_count_limit` | 3279 | 명령 가능한 KEYA raw count 절댓값. nominal 중앙 등가각 약 -27.83°..+25.92° |
| `max_steering_rate_deg_s` | 60.0 | 중앙 등가각 변화율 제한 |
| `tracking_control_enabled` | false | 중앙 등가각 tracking 보정 |

count-to-angle 계수와 `counts_per_rev`는
`config/steering_motor_calibration.yaml` 한 파일이 소유합니다. Control, active HIL,
Gazebo CAN bridge가 같은 값을 사용합니다. 2026-08-21 dual wheel-hub IMU
측정에서 얻은 full 5차 `k1..k5` nominal-centered shape이며 절대 toe나 IMU 장착
yaw 보정값은 아닙니다.

벤치 시퀀스의 기본값은 다음과 같습니다.

```yaml
steer_command_publish_period_ms: 20
steer_target_change_interval_ms: 3000
steer_sequence_start_delay_sec: 3.0
steer_sequence_repeat: true
```

측정 전용 `deadzone_measure_node`는 `DEADZONE_MEASUREMENT.md`의 오프라인 절차에만
사용합니다. 각도와 전류는 `steering_actuator_node`를 읽기 전용
(`actuator_output_enabled:=false`)으로 띄워 `/steering/status`에서 받습니다. 출력이
살아 있는 actuator 노드와 동시에 실행하지 마십시오.
