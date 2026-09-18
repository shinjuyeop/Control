# lateral_ctrl

TeamKAI-DL 차량의 KEYA 조향 액추에이터를 제어하는 ROS 2 Humble 패키지입니다.
`steering_actuator_node` 하나만 SocketCAN 인터페이스를 열고 조향 모터를 제어해야
합니다.

## Position / Torque 선택

코드 기본 `actuation_mode=position`은 기존 absolute-position 동작이다.
`actuation_mode=torque`는 같은 노드와 안전 상태 머신에서 reusable
`TorqueSteeringController`로 calibrated wheel-angle 외부 PD loop를 닫는다.
두 모드는 startup-only이며 KEYA utility의 **position: 0020=3 / torque: 0020=2**
설정을 Host가 자동 변경하거나 검증하지 않는다.

Launch는 `params_file` 위에 `control_tuning_file`의 기존 공통 설정과
`tuning_preset` 프리셋(기본 `driving`)을 읽는다. 다른 조건은 `airborne` / `ground_stationary`다.
프리셋은 `control_tuning.presets/`의 조건별 YAML에 저장한다. 현재 기본은 `torque`이며,
명시적 `actuation_mode:=position`/`torque`가 프리셋보다
우선한다. 기존 HIL은 position 모델에 맞춰 position을 명시한다.

공통 설정 편집은 `ros2 run control_tuning control_tuning_tui`를 사용한다.
P·D는 각각 `A/deg`, `A/(deg/s)`의 명령 환산 단위로 표시한다. 기존 ROS 파라미터명과
내부 raw 단위는 유지한다. [편집·저장·적용 방법](../control_tuning/README.md)을 참고한다.

Torque는 LEFT-positive road-wheel target을 직접 받고, `P=peak/error_full_scale*error`,
`D=-Kd*measured_rate`를 계산한다. Ki=0, 기본 full-scale-error=0.5°로 작은 오차에도
높은 torque를 사용하며, 실제각 차분의 LPF로 damping한다. raw peak/sustained와 시간
예산, calibrated 위치 한계·바깥 방향 차단, 실제 과속·freshness 보호는 별도로 적용한다.
정수 A인 current telemetry는 진단용이다. saturation은 fault가 아니다.

실차에서 바로 사용할 **전체 parameter/default 표, 부호 검증, 최초 raw200 bring-up,
1°→5° step과 지면 부하 튜닝, 긴급 정지 및 Foxglove 토픽**은
[TORQUE_CONTROL.md](TORQUE_CONTROL.md)를 따른다. Torque 운용 중에는 아래 기존
position 전용 rate-limiter/position frame 설명 대신 해당 문서가 적용된다.
무부하 T0-T2 측정은 `torque_sysid_bench.launch.py`, MCAP 후처리는
`torque_sysid_metrics.py`를 사용하며 외장 ROS artifact 저장소가 필요하다.

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

두 파라미터의 코드 기본값은 `false`입니다. 하지만 보정 파라미터도 필수이므로
params file 없는 `ros2 run`을 읽기 전용 실행 방법으로 사용하지 않습니다.
위 launch에 `actuator_output_enabled:=false`를 지정합니다. 두 값은 노드 생성 시 한 번만 읽으며 런타임 arm/disarm
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

직진 계열 시험은 런타임 파라미터 `steering_enabled=false`로 조향 모터만
Disable합니다. 기본값은 `true`입니다. 출력이 허용된 노드는 설정을 받을 때
KEYA Disable을 보내고 이전 목표를 폐기하며, `false` 동안 위치·Enable 명령을
보내지 않습니다. CAN 피드백 조회와 상태 발행은 계속됩니다.
이 상태에서는 유효한 조향 명령이 와도 `WAIT_CMD`를 유지하며, 기존 안전 원인이
있으면 해당 상태를 우선 보고합니다. `true`로 복원해도 기존 출력 허용·AUTO·안전
조건과 새 명령이 모두 필요합니다. 읽기 전용 설정은 이 파라미터로 해제되지 않습니다.
시험 실행기가 설정 응답을 확인한 뒤 주행 명령을 발행합니다. 응답은 설정 수락을
뜻하며 실제 모터 Disable 피드백은 `/steering/status.enabled_feedback_valid`와
`enabled_feedback`으로 확인합니다. 마지막 설정은 조향 노드 재시작 또는 다음
설정까지 유지됩니다.

모드와 안전 상태는 STM이 발행하는 진단 메시지에서 받습니다.

| 토픽 | 타입 | 발행 주체 | 사용 내용 |
| --- | --- | --- | --- |
| `/drive/status` | `kaiev26_msgs/msg/DriveActuatorStatus` | STM_A | `mode`: `0=MANUAL`, `1=AUTO`; `speed_mps`, `part1_valid`: 조향 허용 속력 |
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
| `/steering/target_motor_position_count` | `std_msgs/msg/Int32` | `target_wheel_angle_rad`를 기존 calibration으로 역변환한 KEYA 목표 count. dashboard 표시용 |
| `/steering/enable_commanded` | `std_msgs/msg/Bool` | 노드가 KEYA에 명령한 Enable 상태 |
| `/steering/enabled` | `std_msgs/msg/Bool` | KEYA 피드백으로 확인한 Enable 상태 |

추종 오차에 따른 경고·고장 판정은 비활성화되어 있습니다.
`tracking_error_warning_deg`, `tracking_error_fault_deg`,
`tracking_error_fault_duration_ms` 파라미터는 더 이상 사용하지 않습니다.
`/steering/status`의 `tracking_error_warning`과 `tracking_error_fault`는 항상
`false`이며, 계측용 `tracking_error_rad`와 기존 조향 보정 제어는 유지합니다.

## 상태 머신

| 값 | 상태 | 모터 | 의미 |
| ---: | --- | --- | --- |
| 0 | `INIT` | Disable | 초기화 및 유효 피드백 대기 |
| 1 | `DISABLED` | Disable | MANUAL, 모드 미수신·timeout 또는 invalid mode |
| 2 | `WAIT_CMD` | Disable | 새 명령 대기, 조향 비활성 설정 또는 속력 조건 미충족 |
| 3 | `ACTIVE` | Enable | 새 조향 명령 실행 |
| 4 | `ESTOP` | Disable | STM_B가 보고한 물리 E-stop 또는 허용된 시험 E-stop |
| 5 | `BRAKE` | Disable | `/vehicle/command.brake_engage=true` |
| 6 | `CAN_LOST` | Disable | KEYA CAN 통신 timeout |
| 7 | `FEEDBACK_LOST` | Disable | position feedback timeout |
| 8 | `FAULT` | Disable | KEYA fault 또는 Host torque safety fault |

모터가 Enable될 수 있는 상태는 `ACTIVE`뿐입니다. 안전 입력, MANUAL 전환, CAN
손실, 모터 fault 또는 명령 timeout이 발생하면 노드는 과거 target과 pending
command를 폐기하고 모터를 Disable합니다. 조건이 복구되어도 과거 목표를 되살리지
않고, 차단 사건 이후 받은 fresh pending `/vehicle/command`가 필요합니다.
차단 중 받은 명령도 복구 시점에 fresh하면 사용될 수 있습니다. Host torque
safety latch는 [별도 복구 절차](TORQUE_CONTROL.md#안전-동작)를 따릅니다.

정지 상태의 조향 부하를 줄이기 위해 `/drive/status`의 `part1_valid=true`이고
유한한 `abs(speed_mps) >= minimum_steering_speed_kph / 3.6`일 때 조향을 허용합니다.
코드 기본 최저속도는 1 km/h이며 현재 YAML은 정지 시험을 위해 0 km/h로 설정되어
있습니다. 전·후진 모두 같은 절댓값 조건을 쓰며, 미수신 또는 마지막 수신 후 `drive_status_timeout_ms`
(기본 500 ms)를 넘겨도 Disable합니다. 저속·무효 피드백 수신 시 즉시 Disable하고
기존 목표를 폐기합니다. 조건 미충족 동안에는 위치·Enable 명령을 보내지 않으며,
속력 조건 회복 후 새 조향 명령이 필요합니다. 다른 안전 원인이 없으면
`WAIT_CMD`와 `disable_reason=NONE`으로 표시하고 속력 조건은 경고 로그로 알립니다.
`speed_mps`는 STM_A가 구동모터 RPM에서 환산하고 0 RPM dropout을 보정한 값이므로
실제 정지 순간과 Disable 사이에는 속도 피드백 및 제어 주기 지연이 있습니다.

기본 control tick은 20 ms, 즉 50 Hz입니다. `position` 모드의 `ACTIVE`에서는
rate limit을 적용한 position 명령을 매 tick 보냅니다. 현재 YAML의 `torque` 모드는
direct target을 PD로 제어하고 실제 조향속도 120 deg/s 초과를 fault로 latch합니다. `/vehicle/command`의 기본 timeout은 200 ms이므로
실제 발행자는 이보다 충분히 빠르게 명령을 계속 보내야 합니다.
header stamp가 0이거나 timeout보다 오래됐거나 `steering_target_rad`가 nonfinite인
명령도 즉시 모터를 Disable하고 과거 target을 폐기합니다. 공개 상태에는 기존
`REASON_STEER_COMMAND_TIMEOUT`으로 보고하며, 이후 새롭고 유효한 명령을 기다립니다.
조향 노드는 어떤 `/vehicle/command`에서도 `brake_engage=true`를 즉시 적용하지만,
fresh한 header stamp를 가진 명령의 `brake_engage=false`만 기존 command brake를
해제합니다. 따라서 0 stamp나 오래된 명령은 걸려 있는 command brake를 해제하지
못합니다. fresh한 명령이 `brake_engage=false`와 nonfinite 조향값을 함께 보내면
조향 노드는 command brake는 해제하되 조향 명령은 거부하고 Disable 상태를
유지합니다. `minimum_steering_speed_kph`는 조향 허용 최저 속도이며 기본값은
코드에서는 `1.0`, 현재 YAML에서는 `0.0`입니다. `0.0`이면 유효하고 fresh한 `/drive/status`는
계속 요구하면서
정지 상태 조향을 허용합니다.

## 빌드와 테스트

`kaiev26_msgs`와 `Control`을 `~/KAI_ws/src/`에 배치한 경우:

```bash
cd ~/KAI_ws
si
colcon build --symlink-install --packages-up-to lateral_ctrl

si

colcon test --packages-select lateral_ctrl
colcon test-result --verbose
```

Control 패키지만 선택해 빌드한다면 먼저 현재 계약의 `kaiev26_msgs`를 빌드하고,
`cd ~/KAI_ws` 후 `si`로 빌드 결과를 불러와야 합니다.

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
AUTO, 유효한 KEYA feedback과 구동 속력 조건을 충족한 뒤 새 명령을 받아야
`ACTIVE`가 됩니다.

## 순수 조향 벤치

벤치 노드는 `/drive/status`와 각도 시퀀스를 직접 발행합니다. 현재 기본 시험
status는 유효한 구동 속력을 제공하지 않으므로 조향은 Disable 상태를 유지합니다.
최저속도 설정이 0이어도 `part1_valid`와 fresh한 속도 피드백은 필요합니다.

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
| `drive_status_timeout_ms` | 500 | 조향 속력 허용에 사용하는 `/drive/status` 수신 timeout |
| `steer_command_timeout_ms` | 200 | `ACTIVE`에서 `/vehicle/command` timeout |
| `command_count_limit` | 3279 | 명령 가능한 KEYA raw count 절댓값. nominal 중앙 등가각 약 -27.83°..+25.92° |
| `minimum_steering_speed_kph` | YAML 0.0 / 코드 1.0 | 조향 허용 최저 차속; YAML은 정지 시험용 임시 설정 |
| `max_steering_rate_deg_s` | 60.0 | position 모드 목표각 변화율 제한; torque 안전 한계와 별개 |
| `tracking_control_enabled` | false | 중앙 등가각 tracking 보정 |

count-to-angle 계수와 `counts_per_rev`는
`config/steering_motor_calibration.yaml` 한 파일이 소유합니다. Control, active HIL,
Gazebo CAN bridge가 같은 값을 사용합니다. 2026-08-21 조향 캘리브레이션 계측에서
얻은 full 5차 `k1..k5` nominal-centered shape이며 절대 toe 보정값은 아닙니다.

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
