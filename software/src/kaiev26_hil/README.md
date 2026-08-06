# kaiev26_hil

PC에서 STM_TEST HIL 보드를 운용하고 production control에 시험 명령을 공급하며,
반복 가능한 fault scenario를 실행하는 ROS 2 Python 패키지다. STM_A/B/STM_TEST
firmware나 production controller의 topic contract를 변경하지 않는다.

## 역할 구분

```text
Production: Planning -> /vehicle/command -> Control -> STM_A/B -> Vehicle
Bench/SIL:  Command Generator -> Control -> vcan -> kaiev26_hil can_sim
HIL:        Command Generator -> Control -> STM_A/B
                                      ^       |
                                      |       v
                                  STM_TEST emulator
```

- `vehicle_state_publisher`, visualization, Foxglove: 차량 상태 관찰
- `hil_gui`: STM_TEST 시험 조건 입력과 fault injection
- `bench.can_sim`: vcan 기반 SIL/교육용 plant. 실제 STM_TEST HIL과 별도 경로

## Nodes

### stm_test_bridge

USB serial의 단일 소유자다. 115200 8N1, LF 종료 request-response를 사용하며
주기적으로 `STATUS`를 요청한다. 연결이 끊기면 node를 종료하지 않고 재연결하고,
재연결 후 마지막 desired state를 순서대로 재적용한다.

Parameters:

- `serial_port`
- `baud_rate` (115200)
- `status_poll_hz` (5.0)
- `command_timeout_sec` (0.5)
- `retry_count` (2)
- `reconnect_interval_sec` (1.0)

ROS interface:

| Interface | Type | Purpose |
|---|---|---|
| `/hil/command` | `std_msgs/String` | 검증된 STM_TEST command 입력 |
| `/hil/command_result` | `std_msgs/String` | `OK`/`ERR` 결과 |
| `/hil/status` | `std_msgs/String` | firmware STATUS를 parse한 JSON |
| `/hil/desired_state` | `std_msgs/String` | 재연결 시 재적용할 desired state JSON |
| `/hil/connected` | `std_msgs/Bool` | serial 연결 상태 |
| `/hil/reset_safe_state` | `std_srvs/Trigger` | safe state 전체 적용 |
| `/hil/refresh_status` | `std_srvs/Trigger` | 즉시 STATUS 요청 |
| `/hil/reapply_desired_state` | `std_srvs/Trigger` | 현재 desired state 재전송 |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 연결 및 STATUS age |

STM_TEST에 적절한 기존 `kaiev26_msgs` service가 없으므로 firmware control에는
표준 String/Trigger를 선택했다. 모든 command는 `protocol.py`에서 검증되며 bridge가
한 번에 하나씩 serial request-response로 실행한다. `kaiev26_msgs`는 production
`/vehicle/command` 및 actuator status에 그대로 사용한다.

CLI 예:

```bash
ros2 topic pub --once /hil/command std_msgs/msg/String '{data: "SET MODE AUTO"}'
ros2 service call /hil/reset_safe_state std_srvs/srv/Trigger '{}'
ros2 topic echo /hil/status
```

### command_generator

시험용 Planning 대체 노드다. `kaiev26_msgs/msg/ActuatorCommand`를 production과
동일한 `/vehicle/command`에 발행한다. 별도 HIL target topic을 만들지 않는다.

Parameters:

- `enabled`
- `speed_target_mps`
- `steering_target_rad`
- `brake_engage`
- `publish_rate_hz`

실제 Planning publisher와 동시에 실행하면 안 된다. 기존
`lateral_ctrl/steering_bench_test_node`는 호환성 때문에 유지했지만 신규 HIL 시험은
이 node를 사용한다.

### hil_gui

기존 visualization과 동일하게 PyQt5를 사용한다. GUI는 serial을 열지 않고 ROS를
통해 bridge만 조작한다.

- 연결/STATUS age/EZ·KEYA health
- mode, gear, pedal, speed/RPM/voltage/current/temperature/steering
- 세 E-stop
- CAN enable, TX drop, controller fault
- brake/steering/CAN counter feedback
- Safe State, AUTO+Forward, pedal preset, E-stop, fault clear, STATUS refresh
- 시각적으로 분리된 production `/vehicle/command` 입력

연결이 끊기면 일반 command panel은 비활성화되지만 Safe State는 계속 사용할 수
있다. Safe State는 desired state에 저장되어 재연결 시 적용된다. GUI 종료 시에는
자동으로 command나 safe state를 보내지 않는다.

## Safe state

MANUAL, Forward, pedal/speed/RPM/steering 0, 세 E-stop OFF, 두 CAN ON,
모든 controller fault/TX drop OFF다.

## Scenarios

`scenario_runner`는 timestamped `hil`, `fault`, `command` step을 실행하고 다음
production status의 field equality/range/bitmask를 검사한다.

- `/drive/status`
- `/brake/status`
- `/steering/status`
- `/vehicle/state`
- `/hil/status`

결과는 scenario명, UTC 시작 시간, step 실행 latency, expectation latency,
PASS/FAIL과 실패 원인을 JSON으로 `results_dir`에 기록한다. 기본값은 실행 위치 기준
`test_results/`이며, repository root와 `software/` 아래의 이 디렉터리는 gitignored다.

기본 scenario:

- `smoke_test.yaml`
- `estop_test.yaml`
- `ez_can_timeout.yaml`
- `keya_can_timeout.yaml`

EZ timeout scenario는 현재 STM_A contract에서 확인 가능한 `part1_valid=false`만
검사한다. 이 timeout이 traction을 직접 차단하지 않는 현재 production 정책을
임의로 바꾸지 않는다.

## Launch

STM_TEST 보드는 `firmware/udev/99-kaiev26-stm.rules`가 `/dev/hil_test`라는 고정
이름을 만든다. 규칙 설치 방법과 보드별 serial은
[`../../../firmware/README.md`](../../../firmware/README.md#stm-보드-고정-장치-이름)에
있다. 규칙을 설치하지 않은 호스트에서는 `ls -l /dev/serial/by-id/`로 path를 찾아
그 값을 대신 넣는다. `ttyACM` 번호는 USB 재연결 때 바뀌므로 쓰지 않는다.

HIL bridge와 command generator:

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test
```

GUI:

```bash
ros2 launch kaiev26_hil hil_gui.launch.py \
  serial_port:=/dev/hil_test
```

Scenario:

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test \
  scenario_file:=$(ros2 pkg prefix kaiev26_hil)/share/kaiev26_hil/config/scenarios/smoke_test.yaml
```

## CAN Bench/SIL simulator

기존 `software/src/can_lab` 구현은 `kaiev26_hil.bench`로 이동했다. 기존 source-tree
실행 경로에는 compatibility wrapper가 남아 있다.

```bash
ros2 launch kaiev26_hil bench.launch.py motor_iface:=vcan0 steer_iface:=vcan1
ros2 run kaiev26_hil can_sim --motor-iface vcan0 --steer-iface vcan1 --no-gui --duration 5
```

vcan 생성과 simulator protocol은 `kaiev26_hil/bench/signals_reference.md`를 참고한다.

## Foxglove/visualization

HIL package는 `/vehicle/state` 합성을 대체하지 않는다. 기존 흐름을 그대로 실행하고
Foxglove에서 `/hil/status`, `/vehicle/state`, `/drive/status`, `/brake/status`,
`/steering/status`를 함께 확인한다.

## Build and test

```bash
cd software
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select kaiev26_hil
colcon test-result --verbose
```

필수 runtime dependency는 ROS 2 Humble, `pyserial`, `PyYAML`, PyQt5다. bench GUI는
추가로 Tkinter와 Matplotlib를 사용한다. 실제 하드웨어 배선은
`firmware/STM_TEST/README.md`를 참고한다.
