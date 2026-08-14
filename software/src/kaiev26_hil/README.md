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

SSH/X11 접속, host 준비, STM_A/B Agent, STM_TEST GUI, 기능 시험, KEYA 확장 및
정상 종료를 한 번에 따라 하려면
[`../../../docs/HIL_RUNBOOK.md`](../../../docs/HIL_RUNBOOK.md)를 사용한다.
이 문서는 ROS node·topic·service·GUI 계약만 설명한다. 핀 배치, UART
명령, CAN ID·payload, Plant 수식과 timing은
[`../../../firmware/STM_TEST/README.md`](../../../firmware/STM_TEST/README.md)가 기준이다.

## Nodes

### stm_test_bridge

USB serial의 단일 소유자다. 115200 8N1, LF 종료 request-response를 사용하며
주기적으로 `STATUS`를 요청한다. 연결이 끊기면 node를 종료하지 않고 재연결하고,
재연결 후 마지막 desired state를 순서대로 재적용한다.

Parameters:

괄호 안은 node 선언 기본값이고, launch는 `config/hil.yaml`을 함께 적용한다.

- `serial_port`
- `baud_rate` (115200)
- `status_poll_hz` (5.0, `hil.yaml`에서 10.0으로 올림)
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

- 신호 방향별 화면: STM_TEST 주입 → controller command 관측 → emulated feedback
- mode/gear/pedal DAC와 세 E-stop 주입
- EZ command의 RUN/control mode/RPM/current/plant target 관측
- brake PWM과 KEYA enable/target command 관측
- `Closed-loop simulation` / `Direct feedback override` 응답 모드
- CAN enable, TX drop, controller fault는 별도 advanced tab
- Safe State, AUTO+Forward, pedal preset, E-stop, fault clear, STATUS refresh
- 별도 tab으로 분리된 production `/vehicle/command` 입력

연결이 끊기면 일반 command panel은 비활성화되지만 Safe State는 계속 사용할 수
있다. Safe State는 desired state에 저장되어 재연결 시 적용된다. GUI 종료 시에는
자동으로 command나 safe state를 보내지 않는다.

## Safe state

bridge의 Safe State는 firmware의
[부팅 기본값](../../../firmware/STM_TEST/README.md#81-부팅-기본값)과 같다. bridge는 이 상태를
desired state에 저장하고 serial 재연결 후 재적용한다. command generator의
`enabled`는 별도 상태이므로 Safe State 적용 전에 따로 끄는다.

## Plant

Plant를 켜면 `/vehicle/command` -> STM_A -> EZ CAN -> STM_TEST -> EZ CAN ->
`/drive/status` 경로로 피드백이 닫힌다. Plant 모델, 계수와 RPM 계산은 firmware
[EZkontrol emulator 명세](../../../firmware/STM_TEST/README.md#82-ezkontrol-emulator-fdcan1-pa11pa12)를 따른다.

plant가 켜져 있으면 motion은 명령을 따라가므로 desired state의 speed/RPM과 다른 것이
정상이다. `stm_test_bridge`는 이때 speed/RPM 불일치를 diagnostics에서 제외한다.
기존 scenario는 plant OFF 기준이며 그대로 동작한다.

GUI에서는 `plant`라는 내부 이름 대신 응답 모드로 표시한다. Closed-loop에서는
SPEED/RPM 직접 입력을 비활성화하고 STM_A에서 실제 수신한 command RPM/current와
계산된 target RPM을 보여준다. Direct override로 전환할 때는 직전 plant 출력 RPM을
desired state에 저장하므로 재연결 후에도 같은 값에서 이어진다.

## KEYA steering

GUI는 KEYA 패널을 항상 표시하고 CAN 주변장치 초기화(`keya_ok`)와
실제 controller RX를 구분한다. 유효 RX가 없으면 `NO RX / NOT CONNECTED`,
이후 수신이 끊기면 `STALE RX`로 표시한다. 현재 EZ CAN만 연결된
bench에서는 `NO RX / NOT CONNECTED`가 정상이다. KEYA CAN ID, payload,
heartbeat와 조향 추종 모델은 firmware
[KEYA emulator 명세](../../../firmware/STM_TEST/README.md#83-keya-emulator-fdcan2-pb12pb13)에 있다.

## Firmware protocol compatibility

bridge는 먼저 STATUS의 `protocol`을 읽고 desired state를 재적용한다. Protocol 1
구형 firmware에는 지원하지 않는 `SET PLANT`를 보내지 않으므로 새 GUI도 연결할 수
있지만, command 관측값과 RX age는 unavailable로 표시된다.

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

HIL bridge, command generator와 GUI:

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test
```

이 launch가 실행하는 것은 `stm_test_bridge`, `hil_command_generator`, `hil_gui`다.
STM_A/B용 micro-ROS Agent, `lateral_ctrl`, `vehicle_state_publisher`는 포함하지 않는다.
현재 EZ CAN만 연결된 bench에서는 Agent 두 개와 위 launch까지만 실행하고 KEYA
`NO RX / NOT CONNECTED`를 정상 상태로 취급한다.

`gui`와 `scenario_file`은 서로 독립인 인자다. GUI 없이 bridge와 generator만
띄우려면 `gui:=false`만 주면 된다.

Bench 전체를 한 번에:

```bash
ros2 launch kaiev26_hil hil_bench.launch.py \
  serial_port:=/dev/hil_test \
  stm_a_port:=/dev/hil_a \
  stm_b_port:=/dev/hil_b
```

`hil_bench.launch.py`는 위 `hil.launch.py`에 STM_A/B micro-ROS Agent 두 개를
더한 것이다. Agent를 이미 따로 실행 중이면 `agents:=false`를 준다. 로그가 한
터미널에 모이므로 Agent 로그만 읽어야 할 때는 터미널을 나눠 실행한다. 이 launch도
`lateral_ctrl`과 `vehicle_state_publisher`는 포함하지 않는다.

Scenario. GUI 대신 scenario runner를 붙이려면 `gui:=false`와 `scenario_file`을
함께 준다.

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test \
  gui:=false \
  scenario_file:=$(ros2 pkg prefix kaiev26_hil)/share/kaiev26_hil/config/scenarios/smoke_test.yaml
```

## CAN Bench/SIL simulator

`vcan`에서 구동·조향 CAN 동작을 시험하는 simulator다. 구현은 `kaiev26_hil.bench`
하나이며, 과거 `software/src/can_lab`의 source-tree 실행 경로와 compatibility
wrapper는 제거했다.

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
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-select kaiev26_hil
source ~/KAI_ws/install/setup.zsh
colcon test --packages-select kaiev26_hil
colcon test-result --verbose
```

`test/test_bench_*.py`는 bench simulator의 CAN encode/decode, plant 물리와 runtime
진단을 덮는다. can_lab 패키지에 있던 test를 옮긴 것이라 `kaiev26_hil.bench`를
직접 import한다.

`test/test_hil_gui_smoke.py`는 `QT_QPA_PLATFORM=offscreen`으로 GUI window를 실제로
구성하고 STATUS를 rendering하므로 X11 없이도 실행된다. PyQt5가 없으면 skip된다.
사용하는 STATUS 문자열은 firmware README의 예시와 같은 형식이므로, firmware가
field를 바꾸면 bench가 아니라 이 test에서 먼저 드러난다.

필수 runtime dependency는 ROS 2 Humble, `pyserial`, `PyYAML`, PyQt5다. bench GUI는
추가로 Tkinter와 Matplotlib를 사용한다. host 준비와 전체 build 대상은
[HIL runbook](../../../docs/HIL_RUNBOOK.md#5-소프트웨어-빌드와-테스트)를 따른다.
