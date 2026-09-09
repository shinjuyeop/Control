# kaiev26_hil

PC에서 STM_TEST HIL 보드를 운용하고 production control에 시험 명령을 공급하며,
반복 가능한 fault scenario를 실행하는 ROS 2 Python 패키지다. STM_A/B/STM_TEST
firmware나 production controller의 topic contract를 변경하지 않는다.

## 역할 구분

```text
Production: Planning -> /planning/command -> command_governor -> /vehicle/command -> STM_A/B, lateral_ctrl
vcan SIL:   Test commands -> Control -> vcan -> kaiev26_hil vcan_sim
HIL:        HIL command generator -> /planning/command -> command_governor -> /vehicle/command -> STM_A/B
                                                                                ^       |
                                                                                |       v
                                                                            STM_TEST emulator
```

HIL도 실차와 같은 governor 경로를 지난다. `/vehicle/command`의 발행자는
`command_governor` 하나뿐이다
([`../command_governor/README.md`](../command_governor/README.md)).

- `vehicle_state_publisher`, visualization, Foxglove: 차량 상태 관찰
- `hil_gui`: STM_TEST 시험 조건 입력과 fault injection
- `vcan_sim`: vcan 기반 SIL/교육용 plant. 실제 STM_TEST HIL과 별도 경로

Web 접속, host 준비, 전체 HIL 실행, 브라우저에서의 기능·시나리오 시험과
정상 종료를 한 번에 따라 하려면
[`../../../docs/HIL_RUNBOOK.md`](../../../docs/HIL_RUNBOOK.md)를 사용한다.
이 문서는 ROS node·topic·service·GUI 계약만 설명한다. 핀 배치, UART
명령, CAN ID·payload, Plant 수식과 timing은
[`../../../firmware/STM_TEST/README.md`](../../../firmware/STM_TEST/README.md)가 기준이다.

## Web HIL Console

React 기반 Web 운용 화면은 [WEB_CONSOLE.md](WEB_CONSOLE.md)를 따른다.
벤치·구동·조향·브레이크/안전·고장 주입·시나리오·가상 LCD·진단 페이지를 제공한다.
전역 Operator Remote에서 Mode/Gear/Pedal/E-stop을 조작하며, 속도는 km/h·조향은 degree다.
주행 명령과 engineering 설정은 기능 페이지에 두고, 상단에서 ARMED 상태를 확인한다.
차량 dashboard 진단과 ROS 수신 주기·CAN age도 확인할 수 있다.
기존 PyQt GUI는 유지한다. Web 전용 launch는 명령 생성기를 DISARMED로 시작한다.

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

### hil_command_generator

시험용 Planning 대체 노드다. `kaiev26_msgs/msg/ActuatorCommand`를 production
Planning과 동일한 `/planning/command`에 발행하고, `command_governor`가 이것을
`/vehicle/command`로 재발행한다. 별도 HIL target topic을 만들지 않는다.

Parameters:

- `enabled`
- `speed_target_mps`
- `steering_target_rad`
- `brake_engage`
- `publish_rate_hz`
- `command_topic` (기본 `/planning/command`, 생성자에서 한 번만 읽음)
- `web_heartbeat_timeout_sec` (기본 0, Web 전용 launch는 3초; 시작 시 설정하는 read-only parameter)

실제 Planning publisher와 동시에 실행하면 안 된다. `/vehicle/command`를 직접
발행하는 기존 `lateral_ctrl/steering_bench_test_node`는 호환성 때문에 유지했지만
governor를 우회하므로, 신규 HIL 시험은 이 node를 사용한다.

실차 sysid에서는 이 node를 open-loop 시험입력으로만 사용한다. STM_A AUTO frame의
phase-current 상한은 현재 소스 기준 480 A이며 배터리 전류가 아니므로 EZkontrol 내부 active current limit이 승인된
값인지 export로 확인하거나 STM_A 상한을 안전하게 입증된 값으로 낮추기 전에는
움직이는 실차에서 `enabled=true`로 두지 않는다. 시작할 때
`ros2 topic info -v /vehicle/command`로 publisher가 하나인지 확인하고, 속도 target은
0.25 m/s씩 1초 간격으로 올려 첫 1 m/s crawl을 만든다. 큰 속도 step, 무인 운행,
communication timeout을 정상 정지수단으로 쓰는 것은 이 경로의 계약이 아니다.

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
- 별도 tab으로 분리된 production `/planning/command` 입력 (governor 경유로
  `/vehicle/command`가 됨)

연결이 끊기면 일반 command panel은 비활성화되지만 Safe State는 계속 사용할 수
있다. Safe State는 desired state에 저장되어 재연결 시 적용된다. GUI 종료 시에는
자동으로 command나 safe state를 보내지 않는다.

## Safe state

Web의 Bench Reset(기존 Safe State)은 `/hil/reset_safe_state`를 그대로 사용한다.
bridge의 Safe State는 `HilDesiredState()`의 기본값이다. MCU 부팅 기본값과 달리
plant가 ON이며, E-stop 입력은 해제된다. bridge는 이 상태를
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

| protocol | firmware가 추가한 것 |
|---:|---|
| 1 | 기본 STATUS |
| 2 | plant 및 command 관측 field |
| 3 | FDCAN protocol status field (`*_busoff`, `*_errpass`, `*_warn`, `*_tec`, `*_rec`) |

parser는 알 수 없는 STATUS key를 만나면 예외를 던진다. 따라서 firmware가 field를
추가하면 bridge를 함께 올려야 하며, `protocol` 값도 같이 올려야 한다. 반대로 낮은
protocol의 firmware는 계속 붙는다. 없는 field는 dataclass 기본값으로 채워지고
protocol 3 이전 firmware에서는 bus status가 전부 0으로 보인다.

## Scenarios

`hil_scenario_runner`는 timestamped `hil`, `fault`, `command` step을 실행하고 다음
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

2026-09-08 대체 STM_TEST 보드와 `/dev/hil_test` 매핑을 복구하고 bench 연동을
확인했다. STM_TEST 브레이크 PWM 판정값도 현재 STM_B 출력에 동기화했다.
검증 결과와 남은 사항은 HIL runbook 15절을 따른다.

HIL bridge, command generator와 GUI:

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test
```

이 launch가 실행하는 것은 `stm_test_bridge`, `hil_command_generator`,
`command_governor`, `hil_gui`다.
STM_A/B용 micro-ROS Agent, `lateral_ctrl`, `vehicle_state_publisher`는 포함하지 않는다.
현재 EZ CAN만 연결된 bench에서는 Agent 두 개와 위 launch까지만 실행하고 KEYA
`NO RX / NOT CONNECTED`를 정상 상태로 취급한다.

`gui`와 `scenario_file`은 서로 독립인 인자다. GUI 없이 bridge와 generator만
띄우려면 `gui:=false`만 주면 된다.

Hardware HIL bench 전체를 한 번에:

```bash
ros2 launch kaiev26_hil hil_bench.launch.py \
  serial_port:=/dev/hil_test \
  stm_a_port:=/dev/hil_a \
  stm_b_port:=/dev/hil_b
```

`hil_bench.launch.py`는 위 `hil.launch.py`에 STM_A/B micro-ROS Agent 두 개를
더한 것이다. Agent를 이미 따로 실행 중이면 `agents:=false`를 준다. 로그가 한
터미널에 모이므로 Agent 로그만 읽어야 할 때는 터미널을 나눠 실행한다.
`steering:=true`는 can0의 `lateral_ctrl`을, `state:=true`는
`vehicle_state_publisher`를 함께 띄운다(기본은 둘 다 꺼짐). `gui:=false`는
headless 실행이다.

Scenario. GUI 대신 scenario runner를 붙이려면 `gui:=false`와 `scenario_file`을
함께 준다. `hil_bench.launch.py`도 같은 인자를 통과시키므로 Agent까지 포함해
한 명령으로 돌릴 수 있다.

```bash
ros2 launch kaiev26_hil hil_bench.launch.py \
  gui:=false steering:=true \
  scenario_file:=$(ros2 pkg prefix kaiev26_hil)/share/kaiev26_hil/config/scenarios/smoke_test.yaml
```

scenario에는 fault 주입 전에 해당 경로가 정상임을 확인하는 전제조건
expectation이 들어 있다. 경로가 처음부터 죽어 있으면 결과 expectation이
공허하게 통과할 수 있기 때문으로, 전제가 FAIL하면 시험 환경부터 고친다.

## vcan SIL simulator

`vcan`에서 구동·조향 CAN 동작을 시험하는 software-only simulator다. 구현은
`kaiev26_hil.vcan_sim` 아래에 있으며 물리 STM 보드나 serial 장치를 사용하지
않는다.

```bash
ros2 launch kaiev26_hil vcan_sim.launch.py motor_iface:=vcan0 steer_iface:=vcan1
ros2 run kaiev26_hil vcan_sim --motor-iface vcan0 --steer-iface vcan1 --no-gui --duration 5
```

vcan 생성과 simulator protocol은
`kaiev26_hil/vcan_sim/signals_reference.md`를 참고한다.

## Foxglove/visualization

HIL package는 `/vehicle/state` 합성을 대체하지 않는다. 기존 흐름을 그대로 실행하고
Foxglove에서 `/hil/status`, `/vehicle/state`, `/drive/status`, `/brake/status`,
`/steering/status`를 함께 확인한다.

## Build and test

```bash
cd "${KAIEV26_WS:?set KAIEV26_WS to the ROS 2 workspace root}"
source /opt/ros/humble/setup.zsh
colcon build --symlink-install --packages-up-to kaiev26_hil
source install/setup.zsh
colcon test --packages-select control_gui_common kaiev26_hil
colcon test-result --verbose
```

`test/test_vcan_sim_*.py`는 vcan simulator의 CAN encode/decode, plant 물리와
runtime 진단을 덮고 `kaiev26_hil.vcan_sim`을 직접 import한다.

`test/test_hil_gui_smoke.py`는 `QT_QPA_PLATFORM=offscreen`으로 GUI window를 실제로
구성하고 STATUS를 rendering하므로 X11 없이도 실행된다. PyQt5가 없으면 skip된다.
사용하는 STATUS 문자열은 firmware README의 예시와 같은 형식이므로, firmware가
field를 바꾸면 bench가 아니라 이 test에서 먼저 드러난다.

필수 runtime dependency는 ROS 2 Humble, `pyserial`, `PyYAML`, PyQt5다. vcan
simulator GUI는 추가로 Tkinter와 Matplotlib를 사용한다. host 준비와 전체 build
대상은 [HIL runbook](../../../docs/HIL_RUNBOOK.md#5-소프트웨어-빌드와-테스트)를
따른다.
