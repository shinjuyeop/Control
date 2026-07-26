# lateral_ctrl

> **인터페이스 변경 (2026-07-22)**: canonical 통합 계약 적용.
> 조향 명령은 `/vehicle/actuator_command`(kaiev26_msgs/ActuatorCommand, rad·좌회전 양수),
> 모드는 `/drive/status`의 `mode` 필드, 물리 E-stop은 `/brake/status`의
> `stop_reason_bits` ESTOP bit, AEB는 `/safety/aeb_state`(AEBState)로 받는다.
> 조향 status는 `kaiev26_msgs/SteeringActuatorStatus`(rad)로 발행한다.
> 구 `/cmd/steer`·`/mode`·`/estop`·`/aeb`·`/ctrl/steer` 인터페이스는 제거되었다.


TeamKAI-DL 차량의 KEYA 조향 액추에이터를 제어하는 ROS 2 Humble 패키지입니다.
현재 구조에서는 `steering_actuator_node` 하나만 SocketCAN 인터페이스를 열고 모터를
제어합니다. 과거의 `steering_control_node`와 `steering_feedback_node`는 제거되었으며,
같은 CAN 모터를 제어하는 다른 노드를 동시에 실행하면 안 됩니다.

> **실차 주의:** `steering_actuator.launch.py`는 `hardware_can_enabled=true`,
> `actuator_output_enabled=true`가 기본값입니다 — 띄우는 즉시 CAN 소켓이 열리고
> 모터로 프레임이 나갑니다. 끄려면 두 인자를 `false`로 넘기십시오. 노드에는
> 런타임 arm/disarm 경로가 없으므로(`outputArmed()`는 생성자에서 한 번만 읽는다)
> 무장 여부는 기동 시점에 확정됩니다. 띄우기 전에 바퀴를 공중에 띄우고, 조향
> 기구 주변을 비우고, 물리 E-STOP을 준비하고, CAN 인터페이스와 기계적 중앙을
> 확인하십시오.

## 현재 구성

실행 파일은 두 개입니다.

| 실행 파일 | 역할 |
| --- | --- |
| `steering_actuator_node` | `/drive/status`, `/vehicle/actuator_command`, 안전 입력과 KEYA 피드백을 받아 모터를 제어하고 상태를 발행 |
| `steering_bench_test_node` | 선택적으로 시험 mode·안전 입력을 만들고 절대 바퀴각 시퀀스를 발행 |

주요 내부 모듈은 다음과 같습니다.

- `KeyaCanDriver`: KEYA 확장 CAN frame 송수신 및 feedback parsing
- `SteeringAngleConverter`: 모터 encoder count, 모터각, 바퀴각 변환과 각도 제한
- `WheelAngleTrackingController`: 선택 가능한 바퀴각 tracking 보정; 기본 비활성
- `SteeringActuatorNode`: 50 Hz 중앙 상태 머신과 안전 전환

## 실행 모드 한눈에 보기

조향 시험과 실전 실행은 다음 세 가지 중 하나를 선택합니다. 같은 모터를 제어하는
launch를 둘 이상 동시에 실행하면 안 됩니다.

| 버전 | 목적 | `/drive/status` | `/brake/status` | `/safety/aeb_state` | `/vehicle/actuator_command` |
| --- | --- | --- | --- | --- | --- |
| **A 실전** | 전체 차량 시스템 통합 | STM_A | STM_B | 판단 노드 | 판단 노드 |
| **B-1 순수 벤치** | STM 없이 조향 장치만 시험 | 벤치 노드 | `/steering/test/estop` | `/steering/test/aeb` | 벤치 노드 |
| **B-2 혼합 벤치** | 실제 STM 안전 신호와 모터 연동 시험 | STM_A | STM_B | 판단 노드 또는 미발행 | 벤치 노드 |

선택 기준은 간단합니다.

- 판단 노드가 `/vehicle/actuator_command`와 `/safety/aeb_state`까지 발행한다면 **A 실전**을 사용합니다.
- STM 없이 모터와 안전 상태 머신만 시험하려면 **B-1 순수 벤치**를 사용합니다.
- STM_A의 mode와 STM_B의 E-STOP까지 함께 시험하려면 **B-2 혼합 벤치**를 사용합니다.

각 버전의 핵심 실행 명령은 다음과 같습니다. 모든 새 터미널에서 먼저 ROS 환경을
설정해야 합니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

**A 실전:**

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py
```

**B-1 순수 벤치:**

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

**B-2 혼합 벤치:**

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false \
  safety_injection_enabled:=false
```

혼합 벤치의 `drive_mode_publish_enabled:=false`는 벤치 노드의 시험 `/drive/status`
publisher를 제거하여 STM_A의 실제 `/drive/status`만 사용하게 합니다.
`safety_injection_enabled:=false`는 `/steering/test/estop`과
`/steering/test/aeb` publisher만 제거합니다. 이 옵션은 실제 `/brake/status` 또는 `/safety/aeb_state`
안전 처리를 끄지 않습니다. 혼합 벤치에서도 `/vehicle/actuator_command`는 벤치 노드가 50 Hz로
계속 발행합니다.

## 안전 상태 머신

| 값 | 상태 | 모터 | 의미 |
| ---: | --- | --- | --- |
| 0 | `INIT` | Disable | CAN 초기화 및 유효 피드백 대기 |
| 1 | `DISABLED` | Disable | mode 미수신/timeout, MANUAL 또는 invalid mode |
| 2 | `WAIT_CMD` | Disable | AUTO와 안전 조건은 정상이지만 복구 이후 새 명령 대기 |
| 3 | `ACTIVE` | Enable | 새 명령을 rate limit과 선택적 tracking 보정으로 실행 |
| 4 | `ESTOP` | Disable | E-STOP 활성 |
| 5 | `AEB` | Disable | AEB 활성 |
| 6 | `CAN_LOST` | Disable | CAN heartbeat/응답 timeout |
| 7 | `FEEDBACK_LOST` | Disable | CAN은 살아 있지만 position feedback timeout |
| 8 | `FAULT` | Disable | 모터 fault 감지 |

모터가 Enable될 수 있는 상태는 `ACTIVE`뿐입니다. `WAIT_CMD`에서는 Enable frame,
position frame, holding torque, tracking update를 전송하지 않습니다.

모터의 제어 모드는 펌웨어 설정에서 absolute position mode로 구성해야 합니다. 노드는
시작 시 별도의 mode 변경 frame을 전송하지 않습니다.
Position payload는 장비 데이터시트에 따라 signed 32-bit big-endian 순서를 사용합니다.
`ACTIVE`에서는 50 Hz control tick마다 현재 rate-limited 목표의 CAN position frame을
반복 전송합니다. `/vehicle/actuator_command`도 기본 200 ms timeout보다 빠른 50 Hz로 계속 받아야 합니다.

E-STOP, AEB, MANUAL/invalid mode, mode timeout, steer command timeout, CAN/position
feedback 손실, motor fault가 발생하면 모터를 즉시 Disable하고 현재 target, pending
target, command generation/timestamp, tracking integral/correction을 모두 폐기합니다.
안전 조건이 복구되어도 과거 목표는 복원하지 않으며 `WAIT_CMD` 진입 이후 수신한
새 `/vehicle/actuator_command`가 있어야 다시 활성화됩니다.

`WAIT_CMD`에서 새 명령을 받으면 control tick에서 안전 조건을 다시 확인하고 다음
순서로 활성화합니다.

1. commanded/corrected angle을 현재 실제 바퀴각으로 초기화합니다.
2. 새 target을 저장하고 tracking controller를 reset합니다.
3. Enable frame을 전송합니다.
4. 현재 실제각에 해당하는 position frame을 한 번 전송합니다.
5. `ACTIVE`로 전환합니다.
6. 다음 control tick부터 목표각 방향으로 rate limiting을 적용합니다.

`/vehicle/actuator_command` callback은 수신값과 시각만 저장하며 CAN frame을 직접 전송하거나 상태를
바꾸지 않습니다. 실제 Enable/position 전송은 모든 안전 조건을 다시 확인한 50 Hz
control tick에서만 수행합니다. `ACTIVE`에서 명령 나이가 기본 200 ms를 넘으면 즉시
Disable하고 `WAIT_CMD`로 돌아갑니다. 반대로 `WAIT_CMD`에서 명령 없이 대기하는 것은
정상 상태이며 timeout fault가 아닙니다.

## ROS 인터페이스

주요 구독 토픽:

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/vehicle/actuator_command` | `std_msgs/msg/Float32` | 절대 바퀴각 명령, degree |
| `/drive/status` | `std_msgs/msg/UInt8` | `0=MANUAL`, `1=AUTO`, 나머지는 invalid |
| `/brake/status` | `std_msgs/msg/UInt8` | 0이 아니면 E-STOP 활성 |
| `/safety/aeb_state` | `std_msgs/msg/UInt8` | 0이 아니면 AEB 활성 |
| `/steering/test/estop` | `std_msgs/msg/UInt8` | test injection이 활성일 때 사용하는 시험 입력 |
| `/steering/test/aeb` | `std_msgs/msg/UInt8` | test injection이 활성일 때 사용하는 시험 입력 |

주요 발행 토픽:

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/steering/status` | `lateral_ctrl/msg/SteeringActuatorStatus` | 상태, disable reason, timeout/age, target/actual 및 motor telemetry |
| `/steering/angle` | `std_msgs/msg/Float32` | 실제 바퀴각 |
| `/ctrl/steer` | `std_msgs/msg/Float32` | 기존 소비자를 위한 실제 바퀴각 호환 토픽 |
| `/steering/enable_commanded` | `std_msgs/msg/Bool` | 노드가 명령한 Enable 상태 |
| `/steering/enabled` | `std_msgs/msg/Bool` | 유효한 feedback이 있을 때의 모터 Enable 상태 |

기본 timeout과 제어값:

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `control_period_ms` | 20 | 50 Hz control tick |
| `can_timeout_ms` | 500 | CAN 통신 timeout |
| `feedback_timeout_ms` | 500 | position feedback timeout |
| `mode_timeout_ms` | 1500 | `/drive/status` timeout |
| `steer_command_timeout_ms` | 200 | `ACTIVE` 상태의 `/vehicle/actuator_command` timeout |
| `wheel_limit_deg` | 30.0 | 절대 바퀴각 제한 |
| `max_steering_rate_deg_s` | 60.0 | production 바퀴각 변화율 제한 |
| `tracking_control_enabled` | false | tracking 보정 활성 여부 |

STM_A firmware는 `/drive/status`를 상태 변경 시 즉시, 평상시에는 약 1000 ms 주기로
발행합니다. 실제 두 보드 연결 시험에서 최대 간격 1002 ms가 측정되어 timeout을
1500 ms로 설정했습니다. 하드웨어나 전송 구성이 바뀌면 `ros2 topic hz /mode`,
`ros2 topic delay /mode`, rosbag 최대 간격을 다시 측정해 조정해야 합니다.

## 빌드와 테스트

저장소의 `archive` 아래에도 같은 이름의 패키지가 있으므로 활성 소스 경로를
명시해야 합니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash

colcon build \
  --base-paths software/src \
  --packages-select lateral_ctrl \
  --symlink-install

source install/setup.bash

colcon test \
  --base-paths software/src \
  --packages-select lateral_ctrl
colcon test-result --verbose
```

설치 결과 확인:

```bash
ros2 pkg executables lateral_ctrl
ls install/lateral_ctrl/share/lateral_ctrl/config
ls install/lateral_ctrl/share/lateral_ctrl/launch
```

정상 실행 파일은 `steering_actuator_node`, `steering_bench_test_node` 두 개입니다.
`config/steering_actuator.yaml`과 `config/steering_bench_test.yaml`은 Git에서 추적하는
공용 기본 설정입니다. 개인 장비용 임시 설정은 `*.local.yaml` 또는 `*.local.yml`로
만들어 공용 YAML을 덮어쓰지 마십시오.

## 실제 CAN 준비

기본 인터페이스는 `can0`, bitrate는 250000 bit/s입니다.

```bash
cd ~/Control

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

ip -details link show can0
candump -L can0
```

## A 실전 실행

실제 차량 입력만 사용하는 정상 실행입니다. STM_A가 `/drive/status`, STM_B가 `/brake/status`,
판단/경로 추종 노드가 `/safety/aeb_state`와 `/vehicle/actuator_command`를 발행해야 합니다. 이 launch는 벤치
노드를 실행하거나 시험 publisher를 만들지 않습니다. 실제 CAN과 actuator output은
기본 활성이므로 띄우는 즉시 모터로 프레임이 나갑니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lateral_ctrl steering_actuator.launch.py
```

출력을 차단한 상태로 launch 구성만 확인하려면 두 인자를 `false`로 넘깁니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lateral_ctrl steering_actuator.launch.py \
  hardware_can_enabled:=false \
  actuator_output_enabled:=false
```

`ros2 run lateral_ctrl steering_actuator_node`로 직접 실행하면 C++ 안전 기본값 때문에
hardware/output이 모두 false입니다.

상태 확인:

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /steering/status
ros2 topic echo /steering/enable_commanded
ros2 topic hz /mode
candump -L can0
```

AUTO mode와 유효 CAN/position feedback만 들어온 상태는 `WAIT_CMD`이며 계속
Disable이어야 합니다. 새 `/vehicle/actuator_command`를 받은 뒤에만 `ACTIVE`가 됩니다.

## B-1 순수 벤치 시험

STM_A/STM_B micro-ROS agent 없이 조향 모터와 상태 머신만 시험하는 구성입니다.
실제 STM agent가 실행 중이면 실제 안전 입력도 함께 적용되므로 순수 벤치 시험에서는
agent를 실행하지 마십시오.

기본 벤치 launch는 다음을 함께 실행합니다.

- 실제 `can0` actuator output 활성
- `/mode=AUTO`를 100 ms 주기로 발행
- 3초 뒤 절대 바퀴각 `[5, -5]` degree 왕복 시퀀스 시작
- 현재 목표각을 20ms마다 50Hz로 계속 발행
- 목표 index만 3초마다 변경하고 시퀀스 반복
- actuator 설정의 기본 60 deg/s rate limit 사용
- test E-STOP/AEB 토픽 발행 활성; 초기 값은 둘 다 false

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lateral_ctrl steering_bench_test.launch.py
```

이 명령은 종방향 제어 노드를 실행하거나 throttle/brake 명령을 발행하지 않습니다.
벤치 노드는 조향 시험에 필요한 다음 네 토픽만 발행합니다.

| 발행 토픽 | 타입 | 기본값/주기 | 용도 |
| --- | --- | --- | --- |
| `/drive/status` | `std_msgs/msg/UInt8` | `1`(AUTO), 10 Hz | actuator를 AUTO 시험 상태로 유지 |
| `/vehicle/actuator_command` | `std_msgs/msg/Float32` | `+5/-5` degree, 50 Hz | 현재 절대 목표각과 command heartbeat |
| `/steering/test/estop` | `std_msgs/msg/UInt8` | `0`, 20 Hz | 시험용 E-STOP 정상 상태 |
| `/steering/test/aeb` | `std_msgs/msg/UInt8` | `0`, 20 Hz | 시험용 AEB 정상 상태 |

actuator 노드는 시험 결과로 `/steering/status`, `/steering/angle`, `/ctrl/steer`,
`/steering/enable_commanded`, `/steering/enabled`를 발행합니다. 이들은 상태/feedback
토픽이며 다른 actuator를 구동하는 명령이 아닙니다.

AUTO와 시험용 안전 정상값만 자동 발행하고 조향 목표를 직접 넣으려면 두 터미널을
사용합니다.

터미널 1:

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lateral_ctrl steering_bench_test.launch.py \
  steer_sequence_publish_enabled:=false
```

터미널 2:

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub -r 50 /cmd/steer std_msgs/msg/Float32 '{data: 5.0}'
```

`/vehicle/actuator_command`를 한 번만 발행하면 기본 200 ms timeout으로 모터가 Disable됩니다.
수동 조향 시험에서도 위처럼 50 Hz로 계속 발행하고, 발행 명령을 종료하면 actuator가
timeout으로 Disable되는지 확인하십시오.

## B-2 혼합 벤치 시험

STM_A의 실제 `/drive/status`와 STM_B의 실제 `/brake/status`을 사용하고, 벤치 노드는 `/vehicle/actuator_command`
시퀀스만 발행하는 구성입니다.

| 입력 | 발행 주체 | 혼합 벤치에서의 역할 |
| --- | --- | --- |
| `/drive/status` | STM_A | `0=MANUAL`, `1=AUTO`; MANUAL이면 즉시 Disable |
| `/brake/status` | STM_B | 0이 아니면 즉시 `ESTOP`과 Disable |
| `/safety/aeb_state` | 판단 노드 | 현재 판단 노드가 없으면 publisher 0개일 수 있음 |
| `/vehicle/actuator_command` | 벤치 노드 | 절대 바퀴각 `+5/-5` degree를 50 Hz로 발행 |

실행 전에 STM_A와 STM_B에 해당 펌웨어를 플래시하고, 두 보드를 micro-ROS로
연결해 두어야 합니다. 펌웨어 빌드·플래시 및 micro-ROS agent 설정 방법은 각 STM
펌웨어 문서를 따릅니다. 다음 명령으로 `/stm_a_node`, `/stm_b_node`와 실제
`/drive/status`, `/brake/status`이 보이는 상태에서 혼합 벤치를 시작하십시오.

```bash
ros2 node list
ros2 topic echo --once /mode
ros2 topic echo --once /estop
```

혼합 벤치 실행:

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch lateral_ctrl steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false \
  safety_injection_enabled:=false
```

이 구성에서는 `/drive/status` publisher가 STM_A 하나, `/brake/status` publisher가 STM_B 하나여야
하며 시험용 safety publisher는 0개여야 합니다. 현재 STM 펌웨어에서 `/safety/aeb_state`는 두 보드가
구독하는 호스트 입력이므로, 별도 AEB 노드를 실행하지 않으면 publisher는 없습니다.
물리 mode가 AUTO일 때만 조향이 활성화됩니다.

두 launch 인자는 다음 역할만 합니다.

- `drive_mode_publish_enabled:=false`: 벤치 `/drive/status` publisher를 제거하고 STM_A의
  실제 `/drive/status`만 사용합니다.
- `safety_injection_enabled:=false`: 시험 E-STOP/AEB publisher를 제거합니다. 실제
  `/brake/status`과 `/safety/aeb_state` 구독 및 Disable 처리는 그대로 유지됩니다.

실행 후 별도 터미널에서 입력 소유권과 상태를 확인합니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 node list
ros2 topic info /mode
ros2 topic info /estop
ros2 topic info /aeb
ros2 topic info /cmd/steer
ros2 topic info /steering/test/estop
ros2 topic info /steering/test/aeb
ros2 topic echo /steering/status
```

혼합 벤치의 정상 publisher 수는 다음과 같습니다.

| 토픽 | 정상 publisher 수 |
| --- | ---: |
| `/drive/status` | 1개, STM_A |
| `/brake/status` | 1개, STM_B |
| `/safety/aeb_state` | 판단 노드 미실행 시 0개 |
| `/vehicle/actuator_command` | 1개, 벤치 노드 |
| `/steering/test/estop` | 0개 |
| `/steering/test/aeb` | 0개 |

AUTO와 E-STOP 해제 상태에서는 `ACTIVE`, MANUAL에서는 `DISABLED/MANUAL_MODE`,
E-STOP 활성 상태에서는 `ESTOP`이 되어야 합니다. MANUAL에서 AUTO로 복귀하거나
E-STOP을 해제하면 먼저 `WAIT_CMD`를 거치고, 복구 이후 수신한 새 `/vehicle/actuator_command`로만
다시 `ACTIVE`가 되어야 합니다.

## 벤치 안전 입력과 파라미터 시험

B-1 순수 벤치에서는 Test E-STOP/AEB를 기본적으로 50 ms마다 발행합니다. 다음
runtime parameter로 활성/해제를 시험할 수 있습니다. B-2 혼합 벤치는
`safety_injection_enabled=false`이므로 이 시험 파라미터 대신 STM_B의 물리 E-STOP과
판단 노드의 실제 AEB를 사용합니다. 안전 입력이 true인 동안에는 모든 조향 명령이
폐기되고 모터가 Disable됩니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

# E-STOP 활성/해제
ros2 param set /steering_bench_test_node test_estop_active true
ros2 param set /steering_bench_test_node test_estop_active false

# AEB 활성/해제
ros2 param set /steering_bench_test_node test_aeb_active true
ros2 param set /steering_bench_test_node test_aeb_active false
```

실제 `/brake/status`, `/safety/aeb_state` 토픽도 벤치 실행 중 항상 적용됩니다. 실제 입력을 직접 시험할
때는 별도 터미널에서 다음처럼 발행합니다. `1` 발행을 중단하는 것만으로는 내부 상태가
해제되지 않으므로 반드시 `0`을 명시적으로 발행해야 합니다.

```bash
cd ~/Control
source /opt/ros/humble/setup.bash
source install/setup.bash

# 실제 E-STOP 활성/해제
ros2 topic pub --once /estop std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /estop std_msgs/msg/UInt8 '{data: 0}'

# 실제 AEB 활성/해제
ros2 topic pub --once /aeb std_msgs/msg/UInt8 '{data: 1}'
ros2 topic pub --once /aeb std_msgs/msg/UInt8 '{data: 0}'
```

벤치 명령은 현재각 기준 offset이 아니라 절대 바퀴각입니다. 현재 바퀴가 중앙에서
벗어나 있으면 첫 5도 명령에서 큰 이동이 발생할 수 있습니다. 벤치 노드가 종료되면
MANUAL과 test safety false를 best-effort 발행하며, launch는 actuator 노드도 함께
종료시킵니다.

벤치 시퀀스의 핵심 파라미터는 다음과 같습니다.

```yaml
steer_command_publish_period_ms: 20
steer_target_change_interval_ms: 3000
steer_sequence_repeat: true
```

`steer_command_publish_period_ms`는 actuator의 기본 200ms command timeout보다 충분히
짧아야 합니다. 목표 변경 간격과 command 발행 주기는 서로 독립적입니다. actuator가
`ACTIVE`인 동안 CAN position frame도 50 Hz control tick마다 반복 전송됩니다.

## vcan 확인

```bash
cd ~/Control

sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lateral_ctrl steering_actuator.launch.py can_interface:=vcan0
```

별도 KEYA feedback injector가 있어야 `CAN_LOST`에서 정상 상태로 전환할 수 있습니다.
`candump -L vcan0`과 `/steering/status`로 다음 순서까지 확인해야 합니다.

1. 시작할 때 Disable frame이 전송됩니다.
2. 유효 feedback과 AUTO만 있을 때 `WAIT_CMD`이며 Enable/position frame은 없습니다.
3. 복구 경계 이후 새 `/vehicle/actuator_command`가 들어오면 Enable 다음 현재 실제각 position을 먼저 보냅니다.
4. 다음 control tick부터 rate-limited position frame을 50 Hz로 전송합니다.
5. `/vehicle/actuator_command`가 200 ms 이상 끊기면 Disable하고 `WAIT_CMD`가 됩니다.
6. 새 명령이 들어오면 다시 활성화할 수 있습니다.
7. E-STOP/AEB 해제나 CAN/feedback 복구 후에도 과거 목표를 복원하지 않고 새 명령을 기다립니다.
8. `/drive/status`가 끊기거나 MANUAL이 되면 `DISABLED`와 Disable을 확인합니다.
9. 종료할 때 Disable frame이 전송됩니다.
10. 주기적인 Enable ping이나 중복 CAN 소유 노드가 없어야 합니다.
