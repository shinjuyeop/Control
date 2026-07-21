# lateral_ctrl

TeamKAI-DL 차량의 KEYA 조향 액추에이터를 제어하는 ROS 2 Humble 패키지입니다.
현재 구조에서는 `steering_actuator_node` 하나만 SocketCAN 인터페이스를 열고 모터를
제어합니다. 과거의 `steering_control_node`와 `steering_feedback_node`는 제거되었으며,
같은 CAN 모터를 제어하는 다른 노드를 동시에 실행하면 안 됩니다.

> **실차 주의:** `steering_actuator.launch.py`와 `steering_bench_test.launch.py`는
> `hardware_can_enabled=true`, `actuator_output_enabled=true`가 기본값입니다.
> launch 전에 바퀴를 공중에 띄우고, 조향 기구 주변을 비우고, 물리 E-STOP을
> 준비하고, CAN 인터페이스와 기계적 중앙을 확인하십시오.

## 현재 구성

실행 파일은 두 개입니다.

| 실행 파일 | 역할 |
| --- | --- |
| `steering_actuator_node` | `/mode`, `/cmd/steer`, 안전 입력과 KEYA 피드백을 받아 모터를 제어하고 상태를 발행 |
| `steering_bench_test_node` | 벤치 시험용 AUTO mode와 절대 바퀴각 시퀀스를 발행 |

주요 내부 모듈은 다음과 같습니다.

- `KeyaCanDriver`: KEYA 확장 CAN frame 송수신 및 feedback parsing
- `SteeringAngleConverter`: 모터 encoder count, 모터각, 바퀴각 변환과 각도 제한
- `WheelAngleTrackingController`: 선택 가능한 바퀴각 tracking 보정; 기본 비활성
- `SteeringActuatorNode`: 50 Hz 중앙 상태 머신과 안전 전환

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

E-STOP, AEB, MANUAL/invalid mode, mode timeout, steer command timeout, CAN/position
feedback 손실, motor fault가 발생하면 모터를 즉시 Disable하고 현재 target, pending
target, command generation/timestamp, tracking integral/correction을 모두 폐기합니다.
안전 조건이 복구되어도 과거 목표는 복원하지 않으며 `WAIT_CMD` 진입 이후 수신한
새 `/cmd/steer`가 있어야 다시 활성화됩니다.

`WAIT_CMD`에서 새 명령을 받으면 control tick에서 안전 조건을 다시 확인하고 다음
순서로 활성화합니다.

1. commanded/corrected angle을 현재 실제 바퀴각으로 초기화합니다.
2. 새 target을 저장하고 tracking controller를 reset합니다.
3. Enable frame을 전송합니다.
4. 현재 실제각에 해당하는 position frame을 한 번 전송합니다.
5. `ACTIVE`로 전환합니다.
6. 다음 control tick부터 목표각 방향으로 rate limiting을 적용합니다.

## ROS 인터페이스

주요 구독 토픽:

| 토픽 | 타입 | 설명 |
| --- | --- | --- |
| `/cmd/steer` | `std_msgs/msg/Float32` | 절대 바퀴각 명령, degree |
| `/mode` | `std_msgs/msg/UInt8` | `0=MANUAL`, `1=AUTO`, 나머지는 invalid |
| `/estop` | `std_msgs/msg/UInt8` | 0이 아니면 E-STOP 활성 |
| `/aeb` | `std_msgs/msg/UInt8` | 0이 아니면 AEB 활성 |
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
| `mode_timeout_ms` | 1000 | `/mode` timeout |
| `steer_command_timeout_ms` | 200 | `ACTIVE` 상태의 `/cmd/steer` timeout |
| `wheel_limit_deg` | 30.0 | 절대 바퀴각 제한 |
| `max_steering_rate_deg_s` | 60.0 | production 바퀴각 변화율 제한 |
| `tracking_control_enabled` | false | tracking 보정 활성 여부 |

STM_A firmware는 `/mode`를 상태 변경 시 즉시, 평상시에는 1000 ms 주기로
발행합니다. 현재 1000 ms timeout은 jitter 여유가 없으므로 실차 적용 전
`ros2 topic hz /mode`, `ros2 topic delay /mode`, rosbag 최대 간격을 측정해 정상
주기의 5~10배 이상으로 조정해야 합니다.

## 빌드와 테스트

저장소의 `archive` 아래에도 같은 이름의 패키지가 있으므로 활성 소스 경로를
명시해야 합니다.

```bash
cd /path/to/Control
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

## 실제 CAN 준비

기본 인터페이스는 `can0`, bitrate는 250000 bit/s입니다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up

ip -details link show can0
candump -L can0
```

## Production 실행

상위 판단/경로 추종 노드가 `/mode`, `/cmd/steer`를 주기적으로 발행하는 정상 차량
실행입니다. 이 launch는 실제 CAN과 actuator output을 기본 활성화합니다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lateral_ctrl steering_actuator.launch.py
```

출력을 차단한 상태로 launch 구성을 확인하려면:

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py \
  hardware_can_enabled:=false \
  actuator_output_enabled:=false
```

`ros2 run lateral_ctrl steering_actuator_node`로 직접 실행하면 C++ 안전 기본값 때문에
hardware/output이 모두 false입니다.

상태 확인:

```bash
ros2 topic echo /steering/status
ros2 topic echo /steering/enable_commanded
ros2 topic hz /mode
candump -L can0
```

AUTO mode와 유효 CAN/position feedback만 들어온 상태는 `WAIT_CMD`이며 계속
Disable이어야 합니다. 새 `/cmd/steer`를 받은 뒤에만 `ACTIVE`가 됩니다.

## 벤치 시험

기본 벤치 launch는 다음을 함께 실행합니다.

- 실제 `can0` actuator output 활성
- `/mode=AUTO`를 100 ms 주기로 발행
- 3초 뒤 절대 바퀴각 `[0, 5, 0, -5, 0]` degree 발행
- 명령 간격 3초, 반복 없음
- actuator rate limit 5 deg/s
- test E-STOP/AEB injection 비활성

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

AUTO만 발행하고 조향 명령을 직접 넣으려면:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  steer_sequence_publish_enabled:=false

ros2 topic pub --once /cmd/steer std_msgs/msg/Float32 '{data: 1.0}'
```

외부 STM32의 `/mode`를 사용하려면:

```bash
ros2 launch lateral_ctrl steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false
```

벤치 명령은 현재각 기준 offset이 아니라 절대 바퀴각입니다. 현재 바퀴가 중앙에서
벗어나 있으면 첫 0도 명령에서 큰 이동이 발생할 수 있습니다. 벤치 노드가 종료되면
MANUAL과 test safety false를 best-effort 발행하며, launch는 actuator 노드도 함께
종료시킵니다.

## vcan 확인

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0

ros2 launch lateral_ctrl steering_actuator.launch.py can_interface:=vcan0
```

별도 KEYA feedback injector가 있어야 `CAN_LOST`에서 정상 상태로 전환할 수 있습니다.
`candump -L vcan0`에서 `WAIT_CMD` 중 Enable/position frame이 없고, 새 명령 활성화 시
Enable 다음 현재각 position frame이 한 번 전송되며, 명령 timeout과 안전 입력에서
Disable frame이 전송되는지 확인하십시오.

상세 상태 전환, CAN 시험 체크리스트와 설계 근거는
[`STEERING_CONTROL.md`](STEERING_CONTROL.md)를 참고하십시오.
