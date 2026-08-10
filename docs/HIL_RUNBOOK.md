# KAIEV26 HIL 운용 절차

이 문서는 새 SSH 세션에서 KAIEV26 HIL bench를 준비하고, STM_A/B Agent와
STM_TEST bridge, HIL GUI, 선택적인 KEYA 조향 및 차량 상태 GUI까지 실행·검증·종료하는
절차를 설명한다.

현재 bench에는 STM_A와 연결된 EZKontrol CAN transceiver만 연결되어 있다. 따라서
기본 절차에서는 traction·brake HIL을 실행하고 KEYA는 GUI에서
`NO RX / NOT CONNECTED`로 관찰한다. KEYA CAN2 transceiver가 설치된 뒤에는
[KEYA 포함 전체 HIL](#10-keya-포함-전체-hil선택)을 추가로 실행한다.

이 문서의 명령은 다음 배치를 기준으로 한다.

```text
~/KAI_ws/
├── src/Control/
└── src/kaiev26_msgs/
```

## 1. 실행 구성

```text
HIL command generator ── /vehicle/command ──┬─> STM_A ── EZ CAN1 ──> STM_TEST
                                             │                         │
                                             │    motor feedback CAN1 <┘
                                             │
                                             ├─> STM_B ── brake PWM ──> STM_TEST
                                             │
                                             └─> lateral_ctrl ─ KEYA CAN2 ─> STM_TEST
                                                                         │
                                                   steering feedback <────┘

STM_TEST GPIO/DAC ──> STM_A/B mode, gear, pedal, E-stop inputs
STM_A/B serial ──> micro-ROS Agent ──> /drive/status, /brake/status
STM_TEST serial ──> stm_test_bridge ──> /hil/status
```

역할별 실행 단위는 다음과 같다.

| 실행 단위 | 포함 항목 | 필수 여부 |
|---|---|---|
| STM_A Agent | `/dev/hil_a` ↔ `/stm_a_node` | 필수 |
| STM_B Agent | `/dev/hil_b` ↔ `/stm_b_node` | 필수 |
| `hil.launch.py` | STM_TEST bridge, command generator, HIL GUI | 필수 |
| `hil_bench.launch.py` | 위 세 줄을 한 번에 실행 | 위 세 줄의 대안 |
| `lateral_ctrl` | SocketCAN `can0` KEYA 제어 | CAN2 transceiver 연결 후 |
| `vehicle_state_publisher` | status 3종 → `/vehicle/state` | 조향 status가 있을 때 |
| `vehicle_dashboard` | 전체 차량 상태 모니터링 GUI | 선택 |

`hil.launch.py`는 micro-ROS Agent, `lateral_ctrl`,
`vehicle_state_publisher`를 실행하지 않는다. `hil_bench.launch.py`는 Agent 두
개까지 포함하지만 `lateral_ctrl`과 `vehicle_state_publisher`는 마찬가지로
포함하지 않는다. 전체 차량 stack의 통합 launch는 아직 없다.

## 2. 호스트 최초 1회 준비

### 2.1 런타임 패키지

Ubuntu 22.04/ROS 2 Humble 기준이다.

```bash
sudo apt update
sudo apt install \
  python3-pyqt5 \
  python3-pyqtgraph \
  python3-serial \
  python3-yaml \
  x11-utils \
  can-utils
```

`python3-pyqtgraph`가 설치되지 않으면 `vehicle_dashboard`가 실행되지
않는다. HIL GUI 자체에는 `pyqtgraph`가 필요하지 않다.

```bash
python3 -c "import serial, PyQt5, pyqtgraph; print('GUI dependencies OK')"
```

### 2.2 serial 권한

```bash
sudo usermod -aG dialout "$USER"
```

그룹 변경은 기존 셸에 소급 적용되지 않는다. SSH를 완전히 종료하고 다시 접속한 뒤
확인한다.

```bash
id -nG | tr ' ' '\n' | grep -x dialout
```

장치 권한을 임시로 `chmod 666` 하는 방법은 사용하지 않는다.

### 2.3 고정 장치 이름 설치

```bash
cd ~/KAI_ws/src/Control

sudo install -m 0644 firmware/udev/99-kaiev26-stm.rules \
  /etc/udev/rules.d/99-kaiev26-stm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

다음 세 이름을 사용하며 `ttyACM0/1/2` 번호는 직접 사용하지 않는다.

```bash
ls -l /dev/hil_a /dev/hil_b /dev/hil_test
readlink -f /dev/hil_a
readlink -f /dev/hil_b
readlink -f /dev/hil_test
```

| 이름 | 역할 | 현재 기록된 ST-LINK VCP serial |
|---|---|---|
| `/dev/hil_a` | STM_A firmware | `0025002B3234511137333934` |
| `/dev/hil_b` | STM_B firmware | `003D00433234511137333934` |
| `/dev/hil_test` | STM_TEST firmware | `002E00253235510837333439` |

이 매핑은 ST-LINK virtual COM port 매핑이다. SWD probe가 실제로 어느 MCU에
연결되는지는 별개이므로 이 표만 보고 플래시하지 않는다.

## 3. SSH와 X11 연결

로컬 PC 터미널에서 실행한다. `CONTROL_PC_IP`는 실제 IP나 SSH alias로 바꾸며
`<host>`처럼 꺾쇠를 입력하지 않는다.

```bash
ssh -Y kai@CONTROL_PC_IP
```

원격 프롬프트가 `kai@...`로 바뀐 뒤 확인한다.

```bash
echo "$DISPLAY"
xdpyinfo >/dev/null && echo "X11 OK"
```

정상적인 SSH X11 forwarding에서는 `DISPLAY`가 `localhost:10.0`과 비슷한 값이다.
`:0`은 보통 로컬 PC의 화면이며, SSH 접속이 실패한 상태에서 확인한 값일 수 있다.

GUI가 필요 없는 Agent 터미널은 일반 SSH 접속이어도 된다. HIL GUI나
`vehicle_dashboard`를 실행하는 터미널만 `ssh -Y`가 필요하다.

## 4. 공통 ROS 환경

이 절은 새로 연 모든 원격 터미널에서 실행한다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
```

현재 STM_A/B firmware의 micro-ROS 초기화 경로는 Domain 0 participant를 만든다.
Agent 셸에서 `ROS_DOMAIN_ID`만 190으로 바꾸면 STM이 이동하지 않으므로,
현재는 모든 셸에서 `ROS_DOMAIN_ID=0`을 사용한다.

환경 확인:

```bash
ros2 pkg prefix kaiev26_hil
ros2 pkg prefix micro_ros_agent
ros2 pkg prefix lateral_ctrl
ros2 pkg prefix visualization
```

## 5. 소프트웨어 빌드와 테스트

소스가 바뀌었거나 처음 실행하는 경우 워크스페이스 루트에서 빌드한다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh

colcon build --symlink-install \
  --packages-select \
  kaiev26_msgs \
  micro_ros_msgs \
  micro_ros_agent \
  kaiev26_hil \
  lateral_ctrl \
  vehicle_state_publisher \
  visualization

source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0
```

HIL·시각화 테스트:

```bash
colcon test --packages-select kaiev26_hil visualization
colcon test-result --verbose
```

### STM_TEST 빌드만 확인하기

이 단계는 소스와 ELF를 검증할 뿐 플래시하지 않는다.

```bash
cd ~/KAI_ws/src/Control
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh

cmake -S firmware/STM_TEST --preset Release
cmake --build firmware/STM_TEST/build/Release --clean-first -j4
firmware/STM_TEST/tests/run_host_tests.sh
```

결과 ELF:

```text
~/KAI_ws/src/Control/firmware/STM_TEST/build/Release/STM_TEST.elf
```

현장 보드와 SWD target을 확인하기 전에는 이 ELF를 플래시하지 않는다.

## 6. 실행 전 점검

### 6.1 장치와 권한

```bash
for HIL_DEVICE in /dev/hil_a /dev/hil_b /dev/hil_test; do
  test -r "$HIL_DEVICE" && test -w "$HIL_DEVICE" \
    && echo "OK $HIL_DEVICE -> $(readlink -f "$HIL_DEVICE")" \
    || echo "NO ACCESS $HIL_DEVICE"
done
```

### 6.2 중복 프로세스

같은 serial 장치를 두 프로세스가 열거나 `/vehicle/command` publisher가 둘 이상
생기지 않게 한다.

```bash
pgrep -af 'micro_ros_agent|stm_test_bridge|hil_gui|steering_actuator_node' || true
ros2 node list
ros2 topic info -v /vehicle/command
```

기존 HIL 프로세스가 있다면 해당 프로세스를 실행했던 터미널에서 `Ctrl-C`로
정상 종료한다. 범위를 알 수 없는 `pkill`은 사용하지 않는다.

## 7. 현재 bench 실행: EZ CAN + brake PWM

### 7.1 한 번에 실행하기

X11 forwarding이 있는 `ssh -Y` 터미널 하나에서 Agent 두 개와 bridge·command
generator·GUI를 함께 띄운다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 launch kaiev26_hil hil_bench.launch.py
```

`stm_a_port`, `stm_b_port`, `serial_port`로 장치를 바꿀 수 있고, Agent가 이미
따로 실행 중이면 `agents:=false`로 제외한다.

```bash
ros2 launch kaiev26_hil hil_bench.launch.py \
  serial_port:=/dev/hil_test \
  stm_a_port:=/dev/hil_a \
  stm_b_port:=/dev/hil_b \
  agents:=false
```

이 launch는 HIL bench 범위만 실행한다. `lateral_ctrl`,
`vehicle_state_publisher`, `vehicle_dashboard`는 포함하지 않으므로 필요할 때
[10절](#10-keya-포함-전체-hil선택)과 [11절](#11-통합-차량-상태와-monitoring-gui선택)에서
따로 실행한다.

모든 로그가 한 터미널에 섞이므로, XRCE session 생성 실패처럼 Agent 로그만
읽어야 하는 상황에서는 아래 7.2의 터미널 분리 방식을 사용한다.

### 7.2 터미널을 나눠 실행하기

세 개의 원격 터미널을 사용한다. 터미널 3은 반드시 X11 forwarding이 있는
`ssh -Y` 세션이어야 한다.

#### 터미널 1: STM_A Agent

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/hil_a -b 115200
```

#### 터미널 2: STM_B Agent

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/hil_b -b 115200
```

#### 터미널 3: STM_TEST bridge + command generator + HIL GUI

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test
```

이 launch는 연결 시 bridge의 안전한 기본 desired state를 STM_TEST에 재적용한다.
command generator는 생성되지만 기본값 `enabled=false`라서
`/vehicle/command`를 발행하지 않는다.

레이아웃만 보고 싶을 때는 다음처럼 GUI만 실행할 수 있다. 이 경우 bridge가 없으므로
`DISCONNECTED`가 정상이다.

```bash
ros2 run kaiev26_hil hil_gui
```

## 8. 연결 검증

네 번째 터미널에서 공통 ROS 환경을 source한 뒤 실행한다.

```bash
ros2 node list | sort
```

현재 기본 구성의 기대 node:

```text
/hil_command_generator
/hil_gui
/stm_a_node
/stm_b_node
/stm_test_bridge
```

연결 및 STATUS 확인:

```bash
ros2 topic echo --once /hil/connected
ros2 topic echo --once /hil/status
ros2 topic echo --once /drive/status
ros2 topic echo --once /brake/status
```

주기 확인은 각각 실행하고 `Ctrl-C`로 끝낸다.

```bash
ros2 topic hz /drive/status
ros2 topic hz /brake/status
ros2 topic hz /hil/status
```

endpoint와 중복 publisher 확인:

```bash
ros2 topic info -v /vehicle/command
ros2 topic info -v /drive/status
ros2 topic info -v /brake/status
```

정상 결과:

- `/hil/connected.data: true`
- `/stm_a_node`가 `/drive/status` 발행
- `/stm_b_node`가 `/brake/status` 발행
- STM_TEST 새 firmware는 `/hil/status` JSON의 `protocol`이 `2`
- EZ CAN frame 수신 시 GUI 중앙 EZ panel이 `ACTIVE`
- 현재 KEYA 미연결 bench에서는 KEYA panel이 `NO RX / NOT CONNECTED`

Protocol 1 구형 STM_TEST도 bridge에는 연결되지만 Plant와 command observation
field는 사용할 수 없다.

## 9. 기본 기능 시험

### 9.1 먼저 안전 상태 적용

```bash
ros2 param set /hil_command_generator enabled false
ros2 service call /hil/reset_safe_state std_srvs/srv/Trigger '{}'
ros2 topic echo --once /hil/status
```

Safe State는 MANUAL, Forward, pedal/motion/steering 0, E-stop OFF, CAN ON,
Plant OFF, fault/TX drop OFF다. command generator의 enable은 별도 상태이므로 항상
먼저 끈다.

### 9.2 AUTO 속도 폐루프 시험

GUI에서 다음 순서로 설정하는 것이 가장 직관적이다.

1. 왼쪽에서 `Mode=AUTO`, `Gear=FORWARD` 적용
2. 오른쪽 Response mode를 `Closed-loop simulation`으로 변경
3. `End-to-End Command Source`에서 낮은 speed target을 입력
4. `Publish commands`를 켜고 적용
5. 중앙의 EZ command RPM/current/target과 오른쪽 actual RPM/speed를 비교

동일한 CLI 절차:

```bash
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET MODE AUTO"}'
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET GEAR FORWARD"}'
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET PLANT ON"}'

ros2 param set /hil_command_generator speed_target_mps 2.0
ros2 param set /hil_command_generator steering_target_rad 0.0
ros2 param set /hil_command_generator brake_engage false
ros2 param set /hil_command_generator enabled true
```

관찰:

```bash
ros2 topic echo /hil/status
ros2 topic echo /drive/status
```

종료 시 command 발행을 먼저 끈다.

```bash
ros2 param set /hil_command_generator enabled false
```

### 9.3 MANUAL pedal 폐루프 시험

이 시험은 STM_TEST pedal DAC → STM_A → EZ CAN command → STM_TEST Plant →
motor feedback의 왕복 경로를 확인한다.

```bash
ros2 param set /hil_command_generator enabled false
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET MODE MANUAL"}'
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET PLANT ON"}'
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET PEDAL 10"}'
```

낮은 값에서 시작하고 GUI에서 EZ command current, target RPM, actual RPM을 확인한다.
끝나면 pedal 0과 Safe State를 적용한다.

```bash
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET PEDAL 0"}'
ros2 service call /hil/reset_safe_state std_srvs/srv/Trigger '{}'
```

### 9.4 Direct feedback override

Plant를 끄면 STM_A 명령 대신 STM_TEST가 지정한 speed/RPM을 feedback으로 보낸다.
bridge는 Plant를 끄기 직전 actual RPM을 저장하므로 모드 전환과 재연결 때 값이
갑자기 과거 주입값으로 돌아가지 않는다.

```bash
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET PLANT OFF"}'
ros2 topic pub --once /hil/command std_msgs/msg/String \
  '{data: "SET SPEED 5"}'
```

### 9.5 brake PWM 시험

AUTO command source를 사용할 때 낮은 speed target과 함께 brake를 설정한다.

```bash
ros2 param set /hil_command_generator speed_target_mps 0.0
ros2 param set /hil_command_generator steering_target_rad 0.0
ros2 param set /hil_command_generator brake_engage true
ros2 param set /hil_command_generator enabled true
```

```bash
ros2 topic echo /brake/status
ros2 topic echo /hil/status
```

GUI 중앙 brake panel에서 `brake_period_us`, `brake_us`, decoded state,
`pwm_lost`를 확인한다. 시험 후에는 다음 순서로 해제한다.

```bash
ros2 param set /hil_command_generator brake_engage false
ros2 param set /hil_command_generator enabled false
ros2 service call /hil/reset_safe_state std_srvs/srv/Trigger '{}'
```

## 10. KEYA 포함 전체 HIL(선택)

다음 단계는 STM_TEST FDCAN2 transceiver와 Host USB-CAN이 실제로 연결된 뒤에만
수행한다. 현재 bench에서는 건너뛴다.

### 10.1 SocketCAN 준비

```bash
ip link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
ip -details link show can0
```

### 10.2 lateral controller 실행

출력이 활성화된 `lateral_ctrl`는 `can0`에 KEYA Disable·position frame을 보낸다.
동일 bus를 제어하는 다른 노드가 없는지 먼저 확인한다.

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py \
  can_interface:=can0 \
  hardware_can_enabled:=true \
  actuator_output_enabled:=true
```

검증:

```bash
ros2 topic echo /steering/status
ros2 topic echo /steering/enable_commanded
candump -L can0
```

GUI의 KEYA panel은 유효 command/query를 받으면 `ACTIVE`로 바뀌고, enable 상태,
target shaft angle, simulated shaft angle을 표시한다. KEYA emulator의 조향 추종은
EZ Plant 설정과 독립적이다.

실제 CAN 없이 ROS 인터페이스만 확인하려면 출력 차단 상태로 실행한다.

```bash
ros2 launch lateral_ctrl steering_actuator.launch.py \
  hardware_can_enabled:=false \
  actuator_output_enabled:=false
```

## 11. 통합 차량 상태와 monitoring GUI(선택)

`vehicle_state_publisher`는 drive, brake, steering status를 모두 최소 한 번 받은 뒤에만
`/vehicle/state`를 발행한다. KEYA/lateral controller를 실행하지 않은 현재 구성에서는
`/vehicle/state`가 나오지 않는 것이 정상이다.

```bash
ros2 run vehicle_state_publisher vehicle_state_publisher_node
```

X11 terminal에서 dashboard 실행:

```bash
ros2 launch visualization vehicle_dashboard.launch.py
```

```bash
ros2 topic echo /vehicle/state
```

## 12. 자동 scenario 실행(선택)

`hil.launch.py`가 이미 bridge와 generator를 실행 중일 때 같은 launch를 다시
실행하면 중복 node와 serial 소유권 충돌이 생긴다. 기존 stack을 유지한 채 scenario
runner만 실행한다.

```bash
HIL_SHARE="$(ros2 pkg prefix kaiev26_hil)/share/kaiev26_hil"

ros2 run kaiev26_hil scenario_runner --ros-args \
  -p scenario_file:="$HIL_SHARE/config/scenarios/smoke_test.yaml" \
  -p results_dir:="$HOME/KAI_ws/hil_test_results"
```

GUI 없이 scenario 중심으로 새로 시작할 때는 같은 launch에 `gui:=false`와
`scenario_file`을 준다.

```bash
ros2 launch kaiev26_hil hil.launch.py \
  serial_port:=/dev/hil_test \
  gui:=false \
  scenario_file:="$(ros2 pkg prefix kaiev26_hil)/share/kaiev26_hil/config/scenarios/smoke_test.yaml" \
  results_dir:="$HOME/KAI_ws/hil_test_results"
```

## 13. 정상 종료 순서

1. command generator를 끈다.
2. STM_TEST Safe State를 적용한다.
3. STATUS를 확인한다.
4. GUI/HIL launch, lateral controller, STM_B Agent, STM_A Agent 순서로 `Ctrl-C`한다.

```bash
ros2 param set /hil_command_generator enabled false
ros2 service call /hil/reset_safe_state std_srvs/srv/Trigger '{}'
ros2 topic echo --once /hil/status
```

KEYA 시험에서 `can0`을 올렸고 더 사용하지 않는다면 모든 CAN 프로세스 종료 후 내린다.

```bash
sudo ip link set can0 down
```

강제 종료가 필요하면 먼저 PID와 명령을 정확히 확인하고 해당 PID만 종료한다.

```bash
pgrep -af 'micro_ros_agent|stm_test_bridge|hil_gui|steering_actuator_node'
```

## 14. 문제 해결

### GUI가 열리지 않음

```bash
echo "$DISPLAY"
xdpyinfo >/dev/null && echo "X11 OK"
```

`DISPLAY`가 비었으면 X11 forwarding이 없는 세션이다. 로컬에서
`ssh -Y kai@CONTROL_PC_IP`로 새로 접속한다. GUI entry point는 pip PyQt5의 bundled
Qt가 시스템 Qt와 섞이지 않도록 자동으로 한 번 재실행하므로 수동
`LD_LIBRARY_PATH` 우회는 필요 없다.

### `/hil/connected=false`

```bash
ls -l /dev/hil_test
id -nG
lsof /dev/hil_test
```

- `dialout`이 없으면 그룹 추가 후 SSH 재접속
- 다른 bridge/terminal이 serial을 열었으면 그 프로세스를 정상 종료
- 새 Python + 구형 firmware 조합은 지원되지만 `protocol=1`로 기능이 제한됨

### `/stm_a_node` 또는 `/stm_b_node`가 없음

```bash
ls -l /dev/hil_a /dev/hil_b
ros2 node list
```

- 해당 Agent 터미널에서 XRCE session 생성 로그 확인
- 모든 셸의 `ROS_DOMAIN_ID=0` 확인
- `/dev/hil_a`와 `/dev/hil_b`에 Agent를 각각 하나만 실행

### topic traffic은 있는데 typed message가 깨짐

Host `kaiev26_msgs`와 STM_A/B에 생성된 micro-ROS type support가 다를 수 있다.

```bash
cat ~/KAI_ws/src/Control/firmware/tools/microros_manifest.txt
git -C ~/KAI_ws/src/kaiev26_msgs rev-parse HEAD
```

두 provenance가 다르면 두 STM의 generated artifacts를 함께 재생성·빌드·플래시해야
한다. 재생성만으로 현장 플래시 권한이 생기는 것은 아니다.

### KEYA가 `NO RX / NOT CONNECTED`

현재 bench에서는 정상이다. KEYA CAN2 transceiver, termination, common ground,
SocketCAN `can0`, `lateral_ctrl`가 모두 준비되어야 `ACTIVE`가 된다. `keya_ok`는
STM_TEST의 FDCAN 주변장치 초기화 결과일 뿐 상대 controller 연결 판정이 아니다.

### `vehicle_dashboard`에서 `No module named pyqtgraph`

```bash
sudo apt install python3-pyqtgraph
python3 -c "import pyqtgraph; print(pyqtgraph.__version__)"
```

## 15. 현장 복귀 후 남은 작업

- USB를 하나씩 연결해 VCP serial과 물리 보드 라벨 재확인
- CubeProgrammer에서 각 ST-LINK SWD probe가 실제 어느 MCU에 연결되는지 확인
- STM_TEST target이 확정된 뒤에만 새 `STM_TEST.elf` 플래시
- `/hil/status.protocol=2`와 EZ command observation field 확인
- AUTO speed 및 MANUAL pedal 폐루프 시험
- KEYA transceiver 설치 후 CAN2 RX/feedback 시험

SWD target 불일치가 해소되기 전에는 serial symlink만 근거로 플래시하지 않는다.
