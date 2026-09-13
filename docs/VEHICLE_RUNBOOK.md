# 실차 운용 절차

차량 탑재 PC에서 보면서 쓰는 문서다. PC 셋업(빌드, serial 권한, 보드 고정
이름, can0 자동 준비)은 이 PC에 이미 완료되어 있어 여기 담지 않는다 — PC를
새로 꾸릴 때는 [`HIL_RUNBOOK.md`](HIL_RUNBOOK.md) 2절과 10.1을 따른다.
배경 설명은 [`ARCHITECTURE.md`](ARCHITECTURE.md)에 있다.

명령 흐름은 한 줄이다:

```text
Planning ─ /planning/command → command_governor ─ /vehicle/command → STM_A(구동)·STM_B(브레이크)·조향
```

---

## 1. 구동 순서

### ① 전원 전 확인

- E-stop 3개(dashboard/side/RF) 배선과 동작
- EZkontrol(구동)·KEYA(조향) 전원과 CAN 배선
- **launch가 뜨면 조향이 바로 armed다.** 확인이 끝나기 전에는 launch를 켜지
  않는다

### ② 구동 스택

새 zsh 터미널마다 아래 형식으로 ROS 환경을 잡는다. 이 PC의 `humble` 함수가
Humble 환경과 `ROS_DOMAIN_ID=0`을 설정하므로 별도 export는 필요 없다.
이미 활성화된 셸에서 domain을 변경한 경우에는 `humble`이 이를 되돌리지 않는다.
그때만 `export ROS_DOMAIN_ID=0`으로 복구한다. 다른 PC의 셋업은
[HIL runbook 공통 ROS 환경](HIL_RUNBOOK.md#4-공통-ros-환경)을 따른다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch control_launch vehicle.launch.py
```

can0는 부팅 때 자동으로 준비되어 있다. 이 시점에는 차가 **안전 대기**
상태다 — 명령이 없으면 아무것도 움직이지 않는다.

### ③ 시각화 (선택이지만 권장)

각각 별도 터미널에서 아래 블록을 그대로 붙여 넣으면 된다.

KAI Vehicle Console (브라우저에서 차량 상태만 관찰, 명령 발행 없음):

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch vehicle_console vehicle.launch.py \
  allowed_hosts:=100.112.72.27 \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/vehicle_console/.venv/bin/python
```

연결할 주소:

- **차량 PC 로컬 브라우저:** `http://127.0.0.1:8766/`
- **Tailscale 원격 브라우저:** `http://100.112.72.27:8766/`

Console은 기본 loopback에 바인딩하므로, 차량 PC에서 **Vehicle용 8766 포트 전달**을
별도로 등록한다. HIL용 8767 전달만으로는 Vehicle Console에 접속할 수 없다.

```zsh
# 차량 PC에서 최초 1회 등록. Web launch는 별도로 실행되어 있어야 한다.
tailscale serve --bg --tcp=8766 tcp://127.0.0.1:8766
tailscale serve status
```

원격 PC도 Tailscale에 연결되어 있어야 한다. 공유받은 계정에서 차량 PC가 다른 IP로
보이면 원격 PC의 `tailscale status`로 확인하고, 접속 URL과 `allowed_hosts`를 그 IP로
맞춘다. 상세 설정과 공유 해제는
[Console 원격 사용](../software/src/vehicle_console/README.md#원격-접속)을 따른다.
Qt dashboard는 archive로 이동했다.

Foxglove bridge — 팀 단위로 PC당 하나만 띄운다. 이미 떠 있으면 켜지 않고
아래 접속만 한다 (`/diagnostics` 요약은 구동 스택이 이미 발행 중이라
bridge만 켜면 된다):

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch foxglove foxglove_bridge.launch.py
```

- **Foxglove 접속**: 노트북의 Foxglove 앱에서 `ws://100.112.72.27:8765`
  (Tailscale 기준. 이 PC가 스스로 보는 IP와 노트북에서 보이는 IP가 다르니,
  안 붙으면 노트북에서 `tailscale status`로 이 PC의 IP를 다시 확인한다).
  레이아웃은 `software/src/foxglove/control.json`을 import한다.
  Diagnostics 패널에서 문제 항목이 위로 정렬되어 뜬다
- **bridge는 PC당 하나만 띄운다.** bridge 하나가 모든 파트의 토픽을
  서빙하고 노트북 여러 대가 동시에 접속할 수 있다. 각 파트의 launch에
  bridge를 넣으면 같은 포트를 두고 bind 실패가 나므로, 다른 파트도 자기
  bridge를 만들지 말고 위 주소에 접속한다

### ④ 자율주행 명령 준비

#### 실제 Planning (판단팀)

판단 노드가 `/planning/command`(`kaiev26_msgs/ActuatorCommand`)를 **20 Hz
이상, `header.stamp` 필수**로 발행하면 명령 경로가 완성된다.

#### 실차 테스트 시나리오

Planning 없이 실차 동작을 확인할 때는 `vehicle_test_scenarios`의 시험 runner를
쓴다. HIL 전용 `hil_command_generator`는 실차에서 사용하지 않는다. 새
터미널에서 실행한다.

**터미널 1 — scenario runner 실행:**

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

runner는 `enabled=false`로 시작하므로 이 시점에는 명령 메시지를 발행하지 않는다.
다른 터미널에서 실제 Planning이나 다른 시험 명령원이 없는지 먼저 확인한다.

**터미널 2 — 상태 확인 및 파라미터 설정:**

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 topic info /planning/command
# Publisher count: 1 (vehicle_test_scenario_runner 하나)이어야 함

ros2 param set /vehicle_test_scenario_runner enabled false
ros2 param set /vehicle_test_scenario_runner brake_override false
# 시나리오 설정 전 준비 상태: 명령 발행 중지 + software brake override 해제
```

두 파라미터의 `true`/`false` 의미는 다음과 같다. 각 쌍에서 필요한 명령 하나만
실행한다.

`enabled` — `/planning/command` 발행 여부:

```bash
# true: 선택한 시나리오를 20 Hz로 발행해 주행/조향 명령을 시작
ros2 param set /vehicle_test_scenario_runner enabled true
```

```bash
# false: /planning/command 발행 중지. brake 명령을 보내는 것은 아님
ros2 param set /vehicle_test_scenario_runner enabled false
```

`brake_override` — 활성 시나리오를 software brake 명령으로 덮어쓸지 여부:

```bash
# true: enabled=true일 때 speed 0 + steer 0 + brake_engage true를 계속 발행
ros2 param set /vehicle_test_scenario_runner brake_override true
```

```bash
# false: software brake override 해제. enabled=true면 선택한 시나리오가 다시 발행됨
# 물리 E-stop 상태를 바꾸는 명령은 아님
ros2 param set /vehicle_test_scenario_runner brake_override false
```

아래 블록 중 **시험할 시나리오 하나만** 설정한다. 파라미터 단위는 속도
`km/h`, 조향 중앙 등가각 `deg`, 시간 `sec`다.
일정 속도 직진·속도 step/ramp·제동 시험은 조향 모터를 Disable한다.
이 시험들은 대기·카운트다운·완료 후에도 `steering_center_deg`를 사용하지 않는다.
조향 sweep/slalom·square와 별도 조향 step 시험에서만 센터각을 설정한다.

##### 일정 속도 직진 — 기본 3 km/h

```bash
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner scenario straight
```

##### 속도 step — 1↔3 km/h, 2초 간격

```bash
ros2 param set /vehicle_test_scenario_runner speed_low_kph 1.0
ros2 param set /vehicle_test_scenario_runner speed_high_kph 3.0
ros2 param set /vehicle_test_scenario_runner speed_step_interval_sec 2.0
ros2 param set /vehicle_test_scenario_runner scenario speed_step
```

##### 속도 ramp — 0→3→0 km/h, 편도 5초

```bash
ros2 param set /vehicle_test_scenario_runner speed_low_kph 0.0
ros2 param set /vehicle_test_scenario_runner speed_high_kph 3.0
ros2 param set /vehicle_test_scenario_runner speed_ramp_duration_sec 5.0
ros2 param set /vehicle_test_scenario_runner scenario speed_ramp
```

##### 제동 시험 — 3 km/h 명령 3초 후 brake 유지

```bash
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner brake_delay_sec 3.0
ros2 param set /vehicle_test_scenario_runner scenario brake_test
```

##### 조향 sweep/slalom — 3 km/h, ±3°, 4초 주기

첫 실차 시험은 `3 deg`부터 시작한다. 방향과 tracking을 확인한 뒤에만 진폭을
단계적으로 올린다.

```bash
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner steering_center_deg 0.0
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 3.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 4.0
ros2 param set /vehicle_test_scenario_runner scenario steering_sweep
```

##### 직진 후 조향 sweep/슬라럼 — 5초 직진 후 3 km/h, ±3°, 4초 주기

`initial_hold_sec` 동안은 조향 중앙(0°)에서 `cruise_speed_kph`로 직진하고, 그
시간이 지나야 sweep이 시작된다. 진폭은 위와 동일하게 `3 deg`부터 시작한다.

```bash
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner steering_center_deg 0.0
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 3.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 4.0
ros2 param set /vehicle_test_scenario_runner initial_hold_sec 5.0
ros2 param set /vehicle_test_scenario_runner scenario steering_sweep
```

##### 조향 square — 3 km/h, ±3°, 8초 주기 (한쪽 4초 유지)

`steering_period_sec`의 절반 동안 `center+amplitude`를, 나머지 절반 동안
`center-amplitude`를 유지한다. center를 거치지 않고 `t=0`에서 바로 좌측
진폭으로 step하므로 sweep으로 방향을 확인한 뒤에만 사용한다.

```bash
ros2 param set /vehicle_test_scenario_runner cruise_speed_kph 3.0
ros2 param set /vehicle_test_scenario_runner steering_center_deg 0.0
ros2 param set /vehicle_test_scenario_runner steering_amplitude_deg 3.0
ros2 param set /vehicle_test_scenario_runner steering_period_sec 8.0
ros2 param set /vehicle_test_scenario_runner scenario steering_square
```

시나리오 세부 계약과 전체 파라미터는
[`vehicle_test_scenarios/README.md`](../software/src/vehicle_test_scenarios/README.md)에
있다.

### ⑤ 모드 스위치 AUTO

차량의 물리 모드 스위치를 AUTO로 올리면 자율주행이 시작된다. host에서는
모드를 바꿀 수 없다 — 스위치가 유일한 기준이다. 실제 Planning 대신 ④의 시험
runner를 쓴다면 AUTO가 확인된 뒤 마지막에 발행을 활성화한다.

```bash
ros2 topic echo --once /drive/status   # mode: 1, stop_request: false 확인
ros2 param set /vehicle_test_scenario_runner enabled true
ros2 topic echo --once /planning/command
```

활성화 이후 시나리오나 동작 파라미터를 바꾸면 해당 시나리오의 경과 시간이
0으로 초기화되어 처음부터 다시 시작한다.

MANUAL 주행은 Planning 없이도 된다(페달·기어가 보드 직결).

---

## 2. 잘 떴는지 확인

```bash
ros2 node list
# stm_a_node, stm_b_node, command_governor, steering_actuator_node,
# vehicle_state_publisher, diagnostics_publisher 여섯이 보이면 정상

ros2 topic info /planning/command     # 시험 runner 사용 시 Publisher count: 1
ros2 topic info /vehicle/command      # Publisher count: 1 이어야 함 (2 이상 = 이상)
ros2 topic echo --once /drive/status  # AUTO에서 command_received: true = 명령 도달
ros2 topic echo --once /vehicle/state # 나오면 상태 3종 전부 정상
```

---

## 3. 멈추기·끄기

| 상황 | 방법 |
|---|---|
| **비상 정지** | 물리 E-stop — launch·소프트웨어와 무관하게 항상 동작한다 |
| 자율주행만 중단 | 모드 스위치 MANUAL |
| 시험 명령으로 제동 | `ros2 param set /vehicle_test_scenario_runner brake_override true` — speed/steer 0 + brake를 계속 발행 |
| 정상 종료 | 모드 MANUAL → 시험 runner `enabled false` → runner `Ctrl-C` → Planning/launch 종료 |

다음 시험 전에 `brake_override false`를 다시 설정한다. `enabled=false`만 먼저
주면 명령 발행이 즉시 끊기고 STM_A 500 ms·조향 200 ms timeout으로 정지한다.
계획된 종료는 물리 MANUAL을 먼저 사용한다.

---

## 4. 문제가 있을 때

| 증상 | 확인 | 조치 |
|---|---|---|
| `/dev/stm_a`(또는 `stm_b`)가 없음 | `ls -l /dev/stm_*` | 보드 전원·USB 확인. 보드를 교체했다면 [5절](#5-보드-교체-시-고정-이름-갱신) |
| 조향 노드가 CAN 오류 로그 | `ip link show can0` | USB-CAN 재연결(자동 재설정됨) 후 **launch 재시작** — 조향은 CAN을 시작 때 한 번만 연다 |
| `/planning/command` publisher가 2개 이상 | `ros2 topic info -v /planning/command` | 실제 Planning·`vehicle_test_scenario_runner`·`hil_command_generator` 중복 — 하나만 남기고 종료 |
| `/vehicle/command` publisher가 2개 이상 | `ros2 topic info -v /vehicle/command` | governor를 우회하는 `steering_bench_test_node` 중복 — 실차에서는 종료 |
| AUTO인데 안 움직임 | `/drive/status`의 `command_received` | `false`면 Planning 미발행/stamp 누락. `stop_request: true`면 E-stop·브레이크 원인 확인 |
| 조향이 Enable 안 됨 | `/steering/status`의 `disable_reason` | MANUAL 모드, E-stop, 명령 끊김(200 ms), mode 미수신이 흔한 원인 |
| 명령이 끊겼다 | — | 설계된 안전 동작: 0.5초 내 구동 정지, 조향 대기. Planning 재개하면 자동 복귀 |

---

## 5. 보드 교체 시

새 NUCLEO-G474RE로 교체하면 ① 펌웨어 플래시 ② 고정 이름 갱신 ③ 역할 확인
세 가지가 필요하다. **다른 보드가 꽂혀 있으면 헷갈리기 쉬우므로, 새 보드만
USB에 꽂고 진행하는 것을 권한다.** 여러 보드가 꽂힌 채 해야 하면
[`HIL_RUNBOOK.md`](HIL_RUNBOOK.md#플래시-전-swd-target-확인)의 SWD target
확인을 먼저 거친다.

### ① 펌웨어 빌드·플래시

교체된 역할에 맞는 펌웨어를 빌드한다 (아래는 STM_A 기준, STM_B면 경로만
바꾼다).

```bash
cd ~/KAI_ws
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh
cmake -S src/Control/firmware/STM_A --preset Release
cmake --build src/Control/firmware/STM_A/build/Release
```

새 보드의 ST-LINK probe serial(SN)을 확인하고 플래시한다.

```bash
STM32_Programmer_CLI -l                     # Connected ST-LINK Probes의 SN 확인

STM32_Programmer_CLI -c port=SWD sn=<SN> mode=UR \
  -w src/Control/firmware/STM_A/build/Release/STM_A.elf -v -rst
```

`Download verified successfully`와 `MCU Reset`이 나오면 성공이다.

### ② 고정 이름 갱신

`/dev/stm_a`·`/dev/stm_b`는 보드의 ST-LINK serial에 묶여 있으므로 규칙의
serial을 새 보드 것으로 바꾼다. ①에서 확인한 SN이 그 값이다
(`ls /dev/serial/by-id/`로도 확인 가능).

```bash
# 교체된 역할(stm_a 또는 stm_b)의 serial을 새 값으로 수정
vi ~/KAI_ws/src/Control/firmware/udev/99-kaiev26-stm.rules

cd ~/KAI_ws/src/Control
sudo install -m 0644 firmware/udev/99-kaiev26-stm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty

readlink -f /dev/stm_a /dev/stm_b           # 새 장치를 가리키면 성공
```

### ③ 역할 확인·기록

serial은 장치 이름일 뿐 올라간 펌웨어가 어느 쪽인지는 별개다. Agent를 붙여
node 이름으로 역할까지 대조한다.

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm_a -b 460800
# 다른 터미널에서
ros2 node list                              # /stm_a_node 가 보여야 함 (stm_b도 동일 요령)
```

끝나면 수정한 규칙 파일과
[`../firmware/README.md`](../firmware/README.md#stm-보드-고정-장치-이름)의
serial 표를 함께 커밋해 저장소와 실물이 어긋나지 않게 한다.
