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

새 터미널마다 ROS 환경을 먼저 잡는다.

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 launch control_launch vehicle.launch.py
```

can0는 부팅 때 자동으로 준비되어 있다. 이 시점에는 차가 **안전 대기**
상태다 — 명령이 없으면 아무것도 움직이지 않는다.

### ③ 시각화 (선택이지만 권장)

각각 별도 터미널에서 아래 블록을 그대로 붙여 넣으면 된다.

Foxglove bridge (+ `/diagnostics`):

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 launch visualization foxglove_bridge.launch.py
```

디버그 대시보드 (화면 필요 — 모니터 직결이면 그대로, 원격이면 `ssh -Y`
터미널에서):

```bash
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
export ROS_DOMAIN_ID=0

ros2 launch visualization vehicle_dashboard.launch.py
```

- **Foxglove 접속**: 노트북의 Foxglove 앱에서 `ws://100.112.72.26:8765`
  (Tailscale 기준. 이 PC가 스스로 보는 IP와 노트북에서 보이는 IP가 다르니,
  안 붙으면 노트북에서 `tailscale status`로 이 PC의 IP를 다시 확인한다).
  레이아웃은 `software/src/visualization/control.json`을 import한다.
  Diagnostics 패널에서 문제 항목이 위로 정렬되어 뜬다

### ④ Planning 실행 (판단팀)

판단 노드가 `/planning/command`(`kaiev26_msgs/ActuatorCommand`)를 **20 Hz
이상, `header.stamp` 필수**로 발행하면 명령 경로가 완성된다.

Planning 없이 경로만 확인할 때는 시험 대역을 쓸 수 있다. **구동륜을 들어
올리고 E-stop을 잡은 상태에서만**:

```bash
ros2 run kaiev26_hil command_generator        # 정지 명령, 발행 꺼진 채 시작
ros2 param set /hil_command_generator enabled true
```

### ⑤ 모드 스위치 AUTO

차량의 물리 모드 스위치를 AUTO로 올리면 자율주행이 시작된다. host에서는
모드를 바꿀 수 없다 — 스위치가 유일한 기준이다.

MANUAL 주행은 Planning 없이도 된다(페달·기어가 보드 직결).

---

## 2. 잘 떴는지 확인

```bash
ros2 node list
# stm_a_node, stm_b_node, command_governor, steering_actuator_node,
# vehicle_state_publisher 다섯이 보이면 정상

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
| 정상 종료 | 모드 MANUAL → Planning 종료 → launch 터미널 `Ctrl-C` |

---

## 4. 문제가 있을 때

| 증상 | 확인 | 조치 |
|---|---|---|
| `/dev/stm_a`(또는 `stm_b`)가 없음 | `ls -l /dev/stm_*` | 보드 전원·USB 확인. 보드를 교체했다면 [5절](#5-보드-교체-시-고정-이름-갱신) |
| 조향 노드가 CAN 오류 로그 | `ip link show can0` | USB-CAN 재연결(자동 재설정됨) 후 **launch 재시작** — 조향은 CAN을 시작 때 한 번만 연다 |
| `Publisher count: 2` | `ros2 topic info -v /vehicle/command` | 시험 발행자(`command_generator`·`steering_bench_test_node`·`test_publisher`) 중복 실행 — 하나만 남기고 종료 |
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
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm_a -b 115200
# 다른 터미널에서
ros2 node list                              # /stm_a_node 가 보여야 함 (stm_b도 동일 요령)
```

끝나면 수정한 규칙 파일과
[`../firmware/README.md`](../firmware/README.md#stm-보드-고정-장치-이름)의
serial 표를 함께 커밋해 저장소와 실물이 어긋나지 않게 한다.

---

## 아직 검증 안 된 것

- 이 절차의 launch를 **실차 보드(`/dev/stm_a`·`stm_b`)로 실행한 적은 아직
  없다** (HIL bench 보드로 동형 구조 검증됨). 첫 실행은 구동륜을 들어 올리고
  2절 확인부터 통과시킨다
- 조향 캘리브레이션(`steering_motor_calibration.yaml`)의 실차 실측 검증

확인이 끝나면 이 절을 지우고 확인 날짜를 기록한다.
