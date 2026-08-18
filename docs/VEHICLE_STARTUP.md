# 실차 시동 절차

실차를 구동하는 데 필요한 것만 순서대로 담은 문서다. 배경 설명은
[`ARCHITECTURE.md`](ARCHITECTURE.md), HIL bench는
[`HIL_RUNBOOK.md`](HIL_RUNBOOK.md)를 본다.

명령 흐름은 한 줄이다:

```text
Planning ─ /planning/command → command_governor ─ /vehicle/command → STM_A(구동)·STM_B(브레이크)·조향
```

---

## 1. 최초 1회 — 실차 PC 셋업

이미 셋업된 PC라면 [2절](#2-구동-순서-매번)로 간다.

```bash
# ① 빌드
cd ~/KAI_ws
source /opt/ros/humble/setup.zsh
colcon build --symlink-install
source ~/KAI_ws/install/setup.zsh

# ② serial 권한 — 실행 후 SSH를 완전히 끊고 재접속해야 적용된다
sudo usermod -aG dialout "$USER"

# ③ 보드 고정 이름 (/dev/stm_a, /dev/stm_b)
cd ~/KAI_ws/src/Control
sudo install -m 0644 firmware/udev/99-kaiev26-stm.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty

# ④ can0 자동 준비 — 이후 부팅·USB 재연결 때마다 자동
sudo install -m 0644 host/systemd/80-can0-kaiev26.network /etc/systemd/network/
sudo systemctl enable --now systemd-networkd
```

> ④ 직후 인터넷 이름 해석(DNS)이 죽을 수 있다. **최초 활성화 때만 나타나는
> 일회성 증상**이며 재부팅 후에는 재발하지 않는다. 즉시 복구:
>
> ```bash
> nmcli dev show <wifi_인터페이스> | grep IP4.DNS     # 값 확인 후
> sudo resolvectl dns <wifi_인터페이스> <DNS1> <DNS2>
> sudo resolvectl default-route <wifi_인터페이스> yes
> ```

---

## 2. 구동 순서 (매번)

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

can0는 부팅 때 자동으로 준비되어 있다(1절 ④). 이 시점에는 차가 **안전
대기** 상태다 — 명령이 없으면 아무것도 움직이지 않는다.

### ③ Planning 실행 (판단팀)

판단 노드가 `/planning/command`(`kaiev26_msgs/ActuatorCommand`)를 **20 Hz
이상, `header.stamp` 필수**로 발행하면 명령 경로가 완성된다.

Planning 없이 경로만 확인할 때는 시험 대역을 쓸 수 있다. **구동륜을 들어
올리고 E-stop을 잡은 상태에서만**:

```bash
ros2 run kaiev26_hil command_generator        # 정지 명령, 발행 꺼진 채 시작
ros2 param set /hil_command_generator enabled true
```

### ④ 모드 스위치 AUTO

차량의 물리 모드 스위치를 AUTO로 올리면 자율주행이 시작된다. host에서는
모드를 바꿀 수 없다 — 스위치가 유일한 기준이다.

MANUAL 주행은 Planning 없이도 된다(페달·기어가 보드 직결).

### 관찰 (선택, 별도 터미널)

```bash
ros2 launch visualization foxglove_bridge.launch.py
```

노트북 Foxglove에서 `ws://<PC주소>:8765` 접속. `/diagnostics`에 문제 항목이
위로 정렬되어 뜬다.

---

## 3. 잘 떴는지 확인

```bash
ros2 node list
# stm_a_node, stm_b_node, command_governor, steering_actuator_node,
# vehicle_state_publisher 다섯이 보이면 정상

ros2 topic info /vehicle/command      # Publisher count: 1 이어야 함 (2 이상 = 이상)
ros2 topic echo --once /drive/status  # AUTO에서 command_received: true = 명령 도달
ros2 topic echo --once /vehicle/state # 나오면 상태 3종 전부 정상
```

---

## 4. 멈추기·끄기

| 상황 | 방법 |
|---|---|
| **비상 정지** | 물리 E-stop — launch·소프트웨어와 무관하게 항상 동작한다 |
| 자율주행만 중단 | 모드 스위치 MANUAL |
| 정상 종료 | 모드 MANUAL → Planning 종료 → launch 터미널 `Ctrl-C` |

---

## 5. 문제가 있을 때

| 증상 | 확인 | 조치 |
|---|---|---|
| `/dev/stm_a`(또는 `stm_b`)가 없음 | `ls -l /dev/stm_*` | 1절 ③ udev 설치, USB 재연결. 보드 전원 확인 |
| 조향 노드가 CAN 오류 로그 | `ip link show can0` | can0가 down이면 1절 ④ 설치 여부 확인 후 **launch 재시작** (조향은 CAN을 시작 때 한 번만 연다) |
| `Publisher count: 2` | `ros2 topic info -v /vehicle/command` | 시험 발행자(`command_generator`·`steering_bench_test_node`·`test_publisher`) 중복 실행 — 하나만 남기고 종료 |
| AUTO인데 안 움직임 | `/drive/status`의 `command_received` | `false`면 Planning 미발행/stamp 누락. `stop_request: true`면 E-stop·브레이크 원인 확인 |
| 조향이 Enable 안 됨 | `/steering/status`의 `disable_reason` | MANUAL 모드, E-stop, 명령 끊김(200 ms), mode 미수신이 흔한 원인 |
| 명령이 끊겼다 | — | 설계된 안전 동작: 0.5초 내 구동 정지, 조향 대기. Planning 재개하면 자동 복귀 |

---

## 아직 검증 안 된 것

- 이 절차의 launch를 **실차 장치에서 실행한 적은 아직 없다** (HIL bench
  동형 구조로 검증됨). 첫 실행은 구동륜을 들어 올리고 3절 확인부터 통과시킨다
- 조향 캘리브레이션(`steering_motor_calibration.yaml`)의 실차 실측 검증

확인이 끝나면 이 절을 지우고 확인 날짜를 기록한다.
