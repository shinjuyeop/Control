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

아래 실행 예제는 실제 차량 데스크탑의 zsh 환경을 기준으로 한다.
각 터미널에서 `cd ~/KAI_ws` 후 기존 `si` 별칭으로 workspace 환경을 불러온다.
`si`는 현재 디렉터리의 `install/setup.zsh`를 읽으므로 workspace 루트에서 실행한다.

```bash
cd ~/KAI_ws
si
ros2 launch control_launch vehicle.launch.py
```

```bash
# 공중 시험
ros2 launch control_launch vehicle.launch.py tuning_preset:=airborne

# 접지 정지 시험
ros2 launch control_launch vehicle.launch.py tuning_preset:=driving
```

옵션 없이 실행하면 **주행 프리셋(`driving`)**을 사용한다. 매번 튜닝 TUI를 열 필요는 없다.

`can0` 자동 준비는 호스트 설정을 설치한 PC에서만 적용된다. 실행 전에
`ip -details link show can0`로 확인한다. AUTO에서 명령이 없으면 STM_A는 HALTED,
조향은 명령 대기다. MANUAL의 물리 페달 경로는 별도로 동작한다.

공통 제어 설정은 `ros2 run control_tuning control_tuning_tui`로 편집한다.
상단에서 **주행 / 공중**를 선택하면 조향 응답·출력 제한·보호·운용·수신 시간
분류가 나타난다. `S`는 선택한 프리셋 YAML을 저장하고, 해당 프리셋으로 다음 launch를
실행할 때 반영된다. 실행 중 Torque 튜닝은 MANUAL·정지 상태에서 `A → Enter`로
적용할 수 있다. 운용·수신 시간은 저장 후 기존 launch를 완전히 종료하고 같은 프리셋으로
다시 실행한다. [저장 경로·단위·조작 안내](../software/src/control_tuning/README.md)를 따른다.

두 프리셋 `airborne`/`driving`은 schema v2로 lateral/longitudinal/governor/timeouts를 보관한다.
종방향은 first-test candidate이며 실차 성능 검증값이 아니다. 새 STM_A는 부팅 시 speed DEFAULT,
launch는 MANUAL 정지·무전류에서 선택한 calibration을 한 번 적용하고 readback을 확인한다.
실패/timeout이면 현재 모드를 확인하고 TUI Longitudinal에서 A로 적용한다.
공중 Torque는 8 A/RPM300, 주행 Torque는 480 A/RPM4500이다. Speed fallback의 기존
480 A 경로는 그대로이므로 공중 Torque 제한이 Speed에 적용된다고 가정하지 않는다.
[airborne/A-B 절차와 실제 초기값](reports/2026-09-21_longitudinal_torque_pi.md)을 확인한다.
접지 정지는 별도 calibration preset이 아니라 시나리오 시험 조건이다.

시나리오 TUI에서는 오른쪽 설정의 `MCAP 녹화`와 `시험 조건`에서 Enter를 누른 뒤
↑↓로 고르고 Enter로 확정한다. 시험 조건은 기록용이며 제어 프리셋을 적용하지 않는다.
실행 중인 조향 노드의 launch 프리셋을 시험 조건 기본값으로 가져오므로 보통은 다시
선택할 필요가 없다. 수동 선택은 유지되며 이전 버전 노드에서는 직접 선택한다.
`R`을 누르면 녹화가 켜진 경우 실제 조향·거버너 파라미터 조회와 저장을 마친 뒤
AUTO를 기다린다. 조회 실패 시 시험을 시작하지 않는다. `control_parameters.yaml`과
MCAP의 `/sysid/control_parameters`, `/parameter_events`, 토크 상세 토픽으로 분석한다.

### ③ 시각화 (선택이지만 권장)

각각 별도 터미널에서 아래 블록을 그대로 붙여 넣으면 된다.

KAI Vehicle Console (브라우저에서 차량 상태만 관찰, 명령 발행 없음):

```bash
cd ~/KAI_ws
si
ros2 launch vehicle_console vehicle.launch.py \
  allowed_hosts:=100.112.72.27 \
  web_python:="$PWD/src/Control/software/src/vehicle_console/.venv/bin/python"
```

연결할 주소:

- **차량 PC 로컬 브라우저:** `http://127.0.0.1:8766/`
- **Tailscale 원격 브라우저:** `http://100.112.72.27:8766/`

`100.112.72.27`은 기존 차량 PC 주소 예시다. 실제 주소를 확인해 `allowed_hosts`와
접속 주소를 함께 바꾼다. Tailscale Serve 설정이 없는 PC에서의 등록·해제나
SSH 방식은
[Console 원격 사용](../software/src/vehicle_console/README.md#원격-접속)을 따른다.

Qt dashboard는 archive로 이동했다.

Foxglove bridge — 팀 단위로 PC당 하나만 띄운다. 이미 떠 있으면 켜지 않고
아래 접속만 한다 (`/diagnostics` 요약은 구동 스택이 이미 발행 중이라
bridge만 켜면 된다):

```bash
cd ~/KAI_ws
si
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
이상, `header.stamp` 필수**로 발행하면 명령 경로가 완성된다. 출발은 차량의 물리
모드 스위치를 AUTO로 올려 시작하며 host에서 모드를 바꾸지 않는다.

#### 실차 테스트 시나리오

Planning 없이 실차 동작을 확인할 때는 `vehicle_test_scenarios`의 TUI runner를
쓴다. HIL 전용 `hil_command_generator`는 실차에서 사용하지 않는다.

```bash
cd ~/KAI_ws
si
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

TUI를 열기만 하면 `/planning/command`를 발행하지 않고 MCAP도 시작하지 않는다.
화면에서 현재 토픽·명령 발행자의 GO/NO-GO 원인을 먼저
확인한다. 실제 Planning이나 다른 시험 명령원이 있으면 Run은 NO-GO다.

시험은 다음처럼 분류한다.

| 상위 분류 | 하위 분류 | 시험 |
| --- | --- | --- |
| 입력 응답 시험 | 종방향 입력 | 일정 속도 직진, 속도 Step, 속도 Ramp, 제동 입력 |
| 입력 응답 시험 | 횡방향 입력 | 조향 양방향 Step, 고정 주기 Sine, Chirp 주파수 Sweep |
| 경로 추종 시험 | — | K-City 예선 경로, K-City 본선 경로 |

`↑/↓`로 항목을 이동하고 Enter로 선택한다. `←/→`로 시험 목록과 파라미터 영역을
오가며, 파라미터에서 Enter를 누르면 값을 직접 수정할 수 있다. 단위는 화면에
`km/h`, `deg`, `sec`, `Hz`로 표시된다. 조향각은 중앙 등가 앞바퀴 각도다.

| 키 | 동작 |
| --- | --- |
| `↑` / `↓` | 시험·파라미터 이동; 선택형 항목 편집 중에는 값 전환 |
| `←` / `→` | 시험 목록·파라미터 영역 이동 |
| `Enter` | 시험 선택, 값 편집·확정 |
| `Esc` | 값 편집 취소 |
| `R` | 설정 고정, 선택한 경우 MCAP 시작, AUTO 대기 |
| `S` / `Space` | 속도 0·브레이크 요청 후 기록 종료 |
| `Q` | 안전 정리 후 TUI 종료 |

모든 시나리오의 기본 속도(낮은·높은 속도 포함)는 `0 km/h`, 조향 중심은 `0 deg`다.
조향 진폭은 0으로 바뀐 것이 아니므로 횡방향 시험은 정지 상태에서도 조향 입력을
생성할 수 있다. 실제 시험값은 `R` 전에 설정한다.

종방향 입력은 조향 모터를 Disable하며, 횡방향 입력은 설정된 센터각·진폭을
사용한다. 조향 입력은 목표속도 부근에서 1초 동안 안정된 뒤 시작하며, 15초 안에
정속 판정이 되지 않으면 경고 후 입력을 계속한다. Square는
일반 TUI에서 제외했지만 기존 headless `steering_square`와 고정 YAML launch는
기존 입력 형식 호환용으로 유지한다. 이 YAML도 기본 속도·중심은 0이다.

TUI가 실행되는 동안에는 이전 셸 명령과 로그를 숨기고 종료 시 원래 화면을 복원한다.
상단 상태, RUN 조건, 시험·파라미터, 하단 키 안내의 패널 순서는 고정이며
조건 정렬과 경고·파라미터에 따라 줄 수는 달라진다. 화면 전체를 주기적으로
지우지 않고 변경된 줄을 모아 출력한다. 터미널보다 길면 마우스 휠 또는 `PgUp/PgDn`으로 TUI 전체를
위아래로 스크롤한다. `↑/↓`는 항목을 이동하거나 편집 중인 선택값을 전환한다.
정지·오류 안내는 상단 `VEHICLE STATE · 시험 상태`에 표시한다.

`R`을 누르면 설정을 고정하고 실행 중 편집을 잠근다. 파라미터 마지막의
`MCAP 녹화` 항목에서 Enter → ↑↓ → Enter로 녹화 여부를 선택한다. ON이면 기존
workspace 기록 경로에 저장하며 외장 SSD 마운트를 요구하지 않는다.

```text
~/KAI_ws/records/<시간>_<scenario>_v<최고속도>/
```

녹화 OFF이면 파일과 recorder를 생성하지 않고 시험한다. 경로 시험의 위치추정·
추종 프로세스는 녹화 여부와 관계없이 실행하고 종료 시 정리한다.
ON이면 MCAP 연결 후, OFF이면 시험 준비 후 AUTO를 기다린다.
`R`을 누른 뒤 차량의 물리 모드 스위치를 AUTO로 올린다. TUI가 AUTO를 확인하면
5초를 카운트다운한 뒤 입력을 시작한다. 이미 AUTO여도 `R` 이후 새로 5초를 센다.
정상적인 실행 중 MANUAL 전환은 Run을 끝낸다. 다만 조향 설정 응답 대기 중
mode·통신 검사와 조향 software fault 감지에는
[미수정 제한](SAFETY.md#실차-시나리오-실행기의-현재-제한)이 있다.
재시험은 현재 Run을 정지한 뒤 새 `R`로 시작한다. 설정과 기록 위치, 세부 계약은
[`vehicle_test_scenarios/README.md`](../software/src/vehicle_test_scenarios/README.md)에
있다.

---

## 2. 잘 떴는지 확인

```bash
ros2 node list
# stm_a_node, stm_b_node, command_governor, steering_actuator_node,
# vehicle_state_publisher, diagnostics_publisher, auto_dashboard_summary,
# vehicle_robot_state_publisher 등 기본 launch 구성과 MCU 노드를 확인

ros2 topic info /planning/command     # 시험 runner 사용 시 Publisher count: 1
ros2 topic info /vehicle/command      # Publisher count: 1 이어야 함 (2 이상 = 이상)
ros2 topic echo --once /drive/status  # AUTO에서 command_received: true = 명령 도달
ros2 topic echo --once /vehicle/state # 상태 3종이 최소 한 번 수신됨; 현재 freshness 보장은 아님
```

---

## 3. 멈추기·끄기

| 상황 | 방법 |
|---|---|
| **비상 정지** | 물리 E-stop — launch·소프트웨어와 무관하게 항상 동작한다 |
| 자율주행만 중단 | 모드 스위치 MANUAL |
| 시험 명령으로 제동 | TUI에서 `S` 또는 `Space` — speed 0 + brake를 발행하고 MCAP을 마감 |
| 정상 종료 | TUI `S` → 기록 저장 확인 → 모드 MANUAL → `Q` → Planning/launch 종료 |

비상 상황에서는 TUI보다 물리 E-stop과 MANUAL 전환을 우선한다. `S` 이후 화면에서
MCAP 저장 완료(녹화 OFF이면 시험 정리 완료)를 확인하고 다음 시험을 선택한다.

---

## 4. 문제가 있을 때

| 증상 | 확인 | 조치 |
|---|---|---|
| `/dev/stm_a`(또는 `stm_b`)가 없음 | `ls -l /dev/stm_*` | 보드 전원·USB 확인. 보드를 교체했다면 [5절](#5-보드-교체-시) |
| 조향 노드가 CAN 오류 로그 | `ip link show can0` | USB-CAN 재연결(자동 재설정됨) 후 **launch 재시작** — 조향은 CAN을 시작 때 한 번만 연다 |
| `/planning/command` publisher가 2개 이상 | `ros2 topic info -v /planning/command` | 실제 Planning·TUI `/auto_sysid`·headless runner·`hil_command_generator` 중복 — 하나만 남기고 종료 |
| `/vehicle/command` publisher가 2개 이상 | `ros2 topic info -v /vehicle/command` | governor를 우회하는 `steering_bench_test_node` 중복 — 실차에서는 종료 |
| AUTO인데 안 움직임 | `/drive/status`의 `command_received` | `false`면 Planning 미발행/stamp 누락. `stop_request: true`면 E-stop·브레이크 원인 확인 |
| 조향이 Enable 안 됨 | `/steering/status`의 `disable_reason` | MANUAL 모드, E-stop, 명령 끊김(200 ms), mode 미수신이 흔한 원인 |
| 조향이 `FAULT`, `/steering/torque_safety=rate_limit` | 실제 조향속도 120 deg/s 초과 여부 | 시험 정지·원인 확인 후 **구동용** `control_launch vehicle.launch.py` 완전 종료·재실행. fresh 상태·새 명령 필요; 원인이 남으면 재잠금 |
| 명령이 끊겼다 | — | 설계된 안전 동작: 0.5초 내 구동 정지, 조향 대기. 다른 차단 조건이나 torque latch가 없으면 유효 명령 재개 후 복귀 |

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
