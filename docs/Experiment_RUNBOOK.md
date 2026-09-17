# Experiment RUNBOOK — 토크 모드 조향 SYSID

조향 제어기 개발을 위한 **실험(SYSID·튜닝) 전용** 절차 문서다. 실차 상시
운용은 [`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md), 배경 설계는
[`ARCHITECTURE.md`](ARCHITECTURE.md)·[`INTERFACES.md`](INTERFACES.md)를 따른다.
이 문서는 그 운용 절차 중 실험에 필요한 부분을 발췌해 자립적으로 구성했다.

목적: 방법 A(피드포워드 + 피드백) 토크 모드 조향 제어에 필요한 파라미터를
무부하/접지에서 획득한다. 제어 법칙은 다음과 같다.

```
τ = J·α_ff + Kp·e − Kd·ω + F_c·sign(e)
    (피드포워드) (P)   (D)    (마찰/데드존 보상)
e = θ_target − θ_actual,  ω = 각속도,  α_ff = 목표 각가속도
```

> 상태: 방법 A 제어기는 `steering_actuator_node`의 `actuation_mode=torque` 경로에
> 구현·배선돼 있다(단위/노드 테스트 통과, **실차 응답 미검증**). T0–T2는 기존
> `deadzone_measure_node`, T3는 `steering_actuator_node` + 시나리오 runner로 실행한다.
> 아래 명령은 실제 노드·파라미터 이름으로 확정했다.

> 구현 위치: `software/src/lateral_ctrl/src/torque_steering_controller.cpp`의 PD + 안전
> 계층에 마찰 보상 `F_c·sign(e)`(데드밴드), 관성 피드포워드 `J·α_ff`, 속도별
> `F_c(v)` 스케줄을 얹었다. 운용 계약·전체 파라미터 표는
> [`TORQUE_CONTROL.md`](../software/src/lateral_ctrl/TORQUE_CONTROL.md), 배경 전체는
> [`torque_steering_overview.md`](torque_steering_overview.md) 참조.

---

## 1. 안전 (실험 공통, 최우선)

- 모든 SYSID는 **바퀴를 띄운 무부하**에서 먼저 한다. 접지 재검증은 무부하 확정 후.
- 물리 E-stop 3개(dashboard/side/RF)와 MANUAL 전환을 항상 우선한다.
  ([`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md) 1절 ① 발췌)
- **launch가 뜨면 조향은 바로 armed다.** 배선·E-stop 확인 전 launch 금지.
- 토크 모드(`0020=2`)는 방향 부호가 틀리면 **즉시 발산**한다.
  실험 **T0(부호 검증)** 통과 전 다른 토크 실험 금지.
- 토크 지령은 항상 `±TORQUE_LIMIT`(초기 ±8000 = 정격 80% 이하)로 클램프한다.
  80% 보호(정격 80% 상전류에서 5 s 후 출력 차단)를 원천 회피.
  - T0–T2: `deadzone_measure_node`가 raw ±8000으로 하드 클램프한다.
  - T3: `torque_peak_limit_raw`/`torque_sustained_limit_raw`가 상한이다. 현재 운용 YAML은
    peak 10000이므로 **실험 중에는 8000 이하로 내려서 쓴다**(5.5절 T3).
- **watchdog**: 매 20 ms 토크 프레임을 무조건 송신한다(토크 0이어도). 두 노드 모두
  매 tick 송신한다. 1 s 미송신 시 드라이버가 CAN 단선으로 보고 보호 진입 → 복구는
  `enable` 프레임 재송신(전원 재시작 아님).
- 각도 한계:
  - T3(`steering_actuator_node`): calibration `±3279 count`(약 −27.8°/+25.9°)를 hard
    envelope로 쓰고 안쪽 1° margin에서 바깥 토크를 줄인다. 넘으면 latched FAULT.
  - T0–T2(`deadzone_measure_node`): **각도 한계가 `max_angle_delta` 하나뿐이다.**
    기본값 0(비활성)이므로 차량 장착 상태에서는 반드시 값을 준다. 단위는 모터축
    `motor_angle_deg`이며 중앙→끝 ±3279 count ≈ **±118° 모터축 ≈ ±26° 바퀴**다.
- 동일 KEYA에 토크를 보내는 노드는 **항상 하나**다. 출력이 armed인
  `steering_actuator_node`와 `deadzone_measure_node`를 동시에 띄우지 않는다.

---

## 2. 공통 ROS 환경

새 zsh 터미널마다 설정한다. `humble` 함수가 Humble 환경 + `ROS_DOMAIN_ID=0`을
잡는다. 이미 활성화된 셸에서 domain을 바꿨다면 그때만 `export ROS_DOMAIN_ID=0`.
([`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md) 1절 ②, [`HIL_RUNBOOK.md`](HIL_RUNBOOK.md) 4절 발췌)

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh
```

빌드가 필요할 때(소스 변경 시):

```zsh
cd ~/KAI_ws
humble
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --packages-up-to lateral_ctrl vehicle_test_scenarios control_launch
source ~/KAI_ws/install/setup.zsh
```

---

## 3. 구동 스택과 확인

실험마다 띄우는 스택이 다르다. **두 경우를 섞지 않는다.**

| 실험 | 조향 노드 | 토크 송신원 | 구동 스택 |
|---|---|---|---|
| T0–T2 | `steering_actuator.launch.py` **읽기 전용** 단독 | `deadzone_measure_node` | 띄우지 않음(또는 `steering:=false`) |
| T3, 6절 | `vehicle.launch.py actuation_mode:=torque` (armed) | `steering_actuator_node` | 필요(STM drive/brake status, AUTO) |

`vehicle.launch.py`는 조향을 armed로 포함하므로, T0–T2 중에 기본 인자로 띄우면
측정 노드와 같은 모터에 두 송신원이 생긴다. T0–T2에서 STM 상태 관측이 필요하면
`ros2 launch control_launch vehicle.launch.py steering:=false`로만 띄운다.

can0는 부팅 시 자동 준비된다. T3용 구동 스택을 띄우면 차는 **안전 대기** 상태다.
([`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md) 1절 ②·2절 발췌)

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

# T3: 조향은 비활성으로 시작한다. 시나리오 runner가 활성화한다.
ros2 launch control_launch vehicle.launch.py actuation_mode:=torque steering_enabled:=false
```

관측(선택, 별도 터미널): Foxglove bridge는 PC당 하나만.

```zsh
ros2 launch foxglove foxglove_bridge.launch.py
# 노트북 Foxglove 앱에서 ws://100.112.72.27:8765, control.json import
# "조향 Torque" 탭의 "P / D / FF / 마찰 / 토크 명령" 패널에 방법 A 각 항이 나온다.
```

잘 떴는지 확인(T3):

```bash
ros2 node list
# stm_a_node, stm_b_node, command_governor, steering_actuator_node,
# vehicle_state_publisher, diagnostics_publisher 여섯이 정상
ros2 topic info /vehicle/command      # Publisher count: 1 (2 이상이면 이상)
ros2 topic echo --once /steering/actuation_mode   # data: torque
ros2 topic echo --once /steering/status
```

SYSID 명령원은 **하나만** `/planning/command` 경로에 관여한다. 시나리오 runner와
실제 Planning을 동시에 실행하지 않는다(발행자 충돌).

---

## 4. KEYA 펌웨어 설정 (토크 모드 전환)

실험 전 KEYA Motor Monitor(또는 CAN 파라미터 쓰기)로 설정한다. Host는 `0020`을
읽거나 바꾸지 않는다. 상세 근거는 [토크 모드 방법 A 설계](torque_mode_method_A_design.md) 참고.

| No | 이름 | 현재 | 실험용 조치 |
|---|---|---|---|
| 0020 | Work mode | 3 | **→ 2 (Torque). 필수**, Burn 후 전원 재기동 |
| 0026 | Direction | 0 | **0 유지.** 부호는 T0 결과로 ROS `torque_direction_sign`에서 맞춘다 |
| 0028 | Overload time | 1 | 데드존 돌파 대전류와 충돌 여부 관찰 |
| 0003 | Max Current | 20 | 유지, TORQUE_LIMIT·`keya_protection_current_a`와 정합 확인 |
| 0005/0006 | Current PI | 2.0/0.01 | **유지**(안쪽 전류 루프) |
| 0007/0008 | Speed PI | 0.8/0.01 | 무효(안 쓰임) |
| 0009–0012 | Position PID | 2.0/../2.0 | 무효 → ROS 바깥 루프로 이전 |
| 0013/0030 | Accel/Decel | 10/5 | 무효 가능(실측 확인) |

전류→토크 스케일(데이터시트 확정): 지령 `−10000~+10000` = ∓정격토크,
제한전류 = (지령‱) × 정격전류 × 80%.

---

## 5. 실험 절차 T0–T3

각 run은 `records/<YYYYMMDD_HHMMSS_us>_<label>/rosbag/`에 MCAP 저장.
`/steering/status` 필수, 접지 시 `/drive/status`(속도) 포함.
전류(`motor_current_a`)는 **1 A 양자화**이므로 피드백 금지, 참고·안전 감시용.
피드백은 각도(`actual_wheel_angle_rad` / `motor_angle` 0.036°/count)만 사용.

### T0 — 방향 부호 검증 (최우선, 무부하)

목적: +토크 지령이 +조향각(LEFT)을 만드는지 확인. 반대면 `torque_direction_sign=-1`.
**통과 전 다른 토크 실험 금지.**

1. 바퀴 띄움, 조향 중앙 부근 정렬.
2. `0020=2`, enable.
3. 작은 고정 토크(지령 +300 ≈ breakaway의 3배, 정격 3%) 1.5 s 인가.
4. `actual_wheel_angle_rad` 부호 확인. +토크→+각도면 정상.
5. +/− 양방향 확인(대칭성).

label: `T0_sign_check` · 판정: +지령↔+각도 일치 시 통과. 명령은 5.5절.

### T1 — 마찰(정지/운동) 측정 (무부하)

목적: F_c(정지·breakaway), F_v(운동) 획득.

1. T0 통과 상태에서 시작.
2. 토크 지령을 breakaway 아래(예 60)부터 계단 상승(+10/단계, 0.3–1 s 유지).
3. 각 단계 `SWEEP` 로그의 `delta`(단계 동안 모터축 각도 변화) 관찰.
   - 처음 움직이는 토크 = F_c. 움직인 뒤 등속을 유지하는 토크 = F_v.
4. +/− 양방향(비대칭 가능).
5. 각도 한계(`max_angle_delta`)에서 자동 중단. 대전류가 `0028` 보호에 걸리면
   단계 유지시간 축소.

label: `T1_friction_sweep_pos` / `_neg` · 산출: F_c, F_v(지령 단위).
주의: 무부하에서는 breakaway 직후 가속이 계속돼 몇 단계 안에 각도 한계에 닿는다.
F_v는 거칠게만 나온다. 방법 A에 필요한 값은 F_c다.

### T2 — 등가 관성 J 측정 (무부하)

목적: 피드포워드 계수 J. J = (τ_지령 − 마찰) / α.

1. T1의 F_c를 알고 시작.
2. 정지에서 계단 토크(F_c보다 충분히 큰 값)를 인가.
3. 인가 직후 초기 **각가속도 α**를 각도에서 구한다(2차 다항식 fit).
4. τ_net = τ_지령 − 마찰 → J = τ_net / α.
5. 여러 토크 레벨 반복 → (τ, α) 회귀로 J(기울기)와 마찰(절편)을 함께 확인.
   전류 양자화 무관(지령 토크값 사용).

label: `T2_inertia_step_lvl<torque>` · 산출: J(지령단위 실효 J, raw/(rad/s²)).
SI 단위가 필요하면 τ_rated를 확인한다. `motor_speed_rpm`(이상치 예 −983040 관측)은
쓰지 않고 각도만 쓴다.

### T3 — 바깥 루프 PD 튜닝 + 데드밴드 (무부하)

목적: Kp, Kd(큰 각 ζ≈0.64 재현), 데드밴드 ε 결정. 방법 A 전체 조립.

1. F_c=0, J_ff=0으로 **순수 PD만** 활성화.
2. 큰 각(±15/18°) step으로 Kp, Kd 튜닝 → OS 7–8%(ζ≈0.64).
3. 마찰 F_c(T1) 추가 → 작은 각(±3/5°) 정착오차·오버슛 개선 확인.
4. 목표 근처 미세 떨림 있으면 데드밴드 ε 조정(정착 허용오차와 트레이드오프).
5. 피드포워드 J·α_ff(T2) 추가 → 빠른 궤적(sine) 지연/오버슛 개선 확인.
6. 파라미터는 `ros2 param set`으로 실시간 조정(Burn 불필요).

TUI 기록 폴더: `<시간>_steering_step_v0` / `<시간>_steering_sine_v0` / `<시간>_steering_chirp_v0`
분석 지표: %OS, ζ = −ln(OS)/√(π²+ln²OS), 정착오차, 명령-실측 상관.

---

## 5.5 명령어 가이드 (시나리오별 실행)

VEHICLE_RUNBOOK의 시나리오 명령 스타일을 따른다. 각 실험마다 (1) 노드 실행,
(2) 파라미터 설정, (3) 시작 트리거, (4) MCAP 녹화 순서다.

### 공통 — T0–T2: 읽기 전용 조향 노드 + 녹화

T0–T2는 `steering_actuator_node`를 **읽기 전용**으로 단독 실행한 위에서 한다(3절 표).
이 노드는 조회 프레임만 보내 `/steering/status`를 발행하고, 토크 명령은 측정
노드가 보낸다.

```zsh
# 터미널 A — 읽기 전용 조향 (조회 프레임만, 모터 출력 잠금)
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch lateral_ctrl steering_actuator.launch.py \
  can_interface:=can0 actuator_output_enabled:=false
# 로그에 "READ-ONLY: polling the motor ..."가 찍히는지 확인
```

```zsh
# 터미널 B — MCAP 녹화 (label을 실험마다 바꾼다)
LABEL=T0_sign_check   # 실험별로 교체
STAMP=$(date +%Y%m%d_%H%M%S_%6N)
mkdir -p ~/KAI_ws/records/${STAMP}_${LABEL}/rosbag
cd ~/KAI_ws/records/${STAMP}_${LABEL}/rosbag
ros2 bag record -s mcap /steering/status /deadzone/start /deadzone/stop
```

`0020=2`(Torque)와 Burn은 실험 전 KEYA Motor Monitor에서 완료한다(4절).

`deadzone_measure_node` 동작에서 명령을 짤 때 알아야 할 점:

- 시작 토크는 `start_cmd`이고, `step_hold_ms`마다 `ramp_step`씩 올린다.
- 올린 값이 `|cmd| >= torque_limit`이 되는 순간 **그 값을 유지하지 않고 바로
  Disable**한다. 따라서 유지하고 싶은 최고 토크보다 `torque_limit`을 한 단계 크게 준다.
- `sweep_mode=false`면 hold가 끝났을 때 각도가 1 count라도 변했으면 Disable한다.
- `max_angle_delta`는 **매 tick** 검사하는 유일한 각도 안전 중단이다(모터축 deg).
- `sweep_mode`, `start_cmd`는 **시작 시에만** 읽는다. 바꾸려면 노드를 재시작한다.
  `ramp_direction`, `torque_limit`, `ramp_step`, `ramp_period_ms`, `step_hold_ms`,
  `max_angle_delta`는 `ros2 param set`으로 바로 바뀐다.
- 각 run 전에 `motor_angle_deg`가 중앙(±10°) 근처인지 본다. 벗어나 있으면 반대
  방향 run으로 되돌리거나, Disable 상태에서 손으로 중앙에 맞춘다.

---

### T0 — 방향 부호 검증

`start_cmd`로 +300을 바로 걸고 1.5 s 유지한다. 움직이면 그 hold 끝에서 Disable하고,
안 움직이면 다음 단계(600)가 `torque_limit`에 닿아 즉시 Disable한다.

```zsh
# 터미널 C — 측정 노드
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 run lateral_ctrl deadzone_measure_node can0 --ros-args \
  -p start_cmd:=300 \
  -p ramp_step:=300 \
  -p torque_limit:=600 \
  -p step_hold_ms:=1500 \
  -p ramp_direction:=1 \
  -p max_angle_delta:=20.0
```

```zsh
# 시작: +방향 소토크 인가
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once
ros2 topic echo /steering/status --field actual_wheel_angle_rad
#   +토크(dir=+1) → actual_wheel_angle_rad 증가 + 물리적으로 LEFT면 정상.

# −방향도 확인 (노드 재시작 불필요)
ros2 param set /deadzone_measure_node ramp_direction -1
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once
```

판정 반영: 정상이면 `torque_direction_sign: 1` 유지. 반대면
`software/src/lateral_ctrl/config/steering_actuator.yaml`의 `torque_direction_sign: -1`로
바꾸고 재빌드/재시작한다(startup-only). KEYA `0026`은 Direction=0으로 둔다.

> 방향 규약: **왼쪽(LEFT)이 +**다. 현재 calibration(r3)은 +count = +바퀴각 = 좌회전이고,
> `deadzone_measure_node` 로그의 `motor_angle_deg`(= count × 0.036°)도 같은 부호다.
> 따라서 정상이면 +지령에서 `motor_angle_deg`, `motor_position_count`,
> `actual_wheel_angle_rad`가 모두 증가하고 바퀴는 물리적으로 왼쪽으로 돈다.
> 셋 중 하나라도 어긋나면 부호를 바꾸지 말고 중단한 뒤 장착·calibration을 확인한다.
> [`DEADZONE_MEASUREMENT.md`](../software/src/lateral_ctrl/DEADZONE_MEASUREMENT.md)의
> "dir=+1 → 각도 감소" 기록은 부호를 뒤집은 옛 `/ctrl/steer_handle_hires` 기준이라
> 지금 축과 반대로 적혀 있다. 그 기록만 보고 `-1`로 바꾸지 않는다.

label: `T0_sign_check`

---

### T1 — 마찰(정지/운동) 측정

`sweep_mode:=true`로 breakaway에서 멈추지 않고 계속 올리며 단계마다
`SWEEP,dir=..,cmd=..,rated_pct=..,current=..,angle=..,delta=..` 로그를 남긴다.
기존 데드존 측정에서 F_c ≈ 106(정방향)/−106(역방향)이 초기값이라 그 아래(60)부터
10씩 올린다. 움직인 뒤에는 각도 한계(90° 모터축 ≈ 20° 바퀴)에서 자동 중단된다.

```zsh
# 터미널 C — 계단 sweep (정방향)
ros2 run lateral_ctrl deadzone_measure_node can0 --ros-args \
  -p sweep_mode:=true \
  -p start_cmd:=60 \
  -p ramp_step:=10 \
  -p torque_limit:=400 \
  -p ramp_period_ms:=20 \
  -p step_hold_ms:=300 \
  -p ramp_direction:=1 \
  -p max_angle_delta:=90.0
```

```zsh
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once
# 로그 SWEEP 줄을 CSV로 확보한다(터미널 출력 저장 또는 ~/.ros/log).
#   delta가 처음 0이 아닌 cmd = F_c
#   이후 delta/step_hold가 거의 일정한 구간의 cmd = F_v
# "ANGLE LIMIT ABORT"는 정상 종료다.

# 역방향 (중앙으로 되돌린 뒤)
ros2 param set /deadzone_measure_node ramp_direction -1
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once
```

관측 시간 의존성 확인: `step_hold_ms`를 1000, 8000으로 바꿔 반복한다(런타임 변경 가능).
느린 creep만 보려면 `start_cmd:=100 ramp_step:=2 torque_limit:=112 step_hold_ms:=8000`으로
**노드를 재시작**한다(`start_cmd`는 startup-only).

label: `T1_friction_sweep_pos` / `T1_friction_sweep_neg`
주의: F_c는 `step_hold_ms`(관측 시간)에 종속적이다(300 ms→110, 8 s→106). 방법 A의
`torque_friction_raw`에는 **짧은 hold(300 ms)** 값을 우선 쓴다. 제어 주기(20 ms)
안에 돌파해야 하는 토크라 긴 hold의 creep 문턱보다 크다.

---

### T2 — 등가 관성 J 측정

단발 계단: `start_cmd`로 레벨 L을 바로 걸고, `torque_limit`을 L보다 크게 줘서
hold 동안 유지되게 한다. hold가 끝나거나 `max_angle_delta`에 닿으면 Disable한다.
레벨은 낮은 값부터 올린다. 무부하 가속이 커서 높은 레벨일수록 한계까지 샘플 수가
적어진다(50 Hz status 기준 fit에 최소 4샘플 필요).

```zsh
# 레벨마다 start_cmd/torque_limit을 바꿔 노드를 재시작 (예: 300, 500, 800, 1200)
LVL=300
ros2 run lateral_ctrl deadzone_measure_node can0 --ros-args \
  -p sweep_mode:=false \
  -p start_cmd:=${LVL} \
  -p ramp_step:=1 \
  -p torque_limit:=$((LVL + 1)) \
  -p step_hold_ms:=300 \
  -p ramp_direction:=1 \
  -p max_angle_delta:=90.0
```

```zsh
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once
# 같은 레벨을 2–3회 반복해도 된다. 각 run 사이 1 s 이상 정지해 있으면
# 분석 스크립트가 움직임 시작을 따로 잡는다.
```

분석(레벨별 bag을 `bag:토크` 쌍으로 넘긴다):

```zsh
ros2 run lateral_ctrl torque_sysid_metrics.py inertia \
  ~/KAI_ws/records/<..._T2_inertia_step_lvl300>/rosbag/<rosbag2_...>:300 \
  ~/KAI_ws/records/<..._T2_inertia_step_lvl500>/rosbag/<rosbag2_...>:500 \
  ~/KAI_ws/records/<..._T2_inertia_step_lvl800>/rosbag/<rosbag2_...>:800 \
  --friction-raw 110 --csv ~/KAI_ws/records/T2_inertia.csv
# 출력: run별 α, "F_c 고정" J, 그리고 레벨 2개 이상이면 τ = J·α + F 회귀의 J와 절편 F.
#       절편 F가 T1의 F_c와 크게 다르면 fit 구간(--fit-ms, 기본 100)이나 레벨을 재검토.
```

label: `T2_inertia_step_lvl<torque_limit>`
주의: α는 `/steering/status.actual_wheel_angle_rad`에서만 구한다. 전류·rpm은 안 쓴다.

---

### T3 — 방법 A 제어 (`steering_actuator_node`, torque 모드)

T3부터는 측정 노드를 **종료**하고 3절의 구동 스택을 torque 모드로 띄운다.
명령원은 **실차 시험 TUI**(`vehicle_test_scenario_runner`)다. TUI에서 Step·Sine·Chirp를
고르고 파라미터를 바꾼 뒤 `R`을 누르면 MCAP이 자동으로 녹화된다. TUI가 시험 종류에
맞춰 조향 노드의 `steering_enabled`를 켜고 끈다. 물리 AUTO, E-stop 해제, 신선한 STM
상태가 모두 필요하다. 현재 YAML의 `minimum_steering_speed_kph: 0.0`(정지 조향 허용)을
확인한다.

방법 A 파라미터(모두 `/steering_actuator_node`, 런타임 변경 가능):

| 설계 기호 | ROS 파라미터 | 타입·단위 | 기본 |
|---|---|---|---|
| Kp | `torque_peak_limit_raw` / `torque_full_scale_error_deg` | int / double deg → raw/deg | 2000 / 0.5 |
| Kd | `torque_kd` | double raw/(rad/s) | 1500.0 |
| J_ff | `torque_inertia_ff` | double raw/(rad/s²) | 0.0 |
| — | `torque_feedforward_limit_raw` | int, \|J·α\| 상한 | 1000 |
| — | `torque_feedforward_filter_hz` | double, α LPF | 5.0 |
| F_c | `torque_friction_raw` | **int** raw | 0 |
| ε | `torque_friction_deadband_deg` | double deg | 0.1 |
| F_c(v) | `torque_friction_schedule` | double 배열 `[v_mps, raw, v_mps, raw, ...]` | 없음 |
| TORQUE_LIMIT | `torque_peak_limit_raw`, `torque_sustained_limit_raw` | int raw | YAML 10000/5000 |
| 부호 | `torque_direction_sign` | int ±1, **startup-only** | 1 |

타입을 정확히 맞춘다. int 파라미터에 `106.0`, double 파라미터에 `12`를 주면
`Wrong parameter type`으로 거부된다(예: `torque_kd 1500.0`, `torque_friction_raw 106`).

주의:
- P 게인은 별도 Kp가 아니라 `Kp[raw/deg] = torque_peak_limit_raw / torque_full_scale_error_deg`다.
  `torque_full_scale_error_deg`를 줄이면 P가 세지고, peak를 바꾸면 같은 오차 설정에서도
  Kp가 변한다. **peak를 고정한 뒤 full-scale-error로 Kp를 튜닝**한다.
- 파라미터를 바꾸는 순간 노드가 zero→Disable하고 다음 명령으로 재진입한다(명령원이
  20 Hz로 계속 발행하므로 자동 복귀). 분석 스크립트는 이 끊김에서 step 이력을 끊는다.
- 여러 값을 한 번에 검증·적용하려면
  `/steering_actuator_node/set_parameters_atomically`를 쓴다. 값/타입/관계가 틀리면
  출력·설정 변경 없이 전체를 거부한다.
- `J·α_ff`의 α는 명령 timestamp에서 미분한다. step 입력은 α가 순간 폭주하므로
  (`torque_feedforward_limit_raw`로 잘림) J_ff 검증은 **Sine/Chirp**로 한다.

#### T3 공통 준비

```zsh
# 터미널 A — 구동 스택 (3절)
ros2 launch control_launch vehicle.launch.py actuation_mode:=torque steering_enabled:=false
```

```zsh
# 터미널 B — bring-up 한계와 순수 PD. sustained를 먼저 내려야 peak가 거부되지 않는다.
N=/steering_actuator_node
ros2 param set $N torque_sustained_limit_raw 1000
ros2 param set $N torque_peak_limit_raw 2000
ros2 param set $N torque_peak_duration_ms 200
ros2 param set $N torque_friction_raw 0
ros2 param set $N torque_inertia_ff 0.0
# 큰 step에서 P가 포화되지 않게 full-scale-error를 넓게 시작(Kp = 2000/5 = 400 raw/deg)
ros2 param set $N torque_full_scale_error_deg 5.0
ros2 param set $N torque_kd 1500.0
```

```zsh
# 터미널 C — 실차 시험 TUI (/planning/command 발행자는 이것 하나)
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

TUI 조작(자세한 내용은
[`vehicle_test_scenarios/README.md`](../software/src/vehicle_test_scenarios/README.md)):

| 키 | 동작 |
|---|---|
| `↑`/`↓`, `←`/`→`, `Enter` | 시험 선택, 파라미터 이동·편집 |
| `M` | MCAP 녹화 ON/OFF (기본 ON) |
| `R` | 설정 고정 → MCAP 시작 → AUTO 대기 → 5초 카운트다운 → 입력 |
| `S` / `Space` | 정지·브레이크 요청 후 기록 종료 |
| `Q` | 안전 정리 후 종료 |

- 조향 시험은 `주행속도`에 도달한 뒤 입력을 시작한다. **무부하(바퀴 띄움)에서는
  `주행속도 0`**으로 둔다(0 km/h는 즉시 정속 판정).
- `반복 횟수`를 1 이상으로 두면 입력이 끝나고 2초 뒤 MCAP이 자동 저장된다. 0은 `S`까지 반복.
- 저장 위치: `~/KAI_ws/records/<시간>_<시험>_v<속도>/` 아래 `rosbag/`, `scenario.yaml`,
  `console.log`. TUI MCAP에는 `/steering/torque_*` 진단과 `/parameter_events`가 포함되고,
  `scenario.yaml`의 `steering_parameters`에 **Run 시점의 조향 게인 전체**가 남는다.
  그래서 폴더 이름에 label을 따로 붙이지 않아도 어떤 게인의 run인지 확인할 수 있다.
- 게인은 **run 사이(TUI 대기 중)**에 터미널 B에서 바꾸는 것을 원칙으로 한다. run 중에
  바꾸면 조향이 잠깐 Disable됐다가 다음 명령으로 재진입하고, 그 구간은 분석에서 끊긴다.
- TUI 명령 발행률은 20 Hz 고정이다. 50 Hz가 필요하면 아래 headless 경로를 쓴다.

#### (1) 순수 PD — 큰 각 step

TUI: **조향 양방향 Step** → `주행속도 0`, `센터각 0`, `조향 진폭 15`(중앙 절댓값 + 진폭 ≤ 28°),
`구간 유지시간 5`, `초기 직진시간 0`, `반복 횟수 2` → `R`.

run 사이 Kp·Kd 조정:

```zsh
ros2 param set $N torque_full_scale_error_deg 3.0   # Kp↑ (2000/3 ≈ 667 raw/deg)
ros2 param set $N torque_kd 2500.0                   # 감쇠↑ → OS↓
```

`/steering/torque_safety`가 `rate_limit`이면 과속 latched FAULT다. ceiling
(`torque_hard_rate_limit_deg_s`)부터 올리지 말고 Kd를 올리거나 peak를 낮춘다.
복구: 원인 제거 → `S`로 run 종료 → `ros2 param set $N steering_enabled false` → `true`
→ 다음 `R`(TUI가 steering_enabled를 다시 맞춘다).

진폭 18°도 반복한다. 목표: OS 7–8%(ζ≈0.64).

#### (2) 마찰 보상 — 작은 각 step

TUI: 같은 Step에서 `조향 진폭 3`(이후 5).

```zsh
ros2 param set $N torque_friction_raw 110         # T1 값 (int)
```

`torque_friction_raw 0`인 run과 번갈아 기록해 정착오차·오버슛을 비교한다.

#### (3) 데드밴드 ε

목표 근처에서 `/steering/torque_friction_raw`가 ±F_c로 번갈아 튀며 떨리면 넓힌다.
정착오차가 ε 근처에 머물면 좁힌다. 바퀴각 해상도는 중앙 부근 약 0.013°/count다.

```zsh
ros2 param set $N torque_friction_deadband_deg 0.2
```

#### (4) 관성 피드포워드 — Sine / Chirp

TUI: **조향 Sine · 고정 주기** → `주행속도 0`, `센터각 0`, `조향 진폭 10`, `파형 주기 2`,
`초기 직진시간 2`, `반복 횟수 5` → `R`. 빠른 입력은 **조향 Chirp**
(`시작 주파수 0.2` → `종료 주파수 1.0`).

```zsh
ros2 param set $N torque_inertia_ff 4.0          # T2 값 (double)
ros2 param set $N torque_feedforward_limit_raw 1000
```

`torque_inertia_ff 0.0`인 run과 번갈아 기록해 위상 지연·진폭 오차를 비교한다.
`/steering/target_accel`이 명령 sine의 −Aω²sin 형태를 따라가는지 먼저 본다.
20 Hz 명령으로 α가 거칠면 `torque_feedforward_filter_hz`를 낮추거나 headless 50 Hz를 쓴다.

#### (대안) headless 명령원 — 발행률 변경·스크립트 실행

TUI 대신 파라미터로 제어한다. 이때는 MCAP을 따로 녹화한다.

```zsh
ros2 run vehicle_test_scenarios vehicle_test_scenario_node --ros-args -p publish_rate_hz:=50.0
```

```zsh
R=/vehicle_test_scenario_runner
ros2 topic info /planning/command      # Publisher count: 1
ros2 param set $R scenario steering_sweep       # steering_step | steering_sweep | steering_chirp
ros2 param set $R cruise_speed_kph 0.0
ros2 param set $R steering_center_deg 0.0
ros2 param set $R steering_amplitude_deg 10.0
ros2 param set $R steering_period_sec 2.0
ros2 param set $R enabled true                   # 종료: enabled false
```

```zsh
# MCAP (label 갱신). 토픽 전체가 필요하면 ros2 launch lateral_ctrl mcap_logging.launch.py
LABEL=T3_ff_sine10_p2
STAMP=$(date +%Y%m%d_%H%M%S_%6N)
mkdir -p ~/KAI_ws/records/${STAMP}_${LABEL} && cd ~/KAI_ws/records/${STAMP}_${LABEL}
ros2 bag record -s mcap -o rosbag /steering/status /drive/status /brake/status \
  /planning/command /vehicle/command /parameter_events \
  /steering/actuation_mode /steering/angle_error /steering/rate /steering/unfiltered_rate \
  /steering/torque_p_raw /steering/torque_d_raw /steering/torque_ff_raw \
  /steering/torque_friction_raw /steering/target_accel /steering/torque_unsaturated_raw \
  /steering/torque_command_raw /steering/torque_limit_raw /steering/torque_saturated \
  /steering/torque_peak_limited /steering/torque_settled /steering/torque_safety
```

#### 분석

```zsh
RUN=~/KAI_ws/records/<시간>_steering_step_v0
grep -A30 steering_parameters $RUN/scenario.yaml    # 이 run의 게인
ros2 run lateral_ctrl torque_sysid_metrics.py step $RUN/rosbag --csv $RUN/step_metrics.csv
# step별 %OS, ζ, 10→90% 상승시간, 정착시간(±0.1°), 정착오차(마지막 25% 평균)
# --min-step-deg(기본 0.5), --settle-band-deg(기본 0.1)로 판정 기준 조정
```

#### 확정값 반영

튜닝이 끝나면 `software/src/lateral_ctrl/config/steering_actuator.yaml`에
`torque_full_scale_error_deg`, `torque_kd`, `torque_friction_raw`,
`torque_friction_deadband_deg`, `torque_inertia_ff`와 확정한 peak/sustained를 적는다.

---

### 실험 종료 시 (매 실험 공통)

```zsh
# T0–T2: 수동 중단 (측정 노드)
ros2 topic pub /deadzone/stop std_msgs/msg/UInt8 "{data: 1}" --once
# T3 (TUI): S/Space로 정지 → 저장 완료 표시 후 Q로 종료
# T3 (headless): ros2 param set /vehicle_test_scenario_runner enabled false, 녹화 터미널 Ctrl+C
# 실험 전부 끝나면 KEYA 0020을 운용값으로 복원 + Burn
#   (position 운용이면 3, torque 운용이면 2 유지 — YAML actuation_mode와 일치시킨다)
```

---

## 6. 접지 재검증 (무부하 확정 후)

무부하 T0–T3 확정 후 접지에서 T3 반복. 접지에선 타이어 마찰로 F_c가 커지고
속도 의존 부하가 나타날 수 있다. 기존 조향 시험 안전규칙 준수: 구동륜 격리,
낮은 값부터, 중앙 절댓값+진폭 합 ≤ 28°.
([`vehicle_test_scenarios/README.md`](../software/src/vehicle_test_scenarios/README.md) 발췌)

속도 스케줄 `F_c(v)`:

1. 주행 조향이면 YAML `minimum_steering_speed_kph`를 1.0으로 복원한다.
2. 속도별(예 0, 1, 2, 3 m/s)로 작은 각 step에서 `torque_friction_raw`를 바꿔 가며
   정착오차가 최소가 되는 F_c를 찾는다(TUI 조향 Step의 `주행속도`로 속도 지정).
3. (속도, F_c) 점을 한 파라미터로 넣는다. 한 번의 set이라 길이가 달라져도 원자적이다.

```zsh
# [v_mps, raw, v_mps, raw, ...] — 속도는 오름차순, 값은 모두 double 표기
ros2 param set $N torque_friction_schedule "[0.0, 180.0, 1.0, 150.0, 3.0, 110.0]"
```

스케줄이 있으면 `torque_friction_raw`보다 우선하며, 표 밖의 속도는 양 끝 값을 유지한다.
속도는 `/drive/status.speed_mps` 절댓값이다. 스케줄을 끄려면 한 점(`[0.0, <F_c>]`)으로
바꾸거나 YAML에서 빼고 재시작한다. Kp·Kd는 일단 고정하고, 속도별 응답을 보고
필요할 때만 스케줄을 추가 검토한다.

label: `T3g_pd_fric_ff_amp<deg>_v<speed>`

---

## 7. 기록·종료 컨벤션 (기존 발췌)

- MCAP 경로: `records/<YYYYMMDD_HHMMSS_us>_<label>/rosbag/`.
  `/steering/status` 필수. 실행 설정·로그는 같은 폴더에 `scenario.yaml`,
  `console.log`로 남긴다(기존 SYSID TUI 컨벤션 계승). T3는 `/parameter_events`를 함께
  기록해 어느 구간에 어떤 게인이었는지 남긴다.
- 종료: Stop/필수 통신 단절/녹화 실패 시 브레이크·Disable. Q/Ctrl+C 시 정지
  요청 후 녹화 저장.
- 일반 Planning 복귀: MANUAL 정차 상태에서 조향 노드 재시작 또는
  `steering_enabled=true` 복원. 시험 노드 종료해도 **마지막 모터 설정은 유지**되니
  `0020`과 YAML `actuation_mode`가 일치하는지 확인한다.
- 전류는 시간평균으로만 참고(순간값 금지, 1 A 양자화).

---

## 8. 파라미터 확정 체크리스트

- [ ] T0: +지령↔+각도 부호 정합 (`torque_direction_sign` 확정, 0026=0 유지)
- [ ] T1: F_c(정지), F_v(운동) — +/− 양방향
- [ ] T2: J (지령단위 실효 J 또는 SI, 필요 시 τ_rated)
- [ ] T3: Kp(full-scale-error), Kd (ζ≈0.64), ε(데드밴드)
- [ ] T3: F_c 추가 시 작은 각 개선, J_ff 추가 시 sine 지연 개선 확인
- [ ] 0028 과부하시간이 데드존 돌파와 충돌 안 함
- [ ] TORQUE_LIMIT(≤ ±8000) ↔ 0003 Max Current ↔ `keya_protection_current_a` 정합
- [ ] 확정값을 `steering_actuator.yaml`에 반영
- [ ] 접지 재검증 통과(필요 시 `torque_friction_schedule`)
- [ ] 실험 종료 후 0020·`actuation_mode`·`minimum_steering_speed_kph` 운용값 복원

---

## 9. 구현 현황

| 항목 | 상태 | 위치 |
|---|---|---|
| T0–T2 측정 노드 | 기존 노드 사용. 5.5절 명령을 실제 동작(limit 즉시 Disable, startup-only 파라미터)에 맞춤 | `lateral_ctrl/src/deadzone_measure_node.cpp` |
| 노드 배선(0020=2 경로, torque0→Enable, 매 tick 송신, 부호) | 완료 | `steering_actuator_node` `actuation_mode=torque` |
| 마찰 보상 `F_c·sign(e)` + 데드밴드 | 완료 | `TorqueSteeringController::update`, `torque_friction_*` |
| 관성 피드포워드 `J·α_ff` (명령 timestamp 미분 + LPF + 상한) | 완료 | `TargetAccelerationEstimator`, `torque_inertia_ff`, `torque_feedforward_*` |
| 속도 스케줄 `F_c(v)` (구간 선형 보간) | 완료 | `TorqueSteeringConfig::frictionRawAt`, `torque_friction_schedule` |
| 진단 토픽 | `/steering/torque_ff_raw`, `/steering/torque_friction_raw`, `/steering/target_accel` 추가 | MCAP launch, Foxglove 조향 Torque 탭 |
| step/sine/chirp 명령원 + MCAP 자동 녹화 | 실차 시험 TUI(`vehicle_test_scenario_runner`). torque 진단 토픽·`/parameter_events` 녹화, Run 시점 조향 게인을 `scenario.yaml`에 기록. headless는 `vehicle_test_scenario_node` | `vehicle_test_scenarios` |
| 후처리 | `torque_sysid_metrics.py step`(%OS/ζ/상승·정착/정착오차), `inertia`(α fit, J·F 회귀) | `lateral_ctrl/scripts/` |
| 오차 크기별 F_c 스케줄 | 미구현(설계상 선택 항목). 속도 스케줄 결과를 보고 판단 | — |
| 실차 응답 검증 | **미수행.** T0부터 이 문서대로 진행 | — |

---

## 관련 문서

- [토크 모드 방법 A 설계](torque_mode_method_A_design.md) — 제어법칙·파라미터 3분류
- [토크 조향 종합 설계](torque_steering_overview.md) — 배경·진단·구현 현황
- [`TORQUE_CONTROL.md`](../software/src/lateral_ctrl/TORQUE_CONTROL.md) — torque 운용 계약·전체 파라미터
- [`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md) — 실차 상시 운용(발췌 출처)
- [`INTERFACES.md`](INTERFACES.md) — KEYA CAN 계약
- [`SAFETY.md`](SAFETY.md) — E-stop·timeout·fail-safe
- [`lateral_ctrl/README.md`](../software/src/lateral_ctrl/README.md) — 조향 패키지·상태 머신
- [`DEADZONE_MEASUREMENT.md`](../software/src/lateral_ctrl/DEADZONE_MEASUREMENT.md) — 데드존 측정 기록
