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

## 0. 빠른 시작 — 무엇을 띄우나

환경은 매 터미널 `cd ~/KAI_ws && humble && source ~/KAI_ws/install/setup.zsh`(2절).
공통 전제: KEYA `0020=2`(Torque)+Burn(4절), 바퀴 띄움, E-stop 준비.
**실험(표의 한 행)씩 띄운다. T0–T2 launch와 T3 launch를 동시에 띄우지 않는다.**

| 실험 | 터미널 1 | 터미널 2 | 끝내기 |
|---|---|---|---|
| T0 부호 | `ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t0 direction:=1` (이후 `direction:=-1`) | — | 측정 로그 확인 후 Ctrl+C |
| T1 마찰 | `ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t1 direction:=1` (이후 `-1`) | — | `SWEEP COMPLETE`/`ANGLE LIMIT ABORT` 후 Ctrl+C |
| T2 관성 | `ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=300` (500, 800, 1200 반복) | — | 로그에 멈춤 표시 후 Ctrl+C |
| T3 PD·마찰·FF | `ros2 launch control_launch torque_test.launch.py` | `ros2 run vehicle_test_scenarios vehicle_test_scenario_runner` | TUI `S` → `Q`, 터미널 1 Ctrl+C |

- T0–T2 launch는 읽기 전용 조향 노드 + 측정 노드(시험별 설정 내장) + MCAP 녹화를 함께 띄운다.
  launch 약 3초 뒤 첫 각도를 받으면 **토크가 자동으로 걸린다.**
- T3 launch는 구동 스택을 torque 모드·조향 비활성·bring-up 한계(peak 2000/sustained 1000)로
  띄운다. P·D는 TUI 메뉴에서 고르고 `R`을 누르면 그 값이 적용되고 MCAP이 녹화된다.
- 기록은 모두 `~/KAI_ws/records/<시간>_<시험>/`에 쌓인다.

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

| 실험 | launch | 안에서 뜨는 것 | 토크 송신원 |
|---|---|---|---|
| T0–T2 | `lateral_ctrl torque_sysid_bench.launch.py` | 읽기 전용 `steering_actuator_node` + `deadzone_measure_node` + MCAP | `deadzone_measure_node` |
| T3, 6절 | `control_launch torque_test.launch.py` | `vehicle.launch.py`(STM Agent·governor·조향 torque armed·상태) | `steering_actuator_node` |

`vehicle.launch.py`(와 `torque_test.launch.py`)는 조향을 armed로 띄우므로 T0–T2 launch와
동시에 실행하면 한 모터에 송신원이 둘이 된다. can0는 부팅 시 자동 준비된다.

`torque_test.launch.py` 인자(기본값):

| 인자 | 기본 | 의미 |
|---|---|---|
| `torque_peak_limit_raw` | 2000 | bring-up peak. 실험 중 8000 이하 |
| `torque_sustained_limit_raw` | 1000 | peak 이하 |
| `can_interface` | can0 | KEYA SocketCAN |

관측(선택, 별도 터미널): Foxglove bridge는 PC당 하나만.

```zsh
ros2 launch foxglove foxglove_bridge.launch.py
# 노트북 Foxglove 앱에서 ws://100.112.72.27:8765, control.json import
# "Torque_Control" 탭: 위 인가 토크(전류 환산)·실측 전류, 아래 조향 요청·응답
```

T3 스택 확인:

```bash
ros2 node list          # stm_a_node, stm_b_node, command_governor, steering_actuator_node, ...
ros2 topic echo --once /steering/actuation_mode   # data: torque
ros2 param get /steering_actuator_node torque_peak_limit_raw   # 2000
```

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

### T0–T2 공통 — `torque_sysid_bench.launch.py`

```zsh
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=<t0|t1|t2> [direction:=1|-1] [level:=300]
```

| 인자 | 기본 | 의미 |
|---|---|---|
| `test` | (필수) | `t0` 부호, `t1` 마찰 sweep, `t2` 관성 step |
| `direction` | 1 | 토크 방향. **1 = LEFT(+)**, -1 = RIGHT |
| `level` | 300 | t2 계단 토크 raw (1..7999) |
| `record` | true | MCAP 녹화 (`/steering/status`, `/deadzone/*`, `/rosout`) |
| `records_dir` | `~/KAI_ws/records` | 기록 위치. `<시간>_<label>_pos` 또는 `_neg` 아래 `rosbag/` |
| `start_delay_sec` | 3.0 | 조향 노드·녹화가 뜬 뒤 측정 노드를 띄우는 지연 |
| `can_interface` | can0 | |

launch 안의 시험별 설정(`deadzone_measure_node`):

| test | start_cmd | ramp_step | torque_limit | step_hold_ms | sweep | max_angle_delta(모터축°) |
|---|---|---|---|---|---|---|
| t0 | 300 | 300 | 600 | 1500 | off | 20 |
| t1 | 60 | 10 | 400 | 300 | on | 90 |
| t2 | level | 1 | level+1 | 300 | off | 90 |

- 측정 노드는 첫 유효 각도를 받는 즉시 램프를 시작한다(`auto_start`). 각도 한계 기준은
  그 시점 각도다. 각 run 전 조향을 중앙(±10°)에 두고 launch한다.
- 노드는 `|cmd|`가 `torque_limit`에 닿는 순간 Disable한다. 그래서 유지할 토크보다
  `torque_limit`을 한 단계 크게 잡았다.
- 측정이 끝나도 launch는 살아 있다(토크 0 keep-alive, Disable). 로그를 확인하고 Ctrl+C로
  끝내면 MCAP이 저장된다. 같은 run을 다시 하려면 다시 launch한다.
- 설정을 바꿔 돌리고 싶으면(예: T1 긴 hold) launch 중에 런타임 파라미터를 바꾸고 재시작 토픽을 쏜다.
  `ros2 param set /deadzone_measure_node step_hold_ms 1000` →
  `ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once`
  (`start_cmd`, `sweep_mode`는 시작 시에만 읽으므로 launch 인자 대신 노드를 직접 실행해야 한다.)

### T0 — 방향 부호 검증

```zsh
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t0 direction:=1
# 확인 후 Ctrl+C, 중앙 복귀
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t0 direction:=-1
```

다른 터미널에서 실제 바퀴각을 본다.

```zsh
ros2 topic echo /steering/status --field actual_wheel_angle_rad
```

판정: `direction:=1`에서 바퀴가 물리적으로 **왼쪽**으로 돌고 `actual_wheel_angle_rad`가
증가하면 정상(`torque_direction_sign: 1` 유지). 반대면
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

### T1 — 마찰(정지/운동) 측정

```zsh
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t1 direction:=1
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t1 direction:=-1
```

로그의 `SWEEP,dir=..,cmd=..,rated_pct=..,current=..,angle=..,delta=..` 줄을 본다(MCAP의
`/rosout`에도 남는다).
- `delta`가 처음 0이 아닌 `cmd` = **F_c**
- 이후 `delta/step_hold`가 거의 일정한 구간의 `cmd` = F_v(무부하에서는 거칠다)
- `ANGLE LIMIT ABORT`는 정상 종료다.

F_c는 관측 시간에 종속적이다(300 ms→110, 8 s→106). 방법 A의 `torque_friction_raw`에는
이 launch의 **300 ms hold** 값을 우선 쓴다.

### T2 — 등가 관성 J 측정

```zsh
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=300
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=500
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=800
# 필요 시 1200. 높을수록 각도 한계까지 샘플이 적다(fit에 최소 4샘플).
```

분석(레벨별 bag을 `bag:토크` 쌍으로):

```zsh
R=~/KAI_ws/records
ros2 run lateral_ctrl torque_sysid_metrics.py inertia \
  $R/<시간>_T2_inertia_step_lvl300_pos/rosbag:300 \
  $R/<시간>_T2_inertia_step_lvl500_pos/rosbag:500 \
  $R/<시간>_T2_inertia_step_lvl800_pos/rosbag:800 \
  --friction-raw 110 --csv $R/T2_inertia.csv
# run별 α, F_c 고정 J, 레벨 2개 이상이면 τ = J·α + F 회귀의 J와 절편 F
```

---

### T3 — 방법 A 제어 (`steering_actuator_node`, torque 모드)

측정 launch를 **종료**하고 두 개만 띄운다.

```zsh
# 터미널 1 — 구동 스택 (torque, 조향 비활성, peak 2000/sustained 1000)
ros2 launch control_launch torque_test.launch.py

# 터미널 2 — 실차 시험 TUI
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

TUI가 `R` 때 조향 노드에 `steering_enabled`와 **메뉴의 P·D**를 한 번에 적용하고 MCAP을
녹화한다. 물리 AUTO, E-stop 해제, 신선한 STM 상태가 모두 필요하다. 현재 YAML의
`minimum_steering_speed_kph: 0.0`(정지 조향 허용)을 확인한다.

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

#### TUI 사용법

| 키 | 동작 |
|---|---|
| `↑`/`↓`, `←`/`→`, `Enter` | 시험 선택, 파라미터 이동·편집 |
| `M` | MCAP 녹화 ON/OFF (기본 ON) |
| `R` | 설정 고정 → P·D 적용 → MCAP 시작 → AUTO 대기 → 5초 카운트다운 → 입력 |
| `S` / `Space` | 정지·브레이크 요청 후 기록 종료 |
| `Q` | 안전 정리 후 종료 |

조향 시험(양방향 Step / Sine / Chirp) 메뉴 파라미터:

| 메뉴 | 의미 | 무부하 권장 |
|---|---|---|
| 주행속도 | 이 속도에 도달한 뒤 입력 시작 | **0** |
| 센터각, 조향 진폭 | 중앙 절댓값 + 진폭 ≤ 28° | 0, 시험별 |
| 구간 유지시간 / 파형 주기 / Chirp 주파수 | 입력 모양 | 시험별 |
| 반복 횟수 | 1 이상이면 끝나고 2초 뒤 자동 저장, 0은 `S`까지 | 1–3 |
| **P · full-scale 오차** | `torque_full_scale_error_deg`. 작을수록 P↑, `Kp[raw/deg] = peak / 값` | 시작 5.0 |
| **D 게인** | `torque_kd` raw/(rad/s). 0은 감쇠 없음 | 시작 1500 |

- P·D의 기준은 **메뉴 값**이다. `R`마다 조향 노드에 다시 적용되므로 `ros2 param set`으로
  바꾼 P·D는 다음 Run에서 메뉴 값으로 덮인다. 노드가 거부하면(예: P가 settle 각 0.05° 이하)
  Run이 시작되지 않고 화면에 이유가 뜬다.
- position 모드 조향 노드면 P·D는 적용하지 않고 로그에 표시만 한다.
- 메뉴에 없는 값(마찰 `torque_friction_raw`, 데드밴드, `torque_inertia_ff`, 토크 한계)은
  run 사이에 `ros2 param set /steering_actuator_node ...`로 바꾼다.
- 저장: `~/KAI_ws/records/<시간>_<시험>_v<속도>/` 아래 `rosbag/`, `scenario.yaml`,
  `console.log`. `scenario.yaml`의 `parameters`에 메뉴 값, `steering_parameters`에
  run 종료 시점의 조향 게인 전체(마찰·FF·한계 포함)가 남는다.
- TUI 명령 발행률은 20 Hz 고정이다. 50 Hz가 필요하면 아래 headless 경로를 쓴다.

#### (1) 순수 PD — 큰 각 step

TUI: **조향 양방향 Step** → `주행속도 0`, `센터각 0`, `조향 진폭 15`, `구간 유지시간 5`,
`반복 횟수 2`, `P 5.0`, `D 1500` → `R`.

run마다 P·D만 메뉴에서 바꿔 반복한다(예: P 5.0 → 3.0 → 2.0, OS가 크면 D↑).
목표: 진폭 15°·18°에서 OS 7–8%(ζ≈0.64).

`/steering/torque_safety`가 `rate_limit`이면 과속 latched FAULT다. ceiling
(`torque_hard_rate_limit_deg_s`)부터 올리지 말고 D를 올리거나 P를 낮춘다.
복구: `S`로 run 종료 → `ros2 param set /steering_actuator_node steering_enabled false`
→ `true` → 다음 `R`.

#### (2) 마찰 보상 — 작은 각 step

TUI: 같은 Step에서 `조향 진폭 3`(이후 5), P·D는 (1)의 확정값.

```zsh
ros2 param set /steering_actuator_node torque_friction_raw 110   # T1 값 (int)
```

`torque_friction_raw 0`인 run과 번갈아 기록해 정착오차·오버슛을 비교한다.

#### (3) 데드밴드 ε

목표 근처에서 `/steering/torque_friction_raw`가 ±F_c로 번갈아 튀며 떨리면 넓힌다.
정착오차가 ε 근처에 머물면 좁힌다. 바퀴각 해상도는 중앙 부근 약 0.013°/count다.

```zsh
ros2 param set /steering_actuator_node torque_friction_deadband_deg 0.2
```

#### (4) 관성 피드포워드 — Sine / Chirp

TUI: **조향 Sine · 고정 주기** → `주행속도 0`, `센터각 0`, `조향 진폭 10`, `파형 주기 2`,
`초기 직진시간 2`, `반복 횟수 5`, P·D 확정값 → `R`. 빠른 입력은 **조향 Chirp**
(`시작 주파수 0.2` → `종료 주파수 1.0`).

```zsh
ros2 param set /steering_actuator_node torque_inertia_ff 4.0          # T2 값 (double)
ros2 param set /steering_actuator_node torque_feedforward_limit_raw 1000
```

`torque_inertia_ff 0.0`인 run과 번갈아 기록해 위상 지연·진폭 오차를 비교한다.
`/steering/target_accel`이 명령 sine의 −Aω²sin 형태를 따라가는지 먼저 본다.
20 Hz 명령으로 α가 거칠면 `torque_feedforward_filter_hz`를 낮추거나 headless 50 Hz를 쓴다.

#### (대안) headless 명령원 — 발행률 변경·스크립트 실행

TUI 대신 파라미터로 제어한다. 이때는 P·D를 `ros2 param set`으로 직접 주고 MCAP을 따로 녹화한다.

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
# T0–T2: 즉시 멈추기 ros2 topic pub /deadzone/stop std_msgs/msg/UInt8 "{data: 1}" --once
#        그 뒤 launch 터미널 Ctrl+C (MCAP 저장)
# T3 (TUI): S/Space로 정지 → 저장 완료 표시 후 Q로 종료 → 터미널 1 Ctrl+C
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
ros2 param set /steering_actuator_node torque_friction_schedule "[0.0, 180.0, 1.0, 150.0, 3.0, 110.0]"
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
| T0–T2 실행 | `torque_sysid_bench.launch.py test:=t0` / `t1` / `t2` 한 줄(읽기 전용 조향 + 측정 노드 프리셋 + MCAP). 측정 노드 `auto_start`는 첫 유효 각도 수신 후 시작 | `lateral_ctrl/launch/`, `deadzone_measure_node.cpp` |
| T3 스택 | `torque_test.launch.py`(torque·조향 비활성·bring-up 한계). 한계는 `vehicle.launch.py`/`steering_actuator.launch.py` 인자로 전달 | `control_launch/launch/` |
| 노드 배선(0020=2 경로, torque0→Enable, 매 tick 송신, 부호) | 완료 | `steering_actuator_node` `actuation_mode=torque` |
| 마찰 보상 `F_c·sign(e)` + 데드밴드 | 완료 | `TorqueSteeringController::update`, `torque_friction_*` |
| 관성 피드포워드 `J·α_ff` (명령 timestamp 미분 + LPF + 상한) | 완료 | `TargetAccelerationEstimator`, `torque_inertia_ff`, `torque_feedforward_*` |
| 속도 스케줄 `F_c(v)` (구간 선형 보간) | 완료 | `TorqueSteeringConfig::frictionRawAt`, `torque_friction_schedule` |
| 진단 토픽 | `/steering/torque_ff_raw`, `/steering/torque_friction_raw`, `/steering/target_accel` 추가 | MCAP launch, Foxglove 조향 Torque 탭 |
| step/sine/chirp 명령원 + MCAP 자동 녹화 | 실차 시험 TUI(`vehicle_test_scenario_runner`). 메뉴 P·D를 Run 때 `steering_enabled`와 원자적으로 적용. torque 진단 토픽·`/parameter_events` 녹화, Run 시점 조향 게인을 `scenario.yaml`에 기록(run 종료 시점). headless는 `vehicle_test_scenario_node` | `vehicle_test_scenarios` |
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
