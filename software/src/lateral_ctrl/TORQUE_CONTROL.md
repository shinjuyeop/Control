# KEYA 외부 wheel-angle torque 제어

`steering_actuator_node`의 startup-only `actuation_mode`로 `position`(기본) 또는
`torque`를 선택한다. CAN, AUTO/MANUAL, 출력 arm, timeout, E-stop, brake와 상태
발행은 같은 노드가 소유한다. `kaiev26_msgs` ABI는 바꾸지 않는다.

Mode의 **코드 기본값은 position**이다. 현재 작업 YAML의 운영자 선택은 `torque`이며
이를 보존했다. Launch는 `params_file` 위에 `control_tuning_file`의 사용자 설정을
적용하고, 명시적 `actuation_mode:=...`가 가장 우선한다. 기존 HIL launch는 position을 명시한다.

## 공통 파라미터 TUI

`ros2 run control_tuning control_tuning_tui`에서 응답 튜닝·출력/보호·운용·timeout을
편집한다. P는 `A/deg`, D는 `A/(deg/s)`이며 현재 기본값은 각각 32와 약 0.0418879다.
20 A 환산 기준에서 최대 명령은 16 A, 지속 명령은 8 A다. 실측 전류나 정격 설정을 뜻하지 않는다.
최대 명령을 바꿔도 화면의 P를 유지하도록 `torque_full_scale_error_deg`를 함께 환산한다.

`A`는 MANUAL·정지 상태에서 16개 torque 튜닝을 atomic 적용하고 재조회한다.
`S`는 다음 launch용 사용자 파일에 저장한다. 운용 방식·최소 차속·timeout은 저장 후
노드 재시작이 필요하다. 기존 ROS 키는 유지하므로 아래 표와 CLI는 계속 유효하다.
[TUI 사용법·단위·저장 경로](../control_tuning/README.md)를 참고한다.

튜닝 TUI의 `P`는 공중·접지 정지·주행별 사용자 제어 프리셋을 관리한다. 검증되지 않은
조건별 권장값은 제공하지 않는다. 시나리오 TUI에서 실제 시험 조건을 선택하면 녹화 시
실행 노드의 실제 파라미터와 이후 변경 이벤트·torque 상세 신호를 함께 기록한다.

| Host | KEYA Windows utility | 제어 경로 |
|---|---|---|
| `position` | 0019=2 CAN, **0020=3** Absolute position | 기존 60 deg/s 목표 shaping → 선택적 tracking 보정 → count |
| `torque` | 0019=2 CAN, **0020=2** Torque | 직접 wheel-angle step → 외부 PD → envelope → raw torque |

Host는 0020을 읽거나 변경하지 않는다. 시작 경고를 확인하고 utility에서 설정·Burn을
완료한다. 현재 운용 설정은 Direction=0, Max Current=20 A다. mode 변경에는 노드
재시작과 KEYA 설정 확인이 모두 필요하다. 신규 production node는 없다.

## 제어식과 축

`/vehicle/command.steering_target_rad`는 heading이 아닌 **LEFT-positive 중앙 등가
road-wheel 조향각**이다. actual은 현재 calibration의 encoder count 변환값이다.

```text
target = clamp(requested, calibrated_min + margin, calibrated_max - margin)
e = target - actual
Kp = torque_peak_limit_raw / deg2rad(torque_full_scale_error_deg)
P = Kp * e
D = torque_kd * (0 - measured_wheel_rate_rad_s)
u = P + D
canonical_torque = envelope_and_direction_limit(u)
KEYA_torque = torque_direction_sign * canonical_torque
```

Ki=0이다. direct step의 target rate는 0으로 두어 D kick을 만들지 않는다.
목표에서 오차가 0이어도 움직이는 방향의 반대 torque로 제동한다. Position의
`max_steering_rate_deg_s`는 torque 경로에 적용하지 않는다.

기본 full-scale-error=0.5°에서 정지 상태의 P는 0.1°=peak의 20%,
0.25°=50%, 0.5°=100%다. 큰 오차의 saturation은 정상 동작이다. D·envelope·한계
처리 뒤 실제 송신값은 달라질 수 있다. peak=10000까지 검증 후 설정 가능하다.

부호 변환은 `keyaTorqueCommand()`에서 단 한 번 적용한다. 기본 `+1`은
`DEADZONE_MEASUREMENT.md`의 양의 raw torque→물리 LEFT 관측을 따른 초기 가정이다.
그 문서의 옛 `/ctrl/steer_handle_hires`는 반전된 진단 축이며, 현재 calibration은
positive count→LEFT다. 옛 음의 angle 기록만 보고 `-1`로 바꾸지 않는다. 장착 상태와
Direction 설정이 달라질 수 있으므로 첫 저출력 시험에서 **물리 LEFT와 현재 count/
actual 증가를 모두 확인**한다. 불일치하면 중단하고 장착·calibration을 확인한다.
방향 parameter는 startup-only다.

## 피드백과 주기

제조사 매뉴얼 §4.3.2의 10000 count/rev encoder query를 매 control tick 보낸다.
heartbeat의 360 count/rev 위치는 제어에 사용하지 않는다. driver는 SocketCAN
수신 timestamp를 사용해 큐에 머문 프레임을 새 피드백으로 취급하지 않는다.
각속도는 **서로 다른 fresh 위치 샘플의 calibrated angle 차분 / 실제 수신 간격**이다.
같은 샘플을 재사용해 rate를 0으로 떨어뜨리지 않는다. 필터는
`alpha=1-exp(-2*pi*rate_filter_hz*dt)`, `rate += alpha*(raw_rate-rate)`다.
hard overspeed는 filtered와 unfiltered rate 양쪽을 검사한다.

기본 20 ms/50 Hz. 기존 실측(2026-08-03)은 heartbeat 평균 20.100 ms,
위치 query 왕복 평균 1.393 ms다. 현재 polling은 이전 tick의 응답을 다음 tick에
읽으므로 약 한 tick의 처리 지연이 있다. speed/current query는 10 tick,
fault는 5 tick, voltage/temperature는 25 tick 주기이며 heartbeat도 speed/current를
갱신한다. `torque_control_period_ms=10`은 향후 100 Hz 시험용이다. 설정 변경만으로
100 Hz 실측 성능을 보장하지 않는다. 새 feedback 빈도·age·jitter와 CAN 부하를 먼저
측정한다. tick 지연이 torque feedback timeout보다 크면 fresh feedback이 있어도
정지한다. Host가 멈춘 동안에는 다음 tick까지 정지 프레임을 보낼 수 없다.

## 기본 파라미터

아래 표는 **노드 코드 기본값**이다. launch가 읽는
`config/steering_actuator.yaml`은 다음 값을 덮어쓴다.

| 항목 | 현재 YAML |
|---|---:|
| `actuation_mode` | `torque` |
| `torque_peak_limit_raw` | 10000 |
| `torque_sustained_limit_raw` | 5000 |
| `torque_peak_duration_ms` | 5000 |
| `torque_peak_recovery_ms` | 5000 |
| `minimum_steering_speed_kph` | 0.0 (정지 시험용; 코드 기본값 1.0) |

현재 실차 설정을 확인할 때는 YAML과 `ros2 param get`을 기준으로 한다.

| 이름 | 기본값 | 단위/설명 |
|---|---:|---|
| `actuation_mode` | position | startup-only |
| `torque_direction_sign` | +1 | startup-only, ±1 |
| `keya_protection_current_a` | 20.0 | startup-only, 표시 변환용; KEYA Parameter 0003과 일치 |
| `torque_control_period_ms` | 20 | startup-only, 10..50 ms |
| `torque_feedback_timeout_ms` | 100 | startup-only, 2 tick 이상·200 ms 이하 |
| `torque_full_scale_error_deg` | 0.5 | P만으로 peak 도달하는 오차 |
| `torque_kd` | 1500.0 | raw/(rad/s); 검증 전 초기 damping |
| `torque_peak_limit_raw` | 2000 | 1..10000 |
| `torque_sustained_limit_raw` | 1000 | 1..peak |
| `torque_peak_duration_ms` | 200 | 1 tick 이상·5000 ms 이하 |
| `torque_peak_recovery_ms` | 2000 | peak 재허용 저부하 대기, duration 이상 |
| `torque_settle_angle_deg` | 0.05 | full-scale-error 미만; 0.05..0.15°부터 튜닝 |
| `torque_settle_rate_deg_s` | 0.5 | raw·filtered rate 모두 이 값 미만일 때 settle |
| `torque_rate_filter_hz` | 10.0 | 양수, control Nyquist 이하 |
| `torque_soft_limit_margin_deg` | 1.0 | calibration 양쪽 끝에서 안쪽 margin |
| `torque_hard_rate_limit_deg_s` | 120.0 | 실제 wheel rate 안전 ceiling |

startup-only 이외의 위 gain/envelope 값은 runtime 변경 가능하다. 전체 변경 묶음을
검증한 후 **zero→Disable, 목표·rate 이력 폐기**하고 적용한다. 새 명령과 fresh rate
이력으로만 재진입한다. peak 사용 이력은 보존한다. 한 묶음의 관계 검증이 필요하면
`/steering_actuator_node/set_parameters_atomically`를 사용한다. 값/타입/상호 제약이
잘못되면 출력·설정 변경 없이 reject한다. 기존의 한 번만 읽는 parameter도 runtime
변경을 거부한다. `steering_enabled`와 ROS `use_sim_time`은 기존 용도를 유지한다.

## Peak와 지속 출력

peak보다 작은 sustained 한계는 **전류 A 추정치가 아니다**. 매뉴얼의 continuous
10 A/peak 20 A, §4.3.3.2의 설정 전류×80% 설명 및 5초 정지 예제만으로 현재
Max Current=20 A의 raw→A 전달함수를 확정할 수 없다. 과거 데드존의 약 raw106
측정은 무부하·긴 관측 창 결과이며 전류가 정수 A라 0 A로 보였던 데이터다.
기존 측정 노드의 raw8000 cap 역시 보장된 전류 보호 한계가 아니다.

코드 기본 peak2000/sustained1000/200 ms는 bring-up용 초기 envelope이며 성능 최종값이나
열적 안전 인증값이 아니다. 최초 sign 시험은 아래 raw200으로 더 낮춘다. 현재 운용
YAML은 peak10000/sustained5000/5000 ms와 저출력 회복5000 ms를 선택한다. 이는
KEYA의 연속10 A/peak20 A 사양에서 5초 사용이 안전하다는 인증이 아니므로, 실제
전류·온도·부하를 기록해 유지 여부를 판단한다.

직전 실제 요청 torque가 sustained를 넘었던 **누적 시간**을 계산한다. 다음 한
tick을 예약할 예산이 없으면 sustained로 제한한다. 방향 반전·목표 변경·controller
reset·짧은 disable·runtime tuning은 예산을 초기화하지 않는다. 실제 요청 크기가
sustained의 50% 이하로 recovery 기간 동안 연속 유지된 뒤만 다시 peak를 허용한다.
지속 saturation은 계속 sustained에 머문다. 이는 raw/time envelope이며 열 모델이나
Host current loop가 아니다. 실제 peak 지속 오차는 control tick jitter에 좌우된다.
사용 이력은 프로세스 메모리에 있으므로 노드 재시작 후에는 보존되지 않는다.
재시작을 열 회복의 증거로 삼지 않는다.

## 안전 동작

- ACTIVE 진입: reset→torque0→Enable. 새 위치 샘플로 rate가 준비될 때까지 출력0.
- 종료: torque0→Disable→reset. zero write가 실패해도 Disable을 시도하고 다음
  tick에서 실패한 stop을 재시도한다. CAN 단절 시 실제 수신/차단을 보장할 수 없다.
- MANUAL, E-stop, command brake, `steering_enabled=false`, 낮거나 무효한 drive
  speed, stale drive status, command/CAN/position timeout, invalid feedback, motor
  fault는 기존 state machine을 사용한다. 출력이 잠기면 zero/disable도 보내지 않는다.
- Torque의 command는 원래 header stamp의 남은 유효시간만 사용한다. future stamp,
  nonfinite target은 reject한다. 예전 목표를 재사용하지 않는다.
- Calibration 범위 `[-3279,+3279] count` = 약 `[-27.8275,+25.9178]°`를 hard
  envelope로 사용한다. 이는 **계측 범위**이며 확인된 기계 end-stop이 아니다.
  목표는 기본 margin으로 `[-26.8275,+24.9178]°` 안쪽에 제한한다.
  margin 내에서 바깥쪽 torque는 선형 taper, 경계에서는 차단하고 안쪽 회복 torque는
  허용한다. hard 범위를 실제로 넘으면 즉시 zero+Disable한다.
- MANUAL에서는 조향 모터를 끄고 운전자 조작을 AUTO torque 제어기의 각도·각속도
  fault로 잠그지 않는다. AUTO를 떠날 때 제어 이력을 초기화하므로 수동 조작의
  각속도를 다음 AUTO 명령에 물려주지 않는다. AUTO 복귀 시에는 Enable 전에 현재
  위치가 위 hard envelope 안인지 다시 검사한다. AUTO에서 이미 발생한 fault는
  MANUAL 전환만으로 해제되지 않는다.
- AUTO의 각도 범위 초과, 실제 과속, invalid controller input, control stall/과도한 위치 샘플 간격은 latched
  `FAULT`다. 메시지 ABI의 `REASON_MOTOR_FAULT`를 재사용하고 세부 원인은
  `/steering/torque_safety`에 표시한다. 잠금은 조향 노드의 프로세스 메모리에만 보관한다.
  원인 제거 후 기존 `control_launch vehicle.launch.py`를 완전히 종료하고 재실행하면
  조향 노드가 새로 생성되면서 잠금이 초기화된다. 재실행 없이 해제하려면
  `steering_enabled=false`, `true` 순서로 설정한다. 두 경우 모두 유효한 상태·피드백과
  새로운 command가 있어야 다시 Enable한다. 단순 saturation은 fault가 아니다.
  재실행 후에도 실제 과속이나 KEYA fault가 있으면 다시 정지한다.
  대시보드의 `STEER MOTOR FAULT`는 이 소프트웨어 안전 정지도 포함하므로 문구만으로
  KEYA 스톨을 판단하지 않는다. `fault_bits & ~1`과 `/steering/torque_safety`를 함께 본다.
  [2026-09-18 MCAP 분석](../../../docs/STEERING_FAULT_20260918.md)에 MANUAL 범위 초과가
  다음 시험까지 남았던 원인과 수정 내용을 정리했다.
- `/brake/status` freshness에 별도 timeout이 없는 기존 한계와 물리 E-stop 경로는
  `docs/SAFETY.md`를 따른다. KEYA의 speed/torque watchdog은 최대 1000 ms이며
  Host의 100 ms 위치 감시와 별개다. 독립적인 하드웨어 차단 수단이 필요하다.

## 최초 실차 절차

1. 바퀴를 띄우고 기계 중앙·calibration·E-stop을 확인한다. CAN 250 kbit/s,
   KEYA 0020=2, Direction=0을 확인한다. KEYA를 쓰는 기존 노드/deadzone 노드를
   종료한다. 실제 STM 상태·물리 E-stop을 사용하고 drive/brake status를 가짜로
   발행하지 않는다. standalone으로 실행할 때 기존 통합 steering node는 꺼 둔다.
2. workspace 환경에서 출력을 제어하기 전에 조향을 비활성 상태로 시작한다.

   ```bash
   cd ~/KAI_ws
   si
   ros2 launch lateral_ctrl steering_actuator.launch.py \
     actuation_mode:=torque steering_enabled:=false can_interface:=can0
   ```

3. 다른 터미널에서 낮은 raw 한계를 설정한다. 현재 YAML의
   `minimum_steering_speed_kph=0.0`은 정지 시험용이며 fresh·valid `/drive/status`는
   여전히 필요하다. 읽기 전용 관측만 하려면 위 launch에
   `actuator_output_enabled:=false`를 추가한다.

   ```bash
   ros2 param set /steering_actuator_node torque_sustained_limit_raw 200
   ros2 param set /steering_actuator_node torque_peak_limit_raw 200
   ros2 param set /steering_actuator_node torque_kd 150.0
   ros2 topic echo /steering/status
   # 관측 확인 후 별도 터미널:
   ros2 param set /steering_actuator_node steering_enabled true
   ```

4. `/vehicle/command`의 유일 발행자는 governor로 유지한다. planning 또는 기존
   시험 명령원을 하나만 사용한다. 조향 step을 따로 보내려면 기존 planning 시험
   명령원을 종료하고 아래 명령을 사용한다(속도 목표0).

   ```bash
   ros2 topic pub -r 50 /planning/command kaiev26_msgs/msg/ActuatorCommand \
     '{header: auto, steering_target_rad: 0.0, speed_target_mps: 0.0, brake_engage: false}'
   # 위 publisher를 Ctrl+C로 종료하고 다음 목표를 차례로 실행한다.
   # +1°: steering_target_rad: 0.0174532925
   #  0°: steering_target_rad: 0.0
   # -1°: steering_target_rad: -0.0174532925
   ```

   physical AUTO·E-stop 해제·fresh STM 상태가 모두 필요하다. `+1°`에서 실제 LEFT,
   `actual_wheel_angle_rad` 및 count 증가를 확인한다. 반대면 즉시 E-stop/조향 disable.
   sign 변경은 `params_file`의 `torque_direction_sign` 수정 후 재시작한다.
5. `0→+1°→0→-1°`를 기록한다. peak/full-scale-error로 P를 올리고, overshoot와
   zero crossing 때 반대 torque를 보며 Kd를 증가시킨다. limit을 바꾸면 같은
   full-scale-error라도 Kp가 변하므로 Kd도 재평가한다. gain 변경마다 노드가 잠깐
   Disable하며 새 명령으로 재진입한다. 0.5° full-scale-error는 유지 가능한 목표다.
6. ±5° step에서 rise time·overshoot·settle time·rate·current를 확인한다. 그 다음
   지면 부하에서 작은 step부터 다시 검증하고 peak/sustained/duration을 확대한다.
   과속 fault를 없애려고 ceiling부터 올리지 말고 damping·피드백 지연을 먼저 본다.
7. 실제 fault 후에는 원인 해결→기존 vehicle launch 종료·재실행→새 명령 순서로
   재개한다. 노드를 유지할 때는 `steering_enabled false`→`true`로 잠금을 해제한다.
   position 복귀는 노드 종료·KEYA 0020=3/Burn 확인 후 실행한다.

## T0-T2 측정 도구

`torque_sysid_bench.launch.py`는 T0 부호, T1 마찰, T2 관성 시험 중 하나만 실행한다.
읽기 전용 조향 노드와 실제 torque를 보내는 측정 노드를 함께 띄우므로
`vehicle.launch.py`나 다른 KEYA 송신원과 동시에 실행하면 안 된다. 바퀴를 띄우고
차량 정지·MANUAL/중립·가속 페달 해제·E-stop 준비, 대상 CAN과 KEYA 0020=2를 확인한 뒤
T0부터 진행한다. 이 문서의 코드·테스트 결과는 물리 부호나 안전 한계의 증명이 아니다.

```bash
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t0 direction:=1
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t1 direction:=1
ros2 launch lateral_ctrl torque_sysid_bench.launch.py test:=t2 level:=300
```

기록은 기본적으로 외장 저장소
`~/Storage/ROS2_Workspace_offload/Control/torque_sysid`에 생성된다. 외장 저장소가 별도
mount가 아니면 launch가 기록 전에 중단한다. T2를 두 개 이상의 raw level로 기록한 뒤:

```bash
ros2 run lateral_ctrl torque_sysid_metrics.py inertia \
  <level300-bag>:300 <level500-bag>:500 --friction-raw <T1-result>
ros2 run lateral_ctrl torque_sysid_metrics.py step <step-bag> --csv <output.csv>
```

측정 도구는 조향 기구의 마찰·관성 특성을 분석하는 용도이며 PD 제어 설정을 변경하지 않는다.

## Foxglove와 기록

`foxglove/control.json`의 **조향 Torque** 탭에서 target/actual/error, P/D/raw,
filtered/unfiltered rate, current/speed, mode/safety를 함께 확인한다.
`mcap_logging.launch.py`도 아래 토픽을 기록한다.

| 토픽 | 타입 | 단위/의미 |
|---|---|---|
| `/steering/actuation_mode` | String, transient local | position/torque |
| `/steering/requested_angle` | Float32 | 받아들인 요청 rad, calibration command 범위 clamp 후 |
| `/steering/status.corrected_wheel_angle_rad` | 기존 status | soft clamp 후 실제 PD 목표 rad |
| `/steering/status.actual_wheel_angle_rad` | 기존 status | calibrated actual rad |
| `/steering/angle_error` | Float32 | PD 오차 rad |
| `/steering/rate`, `/steering/unfiltered_rate` | Float32 | rad/s |
| `/steering/torque_p_raw`, `/steering/torque_d_raw` | Float32 | canonical P/D raw 기여 |
| `/steering/torque_unsaturated_raw` | Float32 | canonical P+D |
| `/steering/torque_command_raw` | Int32 | sign 적용 후 KEYA에 요청한 raw; 실제 torque 측정 아님 |
| `/steering/torque_current_limit_command_a` | Float32 | `raw/10000×Parameter 0003×0.8`; signed 명령 전류 제한값 A |
| `/steering/torque_limit_raw` | Int32 | 현재 peak/sustained cap |
| `/steering/torque_saturated`, `/steering/torque_peak_limited` | Bool | clipping·peak 예산 상태 |
| `/steering/torque_settled`, `/steering/torque_rate_ready` | Bool | settle·rate 준비 |
| `/steering/torque_safety` | String | none/angle_limit/rate_limit/invalid_input/feedback_timeout |
| `/steering/status.motor_current_a`, `.motor_speed_rpm` | 기존 status | A/rpm, 진단용 |

명령 전류 제한값은 실제 입력/모터 전류나 torque 측정값이 아니다. Motor current는
KEYA가 보고한 정수 A 해상도 진단값이며 Host 전류 제어의 feedback으로 사용하지 않는다.
torque 진단 토픽은 torque mode에서만 발행한다. 정지 시 제어 기여/출력 진단은0으로
reset되며 actual/current/speed는 기존 status에서 계속 관측한다.

근거: 저장소 `docs/hardware/KEYA_KY170DJD01005-08G-V1_User_Manual_V1.1.pdf`
§II, §4.1.2, §4.3.2, §4.3.3.2, `DEADZONE_MEASUREMENT.md`, 현재 calibration r3.

## 구현 검증 (2026-09-16)

- 기준 commit `cb708f8`, `origin/Shin` fetch 후 일치 확인. 메시지 ABI 변경 없음.
- ROS 2 Humble에서 `kaiev26_msgs`, `lateral_ctrl`, `control_launch`, `foxglove` 실제 colcon build 성공.
  메시지 패키지의 기존 CMake CMP0148 개발 경고 외 조향 빌드 경고 없음.
- GTest 76개 통과(CTest 6개 포함 colcon 집계82, 실패0). CAN ±payload,
  socket kernel timestamp, P/D·settle·sign·hard limit·rate·peak budget,
  node ACTIVE entry/exit/shutdown·timeout·fault·TX 실패·readonly·runtime parameter
  원자성·position regression을 포함한다. 노드 테스트는 실제 노드와 기록용 CAN
  driver를 연결하며 물리 CAN 출력을 사용하지 않는다.
- 두 mode 모두 CAN을 닫고 ROS launch/status 발행, parameter service의 runtime
  mode 변경 reject를 확인했다. CLI 생략→현재 YAML torque, 명시적 position override,
  torque+steering disabled 조합도 실제 parameter service로 확인했다.
  Python launch/YAML/Foxglove 참조 검사 통과.
- HIL 테스트69개 통과(기존68+mode 회귀1), native dashboard renderer가 없는4개 skip.
  Web/bridge-worker 통합 테스트는 이 환경에 fastapi가 없어 실행하지 못했다.
- 실물 차량/HIL 보드에서 torque 응답·부하·전류·온도 검증은 수행하지 않았다.
  기존 position 기반 HIL plant의 통과를 torque 성능 검증으로 해석하지 않는다.

재현 명령:

```bash
cd ~/KAI_ws
si
colcon build --base-paths src/kaiev26_msgs src/Control/software/src/lateral_ctrl \
  --packages-select kaiev26_msgs lateral_ctrl --cmake-args -DBUILD_TESTING=ON
si
ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1 colcon test \
  --packages-select lateral_ctrl --return-code-on-test-failure
colcon test-result --test-result-base build/lateral_ctrl --verbose

cd src/Control
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 python3 -m pytest \
  software/src/vehicle_console/test/hil \
  --ignore=software/src/vehicle_console/test/hil/test_web_console.py \
  --ignore=software/src/vehicle_console/test/hil/test_bridge_worker.py -q
```

## 재시작 복구 회귀 검증 (2026-09-18)

Driver stub 기반 회귀 시험에서 overspeed latch 유지, 노드 재생성 후 latch 해제,
새 명령 대기, 새 명령으로 출력 재개, overspeed 재발 시 재잠금을 확인했다.
`lateral_ctrl`의 CTest 6개 suite, GTest 79개가 통과했다. 이는 Host 노드 수명과
상태 머신의 검증이며 실제 KEYA·차량에서 launch 재시작을 시험한 결과는 아니다.
