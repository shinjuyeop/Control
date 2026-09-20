# longitudinal_ctrl

`/vehicle/command`의 목표 차속과 `/drive/status`의 피드백으로 EZkontrol 상전류
명령을 만들어 `/drive/torque_command`로 보내는 Host 종방향 제어기다. STM_A는
이 명령이 신선하고 `torque_mode=true`일 때만 EZkontrol CAN 명령을 **torque
control**로 보내고, 그 외에는 기존 **speed control**(목표 RPM + 480 A 한도)로
되돌아간다. `lateral_ctrl`이 KEYA 조향을 Host에서 닫는 것과 같은 구조다.

```text
Planning ──/planning/command──> command_governor ──/vehicle/command──┬─> STM_A (HALTED 판단, speed fallback)
                                                                     ├─> STM_B
                                                                     ├─> lateral_ctrl
                                                                     └─> longitudinal_ctrl ──/drive/torque_command──> STM_A
/drive/status ─────────────────────────────────────────────────────────> longitudinal_ctrl
```

정지·brake·Host timeout·STM_B 정지 요청의 **HALTED 판단은 계속 STM_A가
`/vehicle/command`로 수행**한다. 이 노드는 HALTED를 대체하거나 늦추지 않는다.

## 왜 바꾸는가

2026-09-20 오전 18개 기록(EZ speed control, Rotor speed Kp=2/Ki=16)에서 확인한 현상은
[분석·시험 문서](../../../docs/LONGITUDINAL_TORQUE_CONTROL_20260920.md)에 있다. 요약:

- 6 m/s 목표에서 GNSS 8.1 m/s까지 과속. 목표를 지난 뒤에도 EZ가 +50~70 A를 계속
  흘리며 5초 이상 천천히 풀린다(내부 적분 windup).
- 감속(6.0→4.8 m/s ramp)에서 상전류가 −231 → +248 → −212 A로 1초 안에 반전하고,
  EZ가 보고하는 RPM이 9.7 → 12.4 → 171 m/s 상당으로 튀거나 0으로 빠진다. 실제
  차속은 새 목표보다 2 m/s 아래(2.9 m/s)로 내려갔다가 회복한다.
- 같은 기록에서 상전류→가속도 플랜트는 `a = 0.00512·I − 0.228 − 0.0288·v`
  (R²=0.81)로 안정적으로 식별됐다. 즉 모터 토크 응답 자체는 예측 가능하다.

EZ 내부 속도루프 대신 Host가 전류를 직접 명령하면 windup·hunting이 EZ 안에서
생기지 않고, 게인·한계·회생량을 MCAP에 남기며 `ros2 param set`으로 바꿀 수 있다.

## 제어 구조

`speed_torque_controller`(ROS 의존 없음, gtest)가 매 주기 다음을 계산한다.

```text
v_ref  → [가감속 ramp] → [실측 ±tracking_error_limit로 제한] → v_shaped, a_ref
I_ff   = a_ref / K + (c0 + c1·v_shaped) / K            # 식별 모델 feedforward
I_pi   = kp·e + ki·∫e dt,  e = LPF(v_shaped) − LPF(v_meas)  # 같은 필터로 지연을 맞춘 오차, 조건부 적분
I_cmd  = clamp(I_ff + I_pi, −regen_cap, +max_drive)     # 회생 상한은 DC 충전전류로 감시
       → slew 제한 → (v < regen_min_speed면 음수 금지) → (정지 조건이면 0)
```

- **차속 추정**: `/drive/status.speed_mps`(RPM 기반)를 `speed_jump_reject_mps`
  이상 튀는 샘플은 버리고(최대 `speed_jump_reject_max_samples` 연속) 2 Hz LPF한다.
  기록의 9.7·12.4·171 m/s spike와 0 dropout은 모두 이 조건으로 걸러진다.
  `speed_source: gnss`로 `/gnss/fix_velocity`를 쓸 수도 있다.
- **bumpless 진입**: torque 모드로 들어갈 때 형상 목표는 현재 차속에서, 출력은
  `/drive/status.phase_current_a`(EZ가 그 순간 흘리던 전류)에서 시작한다.
- **회생**: `max_regen_current_a`(기본 80 A)까지만 음의 전류를 낸다. `0`이면
  감속 시 타력(0 A)만 쓴다. 실측 `bus_current_a`가 `−regen_bus_current_limit_a`
  (기본 35 A, 배터리 연속 충전 39 A 사양 안쪽)를 넘으면 상한을 5 %/tick씩 줄이고
  벗어나면 1 %/tick씩 되돌린다.
- **정지**: 목표·실측이 모두 `stop_speed_mps` 이하이면 0 A, 적분 초기화. 실제
  정지는 Decision의 `brake_engage`(→ STM_A HALTED, STM_B 서보)가 담당한다.
- **회전수 상한**: torque control 명령의 RPM 필드에는
  `max(v_shaped + speed_limit_margin_mps, speed_limit_floor_mps)`를 보내
  EZ 자체 과속 방지를 살린다. `speed_limit_margin_mps: 0`이면 STM_A 기본 4500 rpm.

## torque 모드 허용 조건

아래가 모두 만족될 때만 `torque_mode=true`를 보낸다. 하나라도 깨지면 즉시
`torque_mode=false`, 0 A를 보내고 `/longitudinal/state`에 `off:<이유>`를 남긴다.
STM_A는 그 즉시 speed control로 돌아간다(EZ 재시작·handshake 없음).

| 순서 | 조건 | 상태 문자열 |
|---|---|---|
| 1 | `torque_control_enabled=true` | `disabled_by_parameter` |
| 2 | `/drive/status` 수신·`drive_status_timeout_ms` 이내 | `drive_status_not_received` / `drive_status_stale` |
| 3 | mode AUTO | `mode_not_auto` |
| 4 | `/brake/status`에 E-stop 없음 | `estop` |
| 5 | `stop_request=false` | `stop_request` |
| 6 | `part1_valid` | `part1_invalid` |
| 7 | `fault_bits=0`(valid일 때) | `motor_fault` |
| 8 | gear FORWARD | `gear_reverse` |
| 9 | `/vehicle/command` 수신·`command_timeout_ms` 이내 | `command_not_received` / `command_stale` |
| 10 | `brake_engage=false` | `brake_engage` |
| 11 | `speed_target_mps ≥ 0` | `target_negative` |
| 12 | 유효 차속 샘플이 `feedback_timeout_ms` 이내 | `speed_feedback_stale` |

STM_A 쪽 하드 한계: 명령 전류 ±480 A clamp, RPM 상한 0..4500, 200 ms timeout 시
speed control 복귀, `CURRENT_SPEED < 0.3 m/s`에서 음의 전류 0으로 절단.

## 파라미터

기본값은 [`config/longitudinal_actuator.yaml`](config/longitudinal_actuator.yaml)이다.
`torque_control_enabled`와 게인·한계는 실행 중 `ros2 param set`으로 바꿀 수 있고
(`configure`가 새 한계로 적분기·회생 상한을 다시 자른다), 발행 주기·timeout·
`speed_source`·타이어/감속비는 시작 시에만 읽는다.

| 파라미터 | 기본 | 의미 |
|---|---:|---|
| `torque_control_enabled` | false | false면 항상 `torque_mode=false` |
| `speed_source` | drive_status | `drive_status` / `gnss` |
| `publish_period_ms` | 20 | 제어·발행 주기. STM_A timeout 200 ms |
| `command_timeout_ms` / `drive_status_timeout_ms` / `feedback_timeout_ms` | 200 / 200 / 300 | 해제 조건 |
| `current_gain_mps2_per_a` | 0.00512 | K. 1 m/s²에 195 A |
| `rolling_resistance_mps2` / `speed_damping_per_s` | 0.228 / 0.0288 | 저항 모델. 정속 전류 ≈ 45 + 5.6·v A |
| `feedforward_accel_scale` / `feedforward_resistance_scale` | 1.0 / 1.0 | 0이면 해당 FF 항 제거 |
| `kp_a_per_mps` / `ki_a_per_m` | 380 / 190 | ωn=1 rad/s, ζ=1 기준 |
| `integrator_limit_a` | 150 | 적분기 절대 상한 |
| `accel_limit_mps2` / `decel_limit_mps2` | 1.0 / 1.0 | 목표 ramp. Decision ramp(1/2)보다 느리면 이 값이 우선 |
| `tracking_error_limit_mps` | 1.0 | 형상 목표가 실측에서 벗어날 수 있는 폭 |
| `max_drive_current_a` / `max_regen_current_a` | 300 / 80 | 출력 envelope. STM_A가 ±480 A로 다시 자름 |
| `regen_bus_current_limit_a` | 35 | DC 충전전류 감시 |
| `current_slew_a_per_s` | 800 | 명령 변화율 |
| `regen_min_speed_mps` / `stop_speed_mps` | 0.5 / 0.3 | 회생 금지 차속 / 정지 판정 |
| `speed_filter_hz` / `speed_jump_reject_mps` / `speed_jump_reject_max_samples` | 2 / 1.0 / 8 | 차속 추정 |
| `speed_limit_margin_mps` / `speed_limit_floor_mps` | 2.0 / 3.0 | torque 명령의 RPM 상한 |

게인은 식별된 K로부터 `kp = 2ζωn/K`, `ki = ωn²/K`로 정한다.

| ωn [rad/s] | ζ | kp [A/(m/s)] | ki [A/m] | 비고 |
|---:|---:|---:|---:|---|
| 0.7 | 1.0 | 270 | 96 | 느림, 첫 실차 확인용 |
| **1.0** | **1.0** | **380** | **190** | 기본값 |
| 1.5 | 1.0 | 580 | 440 | 지연 0.15 s에서 위상여유 충분 |
| 1.0 | 0.7 | 270 | 190 | 약간의 overshoot 허용 |

## 토픽

| 토픽 | 타입 | 방향 | 설명 |
|---|---|---|---|
| `/vehicle/command` | `kaiev26_msgs/ActuatorCommand` | 구독 | 목표 차속, brake |
| `/drive/status` | `kaiev26_msgs/DriveActuatorStatus` | 구독 | 차속·전류·mode·fault |
| `/brake/status` | `kaiev26_msgs/BrakeActuatorStatus` | 구독 | E-stop 원인 |
| `/gnss/fix_velocity` | `geometry_msgs/TwistWithCovarianceStamped` | 구독(옵션) | `speed_source: gnss` |
| `/drive/torque_command` | `kaiev26_msgs/DriveTorqueCommand` | 발행 50 Hz | STM_A 소비 |
| `/longitudinal/state` | `std_msgs/String` | 발행 | `active` 또는 `off:<이유>` |
| `/longitudinal/target_speed`, `shaped_target_speed`, `filtered_target_speed`, `target_accel`, `speed`, `speed_raw`, `speed_error` | `std_msgs/Float32` | 발행 | 목표·추정 |
| `/longitudinal/current_ff`, `current_p`, `current_i`, `current_unsaturated`, `current_command`, `regen_cap`, `speed_rejected_total` | `std_msgs/Float32` | 발행 | 전류 성분 |
| `/longitudinal/saturated`, `slew_limited`, `regen_blocked`, `stopped`, `torque_active` | `std_msgs/Bool` | 발행 | 제한 상태 |

`vehicle_test_scenario_runner`는 위 토픽 전부와 이 노드의 파라미터를 Run 기록에
함께 남긴다(노드가 없으면 `absent`로 기록하고 Run을 막지 않는다).

## 빌드·시험·실행

```bash
cd ~/KAI_ws
si
colcon build --packages-select kaiev26_msgs longitudinal_ctrl
colcon test --packages-select longitudinal_ctrl && colcon test-result --verbose
si
# 단독
ros2 launch longitudinal_ctrl longitudinal_actuator.launch.py torque_control_enabled:=true
# 실차 통합
ros2 launch control_launch vehicle.launch.py longitudinal_torque:=true
# 실행 중 전환·튜닝
ros2 param set /longitudinal_actuator_node torque_control_enabled true
ros2 param set /longitudinal_actuator_node max_regen_current_a 0.0     # 타력 감속 비교
ros2 param set /longitudinal_actuator_node kp_a_per_mps 270.0
```

**STM_A 펌웨어가 `/drive/torque_command`를 구독하는 버전이어야 한다.** 예전
펌웨어는 이 토픽을 모르므로 노드를 켜도 speed control 그대로다. 메시지가
추가됐으므로 `firmware/tools/regen_microros.sh`로 micro-ROS 라이브러리를 다시
만들고 STM_A를 다시 플래시한다(STM_B는 사용하는 타입이 바뀌지 않아 그대로).

## 관련 문서

- 분석·설계·EZ 설정 제안·시험 케이스: [`docs/LONGITUDINAL_TORQUE_CONTROL_20260920.md`](../../../docs/LONGITUDINAL_TORQUE_CONTROL_20260920.md)
- 통신 계약: [`docs/INTERFACES.md`](../../../docs/INTERFACES.md)
- 안전 설계: [`docs/SAFETY.md`](../../../docs/SAFETY.md)
- STM_A: [`firmware/STM_A/README.md`](../../../firmware/STM_A/README.md)
