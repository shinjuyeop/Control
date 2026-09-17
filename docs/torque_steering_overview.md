# 토크 모드 조향 제어 — 종합 설계 문서 (배경 → 결론)

이 문서는 조향 토크 제어 작업 전체를 처음부터 끝까지 정리한 것이다. Claude Code가
이 문서 하나로 배경·진단·설계·기존 구현·남은 작업을 파악할 수 있게 구성했다.
실행 절차는 [`Experiment_RUNBOOK.md`](Experiment_RUNBOOK.md), 제어법칙 상세는
[`torque_mode_method_A_design.md`](torque_mode_method_A_design.md)를 함께 본다.

---

## 1. 배경 — 왜 토크 모드를 검토하는가

### 1.1 시스템 개요
- 조향 액추에이터: KEYA KY170DJD01005-08G-V1 서보모터 + KEYA 드라이버, CAN 제어.
- 드라이버는 3중 캐스케이드(Position → Speed → Current)를 내부에서 돌린다.
- 현재 운용은 **Position 모드**(`0020=3`): ROS가 목표각을 CAN으로 주면 드라이버가
  세 루프를 모두 내부에서 닫는다.
- 제어 주기: ROS 측 50 Hz(20 ms). 드라이버 내부 전류 루프는 kHz.

### 1.2 최초 동기
자율주행 조향에서 오버슛이 관측됨. 최초 가설은 "차량 관성을 모터가 못 이겨서
밀린다 → 토크를 직접 제어하면 관성을 이길 수 있다"였다.

---

## 2. 진단 — 데이터가 밝힌 실제 원인

여러 차례의 무부하/접지 로그(MCAP) 분석으로 가설을 검증했고, 최초 가설은
**부분적으로 틀렸다**는 것이 밝혀졌다.

### 2.1 오버슛은 관성이 아니라 감쇠 부족 + 데드존
- 오버슛이 진동(목표를 넘고 되돌아오며 반복) 형태 → 순수 관성이면 안 나오는
  저감쇠(underdamped) 2차 응답. 관성이 벽이면 언더슛(못 미침)이 나와야 한다.
- 게인에 민감(Position Kp 0.5=과증폭, 0.3=응답소멸) → 관성 벽이 아니라 튜닝
  가능한 폐루프 영역.
- **띄운(무부하) 상태에서도, 속도와 무관하게** 조향 step만으로 오버슛 발생
  → 타이어-지면 마찰·차량 관성·속도 모두 주원인이 아님. 남는 건 **조향계 자체의
  회전 관성 + 제어 감쇠 부족**.

### 2.2 작은 각일수록 오버슛이 큰 진짜 이유 = 정지 마찰 데드존
- 무부하 진폭 스윕(±3°~±18°) 결과:

  | 진폭 | 오버슛 | 감쇠비 ζ |
  |---|---|---|
  | ±3° | ~100% | ~0 (발산 직전) |
  | ±7° | ~68% | 0.12 |
  | ±12° | ~35% | 0.32 |
  | ±15° | ~8~12% | 0.56~0.64 |
  | ±18° | ~4~7% | 0.64~0.72 |

- **큰 각은 이미 이상적**(ζ≈0.64, 오버슛 7% 내외). 문제는 작은 각.
- 작은 각 오버슛의 원인은 **정지 마찰 데드존**: 출발 시 마찰을 뚫느라 지연되고
  뚫는 순간 운동량이 쌓여 목표를 크게 지나침. 정착 시엔 목표 근처 오차가
  데드존보다 작아 못 뚫고 멈춤(정상상태 오차). breakaway ≈ ±106 raw(정격 약 1%).
- 이는 **비선형(데드존)** 문제라 선형 게인으로 못 잡음. 실험으로 확인:
  - `0014`(Slow Down in Advance) 20→40: 작은 각 오버슛 변화 없음.
  - Position Ki 0.001→0.005: 적분이 데드존 뚫는 순간 반대로 튕겨 정착오차 악화.
- KEYA 파라미터 목록(0000~0063) 전수 확인: **마찰/데드존 보상 파라미터 없음**
  (0015/0016의 compensation은 엔코더/리졸버 영점 보정으로 무관).

### 2.3 결론
- 큰 각: Position 모드로 충분(이미 잘 됨).
- 작은 각: 데드존이 원인이고, Position 모드의 선형 게인으로는 원리적으로 못 잡음.
- 데드존 보상은 **제어 출력에 마찰 상쇄 토크를 직접 더하는 것**으로만 가능 →
  이것이 토크 모드로 가는 **정당한 이유**. 단, 이유는 최초의 "관성"이 아니라
  **마찰(데드존) 보상**으로 바뀌었다.

---

## 3. 설계 — 방법 A (피드포워드 + 피드백)

### 3.1 제어 구조 선택
- 전환(구역별 Position↔Torque 스위칭)은 채터링·bumpless·재-enable 비용이 커서
  기각. 대신 **토크 모드 단일 고정 + 연속 제어법칙**을 택함.
- 판단팀이 목표각 궤적을 이미 다듬어 넘겨준다고 가정 → 자체 궤적 생성(방법 B) 안 함.

### 3.2 제어 법칙 (매 20 ms 재계산)
```
τ = J·α_ff + Kp·e − Kd·ω + F_c·sign(e)
```
- `e = θ_target − θ_actual` (위치 오차)
- `ω` = 각속도(측정 각도 미분 + LPF)
- `α_ff` = 목표 각가속도(판단팀 궤적의 2차 미분, 또는 target_rate 변화율)
- `J·α_ff` : 관성 피드포워드(선제 보상)
- `Kp·e` : 위치 오차 비례
- `Kd·ω` : 감쇠(오버슛 억제)
- `F_c·sign(e)` : **마찰/데드존 보상 — 이 항이 핵심**. 오차 방향으로 breakaway
  토크를 항상 더해 미세 오차도 데드존을 넘어 교정.
- 목표 근처 `|e| < ε`(데드밴드)에서는 F_c를 죽여 미세 떨림 방지.
- 출력은 `±TORQUE_LIMIT`(정격 80% = ±8000 이하)로 클램프.

### 3.3 층위별 속도 보정(접지 단계)
속도 의존성은 항마다 다름:
- **F_c: 크게 변함**(저속 큰 마찰 → 고속 작아짐). 우선 보정 대상.
- **J: 거의 불변**(조향계 회전 관성은 차속 무관).
- **Kp·Kd: 중간**(타이어 복원 토크가 유효 강성에 기여).
- 보정 전략: **게인 스케줄링**. 속도를 x축, F_c를 y축으로 피팅(F_c(v)),
  현재 속도로 보간. Kp·Kd는 일단 고정, 속도별 응답 보고 필요 시 추가.
  (연속 보간이라 불연속 없음.)

---

## 4. 기존 구현 현황 (다른 사람이 앞서 작성)

`software/src/lateral_ctrl/src/torque_steering_controller.cpp` (+ .hpp, 테스트)에
**PD 제어 + 강력한 안전 계층**이 이미 구현돼 있음. `origin/Shin`(63edd73 이후)에서
`steering_actuator_node`의 startup-only `actuation_mode=torque`로 **이미 배선**돼 있다
(0020=2 경로, torque0→Enable, 매 tick 송신, `keyaTorqueCommand` 부호, 진단 토픽).
아래 4.2의 "노드 배선"은 이 작업으로 해소됐다.

### 4.1 이미 있는 것
- 제어: `p_raw = (peak_limit_raw / full_scale_error_rad)·e`, `d_raw = kd·(target_rate − ω)`.
  → **P + D**. `target_rate`를 쓰므로 **속도 피드포워드는 부분적으로 있음**.
- 안전: peak/sustained 이중 전류 제한 + 시간 적산(80% 보호 소프트 회피),
  각속도 하드리밋(필터/비필터 이중), soft-limit 마진 테이퍼링(엔드스톱 보호),
  feedback timeout / angle limit / rate limit fault, settled 판정(정착 시 토크 0),
  방향 부호(`keyaTorqueCommand(cmd, direction_sign)`).
- 설정(`TorqueSteeringConfig` 기본값): kd=1500 raw/(rad/s), peak_limit=2000,
  sustained=1000, peak_duration=0.2 s, full_scale_error=0.5°, control_period=0.02 s,
  min/max_angle=±0.4 rad, hard_rate_limit=120°/s, rate_filter=10 Hz.

### 4.2 없는 것 (= 우리가 얹을 것)
1. **마찰 보상 `F_c·sign(e)`** — 가장 중요. 데드존 문제의 해법이 빠져 있음.
2. **관성 피드포워드 `J·α_ff`** — target_rate(속도)는 쓰나 α(각가속도) 기반 관성
   보상은 없음.
3. **속도 게인 스케줄링** — Kp·Kd·전류제한이 전부 고정 상수. 접지 속도 보정 없음.
4. **노드 배선** — 컨트롤러를 `steering_actuator_node`(또는 신규 노드)에 연결하고
   토크 모드(`0020=2`)로 실제 CAN 송신하는 경로.

### 4.3 재사용 방침
안전 계층·입력 검증은 이 구현이 우리 설계보다 정교하므로 **재사용**한다. 우리
작업은 처음부터 만드는 게 아니라 이 골격에 4.2의 네 가지를 얹는 것이다.

### 4.4 방법 A 확장 구현 (4.2 반영)
- 제어식: `u = clamp_ff(J_ff·α_ff) + P + D + F_c(v)·sign(e)` → 기존 peak/sustained
  envelope·soft-limit taper·settle(토크 0) 그대로 통과. 각 항 기본값 0이라 PD 응답 불변.
- `F_c·sign(e)`: `|e| ≤ ε`에서는 0(`torque_friction_raw`, `torque_friction_deadband_deg`).
- `J·α_ff`: `/vehicle/command`에 궤적 미분이 없고 메시지 ABI를 바꾸지 않으므로, 명령
  **timestamp 기준** 2차 차분 + LPF로 α를 추정한다(`TargetAccelerationEstimator`).
  제어 tick 기준 미분은 20 Hz 명령을 50 Hz로 잡을 때 임펄스로 aliasing되어 기각.
  명령 간격이 `steer_command_timeout_ms`를 넘거나 disable되면 이력을 버린다.
  `|J·α|`는 `torque_feedforward_limit_raw`로 자른다(step 입력의 α 폭주 대비).
- 속도 스케줄: `torque_friction_schedule = [v, F_c, v, F_c, ...]` 구간 선형 보간,
  표 밖은 양 끝 유지. 속도는 `/drive/status.speed_mps` 절댓값. 한 파라미터라
  `ros2 param set` 한 번으로 원자적으로 바뀐다. Kp·Kd 스케줄은 보류(3.3 방침).
- 모든 신규 파라미터는 런타임 변경 가능하며 기존 규칙(검증 후 zero→Disable→재진입)을 따른다.
- 관측: `/steering/torque_ff_raw`, `/steering/torque_friction_raw`, `/steering/target_accel`.
- 후처리: `lateral_ctrl/scripts/torque_sysid_metrics.py` (`step`: %OS·ζ·정착, `inertia`: J·F 회귀).

---

## 5. 남은 작업 (구현 순서)

1. **데이터 획득 실험(무부하)** — F_c, J, Kp·Kd, ε 측정.
   기존 `deadzone_measure_node`로 T0~T2 즉시 가능. [Experiment_RUNBOOK 참조]
2. ~~**컨트롤러 확장**~~ — 완료(4.4). `F_c·sign(e)` + 데드밴드 ε, `J·α_ff`(명령
   timestamp 2차 차분), `F_c(v)` 스케줄.
3. ~~**노드 배선**~~ — 완료(기존 `actuation_mode=torque`). 부호(`torque_direction_sign`)는
   실차에서 T0로 먼저 검증.
4. **PD 재튜닝** — 50 Hz 루프에서 큰 각 ζ≈0.64 재현(기존 kd=1500이 출발점).
5. **마찰·피드포워드 검증** — 작은 각 오버슛/정착 개선 확인.
6. **접지 재검증 + 속도 스케줄** — F_c(v) 측정·피팅, 필요 시 Kp·Kd(v).

### 핵심 리스크
- 부호 오설정 시 즉시 발산 → T0(부호 검증)을 최우선 게이트로.
- 전류 1 A 양자화 → 피드백에 전류 쓰지 말 것(각도만). 전류는 안전 감시·참고용.
- 80% 보호(5 s) → TORQUE_LIMIT 클램프 + 기존 peak/sustained 적산으로 회피.
- watchdog: 1 s 미송신 시 보호. 복구는 enable 프레임 재송신(전원재시작 아님).
- 무부하 튜닝이 접지에서 유지 안 될 수 있음 → 접지 재검증 필수.

---

## 6. 관련 문서
- [`Experiment_RUNBOOK.md`](Experiment_RUNBOOK.md) — 실험 절차·명령어(T0~T3)
- [`torque_mode_method_A_design.md`](torque_mode_method_A_design.md) — 제어법칙·파라미터 3분류
- [`TORQUE_CONTROL.md`](../software/src/lateral_ctrl/TORQUE_CONTROL.md) — torque 운용 계약·전체 파라미터
- [`VEHICLE_RUNBOOK.md`](VEHICLE_RUNBOOK.md) — 실차 상시 운용
- [`DEADZONE_MEASUREMENT.md`](../software/src/lateral_ctrl/DEADZONE_MEASUREMENT.md) — 데드존 측정 기록
- 기존 구현: `software/src/lateral_ctrl/src/torque_steering_controller.cpp`
