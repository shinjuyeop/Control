# EZkontrol 토크 제어 재구성 지침 (STM_A 종방향 구동)

> 대상: Claude Code. 이 문서는 2026-09-29 ~ 10-01 공중(airborne) 시험 로그 분석과 설계 논의를 정리한 것이다.
> 코드를 수정하기 전에 **반드시 현재 저장소 코드를 먼저 읽고** 이 문서의 위치·이름과 대조할 것.
> 분석에 사용한 스냅샷(`Control-main` zip, 2026-09-29)보다 실제 빌드가 더 새롭다
> (예: longitudinal config v3, `coast_current_floor_a` 필드가 이미 존재). 이름·인덱스가 다르면 현재 코드를 따른다.

---

## 0. 작업 원칙

1. **안전 경로는 바꾸지 않는다.** 브레이크 요청, E-stop, STM_B safety frame 무효/timeout, AUTO host 명령 timeout,
   목표 속도 ≤ 0 → `RUN=0`(HALTED) + 0 A. 이 의미와 우선순위는 그대로 유지한다.
2. 새 동작은 모두 **설정값(host config 또는 `#define`)으로 켜고 끌 수 있게** 만든다.
   모터 컨트롤러를 Square-wave → Sine-wave(FOC)로 전환 시험 중이므로, FOC에서는 하한 등을 0으로 꺼야 할 수 있다.
3. 기존 host 테스트(`firmware/STM_A/tests/longitudinal_controller_test.c`)를 유지·확장하고, 새 로직마다 단위 테스트를 추가한다.
4. 순수 로직(판정, 하한, 디바운스, 매핑)은 `longitudinal_controller.{c,h}` 같은 HAL 비의존 모듈에 두고,
   `app_freertos.c`에는 호출만 남긴다. SITL/HIL(`firmware/STM_TEST`, `software/.../hil`)과 공유 가능하게.
5. 변경 후 `docs/INTERFACES.md`, `docs/CONTROL_TUNING_PARAMETERS.md`, 관련 보고서 목록을 갱신한다.

---

## 1. 배경: 관측된 현상과 근거

### 1.1 시스템 구성 (관련 부분)

- 모터 컨트롤러: EZkontrol C481200 (48 V, 최대 상전류 1200 A), **BLDCM Square-wave, Hall(120°, 6 PPR), CPWM**, 극쌍 4.
- CAN: 명령 `0x0C01EFD0`(phase current 0.1 A/bit −3200 offset, rpm −32000 offset, Byte4 bit0 RUN, bit1 speed mode),
  상태 Part I `0x1801D0EF`(bus V/I, phase I, rpm), Part II `0x1802D0EF`(온도, RUN/모드, fault). 50 ms 주기.
- 문서상 Torque 모드: 목표 상전류로 토크, RPM 필드는 최대 속도. 전류·속도는 **부호로 방향을 구분**.
- STM_A 현재 구조
  - MANUAL: `PEDAL/100 × MANUAL_TORQUE_LIMIT_A(480)`, 페달 2% 미만 → 0. 기울기 제한 없음. Torque 모드, RUN=1.
  - AUTO: `Long_Update()` 속도 PI → 전류 `[0, max_current]`. coasting(목표 초과 시 0 A),
    hysteresis 후 `reapply_current`/`reapply_slew`로 재인가. 별도 LT 회생 경로(최대 −20 A, 100 A/s).
  - `Long_CheckInhibit()`: `actual < -0.05 m/s`이면 `LONG_REVERSE` → `Long_Reset` + HALTED.
- 컨트롤러 설정(20260908): EBS 0 A / auto Disable, Hold 0 A, ARB Disable(의도된 회생 기능 꺼짐),
  Phase current Kp/Ki 4/512, Acceleration/Deceleration 100/100, **과전압 차단 75 V(팩 상한 58.8 V보다 매우 높음)**.

### 1.2 문제 현상

MANUAL·AUTO 모두에서 **회전 중 상전류가 음수로 크게 튀고(수십~수백 A), 보고 RPM이 0/음수로 붕괴**한다.
공중(무부하)에서도 재현된다. 바퀴도 실제로 크게 제동된다.

### 1.3 로그 근거 (공중, AUTO Torque, 20 ms `/drive/status`)

**(a) 0 A coasting 후 회전 중 재인가 (2026-09-29, 10/40 A envelope, 하한 없음)**

- 0 A 명령 시 피드백 phase/bus = 정확히 0.0 → 컨트롤러가 **출력(브리지)을 끈다.**
- 약 490~505 rpm에서 PI가 1~2 A부터 재인가하는 순간 붕괴: 11건 중 8건.
  - 예: 22:40 run t=100.08 s, 명령 1 A → phase **−84.5 A**, bus **+50.8/−36.2 A**, rpm 494 → 0 → −98 → 322 → 0.
  - 정상 사례(22:32~22:34, 6건)는 재인가 순간 RPM 추정이 연속(472→465→454), 스파이크 없음.
- 보고 RPM 변화율이 물리적으로 불가능(수만 rpm/s; 40 A 가속은 약 1000 rpm/s) → 컨트롤러가 속도·위치 추정을 잃음.

**(b) 명령 대비 피드백 추종 (전체 로그, 명령 안정 구간, rpm > 100)**

| 명령 | n | 피드백 평균 | std | p10–p90 |
|---|---|---|---|---|
| 0–4 A | 542 | **0.8** | 2.5 | 0.0–2.2 |
| 4–8 A | 556 | 7.8 | 3.4 | 3.8–12.8 |
| 8–12 A | 4589 | 9.6 | 4.3 | 4.4–15.7 |
| 12–16 A | 329 | 15.1 | 6.0 | 6.9–21.7 |
| 35–45 A | 37 | 37.1 | 5.9 | 28.6–44.5 |

→ **약 4 A 이하 데드존**, 그 이상은 평균 추종(40 A에서 약 −7%), 순간 리플 ±5~7 A(방형파 정류 + Part I 순간값 샘플링).

**(c) coast 하한 12 A 적용 run (2026-09-30 00:11, 20↔40 km/h, max 80 A, slew_down 50 A/s)**

- 사용자 실험상 붕괴를 막는 최소 하한이 약 12 A였음.
- 그러나 하한이 **coasting 중에만** 적용되고 coasting→재인가 전환에서 빠져, 명령이 12 A 아래로 내려간 프레임이 3번 발생.
  **3번 모두 같은/다음 프레임에 붕괴**, 그 외 구간은 붕괴 0건.

| t | 명령 | phase | bus | rpm |
|---|---|---|---|---|
| 7.97 | 9.5 A | +257.6 A | +171.7 A | 948→802→0 |
| 9.78 | 11.1→10.6 A | −230.4 A | +176.2 A | 1003→842→0→−377 |
| 11.72 | 11.2 A | −17→+98.9 A | −29.8→+99.7 A | 959→0→509→−596 |

- bus 큰 양수 + 급제동 = 역상 제동(plugging) 성격.

**(d) HALTED 연쇄 (22:29 run)**

- 스파이크 후 저속에서 speed_mps −0.11/−0.13 → `LONG_REVERSE` → 한 프레임 HALTED → 즉시 RUN 재시작.
- 고속에서는 RPM despike gate가 음수 RPM을 막지만, 저속(gate 기본폭 40 rpm)에서는 −21/−25 rpm이 통과한다.

### 1.4 원인 판단 (현재 상태)

| 가설 | 상태 |
|---|---|
| EZkontrol(Square-wave)이 **굴러가는 중 약 12 A 미만 명령/0 A→재인가**에서 불안정(출력 on/off, 추정 리셋) | **가장 유력.** (a)(c) 근거 |
| Hall 신호 글리치 | 가능성 낮아졌으나 미배제. 오실로스코프 글리치 트리거 시험 예정 |
| 극쌍 설정 오류 | 배제(방형파 Hall 정류는 극쌍과 무관, 지상 RPM–GNSS 일치) |
| STM_A 명령 로직 자체 | 원인 아님(명령 모양이 다른 MANUAL/AUTO 모두 재현). 다만 위험 영역을 명령하지 않도록 회피 설계 필요 |
| `LONG_REVERSE` 즉시 HALTED | 최초 원인 아님, **2차 연쇄 요인** |

컨트롤러는 Sine-wave(FOC) 전환 시험 중이다. FOC에서는 0 A도 능동 제어라 위 문제가 사라질 수 있으나 미검증.
따라서 아래 기능은 **Square/FOC 양쪽에서 설정으로 조정 가능**해야 한다.

---

## 2. 요구사항

우선순위: **P0** 이번 재구성 필수, **P1** 함께 하면 좋음, **P2** 데이터 확보 후.

### P0-1. 최소 구동 전류(굴러가는 중 하한)를 최종 출력에 강제

**문제**: 현재 하한이 coasting 분기에서만 적용되어 재인가 경로·다른 분기에서 하한 아래 명령이 나간다.

**요구**
- 상태 `rolling_hold_active`를 hysteresis로 판정:
  - ON: 전진, feedback valid, `CURRENT_SPEED >= HOLD_ON_SPEED_MPS`(기본 0.35 m/s ≈ 65 motor rpm)
  - OFF: `CURRENT_SPEED <= HOLD_OFF_SPEED_MPS`(기본 0.20 m/s) 또는 후진/feedback invalid/HALTED 경로/회생 경로
- `rolling_hold_active`이면 **모든 구동 분기(AUTO Torque, AUTO external current, MANUAL 전진)의 최종 명령**에
  `current = max(current, DRIVE_CURRENT_MIN_A)` 적용. 송신 직전 한 곳에서 처리(분기마다 흩뿌리지 말 것).
- 설정: `DRIVE_CURRENT_MIN_A` 기본 12.0(Square), **0이면 기능 비활성**(FOC 시험용). 기존 `coast_current_floor_a`가 있으면 이 의미로 통합.
- 정지 상태(hold OFF)에서 출발: 0 → `DRIVE_CURRENT_MIN_A` 이상으로 **스텝**(데드존 1~4 A를 천천히 지나지 않게).
  즉 `0 < 명령 < DRIVE_CURRENT_MIN_A`인 값은 출력하지 않는다(0 또는 ≥ 하한). 이 양자화도 설정으로 끌 수 있게.
- 텔레메트리: 하한 적용 여부 플래그를 `/drive/longitudinal_status`(또는 status)에 추가.

**테스트**: 재인가/coasting/분기 전환 어느 경로에서도 hold 활성 중 출력 < 하한이 없음을 검증.

### P0-2. 하한 적용 시 가속 지연 제거 (PI 상태 seeding)

**문제**: 출력만 `max()`하면 PI 내부 `s->current`는 0에 머물러, 재가속 시 0부터(reapply_slew로) 올라와 하한 위로 나오기까지 0.3~0.5 s 평평한 구간이 생긴다.

**요구**
- hold 활성 중 `Long_Update()` 호출 전에 `controller.current = max(controller.current, DRIVE_CURRENT_MIN_A)`.
- 하한 ≥ `reapply_current`이면 느린 재인가 구간이 자동으로 건너뛰어지는지 확인(필요 시 reapply 로직이 하한을 기준으로 동작하게 수정).
- 적분항 `i`와 anti-windup 의미는 유지.
- 대안 설계: `Long_Update()`에 `floor` 인자를 추가해 하한 클램프·slew 하한을 내부에서 처리. 어느 쪽이든 테스트로 동작을 고정.

### P0-3. MANUAL 페달 매핑 재정의 (하한 활성 시)

- 전진 + hold 활성: `I = DRIVE_CURRENT_MIN_A + PEDAL/100 × (MANUAL_TORQUE_LIMIT_A − DRIVE_CURRENT_MIN_A)`.
  페달 0%여도 하한 유지, 1%부터 즉시 반응.
- hold 비활성(정지)·후진: 기존 매핑.
- 페달 데드밴드(2%)는 유지.

### P0-4. `LONG_REVERSE` 디바운스

- 현재 `actual < -0.05` 한 샘플로 즉시 inhibit → HALTED.
- 변경: `actual < -REVERSE_SPEED_MPS`(기본 0.30)가 **N 프레임 연속**(기본 3 = 150 ms)일 때만 `LONG_REVERSE`.
- 디바운스 진행 중에는 RUN 유지, 전류는 0(또는 hold 하한 규칙 따름). 진짜 역회전은 그대로 HALTED.
- 목표 음수(`target < 0`) 판정은 즉시 유지.
- 카운터는 `LongController`/`LongGate`에 두고 `Long_Reset` 정책 정리. 테스트 추가(순간 음수 1~2프레임 → HALTED 안 됨, 3프레임 → HALTED).

### P0-5. 차단 기울기 정책

- 로그상 **빠르게 0으로 내리기(400 A/s)는 안전**, **고속에서 저전류 구간을 천천히 지나기(50 A/s)는 위험**.
- 하한이 활성이면 하한까지는 `slew_down`, 하한 아래로 내려가야 할 때(hold OFF로 전환 등)는 **한 프레임에 0**.
- `slew_down` 기본값과 범위 검토(문서화). 설정 검증(`Long_DecodeRequest`) 범위 갱신.

### P0-6. 붕괴 감지 + 진단 텔레메트리

- 감지 조건(설정 가능, 기본값 예시):
  - `|phase_fb| > |cmd| + 100 A` 또는 `phase_fb < -30 A`(회생 미명령 시)
  - 또는 raw RPM 프레임 변화가 despike gate 초과
  - 또는 `bus_current > max(cmd 기반 기대치) + 100 A`
- 감지 시: 이벤트 카운터 증가, 마지막 이벤트 스냅샷(cmd, phase, bus, raw rpm, hold 상태, 분기) 저장·발행.
  제어 동작은 **기본 OFF**(관측 전용). 옵션으로 "N 프레임 hold 하한 유지·재인가 금지" 반응을 켤 수 있게.
- 목적: Square/FOC 비교 시험에서 붕괴 건수를 펌웨어가 직접 집계.

### P1-1. 피드포워드 구조 (파라미터 기본 0)

- `I_cmd = I_ff + PI`, `I_ff = (m·a_des + F(v)) / b`.
  - `F(v) = c0 + c1·v (+ c2·v²)`, `b`(A→N 환산 또는 A→m/s²), `m`, `a_des`(목표 속도 변화율 또는 host 제공).
- 모든 계수 기본 0 → 기존 동작과 동일. host config로 주입(버전 올리고 `Long_DecodeRequest` 범위 검증).
- 목적: 실차 정속 전류를 PI가 아닌 FF가 담당해 coasting↔재인가 전환 횟수 감소, 명령을 하한 위 안정 영역에 유지.
- 계수는 지상 시험(coast-down, 일정 전류 가속)으로 결정 예정. 지금은 구조와 테스트만.

### P1-2. hysteresis 확대 옵션

- coasting 진입/해제(`deadband`, `hysteresis`)를 설정 범위 안에서 넓게 쓸 수 있게 검증 범위 확인.
  (현재 hysteresis 0.12 m/s → 0.3 m/s 이상 실험 예정)

### P1-3. HIL/SITL 반영

- `firmware/STM_TEST` EZkontrol 에뮬레이터에 선택적 "Square-wave 결함 모델" 추가:
  `|cmd| < 4 A`면 출력 0, 회전 중 `0 < cmd < 12 A` 또는 0→재인가 시 일정 확률/조건으로 phase 스파이크·RPM 0/음수 주입.
- 목적: P0 로직(하한, 디바운스, 감지)을 벤치에서 회귀 시험.

### P2 (데이터 확보 후, 이번 범위 아님)

- FOC 확정 시 전류 스케일 재정의(`MANUAL_TORQUE_LIMIT_A`, `max_current`, `HIL_EZ_PLANT_TORQUE_FULL_SCALE_A`).
- 회생 확장(`AUTO_REGEN_MAX_A`, slew, PI 음수 출력 허용) — 배터리 연속 충전 39 A, 팩 상한 58.8 V 조건 필수.
- 기계식 브레이크 비례 제어(STM_B 서보 중간 각도) 및 blended 감속.

---

## 3. 수정 대상 위치 (스냅샷 기준, 현재 코드로 확인)

| 파일 | 내용 |
|---|---|
| `firmware/STM_A/Core/Src/app_freertos.c` `StartControl()` | AUTO/MANUAL 분기, `send_motor_can()` 호출. 최종 출력 하한·hold 판정 삽입 지점 |
| 같은 파일 `prepare_longitudinal()` | `Long_CheckInhibit()` 호출, feedback valid 판정 |
| 같은 파일 `parse_motor_status_part1()`, `despike_motor_rpm()` | RPM despike, `CURRENT_SPEED` 갱신, 붕괴 감지 입력 |
| 같은 파일 `publish_longitudinal()` | status 배열(v3 인덱스 확인). 새 필드 추가 시 버전·`LONG_STATUS_COUNT` 갱신 |
| `firmware/STM_A/Core/Src/longitudinal_controller.c/.h` | `Long_Update`, `Long_CheckInhibit`, `Long_DecodeRequest`, `LongConfig` |
| `firmware/STM_A/tests/longitudinal_controller_test.c` | 단위 테스트 |
| `firmware/STM_TEST/Core/Src/hil_logic.c` | EZ 에뮬레이터(P1-3) |
| host 측 config 송신·TUI (`software/...`) | 새 config 필드, 표시 이름(`drive_coast_current_floor_a` 등) |
| `docs/INTERFACES.md`, `docs/CONTROL_TUNING_PARAMETERS.md` | 프레임/필드/파라미터 문서 |

---

## 4. 설정값 기본값 제안

| 이름 | 기본(Square) | FOC 시험 | 비고 |
|---|---|---|---|
| `DRIVE_CURRENT_MIN_A` (= coast floor) | 12.0 | 0.0 (비활성) | 0이면 P0-1~3 비활성 |
| `HOLD_ON_SPEED_MPS` / `HOLD_OFF_SPEED_MPS` | 0.35 / 0.20 | 동일 | 약 65 / 37 motor rpm |
| `REVERSE_SPEED_MPS` / `REVERSE_DEBOUNCE_FRAMES` | 0.30 / 3 | 동일 | |
| `slew_down` | 400 A/s 권장 | 시험 | 50 A/s는 Square에서 위험 관측 |
| 붕괴 감지 반응 | OFF(관측만) | OFF | |
| FF 계수 | 0 | 0 | 지상 시험 후 |

환산: `v[m/s] = rpm × π × 0.47 / (60 × 4.60)`.

---

## 5. 검증 시나리오와 합격 기준

**공중(airborne), 기록: `/drive/status` 20 ms, `/drive/longitudinal_status`**

1. AUTO `speed_step` 10↔20 km/h, 20↔40 km/h, 각 4회 반복, slew_down 50/400.
2. MANUAL: 약 500/1000 rpm에서 페달 완전 해제 1~2 s → 재가속 5회. 1000 rpm에서 페달을 천천히 해제 5회.
3. 정지 출발, 후진 기어, 브레이크 요청, host timeout, 목표 0 → 기존 HALTED 동작 불변 확인.

**합격 기준(Square, 하한 12 A)**
- hold 활성 중 `commanded_phase_current_a < DRIVE_CURRENT_MIN_A`인 프레임 **0건**.
- 붕괴 감지 이벤트 **0건**(phase 최저 > −10 A, bus 최대 < 기대치 + 30 A, RPM 연속).
- 재가속 시 하한에서 지연 없이 `slew_up`으로 상승(이전 대비 목표 첫 통과 시간 비교).
- 순간 음수 속도 1~2 프레임에서 HALTED 미발생.

**FOC 시험(하한 0)**: 위 시나리오를 그대로 돌려 붕괴 건수, 0~8 A 추종 평균을 Square 로그와 비교.

**host 테스트**: 하한 강제(모든 분기), seeding, 페달 매핑, 디바운스, 0/하한 양자화, config 범위 검증.

---

## 6. 하지 말 것

- HALTED 안전 경로를 "부드럽게" 하려고 지연·우회하지 말 것.
- 근거 없이 PI 게인·전류 한도를 같이 바꾸지 말 것(한 번에 한 변수).
- 음의 전류(회생) 경로를 이번 범위에서 확대하지 말 것.
- 이 문서의 수치(12 A, 4 A, 속도 임계)를 하드코딩하지 말 것 — 모두 설정값.

## 7. 코드 밖 조치 (참고, 팀이 별도 수행)

- EZkontrol 과전압 차단 75 V → 약 60 V 하향.
- Hall 신호 오실로스코프 글리치 트리거 시험(Pulse width < 200 µs, Single).
- Sine-wave 전환 시 셀프러닝 전후 파라미터 비교 기록.

---

## 8. 구현 결과 (2026-10-01, kty)

P0-1~P0-6, P1-1, P1-3은 STM_A longitudinal ABI v4로 구현했고 P1-2는 범위 검증만 유지했다.
상세 매핑, 데이터 분석, 사인파 시험 계획은 [재구성 보고서](reports/2026-10-01_ezkontrol_sine_rework.md),
wire 계약은 [INTERFACES v4](INTERFACES.md#longitudinal-calibration-v5)를 따른다.
이 문서와 달라진 점:

- 기준 파형은 사인파다. 부팅 기본·프리셋 모두 사인파 하한 0(회피 동작 비활성)이고, 방형파 12 A는
  파형별 값으로 보존해 TUI에서 파형만 바꾸면 켜진다(`DRIVE_CURRENT_MIN_A` = v3 slot 12 통합).
- 1.3(c)의 인과는 일부 반대다. 00:11 run 7.9 s·11.6 s는 12 A 고정 명령에서 RPM 붕괴가 먼저였고
  12 A 미만 명령은 그 결과였다. 하한 12 A run 3개에서 붕괴 시작 11건 중 9건이 12 A 이상 명령이었고,
  EZ 내부 Speed 모드에서도 붕괴가 났다.
- 공중 정속 전류(약 9~12 A) 때문에 방형파 12 A 하한은 공중 저속 목표 추종과 충돌한다(5장 시나리오 1).
- 붕괴 감지 기본값은 예시(100 A, −30 A, 100 A, despike gate) 대신 기록에서 정한 상수다:
  RPM ≥400에서 0/부호 반전/>200 rpm, 상전류 > 명령+60 A 또는 < −30 A, 버스 > 명령+60 A
  (검출 48/49, 오검출 0). despike gate(77 rpm)는 공중 정상 가속에서 오검출 169건이었다.
- MANUAL 페달 한도는 kty 9/29 envelope를 따라 Torque/외부 전류 calibration이면 `max_current`,
  Speed calibration이면 480 A다.
