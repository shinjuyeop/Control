# 회생제동 도입 + Governor slew 제거 지침 (STM_A longitudinal / command_governor)

> 2026-10-07 통합: 기존 주행·공중 Speed 프리셋과 governor 가·감속 slew를 유지한다.
> 아래 slew 제거와 공중 Torque·적분 25 A 변경은 채택하지 않았다. 회생 기능·보호·Run 설정 비교는 유지하며 회생 기본값은 꺼짐이다.

> 대상: Claude Code. 2026-10-01 EZkontrol **Sine-wave(FOC)** 전환 후 공중 시험 결과와 설계 논의를 정리한 문서다.
> 선행 문서: `docs/EZKONTROL_TORQUE_CONTROL_REWORK.md`, `docs/reports/2026-10-01_ezkontrol_sine_rework.md`.
> **수정 전에 반드시 현재 저장소 코드를 먼저 읽고** 이 문서의 이름·인덱스·위치와 대조할 것. 다르면 현재 코드가 기준이다.
> 이 문서의 수치는 시험 후보값이다. 하드코딩하지 말고 설정값으로 둔다.

---

## 0. 작업 원칙

1. **안전 경로는 바꾸지 않는다.** 브레이크 요청, E-stop, STM_B safety frame 무효/timeout, host timeout,
   목표 속도 ≤ 0 → 즉시 `RUN=0`/0 A. 회생이 이 경로보다 우선하면 안 된다.
2. 회생은 **기본값에서 꺼져 있어야 한다**(한도 0 = 비활성). 프리셋에서 명시적으로 켠다.
3. 한 기능씩 단계적으로: Part A(governor slew 제거) → Part B1(수동 회생 경로 보강) → Part B2(PI 자동 회생).
   B2는 B1의 실차 확인 전에는 기본 비활성으로만 머지한다.
4. 순수 로직은 HAL 비의존 모듈(`longitudinal_controller.{c,h}`)에 두고 host 테스트를 추가한다.
5. ABI를 바꾸면 버전을 올리고(`v4 → v5`) `docs/INTERFACES.md`, `docs/CONTROL_TUNING_PARAMETERS.md`,
   `control_tuning`(schema/presets/TUI), `control_tools`(분석), 녹화 파라미터 캡처를 같이 갱신한다.

---

## 1. 배경

### 1.1 Sine(FOC) 전환 후 공중 시험 결과 (2026-10-01, airborne 프리셋 Kp 15 / Ki 10 / 80 A)

- 10↔15, 10↔30 km/h speed_step 반복에서 **RPM 붕괴 0건**, 음수 RPM 0건, fault 0.
  방형파에서 붕괴가 집중되던 "0 A coasting → 회전 중 재인가"가 같은 속도대에서 재현되지 않았다.
- 0 A 명령 중 피드백 상전류: 평균 ≈ 0.2 A, std 1.9 A(~700 rpm) ~ 7.8 A(~1400 rpm), 정확히 0.0인 프레임 0%
  → **FOC는 0 A에서도 브리지를 유지**한다.
- coasting(0 A) 감속은 약 1 m/s². 30→10 km/h에 약 5 s. **감속 수단이 coasting뿐이라 느리다** → 회생 필요.
- 정상상태 오차(−0.3~−0.7 m/s)는 airborne 프리셋 `integrator_limit_a = 6`이 원인(정속 필요 전류 10~17 A).
  → 프리셋 값 25로 상향 예정(이 문서의 코드 작업 대상 아님, 4장 참고).

### 1.2 현재 회생 관련 구현 (스냅샷 기준, 현재 코드로 확인)

- `app_freertos.c`: `/vehicle/drive_current_command`의 **0 이하 상전류**를 AUTO + Torque 모드에서 회생 요청으로 받는다
  (`cb_current` → `Long_ReceiveRegen`). `AUTO_REGEN_MAX_A = 20` 클램프, `AUTO_REGEN_SLEW_A_PER_S = 100`.
  `s_regen_current < 0`이면 `{rpm 0, current s_regen_current, RUN, torque, LONG_PATH_REGEN}` 송신, PI는 `Long_Reset`.
- `joystick_teleop_node.cpp`: LT 트리거 → `phase_current_a = -lt * regen_max_a`, **`regen_max_a` 기본 1.0 A**.
- `Long_Update` 출력은 `[0, max_current]` → **속도 PI는 회생을 만들지 않는다.**
- **빠진 것**: 속도 하한(정지 후 역구동 방지), 버스 전압·충전 전류 제한, 회생 파라미터의 설정화, 회생 상태 텔레메트리,
  Sine 기준 붕괴 감지 임계 조정.

### 1.3 확인되지 않은 사항 (시험으로 판정, 코드는 양쪽 결과를 견딜 것)

- EZkontrol이 Torque 모드에서 **양의 회전 + 음의 목표 상전류**를 회생 토크로 실행하는지
  (CAN 문서는 "구동 시 전류·속도 부호 일치"만 기술).
- 회생 시 보내는 **RPM 필드(현재 0)**가 속도 제한으로 작동해 정지 후 역회전을 막는지.
- EZ 앱의 **EBS 설정(reference 0 A, max voltage 58 V, auto Disable)**이 CAN 음수 전류를 제한하는지.

### 1.4 배터리·컨트롤러 조건

- 팩 51.8 V(14S), **상한 58.8 V**, **연속 충전 39 A**. Part I에서 bus voltage(0.1 V), bus current(0.1 A, 음수=충전)를 받는다.
- EZ 과전압 차단은 운용자가 75 V → 약 60 V로 하향(코드 밖 조치).

### 1.5 Sine에서 확인된 붕괴 감지 오검출

- `phase < −30 A` 규칙: 고속(2244~2838 rpm) 0 A coasting 중 FOC 전류 잡음 −36~−40 A로 발동(MANUAL 경로). RPM은 연속.
- `RPM 변화 > 200 rpm/프레임` 규칙: 350 A 공중 가속에서 578→959 rpm/프레임으로 발동(실제 가속).
- 회생을 도입하면 음수 상전류는 **정상 상태**가 되므로 phase-low 규칙은 반드시 명령 부호를 고려해야 한다.

---

## 2. Part A — command_governor 속도 slew 제거 (P0)

### 2.1 목적

Governor의 `SpeedSlew`(가속/감속 목표 변화율 제한)를 **완전히 제거**한다. 종방향 변화율 제어는 STM_A
(전류 slew, PI, 이후 회생 slew)와 상위 계획기가 담당한다. Governor는 **입력 검증·timeout·brake/0 즉시 전달·재발행**만 한다.

### 2.2 요구사항

- `software/src/command_governor/`
  - `governor_core.{hpp,cpp}`의 `SpeedSlew` 클래스와 사용처 삭제. `evaluate_input` 등 검증 로직은 유지.
  - `command_governor_node.cpp`: `deceleration_slew_mps2`, `acceleration_slew_mps2` 파라미터 선언·검증·로그·동적 변경 처리 삭제.
    `speed_target_mps`는 그대로 전달(brake_engage=true면 기존대로 0).
  - `config/command_governor.yaml`에서 두 키 삭제.
  - `test/test_governor_core.cpp`의 slew 테스트 삭제, 검증·brake·timeout 테스트는 유지/보강.
  - README의 "속도 명령 slew" 절과 파라미터 표 행 삭제, 동작 설명 갱신.
- `software/src/control_launch/launch/vehicle.launch.py`: governor에 slew 파라미터를 넘기는 코드 삭제
  (`acceleration_slew_mps2 = 0.0` 강제 포함).
- `software/src/vehicle_console/launch/hil.launch.py`: 같은 파라미터 전달 삭제.
- `software/src/control_tuning/`
  - schema/naming에서 `governor_deceleration_slew_mps2`, `governor_acceleration_slew_mps2` 삭제.
  - presets(`airborne.yaml`, `driving.yaml`)에서 삭제. **구 프리셋에 이 키가 있으면 오류가 아니라 무시 + TUI 알림**,
    S 저장 시 파일에서 제거(기존 v2→v3 이관 방식과 동일한 정책).
  - TUI 표시 항목 삭제, 관련 테스트(`test_tui.py`, `test_presets.py`) 갱신.
- `software/src/control_tools/`: 녹화 파라미터 캡처·분석에서 키가 없어도 동작. 과거 MCAP의 값은 읽기만(호환).
- 문서: `docs/CONTROL_TUNING_PARAMETERS.md`, `docs/INTERFACES.md`, `docs/VEHICLE_RUNBOOK.md` 등에서 governor slew 언급 삭제/갱신.
  `.yaml.bak` 파일은 저장소에 커밋하지 않는다(있으면 .gitignore 확인).

### 2.3 수용 기준

- `grep -rn "slew_mps2"` 결과가 과거 기록 호환 코드·과거 보고서 외에는 없다.
- governor 단위 테스트 통과, `brake_engage`/목표 0/음수 목표/timeout 동작 불변.
- 구 프리셋 로드 시 오류 없이 경고, 저장 후 키 제거 확인.

---

## 3. Part B — 회생제동

### B0. 코드 밖 선행 조건 (운용자, 참고용)

- EZ 과전압 차단 약 60 V, EBS auto **Disable 유지**, EBS reference는 시험 결과 전까지 0 유지, EBS max voltage ≤ 58 V(권장 57).
- 공중 R1 시험(−5/−10/−20 A)으로 1.3 미확인 사항 판정 후 B2 활성화.

### B1. 수동/외부 회생 경로 보강 (P0)

**B1-1. 회생 파라미터 설정화 (ABI v5)** — 하드코딩 상수를 calibration 필드로 옮긴다.

| 필드(예시 이름) | 기본 | 범위 | 의미 |
|---|---|---|---|
| `regen_max_current_a` | **0 (비활성)** | 0…60 | 회생 상전류 크기 상한(양수로 저장, 음수로 송신). 0이면 모든 회생 무효 |
| `regen_slew_a_s` | 100 | 10…1000 | 회생 전류 변화율(증가·감소 공통) |
| `regen_fade_speed_mps` | 1.5 | 0.3…5 | 이 속도 아래부터 회생 한도를 선형 감소 |
| `regen_cutoff_speed_mps` | 0.8 | 0.1… < fade | 이 속도 이하에서 회생 0 |
| `regen_voltage_soft_v` | 57.0 | 40…60 | bus 전압 이 값부터 회생 한도 선형 감소 |
| `regen_voltage_hard_v` | 58.0 | soft…60 | 이 값 이상 회생 0 |
| `regen_bus_current_limit_a` | 30 | 1…39 | 측정 bus 충전전류(음수)의 크기 상한. 초과 시 회생 감소 |

- 현재 `AUTO_REGEN_MAX_A`, `AUTO_REGEN_SLEW_A_PER_S` 상수는 위 필드로 대체(기본 상한 0이므로 기존 20 A 동작은 프리셋으로 재현).
- host(`control_tuning`) schema·preset·TUI·readback 비교, `Long_DecodeRequest` 범위 검증, v4 요청 호환(누락 필드 = 기본값) 처리.

**B1-2. 회생 허용 조건 (모두 만족할 때만 음수 전류)**
- AUTO, Torque(또는 외부 전류) 경로, `s_long_inhibit == LONG_OK`, feedback valid(Part I < 500 ms, Part II < 1 s),
  fault 0, EZ 제어 모드 Torque, 전진(측정 속도 > cutoff, 목표 ≥ 0), 브레이크/HALTED 경로 아님.
- 하나라도 깨지면 회생 목표 0 → `regen_slew_a_s`로 0 복귀. 단 **안전 정지 경로는 slew 없이 즉시 0/HALTED**.

**B1-3. 회생 한도 계산 (매 50 ms)**
```
limit = regen_max_current_a
      × speed_factor(v)    // v ≤ cutoff: 0, cutoff…fade: 선형 0→1, ≥ fade: 1
      × voltage_factor(V)  // V ≤ soft: 1, soft…hard: 선형 1→0, ≥ hard: 0
bus_guard: 측정 bus current < −regen_bus_current_limit_a 이면 limit을 프레임당 일정 비율(예: 20%) 감소, 회복 시 slew로 복귀
target = −min(요청 크기, limit)
output = slew(target, regen_slew_a_s)
```
- Part I이 오래되면(전압·전류 미상) limit = 0.
- 정지 후 역구동 방지가 핵심이다. cutoff 이하에서 반드시 0이 되는지 테스트로 고정.

**B1-4. 회생 시 CAN RPM 필드**
- 현재 0. 1.3의 시험 결과를 받기 전까지 **설정 가능한 선택지**로 둔다: `0`(현재) 또는 `현재 측정 rpm 근처의 양수 상한`(예: rpm_ceiling).
  기본은 현재 동작(0) 유지. 필드: `regen_rpm_field_mode` 0/1.

**B1-5. 텔레메트리 (status v5에 추가)**
- 회생 요청값, 계산된 limit, speed/voltage factor, bus guard 활성, 최종 회생 전류, 회생 경로 활성 플래그.
- `control_tools longitudinal_analysis`에 회생 구간 요약(감속도, bus 충전전류 min, 전압 max, 명령 대비 추종) 추가.

**B1-6. 조이스틱**
- `joystick_teleop_node`의 `regen_max_a` 기본 1.0 A 유지하되 launch 인자로 노출(시험 시 5/10/20 A 지정).
  최종 상한은 STM_A `regen_max_current_a`가 결정한다는 점을 README에 명시.

### B2. 속도 PI 통합 자동 회생 (P1, 기본 비활성)

- `regen_max_current_a > 0` **그리고** 새 플래그 `pi_regen_enabled = 1`일 때만 동작. 기본 0.
- `Long_Update` 출력 범위를 `[−regen_limit(B1-3), max_current]`로 확장.
- **구동↔회생 전환 히스테리시스**: 속도 오차 `e = target − v`
  - 구동 → 회생 진입: `e < −regen_enter_error_mps`(기본 0.3) **그리고** 구동 전류가 0까지 내려온 뒤.
  - 회생 → 구동 복귀: `e > −regen_exit_error_mps`(기본 0.1).
  - 그 사이 불감대에서는 0 A(coast). 0 A를 지나며 부호가 한 프레임에 뒤집히지 않게 전류는 slew로만 0을 통과.
- 회생 영역 P항: 별도 게인 `regen_kp_a_per_mps`(기본 0 → 꺼짐; 시험 후 결정). 적분은 회생 영역에서 동결(음의 적분 부채 금지 원칙 유지).
- FF가 켜져 있으면 FF 음수 성분(감속 요구)을 회생으로 쓰지 않는다(B2 1차 범위 밖).
- 목표 0/브레이크는 기존대로 즉시 0/HALTED(회생으로 정지시키지 않음).

### B3. 붕괴 감지 규칙 조정 (P0, B1과 함께)

- **phase-low 규칙**: 명령이 회생(음수)일 때는 적용하지 않는다. 0 A 명령일 때는 파형별 임계 사용:
  `collapse_phase_low_square_a`(기본 −30), `collapse_phase_low_sine_a`(기본 −60).
- **phase-high/bus-high 규칙**: 회생 중에는 `|phase| > |C| + 60` 형태로 크기 기준 적용.
- **RPM 규칙**: 명령 전류가 클 때 실제 가속을 오검출하지 않도록, 허용 변화량을
  `max(200, k × |command| )` rpm/프레임(예: k = 3)으로 하거나, 직전 N 프레임 평균 가속 대비 이탈로 판정.
  기준은 2026-09-29~10-01 MCAP(방형파 붕괴 49건, Sine 오검출 사례)를 회귀 데이터로 써서 정한다.
- 펌웨어 상수와 `control_tools`의 `collapse` 요약 규칙을 같은 정의로 유지.

---

## 4. 같이 처리하면 좋은 운용 문제 (P2)

- **launch calibration helper가 실패해도 구동 스택이 계속 돈다**(firmware 모드). 2026-10-01에 STM_A 재부팅 후 DEFAULT
  (480 A, Kp 180)로 공중 시험이 3회 진행됨. 개선안 중 택1:
  - 실패 시 **AUTO 시험 스크립트(vehicle_test_scenarios)가 시작을 거부**(readback source가 HOST_OVERRIDE가 아니거나 프리셋과 불일치하면).
  - 또는 STM_A revision이 0으로 돌아가면(재부팅 감지) helper를 조건 충족 시 재시도(자동 재전송 금지 정책과 충돌하면 경고만).
- airborne 프리셋 `drive_integrator_limit_a` 6 → 25 (값 변경만, 범위 검증 이미 허용).
- control_tuning README의 airborne 표(40 A / Ki 2)를 실제 프리셋 값과 맞춤.

---

## 5. 시험·수용 기준

**host 단위 테스트**
- governor: slew 없음, brake/0/음수/timeout 불변.
- 회생: 비활성 기본, 허용 조건 각각 깨질 때 0, speed fade/cutoff, voltage soft/hard, bus guard, slew, 안전 정지 즉시 0,
  cutoff 이하에서 절대 음수 전류 없음, v4 요청 호환.
- B2: 진입/복귀 히스테리시스, 0 통과 slew, 적분 동결, 비활성 시 기존 출력과 비트 단위 동일.
- B3: 회귀 MCAP 사례로 검출/오검출 표 재현.

**HIL(STM_TEST)**: 에뮬레이터에 음수 전류 시 감속하는 플랜트(선택) 추가, cutoff에서 0 복귀 확인.

**공중(운용자)**
- R1: `regen_max_current_a` 5 → 10 → 20, LT로 1~2 s 요청. 판정: 경로 5, phase가 음수 명령 추종, **bus 전류 음수**,
  감속도 > coasting(≈1 m/s²), 전압 상승 < soft, 붕괴·fault 0.
- R2: 회생 유지한 채 감속 → cutoff 이하에서 0 A 복귀, 역회전 없음.
- B2는 R1·R2 통과 후 `pi_regen_enabled = 1`, `regen_kp` 소값으로 speed_step 10↔30 km/h.

**지상**: R1~R2 반복, 감속도·bus 충전전류·전압 기록 후 기본값 확정.

---

## 6. 하지 말 것

- 회생을 기본 활성으로 두지 말 것.
- 안전 정지(브레이크·E-stop·timeout·목표 0)를 회생 감속으로 대체하거나 지연시키지 말 것.
- EBS auto를 코드나 문서에서 권장하지 말 것(EZ 자체 제동이 STM_A 제어와 충돌).
- 배터리 한계(58.8 V, 39 A)를 넘는 기본값을 두지 말 것.
- PI 게인·전류 한도·회생 파라미터를 같은 시험에서 동시에 바꾸지 말 것.

---

## 7. 구현 결과 (2026-10-02, kty)

Part A, B1, B2, B3, 4장 P2는 STM_A longitudinal ABI v5와 host 갱신으로 구현했다. 매핑, 사인파 기록 평가,
R1·R2·B2 시험 계획은 [회생 보고서](reports/2026-10-02_regen_governor_rework.md), wire 계약은
[INTERFACES v5](INTERFACES.md#longitudinal-calibration-v5)를 따른다. 이 문서와 달라진 점:

- B2는 `Long_Update` 출력 범위를 음수로 넓히지 않았다. `Long_PiRegen()`이 회생 요청 크기만 만들고
  LT/외부 요청과 같은 `Long_RegenFrame()` 한도·slew를 거친다. 구동 PI 출력은 계속 `[0, max_current]`이고
  `pi_regen_enabled` 0이면 비트 단위로 같다. 회생 요청은 `regen_kp × (과속 − regen_exit_error)`라 해제 지점에서 0이다.
- 회생이 0 A로 돌아오는 프레임도 회생 경로가 차지한다. 그다음 프레임부터 구동 PI가 0 A에서 재인가 구간을 거친다.
  LT 회생 중에도 PI를 `Long_Reset`으로 지우지 않고 동결한다(적분 유지).
- 회생 허용의 "목표 ≥ 0"은 목표 > 0으로 구현했다. 목표 0은 기존대로 즉시 HALTED이며 회생하지 않는다.
- 외부 전류 모드의 음수 명령은 회생 요청이고 구동 입력은 0 A다. 양수 명령이 오면 회생 요청은 바로 0이 된다.
- `regen_rpm_field_mode` 1의 양수 상한은 측정 RPM 근처 값이 아니라 기존 AUTO RPM 상한(`rpm_limit`)이다.
- bus guard 텔레메트리는 활성 플래그 대신 충전전류 보호 배율(1 = 감소 없음)로 보낸다.
- B3의 비교 기준 C는 최신 명령이 아니라 최신·직전 송신 명령 중 큰 크기다. Part I 프레임이 직전 명령을
  반영할 수 있기 때문이다. 같은 이유로 phase-low는 두 명령이 모두 0 이상일 때만 본다. RPM 허용폭은
  max(200, 3 rpm/A × C)로 정했다(방형파 검출 48/49 유지, 사인파 오검출 13 → 4구간).
- P2는 첫 번째 안을 택했다. 시나리오 TUI가 구동 Run 시작 시 STM_A readback을 런치 프리셋과 비교해 다르면
  거부한다. helper 자동 재전송은 넣지 않았다.
