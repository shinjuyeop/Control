# 종방향 Torque Control 시험 시나리오와 파라미터 튜닝 가이드 — 2026-09-29

> **2026-10-01 갱신.** 여기의 ABI v2(config 16개)는 배포되지 않았고 main의 v3와 합쳐
> [STM_A longitudinal ABI v4](INTERFACES.md#longitudinal-calibration-v5)로 바뀌었다. feedforward 4개 값은 v4 끝에 남았지만
> 기본값은 0(꺼짐)이고 전류 이득은 파형별(방형파/사인파)이다. 과속 시 feedforward를 0으로 지우던
> coast latch는 연속 PI로 바뀌어 P항이 출력을 낮춘다. 부팅 기본은 Torque·사인파이며 Kp 380/Ki 190은
> 기본값이 아니다. MANUAL 페달 envelope는 유지된다. 현재 계약과 시험 순서는
> [재구성 보고서](reports/2026-10-01_ezkontrol_sine_rework.md)를 따른다.

대상 코드: STM_A 20 Hz 종방향 PI + 식별 플랜트 feedforward, MANUAL 페달 envelope 공유,
구동 속도 게인 Kp 380 / Ki 190 후보, calibration wire ABI v2.
근거와 구현 범위는 [feedforward 보고서](reports/2026-09-29_longitudinal_feedforward.md),
이전 PI·MCAP baseline은 [2026-09-21 보고서](reports/2026-09-21_longitudinal_torque_pi.md)를 따릅니다.
운용 절차 전체는 [실차 운용](VEHICLE_RUNBOOK.md), 안전 정책은 [SAFETY.md](SAFETY.md)가 기준입니다.

> **이 문서의 모든 수치는 first-test candidate입니다.** 실차·HIL에서 실행하지 않았고
> STM_A를 flash하지 않은 상태에서 작성했습니다. 가속 성능, overshoot/undershoot,
> chain shock 변화는 아직 측정된 적이 없습니다. 아래 합격 기준은 **관측 항목**이며
> 검증된 차량 성능 기준이 아닙니다.

## 0. 시작 전 확인

| 항목 | 값 | 확인 방법 |
|---|---|---|
| STM_A firmware | ABI v2, status 32 float | `ros2 topic echo /drive/longitudinal_status --once` |
| boot calibration | speed / DEFAULT / revision 0 | 같은 topic의 index 3~5 readback |
| host 패키지 | 같은 커밋의 control_tuning | v1 status는 host가 거부 |
| 차량 상태 | MANUAL, 정지, 페달 해제, 전류 0 | `/drive/status`의 mode·speed·pedal·phase_current |

**v1 payload는 양방향에서 거부됩니다.** 예전 STM_A에 새 host를 붙이면 readback이 오지 않고
launch calibration helper가 적용 실패로 끝납니다. 보드와 host를 같은 커밋으로 맞춥니다.

### 안전 전제

- 브레이크는 STM_B 경로이며 이 변경과 무관합니다. Torque 모드에는 **회생이 없습니다.**
  목표를 낮추면 0 A 타력 주행만 일어나므로 speed 모드보다 감속거리가 길 수 있습니다.
  감속·정지는 브레이크로 계획합니다.
- 안전 정지(STM_B brake, E-stop, safety frame 무효, host timeout)는 slew를 거치지 않고
  즉시 0 A·RUN 해제입니다. 페달 이력도 함께 지워집니다.
- **airborne 프리셋이 적용된 상태에서는 MANUAL 페달도 20 A / 600 rpm으로 제한됩니다.**
  그 상태로 실차를 운전하려 하면 페달이 거의 듣지 않습니다. 지상 주행 전에 반드시
  driving 프리셋을 적용하고 readback을 확인합니다.
- 9/20 기록의 32,000 RPM 보고 문제는 그대로 남아 있습니다. freshness만으로 걸러지지
  않으므로 T9를 통과하기 전에는 driving 고속 시험을 진행하지 않습니다.

### 계산값 참고

| 항목 | 계산 | 값 |
|---|---|---:|
| driving 목표 상한 | 4500 rpm 환산 | 24.1 m/s |
| airborne 목표 상한 | 600 rpm 환산 | 3.21 m/s |
| P항 포화 오차 | 480 A / 380 A per m/s | 1.26 m/s |
| deadband 경계 P항 | 380 × 0.05 | 19 A |
| feedforward 가속 항 | 1 m/s² ÷ 0.00512 | 195 A |

feedforward 정상주행 전류는 `I = 44.5 + 5.63·v` A입니다.

| 차속 m/s | 0 | 3 | 6 | 10 |
|---|---:|---:|---:|---:|
| feedforward 전류 A | 44.5 | 61.4 | 78.3 | 100.8 |

## 1. 준비 절차

```zsh
cd ~/KAI_ws
source install/setup.zsh

# 1) 프리셋 편집·저장 (구동 스택 실행 전)
ros2 run control_tuning control_tuning_tui        # 3 구동응답 / 4 구동출력, S로 저장

# 2) 구동 스택 (공중 시험)
ros2 launch control_launch vehicle.launch.py tuning_preset:=airborne
# 지상 시험
ros2 launch control_launch vehicle.launch.py      # 기본 driving

# 3) 관측
ros2 launch vehicle_console vehicle.launch.py allowed_hosts:=100.112.72.27 \
  web_python:="$HOME/KAI_ws/src/Control/software/src/vehicle_console/.venv/bin/python"

# 4) 녹화 (시험 TUI를 쓰면 TUI에서 ON)
ros2 launch control_tools mcap_logging.launch.py output_base_dir:="$HOME/KAI_ws/records"
```

설정 변경은 **저장 후 같은 프리셋으로 구동 스택을 재시작**할 때 적용됩니다. TUI는 실행값을
바꾸지 않습니다. STM_A는 MANUAL·정지·페달 0·명령 전류 0.1 A 미만·실제 상전류 1 A 이하에서만
calibration을 받습니다. 적용 결과는 요청값이 아니라 readback으로 판단합니다.

| result | 뜻 | 조치 |
|---:|---|---|
| 0 | 대기 | readback 수신 확인 |
| 1 | 적용 완료 | revision +1, source HOST_OVERRIDE 확인 |
| 2 | 잘못된 설정 | 범위 위반. 4장 허용 범위 확인 |
| 3 | MANUAL 정지·무전류 조건 불충족 | 모드·정지·페달·전류 확인 후 재시작 |
| 4 | 다른 설정이 먼저 적용됨 | revision 확인 후 다시 저장 |
| 5 | 다른 요청 처리 중 | 잠시 후 재시도 |

## 2. 시나리오 목록

| 번호 | 시나리오 | 모드 | 프리셋 | 차량 상태 |
|---|---|---|---|---|
| T0 | 통신·boot readback 확인 | MANUAL | 임의 | 정지 |
| T1 | MANUAL 페달 envelope | MANUAL | airborne | 구동륜 띄움 |
| T2 | AUTO 저속 FF/PI 분담 | AUTO | airborne | 구동륜 띄움 |
| T3 | 안전 gate와 복구 | AUTO | airborne | 구동륜 띄움 |
| T4 | MANUAL↔AUTO 이력 분리 | 전환 | airborne | 구동륜 띄움 |
| T5 | Speed calibration fallback | 둘 다 | airborne | 구동륜 띄움 |
| T6 | 프리셋 전환과 페달 복귀 | MANUAL | driving | 정지 |
| T7 | 지상 A/B 속도 계단 | AUTO | driving | 직선 주행 |
| T8 | 목표 계단에서 가속 항 제한 | AUTO | driving | 직선 주행 |
| T9 | RPM 피드백 이상 점검 | AUTO | driving | 전 구간 |

T1~T5는 구동륜을 띄우고 차체를 고정한 상태에서 수행합니다. T6 이후는 지상입니다.
**T1~T5를 통과하지 못하면 지상 시험으로 넘어가지 않습니다.**

### T0 · 통신과 boot readback

- **목적**: firmware/host 조합과 기본 calibration을 확인한다.
- **절차**: MANUAL·정지에서 구동 스택을 실행하고 `/drive/longitudinal_status`를 받는다.
- **관측**: 32 float, version 2, 약 5 Hz. result·source·revision, config 16개 값,
  Part I/II valid, mode SPEED, inhibit `비활성`.
- **합격**: boot가 speed / DEFAULT / revision 0이고 Kp 380, Ki 190,
  feedforward 0.00512 / 0.228 / 0.0288 / 1.0으로 읽힌다.
- **중단**: readback이 없거나 길이가 다르면 firmware와 host 커밋이 어긋난 것이다.

### T1 · MANUAL 페달 envelope

- **목적**: 페달이 적용된 전류 한도·회전수 상한·상승/하강률을 쓰는지 확인한다.
  이번 변경에서 동작이 바뀐 부분이다.
- **조건**: airborne 적용 완료(HOST_OVERRIDE, torque, 20 A, 600 rpm), 구동륜 띄움, MANUAL.
- **절차**
  1. 페달을 아주 조금(10 % 내외) 밟고 유지한다. 명령 전류와 실제 상전류를 본다.
  2. 페달을 100 %까지 천천히 밟아 상한에 닿는지 본다.
  3. 페달을 한 번에 놓고 전류가 0으로 내려가는 시간을 본다.
  4. 후진 기어에서 1~2번을 반복한다.
- **관측**: `/drive/status`의 commanded/actual phase current, commanded motor rpm,
  controller_control_mode(TORQUE), `/drive/longitudinal_status`의 inhibit.
- **합격 관측값**

  | 항목 | 기대 |
  |---|---|
  | 페달 100 % 명령 전류 | 20 A에서 멈춘다 |
  | 명령 회전수 | 600 rpm, 후진은 −600 rpm |
  | 0 → 20 A 도달 시간 | 약 0.17 s, 120 A/s 상승률 |
  | 20 → 0 A 도달 시간 | 약 0.05 s, 400 A/s 하강률 |
  | inhibit | `비활성`. MANUAL에서는 AUTO 루프 기준이라 정상이다 |
- **중단**: 페달 0에서 전류가 남거나, 한도를 넘거나, 기어 부호가 반대면 중단한다.

### T2 · AUTO 저속 동작과 feedforward 경로 확인

- **목적**: 공중에서 AUTO 루프가 방향·목표대로 도는지, feedforward 경로가 계산되는지 본다.
- **절차 A (airborne 기본, feedforward 꺼짐)**: AUTO로 전환하고 목표를
  0.3 → 0.6 → 0.3 m/s로 각 10 s 유지한다.
- **합격 관측값 A**: 목표 0에서 0 A·HALTED. 목표 인가 후 P가 오차를 따르고 적분이
  6 A 한도 안에서 움직인다. FF 전류는 0이다. 회전 방향이 목표와 같다.
- **절차 B (feedforward 기능 확인)**: driving 식별값을 그대로 쓰면 정상주행 전류
  44.5 A가 airborne 20 A 한도에서 즉시 포화되어 분담을 볼 수 없다. 따라서 **벤치 확인용**
  값으로만 한 번 켠다. MANUAL·정지에서 아래로 저장·재시작한다.

  | 항목 | 벤치 확인값 | FF 기여 |
  |---|---:|---|
  | 구동 전류당 가속도 | 0.05 | — |
  | 구동 주행저항 가속도 | 0.2 | 정속 약 4.3 A |
  | 구동 속도 저항 계수 | 0.03 | 0.5 m/s에서 +0.3 A |
  | feedforward 가속 한도 | 0.5 | 최대 +10 A |
- **합격 관측값 B**: 목표 인가 직후 FF가 먼저 오르고, 정속에서 FF가 4~5 A로 유지되며
  적분이 한쪽으로 계속 커지지 않는다. 목표를 올리는 동안만 FF 가속 항이 더해진다.
- **필수 후처리**: B가 끝나면 **네 값을 다시 0으로 되돌려 저장·재시작한다.**
  이 값들은 차량 물성이 아니라 20 A 한도 안에서 경로를 보기 위한 임의값이다.
  공중 가속도로 차량 성능을 평가하지 않는다.
- **중단**: 바퀴 회전 방향이 목표와 반대이거나, 목표 0인데 전류가 남으면 중단한다.

### T3 · 안전 gate와 복구

- **목적**: inhibit 조건마다 0 A·HALTED로 가고 정해진 순서로만 복구하는지 확인한다.
- **절차와 기대**

  | 자극 | 기대 inhibit | 기대 출력 |
  |---|---|---|
  | 목표 0 발행 | 정지 또는 허용 상태에서 PI reset | 0 A |
  | 브레이크 요청 | `정지` | 0 A, RUN 0 |
  | host 명령 중단 500 ms 초과 | `새 명령 대기` | 0 A, RUN 0 |
  | Part I 또는 II 200 ms 단절 | `피드백 단절` | 0 A, RUN 0 |
  | 피드백 복구 직후 | `새 명령 대기` | 복구 후 받은 새 명령까지 0 A |
  | 음의 목표 또는 −0.05 m/s 미만 실측 | `역방향 미지원` | 0 A, speed로 자동 전환하지 않음 |
  | MANUAL 전환 | `비활성` | 페달 경로로 이관, PI 이력 reset |
- **합격**: 각 자극에서 전류가 즉시 0이 되고, 자극 해제만으로 재출발하지 않는다.
  피드백 복구 뒤에는 **복구 이후에 수신한** 새 명령이 있어야 재개한다.
- **기록**: 자극 시각을 운영자가 annotation한다.

### T4 · MANUAL↔AUTO 이력 분리

- **목적**: 페달 명령과 PI 이력이 서로를 오염시키지 않는지 확인한다.
- **절차**
  1. AUTO에서 목표를 유지해 적분이 쌓인 상태를 만든다.
  2. MANUAL로 전환하고 페달을 밟는다. 전류가 페달만 따르는지 본다.
  3. 다시 AUTO로 전환한다. 첫 출력이 0에서 slew로 오르는지 본다.
  4. AUTO 주행 중 피드백을 끊어 reset을 만든 뒤, MANUAL에서 페달 출력이 영향받지 않는지 본다.
- **합격**: 전환 직후 전 모드의 전류가 이어지지 않는다. AUTO 재진입 시 적분이 0에서 시작한다.
  MANUAL 페달 출력이 AUTO reset 때문에 끊기지 않는다.

### T5 · Speed calibration fallback

- **목적**: mode를 speed로 되돌렸을 때 기존 경로가 그대로인지 확인한다.
- **절차**: MANUAL·정지·무전류에서 `구동 제어 방식`을 speed로 저장·재시작한다.
- **합격 관측값**: AUTO에서 EZ 내부 속도제어로 명령하고 mode bit가 speed다.
  MANUAL 페달은 **0~480 A 기존 매핑**으로 돌아가고 상한 회전수는 4500 rpm이다.
  airborne에서는 AUTO speed도 20 A / 600 rpm envelope를 사용한다.
- **주의**: speed 모드에서 `/drive/status`의 commanded_phase_current_a는 **한도**이며
  토크 명령이 아니다.

### T6 · 프리셋 전환과 페달 복귀

- **목적**: 지상 주행 전에 driving envelope가 실제로 적용됐는지 확인한다.
- **절차**: 구동륜을 내리고 MANUAL·정지·무전류에서 driving을 저장·적용·재시작한다.
- **합격**: readback이 torque / 480 A / 4500 rpm / Kp 380 / Ki 190 /
  feedforward 0.00512·0.228·0.0288·1.0이고 source가 HOST_OVERRIDE다.
  페달을 조금 밟았을 때 명령 전류가 20 A 한도에 걸리지 않는다.
- **중단**: 여기서 airborne 값이 읽히면 **지상 주행을 시작하지 않는다.**

### T7 · 지상 A/B 속도 계단

- **목적**: 같은 조건에서 A(speed)와 B(torque+feedforward)를 비교한다.
- **조건**: 같은 노면·방향·적재·SOC·온도, 같은 목표 파형, 확보된 직선 구간.
  설정 전환은 매번 MANUAL 정지·무전류에서 한다.
- **목표 파형**: `0 → 20 → 10 → 20 km/h`, 각 구간 5 s 유지. 시험 TUI 사용 시
  `speed_low_kph` / `speed_high_kph`를 채운다.

```zsh
# 목표 속도를 config에 채운 뒤 실행
ros2 launch vehicle_test_scenarios auto_longitudinal_speed_step.launch.py
```

- **관측 항목**: 10–90 % 상승시간, 목표 첫 통과, 평균·최대 GNSS 가속도, overshoot,
  하향 목표에서의 undershoot, ±0.2 m/s 2 s 정착, 속도 오차 RMS,
  상전류·버스전류 median/p95/peak, 타력 시간과 재인가 횟수, 포화·anti-windup 시간,
  FF/P/I 분담, chain 소리 시각.
- **A 쪽 참고값**: 2026-09-19 v6 기록(목표 6 m/s)의 speed 모드 값이다. **다른 날·조건이므로
  기준선이 아니라 참고다. A는 같은 날 다시 측정한다.**

  | 항목 | 9/19 v6 speed 모드 |
  |---|---:|
  | 10–90 % 상승 | 4.5 s |
  | 목표 첫 통과 | 6.1 s |
  | 최고 GNSS | 8.14 m/s, 목표 대비 +35.7 % |
  | 상전류 median / p95 / peak | 234 / 369 / 530 A |
  | 0.5 s 가속도 peak | 1.397 m/s² |
  | 하향 목표 undershoot | 약 2.4 m/s |
- **판단 기준**
  - 반복 시험의 run-to-run 분산과 함께 판단한다. 1회 비교로 결론 내지 않는다.
  - 분산보다 명백한 상승시간 악화나 최대 가속도 감소는 승인하지 않는다. 이 경우
    전류 추종, 목표 램프, 포화, FF/P/I 분담을 먼저 확인하고 4장 순서로 한 항목씩 조정한다.
  - 감속 성능은 브레이크 계획과 함께 평가한다. PI만으로 모든 조건의 undershoot가
    개선된다고 가정하지 않는다.

### T8 · 목표 계단에서 가속 항 제한

- **목적**: `ff_accel_limit`가 상위 목표의 급변을 흡수하는지 확인한다.
- **절차**: 정속 유지 중 목표를 한 번에 크게(예 10 → 20 km/h) 바꾼다.
- **합격 관측값**: feedforward 전류가 `(가속 한도 + 주행저항 + 속도저항 × 차속) ÷
  전류당 가속도`에서 잘린다. driving 후보로 10 → 20 km/h 계단이면 계단 시점 2.8 m/s에서
  약 256 A, 20 km/h 도달 부근 5.6 m/s에서 약 271 A다. 전류 명령은 상승률 안에서 오르고,
  목표 변화율이 `1/dt` 배로 증폭된 스파이크가 없다.

### T9 · RPM 피드백 이상 점검

- **목적**: PI와 feedforward가 쓰는 속도 피드백의 신뢰성을 확인한다.
- **절차**: T7·T8 기록에서 원본 `motor_rpm`, RPM 환산 차속, GNSS 수평 속도, FF/P/I 전류를
  같은 시각축에서 비교한다.
- **합격 관측값**: 정속에서 RPM 환산과 GNSS가 1 % 안에서 일치한다. 감속·회생 구간에서
  `|RPM 환산 − GNSS|`의 p95와 최대를 기록한다. 32,000 RPM 같은 포화 보고가 있으면
  **fresh 판정을 통과하더라도 실차 고속 Torque 시험을 중단한다.**
- **주의**: 이번 변경에 RPM 필터를 추가하지 않았다. 기존 0 dropout hold만 있다.

## 3. 기록과 분석

```zsh
# 구간 지표 (--start/--end는 첫 양의 AUTO 명령 기준 초)
ros2 run control_tools longitudinal_analysis \
  ~/KAI_ws/records/<폴더>/rosbag --start 0 --end 10.5 --target 6

# PI/FF 샘플 전체를 CSV로
ros2 run control_tools longitudinal_analysis ~/KAI_ws/records/<폴더>/rosbag \
  --pi-csv ~/KAI_ws/records/<폴더>/pi.csv
```

A/B는 **같은 창과 같은 목표 파형**으로 비교합니다. 출력의 `acceleration_source`가
`gnss`인지 확인합니다. `speed_mps`면 RPM 환산 속도를 쓴 것이므로 차체 가속도로 취급하지 않습니다.
`exploratory_fit_accel_equals_gain_current_plus_offset`은 탐색용 2변수 회귀이며
시스템 동정이 아닙니다. 녹화에는 `/drive/longitudinal_status`가 포함되어야 합니다.

---

## 4. 파라미터 튜닝 가이드라인

### 4.1 원칙

1. **한 번에 한 항목만** 바꾸고 같은 조건에서 다시 측정합니다.
2. 변경은 MANUAL·정지·무전류에서만 받아들여집니다. 저장 후 같은 프리셋으로 구동 스택을
   재시작하고 **readback으로 확인**합니다. 요청값을 적용값으로 보지 않습니다.
3. 조정 순서는 **feedforward 상수 → Kp → Ki → 적분 한도 → slew → deadband/hysteresis →
   reapply**입니다. 모델 오차를 게인으로 덮지 않습니다.
4. 허용 범위는 코드 계약이며 검증된 tuning range가 아닙니다. 범위 안이라도 안전한
   값이라는 뜻은 아닙니다.
5. 공중 시험값으로 지상 성능을 판단하지 않습니다. airborne은 기능 확인용입니다.

### 4.2 허용 범위

| 항목 | 허용 범위 | driving 후보 | 첫 조정 폭 |
|---|---|---:|---|
| 구동 제어 방식 | speed / torque | torque | — |
| 구동 속도 P 게인 | 0 초과 ~ 1000 | 380 | ±20 % |
| 구동 속도 I 게인 | 0 ~ 200 | 190 | ±25 % |
| 구동 전류 한도 | 0 초과 ~ 480 | 480 | 내릴 때만 |
| 구동 적분 전류 한도 | 0 ~ 전류 한도 | 120 | ±30 A |
| 구동 전류 상승률 | 0 초과 ~ 9600 | 2400 | ±400 A/s |
| 구동 전류 하강률 | 0 초과 ~ 9600 | 4800 | ±800 A/s |
| 구동 속도 데드밴드 | 0 ~ 0.2 | 0.05 | ±0.01 m/s |
| 구동 타력 해제 오차 | 데드밴드 초과 ~ 0.5 | 0.12 | ±0.02 m/s |
| 구동 재인가 전류 구간 | 0 ~ 전류 한도 | 20 | ±5 A |
| 구동 재인가 전류 상승률 | 0 초과 ~ 상승률 | 200 | ±50 A/s |
| 구동 모터 회전수 한도 | 100 ~ 4500 | 4500 | — |
| 구동 전류당 가속도 | 0 또는 0.0005 ~ 0.05 | 0.00512 | 재식별로만 |
| 구동 주행저항 가속도 | 0 ~ 2 | 0.228 | 재식별로만 |
| 구동 속도 저항 계수 | 0 ~ 1 | 0.0288 | 재식별로만 |
| feedforward 가속 한도 | 0 ~ 5 | 1.0 | ±0.25 m/s² |

전류당 가속도를 0으로 두면 feedforward 전체가 꺼지고 순수 PI가 됩니다. 되돌리기가
가장 빠른 안전판입니다.

### 4.3 게인 계산

`K`를 전류당 가속도라고 할 때 목표 대역 `ω_n`, 감쇠 `ζ`에 대해

```
Kp = 2·ζ·ω_n / K      Ki = ω_n² / K
```

`K = 0.00512` 기준입니다.

| ω_n rad/s | ζ | Kp | Ki | 비고 |
|---:|---:|---:|---:|---|
| 0.5 | 1.0 | 195 | 49 | 보수적. 첫 실차에서 진동이 보이면 여기까지 내린다 |
| 0.7 | 1.0 | 273 | 96 | 중간 후보 |
| 1.0 | 1.0 | 391 | 195 | 계산값. 실제 적용 후보는 380 / 190 |
| 1.0 초과 | 1.0 | — | 200 초과 | **Ki 상한 200에 걸려 사용할 수 없다** |

현재 후보 380 / 190은 `ω_n ≈ 0.98 rad/s`, `ζ ≈ 0.99`에 해당하며, ζ = 1을 유지하는 한
**코드 범위에서 가능한 가장 빠른 대역**입니다. 더 빠르게 하려면 ζ를 낮추는 선택이
되므로 진동 위험을 감수해야 합니다. 20 Hz 이산화에서 `ω_n·T = 0.05`입니다.

### 4.4 feedforward 재식별

노면, 적재, 타이어 압력, SOC가 바뀌면 상수를 다시 구합니다. 순서는 다음과 같습니다.

1. **K**: 직선·정상 가속 구간에서 분석 도구의
   `exploratory_fit_accel_equals_gain_current_plus_offset`의 기울기를 본다.
   `net_acceleration_per_amp_p10_median_p90`의 중앙값도 함께 본다.
2. **c0, c1**: 서로 다른 두 정속 구간(예 3 m/s, 8 m/s)에서 실제 상전류의 중앙값을 읽는다.
   `I = (c0 + c1·v) / K`이므로 두 점의 기울기가 `c1/K`, 절편이 `c0/K`다.
   여기에 1번의 `K`를 곱해 `c0`, `c1`을 얻는다.
3. 새 값으로 저장·재시작한 뒤 정속에서 적분이 0 근처에 머무는지 확인한다.
   적분이 한쪽으로 계속 커지면 아직 c0·c1이 작은 것이다.

2변수 회귀만으로는 c0와 c1이 분리되지 않습니다. 반드시 서로 다른 속도대에서 읽습니다.
경사가 있는 구간은 c0에 섞이므로 같은 구간을 왕복해 평균을 씁니다.

### 4.5 증상별 처방

| 증상 | 먼저 볼 신호 | 조정 | 방향 |
|---|---|---|---|
| 정속에서 적분이 계속 커지고 속도가 목표보다 낮다 | I 전류, FF 전류 | 주행저항·속도저항 | 재식별 후 상향 |
| 정속에서 FF만으로 과속한다 | FF 전류, 타력 상태 | 주행저항·속도저항 | 재식별 후 하향 |
| 가속 초기 전류가 늦게 오른다 | 전류 명령 기울기 | 전류 상승률, feedforward 가속 한도 | 상향 |
| 가속 내내 전류가 상한에 붙어 있다 | 포화·anti-windup | 전류 한도는 유지, 상위 목표 램프 | 목표 램프 완화 |
| 목표 근처에서 진동한다 | P 전류 부호 반전 | P 게인 | 20 %씩 하향, 그다음 I 게인 |
| 목표 도달이 느리고 오차가 남는다 | I 전류 증가 속도 | I 게인 | 25 %씩 상향, 적분 한도 확인 |
| 목표를 넘은 뒤 오래 구동한다 | 타력 상태, I 전류 | 데드밴드, 타력 해제 오차 | 데드밴드 하향 |
| 타력과 구동을 자주 왕복한다 | 타력 상태 토글 | 타력 해제 오차 | 상향 |
| 재인가 때 chain 충격이 있다 | 재인가 횟수, 전류 기울기 | 재인가 전류 구간·상승률 | 하향 |
| 전류 명령이 거칠고 튄다 | 원본 RPM vs GNSS | 먼저 T9를 확인 | 게인 조정 전에 피드백 점검 |
| 데드밴드 안에서 미세 진동 | P 전류 | 데드밴드 | 0.05 → 0.08 |
| 감속에서 목표 아래로 크게 내려간다 | 타력 시간, 브레이크 | 상위 감속 계획 | Torque에는 회생이 없음 |

### 4.6 하지 않을 것

- 허용 범위를 넓히기 위해 firmware 검사값을 바꾸지 않습니다. 범위는 20 Hz 수치 권한과
  하드웨어 능력을 보호합니다.
- 실차 A/B 없이 `Long_DefaultConfig()`를 동결하지 않습니다. 동결은 동등 조건 MCAP과
  readback을 함께 보관한 뒤에만 합니다. 그때 default mode를 torque로 바꾸는 선택을 검토합니다.
- airborne envelope로 지상 주행을 시험하지 않습니다.
- RPM 이상을 게인이나 slew로 덮지 않습니다. 원인 점검이 먼저입니다.
- 회생·EBS·배터리 충전 제어를 이 시험 범위에서 켜지 않습니다.

## 5. 결과 정리

시험마다 다음을 같은 폴더에 남깁니다.

1. MCAP과 적용된 readback 값 전체, 프리셋 파일 사본.
2. 분석 도구 JSON 출력과 A/B 비교표.
3. 노면·기온·SOC·적재·타이어 압력, 시험 시각.
4. 운영자 annotation: chain 소리, 체감 가속, 중단 사유.
5. 바꾼 파라미터와 바꾼 이유, 다음에 바꿀 항목.

동등 조건 A/B가 반복 재현되면 검증된 driving 전체 config를 firmware default로 동결하고,
같은 값으로 host 테스트와 Release 빌드를 다시 실행한 뒤 flash합니다. PC override 없이
DEFAULT readback과 성능을 한 번 더 확인합니다.
