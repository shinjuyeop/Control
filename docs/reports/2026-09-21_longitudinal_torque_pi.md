# AUTO Torque PI와 실차 기준선 — 2026-09-21

이번 변경은 기존 EZkontrol Speed Control 및 MANUAL pedal Torque 경로를 보존하고,
STM_A의 기존 50 ms Control task에 forward AUTO용 positive-current PI를 추가한다.
**아래 calibration은 MCAP 기반 first-test candidate다. 실차 가속 성능 유지,
overshoot/undershoot·chain shock 감소는 아직 검증하지 않았다.**
실측 없는 새 Torque 결과를 만들거나 기존 Speed 기록을 새 제어기의 결과로 해석하지 않는다.

후속 [9/20 전체 MCAP 분석](2026-09-21_0920_longitudinal_review.md)에서
fresh/valid 상태의 32,000 RPM 보고가 확인됐다. **현재 freshness gate만으로 걸러지지 않으므로
Torque 실차 적용 전 RPM 입력 이상 처리를 점검해야 한다.** 아래 gains는 계속 candidate이며
후속 분석에서 코드나 calibration을 변경하지 않았다.

## 1. 사용 기록과 분석 방법

`~/KAI_ws/records/260919/quali_succeed_run/`에서 다음 여섯 bag을 직접 읽었다.
각 경로 아래 `rosbag/rosbag_0.mcap`이며 원본은 수정하지 않았다.

- `20260919_153505_599783_decision_case_01_pure_pursuit_v1p3`
- `20260919_154538_614780_decision_case_01_pure_pursuit_v3`
- `20260919_155517_534099_decision_case_01_pure_pursuit_v4`
- `20260919_160206_275928_decision_case_01_pure_pursuit_v6`
- `20260919_164407_768359_decision_case_01_pure_pursuit_v8p3`
- `20260919_165039_919464_decision_case_01_pure_pursuit_v7`

9/19 morning/afternoon SYSID, 9/20 Decision/cone MCAP도 발견했지만 이 보고서의 수치에는
포함하지 않았다. 주 기준은 충분한 가속·감속·재가속이 있는 v6이며, 높은 가속 비교는
v8p3/v7이다. `v` 숫자는 이 예선 기록에서는 m/s다.

`/vehicle/command`, `/drive/status`, `/brake/status`, `/gnss/fix_velocity`,
`/parameter_events`를 MCAP 수신 시각으로 정렬했다. Drive header=0도 수신 시각으로 읽는다.
명령·GNSS·brake는 직전 250 ms 안의 샘플만 결합한다. AUTO, brake-off,
Part I/II valid, stop_request=false를 기본 조건으로 사용한다.

가속도는 GNSS 수평 속력의 0.5 s 차분이다. 최종 이벤트 비교 도구는 10 Hz에 재표본화하고
연속 6개 표본이 유효할 때만 가속도를 계산한다. 단순히 50 Hz ROS 발행 횟수를 독립적인
CAN 측정 횟수로 보지 않는다. 차체 가속 추정에는 GNSS가 우선이며 RPM 환산 속도는 대조용이다.
초기 탐색의 보간 통계와 최종 10 Hz 표의 수치는 약간 다를 수 있다.

기존 `/drive/status.commanded_phase_current_a=480 A`는 **Speed mode current limit**이다.
다음 표는 실제 `phase_current_a`다. 모든 선정 구간의 driver feedback mode는 SPEED(1)였다.
기존 기록에는 새 PI의 상태가 없으며 조향 live calibration도 현재 YAML에서 역추정하지 않았다.

## 2. Native Speed Control baseline

첫 positive AUTO command를 t=0으로 둔다. v1p3/v3/v4는 0–15 s,
v6/v8p3/v7는 0–10.5 s 창이다. 일부 run은 AUTO 진입 시 목표가 이미 약 3 m/s다.
따라서 이 표는 완전한 동일 0→target step 시험이 아니다. 향후에는 기록된 명령 파형을 맞춘다.

| Run 목표 m/s | GNSS 10–90% s | 목표 첫 통과 s | 최고 GNSS m/s | 실제 상전류 median / p95 / peak A | GNSS 0.5 s 가속도 peak m/s² |
|---|---:|---:|---:|---:|---:|
| 1.3 | 4.8 | 7.1 | 1.701 | 74 / 107 / 110 | 0.299 |
| 3 | 4.2 | 5.7 | 4.038 | 107 / 188 / 206 | 0.703 |
| 4 | 4.0 | 5.6 | 5.449 | 128 / 254 / 308 | 0.944 |
| **6** | **4.5** | **6.1** | **8.136** | **234 / 369 / 530** | **1.397** |
| 8.3 | 4.9 | 7.1 | 10.646 | 287 / 462 / 504 | 1.663 |
| 7 | 4.5 | 6.4 | 9.293 | 247 / 411 / 530 | 1.573 |

v6 첫 창 평균 가속도는 약 0.791 m/s²(plateau 포함), 10–90% 평균은 약 1.07 m/s²다.
±0.2 m/s를 나머지 창 동안 2 s 이상 유지하는 settling은 모두 관측되지 않았다.
`null`은 창 내 미정착/관측 종료이며 무한대나 0초로 해석하지 않는다.

전체 brake-off AUTO v6의 실제 phase current median/p95/peak는 112/255/530 A,
bus current min/median/max는 −310/26/286 A였다. 480 A 한도 명령보다 큰 순간
측정값은 실제 current-limit tracking/계측을 시험할 이유이며 한도를 올릴 근거가 아니다.
음의 bus current가 존재하므로 기존 speed mode에서 회생이 전혀 없었다고 주장하지 않는다.

### v6 감속과 재인가

- KST **16:07:50.553**, 목표 6 m/s에 GNSS **8.142 m/s**: +2.142 m/s, +35.7%.
- t=6–10 s에서 이미 target을 넘는데 실제 상전류 중앙값은 약 **253→73 A**로
  몇 초에 걸쳐 감소했다. 목표 하향 뒤에야 0 A/음의 전류 구간이 나타난다.
- t=11–20 s 창에서 목표 3.2 m/s 하향 후 최저 GNSS는 재표본화 기준 약 **0.805 m/s**,
  원본 이전 대조는 16:07:55.653의 **0.786 m/s**였다. undershoot 약 **2.4 m/s**.
- 목표 하향 창의 actual phase current min/median/max는 −256/87/264 A,
  bus current min/median/max는 −122/11/239 A. actual |phase|<5 A는 약 0.4 s.
- t=15–19 s, 느려진 뒤 실제 current가 약 **73→149 A**, GNSS 약 **1.04→2.30 m/s**로
  재상승했다. brake 명령/상태는 해제돼 있었다. 전류 limit 480 A는 이 동안 그대로다.
- t=30–42 s 재가속 창은 10–90% 약 **4.2 s**, 목표 첫 통과 **5.9 s**, 최고 **7.094 m/s**,
  phase median/p95/peak **142/187/207 A**. 앞선 가속과 출발속도·명령 파형이 다르다.
- 감속 창의 |RPM 속도−GNSS| p95는 약 **3.12 m/s**, 최대 **4.89 m/s**다.
  전체 v6 p95는 약 1.07 m/s다. 급격한 RPM 변화를 실제 차체 가속도로 사용하지 않는다.

토크 재인가 시점은 보이지만 독립적인 chain shock 센서나 동기화한 소리/event annotation이
없어 충격 크기를 수치로 확정하지 못한다. 새 PI도 RPM 입력을 쓰므로 **fresh하다는 사실이
물리적으로 정확한 RPM임을 보장하지 않는다.** 큰 RPM/GNSS 불일치는 airborne와 ground에서
원본 CAN·독립 회전 측정으로 별도 확인해야 한다. 새 임의 spike 필터를 추가하지 않았다.

## 3. Plant 추정과 candidate

직선 명령 |steering|<3°, |RPM speed−GNSS|<0.5 m/s, 실제 phase>20 A,
GNSS acceleration>0.2 m/s²인 연속 창만 사용했다. 첫 가속의 0.5 s 평균 current와
가속도를 선형 회귀하면 v3/v4/v6/v8p3/v7에서 대략:

`a ≈ (0.0042~0.0046) × I_phase − (0.15~0.27) [m/s²]`

v6은 `a≈0.00438 I−0.256`, net a/I 중앙값 약 **0.00342 (m/s²)/A**였다.
전체 v6 직선 탐색 중앙값은 약 0.00289였다. 이 차이는 저속/고속·재가속 조건과 시간 정렬의
영향이다. rolling resistance, SOC, 도로 경사, 속도, EZ 내부 제한과 전류 응답 지연을 분리한
시스템 동정이 아니다. 10 Hz 창도 서로 겹치므로 회귀 표본을 독립 실험으로 취급하지 않는다.

| 항목 | driving candidate | airborne candidate | 첫 ground 조정 범위/근거 |
|---|---:|---:|---|
| control_mode | torque | torque | Speed A/B는 같은 프리셋에서 speed 선택 |
| Kp A/(m/s) | **180** | **8** | 120–260; gain≈0.0044에서 P-only 시정수≈1.26 s, error 2 m/s에서 360 A |
| Ki A/(m/s·s) | **12** | **1** | 4–30; Kp/Ki=15 s, 작은 적분으로 시작. MCAP에서 정확히 동정한 값 아님 |
| max drive A | **480** | **8** | 기존 command capability 유지; airborne 한도를 driving에 사용하지 않음 |
| integrator limit A | **120** | **3** | 60–160 후보; 지속 drag 보상, overspeed debt 제거 |
| slew up A/s | **2400** | **40** | 1600–4800; 480 A까지 이상적 0.2 s, 정상 full acceleration을 느리게 제한하지 않음 |
| slew down A/s | **4800** | **160** | 2400–9600; 480→0 A를 0.1 s에 감축 가능 |
| deadband m/s | **0.05** | **0.05** | 0.03–0.08; 적분 noise 억제 |
| coast release hysteresis m/s | **0.12** | **0.12** | 0.08–0.18, deadband보다 커야 함 |
| reapply low-current band A | **20** | **2** | coast 뒤 이 구간만 별도 상승률 |
| reapply slew A/s | **200** | **10** | driving 첫 20 A 약 0.1 s; 이후 fast slew |
| Torque motor RPM ceiling | **4500** | **300** | airborne 약 1.6 m/s 환산, PI target도 ceiling 이하로 clamp |

Kp/한도/빠른 slew는 native의 빠른 가속을 보존하기 위한 선택이다. 성능 동등성을
실측으로 입증한 숫자는 아니다. 매우 낮은 airborne 전류는 기능 확인용이며 공중 가속도로
차량 성능을 평가하지 않는다. 실제 가속이 기준보다 느려지면 matched condition에서
current tracking, target ramp, saturation, P/I를 확인하고 이 범위에서 한 변수씩 조정한다.

## 4. 제어 구조와 reset

`Decision speed → Governor validity → STM_A StartControl (20 Hz) → EZkontrol`

- `speed`: 기존 m/s→wheel RPM→motor RPM, current field=480 A limit, bit1=1.
  부호 있는 AUTO reverse도 그대로다. 이 경로에는 새 PI/current limit을 섞지 않는다.
- `torque`: `e=target−measured`, `P=Kp e`, `I_next=clamp(I+Ki e dt,0,Imax)`.
  error deadband 안에서는 I 증가/감소를 멈추고 기존 I를 유지한다.
  upper/lower output clamp 및 slew-up saturation을 더 악화하는 적분은 하지 않는다.
- `e < −deadband`이면 coast latch, I=0, 출력 목표=0. `e > hysteresis`일 때
  latch를 해제하고 low-current reapplication slew를 사용한다.
- 출력은 **0…max drive A**. target=0, brake, drive stop/E-stop, Host timeout,
  MANUAL, Torque 비활성, invalid/stale feedback은 PI history를 reset한다.
  정상의 작은 target 변화만으로 reset하지 않는다. 안전 정지에는 slew를 적용하지 않는다.
- Part I **및 Part II** 수신 age<기존 **200 ms**를 요구한다. stale이면 recoverable
  HALTED/inhibit. feedback 복구를 Control task가 확인한 후 받은 새 Host command가 있어야
  재개한다. 기존 Host 500 ms/보드 간 100 ms timeout은 늘리지 않았다.
- Torque reverse target 또는 후진 속도<−0.05 m/s는 HALTED/REVERSE inhibit.
  implicit Speed fallback은 하지 않는다. AUTO 방향은 기존처럼 Host speed 부호를 따른다.
- CAN current sign, mode bit 및 RUN bit 계약은 기존과 같다. 2022 MCU-to-VCU 표는
  bit1=0 Torque/1 Speed이고 2023 Instruction p5는 bit1 불필요라고 한다.
  기존 실차 bit1 계약을 유지하고 실제 driver mode는 `/drive/status` Part II로 관측한다.
- Torque RPM은 Instruction p5의 max-speed 의미와 기존 MANUAL 계약에 맞춰 사용한다.
  EZ 내부 speed ceiling 동작과 0 A coast에서 실제 bus current는 실기에서 확인해야 한다.
  EBS enable, negative/regen current, battery charge control은 추가하지 않았다.

`Core/Inc/longitudinal_controller.h`, `Core/Src/longitudinal_controller.c`는 HAL/RTOS 없이
host에서 컴파일한다. 새 task는 없다. steering fault/recovery 및 STM_B 코드는 변경하지 않았다.

## 5. Runtime calibration, preset, rollout

[정확한 wire 계약](../INTERFACES.md#longitudinal-calibration-v1)을 따른다.
이미 정적 micro-ROS library에 있는 `std_msgs/Float32MultiArray`를 사용하므로
kaiev26_msgs/DriveActuatorStatus schema와 STM_B firmware rollout은 필요하지 않다.
fixed order/version, bounded storage를 사용하며 동적 string/array 할당을 추가하지 않는다.

`control_tuning → /drive/longitudinal_config → uROS mailbox → 다음 StartControl → RAM`

STM_A가 MANUAL, fresh Part I/II, |speed|≤0.05 m/s, pedal=0,
|commanded current|<0.1 A, |actual phase current|≤1 A를 확인한다. MANUAL은 pedal=0에도
RUN을 전송하므로 RUN bit만으로 traction active를 판단하지 않는다.
mode와 모든 gains는 이 gate를 동일하게 통과해야 한다. revision compare-and-swap과
critical section이 concurrent host 변경/torn config를 막는다. unsafe 요청은 저장해 두지 않고
거절한다. 실행 중 자동 적용 대기열이 생기지 않는다.

Boot는 `DEFAULT`, revision=0, **speed**와 driving candidate gains다. Host가 적용하면
`HOST_OVERRIDE`. Agent reconnect만으로 RAM을 덮어쓰지 않으며 명령은 기존처럼 reset한다.
MCU reboot는 DEFAULT로 돌아간다. launch의 calibration helper는 status/request/revision/full config의 실제 wire 값을
비교한다. readback timeout이면 적용 여부 미확인으로 보고 자동 재전송하지 않는다.

후속 이름 정리에서 preset schema v3의 `steering`, `drive`, `governor`, `timeouts`로 전환했다. [파라미터 연결·호환표](../CONTROL_TUNING_PARAMETERS.md)를 따른다. 기존 v2는 읽을 수 있고 S 저장 시 v3로 변환한다.
checked-in 프리셋은 **airborne/driving 둘**이며 lateral 수치는 작업 시작값을 그대로 옮겼다.
옛 v1은 오류를 내며 종방향 값을 숨겨 채우지 않는다. 사용자 파일은 기존 values를 네 section에
배치하고 mode/모든 구동 값을 명시해 v3로 바꿔야 한다. checked-in 두 파일을 양식으로
쓰되 사용자 조향값을 덮어쓰지 않는다. 접지 정지는 `stationary` 시험 annotation이며 preset이 아니다.

TUI는 80열 이상에서 `1 조향응답 · 2 조향출력 · 3 구동응답 · 4 구동출력 ·
5 수신제한`을 폭에 맞춰 간격을 두고 한 행에 표시하며 `1–5`/←→로 고른다.
조향 제어 방식은 조향응답, 최소 허용 차속은 조향출력, 상위 명령 유효시간은 수신제한에 있다.
후속 UI 변경으로 A 즉시 적용 기능을 제거했다. TUI는 현재값 조회·편집·S 파일 저장만 하며,
선택한 프리셋으로 구동 스택을 재시작할 때 반영한다. MCU RAM calibration 계약은 유지한다.

`control_launch vehicle.launch.py`는 선택한 longitudinal preset을 별도 one-shot helper로
적용한다. 15 s 안에 safe state/readback을 확보하지 못하면 오류를 표시하고 종료한다.
이때 launch를 시작했다는 이유로 적용됐다고 보지 말고 Console/TUI readback을 확인한다.
예전 STM_A는 status가 없으므로 적용 실패로 드러난다. 실차 launch는 이번 작업에서 실행하지 않았다.

Decision에는 accel 1 m/s², decel 2 m/s², signal-release accel 2 m/s²의 ramp가 있다.
Governor는 기존 validity/gating만 담당하도록 유지한다. PI는 속도 오차→전류,
current slew는 전류 변화/재인가 충격, EZ는 전류 내측루프/최대 RPM을 담당한다.
추가 governor accel/decel/jerk ramp로 double filtering을 만들지 않았다.

## 6. 관측과 녹화

5 Hz `/drive/longitudinal_status`는 전체 적용 calibration, source/revision/result,
error/P/I/unsaturated/final current, saturation/anti-windup/coast/feedback/inhibit를 발행한다.
27 float=108 bytes payload, CDR/layout overhead 포함 약 120 bytes/sample이다.
기존 460800 baud 링크에서 약 600 bytes/s + XRCE framing이며 실제 link 부하는 실기 확인 대상이다.
`/drive/status`의 실제 phase/bus current, mode, RPM/current command는 그대로 사용한다.
Console Drive 화면에서 두 모드·출처·PI 상태와 실제 current를 함께 볼 수 있다.

Control의 `mcap_logging.launch.py`와 `vehicle_test_scenarios` recorder에 두 새 topic을 넣었다.
**별도 Decision 저장소의 자체 recorder 목록에는 아직 이 topic이 없다.** 이 A/B 시험은
Control scenario recorder를 사용하거나 Control MCAP logger를 함께 실행해야 한다.
Decision의 기존 데이터/명령 계획·브랜치는 이번 작업에서 변경하지 않았다.

분석 도구는 같은 방법으로 기존/새 MCAP을 읽는다. `--start/end`는 첫 positive AUTO
command 기준 초이며, low→high 또는 high→low의 명목 target을 `--target`으로 지정한다.
음의 step에서는 `target_overshoot_mps`가 하향 목표 아래의 undershoot 크기다.
coast=actual |phase|<5 A, reapplication=coast에서 actual>20 A로 넘어가는 횟수이며
음향 충격의 대용 측정값이 아니다. GNSS가 없으면 RPM 속도를 사용했다고 출력한다.

```bash
cd ~/KAI_ws
si
python3 -m control_tuning.longitudinal_analysis \
  records/260919/quali_succeed_run/20260919_160206_275928_decision_case_01_pure_pursuit_v6/rosbag \
  --start 0 --end 10.5 --target 6
```

외장 artifact root가 이번 환경에는 없어 대용량 CSV/새 bag을 만들지 않았다.
분석은 필요한 저용량 신호와 메모리/임시 캐시로 수행했고 숫자·방법은 이 문서에 남긴다.

## 7. 첫 airborne 시험 — 운영자가 수행

1. 차체를 확실히 고정하고 구동륜을 띄운다. E-stop/servo/주변 접근을 확인한다.
   MANUAL, pedal released, speed=0, actual/commanded current≈0에서 시작한다.
2. 빌드된 STM_A를 식별해서 flash한 뒤 status/Part I/II와 통신 복귀만 먼저 확인한다.
   Console default speed/source DEFAULT를 확인한다.
3. `tuning_preset:=airborne`로 구동 stack을 실행한다. TUI 구동응답·구동출력에서
   Torque/8 A/Kp8/Ki1/RPM300 및 HOST_OVERRIDE readback을 확인한다.
4. Control scenario recorder로 `/drive/status`, command/brake, longitudinal status를 기록한다.
   첫 낮은 목표 0.3→0.6 m/s에서 방향·RPM·feedback·P/I·실제 current를 확인한다.
   이 작업을 수행한 agent는 movement command를 발행하지 않았다.
5. target 0이면 current=0/HALTED, brake/E-stop/host timeout이면 같은 정지 결과인지 확인한다.
   MANUAL 복귀 시 PI reset을 확인한다. MANUAL pedal은 기존 기능이므로 계속 해제해 둔다.
6. 제어된 bench 조건에서 feedback loss→HALTED→feedback 복구→새 command 이후 재개를 확인한다.
   실제 drive status mode가 Torque인지, 0 current에서 bus current/바퀴가 어떻게 반응하는지 확인한다.
7. MANUAL 정지·무전류에서 mode=speed로 적용하고 fallback을 확인한다.
   **Speed fallback은 기존 480 A limit/원래 RPM 경로**다. airborne 8 A/RPM300 설정은
   Torque 경로 전용이므로 Speed 시험은 별도의 작은 목표/짧은 시간으로 수행한다.

## 8. 첫 driving A/B와 최종 freeze

동일 노면/방향/적재/SOC/온도, 같은 Decision/scenario target 파형으로 A=Speed,
B=Torque를 번갈아 비교한다. 설정 전환은 매번 MANUAL 정지·무전류에서 한다.
B는 driving candidate, regen/EBS 변경 없이 positive traction+coast다.
첫 기능 확인 뒤 운영자가 확보한 직선 공간에서 `0→20→10→20 km/h` 또는
v6 기록과 일치하는 목표/hold를 사용한다. HOLD가 부족하면 settling 미관측으로 남긴다.

- 10–90%, target crossing, peak/mean GNSS acceleration, overshoot, 하향 undershoot,
  settling ±0.2 m/s/2 s, RMS error를 같은 창에서 비교한다.
- phase/bus current peak·median/RMS, actual current reduction/reapplication,
  command vs actual current, RPM vs GNSS, saturation/anti-windup을 확인한다.
- chain 소리/충격 시각을 GNSS/MCAP 시각과 동기화해 운영자가 annotation한다.
- 가속 성능 동등성은 반복 시험의 run-to-run 분산과 함께 판단한다. 반복 변화보다 명백한
  rise time 악화/peak acceleration 감소는 개선 완료로 승인하지 않는다. 그 경우 current
  tracking과 target ramp부터 확인하고 Kp/current/slew를 하나씩 조정한다.
- no-regen coast는 native speed braking보다 감속거리가 길 수 있다. PI만으로 모든 조건에서
  undershoot/overshoot/코너 진입을 동시에 개선했다고 가정하지 않는다. 감속 시작 위치는
  실제 차속/가용 감속도를 바탕으로 별도 planning 시험에서 평가한다.

대회 전 확정값은 동등 조건 A/B MCAP과 환경·실제 readback을 함께 보관하고, 검증된 driving
전체 config를 `Long_DefaultConfig()`에 동결한다. 그때만 default mode=torque 선택을 검토한다.
같은 값으로 host tests/Release build를 실행하고 STM_A identity/안전 상태를 확인해 flash한다.
PC override 없이 DEFAULT readback과 성능을 다시 확인한다. Host reconnect로 calibration이
바뀌지 않는지, MCU reboot 때 frozen default가 복원되는지 확인한다.

## 9. 검증 및 배포 결과

소프트웨어/firmware 검사 결과는 다음과 같다. 실기 통신·전류·주행 성능은
보드 미연결로 이번에 확인하지 않았다. `STM32_Programmer_CLI -l`은 **No ST-Link detected**,
DFU device 없음이었고 `/dev/stm_a`도 없었다. **STM_A/STM_B 모두 flash하지 않았다.**


- Host pure C tests: PI zero/positive error, upper/lower saturation, no negative integral debt,
  integral retention/reset, fast up/down slew, coast/hysteresis/reapplication, feedback-loss/new-command
  recovery, stop/disabled/reverse gates, finite/range rejection, atomic full-config commit/revision conflict,
  duplicate request, safe calibration gate 통과. 기존 CAN encoder를 공통 inline 함수로 분리해
  Speed/Torque mode bit, signed reverse, 480 A field, HALTED/life byte도 직접 검사했다.
- Control Tuning + vehicle_test_scenarios Python tests: 실제 CAN/output-disabled launch 통합 포함
  **246 passed**. 추가 longitudinal launch 설정 전달 2건도 통과했다 (후속 targeted run: 28 passed, 기존 full launch 항목 1 skipped).
- Console Vehicle API/runtime health: **36 passed**. 실제 DDS→WebSocket에서
  Torque mode, HOST_OVERRIDE, P/I readback 및 stale/reconnect 동작을 확인했다.
- 기존 lateral_ctrl **119**, command_governor **12** tests, 오류/실패 0.
- Console TypeScript/Vite production build 및 웹 domain tests **4 passed**.
- ROS packages control_tuning, lateral_ctrl, command_governor, control_launch, vehicle_console,
  vehicle_test_scenarios 빌드 완료. 기존 mixed copy/symlink 설치의 colcon asset 오류는
  Console 일반 설치 빌드로 해결했다. 제품 코드에 별도 설치 우회 로직을 추가하지 않았다.
- STM_A Release clean build 성공 후 CAN encoder 추출까지 incremental rebuild 성공.
  **FLASH 116248 B / 512 KiB (22.17%), RAM 118072 B / 128 KiB (90.08%)**.
  `_gettimeofday is not implemented` newlib 경고는 기존과 동일하며 링크는 성공했다.
  보드에서 micro-ROS entity 생성, free heap/stack margin, serial 부하·실제 CAN 송신은 미검증이다.
- 제공한 분석 entrypoint로 원본 v6 bag을 다시 읽어 표의 4.5 s rise / 6.1 s arrival /
  1.397 m/s² peak / plant fit을 재현했다. 원본을 재생하거나 차량에 publish하지 않았다.

변경 중심: STM_A PI/RTOS 연결/CAN encoder, control_tuning schema/preset/runtime/TUI/analysis,
control_launch helper, Console telemetry/UI, Control record topic 목록, scenario condition 분리,
관련 tests/README/INTERFACES/이 보고서. 변경 전체 목록은 해당 commit의 file list로 확인한다.
