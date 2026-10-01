# 회생제동 도입과 거버너 slew 제거 — 2026-10-02

[회생·거버너 지침](../EZKONTROL_REGEN_AND_GOVERNOR_REWORK.md)의 Part A, B1, B2, B3와 4장 운용 항목을
구현한 기록이다. STM_A longitudinal ABI는 v4 → v5다.

**회생은 기본으로 꺼져 있다(`regen_max_current_a` 0). 코드는 host 단위 테스트와 STM_A·STM_TEST ARM 빌드까지만
확인했다. STM_A를 flash하지 않았고 HIL·공중·지상 회생 시험(R1·R2)은 하지 않았다.** 2026-10-01 시험에 쓰인
STM_A는 v4이므로 새 host는 v5 STM_A와 함께 올려야 프리셋이 적용된다.

## 0. 요약

| 지침 | 구현 |
|---|---|
| Part A | `command_governor`의 `SpeedSlew`와 두 파라미터 삭제. 거버너는 검증·timeout·재발행만 하고 brake면 속도 0. launch(vehicle·HIL)의 slew 전달, Control Tuning 항목, 프리셋 키 삭제. 옛 프리셋·설정 파일의 키는 오류 없이 무시하고 TUI가 알린 뒤 S 저장 시 지운다 |
| B1-1 | 회생 7개 필드 + RPM 필드 모드를 calibration v5로(`AUTO_REGEN_MAX_A`·`AUTO_REGEN_SLEW_A_PER_S` 삭제). v4 이하 요청은 회생 꺼짐 기본값으로 받는다 |
| B1-2/3 | `Long_RegenFrame()`: 한도 = 최대 × 속도 계수 × 전압 계수 × 충전전류 보호. 한도는 즉시, 요청 변화와 허용 조건 상실은 `regen_slew`. Part I 단절이면 한도 0. 안전 정지·MANUAL·Speed는 즉시 0 |
| B1-4 | `regen_rpm_field_mode` 0(RPM 0, 기존)·1(RPM 한도를 양수 상한) |
| B1-5 | status v5에 요청·한도·속도/전압 계수·충전 보호 배율·최종 회생 전류·회생 경로·PI 회생 모드. `longitudinal_analysis`에 `regen` 요약, 콘솔 표시 |
| B1-6 | 조이스틱 `regen_max_a`는 이미 launch 인자. README에 최종 상한이 STM_A `regen_max_current_a`임을 적음 |
| B2 | `pi_regen_enabled`(기본 0): 진입 과속 0.3, 해제 0.1 m/s, 회생 P 게인(기본 0). 진입은 구동 출력이 0 A일 때만, 회생 중 PI 적분·모드 동결, 회생이 0 A로 돌아오는 프레임까지 회생 경로 → 부호가 한 프레임에 바뀌지 않음. 비활성이면 구동 출력이 비트 단위로 같다(테스트) |
| B3 | 붕괴 감지를 크기 기준(직전·최신 명령 중 큰 값)으로, RPM 허용폭 max(200, 3 rpm/A × 명령), 음수 상전류 임계는 파형별 설정(방형파 −30, 사인파 −60 A)이며 최신·직전 명령 중 하나라도 회생(음수)이면 적용하지 않음(회생이 0 A로 끊긴 다음 프레임 포함) |
| 4장 | 시나리오 TUI가 구동 Run 시작 시 STM_A readback을 런치 프리셋과 비교해 다르면 거부. airborne 적분 한도 25 A, control_tuning README 표를 실제 프리셋 값으로 갱신 |
| 5장 HIL | STM_TEST plant가 회생 명령(음수 전류 + 양수 RPM 상한)에서 0 rpm으로 감속 |

## 1. 사인파 기록으로 본 붕괴 감지 (B3)

2026-10-01 22:55~23:46 공중 speed_step 12개(EZ 사인파, STM_A v4). 앞의 4개는 STM_A 재부팅 뒤 프리셋이 다시
적용되지 않아 DEFAULT(480 A, Kp 180, 2400/4800 A/s)였고(readback source DEFAULT, revision 0), 나머지는 airborne 프리셋(80 A, Kp 15)에서 적분 한도를 6 → 10 →
13 → 15 A로 올린 기록이다.

v4 규칙(phase < −30 A, RPM 200 rpm/프레임)은 사인파 기록에서 13개 구간을 잡았다.

- 0 A 명령 중 FOC 전류 잡음: 1400~2600 rpm에서 −32 ~ −51 A(사인파는 0 A에서도 브리지를 유지한다).
- 400 A대 가속: 사인파 공중 정상 가속은 상전류 1 A당 프레임당 1.7~2.2 rpm(최대 2.8)이고 450 A에서 600 rpm까지 나왔다.
- 빠른 전류 램프(2400 A/s)에서 측정 상전류가 명령보다 한 프레임 늦어 phase-high.

v5 규칙에서는 4개 구간으로 줄었다. 방형파 붕괴 검출은 48/49로 그대로이고, 방형파 정상 구간 오검출은 1프레임이다.

| 남은 구간 | 판단 |
|---|---|
| 23:11 run 17.2 s, 26.9~27.8 s | DEFAULT 고이득에서 RPM이 프레임마다 ±100~200 rpm 흔들림(0 A coasting은 프레임당 약 9 rpm). 27.8 s에는 RPM 0과 상전류 166 A 뒤 Part I이 약 150 ms 끊김 → 실제 이상, 잡아야 함 |
| 23:03 run 11.2 s, 23:11 run 9.6 s | 정지 출발 램프. `/drive/status`의 명령값이 송신보다 최대 20 ms 늦게 기록돼 생긴 오프라인 평가의 착시. 펌웨어는 수신 시점의 최신 송신 명령과 비교한다 |

사인파 −60 A와 관측 최저 −51 A의 여유는 9 A로 작다. 더 높은 속도에서 잡음이 커지면 프리셋의
`drive_collapse_phase_low_sine_a`를 조정한다.

## 2. 회생 동작

```
request = max(LT/외부 요청, 속도 PI 요청)
limit   = regen_max × speed_factor × voltage_factor × bus_scale
          speed_factor: v ≤ 0.8 → 0, 0.8~1.5 m/s 선형, ≥ 1.5 → 1
          voltage_factor: V ≤ 57 → 1, 57~58 V 선형, ≥ 58 → 0
          bus_scale: 충전전류 > 30 A인 프레임마다 × 0.8, 회복은 regen_slew 속도
target  = allowed ? −min(request, limit) : 0
current = slew(target, regen_slew), 그다음 [−limit, 0]로 즉시 제한
```

- 한도가 즉시 적용되므로 차단 속도 이하에서는 같은 프레임에 0 A다(테스트로 고정). 허용 조건(AUTO 전류 경로,
  inhibit OK, Part I/II 신선, fault 0, EZ torque 모드)이 깨지면 회생 변화율로 0에 돌아온다.
- 회생 경로가 0 A로 돌아오는 프레임도 회생이 차지한다. 그다음 프레임부터 구동 PI가 0 A에서 재인가 구간을
  거쳐 올라간다. LT 회생 중에는 이전처럼 PI를 지우지 않고 동결해 적분을 유지한다.
- 외부 전류 모드에서 음수 명령은 회생 요청이고 구동 입력은 0 A로 신선하게 유지한다. 양수 명령은 회생 요청을
  0으로 되돌려 이전 음수가 500 ms 동안 남지 않게 한다.
- B2 폐루프 시험(공중 모형, 30 → 10 km/h)에서 부호 반전 프레임 0, 회생 구간 적분 불변, 목표 ±0.5 m/s 정착.

## 3. 직접 해야 할 시험과 적용

모두 STM_A v5 flash와 host 갱신 후, 한 번에 한 변수만 바꾼다. 판정에는 status 70~77(회생), `longitudinal_analysis`의
`regen`·`collapse` 요약, `/drive/status`의 bus 전류·전압을 쓴다.

**R1 공중 LT 회생 (지침 1.3 미확인 사항 판정)**
- 프리셋 airborne, `회생 최대 전류` 5 A, 조이스틱 `regen_max_a:=5`. 20 km/h 정속에서 LT 1~2 s, 5회. 이어서 10, 20 A.
- 판정: 경로 5(회생), 상전류가 음수 명령을 추종, **bus 전류 음수**, 감속도가 coasting(약 1 m/s²)보다 큼,
  전압이 57 V 미만, 붕괴·fault 0.
- bus 전류가 음수가 아니거나 상전류가 0 근처면 EZ가 이 조합을 회생으로 실행하지 않는 것이다. 그때 `회생 RPM 필드`를
  ceiling으로 바꿔 한 번 더 확인하고, 둘 다 안 되면 EZ EBS·회생 설정(EBS auto는 Disable 유지)을 먼저 점검한다.

**R2 공중 차단 확인**
- 회생을 유지한 채 감속시켜 차단 속도(0.8 m/s) 아래까지 내려간다. 회생 전류가 같은 프레임에 0 A가 되고
  역회전(음수 속도)·HALTED가 없는지 본다. `회생 RPM 필드` zero/ceiling을 각각 확인한다.

**B2 PI 자동 회생 (R1·R2 통과 후)**
- `속도 PI 자동 회생` enabled, `PI 회생 P 게인` 10에서 시작, speed_step 10↔30 km/h. 하강 단계의 목표 첫 통과
  시간, 오버슛, 회생 구간 감속도를 coasting과 비교하며 게인을 올린다. 진입/해제 과속(0.3/0.1 m/s)은 그다음 변수다.

**지상**
- R1·R2를 반복해 감속도·bus 충전전류·전압을 기록하고 회생 최대 전류와 충전전류 한도 기본값을 정한다.

## 4. 검증

- STM_A host 테스트: 기존 회귀 + 회생 한도·감소·차단·전압·충전 보호·slew·안전 리셋, PI 회생 히스테리시스·
  폐루프 부호 연속·적분 동결·비활성 비트 동일, 붕괴 v5(방형파·사인파·회생 사례), v5 범위·v4 호환.
- STM_TEST host 테스트(회생 plant), STM_A·STM_TEST Release ARM 빌드.
- command_governor colcon gtest, control_tuning·control_tools·vehicle_test_scenarios pytest(격리 ROS 도메인).
- 미실행: STM_A flash, HIL, R1·R2·B2 공중·지상 시험, vehicle_console pytest(fastapi 미설치).
