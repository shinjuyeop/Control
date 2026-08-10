# 스티어링 모터 데드존(Breakaway Torque) 측정

KEYA KY170DJD01005-08G-V1 스티어링 모터를 Torque control 모드로 두고, "얼마나 작은 torque 명령부터
휠이 실제로 움직이기 시작하는지"(정지 마찰 breakaway 문턱)를 측정한 기록.

## 사용 노드

- `steering_actuator_node`(읽기 전용) — `hardware_can_enabled:=true`,
  `actuator_output_enabled:=false`로 띄우면 버스를 열어 조회 프레임만 보내고 `/steering/status`를
  발행한다. enable/disable/position 프레임은 한 장도 나가지 않으므로 아래 측정 노드의 torque 명령과
  충돌하지 않는다. 측정에 쓰는 필드는 `motor_angle_deg`(legacy 프로토콜의 Encoder count value query
  기반, 10000 count/turn = 0.036°/count, 중앙 등가각 calibration을 거치지 않은 모터축
  원시 각도), `motor_current_a`(드라이버 실측 전류, A), 값이 살아 있는지 보는 `feedback_valid`.

- `deadzone_measure_node` — torque cmd를 0부터 계단식으로 올리면서, 각 단계를 `step_hold_ms`만큼
  유지한 뒤 `motor_angle_deg`가 그 시작 시점 대비 조금이라도 달라졌는지 확인. 안 바뀌면 다음 단계로,
  바뀌면 그 순간의 torque cmd와 **그 순간의 실측 전류**를 데드존 판정값으로 로그에 남기고 정지 +
  disable. 구독 토픽은 `status_topic`(기본 `/steering/status`) 하나다.

이전에는 `steering_feedback_node`가 `/ctrl/steer` 계열 토픽으로 같은 값을 따로 발행했다. 이 노드는
읽기 전용 모드가 같은 일을 하게 되면서 삭제했다. 아래 기록에 나오는 `/ctrl/*` 토픽 이름은 당시
그대로 남겨 둔 것이다.

### 모터축 원시 각도를 기준으로 삼기로 함

기계팀에서 받은 캘리브레이션 다항식(핸들각→wheel각 변환)이 정확한지 검증이 안 된 상태라,
**인지/판단 파트에는 이 다항식을 거치지 않은 모터축 원시 각도를 기준으로 전달하기로 함.** 데드존
측정도 이 값을 기준으로 봐야 다항식 오차와 무관하게 순수 모터/기구 특성만 볼 수 있음.

`motor_angle_deg`는 raw count와 같은 모터축 부호를 그대로 쓴다. 아래 "부호 규약 확인(ISO8855)"은
부호를 반전한 옛 `/ctrl/steer_handle_hires` 기준으로 적은 기록이라, 같은 실험을 `motor_angle_deg`로
다시 보면 `ramp_direction`과 각도 증감의 부호 관계가 반대로 나온다. 방향이 바뀐 게 아니라 보고 있는
축이 다른 것이다.

## 실행 방법

```bash
# 터미널1 — 읽기 전용. 조회 프레임만 나가고 모터는 건드리지 않는다
ros2 launch lateral_ctrl steering_actuator.launch.py \
    can_interface:=can0 actuator_output_enabled:=false

# 터미널2 (노드는 한 번만 띄우고 계속 재사용)
ros2 run lateral_ctrl deadzone_measure_node can0 --ros-args \
    -p step_hold_ms:=300 -p ramp_direction:=1

# 터미널3 — 반복 측정 시 노드 재시작 없이 이걸로 재실행
ros2 topic pub /deadzone/start std_msgs/msg/UInt8 "{data: 1}" --once

# 방향 전환 (재시작 불필요, torque cmd는 0으로 안전 리셋됨)
ros2 param set /deadzone_measure_node ramp_direction -1
```

## 판정 방식

- 인코더 카운트가 정수라서 안 움직이면 각도값이 완전히 동일하게 나온다는 점을 이용 — threshold 없이
  "값이 조금이라도 달라졌는가"만으로 판정. 지금 기준인 `motor_angle_deg`는 0.036°/count다
- `ramp_period_ms`(20ms, CAN watchdog keep-alive 겸용)와 `step_hold_ms`(300ms, 판정 대기 시간)는
  분리되어 있음

## 이번 세션에서 고친 버그

1. **`steering_feedback_node.cpp` — fault_code 오독**
   Err_H/Err_L(16bit)을 0~16 사이 단일 열거값(LED 깜빡임 횟수 표, 데이터시트 4.2.1절)으로 잘못 해석해서
   `fault_code >= 2`면 무조건 경고를 찍고 있었음. 실제로는 데이터시트 5.2절 표에 따른 **비트마스크**라
   여러 fault가 동시에 설 수 있음. 비트별로 디코딩하도록 수정(`fault_bit_name()`), 상태 비트(bit0
   Enable/Disable)와 예약 비트(5,13)는 제외하고 실제 fault 비트만 경고. `/ctrl/steer_fault`도 `UInt8`
   → `UInt16`으로 변경(기존엔 상위 바이트가 통째로 잘려서 CAN disconnected 같은 bit8 이상 fault가
   토픽에서 사라지고 있었음).

2. **`deadzone_measure_node.cpp` — enable_motor 재호출 누락**
   `enable_motor()`가 생성자에서 딱 한 번만 호출되고, `/deadzone/start`로 재시작하는 `start_ramp()`에는
   없었음. 그래서 한 번 detect/stop 되어 `disable_motor()`가 호출된 뒤로는 노드를 재시작하지 않는 한
   드라이버가 계속 Disable 상태로 남아 torque 명령이 실제로 안 먹혀서 재측정이 항상 실패했음.
   `start_ramp()` 진입 시 매번 `enable_motor()`를 다시 보내도록 수정.

3. **드라이버 파라미터 0020(Work mode)이 Absolute position control로 되돌아가 있었음**
   증상: `/ctrl/steer_fault`는 0(정상 Enable)인데 torque cmd를 아무리 올려도
   `/ctrl/steer_motor_current`가 전혀 안 움직임 — Enable/Disable과 제어모드는 별개 레지스터라 fault
   비트로는 안 잡힘. 해결: Windows 유틸리티에서 0020=2(Torque control)로 다시 설정 후 **반드시 Burn까지
   눌러 E²ROM에 저장**, 드라이버 전원 재기동(데이터시트 4.1.2.5~4.1.2.9절).

## 이번 세션에서 추가한 기능

- **`/ctrl/steer_handle_raw` 퍼블리셔 추가** (`steering_feedback_node.cpp`) — 캘리브레이션 다항식을
  거치지 않은 raw handle 각도(부호 보정만 적용). 다항식 신뢰도와 무관하게 쓸 수 있는 값.
- **`deadzone_measure_node`에 실측 전류 로깅 추가** — `/ctrl/steer_motor_current`를 구독해서
  DEADZONE DETECTED / LIMIT REACHED 로그에 `measured_current=%.2fA`로 실제 측정된 전류를 같이 남김.
  (아래 "최종 결과" 표의 "환산 전류"는 이 기능을 추가하기 전에 torque cmd 비율로 근사 계산한 값이라
  실측치가 아님 — 재측정 시부터는 로그에 실측 전류가 직접 찍힘)
- **고정밀 위치 query 추가("경로 B") — `/ctrl/steer_handle_hires`** (`steering_feedback_node.cpp`)
  드라이버 제어모드(0019=CAN)는 그대로 두고, 기존 heartbeat(360 count/turn)와 별개로 legacy 프로토콜
  안에 있는 **Encoder count value query**(데이터시트 4.3.2절, `40 04 21 02 ...` → 응답
  `60 04 21 02 <4byte>`, 10000 count/turn)를 50ms마다 추가로 보내서 파싱. CANopen으로 완전히
  갈아타지 않고도 heartbeat 대비 약 27배 촘촘한 위치(count당 0.036°)를 얻음. 실전 검증(손으로 핸들
  돌려서 raw/hires 둘 다 같은 방향으로 반응하는지 확인) 완료. `hires_enabled`(기본 true),
  `hires_query_period_ms`(기본 50) 파라미터로 조절 가능.
- **부호 규약 확인(ISO8855)** — `dir=+1`(양의 torque cmd)로 램프했을 때 각도가 감소(좌측 방향)하고,
  손으로 핸들을 좌측으로 돌렸을 때도 동일하게 감소하는 것을 확인. 즉 `dir=+1` = 좌측 = 음수 방향으로
  일관되게 매핑되고 있어 **부호 반전 불필요**로 결론.
- **`deadzone_measure_node`에 `sweep_mode` 파라미터 추가** — 첫 움직임에서 멈추지 않고 `torque_limit`
  까지 계단식으로 끝까지 올리며 매 스텝 `SWEEP,dir=...,cmd=...,rated_pct=...,current=...,angle=...`
  형태로 CSV 로그를 남김. torque cmd(=전류 setpoint 비율) vs 실측 전류 상관관계식을 뽑아 Gazebo
  시뮬레이션 파라미터로 쓰기 위한 다음 실험용으로 추가함 (아직 실측 미실시, "한계 및 다음 단계" 참고).

## 측정 중 알아낸 특이사항

- **절대 위치가 0점 조정 안 되어 있었음**: 인코더가 17-bit multi-turn absolute encoder라 전원이
  꺼져도 위치를 기억하는데(배터리 필요, 데이터시트 III절), 초기엔 raw_counts가 약 4644(≈12.9바퀴 어긋난
  위치)였음. 이 상태에서 나오는 큰 wheel 각도 값(-5316° 등)은 버그가 아니라 이 위치를 캘리브레이션
  다항식에 그대로 대입한 결과. 실제 주행 전에는 데이터시트 4.3.3.4절 절차로 기계적 0점을 다시 잡아야 함.
- **인코더 분해능이 위치에 따라 다르게 느껴짐**: 판정에 쓰는 wheel 각도는 비선형 다항식으로 변환되기
  때문에, raw count 1칸이 실제로는 위치에 따라 다른 각도폭을 의미함. 어긋난 위치(핸들각 ≈4644°)에서는
  1 count ≈ 2.073°, 0점 근처(핸들각 ≈8°)에서는 1 count ≈ 0.231°로 약 9배 더 촘촘함. 즉 0점 근처에서 잰
  값이 quantization 오차가 작고, 실제 조향 범위와도 더 가까워 더 신뢰도가 높음.
- **모터 disable 상태에서도 위치가 안정 지점으로 살짝 되돌아감**: 케이블/기어 백래시나 프리로드로 보이며,
  노드를 재시작하지 않고 반복하면 시작 위치가 매번 동일해서 결과가 매우 재현성 있게 나옴(재시작하면 CAN
  재연결 등으로 시작 조건이 달라져 결과가 들쭉날쭉했음).
- **모터 전원이 실수로 한 번 뽑혔다 들어오면서 절대 위치 기준이 리셋됨** (기존 4644 → 8 근처). 인코더
  백업 배터리가 없으면 전원이 끊기는 순간 누적 위치가 사라진다는 데이터시트 III절 설명과 일치. 버그
  아님. 이후 측정은 이 새 기준점(핸들각 ≈8°, 0점에 훨씬 가까움)에서 진행.
- **USB-CAN 어댑터가 물리적으로 재연결되면 `can0` 인터페이스가 `state DOWN`으로 초기화됨** —
  ifindex도 바뀜(예: `6: can0` → `5: can0`). `candump`에 아무것도 안 뜨면 배선 문제가 아니라 이거일 수
  있음. `sudo ip link set can0 up type can bitrate 250000`으로 다시 올려줘야 함.
- **전류 텔레메트리는 legacy CAN/RS232/CANopen 어느 프로토콜을 쓰든 정수 A 단위**라는 걸 데이터시트
  전체(4.3.2, 4.4.3, 6.4.5절)에서 확인함. breakaway 근처(1% 내외, 0.1A 수준)의 전류는 어느 프로토콜로
  재도 0으로 찍힘 — CANopen으로 갈아탄다고 해결되는 문제가 아님.

## 최종 결과

### 1차 (raw/wheel 기준, 360 count/turn, 0점 근처 핸들각 ≈8°에서 10회씩 반복)

| 방향 | torque cmd (10회) | 대표값 | % of rated | 환산 전류(정격 10A 기준) |
|---|---|---|---|---|
| 정방향 (dir=+1) | 125, 115, 120, 115×7 | **115** | 1.1~1.2% | ≈0.115A |
| 역방향 (dir=-1) | -140 ×10 (전부 동일) | **-140** | 1.4% | ≈0.14A |

정방향 첫 2회(125, 120)는 방향 전환 직후 위치가 완전히 settle되기 전 영향으로 보이는 이상치.
나머지는 매우 안정적으로 재현됨.

### 2차 (hires 기준, 10000 count/turn — Path B 적용 후, 검증용 단발 측정)

드라이버 제어모드가 Absolute position control로 되돌아가 있던 버그를 고친 뒤 재측정. 정방향(dir=+1)
1회:

```
torque cmd = 110 (1.1% of rated), measured_current=0.00A, dir=+1,
angle 0.612° -> 0.576° (Δ=-0.036°)

torque cmd = -110 (1.1% of rated), measured_current=0.00A, dir=-1,
angle 0.648° -> 0.684° (Δ=0.036°)

torque cmd = -106 (1.1% of rated), measured_current=0.00A, dir=-1,
angle 1.260° -> 1.296° (Δ=0.036°)
```
step_hold_ms 를 늘렸을 때 106으로 값이 변함

터미널 2 : 
ros2 run lateral_ctrl deadzone_measure_node can0 --ros-args       -p angle_topic:=/ctrl/steer_handle_hires       -p ramp_direction:=-1       -p ramp_step:=1       -p step_hold_ms:=8000       -p start_cmd:=101       -p torque_limit:=110


Δ=-0.036°는 360°/10000count = 0.036°/count와 정확히 일치 — **hires 인코더 카운트 정확히 1칸** 단위로
breakaway를 잡은 것으로, 1차(360 count/turn, 0.231°/count) 대비 약 6배 더 정밀한 판정. 1차 결과(115,
-140)와도 같은 자릿수라 정합성 확인됨. 전류가 0.00A로 찍힌 건 측정 오류가 아니라 이 드라이버의 전류
텔레메트리가 정수 A 단위라 0.1A대 실제 전류는 어차피 0으로 반올림되기 때문(위 "측정 중 알아낸
특이사항" 참고).

### 3차 (hires + `step_hold_ms` 늘려서 재검증 — hold time 의존성 확인)

`step_hold_ms=300`(0.3초)에서 감지된 110이 진짜 문턱인지 의심 — 300ms 안에 못 잡은 느린 creep이
있을 수 있다고 보고 `step_hold_ms=8000`(8초)으로 늘려서 재측정(`deadzone_measure_node`에 `start_cmd`
파라미터 추가해서 0부터 다시 안 올리고 원하는 torque부터 바로 테스트 가능하게 함).

| 방향 | hold 300ms | hold 8000ms |
|---|---|---|
| 정방향 | 110 | **106** (재현 2회) |
| 역방향 | -110 | **-106** |

8초로 늘리니 106에서도 정확히 1 hires count(Δ=0.036°) 움직임이 확인됨 — 300ms 관측 창이 너무 짧아서
느린 creep을 놓치고 있었던 것. 100(8초 유지)에서는 안 움직여서 진짜 문턱은 (100, 106] 사이로 좁혀짐.
정방향/역방향 모두 크기 106으로 대칭 확인.

**중요한 한계**: 이 "106"도 절대적인 진짜 값은 아님. Breakaway torque는 시스템에 순수 Coulomb
마찰만 있는 게 아니라 기어/케이블 backlash·프리로드로 인한 점탄성(viscoelastic) creep 성분이 있어서,
**대기 시간을 늘릴수록 문턱값은 계속 낮아질 수 있음**(300ms→110, 8초→106, 더 기다리면 더 낮아질 수도
있음 — 수렴한다는 보장 없음). 그러므로 "진짜 최소 breakaway torque"라는 단일 상수보다는, **"이 정도
대기 시간 안에 반응하려면 필요한 torque"**로 이해해야 함. 실용적으로 의미 있으려면 이 hold_ms를
실제 조향 제어 루프가 요구하는 응답 시간에 맞춰서 다시 재는 게 맞음(지금은 임의로 300ms/8초를 고른
것일 뿐).

## 한계 및 다음 단계

- 위 "3차"에서 설명한 대로, breakaway torque 값은 관측 시간(`step_hold_ms`)에 종속적 — 절대 상수
  아님. 실제 제어 주기에 맞춰 hold_ms를 정하고 재측정해야 실용적으로 의미 있는 값이 나옴.
- hires(0.036°/count)는 버킷 자체가 이미 충분히 작아서, CANopen으로 갈아타도 추가로 얻을 수 있는
  각도 해상도 이득은 크지 않다고 판단 — 이관 불필요.
- position 모드(`steering_control_node`)로 다시 돌아갈 때는 4.3.3.4절 기계적 0점 캘리브레이션을
  먼저 해야 함(지금은 0점이 안 맞은 상태로 방치됨).
- **Position 모드가 실제로 Position→Speed→Torque 3단 캐스케이드인지 검증 필요** — Windows 유틸리티로
  `0007/0008`(speed loop Kp/Ki)을 바꿔가며 Position 모드에서 동일 스텝 응답이 달라지는지 확인.
  (담당: 팀원이 진행 예정)
- **다음 실험 — torque-전류-각도 상관관계식 도출 (Gazebo 파라미터용, 실차 장착 후)**:
  `deadzone_measure_node`의 `sweep_mode:=true`로 breakaway 지점에서 멈추지 않고 `torque_limit`까지
  계단식으로 끝까지 올리며 각 단계 steady-state (torque cmd, 전류, 각도)를 CSV 로그(`SWEEP,...`)로
  남기는 기능을 추가함. **벤치(무부하) 상태에선 의미가 제한적** — 조향 랙/타이어 부하가 없으면 각도가
  한 값에 정착하지 않고 계속 돌기만 함. 실차에 장착해서 실제 부하(랙 마찰, 정지 조향 시 타이어-지면
  마찰) 하에서 재측정해야 함. 아직 미실시.

## 2026-08-03 재연결 읽기 전용 확인

실물 KEYA를 다시 연결해 출력이 잠긴 상태에서 CAN 통신과 측정 경로만 확인했다. 이
호스트에서 KEYA는 `can1`(PCAN-USB), 250 kbit/s였으며 `can0`은 구동 계통용
CANable이었다. 인터페이스 번호는 USB 열거 순서에 따라 바뀔 수 있으므로 다음 벤치에서도
실제 어댑터를 먼저 식별해야 한다.

```bash
ros2 run lateral_ctrl steering_actuator_node --ros-args \
  -p can_interface:=can1 \
  -p hardware_can_enabled:=true \
  -p actuator_output_enabled:=false
```

이 세션에서는 enable, disable, position, torque 프레임을 보내지 않았다. 위치·fault·속도·
전류·전압·온도 query만 보냈고, 종료 뒤 노드와 Agent 프로세스가 모두 사라진 것을 확인했다.

### 실측 결과

| 항목 | 결과 |
|---|---|
| KEYA heartbeat | `0x07000001`, 100구간 평균 `20.100 ms` |
| heartbeat 최소/최대 간격 | `19.852 ms` / `20.342 ms` |
| SDO 위치 query 왕복 | 20회 평균 `1.393 ms`, 최소 `1.321 ms`, 최대 `1.460 ms` |
| heartbeat 위치 | `FFFA` = signed `-6`, 360 count/turn 기준 약 `-6°` |
| SDO 위치 | `FFFFFF53` = signed `-173`, 10000 count/turn 기준 `-6.228°` |
| 모터 상태 | Disable, `0 rpm`, `0 A`, `11 V`, `34°C` |
| fault/status | `0x0001`; bit 0은 fault가 아니라 Disable 상태 |

heartbeat와 SDO 위치는 각자 해상도로 환산했을 때 약 `0.228°` 차이로 일치했다.
따라서 360 count/turn heartbeat와 10000 count/turn SDO 위치 경로가 모두 현재 장치에서
동작한다. MANUAL/Disable 상태에서도 SDO 위치 응답과 `/steering/status.feedback_valid=true`는
확인했지만, 축을 손으로 움직이며 count가 연속적으로 따라오는지와 락투락 폭은 측정하지 않았다.

### 아직 확인하지 못한 경계

- KEYA 작업 모드 Parameter 0020은 CAN heartbeat나 현재 SDO query로 확인할 수 없다.
  제조사 문서의 RS232 상태 질의 `ED 00 00 00 00 00 00 00`이 필요하다. 당시 호스트에는
  KEYA용 USB-RS232가 없고 STM ST-LINK의 `/dev/ttyACM0/1`만 있었다.
- 데드존 측정 전에는 RS232 설정에서 `0020=2`(Torque)를 확인하고, 측정 뒤 반드시
  `0020=3`(Absolute position)으로 복귀시켜 Burn 및 전원 재기동해야 한다.
- Parameter 0034를 `5 ms`로 바꾼 heartbeat 주기 시험도 같은 설정 연결이 필요하다.
- 전원 재인가 위치 유지, 락투락 count 폭, CW/CCW 데드존은 이번 세션에서 수행하지 않았다.
- 상태등은 CAN상 추가 fault 없이 Disable만 보고했다. 눈으로 본 빨강/보라 조합은 파란
  status 점멸과 다른 표시등이 겹쳐 보인 것일 수 있으나, 사진이나 분리 관찰 없이 확정하지 않는다.

이 확인 뒤 KEYA는 벤치에서 분리하기로 했다. 다음 재연결 전까지는 위 미확인 항목을
완료로 취급하지 않는다.
