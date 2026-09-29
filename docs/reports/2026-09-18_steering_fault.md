# 2026-09-18 ASMS 전환 후 조향 무응답 분석

반복된 `STEER MOTOR FAULT`의 직접 원인은 KEYA 스톨 비트가 아니라 Host torque
제어기의 **`angle_limit` 잠금**이었다. MANUAL 조작도 AUTO 조향 보정 범위로 검사해
새 fault를 만들고, 조향을 중앙으로 돌리거나 ASMS를 다시 켜도 잠금이 남는 문제가
확인됐다. 시험 TUI가 이 소프트웨어 fault를 놓쳐 출발 준비를 계속하는 문제도 수정했다.

후속 사용자 요청으로 **AUTO에서도 `angle_limit` fault를 발생시키지 않도록 변경했다.**
노드의 encoder 범위 검사와 torque 제어기의 각도 범위 fault 경로를 제거했다.
목표각 clamp와 바깥 방향 torque 차단은 유지하며, 범위 밖에서도 안쪽 복귀 torque를
허용한다. 이 문서 작성 당시에는 과속·피드백 이상·통신 단절 fault를 유지했다.
2026-09-19 변경으로 조향속도 기반 fault와 Host latch는 제거됐고, 피드백 이상은
조건이 정상화되면 새 명령으로 자동 복구한다. E-stop·CAN 단절·KEYA 실제 fault·
유효하지 않은 피드백 차단은 유지한다.
아래 원인 분석과 초기 수정·검증 내역은 `c048eaa` 당시 기준이며, 현재 동작은
[Torque 안전 동작](../../software/src/lateral_ctrl/TORQUE_CONTROL.md#안전-동작)을 따른다.

## 근거와 확인 범위

- 원본: `/home/kai/KAI_ws/records/260918_torque_control_test`의 17개 `rosbag/rosbag_0.mcap`.
  원본은 수정하지 않았다. 시간은 MCAP 수신 시각, 아래 절대 시각은 KST다.
- `/steering/status`, `/steering/torque_safety`, torque 명령·명령전류, 각속도,
  `/drive/status`, `/brake/status`, `/vehicle/command`를 역직렬화해 비교했다.
  각 시험의 `control_parameters.yaml`과 `console.log`도 대조했다.
- 조향 상태 총 **19,229개**: `fault_bits & ~1 != 0`인 샘플 **0개**,
  `feedback_valid=false`인 샘플 **0개**. 위치 피드백 age 최대 약 **19.01 ms**.
  `fault_bits=1`은 Disable 상태이지 스톨 고장이 아니다.
- 8개 시험은 첫 조향 샘플부터 `torque_safety=angle_limit`, `state=FAULT`였으며,
  해당 기록 전체에서 torque 명령전류는 0 A였다. 이미 조향 출력이 차단된 상태에서
  ASMS를 켰으므로, 이 시험의 ON 직후 정렬 과정이 새 스톨을 만든 것으로 볼 수 없다.
- 새 `none → angle_limit` 전환은 아래 두 기록에서 관측했다. 그 외 8개 잠금의
  **최초 발생 순간은 해당 MCAP 시작 전**이므로 개별 발생 원인까지 단정하지 않는다.

| 시험 시작 시각 / 이름 접미사 | 조향 기록 결과 |
| --- | --- |
| 19:28:30, 19:28:34, 19:28:49 / `steering_step_v0` | 약 1초의 MANUAL 기록, fault 없음 |
| 19:44:35 / `steering_step_v0` | 약 0.6초, `/steering/status` 없음: 조향 진단 불가 |
| 21:00:20, 21:01:01 / `steering_step_v3` | 시작부터 `angle_limit`, ASMS ON 뒤에도 유지 |
| 21:01:41 / `steering_step_v3` | MANUAL 기록, fault 없음 |
| 21:01:46 / `steering_step_v3` | AUTO 조향 활성 구간 있음, 기록 내 fault 없음 |
| 21:05:29, 21:07:03 / `steering_sine_v5` | 시작부터 `angle_limit`, ASMS ON 뒤에도 유지 |
| 21:07:21 / `steering_sine_v5` | AUTO 조향 활성 구간 있음, 기록 내 fault 없음 |
| 21:12:42, 21:12:57, 21:13:02, 21:13:45 / `steering_step_v5` | 시작부터 `angle_limit` |
| 21:14:07 / `steering_step_v5` | AUTO 중 정상 제어, MANUAL 전환 후 새 `angle_limit` |
| 21:19:26 / `steering_step_v5` | AUTO 중 정상 제어, MANUAL 전환 후 새 `angle_limit` |

### 실제 발생 순서

| 항목 | `20260918_211407_198936_steering_step_v5` | `20260918_211926_776150_steering_step_v5` |
| --- | --- | --- |
| fault 시각 | 21:14:53.944 | 21:20:36.749 |
| MCAP 첫 분석 토픽 수신 이후 | +46.645 s | +67.383 s |
| MANUAL 수신 이후 | 약 1.191 s | 약 1.225 s |
| 직전 count → fault count | −3247 → **−3292** | −3269 → **−3299** |
| fault 시 보정 환산각 | −27.968° | −28.045° |
| mode / Enable 명령 | MANUAL / false | MANUAL / false |
| torque 명령 / 전류 피드백 | 0 / 0 A | 0 / 0 A |
| KEYA `fault_bits` | 1 (Disable) | 1 (Disable) |
| 위치 피드백 age | 18.48 ms | 18.53 ms |

모터를 끈 뒤 MANUAL에서 encoder가 AUTO 보정 한계 `−3279 count`를 넘어갔다.
이는 보정 범위 밖이라는 뜻이며, 기계적 end-stop 충돌이나 모터 스톨의 증거는 아니다.
범위 밖 환산각은 원인 식별용이며 실측 바퀴각 정확도를 보장하지 않는다.

## 원인 → 해결 방안 → 적용한 수정

### 1. MANUAL 조작까지 AUTO fault로 잠금

**원인:** `SteeringActuatorNode::controlTick()`이 drive mode와 관계없이
`motor_position_count`와 torque feedback을 검사했다. `steering_enabled=true`이면
모터가 이미 Disable인 MANUAL에서도 `torqueFault(ANGLE_LIMIT)`를 호출했다.
`torque_fault_latched_`는 중앙 복귀·새 명령·ASMS 재전환으로 지워지지 않는다.

**해결 방안:** 운전자 수동 조작과 AUTO 제어의 감시 범위를 구분한다. AUTO 진입 시
현재 위치 검사와 AUTO 중의 안전 정지는 계속 적용한다.

**적용:** torque feedback의 각도·각속도 검사는 `DriveMode::AUTO`에서 수행한다.
기존 MANUAL 전환의 torque 0 → Disable → 제어 이력 reset은 유지한다. AUTO 복귀 시
범위 밖이면 Enable 전에 fault로 차단한다. AUTO에서 이미 발생한 fault는 MANUAL을
거쳐도 해제되지 않는다. 과속 한계·보정 범위·전류 한도·통신 timeout은 바꾸지 않았다.

### 2. 시나리오가 소프트웨어 조향 fault를 놓침

**원인:** `SysidNode._receive()`는 KEYA `fault_bits & ~1`과 통신 상태만 확인했다.
Host의 `STATE_FAULT` / `REASON_MOTOR_FAULT`는 검사하지 않아, 조향 출력이 차단되어도
GO로 판단했다. 21:01:01 시험은 시작부터 fault였지만 `console.log`에 카운트다운과
`정속 준비`가 기록됐고 `/vehicle/command.speed_target_mps`는 **0.8333 m/s**까지 나갔다.
이 기록의 실제 차속은 0이었다. 같은 결함은 구동이 응답하는 상황에서 조향 불능인 채
출발 명령을 내릴 수 있으므로 시작·실행 중 검사에 포함해야 한다.

**해결 방안 / 적용:** 위 state 또는 disable reason이면 `조향 제어 안전 정지`로
판정한다. Run 시작 전에는 녹화·출력 시작을 막고, 진행 중에는 기존 종료 경로로
목표속도 0 및 브레이크를 요청한다. 노드 fault를 자동 해제하지 않는다.

### 3. `STEER MOTOR FAULT` 문구가 스톨처럼 보임

**원인:** `AutoDashboardSummary.steer_state()`는 Host의 `STATE_FAULT`도
`STEER_MOTOR_FAULT`로 변환한다. Torque 안전 정지가 기존 메시지 ABI의
`REASON_MOTOR_FAULT`를 사용하므로 LCD 문구만으로 하드웨어/소프트웨어 원인을 구분할
수 없다.

**해결 방안 / 적용:** 운용 문서에 `fault_bits & ~1`과 `/steering/torque_safety`를
함께 확인하도록 명시했다. 이번 수정은 기존 ROS 메시지와 대시보드 CAN 프로토콜을
유지하며, TUI에서는 소프트웨어 정지를 별도 문구로 표시한다.

## 스톨 가설 및 다른 무응답 요인

- **정지 마찰 가능성:** 21:07:21 시험의 ASMS ON 후 1–7초 지점에서 실제 차속 0,
  환산각 약 +0.233°, 목표 0°, 명령전류 약 −0.933 A, 전류 피드백 −1 A가 관측됐다.
  작은 P 출력으로 정지 마찰을 넘지 못하는 현상과 부합하지만, 원시 CAN이나 별도
  토크 측정 없이 마찰·드라이버 내부 동작을 확정할 수는 없다. 이 기록에서는
  `torque_safety=none`이며 하드웨어 fault도 없다. 반복 fault의 직접 원인과는 별개다.
  미소 오차까지 정렬해야 한다면 접지 조건에서 breakaway 전류와 잔류 오차를 측정한
  뒤 settle 허용오차 또는 마찰 보상을 조정해야 한다. 이번에는 이 근거만으로
  P·전류 한도를 올리거나 마찰 보상을 추가하지 않았다.
- **최저 차속에 따른 정상 대기:** 21:12 이후 snapshot은
  `minimum_steering_speed_kph=0.5`다. ASMS ON 상태여도 그 미만에서는 `WAIT_CMD`로
  출력이 차단된다. 21:14:07 / 21:19:26 시험의 첫 ACTIVE는 ASMS ON 후 각각
  약 6.488 / 7.495초였고, 당시 차속은 약 0.559 km/h였다. 초기 차속이 약
  0.481 km/h로 내려갈 때 WAIT_CMD로 되돌아가는 구간도 있다. 이는 fault가 아닌
  현재 속도 조건의 동작이다. 기존 미커밋 주행 프리셋의 0.5 km/h 설정을 보존했다.
- 이번 자료는 KEYA의 실제 `0020` 설정, 기계 간섭, 배선, 실제 바퀴각을 직접
  검증하지 않는다. 다만 이들을 이번 반복 `angle_limit`의 직접 원인으로 지목할
  기록은 없고, 기록된 모든 조향 피드백은 유효했다.

## 검증 및 반영 범위

- `lateral_ctrl`을 `/tmp`의 별도 빌드 디렉터리에서 빌드했다. 운용 install과
  실행 중인 차량 노드는 교체하지 않았다.
- 기존 CTest 6개 suite, **82개 테스트 통과**. 새 회귀 항목은 MANUAL 범위 초과 후
  중앙 복귀·새 AUTO 명령, 범위 밖 AUTO 진입 차단, 기존 AUTO fault의 MANUAL 왕복
  유지다. 기존 E-stop·timeout·과속·하드웨어 fault·출력 실패 검사도 통과했다.
- 기존 `test_sysid.py`의 **49개 테스트 통과**. 시작 전 fault 차단 및
  카운트다운/실행 중 fault의 정지·브레이크 요청을 포함한다.
  별도 recorder/실제 governor를 띄우는 기존 통합 테스트 2개는 실행하지 않았다.
- 실제 차량 재시험·플래시·운용 프로세스 재시작·push는 하지 않았다. 실차 적용 시에는
  수정 패키지 빌드와 운용 노드 재시작이 필요하며, 접지 상태에서의 개선은 미검증이다.
