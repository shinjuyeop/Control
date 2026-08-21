# command_governor

`/planning/command`(Planning의 의도)를 받아 `/vehicle/command`(액추에이터 계약)로
재발행하는 명령 파이프의 한 단계다. **이 노드가 `/vehicle/command`의 유일한
발행자다.** 단일 writer 보장은 운영 절차가 아니라 이 구조에서 나온다.

```text
Planning ──/planning/command──> command_governor ──/vehicle/command──┬─> STM_A
                                                                     ├─> STM_B
                                                                     └─> lateral_ctrl
```

입출력이 같은 타입(`kaiev26_msgs/ActuatorCommand`)이므로 STM_A/B 펌웨어와
lateral_ctrl은 변경 없이 기존 구독을 유지한다.

## 현재 단계: 패스스루

지금은 형상(shaping) 로직이 없다. 필드 4개를 그대로 전달하고, 정합성 게이트만
적용한다. 이 단계의 목적은 토폴로지 확정이다 — 단일 발행자, 토픽 체인, stamp
규율, QoS 정합, HIL 경로 편입을 제어 수학 없이 먼저 검증한다.

이후 단계에서 이 노드에 들어올 것 (설계 스케치 기준):

- 종방향: 속도 상한, 가·감속 램프, jerk 제한
- 횡방향: 조향각 형상, 속도 의존 조향율 제한
- 축간 협조: 곡률 기반 속도 상한, 마찰원
- `/governor/status` 진단 (무엇을 얼마나 잘랐는지)

## 동작

- **수신 즉시 재발행.** 자체 타이머가 없다. 타이머 재발행은 최대 한 주기의
  지연을 더해 조향 200 ms 예산을 깎는다.
- **입력이 끊기면 출력도 끊긴다.** keep-alive로 마지막 값을 유지하지 않는다 —
  상류 정지를 가리기 때문이다. 출력이 끊기면 STM_A의 500 ms Host timeout과
  조향의 200 ms freshness가 기존 그대로 안전망이 된다.
- **출력은 이 노드의 클럭으로 재-stamp한다.** 하류(조향)가 stamp 나이로
  신선도를 판정하므로, 발행자 클럭 정합 책임을 이 노드 한 곳으로 모은다.
- **정합성 게이트** (`governor_core`, host gtest로 시험):

| 판정 | 조건 |
|---|---|
| `REJECT_INVALID_STAMP` | 입력 stamp가 0(미기록) 또는 sec가 음수(손상·언더플로) |
| `REJECT_NON_FINITE` | `speed_target_mps` 또는 `steering_target_rad`가 NaN/Inf |
| `REJECT_STALE` | 입력 나이 > `input_timeout_ms` |
| `REJECT_FUTURE_STAMP` | 입력 나이 < −`input_timeout_ms` (클럭 불일치) |

거부된 입력은 발행되지 않고 1초 throttle로 경고 로그에 남는다.

거부는 **메시지 전체를 버린다 — `brake_engage=true`가 실려 있어도.** 정합성이
깨진 명령으로는 어떤 액추에이터도 움직이지 않는다는 의도적 선택이다. 상류가
죽으며 남긴 stale brake는 여기서 소실되지만, 제동 안전은 이 경로에 의존하지
않는다: 물리 E-stop은 STM_B 독립 경로이고, AUTO 명령 두절은 STM_A 500 ms
HALTED로 정지한다.

## 하지 않는 일

거버너는 명령 파이프의 한 단계지 제어기가 아니다.

- 액추에이터 구동, CAN 통신
- enable/disable 상태기계 (조향의 것은 lateral_ctrl에 있다)
- 캘리브레이션, 각도 변환
- 상태 판정·진단 합성 (visualization/diagnostics_publisher의 몫)
- 하드 바운드 — 최종 한도는 액추에이터 쪽이 지킨다: 종방향은 STM_A 펌웨어
  (RPM 4500/−1500, MANUAL 175 A), 횡방향은 lateral_ctrl
  (`command_count_limit` 3279 ≈ -27.83°..+25.92°, slew 60 deg/s). 거버너는 그 안에서
  형상을 만든다.

## 계약 (발행자로서 지키는 것)

- QoS: RELIABLE, KeepLast(10) — 구독자 전원이 기본 RELIABLE이므로
  BEST_EFFORT 발행자는 매칭 자체가 안 된다
- nonzero `header.stamp`, 시스템 클럭 기준 — 조향은 stamp 나이 200 ms 초과 시
  즉시 Disable한다
- `brake_engage`를 매 메시지 명시 — 걸린 브레이크는 fresh한
  `brake_engage=false`로만 풀린다
- 발행률은 입력을 따른다. 실차 시험의 `vehicle_test_scenario_runner`와 HIL의
  `hil_command_generator`는 20 Hz로 발행한다. 5 Hz는 조향 200 ms 예산의
  경계라 간헐 Disable이 난다

## 파라미터

| 이름 | 기본값 | 의미 |
|---|---|---|
| `input_topic` | `/planning/command` | 구독 토픽 |
| `output_topic` | `/vehicle/command` | 발행 토픽 |
| `input_timeout_ms` | `200.0` | 입력 나이 게이트. 조향 `steer_command_timeout_ms`와 같은 값. 0 이하면 launch가 실패한다 |

세 파라미터 모두 시작 시 한 번만 읽는다. 런타임 `ros2 param set`은
`restart required` 사유로 거부되므로 조용한 no-op이 되지 않는다.

## 빌드·시험·실행

```bash
cd ~/KAI_ws
colcon build --packages-select command_governor
colcon test --packages-select command_governor && colcon test-result --verbose
source install/setup.bash
ros2 launch command_governor command_governor.launch.py
```

발행자가 하나뿐인지 확인:

```bash
ros2 topic info -v /vehicle/command   # Publisher count: 1 (command_governor)
```

위 단독 launch는 개발용이다. 실차에서는
[`control_launch/vehicle.launch.py`](../control_launch/launch/vehicle.launch.py)가
respawn과 함께 띄우고, HIL에서는 `kaiev26_hil`의 `hil.launch.py`가
respawn 없이(크래시가 드러나도록) 띄운다.

## 관련 문서

- 통신 계약: [`../../../docs/INTERFACES.md`](../../../docs/INTERFACES.md)
- 시스템 구조: [`../../../docs/ARCHITECTURE.md`](../../../docs/ARCHITECTURE.md)
- HIL 경로: [`../kaiev26_hil/README.md`](../kaiev26_hil/README.md)
- 실차 시험 입력: [`../vehicle_test_scenarios/README.md`](../vehicle_test_scenarios/README.md)
- 횡방향 소비자: [`../lateral_ctrl/README.md`](../lateral_ctrl/README.md)
