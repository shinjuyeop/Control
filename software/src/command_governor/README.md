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

실차 시험의 TUI 실행 파일은 `vehicle_test_scenario_runner`(ROS node `/auto_sysid`),
headless 실행 파일은 `vehicle_test_scenario_node`(ROS node `/vehicle_test_scenario_runner`)다.
둘을 포함한 `/planning/command` 명령원은 한 번에 하나만 사용한다. 시나리오
기본 목표속도는 0이며, 실제 입력값과 실행기 제한은
[시나리오 문서](../vehicle_test_scenarios/README.md)를 따른다.

## 속도 명령 slew

양수 목표속도를 낮출 때만 `deceleration_slew_mps2`로 하강률을 제한한다.
기본 0.5 m/s²에서는 5→2 m/s 명령을 6초에 걸쳐 낮춘다. 음수 목표는 그대로 전달한다.
Speed 프리셋은 `acceleration_slew_mps2`로 양수 목표 상승률도 제한한다.
TUI 초기값 1.0 m/s²에서는 0→5 m/s에 5초가 걸린다. Torque/외부 전류 프리셋과
외부 종방향 제어기를 선택한 vehicle launch는 가속 제한을 0(해제)으로 적용한다. 목표 0은 즉시 전달해 STM_A가 토크컷하고 타력감속한다.
`brake_engage=true`는 즉시 전달하며 속도 목표를 0으로 만든다. 조향은 가공하지 않는다.

계산은 연속된 유효 입력의 monotonic 수신 시간 차를 사용한다. 입력 공백이
`input_timeout_ms`를 넘으면 이전 slew 상태를 버린다. 가속 제한이 켜져 있으면
첫 입력 및 공백 이후 첫 입력은 0부터 다시 시작한다. 해제 상태면 새 목표부터 시작한다.
0으로 설정하면 slew 제한을 해제한다. 이는 명령 변화율 제한이며 실제 차량
감속도나 jerk를 보장하지 않는다. 실제 응답은 모터 제어기와 차량에서 확인해야 한다.

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
- 상태 판정·진단 합성 (`vehicle_state_publisher/diagnostics_publisher`의 몫)
- 하드 바운드 — 최종 한도는 액추에이터 쪽이 지킨다: 종방향은 STM_A 펌웨어
  (RPM 4500/−1500, AUTO/MANUAL 480 A), 횡방향은 lateral_ctrl
  (`command_count_limit` 5300 ≈ -28.33°..+28.03°, slew 60 deg/s). 거버너는 그 안에서
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
| `deceleration_slew_mps2` | `0.5` | 양수 속도 목표 하강률 (m/s²). 0이면 제한 해제. 음수·비유한 값은 거부 |
| `acceleration_slew_mps2` | Speed 프리셋 `1.0`, 노드 단독 기본 `0.0` | 양수 목표 상승률 (m/s²). 0이면 제한 해제 |

파라미터 모두 시작 시 한 번만 읽는다. 런타임 `ros2 param set`은
`restart required` 사유로 거부되므로 조용한 no-op이 되지 않는다.

`input_timeout_ms`는 [Control Tuning](../control_tuning/README.md)의 `수신 시간 제한` 분류에서
수정하고 `S`로 선택한 프리셋을 저장한다. 단독 launch와 `control_launch vehicle.launch.py` 모두
`tuning_preset`(기본 `driving`)을 읽는다. `airborne`도 선택할 수 있다.
기본 프리셋 폴더는 Control 저장소의 `software/src/control_tuning/config/control_tuning.presets/`다.
사용자 지정 `control_tuning_file:=bench.yaml`이면 `bench.presets/`를 읽는다.
저장 후 거버너를 포함한 launch를 같은 프리셋으로 재시작한다.
감속 slew는 `구동응답`의 `governor_deceleration_slew_mps2`로 편집한다.
Speed/Torque에 표시하며 외부 전류 모드에는 표시하지 않는다.
가속 slew는 같은 분류의 `governor_acceleration_slew_mps2`이며 Speed에서만 표시한다.
프리셋 선택 없이 실행하는 HIL bench launch는 가속 slew를 해제한다.

## 빌드·시험·실행

```bash
cd ~/KAI_ws
si
colcon build --packages-select command_governor
colcon test --packages-select command_governor && colcon test-result --verbose
si
ros2 launch command_governor command_governor.launch.py
```

발행자가 하나뿐인지 확인:

```bash
ros2 topic info -v /vehicle/command   # Publisher count: 1 (command_governor)
```

위 단독 launch는 개발용이다. 실차에서는
[`control_launch/vehicle.launch.py`](../control_launch/launch/vehicle.launch.py)가
respawn과 함께 띄우고, HIL에서는 `vehicle_console/hil.launch.py`가
`start_bench:=true`일 때 respawn 없이(크래시가 드러나도록) 띄운다.

## 관련 문서

- 통신 계약: [`../../../docs/INTERFACES.md`](../../../docs/INTERFACES.md)
- 시스템 구조: [`../../../docs/ARCHITECTURE.md`](../../../docs/ARCHITECTURE.md)
- HIL 경로: [`../vehicle_console/HIL_BACKEND.md`](../vehicle_console/HIL_BACKEND.md)
- 실차 시험 입력: [`../vehicle_test_scenarios/README.md`](../vehicle_test_scenarios/README.md)
- 횡방향 소비자: [`../lateral_ctrl/README.md`](../lateral_ctrl/README.md)
