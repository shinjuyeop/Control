# Control Tuning 파라미터 이름과 호환 계약

조향·구동을 함께 편집하므로 **담당 계통 + 항목 + 단위**가 이름에 드러나게 한다.
TUI/프리셋/정규화된 MCAP 분석값에는 같은 canonical key를 사용한다.
기존 ROS 파라미터, topic/message type 및 STM_A v1 배열 순서는 연결용 계약으로 유지한다.
새 이름을 기존 이름으로 단순 치환할 때 단위까지 같다고 가정하면 안 된다.

## 이름 규칙

- `steering_`: KEYA 조향 제어 또는 조향 허용 판단에 사용하는 설정.
- `drive_`: STM_A AUTO Speed/Torque 공통 구동 envelope와 Torque 속도 PI 설정.
- `governor_`: 상위 명령의 유효성 검사와 속도 명령 변화율.
- `kp`, `ki`, `kd`: P/I/D 게인. `_a_per_deg`, `_a_per_mps` 등은 출력 전류와 오차의 단위를 표현한다.
- `limit_a`: 명령/적분 전류 한도. MCAP의 실측 `phase_current_a` 또는 `motor_current_a`와 구분한다.
- `timeout_ms`: 해당 소비자가 판단하는 유효시간. `steering_vehicle_speed_timeout_ms`는
  조향 허용 판단용 차속의 수신 제한이며 STM_A 구동 속도 PI의 timeout이 아니다.

특히 `steering_torque_feedback_timeout_ms`는 **조향 Torque 모드의 위치 피드백·샘플 간격·
제어 지연 제한**이다. 구동 STM_A의 `MOTOR_FEEDBACK_TIMEOUT_MS=200`은 별도 firmware 상수다.
이 두 timeout은 별도 계약이며 이번 envelope 변경에서도 값을 바꾸지 않는다.

## 새 이름과 기존 연결

아래 ROS 항목은 해당 노드의 parameter 이름이다. 조향 node는 `/steering_actuator_node`,
거버너는 `/command_governor`다. STM_A 항목은 v1 배열의 논리 필드명이며
startup calibration helper도 기존 이름의 ROS 파라미터를 받는다.
TUI 상세에는 새 설정키와 기존 연결을 함께 표시한다.

| 새 설정키 | 화면 이름 | 단위 | 기존 연결 |
|---|---|---|---|
| `steering_control_mode` | 조향 제어 방식 | 선택값 | 조향 ROS: `actuation_mode` |
| `steering_kp_a_per_deg` | 조향 P 게인 | A/deg | 조향 ROS: `torque_full_scale_error_deg` |
| `steering_kd_a_per_deg_s` | 조향 D 게인 | A/(deg/s) | 조향 ROS: `torque_kd` |
| `steering_settle_error_deg` | 조향 정착 오차 | deg | 조향 ROS: `torque_settle_angle_deg` |
| `steering_settle_rate_deg_s` | 조향 정착 각속도 | deg/s | 조향 ROS: `torque_settle_rate_deg_s` |
| `steering_rate_filter_hz` | 조향 각속도 필터 | Hz | 조향 ROS: `torque_rate_filter_hz` |
| `steering_peak_current_limit_a` | 조향 최대 전류 한도 | A | 조향 ROS: `torque_peak_limit_raw` |
| `steering_sustained_current_limit_a` | 조향 지속 전류 한도 | A | 조향 ROS: `torque_sustained_limit_raw` |
| `steering_peak_duration_ms` | 조향 고출력 허용시간 | ms | 조향 ROS: `torque_peak_duration_ms` |
| `steering_peak_recovery_ms` | 조향 고출력 회복대기 | ms | 조향 ROS: `torque_peak_recovery_ms` |
| `steering_angle_margin_deg` | 조향 한계각 여유 | deg | 조향 ROS: `torque_soft_limit_margin_deg` |
| `steering_min_vehicle_speed_kph` | 조향 허용 최소 차속 | km/h | 조향 ROS: `minimum_steering_speed_kph` |
| `governor_command_timeout_ms` | 거버너 명령 유효시간 | ms | 거버너 ROS: `input_timeout_ms` |
| `governor_acceleration_slew_mps2` | 거버너 가속 slew | m/s² | 거버너 ROS: `acceleration_slew_mps2` |
| `governor_deceleration_slew_mps2` | 거버너 감속 slew | m/s² | 거버너 ROS: `deceleration_slew_mps2` |
| `steering_command_timeout_ms` | 조향 명령 수신 제한 | ms | 조향 ROS: `steer_command_timeout_ms` |
| `steering_can_timeout_ms` | 조향 CAN 수신 제한 | ms | 조향 ROS: `can_timeout_ms` |
| `steering_position_feedback_timeout_ms` | 조향 위치 피드백 제한 | ms | 조향 ROS: `feedback_timeout_ms` |
| `steering_torque_feedback_timeout_ms` | 조향 토크모드 피드백 제한 | ms | 조향 ROS: `torque_feedback_timeout_ms` |
| `steering_vehicle_mode_timeout_ms` | 조향용 차량모드 수신 제한 | ms | 조향 ROS: `mode_timeout_ms` |
| `steering_vehicle_speed_timeout_ms` | 조향용 차속 수신 제한 | ms | 조향 ROS: `drive_status_timeout_ms` |
| `drive_control_mode` | 구동 제어 방식 | 선택값 | STM_A v1: `control_mode` |
| `drive_speed_kp_a_per_mps` | 구동 속도 P 게인 | A/(m/s) | STM_A v1: `speed_kp_a_per_mps` |
| `drive_speed_ki_a_per_mps_s` | 구동 속도 I 게인 | A/(m/s*s) | STM_A v1: `speed_ki_a_per_mps_s` |
| `drive_speed_deadband_mps` | 구동 속도 데드밴드 | m/s | STM_A v1: `speed_deadband_mps` |
| `drive_coast_release_error_mps` | 구동 타력 해제 오차 | m/s | STM_A v1: `speed_hysteresis_mps` |
| `drive_current_limit_a` | 구동 전류 한도 | A | STM_A v1: `max_drive_current_a` |
| `drive_integrator_limit_a` | 구동 적분 전류 한도 | A | STM_A v1: `integrator_limit_a` |
| `drive_current_slew_up_a_s` | 구동 전류 상승률 | A/s | STM_A v1: `current_slew_up_a_s` |
| `drive_current_slew_down_a_s` | 구동 전류 하강률 | A/s | STM_A v1: `current_slew_down_a_s` |
| `drive_reapply_current_band_a` | 구동 재인가 전류 구간 | A | STM_A v1: `reapply_current_a` |
| `drive_reapply_current_slew_a_s` | 구동 재인가 전류 상승률 | A/s | STM_A v1: `reapply_slew_a_s` |
| `drive_motor_speed_limit_rpm` | 구동 모터 회전수 한도 | rpm | STM_A v1: `motor_rpm_limit` |

조향 P/D·전류 한도는 physical-unit TUI 값과 native parameter 값 사이에 변환이 있다.
예를 들어 조향 `steering_kp_a_per_deg`는 `torque_full_scale_error_deg`에 단순 이름 변경으로
저장하지 않는다. 기존 peak current/reference 환산식을 그대로 사용한다.
`drive_current_limit_a`와 `drive_motor_speed_limit_rpm`은 **AUTO Speed/Torque 공통 envelope**다.

| 항목 | Speed Control | Torque Control |
|---|---|---|
| `drive_current_limit_a` | EZ native speed-control current limit field | PI phase-current command ceiling |
| `drive_motor_speed_limit_rpm` | signed target motor RPM의 ±preset 범위 | EZ torque-mode motor RPM ceiling |

CAN 송신부의 최종 전진 4500 / 후진 1500 rpm 물리 clamp는 별도로 유지한다.
Driving 480 A/4500 rpm에서 기존 Speed baseline CAN 명령은 동일하다.
Airborne는 Kp15/Ki2, 40 A/4500 rpm의 first-test candidate이며 두 AUTO mode에 같은 envelope를 쓴다.
RPM 한도는 driving과 같고, 전류 한도는 공중 시험용으로 낮게 유지한다. 기존
20 A/600 rpm 기능 확인 후보와 달리 40 km/h 목표를 RPM에서 제한하지 않지만,
실제 도달 속도와 전류는 아직 공중 시험으로 검증되지 않았다.
나머지 speed PI, 적분 한도, deadband/coast, current slew/reapply 항목은 Torque 전용이다.
MANUAL pedal/gear는 이 AUTO envelope의 영향을 받지 않는다.

TUI는 편집 중인 `drive_control_mode`에 맞는 구동 항목만 표시한다.
Speed는 모드·전류 한도·RPM 한도·governor 가속/감속 slew를 표시하고,
Torque는 가속 slew를 숨기고 PI·deadband·감속 최소 전류·적분 한도·전류 slew/reapply를 추가한다.
`external_current`는 모드·전류 한도·RPM 한도를 표시한다.
연속 PI가 사용하지 않는 `drive_coast_release_error_mps`는 숨긴다.
숨긴 설정값은 프리셋과 STM_A ABI에 보존되며 다른 모드로 돌아가면 재사용된다.
Speed/외부 전류에서는 숨겨진 PI 한도와 현재 전류 한도의 관계를 검사하지 않고,
Torque로 전환해 저장할 때 다시 검사한다.

governor 감속 slew 기본값은 0.5 m/s²이며 0이면 제한 해제다.
양수 목표로 낮출 때만 적용하므로 5→2 m/s에는 6초가 걸린다.
0 목표는 즉시 토크컷·타력감속하고 명시적 브레이크는 지연하지 않는다.
Speed 전용 가속 slew 초기값은 1.0 m/s²이고 0이면 제한 해제다.
켜져 있으면 첫 명령과 통신 공백 후 재개 시 0부터 목표까지 상승한다.
Torque/외부 전류 프리셋에서는 저장값을 보존하면서 governor 적용값만 0으로 만든다.
기존 v2/v3 프리셋에 이 항목이 없으면 기본 설정에서 채우고 다음 저장 때 기록한다.

## 프리셋 schema v3

```yaml
schema_version: 3
condition: driving
steering_current_reference_a: 20.0
steering:
  steering_control_mode: torque
  steering_kp_a_per_deg: 2.4
  # 나머지 조향 항목도 명시
drive:
  drive_control_mode: torque
  drive_speed_kp_a_per_mps: 180.0
  drive_current_limit_a: 480.0
  # 나머지 구동 항목도 명시
governor:
  governor_command_timeout_ms: 200.0
timeouts:
  steering_torque_feedback_timeout_ms: 100
  # 나머지 수신 제한도 명시
```

위는 구조 설명용 발췌다. 실행에는 checked-in `driving.yaml`/`airborne.yaml`처럼
관리 대상 전체 항목이 있어야 한다. 분류 이동이나 이름 변경은 calibration 값을 바꾸지 않는다.

- v2의 `lateral`/`longitudinal`과 예전 키는 읽을 때 `steering`/`drive`의 새 키로 연결한다.
  읽기만으로 원본 파일을 덮어쓰지 않는다. TUI가 이전 형식임을 알리고 S 저장 시 v3로 전환한다.
- S 저장은 기존 파일을 `.yaml.bak`에 보관한다. 편집한 값이 없어도 v2 파일은 S로 변환할 수 있다.
- 같은 항목의 새/옛 이름을 동시에 쓰고 값이 다르면 오류다. 한쪽을 조용히 우선하지 않는다.
- 누락한 gain/current/timeout을 default로 채우지 않는다.
- 종방향 값이 없던 v1 flat preset은 여전히 명시적 이관이 필요하다.
- 공통 `control_tuning.yaml`은 기존 node별 native ROS override 형식을 유지한다.
  런치 인자와 `ros2 param`에 새 TUI 키를 그대로 전달하는 것이 아니다.

TUI는 현재값 조회·편집·S 저장만 한다. **A 즉시 적용 기능은 없다.**
선택한 프리셋으로 구동 스택을 재시작할 때 적용된다. MCU config 적용 gate와 readback 검사는 유지한다.

## MCAP / Console / 분석

- `/sysid/control_parameters`와 `control_parameters.yaml` snapshot은 **schema v2**로 기록한다.
  `display_values`/`display_units`에 위 canonical key를 사용하며 조향·거버너 모두 기록한다.
- `ros__parameters`는 실제 노드에서 받은 원래 이름·단위 그대로다.
  `/parameter_events`도 원래 노드명/parameter 이름을 유지한다. 새 이름의 가상 ROS 값으로
  바꾸거나 YAML의 값으로 runtime 값을 채우지 않는다.
- 기존 snapshot v1은 `control_tuning.naming.canonical_capture()`로 새 분석 이름에 매핑한다.
  원본 MCAP을 수정하지 않으며 이전 데이터의 제거된/추가 필드는 보존한다.
  v1에 없던 거버너 `display_values`를 기본값으로 새로 만들지 않는다.
- `/drive/longitudinal_status`는 **Float32MultiArray ABI v1 그대로**다.
  host `decode_status()`가 기존 `config`와 새 이름의 `display_values`를 함께 제공한다.
  Console과 MCAP 분석에서 새 이름을 쓴다. 새 Console은 기존 서버의 `config`도 읽을 수 있다.
- 기존 Drive/SteeringActuatorStatus, `/drive/status`, `/steering/status` 토픽명과 실측 필드는 바꾸지 않았다.
  이름/MCAP 타입 변경 때문에 micro-ROS를 재생성할 필요는 없다. 단, **Speed/Torque 공통 envelope 동작은 수정된 STM_A firmware를 flash해야 한다.**
  이전 firmware의 config readback만으로 Speed 경로가 preset current/RPM 한도를 준수한다고 판단하지 않는다.

`longitudinal_analysis.read_bag()`는 실제 MCAP JSON snapshot도 정규화해
`parameter_captures`로 반환한다. 원본 parameter events와 MCU config도 함께 남겨
source 값과 해석값을 비교할 수 있다.
