# visualization

`visualization`은 현재 차량의 canonical ROS 2 명령과 액추에이터 상태를
관찰하는 패키지다. 개발자용 read-only Qt 디버깅 GUI, Foxglove에서 경로와
장면을 확인하기 위한 Marker/GPS demo helper, 실차 없이 화면을 점검하기 위한
synthetic test publisher를 제공한다.

## 이 패키지가 하지 않는 일

- 실차 운용용 차량 명령 생성 또는 중재(synthetic test publisher 제외)
- 안전 판단
- command governor
- 액추에이터 제어
- canonical 상태를 Foxglove용 scalar 토픽으로 반복 발행

`vehicle_dashboard`는 publisher나 parameter 변경 UI를 만들지 않는다. 화면에
보이는 값만으로 메시지 계약에 없는 상태를 추정하지도 않는다.

## 관찰하는 canonical 토픽

| 토픽 | 타입 | 실제 운용 시 발행 주체 | GUI 주요 필드 |
|---|---|---|---|
| `/vehicle/command` | `kaiev26_msgs/msg/ActuatorCommand` | 차량 명령 경로 | 목표 속도, 목표 조향각, command brake |
| `/drive/status` | `kaiev26_msgs/msg/DriveActuatorStatus` | 구동 액추에이터 상태 노드 | mode, gear, 속도, RPM, 전압/전류/온도, timeout, valid/age, fault bits |
| `/brake/status` | `kaiev26_msgs/msg/BrakeActuatorStatus` | 브레이크 액추에이터 상태 노드 | brake commanded, stop reason, actual valid, command age |
| `/steering/status` | `kaiev26_msgs/msg/SteeringActuatorStatus` | 조향 액추에이터 상태 노드 | state/reason, 목표/명령/실제 각도, timeout/valid/age, 모터 진단, fault bits |

`foxglove_test.launch.py`를 실행할 때만 위 네 토픽의 발행 주체는
`test_publisher`다.

## Foxglove

Foxglove는 custom message 내부 필드를 직접 표시할 수 있으므로 필드마다
별도 scalar 토픽을 만들지 않는다. Plot이나 Raw Messages 패널에서 다음과
같은 message path를 바로 선택한다.

- `/vehicle/command.speed_target_mps`
- `/vehicle/command.steering_target_rad`
- `/vehicle/command.brake_engage`
- `/drive/status.speed_mps`
- `/drive/status.motor_rpm`
- `/drive/status.bus_voltage_v`
- `/drive/status.bus_current_a`
- `/drive/status.phase_current_a`
- `/brake/status.brake_commanded`
- `/brake/status.stop_reason_bits`
- `/steering/status.target_wheel_angle_rad`
- `/steering/status.actual_wheel_angle_rad`
- `/steering/status.state`
- `/steering/status.fault_bits`

추천 확인 항목은 target/actual speed, target/actual steering, bus
voltage/current, motor temperature, brake reason, steering state, message
timeout과 age다.

bridge는 `foxglove_bridge.launch.py`로 띄우며 **PC당 하나만** 실행한다 —
하나가 ROS graph 전체를 서빙하고 클라이언트 여러 대가 동시 접속한다. launch가
기본값과 다른 `send_buffer_limit`(100 MB)을 고정하므로 `ros2 run`으로 직접
띄우지 않는다.

## diagnostics_publisher

status 3종(`/drive/status`, `/brake/status`, `/steering/status`)을 판정해
`/diagnostics`(`diagnostic_msgs/DiagnosticArray`)로 1 Hz 요약하는 headless
노드다. Foxglove Diagnostics 패널이 등급순으로 정렬하므로 문제 항목만 위로
올라온다.

실차에서는 관찰이 아니라 구동 스택 소속이라
`control_launch/vehicle.launch.py`가 띄운다 — Foxglove를 켜지 않아도
발행되어 bag에 남는다. 단독 실행은 `ros2 run visualization
diagnostics_publisher`. 구독하는 status가 없으면 STALE로 표시되므로, 조향
없는 최소 HIL bench에는 포함하지 않는다.

## 빌드

현재 workspace에서 `kaiev26_msgs`와 `Control`은 나란히 있으므로 다음처럼
함께 빌드한다.

```bash
source /opt/ros/humble/setup.bash
cd ~/KAI_ws
colcon build \
  --base-paths src/kaiev26_msgs src/Control/software/src \
  --packages-select kaiev26_msgs kaiev26_hil visualization
source install/setup.bash
```

`vehicle_dashboard`는 PyQt5 번들 Qt 우선순위 workaround인
`kaiev26_hil.qt_runtime`을 공유하므로 `kaiev26_hil`도 함께 빌드한다. 같은 파일을
두 패키지에 복제하지 않기 위한 선택이다.

## 실제 차량 GUI 실행

```bash
ros2 run visualization vehicle_dashboard
```

또는:

```bash
ros2 launch visualization vehicle_dashboard.launch.py
```

### 화면 구성

한 페이지에 전부 들어가며 스크롤도 plot도 없다. 디버깅 read-out에서 스크롤해야
보이는 값은 보지 않은 값이기 때문이다. 신호 흐름 순서로 세 열이다.

1. **Command · physical switches** — `/vehicle/command`가 요청한 값, `/drive/status`가
   보고하는 물리 스위치(mode·gear·pedal), `/brake/status`의 제동과 정지 원인
2. **Longitudinal** — `/drive/status`. 목표/실제 속도와 오차를 큰 글씨로 두고,
   EZkontrol Part I·II 진단이 뒤따른다
3. **Lateral** — `/steering/status`. 상태 머신과 disable 이유, 그리고
   target → commanded → corrected → actual 각도 파이프라인

읽는 방법은 두 가지 장치에 기대고 있다.

- **Active conditions** — 헤더 아래 띠는 메시지가 이미 싣고 있는 원인 필드
  (`stop_reason_bits`, `disable_reason`, 각종 `*_timeout`)를 **출처 필드명과 함께**
  그대로 나열한다. 진단하지 않고 보고만 한다. 이 패키지 계약대로, 메시지가
  말하지 않은 상태를 GUI가 추정하는 일은 없다.
- **Flag chips** — bool은 정상일 때 흐리게, 아닐 때만 불이 들어오는 chip이다.
  각 chip은 어느 쪽이 정상인지를 따로 갖는다. `command_received`는 false가
  문제이고 `can_timeout`은 true가 문제이며, brake `actual_valid`와 steering
  `enabled_feedback_valid`는 이 하드웨어에서 계약상 항상 false라 절대 fault로
  표시하지 않는다.

### Parameter

| parameter | 기본값 |
|---|---:|
| `command_stale_ms` | 300 |
| `drive_status_stale_ms` | 300 |
| `brake_status_stale_ms` | 300 |
| `steering_status_stale_ms` | 500 |
| `ui_update_hz` | 4 |

`ui_update_hz`는 원격 X11 기준으로 정한 값이다. plot이 없어져서 한 tick은 값이
바뀐 위젯만 다시 그리므로 예전보다 훨씬 가볍다. 로컬 화면에서는 10으로 올리고,
링크가 느리면 2로 낮추거나 Foxglove bridge를 쓴다(픽셀 대신 데이터만 보낸다).

## Synthetic test 실행

> **경고: `test_publisher`는 `/vehicle/command`를 직접 발행한다. 실제 차량
> 제어 시스템과 절대로 동시에 실행하지 않는다.**

실차와 분리된 환경에서 다음 launch는 `test_publisher`,
`gps_waypoint_publisher`, `vehicle_dashboard`를 함께 실행한다.

```bash
ros2 launch visualization foxglove_test.launch.py
```

약 70초 동안 AUTO 정상 주행, 완만한 속도/조향 변화, command brake, 물리
E-stop, command timeout, motor fault bit, steering fault 상태가 반복된다.
random noise를 쓰지 않아 매 실행 결과가 같다.

## 상태 표기의 의미

- `NO DATA`: GUI 프로세스가 해당 토픽을 아직 받지 못했다.
- `OK`: local receive age가 설정된 stale 기준 안에 있다.
- `STALE`: 마지막 수신 이후 local monotonic 시간이 stale 기준을 넘었다.
- `N/A`: 메시지는 받았지만 해당 `valid` 필드가 false이거나 현재 하드웨어
  계약에서 지원하지 않는 값이다. `N/A`를 fault 또는 값 0으로 해석하지 않는다.

freshness는 MCU의 header stamp가 아니라 GUI 프로세스의 실제 수신 시각을
기준으로 계산한다. 예를 들어 `BrakeActuatorStatus.actual_valid=false`는
현행 브레이크 서보의 실제 피드백 미지원 상태이며 fault가 아니다.

## GPS/waypoint demo helper

`gps_waypoint_publisher`는 실제 GNSS 수신기나 localization estimator가 아니다.
`/drive/status.speed_mps`와 valid한
`/steering/status.actual_wheel_angle_rad`를 이용해 Foxglove용 가상 장면을
움직이는 demo/helper다. 다음 시각화 출력만 발행한다.

- `/gps/fix`
- `/viz/waypoints`
- `/viz/scene`
- TF (`map` → `base_link` → `gps`)

조향 feedback이 invalid이면 실제 조향각이라고 단정하지 않고 demo 적분에서
0 rad를 사용한다.

## command governor

명령 경로에 `command_governor`가 들어와 있다
([`../command_governor/README.md`](../command_governor/README.md)).

```text
/planning/command
        ↓
command_governor (현재 패스스루, 형상은 이후 단계)
        ↓
/vehicle/command
```

향후 GUI에서는 planning / governed / actual 값을 비교할 예정이다. 거버너가
형상을 시작하면 `/governor/status`류의 "무엇을 얼마나 잘랐는지" 표시가 함께
필요하다.

## 문제 해결

- `kaiev26_msgs`를 찾지 못하면 위 빌드 명령처럼 메시지 패키지를 함께
  빌드하거나 해당 메시지 workspace의 `install/setup.bash`를 먼저 source한다.
- Qt display를 열지 못하면 `DISPLAY`와 GUI 세션을 확인한다. SSH에서는 X11
  forwarding 또는 적절한 원격 desktop 환경이 필요하다.
- 토픽이 `NO DATA`이면 `ros2 topic list`와 `ros2 topic info <토픽>`으로
  발행 주체와 ROS domain을 확인한다.
- 실제 차량에서는 `test_publisher` 또는 `foxglove_test.launch.py`가 함께
  실행되고 있지 않은지 반드시 확인한다.
- `ros2 topic info /vehicle/command -v`로 publisher가 하나인지 확인한다.
