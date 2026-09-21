# KAI Vehicle Console

KAI Vehicle Console을 하나의 ROS package로 설치·실행한다. 공용 frontend와
관측 계층을 공유하면서 Vehicle과 HIL은 독립 backend로 동작한다.

```text
vehicle_console/
├── vehicle_console/common/   Web · ROS reception · runtime interpretation
├── vehicle_console/vehicle.py Vehicle monitoring + steering fault acknowledgment
├── vehicle_console/hil/      장비 · 명령 · scenario · fault · virtual HMI · vcan
├── launch/                  vehicle.launch.py / hil.launch.py
└── web/                     단일 React/Vite frontend
```

```bash
ros2 launch vehicle_console vehicle.launch.py  # VEHICLE · MONITOR + FAULT ACK / 8766
ros2 launch vehicle_console hil.launch.py      # HIL / 8767
```

The profile is fixed by the launched server. Vehicle subscribes to canonical topics
and streams telemetry over an outbound WebSocket; inbound WebSocket application
messages are closed with code 1008. Its only ROS service client acknowledges the
Host steering HARD latch through an explicit same-origin POST. It creates no
vehicle/HIL command publisher, parameter-write service, serial connection or scenario runner.

ROS's standard parameter-events publisher is the only implicit rclpy publisher;
remote parameter services and rosout are disabled. The Web server exposes health,
static assets and `/ws`, without a history, command, reset, scenario or touch API.

## 설치와 실행

ROS 2 Humble 및 기존 frontend의 Node.js 환경을 사용한다. Web Python 환경은
ROS Python modules를 볼 수 있어야 한다. 새 환경을 만들 때:

```bash
cd ~/KAI_ws/src/Control/software/src/vehicle_console
python3 -m venv --system-site-packages .venv
.venv/bin/pip install -r requirements-web.txt
cd web
npm ci
npm run build
cd ~/KAI_ws
si
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --packages-select vehicle_console vehicle_state_publisher foxglove control_launch
si
source src/Control/software/src/vehicle_console/.venv/bin/activate
ros2 launch vehicle_console vehicle.launch.py
```

브라우저는 **http://127.0.0.1:8766**에 접속한다. launch는 활성화된 `VIRTUAL_ENV`의
Python을 기본으로 사용한다. 다른 Python은 `web_python:=/absolute/path/to/python`으로
지정한다. 공용 `.venv`는 두 profile과 HIL CLI가 사용할 의존성을 제공한다.
패키지에 HIL 의존성을 함께 설치해도 Vehicle 실행 시 HIL backend를 초기화하지 않는다.

Frontend를 바꾸면 `npm run build` 뒤 `colcon build`를 실행한다. 유일한 source는
`vehicle_console/web`, 설치 위치는 `share/vehicle_console/web`이다.

HIL은 같은 패키지의 별도 진입점으로 실행한다:

```bash
ros2 launch vehicle_console hil.launch.py  # Web backend만, 기본 8767
# 전체 bench 실행 인자와 UNO 연결은 HIL 운용 문서를 따른다.
ros2 launch foxglove foxglove_bridge.launch.py  # 상세 분석, 기본 8765
```

Foxglove(8765), Vehicle(8766), HIL(8767)은 서로 다른 기본 포트를 사용한다. Console은 Foxglove를
자동으로 띄우지 않는다. 구동 launch는 `control_launch/vehicle.launch.py`, 관측만
시작하는 launch는 **`vehicle_console/vehicle.launch.py`**다.

관측용 Vehicle Console 재시작은 조향 상태에 영향을 주지 않는다. 조향 복구는
[구동 launch 문서](../control_launch/README.md#조향-정지와-자동-복구)를 따른다.
HIL의 Scenarios는 벤치 시험 기능이며 실차 입력 응답·경로 추종 TUI는
[vehicle_test_scenarios](../vehicle_test_scenarios/README.md)에서 실행한다.

## Profile과 표시

| Profile | Pages / capabilities |
|---|---|
| VEHICLE · MONITOR + FAULT ACK | Overview, Drive, Steering, Safety / Brake, ECU Runtime, Diagnostics |
| HIL | Bench, 공용 Drive/Steering/Safety/Runtime/Diagnostics, Fault Injection, Scenarios, Virtual LCD/Touch, control ownership/ARM/heartbeat |

Profile은 선택한 서버 진입점에 고정되며 브라우저 전환 기능이 없다.
서버의 `profile`/`capabilities`가 navigation을 구성한다. Vehicle 화면에 HIL
controls나 비활성 버튼을 남겨놓지 않는다. HIL-only 모듈은 `vehicle_console.hil`에 두며 Vehicle은 이 모듈을 import하지 않는다.
HIL의 UNO touch readiness는 별도로
현재 owner, USB 상태, firmware 보고 freshness와 generation에 의존한다.

Overview는 명령, mode/gear/pedal/RUN, 목표/관측 속도와 조향각, 브레이크 요청,
STM_B가 보고한 E-stop, active conditions, ECU A/B 상태를 요약한다. 좁은 화면은
패널을 세로로 배치하며 상단 탐색을 가로로 스크롤할 수 있다.
동일한 StateRail, TargetActual, SteeringArc, telemetry/flag 해석과 runtime
instruments를 HIL에서도 사용한다. HIL-only controls는 HIL wrappers에 남는다. Headless 도구도 같은 패키지의 CLI로 유지한다.

## Topics와 freshness

모두 canonical interface를 **best-effort, volatile, depth 5**로 구독한다. Reliable
publisher와 best-effort RuntimeStats를 함께 받을 수 있다. 이 값들은 화면 freshness
기준이며 MCU timeout이나 safety gate를 변경하지 않는다.

| Topic | Message | Vehicle stale 기본값 |
|---|---|---|
| `/vehicle/command` | `kaiev26_msgs/msg/ActuatorCommand` | 300 ms |
| `/drive/status` | `kaiev26_msgs/msg/DriveActuatorStatus` | 300 ms |
| `/brake/status` | `kaiev26_msgs/msg/BrakeActuatorStatus` | 300 ms |
| `/steering/status` | `kaiev26_msgs/msg/SteeringActuatorStatus` | 500 ms |
| `/stm_a/runtime_stats`, `/stm_b/runtime_stats` | `kaiev26_msgs/msg/RuntimeStats` | 3000 ms, 수신과 firmware snapshot 진행 모두 필요 |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 3000 ms, hardware_id/name별로 독립 |

일반 status의 기본값은 기존 Qt Vehicle Dashboard와 같다. launch 인자는
`command_stale_ms`, `drive_stale_ms`, `brake_stale_ms`, `steering_stale_ms`다.
`update_hz` 기본값은 5다. Topic 수신 Hz와 Web 갱신 Hz는 다른 값이다.

NO DATA / LIVE / STALE 및 수신 age를 표시한다. WAN이 조용히 끊겨도 브라우저의
monotonic elapsed time을 수신 age에 더해 만료시키며, 마지막 속도 등을 live처럼
유지하지 않는다. 약 2.5초 이상 Web frame이 없으면 연결을 닫고 1.5초 간격으로
재연결한다. 진단은 여러 publisher의 항목을 합치고 stale 상세는 과거 기록으로만
펼쳐 볼 수 있다. Humble의 DiagnosticStatus `byte`는 숫자 severity로 한 번 변환한다.

## Runtime health

`vehicle_console/common/runtime_config.py`가 MCU task ID와 stack/heap allocation을,
`runtime_health.py`가 유효성·rate·history·stale 해석을 소유한다. HIL의 명시적인
Bench Reset acknowledgment만 `vehicle_console/hil/runtime_health.py`의 subclass에 남았다.
Vehicle에는 acknowledgment 액션이 없다.

Wall time은 관측 시간이며 WCET가 아니다. Stack free는 minimum-ever HWM,
`cpu_load_valid=false`는 미지원, STM_B E-stop latency는 EXTI callback→task start다.
Deadline 누적 이력과 최근 증가, timing unavailable, counter discontinuity 해석은
기존 구현을 공유한다. Firmware/message wire 계약은 바꾸지 않았다.


## Steering HARD fault 확인·해제

Steering의 Host safety policy는 원본 fault, 활성 bit별 이름/등급, recovery attempts와
lifetime total, 성공 시각, reconnect, command/CAN/position/fault age를 표시한다.
RECOVERABLE attempts는 통계이며 횟수나 지속시간으로 영구 FAULT가 되지 않는다.

**CLEAR HARD FAULT LATCH**는 실제 KEYA 고장이 사라진 뒤 Host latch만 acknowledge한다.
`hard_fault_clearable=true`인 fresh 진단에서 버튼이 활성화되며, 실제 서비스가
호출 시점의 조건을 다시 검사한다. MANUAL, 유효 |속도|≤0.05 m/s, fresh drive/brake/CAN/
position/fault, E-stop·command brake inactive, active HARD=0, reconnect·movement·probe
미진행, stop TX 완료가 필요하다. 성공해도 Enable하지 않고 목표·이력을 지운 뒤 WAIT_CMD로
이동한다. 이후 AUTO와 clear 이후에 생성된 fresh command가 필요하다.

`POST /api/steering/clear-fault-latch`는 고정된 `/steering/clear_fault_latch`
(`std_srvs/srv/Trigger`)에만 연결된다. 결과는 Console에 그대로 표시하며 성공 응답만으로
관측 상태를 바꾸지 않는다. 동시 요청은 409, 서비스 미준비는 503, 3초 응답 timeout은
504/result unknown이며 자동 재시도하지 않는다. WebSocket은 관측 전용이고 주행 명령,
임의 ROS service, parameter 변경은 제공하지 않는다. 기본 localhost/명시적 Host allowlist와
same-origin 검사를 유지하며 이 경계 안의 운영자가 해제 요청을 수행한다.

`/diagnostics`는 공용 ObserverNode에서 `(hardware_id, name)`별로 병합한다. 각 항목의
freshness가 독립적이므로 drive/brake array가 steering/safety를 삭제하거나 오래된 상태를
fresh로 만들지 않는다. Vehicle 3초, HIL 기본 1초 stale 기준은 유지한다.

## 원격 접속

기본 bind는 `127.0.0.1`이다. `bind`, `port`, `allowed_hosts`는 startup/launch 인자다.
`allowed_hosts`는 쉼표로 나눈 **명시적 host/IP**이며 wildcard는 허용하지 않는다.
Host 검사는 HTTP와 WebSocket 모두에 적용하고, WebSocket은 동일한 Host와 Origin의
HTTP/HTTPS 요청만 받는다. HARD latch clear POST도 동일한 Host/Origin 검사를 통과해야 한다.

SSH 터널:

```bash
ssh -N -L 8766:127.0.0.1:8766 kai@vehicle-pc
```

Tailscale에서는 원격 브라우저가 실제로 사용할 주소를 먼저 확인해 지정한다. 공유된
tailnet에서 보이는 IP가 서버 자체 IP와 다를 수 있다. 예시 주소는 실제 주소로 바꾼다.

```bash
ros2 launch vehicle_console vehicle.launch.py allowed_hosts:=100.112.72.27
sudo tailscale serve --bg --yes --tcp=8766 tcp://127.0.0.1:8766
tailscale serve status
# 공유 해제
tailscale serve --tcp=8766 off
```

원격 브라우저는 `http://100.112.72.27:8766`으로 접속한다. 기존 Tailscale ACL과
장비 접근권한을 사용하며 앱 자체 사용자 인증은 제공하지 않는다. 공개 Funnel이나
Host/Origin wildcard를 사용하지 않는다. `--bg` 설정의 lifetime은 사용자가 관리한다.
이번 refactor의 검증은 Tailscale 설정을 변경하지 않았다.

## 전송량과 검증

Vehicle snapshot에는 필요한 status 필드와 정규화된 runtime instrument 값만 보낸다.
HIL desired/emulator/catalog/event history/ownership/PNG는 포함하지 않는다. ROS header,
중복 raw RuntimeStats 및 화면에서 쓰지 않는 runtime counters도 전송하지 않는다.
JSON은 그대로 유지한다. 원본 메시지와 상세 계측 분석은 ROS/Foxglove에서 확인한다.

2026-09-11 기존 HIL board graph를 **구독만** 한 installed Vehicle server에서,
초기 5 frame을 제외한 실제 WebSocket 30 frame 측정: **4.993 Hz**, 평균 **8398.1 B**,
약 **41931.7 B/s = 0.3355 Mbps**. 이때 6개 토픽이 LIVE였고 `/vehicle/command`는
미수신이었다. 테스트를 위해 명령을 발행하지 않았다. 수치는 클라이언트 한 개의
비압축 JSON payload 기준이며 TCP/TLS/Tailscale overhead는 제외한다. 모든 7개 토픽을
제공한 isolated ROS sample에서는 4.977 Hz, 평균 8315.6 B, 0.3311 Mbps였다. Diagnostics
항목 수에 따라 크기가 달라지고 여러 브라우저의 전송량은 합산된다.

```bash
cd ~/KAI_ws
si
cd ~/KAI_ws/src/Control
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  software/src/vehicle_console/.venv/bin/python -m pytest software/src/vehicle_console/test -q
ROS_DOMAIN_ID=196 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  software/src/vehicle_console/.venv/bin/python -m pytest software/src/vehicle_state_publisher/test -q
cd software/src/vehicle_console/web
npm test
npm run typecheck
npm run build
```

Browser tests use the existing Playwright/Chrome surface and skip if those tools
are absent. Backend tests use actual isolated ROS pub/sub, with no hardware.
Full results and remaining physical limits are in [CONSOLE_VALIDATION.md](CONSOLE_VALIDATION.md).
Qt source/tests/launch/docs are preserved in `archive/software/legacy_hil_gui` and
`legacy_vehicle_dashboard`, with the unused helper in `archive/software/control_gui_common`.
The subtree is excluded by `COLCON_IGNORE`. See [parity matrix](CONSOLE_PARITY.md).
Tkinter and Matplotlib remain active dependencies of the vcan SIL GUI.

HIL operational details: [HIL_WEB.md](HIL_WEB.md); headless/serial/SIL: [HIL_BACKEND.md](HIL_BACKEND.md).

## 이전 패키지 이관

`kaiev26_hil` 전체의 source/tests/config/launch/docs는
`archive/software/kaiev26_hil`에 보관한다. 그 archive는 `COLCON_IGNORE`로 제외되며,
active backend는 `vehicle_console.hil`이다. 노드 이름과 ROS topic/service/parameter는
유지하므로 firmware 및 다른 제어 노드의 계약은 바뀌지 않는다.

| 이전 실행 | 현재 실행 |
|---|---|
| `kaiev26_hil hil_web.launch.py` | `vehicle_console hil.launch.py` |
| `kaiev26_hil hil.launch.py` | `vehicle_console hil.launch.py web:=false start_bench:=true agents:=false` |
| `kaiev26_hil hil_bench.launch.py` | `vehicle_console hil.launch.py web:=false start_bench:=true` |
| `kaiev26_hil vcan_sim.launch.py` | `vehicle_console vcan_sim.launch.py` |
| `ros2 run kaiev26_hil <tool>` | `ros2 run vehicle_console <tool>` |

HIL 결과 기본값 `~/.local/state/kaiev26_hil/web_results`는 이전 결과를 계속 표시하기
위해 유지했다. 구버전 프로세스 종료를 확인한 뒤 옛 source 경로의 호환 symlink와
`build/kaiev26_hil`, `install/kaiev26_hil`을 제거했다. Archive와 기존 시험 결과는 보존한다.

Active launch는 세 개다: HIL의 Web/headless를 제공하는 `hil.launch.py`, 실차 관측·HARD latch acknowledgment
`vehicle.launch.py`, 독립 CAN simulator인 `vcan_sim.launch.py`.
HIL 중간 launch 두 개의 기능은 `hil.launch.py`로 통합했다. `web:=false`에서도
`start_bench:=true`를 지정해야 backend를 시작한다. Headless generator 기본값은 기존과
같이 enabled=true/watchdog=0이며 Web은 항상 DISARMED/watchdog=3 s로 시작한다.

구동 calibration은 `drive_` 설정키로 구분해 표시합니다. 기존 STM_A v1 readback의 native `config`와 새 `display_values` 매핑은 [파라미터 호환표](../../../docs/CONTROL_TUNING_PARAMETERS.md)를 따릅니다.
