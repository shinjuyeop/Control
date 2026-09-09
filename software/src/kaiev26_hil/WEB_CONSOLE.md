# Web HIL Console

React 기반 HIL 운용 화면이다. 기존 PyQt 화면과 별도로 제공하며 serial은 기존
`stm_test_bridge`가 단독으로 소유한다. 펌웨어와 production topic 계약은 바꾸지 않는다.

## 화면과 Operator Remote

모든 페이지에서 같은 **Operator Remote**를 사용한다. Desktop에서는 오른쪽 고정 패널,
1100 px 이하에서는 상단 **Remote** 버튼으로 여는 drawer다. 좁은 화면의 패널은 닫기 버튼,
바깥 영역 또는 Esc로 닫으며, 열린 동안 키보드 포커스가 패널 안에 머문다.

Remote 상단은 MANUAL/AUTO, FWD/REV, 가속 페달 0~100%, DASH/SIDE/RF E-stop 입력을 담당한다.
E-stop은 **STM_TEST 입력 주입**이며 물리 비상정지 버튼이 아니다. 그 아래 **주행 명령**에서
Host 목표 속도·조향각·브레이크 요청을 입력하고 발행/해제한다. 페달 주입과 별도 경로다.
Quick preset, fault, Plant/RPM/전압/전류 설정은 각 기능 페이지에 둔다.
Remote는 조작 위주로 표시하며, 조작 불가 사유는 잠겼을 때만 표시한다.
반복 설명, 수신 age, 별도 페달 관측 줄은 중앙 화면으로 옮겼다.
벤치·구동의 **페달 관측 · STM_A**에서 수신 비율과 ADC 환산 전압을 확인한다.
전압은 `pedal_adc_raw × 3.3 / 4095`로 계산한 STM_A ADC 입력 기준 추정값이며,
별도 전압계 측정값이 아니다.
주행 목표 왼쪽에 페달을 배치하고, 구동에서는 속도 오차 오른쪽에 Part I·II를 묶는다.
Part별 CAN age와 유효 상태는 별개로 표시하며 좁은 화면에서는 줄을 나눈다.

- **벤치**: 연결 상태, 주요 입력 → 명령 → 피드백, 구동/조향 요약과 최근 이벤트.
- **구동**: 목표/관측 속도/오차, EZ Part I·II age, 속도·RPM 통합 그래프와 전류 그래프,
  구동 보드 상세값과 에뮬레이터 응답/피드백 설정.
- **조향**: 로드휠 target → commanded → corrected → actual과 오차,
  KEYA 모터축 피드백·수신 상태·플래그.
- **브레이크·안전**: 제동 요청, PWM, stop reason, RUN·구동 정지 요청·상전류 명령,
  Host 브레이크 명령과 실제 피드백 지원 여부.
- **가상 LCD**: UNO가 보고한 적용 표시값으로 실제 C++ renderer의 480×320 화면 재구성.
- **고장 주입**: 양쪽 CAN 활성화, TX DROP, controller fault, 주입한 고장 해제.
- **시나리오**: 기존 YAML 단계·관측 조건, 실행·중단, PASS/FAIL, JSON 다운로드와 이전 결과.
- **진단·로그**: ROS 수신 Hz/주기/age/누적, CAN 오류 카운터, bus-off, diagnostics, 원본 명령 로그.

주행 명령 입력은 오른쪽 공통 패널에 한 번만 표시한다. 패널이 길면 아래로 스크롤한다.
좁은 화면에서도 상단 Remote로 열 수 있다. 속도·조향·브레이크 초안은 페이지를
바꿔도 유지되며 적용할 때 세 값을 함께 보낸다. ARM 확인창에서 전송할 값을 확인한다.
목표 속도 초안은 새로 화면을 열면 0 km/h로 시작하며 이전 generator 목표를 가져오지 않는다.
속도·조향각은 입력 중 빈칸을 허용하고, 빈칸으로 포커스를 벗어나면 0을 채운다.
입력 편집이나 포커스 이동만으로 명령을 발행하지 않는다.
페이지 이동 자체로 명령을 보내지 않는다. 운전자 입력의 선택 표시는 보드 관측값이며,
ACK를 받았다고 요청값으로 바꾸지 않는다. 페달은 드래그를 놓을 때 한 번 전송한다.

Remote에는 기존 조작권·fresh·busy·시나리오 잠금을 그대로 적용한다. 상단에는 연결 상태,
조작권, **COMMAND SOURCE ARMED/DISARMED/UNKNOWN**, 시나리오 실행 상태를 항상 표시한다.
발행 상태를 읽지 못하면 DISARMED로 추정하지 않는다.

기본 UI는 **Control Rack** 테마다. 밝은 회색 작업대와 계측 모듈 형태의 패널,
진한 본문, 파란 선택 상태를 사용한다. Operator Remote는 어두운 제목 띠로 구분한다.
본문·조작 항목은 14–16 px,
보조 정보는 최소 12 px이며 한글 상태 문구도 같은 sans-serif 글꼴을 사용한다.
숫자는 tabular로 정렬하고 코드·토픽은 한글 fallback을 포함한 monospace를 사용한다.
좁은 화면에서는 글자를 축소하는 대신 패널을 세로로 배치하며, 그래프 축 글자도
패널 너비에 따라 작아지지 않는다. 그래프의 목표선은 갈색, 관측선은 파란색이며
전류 보조선은 보라색이다. 정상 수신은 green, stale/warning은 amber,
fault/E-stop은 red로 구분한다. 정상 상태의 배경 강조는 줄이고 ARMED는 상단에 강조한다.

그래프는 최근 60초를 보관한다. 그래프 일시정지는 표시만 멈추며 HIL 실행이나 Remote를
멈추지 않는다. 구동 통합 그래프는 같은 시간축에 속도(km/h, 왼쪽)와 RPM(오른쪽)을
표시하며 축별로 자동 범위를 계산한다. RPM은 점선으로 구분하고 각각의 수신 출처를 표시한다.
UI는 5 Hz, bridge STATUS poll은 기존 10 Hz다.

조향의 **로드휠 위치 추종**은 `/steering/status`의 명령각과 관측각을 비교한다.
관측각은 KEYA 엔코더 count를 조향 보정식으로 환산한 중앙 등가 조향각이며 독립적인
바퀴 각도 센서값이 아니다. **KEYA 모터축 피드백**은 `/hil/status`의 수신 모터축 목표와
STM_TEST가 모사한 모터축 회전각을 비교한다. 두 그래프는 기준 각도가 다르다.

가상 LCD는 기본 **1×**(화면 영역 480×320)로 열리며 확대 버튼으로 2×를 선택할 수 있다.
좁은 화면에서는 가용 너비에 맞춘다. 페이지 내부 가용 너비가 900 px 이상이면
UNO Dashboard와 디스플레이 연결을 좌우로, 그보다 좁으면 세로로 배치한다.

### 디자인 시안

운용 화면 하단의 **디자인 시안 ↗**은 별도 탭으로 연다.
`http://127.0.0.1:8765/?preview=pitwall`은 어두운 F1 텔레메트리 스타일,
`http://127.0.0.1:8765/?preview=rack`은 밝은 계측 랙 스타일이다.
상단에서 두 배치와 정상 주행·E-stop·CAN 지연 예시를 비교할 수 있다.
시안의 입력 조작은 화면 내부 상태만 바꾸며 예시 계측값과 구분한다.
시안은 WebSocket·ROS·USB 연결을 만들지 않고 실제 명령을 보내지 않는다.
실제 운용 화면은 Control Rack 테마를 사용하며 시안의 예시 계측값은 가져오지 않는다.

### Frontend 구성

- `web/src/app`: App shell, navigation, global status와 responsive dock/drawer.
- `web/src/hooks`: 단일 WebSocket 연결과 공유 조작/명령 초안/시나리오 상태.
- `web/src/pages`: 관측·시험 목적별 페이지.
- `web/src/components`: Operator Remote, 주행 명령, ARM 확인창, 그래프·telemetry.
- `web/src/domain`: wire types, 단위 변환, 상태·fault 의미와 표시 형식.
- `web/src/styles`: 공통 토큰, 레이아웃, 계측 표시, Remote와 페이지 스타일.

backend action, ROS topic/service, lease, watchdog, scenario runner와 UNO 표시 보고 계약은
변경하지 않았다. MCU Resource는 아직 실제 수집 경로가 없어 빈 페이지를 만들지 않았다.

## 설치와 빌드

Node.js 20.19 이상 또는 22.12 이상, ROS 2 Humble 환경을 사용한다.
실행 예제는 이 PC의 zsh `humble` 함수(Humble 환경 로드 + Domain 0 설정)를 사용한다.
이미 활성화 후 domain을 바꾼 셸과 다른 PC의 설정은
[HIL runbook 공통 ROS 환경](../../../docs/HIL_RUNBOOK.md#4-공통-ros-환경)을 따른다.
Web의 Python 의존성은 기존 GUI 환경과 분리해 설치할 수 있다.
현재 호스트에서는 아래 `.venv`에 설치되어 있다.

```zsh
cd /home/kai/KAI_ws/src/Control/software/src/kaiev26_hil
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv pip install --python .venv/bin/python -r requirements-web.txt
cd web
npm ci
npm run build

cd ~/KAI_ws
humble
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --packages-select kaiev26_hil
source ~/KAI_ws/install/setup.zsh
```

`uv`를 쓰지 않는 환경에서는 `python3-venv` 설치 후
`python3 -m venv --system-site-packages .venv`와 `.venv/bin/pip install -r requirements-web.txt`로
같은 환경을 만든다. `npm run build` 결과는 colcon 빌드 시 package share에 설치된다.
프론트엔드를 수정한 뒤에는 두 빌드를 순서대로 실행한다.

## 화면만 실행 / 기존 ROS 상태 관찰

다음 기본 실행은 Web bridge만 시작한다. Agent, USB serial, CAN 인터페이스를 열지 않는다.
이미 실행 중인 ROS와 같은 domain이면 수신한 상태를 표시한다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch kaiev26_hil hil_web.launch.py \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/kaiev26_hil/.venv/bin/python
```

브라우저: **http://127.0.0.1:8765**

보드가 없어도 모든 페이지를 열 수 있으며 미수신 값은 `—`로 표시한다.
가짜 연결이나 시뮬레이션 값을 실제 보드 상태로 표시하지 않는다.

별도의 소프트웨어 확인에는 `ROS_DOMAIN_ID=192 ROS_LOCALHOST_ONLY=1` 등
실차/벤치와 분리된 domain을 사용한다.

## Web으로 벤치 전체 실행

기존 `hil.launch.py`, `hil_bench.launch.py`, PyQt GUI, 별도 scenario runner와 중복 실행하지 않는다.
실차 Planning 또는 다른 `/planning/command` 발행자와도 동시에 실행하지 않는다.

정지·중립, 가속페달 해제, E-stop 사용 가능 상태에서 대상 보드와 벤치 배선을 확인하고,
실제 액추에이터 연결 여부 및 서보/LV 전원 상태를 확인한 후 실행한다.
이 명령은 실제 USB 장치를 여는 HIL 실행이다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch kaiev26_hil hil_web.launch.py \
  start_bench:=true \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/kaiev26_hil/.venv/bin/python
```

기본 장치는 `/dev/hil_test`, `/dev/hil_a`, `/dev/hil_b`다.
기존 bench와 동일하게 `agents:=false`, `steering:=true`, `state:=true`를 지원한다.
조향/차량 상태 노드를 시작할지는 연결된 벤치 구성에 맞춘다.
Web 전용 launch는 **generator DISARMED**, heartbeat timeout **3초**로 시작한다.
기존 PyQt launch의 기본값은 유지한다.

1. 상단 **조작권 가져오기**로 이 브라우저를 유일한 Web 조작자로 지정한다.
2. 입력 조작을 수행하고 명령 ACK와 보드 관측값을 확인한다.
3. 주행 명령은 **ARM · 명령 발행**, 시나리오는 해당 시험의 명령 발행 허용 후 실행한다.
4. 시나리오 실행 중에는 수동 입력을 잠근다. 종료/중단 시 명령 발행을 끄고 Bench Reset을 요청한다.
5. 조작권 해제 또는 조작 브라우저 연결 종료 시에도 같은 정리를 수행한다.

브라우저 heartbeat가 3초간 끊기면 Web 조작권을 회수한다.
명령 생성기도 Web heartbeat를 직접 감시하므로 Web 서버가 비정상 종료되어도
3초 내에 `enabled=false`로 전환한다. 재접속만으로 발행을 다시 켜지 않는다.
WebSocket 연결은 유지되지만 브라우저 heartbeat만 멈춘 경우에는 조작권 회수와
별개 generator watchdog이 순차적으로 적용될 수 있다. 이는 물리 E-stop을 대체하지 않는다.

**Bench Reset은 물리 비상정지가 아니다.** 기존 UI의 Safe State를 이름만 바꾼 것이다.
backend action `safe`와 `/hil/reset_safe_state`는 그대로이며, 명령 발행 해제와 bridge의 기본 desired state
큐 등록이다. 기본값에는 E-stop 해제와 plant ON이 포함된다. 보드가 끊겨 있으면
재연결 때 적용된다. 서비스 응답은 큐 등록이며 물리 상태나 PWM 적용의 증명이 아니다.

## 표시의 의미

- 주입 요청(`desired`)과 STM_TEST의 STATUS 관측값을 분리한다.
  ACK를 받았다고 화면의 관측값을 요청값으로 덮어쓰지 않는다.
- STATUS 수신과 별도로 각 ROS topic의 수신 age를 표시한다.
  화면의 1초 stale 판정은 표시용이며 firmware timeout 정책을 변경하지 않는다.
- CAN 초기화 성공, RX 프레임 수신, 오류 상태를 별도로 표시한다.
  Protocol 1의 명령 관측 필드나 Protocol 3 이전의 bus 진단은 미지원으로 표시한다.
- `TX DROP`은 firmware STATUS에 없으므로 **bridge 요청 상태**라고 표시한다.
- Web의 속도 입력·관측값·오차·그래프는 **km/h**, 조향 입력·관측각·오차·그래프는 **degree(°/deg)**다.
- ROS `/planning/command`, `/vehicle/command`, `/drive/status`는 기존 **m/s·rad** 계약을 유지한다.
  Web 전송 시 km/h를 3.6으로 나누고 degree를 rad로 변환하며, 수신 값은 반대로 변환한다.
  예: Web의 36 km/h·30° → ROS의 10 m/s·π/6 rad.
- 기존 입력 범위는 같은 물리 범위로 유지한다: 속도 ±180 km/h(기존 ±50 m/s),
  로드휠 조향각 약 ±57.2958°(기존 ±1 rad). STM_TEST 직접 속도 주입은 기존 ±200 km/h다.
- STM_TEST 모터축 각도는 원래 degree이므로 재변환하지 않는다. 모터축과 로드휠은 구분해 표시한다.
- 시나리오 단계와 조건의 속도/각도 값도 화면에서 변환한다. 원본 ROS 명령 로그와 결과 JSON은
  진단을 위해 기존 필드명과 단위를 보존한다.
- 브레이크 PWM은 전기 신호의 관측이며 서보 실제 위치 피드백이 아니다.

### ROS 수신 주기와 EZkontrol CAN age

진단·로그 페이지에서 ROS 토픽별 **수신 Hz, 평균 주기(ms), 마지막
수신 경과시간(ms), 누적 수신 수**를 표시한다. Web 백엔드의 ROS 콜백 수신 시각을
기준으로 최근 5초의 메시지 간격을 평균낸다. 표본이 하나면 Hz/주기는 `—`,
1초 이상 수신이 없으면 Hz는 0으로 표시한다. 누적 횟수는 백엔드 재시작 시 초기화된다.
WebSocket 화면 갱신 주기(5 Hz)를 ROS 수신 주기로 표시하지 않는다.

- **EZkontrol Part I (`0x1801D0EF`)**: `/drive/status.part1_age_ms`, `part1_valid`
- **EZkontrol Part II (`0x1802D0EF`)**: `/drive/status.part2_age_ms`, `part2_valid`

이 CAN age는 **STM_A가 상태 메시지를 작성할 때 마지막 해당 CAN 프레임 수신으로부터
지난 시간**이다. 유효 기간이 지나도 유한한 age는 INVALID 표시와 함께 보여주고,
아직 한 번도 수신하지 않아 무한대인 값은 `—`로 표시한다. `/drive/status` 자체가
끊기면 오래된 CAN age를 현재값처럼 표시하지 않는다.
Part별 CAN Hz를 ROS 토픽 Hz로 추정하지 않는다. 현재 메시지에는 각 Part의
모든 수신 시각/카운터가 없으므로 정확한 개별 CAN 수신 주기는 제공하지 않는다.

프론트엔드 수정 후에는 브라우저를 새로고침한다. 이미 실행 중인 구버전 Web
백엔드에서 Hz/주기가 `—`이면 기존 launch를 종료하고 같은 명령으로 재시작해야 한다.

## 결과와 접속 범위

결과 JSON은 기본 `~/.local/state/kaiev26_hil/web_results/<run-id>/`에 저장한다.
작은 시험 결과만 저장하며, 대용량 bag/영상 저장 기능은 이번 범위에 없다.
`results_dir:=...`로 변경할 수 있고 API는 최근 30개 결과를 표시한다.
이벤트 로그 150개와 그래프 60초는 메모리에만 유지된다.

서버는 **127.0.0.1에만** 바인딩한다. WebSocket은 동일 Origin만 허용하고
하나의 프로세스/worker에서 실행한다. 이번 버전에는 Tailscale 배포와 사용자별 인증을
포함하지 않는다. 네트워크 공개용으로 bind/Host/Origin 검사를 제거하지 않는다.
개발용 원격 화면 확인은 기존 SSH 터널로 가능하다.

```bash
# 원격 노트북에서 실행. hil-host는 실제 SSH 호스트 이름으로 바꾼다.
ssh -N -L 8765:127.0.0.1:8765 kai@hil-host
```

접속은 노트북의 `http://127.0.0.1:8765`다. ROS DDS 자체의 접근권한은 기존 신뢰된
벤치 네트워크를 전제로 한다. 다른 CLI/GUI가 같은 domain을 조작하지 않도록 한다.

## 가상 LCD 연결

가상 LCD는 ROS 상태로 UNO 화면을 추정하지 않는다. UNO의 적용 표시값·페이지·CAN LOST
보고를 전용 USB serial reader가 읽고, 실제 펌웨어 C++ renderer로 PNG를 만든다.
PC용 renderer를 먼저 빌드한 뒤 colcon으로 설치한다.

```zsh
cd /home/kai/KAI_ws/src/Control
cmake -S firmware/UNO_R4_DASHBOARD/dashboard_preview/native \
  -B firmware/UNO_R4_DASHBOARD/build/host -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/UNO_R4_DASHBOARD/build/host --parallel 2
cmake --install firmware/UNO_R4_DASHBOARD/build/host --prefix software/src/kaiev26_hil

cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --packages-select kaiev26_hil
```

UNO에는 상태 보고 지원 펌웨어가 필요하다. 업로드는 대상 보드를 확인해 별도로 수행한다.
USB protocol과 빌드 방법은 [UNO README](../../../firmware/UNO_R4_DASHBOARD/README.md)를 따른다.
`requirements-web.txt`의 Pillow를 PC PNG 인코딩에 사용한다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch kaiev26_hil hil_web.launch.py \
  start_bench:=true steering:=true state:=true \
  dashboard_port:=/dev/arduino_hil \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/kaiev26_hil/.venv/bin/python
```

`dashboard_port` 기본값은 빈 문자열이다. 명시했을 때만 UNO serial을 연다.
이미 벤치 스택을 실행 중이면 중복 launch하지 않는다. Arduino Serial Monitor도 같은
포트를 동시에 열지 않는다. 이 reader는 `HIL_LCD ON/OFF`만 보내며, 터치나 차량 명령을 보내지 않는다.

왼쪽 **가상 LCD**에서 확인한다. USB OPEN은 포트가 열렸다는 뜻이며 UNO 상태 보고
수신과는 별개다. 새 보고를 1초 이상 받지 못하면 마지막 이미지를 흐리게 표시한다.
반대로 USB 보고가 계속 들어오면서 CAN LOST가 참이면 UNO가 실제 적용한 CAN LOST 화면을 표시한다.

이미지는 같은 drawing 코드와 아셋으로 재구성한 논리 화면이다. 실제 SPI 출력·패널 방향·
RGB/BGR·백라이트·물리 터치까지 증명하지 않는다. 실제 LCD의 partial redraw 잔상 같은
하드웨어/출력 경로 문제도 별도 확인이 필요하다. 터치 입력과 Dashboard 입력 CAN은 후속 범위다.

## 검증

```zsh
cd /home/kai/KAI_ws/src/Control
humble
source ~/KAI_ws/install/setup.zsh
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  software/src/kaiev26_hil/.venv/bin/python -m pytest software/src/kaiev26_hil/test -q
```

Web boundary 시험은 실제 ROS pub/sub 및 command generator를 사용한다.
시험용 peer는 정해진 관측값/ACK만 제공하며 STM_TEST나 물리 차량을 모사하지 않는다.
보드 연결, 실제 CAN/PWM 왕복, 물리 LCD/터치는 별도로 확인해야 한다.

### 2026-09-09 실제 보드 Web HIL 확인

사용자가 구동모터·조향모터·브레이크 서보와 모두 분리된 HIL 벤치임을 확인했다.
`/dev/hil_a`, `/dev/hil_b`, `/dev/hil_test`의 USB 식별자를 현재 udev/runbook과
대조한 뒤, 실제 STM_A/B Agent·STM_TEST bridge·조향 제어기·차량 상태 노드를
Web과 연결했다. 펌웨어 수정/플래시는 수행하지 않았다.

브라우저 조작으로 페달, FWD/REV, MANUAL/AUTO 전환이 보드 상태에 반영됨을 확인했다.
속도 목표 1.0 m/s에서 약 0.99 m/s, 조향 목표 0.1 rad에서 약 0.10001 rad의
에뮬레이터 피드백을 관측했다. DASH/SIDE/RF E-stop 각각에 대해 STM_B 제동 요청과
STM_TEST PWM의 ENGAGED 판정을 확인했다. 제어 브라우저를 닫으면 generator의
`enabled=false`와 조작권 해제를 확인했다.

최초 실행에서 E-stop/KEYA timeout 시나리오가 지연으로 FAIL했다. Web executor가
`spin_once()`로 콜백 하나를 처리한 뒤 5 ms씩 쉬어, 보드 상태와 scenario 구독을
동시에 처리할 때 수신 큐가 밀리는 문제가 있었다. 대기를 1 ms로 줄인 뒤,
**기존 YAML 판정 제한을 변경하지 않고** 실제 보드로 재시험했다.

| Scenario | 재시험 결과 | 조건 변화 관측 시간 |
|---|---|---|
| `smoke_test` | PASS | 기존 구동/조향 통신 조건 충족 |
| `estop_test` | PASS | 주입 예정 시점 이후 약 28 ms에 제동 요청/사유 관측 |
| `ez_can_timeout` | PASS | TX DROP 예정 시점 이후 약 201 ms에 Part I 무효 관측 |
| `keya_can_timeout` | PASS | TX DROP 예정 시점 이후 약 513 ms에 CAN timeout 관측 |

시간은 scenario runner의 ROS 관측 기준이며 물리 액추에이터 반응 시간의 측정이 아니다.
원본 실패 기록과 재시험 PASS JSON은 `~/.local/state/kaiev26_hil/web_results/`에
함께 남겨두었다. 수동 시험 관측값은 같은 상위 디렉터리의
`bench_web_20260909.json`, `bench_web_20260909_recheck.json`에 저장했다.
시험용 스택 종료 후 pedal=0, MANUAL/FWD, DASH E-stop=ON, EZ RUN=0, KEYA disable,
브레이크 PWM 약 1303 μs/ENGAGED로 복귀시켰다.

### 2026-09-09 UNO 가상 LCD 실제 연결

사용자 승인 후 UNO R4 Minima(USB serial `3114101259323437A93033334B57305C`)에
상태 보고 추가판을 업로드했다. UNO의 MANUAL_TFT 적용 상태, CAN LOST=false,
snapshot 누적 증가를 USB에서 수신했고, Web의 가상 LCD에서 480×320 PNG가 표시되는 것을 확인했다.
임시 확인용 serial reader와 브라우저는 종료했다.

사용자 요청에 따라 `dashboard_port:=/dev/arduino_hil`을 포함한 전체 Web HIL launch를
명령 발행 OFF로 실행해 두었다. 실행별 PID·종료 명령·로그는
`~/.local/state/kaiev26_hil/web_runtime_20260909_7x0_c9jo/runtime.json`에 기록했다.
이 기록의 PID는 해당 실행에만 해당하며, 다른 시점에는 현재 실행 상태를 다시 확인한다.
