# KAIEV26 HIL 운용 절차

HIL을 준비하고 Web에서 연결 상태·입력·피드백·시나리오를 확인하는 절차다.
일상 조작은 브라우저에서 하고, 터미널은 설치·빌드·실행·종료와 장치 복구에 사용한다.
화면 구성, USB 표시 보고 계약과 상세 빌드는
[Web Console](../software/src/vehicle_console/HIL_WEB.md)을 따른다.

이 문서는 실제 구동모터·조향모터·브레이크 서보와 분리된 HIL 벤치를 기준으로 한다.
실제 액추에이터를 연결했다면 전원·기구·E-stop 상태를 별도로 확인해야 한다.

## 바로 실행

이 PC의 새 zsh 터미널에서 아래 명령으로 Web과 전체 HIL 벤치·가상 LCD를 실행한다.
`humble`은 `~/.zshrc`에 정의된 함수이며 Humble 환경과 Domain 0을 설정한다.
이미 HIL launch가 실행 중이면 중복 실행하지 않는다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch vehicle_console hil.launch.py \
  start_bench:=true steering:=true state:=true \
  dashboard_port:=/dev/arduino_hil \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/vehicle_console/.venv/bin/python
```

로컬 브라우저: **http://127.0.0.1:8767/**

Tailscale 원격 브라우저: **http://100.112.72.27:8767/**
([원격 접속 준비](#3-원격-브라우저-접속) 필요).

## 1. 실행 구성

```text
Browser ── WebSocket ── hil_web_bridge
                         ├─ /hil/* ── stm_test_bridge ── STM_TEST
                         ├─ command generator ── governor ── STM_A/B · 조향 제어기
                         └─ UNO USB 표시 보고 ── 공통 C++ renderer ── 가상 LCD

STM_TEST GPIO/DAC ──> STM_A/B ── CAN/PWM ──> STM_TEST emulator
STM_A/B ── micro-ROS Agent ──> 구동·브레이크 상태
조향 제어기 ── can0 ──> STM_TEST KEYA emulator
STM_B ── Dashboard CAN ──> UNO ── 실제 LCD
```

`hil.launch.py start_bench:=true` 하나가 Agent 두 개, STM_TEST bridge,
command generator, governor와 Web을 실행한다. `steering:=true`는 조향 제어기,
`state:=true`는 차량 상태 통합, `dashboard_port:=/dev/arduino_hil`은 가상 LCD용
UNO USB reader를 추가한다.

같은 HIL launch의 Web/headless 실행, 별도 Agent나 scenario runner와
중복 실행하지 않는다. 실차용 `control_launch/vehicle.launch.py`도 함께 실행하지 않는다.

## 2. 호스트 최초 1회 준비

### 2.1 런타임 패키지

Ubuntu 22.04 / ROS 2 Humble 환경을 사용한다. Web Python 환경·Node.js·프론트엔드와
LCD renderer 준비는 [Web Console 설치와 빌드](../software/src/vehicle_console/HIL_WEB.md#설치와-빌드),
[가상 LCD 연결](../software/src/vehicle_console/HIL_WEB.md#가상-lcd-연결)을 따른다.
Web 운용에는 X11 forwarding이 필요 없다.

### 2.2 serial 권한

```bash
sudo usermod -aG dialout "$USER"
```

그룹 변경은 기존 셸에 소급 적용되지 않는다. SSH를 완전히 종료하고 다시 접속한 뒤
확인한다.

```bash
id -nG | tr ' ' '\n' | grep -x dialout
```

장치 권한을 임시로 `chmod 666` 하는 방법은 사용하지 않는다.

### 2.3 고정 장치 이름 설치

```bash
cd ~/KAI_ws/src/Control

sudo install -m 0644 firmware/udev/99-kaiev26-stm.rules \
  /etc/udev/rules.d/99-kaiev26-stm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
```

다음 고정 이름을 사용하며 `ttyACM0/1/2/3` 번호는 직접 사용하지 않는다.

```bash
ls -l /dev/hil_a /dev/hil_b /dev/hil_test
ls -l /dev/arduino_hil
readlink -f /dev/hil_a
readlink -f /dev/hil_b
readlink -f /dev/hil_test
readlink -f /dev/arduino_hil
```

| 이름 | 역할 | 현재 기록된 ST-LINK/USB serial |
|---|---|---|
| `/dev/hil_a` | STM_A firmware | `0025002B3234511137333934` |
| `/dev/hil_b` | STM_B firmware | `003600283235510B37333439` |
| `/dev/hil_test` | STM_TEST firmware | `002000203235511537333439` |
| `/dev/arduino_hil` | UNO R4 Dashboard USB serial | `3114101259323437A93033334B57305C` |

STM 세 행은 ST-LINK VCP, Arduino 행은 native USB serial 매핑이다. SWD probe가
실제로 어느 MCU에 연결되는지는 별개이므로 이 표만 보고 플래시하지 않는다.

기존 HIL_TEST SN `002E00253235510837333439`는 2026-08-27 차량 STM_B 교체
보드로 전용되었고, 현재 HIL_TEST는 새 SN `002000203235511537333439`로
배정되어 있습니다.

## 3. 원격 브라우저 접속

### 3.1 Tailscale 직접 접속

현재 HIL PC와 사용자 측 공유 장비 주소는 `100.112.72.27`로 통일했다.
팀원은 Tailscale 연결 후 **http://100.112.72.27:8767/** 를 연다.
SSH 연결이나 VS Code 자동 포트 전달은 필요 없으며, HIL launch는 서버에서 한 번만 실행한다.

HIL 서버에서 최초 한 번 공유를 설정한다. 이미 설정돼 있으면 상태만 확인한다.

```bash
sudo tailscale serve --bg --tcp=8767 tcp://127.0.0.1:8767
tailscale serve status
```

이전 HIL용 8765 전달이 남아 있다면 새 8767 전달을 등록한 뒤 아래 명령으로 제거한다.
Foxglove 기본 포트 8765, Vehicle Console 8766, HIL Console 8767을 사용한다.

```bash
sudo tailscale serve --tcp=8765 off
```

서버 자체는 `127.0.0.1:8767`에서 실행되고, Tailscale Serve가 원격 TCP 연결을 전달한다.
`--bg` 설정은 터미널 종료 뒤에도 유지되지만 HIL 웹 서버는 별도로 실행 중이어야 한다.
장비 공유 초대 수락 또는 같은 tailnet의 접근 권한과 서버 TCP 8767 접근 허용이 필요하다.
접속자 PC의 수신 포트나 공유기 포트포워딩을 열 필요는 없다.
접속자는 조작권 획득도 요청할 수 있으므로 HIL 조작을 맡길 팀원에게 접근을 허용한다.

Tailscale 공유 장비는 사용자 네트워크마다 다른 IP로 보일 수 있다. 위 주소로 peer를
찾지 못하면 **접속자 PC에서** `tailscale status`로 `kai`의 주소를 확인한다.
서버에서 실행한 `tailscale ip -4`만으로 접속자의 주소를 판단하지 않는다.
다른 주소를 사용한다면 웹 서버의 명시적 허용 Host에도 등록하고 빌드 후 웹을 재시작한다.
Host·동일 Origin 검사를 제거하거나 전체 주소를 허용하지 않는다.
상세 설정은 [Web Console Tailscale 공통 주소](../software/src/vehicle_console/HIL_WEB.md#tailscale-공통-주소)를 따른다.

공유만 끄고 로컬 웹 서버를 유지하려면 HIL 서버에서 실행한다.

```bash
sudo tailscale serve --tcp=8767 off
```

### 3.2 SSH 포트 전달로 접속

SSH 방식도 사용할 수 있다. 접속자 PC에서 실행하며 `CONTROL_PC_IP`는
그 PC에서 확인된 HIL 주소나 SSH alias로 바꾼다.

```bash
ssh -N -L 8767:127.0.0.1:8767 kai@CONTROL_PC_IP
```

포트 전달을 유지한 상태에서 접속자 브라우저의 `http://127.0.0.1:8767/`를 연다.
HIL 서버가 이미 실행 중이면 launch를 추가로 실행하지 않는다.

## 4. 공통 ROS 환경

새 실행 터미널에서 한 번 설정한다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh
```

`humble`은 `/opt/ros/humble/setup.zsh`를 불러오고 `ROS_DOMAIN_ID=0`을 설정한다.
현재 STM_A/B firmware도 Domain 0 participant를 만들므로 일반 실행 예제에는
별도의 domain 설정을 반복하지 않는다.

단, 이미 활성화된 셸(`ROS_HUMBLE_ACTIVE=1`)에서는 `humble`이 바로 반환한다.
그 이후 domain을 191 등으로 변경했다면 다시 호출해도 0으로 돌아오지 않으므로,
벤치 실행 전 그 셸에서 `export ROS_DOMAIN_ID=0`으로 복구한다.
개발 테스트의 격리용 domain 지정은 그대로 유지한다.

`humble`이 없는 다른 PC에서는 먼저 이 함수에 해당하는 Humble 환경 로드와 Domain 0
설정을 준비해야 한다. 함수 없이 위 명령을 그대로 사용할 수는 없다.

## 5. 소프트웨어 빌드와 테스트

처음 설치하거나 소스가 바뀌었을 때만 빌드한다. Web 프론트엔드와 native LCD renderer를
[Web Console](../software/src/vehicle_console/HIL_WEB.md)의 순서대로 빌드·설치한 뒤 실행한다.

```zsh
cd ~/KAI_ws
humble
unset COLCON_CURRENT_PREFIX
colcon build --symlink-install --packages-up-to vehicle_console
source ~/KAI_ws/install/setup.zsh
```

개발 중 HIL 소프트웨어 검증은 보드와 분리된 domain에서 한다.

```bash
cd ~/KAI_ws/src/Control
ROS_DOMAIN_ID=191 ROS_LOCALHOST_ONLY=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  software/src/vehicle_console/.venv/bin/python -m pytest software/src/vehicle_console/test/hil -q
```

Web의 시나리오 시험과 위 개발용 테스트는 별개다. 평소 벤치 운용 중에는 테스트 명령을
반복할 필요가 없다.

### STM_TEST 빌드만 확인하기

이 단계는 소스와 ELF를 검증할 뿐 플래시하지 않는다.

```bash
cd ~/KAI_ws/src/Control
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh

cmake -S firmware/STM_TEST --preset Release
cmake --build firmware/STM_TEST/build/Release --clean-first -j4
firmware/STM_TEST/tests/run_host_tests.sh
```

결과 ELF:

```text
~/KAI_ws/src/Control/firmware/STM_TEST/build/Release/STM_TEST.elf
```

현장 보드와 SWD target을 확인하기 전에는 이 ELF를 플래시하지 않는다.

## 6. 실행 전 점검

장치 이름과 접근 권한은 터미널에서 확인한다.

```bash
ls -l /dev/hil_a /dev/hil_b /dev/hil_test /dev/arduino_hil
```

기존 실행이 남아 있다면 해당 launch 터미널에서 `Ctrl+C`로 종료한다. 실행 주체를
모를 때만 아래에서 PID·부모·프로세스 그룹과 명령을 확인한다. 확인되지 않은 프로세스를
종료하거나 전역 `pkill`을 사용하지 않는다.

```bash
ps -eo pid,ppid,pgid,args | rg 'hil_web_bridge|stm_test_bridge|micro_ros_agent|hil_command_generator|steering_actuator_node|command_governor'
```

조향을 포함하려면 launch 전에 [SocketCAN 준비](#101-socketcan-준비)를 완료한다.

## 7. Web 벤치 실행

[공통 ROS 환경](#4-공통-ros-환경)을 설정한 터미널에서 전체 벤치를 실행한다.

```zsh
cd ~/KAI_ws
humble
source ~/KAI_ws/install/setup.zsh

ros2 launch vehicle_console hil.launch.py \
  start_bench:=true steering:=true state:=true \
  dashboard_port:=/dev/arduino_hil \
  web_python:=/home/kai/KAI_ws/src/Control/software/src/vehicle_console/.venv/bin/python
```

로컬 브라우저: **http://127.0.0.1:8767/**

Tailscale 원격 브라우저: **http://100.112.72.27:8767/**
([원격 접속 준비](#3-원격-브라우저-접속) 필요).

- 조향/상태 통합을 사용하지 않을 때만 해당 옵션을 생략한다.
- UNO를 연결하지 않을 때는 `dashboard_port`를 생략한다.
- 별도 Agent를 의도적으로 운용 중인 구성에서는 `agents:=false`를 준다.
- 화면만 확인하려면 `start_bench`, `steering`, `state`, `dashboard_port`를 생략한다.

Web 전용 실행은 주행 명령 **DISARMED**로 시작한다. Web 브라우저는 처음에 모니터링
모드이며 상단 **조작권 가져오기**를 눌러야 입력을 변경할 수 있다.

## 8. Web에서 연결 확인

| 확인할 내용 | Web 위치 | 정상 확인 기준 |
|---|---|---|
| STM_TEST · STM_A · STM_B 연결 | **벤치** 연결 상태 | 각 상태가 새로 수신됨 |
| 페달 입력 | **벤치·구동 → 페달 관측 · STM_A** | STM_A 수신 비율·ADC 환산 전압 확인 |
| EZ · KEYA CAN | **구동·조향**, **진단·로그** | RX 증가와 최근 RX age 확인 |
| ROS 토픽 수신 주기 | **진단·로그** | Hz·주기·마지막 수신 이후·누적 값 확인 |
| EZkontrol Part I·II | **구동**의 Part I·II 표시 | 각각 VALID와 CAN age 확인 |
| STM_TEST firmware protocol | 화면 하단 | 현재 firmware는 protocol 3 |
| 보드 명령·상태·해제 사유 | **구동·조향·브레이크/안전** | 요청과 관측값, 상태 플래그 대조 |
| UNO 표시 상태 | **가상 LCD** | USB OPEN, UNO LIVE, Dashboard CAN 상태 확인 |

CAN 초기화 OK만으로 상대와 통신한다고 판단하지 않는다. KEYA 제어기를 생략한 구성은
`NO RX`일 수 있다. `—`는 미수신·무효·미지원 값이며 0과 다르다.
ROS Hz는 PC 수신 간격, Part I/II age는 STM_A가 보고한 마지막 CAN 수신 경과시간이다.
상단 새로고침 버튼으로 STM_TEST STATUS를 요청할 수 있다.
상단 E-STOP은 STM_TEST의 DASH/SIDE/RF 입력 합산이다. CLEAR는 모두 해제,
ACTIVE는 하나 이상 입력, UNKNOWN은 수신 지연·미확인이다. 물리 비상정지 상태의 증명은 아니다.
Desktop에서도 **Controls**로 도크를 접어 중앙 영역을 넓힐 수 있으며 초안과 조작권은 유지된다.

## 9. 기본 기능 시험

속도는 **km/h**, 조향은 **degree**로 입력한다. 조작 후에는 ACK와 실제 관측값을
함께 확인한다. Mode/Gear/Pedal/E-stop은 모든 페이지의 **Control Dock**에서,
주행 목표는 Control Dock 하단의 **주행 명령**에서, engineering 설정은 각 기능 페이지에서 조작한다.
패널이 길면 아래로 스크롤하고, 좁은 화면에서는 상단 **Controls**로 패널을 연다.

### 9.1 Bench Reset

기존 Safe State의 UI 명칭을 바꾼 것이며 backend 동작은 동일하다.

상단 **조작권 가져오기 → Bench Reset**을 누른다. Web이 주행 명령 발행을 해제하고
bridge 기본 입력을 큐에 넣는다.

기본 입력은 MANUAL/FWD, 페달·motion·조향 초기값 0, E-stop OFF, CAN ON,
Plant ON, fault/TX DROP OFF다. 보드 연결이 끊겼다면 재연결 후 적용된다.
Bench Reset은 E-stop 해제도 포함하므로 물리 비상정지와 용도가 다르다.

### 9.2 AUTO 속도 폐루프 시험

1. **구동** 페이지를 열고 오른쪽 Control Dock에서 AUTO와 FWD를 선택한다.
2. **구동 → 에뮬레이터 응답 설정**에서 폐루프 시뮬레이션을 선택한다.
3. **주행 명령**에 낮은 목표 속도(예: 3.6 km/h)와 조향각 0°를 입력한다.
4. **ARM · 명령 발행**을 누르고 표시된 목표를 확인해 발행한다.
5. **구동 명령과 응답**에서 목표와 관측값을 비교한다. 왼쪽 축은 km/h, 오른쪽 축의 점선은 RPM이다.
6. 시험을 마치면 **발행 해제**를 누른다.

### 9.3 MANUAL pedal 폐루프 시험

1. Control Dock 하단의 주행 명령을 **발행 해제**하고, Control Dock에서 MANUAL을 선택한다.
   에뮬레이터 응답 설정은 폐루프 시뮬레이션으로 둔다.
2. Control Dock의 페달을 낮은 값(예: 10%)으로 설정한다. 슬라이더는 놓을 때 한 번 전송한다.
3. STM_A의 페달/ADC raw, 명령 상전류와 STM_TEST의 RPM 응답을 확인한다.
4. Control Dock 페달을 0%로 놓고 필요하면 **Bench Reset**으로 복귀한다.

이 시험은 STM_TEST DAC → STM_A ADC → EZ CAN 명령 → 에뮬레이터 피드백 경로를 확인한다.

### 9.4 직접 피드백 주입

**구동 → 에뮬레이터 응답 설정**에서 **직접 피드백**을 선택하고 속도 또는 RPM을
입력·적용한다. 전압·전류·온도는 같은 패널의 펼침 영역에서 설정한다.
bridge는 폐루프를 끄기 직전 RPM을 보관해 재연결 시 과거 주입값으로 되돌아가지 않게 한다.

### 9.5 브레이크와 E-stop

- Host 제동 시험: AUTO에서 **주행 명령**의 목표 속도·조향각을 0으로 두고
  **브레이크 요청**을 켠 뒤 명령을 발행한다. 이후 목표 변경도 적용 버튼으로 전송한다.
- E-stop 시험: **브레이크 · 안전**을 열고 Control Dock의 DASH/SIDE/RF 입력을
  각각 켜고 해제하며 관측한다.
- **브레이크 · 안전**에서 STM_B 제동 요청, PWM 판정·펄스 폭·주기와 정지 사유를 확인한다.
- 시험 후 브레이크 요청을 해제·적용하고 주행 명령을 **발행 해제**한다.

PWM 관측은 서보의 실제 위치 피드백이 아니다. CAN ON/OFF·TX DROP·컨트롤러 fault는
**고장 주입** 화면에서 시험하며, 종료 후 **고장 주입 해제** 또는 **Bench Reset**으로 복귀한다.

## 10. KEYA 포함 전체 HIL

전체 Web 실행의 `steering:=true`가 조향 제어기를 포함한다. 별도 조향 launch를
추가 실행하지 않는다. Web에서 할 수 없는 can0 설정·복구는 아래 절차를 따른다.

### 10.1 SocketCAN 준비

권장: 호스트에 1회 systemd-networkd 설정을 설치하면 부팅과 USB-CAN 재연결
때마다 can0가 자동으로 준비되어 아래 수동 명령이 필요 없어진다. launch 전에
can0가 안 올라와 있으면 조향 노드가 CAN 열기에 한 번 실패한 뒤 재시도 없이
뜨므로, 자동화가 이 실수 자체를 없앤다.

```bash
cd ~/KAI_ws/src/Control
sudo install -m 0644 host/systemd/80-can0-kaiev26.network /etc/systemd/network/
sudo systemctl enable --now systemd-networkd
ip -details link show can0   # state UP, bitrate 250000 확인
networkctl status can0       # SETUP=configured, Network File이 위 파일이면 관리 인수 완료
```

**UP + bitrate만으로는 부족하다.** 2026-08-19에 인터페이스가 UP이고 bitrate도
정상인데 adapter의 USB 데이터 경로가 죽어 양방향 모두 0프레임인 상태를 겪었다
(아래 문제 해결 참조). 설정 후 Web의 KEYA RX/TX와 수신 age까지 확인한다. raw frame 확인이 필요할 때만
[문제 해결](#14-문제-해결)의 `candump`를 사용한다.

2026-08-18 bench PC에 설치했다. `networkctl`에서 can0가 이 설정 파일로
configured 상태인 것과 다른 인터페이스가 전부 unmanaged(NetworkManager 충돌
없음)인 것을 확인했다. 부팅·USB 재연결 자동 적용은 networkd가 링크 출현 시
설정을 다시 적용하는 동작이며, 다음 재부팅에서 자연 확인된다. 실차 PC에도
셋업 시 같은 명령으로 설치한다.

> **주의:** 같은 날 networkd 활성화 직후 systemd-resolved의 per-link DNS
> 등록이 전부 비어 DNS 해석이 죽는 현상이 있었다(IP 직결은 정상). resolved와
> NetworkManager는 재시작되지 않았고 정확한 소거 경로는 확정하지 못했다.
> 복구는 NM이 이미 아는 DHCP DNS 값을 되돌려 넣는 두 줄이며 연결이 끊기지
> 않는다:
>
> ```bash
> nmcli dev show <wifi_if> | grep IP4.DNS   # 값 확인
> sudo resolvectl dns <wifi_if> <DNS1> <DNS2>
> sudo resolvectl default-route <wifi_if> yes
> ```
>
> `systemctl restart NetworkManager`도 복구되지만 wifi가 수 초 끊겨 SSH
> 세션이 얼어붙으므로 원격 작업 중에는 쓰지 않는다.
>
> 2026-08-18 재부팅으로 판정 완료: can0는 수동 명령 없이 이 설정 파일로
> 자동 준비됐고(bitrate 250000), wifi DNS와 tailscale MagicDNS 모두 정상
> 재등록됐다. DNS 소거는 networkd를 처음 활성화하는 순간에만 나타나는
> 일회성 부작용이며, 새 호스트에 설치할 때는 위 resolvectl 복구를 바로
> 실행하면 된다. 부팅 재발이 없으므로 networkd 방식을 확정한다.

수동으로 할 때는 다음과 같다.

```bash
ip link show can0
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
ip -details link show can0
```

`state ERROR-ACTIVE`가 정상이다.

`restart-ms`는 주지 않는다. 이 bench의 USB-CAN은 gs_usb(candleLight 계열)이고
해당 드라이버는 bus-off 자동 복구를 구현하지 않으므로
`Error: Device doesn't support restart from Bus Off`로 거부된다. Host가 bus-off에
빠지면 수동으로 복구한다.

```bash
sudo ip link set can0 down && sudo ip link set can0 up
```

### 10.1.1 can0을 내리면 STM_TEST 송신이 멈췄다가 스스로 복구된다

CAN은 수신 노드가 ACK를 주어야 송신이 완료된다. `can0`을 내리면 STM_TEST FDCAN2는
ACK를 받지 못해 TX FIFO 3칸이 그대로 차고 송신이 멈춘다. `keya_tx`가 정확히 3만큼
오른 뒤 정지하는 것이 이 상태다.

**보드 리셋은 필요 없다.** `can0`을 다시 올리면 송신이 즉시 재개된다.

2026-08-14 실측(firmware protocol 3)에서 확인한 값이다.

| 시점 | `keya_tx` | `keya_busoff` | `keya_errpass` | `keya_tec` |
|---|---:|---:|---:|---:|
| `can0` down 직전 | 362 | 0 | 0 | 0 |
| down 직후 | 365 | 0 | 1 | 128 |
| 25초 경과 | 365 | 0 | 1 | 128 |
| `can0` up 직후 | 365 | 0 | 0 | 125 |
| 이후 | 재개 | 0 | 0 | 0으로 감소 |

이것은 bus-off가 아니라 **error-passive**다. CAN 규격의 fault confinement 예외 조항
때문에, error-passive 송신자가 ACK 에러를 겪을 때는 TEC를 올리지 않는다. 그래서
TEC가 정확히 128에서 고정되고 bus-off 문턱인 255에 도달하지 못한다.

TEC는 성공 송신 1건당 1씩 줄어든다. KEYA heartbeat은 1 Hz라 TEC가 0까지 돌아오는 데
약 2분 걸리지만, 송신 자체는 `can0` up 즉시 재개되므로 기능상 기다릴 필요는 없다.

판정에는 `keya_busoff`, `keya_errpass`, `keya_tec`을 쓴다. `keya_ok`는 FDCAN 주변장치
초기화 결과일 뿐이라 이 상태에서도 계속 1이다.

FDCAN1(EZ)도 `WAIT_HANDSHAKE`에서 `0x55`를 무조건 송신하므로 STM_A가 없으면 같은
상태가 된다. 증상은 `ez_tx`가 3에서 멈추는 것이고, 판정 field는 `ez_*`로 같다.

> transceiver를 물리적으로 분리한 경우는 확인하지 않았다. 그때는 ACK 에러가 아니라
> bit error가 나므로 위 예외가 적용되지 않고 실제 bus-off로 갈 수 있다.

### 10.2 조향 연결 확인

**조향** 페이지의 수신 상태와 **조향 보드 관측**에서 통신 정상, 피드백 유효, RX age를 확인한다.
**진단·로그**의 KEYA CAN 항목은 peripheral 초기화와 실제 RX, TEC/REC, bus-off를 구분한다.
KEYA 모터축 추종은 EZ 폐루프/직접 피드백 선택과 독립적이다.

### 10.3 조향 폐루프 시험

1. **조향**을 열고 Control Dock에서 AUTO를 선택한다. 주행 목표 속도를 낮게 설정한다(예: 3.6 km/h).
2. 명령을 발행하고 **관측 속도가 1 km/h 이상**, 조향 통신·피드백이 유효한지 확인한다.
3. 목표 조향각을 입력한다(예: 5.73°). **목표 변경**으로 적용한다.
4. **로드휠 위치 추종**에서 명령각과 관측각의 수렴을 확인한다.
5. **조향 보드 관측**에서 ACTIVE, 해제 사유 NONE을 확인한다.
   모터축 움직임은 **조향 → KEYA 모터축 피드백**에서 별도로 본다.
6. 시험 후 **발행 해제**, 필요하면 **Bench Reset**으로 복귀한다.

조향은 AUTO와 유효한 관측 속도 조건을 충족해야 활성화된다. 해제 상태라면
목표를 반복 입력하기보다 보드가 보고한 해제 사유를 먼저 확인한다.
이 시험은 통신과 에뮬레이터 추종을 확인하며 실제 조향 기구의 캘리브레이션을 증명하지 않는다.

## 11. 가상 LCD 확인

왼쪽 **가상 LCD**를 연다. UNO가 적용한 MANUAL/AUTO 페이지와 표시값이 480×320 이미지로
재구성된다. USB OPEN만으로 상태 보고 수신이 확인되는 것은 아니므로 **UNO LIVE**와
보고 수신 경과시간을 함께 확인한다.

USB 보고가 살아 있으면서 Dashboard CAN이 끊기면 CAN LOST 화면을 표시한다.
USB 보고 자체가 끊기면 마지막 이미지를 흐리게 표시한다.
실제 패널·SPI·백라이트와 물리 터치 검증은 별도로 필요하다.

## 12. Web 시나리오 실행

1. **시나리오**에서 시험을 선택하고 단계·관측 조건을 읽는다.
2. 해당 시험의 주행 명령 발행을 허용하고 **시나리오 실행 → ARM · 시험 시작**을 누른다.
3. 단계별 결과와 최종 PASS/FAIL을 확인한다. 실패하면 관측 조건과 **진단·로그**를 함께 본다.
4. 중단하려면 **중단 · Bench Reset**을 누른다. 종료·중단 시 Web이 명령 발행 해제와
   Bench Reset을 요청한다.
5. 현재 결과와 이전 시험 기록은 화면에서 JSON으로 다운로드한다.

시나리오 실행 중에는 수동 입력이 잠긴다. 기본 시험은 `smoke_test`, `estop_test`,
`ez_can_timeout`, `keya_can_timeout`이다. timeout 시험은 주입 전 정상 통신 조건도 검사한다.

## 13. 정상 종료 순서

1. 실행 중인 시험은 **중단 · Bench Reset**으로 끝낸다.
2. Control Dock 페달을 0%로 놓고, 그 아래 주행 명령의 **발행 해제**를 확인한다.
   필요하면 상단 **Bench Reset**을 적용한다.
3. **조작권 해제**를 누른다. 이 동작도 명령 발행 해제와 기본 입력 적용을 요청한다.
4. 전체 서버를 끄려면 launch 터미널에서 **Ctrl+C**를 누른다. 실행별 종료 명령을
   별도로 받은 경우에는 그 launch PID의 종료 명령을 사용한다.
5. 실제 전원을 끄기 전에는 벤치 배선과 액추에이터 전원 상태를 확인한다.

브라우저 종료는 조작권 해제이며 서버 종료와 다르다. 서버·Agent·USB reader까지
종료하려면 launch를 종료해야 한다. 명령 발행 OFF는 MANUAL 입력과 모든 MCU 출력을
자동 차단한다는 뜻이 아니다.

## 14. 문제 해결

### Web 페이지가 열리지 않음

launch 터미널의 오류와 접속 주소를 먼저 확인한다. Tailscale 직접 접속이라면
접속자 PC의 일반 터미널에서 확인한다(VS Code의 원격 터미널이 아님).

```bash
tailscale ping --c 3 100.112.72.27
curl -v --connect-timeout 5 http://100.112.72.27:8767/api/health
```

- `no matching peer`: 해당 PC에서 장비가 다른 주소로 보이거나 공유·연결이 없는 상태다.
  그 PC의 `tailscale status`로 실제 주소를 먼저 확인한다.
- 연결 시간 초과: TCP 연결 경로, Tailscale 접근 정책, 방화벽과 서버 공유 상태를 확인한다.
  ping 성공만으로 TCP 8767 접근이 허용됐다고 판단하지 않는다.
- HTTP 400: 접속 주소가 웹 서버의 허용 Host에 있는지와 수정 후 재시작 여부를 확인한다.
- HTTP 200이지만 실시간 연결 실패: WebSocket 오류와 Host·Origin을 확인한다.

HIL 서버에서는 `tailscale serve status`로 공유 설정을 확인한다.
SSH 포트 전달 방식이라면 해당 터널이 유지되는지 확인한다.
서버의 실제 listen 상태가 필요할 때는 다음 명령을 사용한다.

```bash
ss -ltnp 'sport = :8767'
```

화면은 새 버전인데 수신 Hz·주기·누적이 계속 `—`라면 기존 Web 백엔드가 남아 있을 수 있다.
입력 시험을 종료한 뒤 launch를 재시작하고 브라우저를 새로고침한다.

### 보드 상태가 수신되지 않음

**벤치** 연결 상태와 **진단·로그**를 먼저 확인한다. 그다음 해당 장치가 존재하고
다른 프로세스에 점유되지 않았는지 확인한다.

```bash
ls -l /dev/hil_a /dev/hil_b /dev/hil_test /dev/arduino_hil
fuser /dev/hil_a /dev/hil_b /dev/hil_test /dev/arduino_hil
```

Web과 Agent가 Domain 0인지, launch 로그에 STM_A/B XRCE session 생성이 있는지 확인한다.
같은 serial을 별도 터미널·Serial Monitor가 열고 있으면 그 소유자가 정상 종료해야 한다.

### topic traffic은 있는데 typed message가 깨짐

Host `kaiev26_msgs`와 MCU에 생성된 micro-ROS type support가 다를 수 있다.
이 확인은 Web에 없으므로 provenance를 직접 대조한다.

```bash
cat ~/KAI_ws/src/Control/firmware/tools/microros_manifest.txt
git -C ~/KAI_ws/src/kaiev26_msgs rev-parse HEAD
```

서로 다르면 재생성·빌드와 대상 보드 업데이트 범위를 확인한다.
재생성만으로 현장 플래시 권한이 생기는 것은 아니다.

### KEYA가 NO RX이거나 조향이 활성화되지 않음

먼저 **조향 보드 관측**의 해제 사유와 **진단·로그**의 KEYA RX age·TEC/REC·bus-off를 본다.
AUTO, 관측 속도 1 km/h 이상, 유효한 피드백, fresh 명령 등의 조건을 확인한다.

Web만으로 Host USB-CAN 상태를 구분하지 못할 때는 raw traffic을 확인한다.

```bash
ip -details -statistics link show can0
candump -T 3000 can0
```

- can0가 DOWN 또는 BUS-OFF면 [SocketCAN 준비](#101-socketcan-준비)에 따라 복구한다.
- `keya_errpass=1`, TEC 128 부근에서 TX가 멈추면 ACK가 없는 상태일 수 있다.
  [기존 error-passive 관측](#1011-can0을-내리면-stm_test-송신이-멈췄다가-스스로-복구된다)을 참고한다.
- STM_TEST의 TX는 늘고 TEC는 0인데 Host RX/TX가 멈췄다면 USB-CAN adapter의 USB
  경로 문제를 의심한다. 2026-08-19에는 link down/up과 USB 재열거로 복구되지 않았고
  adapter의 물리 재연결 후 복구됐다. 원격에서 배선·전원을 임의로 변경하지 않는다.

명령 충돌이 의심될 때만 endpoint 소유자를 확인한다. 정상 `/vehicle/command` 발행자는
`command_governor` 하나이며, Web의 연결 상태 표는 publisher 목록을 대신하지 않는다.

```bash
ros2 topic info -v /vehicle/command
ros2 topic info -v /planning/command
```

### 가상 LCD에 UNO 보고가 없음

**가상 LCD → 디스플레이 연결**에서 오류를 확인한다. launch에 `dashboard_port`를
지정했는지, UNO가 상태 보고 지원 펌웨어인지, PC용 native renderer를 빌드·설치했는지
확인한다. USB OPEN이지만 보고가 없으면 다른 ROS 값으로 정상 표시를 추정하지 않는다.
빌드·프로토콜은 [Web Console 가상 LCD 연결](../software/src/vehicle_console/HIL_WEB.md#가상-lcd-연결)을 따른다.

## 15. 진행 상태와 남은 작업

### 완료

- `/hil/status.protocol=3`와 EZ command observation field 확인
- KEYA CAN2 transceiver 설치, 양방향 통신 및 조향 폐루프 검증(10.3)

### 2026-09-08 재배선 bench 확인

사용자가 모든 실제 액추에이터와 분리된 bench임을 확인한 상태에서 수행했다.
2.3절의 네 USB serial/고정 이름을 대조하고, STM 세 보드의 SWD Device ID
`0x469`, 개별 MCU 식별 값과 전압(A 3.29 V, B 3.27 V, TEST 3.30 V)을 확인했다.
STM_A/B/TEST Release 이미지는 기록 후 verification과 reset을 완료했다.
UNO R4는 Renesas core 1.6.0으로 빌드(Flash 217,032 bytes)하고 DFU upload를 완료했다.

| 확인 경로 | 관측 결과 |
|---|---|
| TEST DAC → A ADC → EZ CAN | pedal 25%에서 ADC 1176, command 120 A, emulator RPM 1118; 전류/RPM은 모사 값 |
| TEST mode/gear → A | MANUAL/AUTO 전환과 REVERSE 입력 반영; 후진 command -120 A/-1500 RPM |
| TEST E-stop 3선 → B → A UART | DASH/SIDE/RF 각각 brake=true, stop_request=true, EZ RUN=0; B GPIO PC0/PC1/PB4의 LOW도 각각 확인 |
| AUTO 속도 폐루프 | target 1.0 m/s → feedback 약 0.995 m/s |
| Host → KEYA CAN2 → TEST | target 0.1 rad → feedback 약 0.10001 rad, communication_ok=true |
| AUTO Host brake | brake=true, reason=1, traction RUN=0, steering disable |
| TEST → A CAN → B UART → Arduino CAN | 주입한 52.3 V/12.5 A/40 °C 수신, vehicle/detail=OK, Part I/II=VALID, ignored=0 |
| B PWM → TEST PA6 | 주기 약 19,980 µs, engage 약 1303 µs, release 약 1663 µs, pwm_lost=false |

EZ/KEYA firmware의 TEC/REC와 bus-off는 시험 샘플에서 모두 0이었다.
Host can0의 bus-errors/error-warn/error-pass/bus-off도 0이었다. Host RX dropped는
누적 107이며 시험 전 기준값이 없어 이번 시험에서 발생한 것으로 단정하지 않는다.

**수정 및 재검증:** STM_B 소스의 55°/120° 출력(1305/1665 µs)에 맞춰
STM_TEST의 브레이크 판정값을 동기화했다. 허용 오차 ±50 µs와 timeout 100 ms는
유지했다. 기존 host logic test와 Release 빌드를 통과하고 HIL TEST 재플래시 및
verification을 완료했다. DASH E-stop ON → OFF → ON 시험에서 실제 수신
1303 → 1662 → 1303 µs가 `ENGAGED → RELEASED → ENGAGED`로 판정되고,
세 상태 모두 `pwm_lost=0`임을 확인했다. 기존 `UNKNOWN` 불일치는 해결됐다.

LCD 실제 화면과 CTP 터치 배선/전압은 미검증이다. 현재 Arduino firmware에는
터치 입력 처리가 없다. 저항계로 종단 60 Ω 및 전원/접지 연속성을 측정하지 않았다.
시험 후 pedal=0, MANUAL/FWD, DASH E-stop=ON, traction RUN=0, KEYA disable로 두고
시험용 ROS launch와 모든 자식 프로세스를 종료했다.

### 2026-09-09 Web HIL·UNO 가상 LCD 확인

실제 액추에이터와 분리된 벤치에서 Web 입력의 보드 반영, 속도·조향 폐루프,
E-stop 3종과 PWM, 브라우저 종료 시 명령 발행 해제를 확인했습니다.
Web ROS callback 처리 지연을 수정한 뒤 기존 판정 제한 그대로
`smoke_test`, `estop_test`, `ez_can_timeout`, `keya_can_timeout`이 모두 PASS했습니다.
UNO 상태 보고 추가판을 업로드하고 실제 USB 보고로 480×320 가상 LCD 표시까지 확인했습니다.
세부 결과와 한계는 [Web Console 검증 기록](../software/src/vehicle_console/HIL_WEB.md#검증)을 따릅니다.

### 남은 작업

- 새 LCD 실제 표시 및 터치 모듈 모델/전압/입력 기능 확인
- 조향 캘리브레이션 실측 검증. KEYA emulator는 선형이라 HIL로는 불가능하며
  실제 조향 기구가 필요하다
- transceiver 물리 분리 시 FDCAN이 실제 bus-off로 가는지 확인. `can0` down은
  error-passive에서 자동 복구되는 것을 2026-08-14에 실측했으나, bit error가 나는
  이 경우는 미확인이다. bus-off로 확인되면 복구 로직이 필요하다
- 실제 CAN 분리·전원 변화 등 기존 TX DROP 시나리오 밖의 장치 고장 조건 확인.
  `keya_can_timeout` 전제조건 추가판은 2026-09-09 Web HIL 재시험에서 PASS했다

### 플래시 전 SWD target 확인

serial symlink만 근거로 플래시하지 않는다. 대상 MCU는 플래시를 읽어 확인한다.

```bash
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh
STM32_Programmer_CLI -l                       # probe SN 목록

STM32_Programmer_CLI -c port=SWD sn=<SN> mode=hotplug \
  -u 0x08000000 0x18800 /tmp/target_read.bin
strings -n 5 /tmp/target_read.bin | grep -E 'stm_a_node|stm_b_node|drive/status|brake/status'
```

node 이름과 토픽 문자열이 firmware에 그대로 박혀 있으므로 어느 보드인지 확정할 수
있다. 2026-08-12 STM_A 플래시는 이 방법으로 target을 확인한 뒤 수행했다.

### RuntimeStats firmware의 micro-ROS baud

RuntimeStats를 발행하는 STM_A/B는 LPUART1 **460800 baud**입니다. `hil_bench`,
`hil_web`, `vehicle` launch의 `agent_baud` 기본값을 동일하게 사용합니다. 이전
115200 firmware를 실행할 때만 `agent_baud:=115200`을 명시하십시오. 두 Agent를
기존 속도로 둔 채 MCU만 새 firmware로 바꾸면 통신이 복구되지 않습니다.
STM_TEST와 Arduino serial 및 STM_A↔STM_B USART1은 여전히 115200입니다.
