# Control firmware

STM_A는 종방향 모터와 차량 입력을 담당하고, STM_B는 물리 E-stop과 브레이크
서보를 담당합니다. 두 보드는 USART1으로 차량 상태와 최종 브레이크 요청을 서로
교환하며, 각 보드는 LPUART1 460800 baud로 별도의 micro-ROS Agent에 연결됩니다.
보드 간 USART1과 STM_TEST/Arduino serial은 115200을 유지합니다.

## 디렉터리 구조

```text
firmware/
├── STM_A/             # 차량 입력과 EZkontrol 구동 모터 제어
├── STM_B/             # E-stop·brake와 CAN dashboard를 담당하는 production STM_B
├── STM_TEST/          # STM_A/B 입출력과 EZ/KEYA CAN을 대신하는 HIL emulator
├── UNO_R4_DASHBOARD/  # UNO R4 Minima CAN receiver와 MSP4030 UI
├── common/            # STM 공통 주기·성능 계측
├── KEYA/              # 조향 드라이버 설정·프로토콜 자료
├── micro_ros_agent/   # vendored micro-ROS Agent
├── tools/             # micro-ROS 라이브러리 재생성 보조 스크립트
└── udev/              # STM 보드 고정 장치 이름 규칙
```

보드 내부 Task와 Peripheral은
[`STM_A/README.md`](STM_A/README.md),
[`STM_B/README.md`](STM_B/README.md),
[`UNO_R4_DASHBOARD/README.md`](UNO_R4_DASHBOARD/README.md),
[`STM_TEST/README.md`](STM_TEST/README.md)에 정리되어 있습니다.

STM_A와 STM_B는 실차 기준 펌웨어이고, STM_TEST는 실차 없이 두 보드의 입출력을
시험하기 위한 bench 전용 펌웨어입니다. STM_B는 safety/UART/micro-ROS 경로와
별개인 저우선순위 CAN dashboard task로 표시 snapshot을 송신하고,
UNO_R4_DASHBOARD가 이를 받아 MSP4030 UI를 렌더합니다. UNO_R4_DASHBOARD는 Arduino
Renesas UNO core로 빌드합니다. STM_TEST는 LPUART1 ASCII protocol로 PC와 직접
통신하므로 Agent가 필요 없습니다. 이전 직접-SPI LCD STM_B는
[`../archive/firmware/STM_B_DIRECT_LCD`](../archive/firmware/STM_B_DIRECT_LCD)에
보관합니다.

STM 공통 주기·성능 계측은 [common](common/README.md)을 따른다.

## 보드와 핀 표기

STM_A·STM_B·STM_TEST 세 보드는 NUCLEO-G474RE(MB1367, STM32G474RETx)를 사용합니다. 각 README의 핀맵은
MCU 핀 이름과 ST morpho 헤더 번호를 함께 적습니다.

- `CN7`, `CN10`은 각각 2×19 ST morpho 헤더이고 홀수 번호와 짝수 번호가 각각 한
  열을 이룹니다. 예를 들어 `PA11`은 `CN10-14`입니다.
- 괄호 안의 `A0`, `D7` 같은 표기는 같은 신호가 나오는 Arduino Uno 헤더
  실크스크린 이름입니다. 표기가 없는 핀은 morpho 헤더에서만 나옵니다.
- GND는 `CN7-8`, `CN7-19`, `CN7-20`, `CN7-22`, `CN10-9`, `CN10-20`에 있습니다.
- 커넥터의 물리적 배치와 solder bridge 옵션은 보드 매뉴얼 UM2505를 기준으로
  합니다. 저장소에는 해당 매뉴얼 사본이 없습니다.

세 보드를 서로 연결하는 핀 짝은 각 README의 `4.4 배선 상대`에 있습니다.

## 명령과 안전 경로

상위 Planning 계층은 `kaiev26_msgs/msg/ActuatorCommand`를
`/planning/command`로 발행하고, Host의 `command_governor`가 이를
`/vehicle/command`로 재발행합니다. `/vehicle/command`의 발행자는
`command_governor` 하나입니다. Planning의 production 구현은 이 저장소에 포함하지 않으며
실차 시험에서는 `vehicle_test_scenario_runner`, HIL에서는
`hil_command_generator`가 그 자리를 대신합니다.

- STM_A는 `speed_target_mps`와 `brake_engage`를 직접 받습니다.
- STM_B는 같은 메시지의 `brake_engage`를 독립적으로 받습니다.
- STM_B는 물리 모드 상태를 STM_A의 UART vehicle-status frame에서 받습니다.
- STM_B는 물리 E-stop과 AUTO의 `brake_engage`를 OR한 최종
  `BRAKE_REQUEST`로 브레이크 서보를 움직이고, 같은 값을 UART safety-status
  frame으로 STM_A에 보냅니다.
- STM_A는 AUTO의 직접 `brake_engage`, STM_B의 `BRAKE_REQUEST`, STM_B UART
  timeout 중 하나라도 참이면 종방향 모터에 정지·HALTED 명령을 보냅니다.

브레이크 서보를 직접 구동하는 보드는 STM_B뿐입니다. STM_A는 브레이크 서보를
구동하지 않습니다.

MANUAL에서는 Host의 `brake_engage`가 브레이크 서보와 수동 페달 구동을 정지시키지
않습니다. 물리 E-stop만 MANUAL의 브레이크 서보를 작동시킵니다. 따라서 AUTO에서
Host 브레이크가 걸린 상태로 MANUAL로 전환하면, upstream이
`brake_engage=false`를 다시 보내지 않아도 Host 원인의 브레이크는 해제됩니다.

STM_A의 종방향 애플리케이션 명령 주기는 50 ms, 즉 20 Hz입니다. 이것은
EZkontrol 모터 드라이버 내부의 전류 제어 주기가 아니라 STM_A가 속도 또는 토크
목표를 갱신하는 주기입니다.

## EZkontrol CAN

STM_A가 EZkontrol 모터 드라이버와 CAN으로 통신합니다.

- STM_A는 모터 드라이버의 handshake 요청을 받고 ACK 전송에 성공하기 전까지
  traction 명령을 보내지 않습니다.
- STM_A는 EZkontrol status Part I과 Part II를 모두 수신합니다.
- Part I은 RPM, bus voltage, bus current와 phase current를 갱신합니다.
- Part II는 RUN 상태, control mode, controller·motor temperature와 fault bits를
  갱신합니다.
- STM_A는 이 값을 `/drive/status`의 `DriveActuatorStatus`로 발행합니다.

STM_B는 EZkontrol CAN Part I/II를 수신하지 않습니다. STM_B가 표시하는 속도,
기어와 모드는 STM_A가 USART1 vehicle-status frame으로 보낸 값입니다.

## STM 빌드

세 STM 프로젝트의 CMake preset은 모두 GNU Arm Embedded `arm-none-eabi` toolchain을
사용합니다. STM_A preset은 `/opt/st/stm32cubeclt_1.20.0/`의 compiler 경로를
명시하고, STM_B와 STM_TEST preset은 `PATH`에서 compiler를 찾습니다.

아래 명령은 권장 워크스페이스인 `~/KAI_ws` 루트에서 실행합니다.

```bash
cd ~/KAI_ws

cmake -S src/Control/firmware/STM_A --preset Release
cmake --build src/Control/firmware/STM_A/build/Release

cmake -S src/Control/firmware/STM_B --preset Release
cmake --build src/Control/firmware/STM_B/build/Release

cmake -S src/Control/firmware/STM_TEST --preset Release
cmake --build src/Control/firmware/STM_TEST/build/Release
```

현재 CMake target이 직접 생성하는 결과물은 각 build 디렉터리의 `.elf`입니다.
HEX·BIN이 필요하면 사용하는 flash 도구에 맞게 별도로 변환하십시오. 플래시
절차는 사용하는 ST-Link 환경에 맞게 수행하고, 대상 보드가 어느 것인지 먼저
확인하십시오.

STM_TEST는 host에서 도는 logic test를 함께 제공합니다. 절차는
[`STM_TEST/README.md`](STM_TEST/README.md#9-빌드)에
있습니다.

## micro-ROS Agent 빌드

`firmware/STM_A`와 `firmware/STM_B`에는 `COLCON_IGNORE`가 있으므로, 다음 명령은
vendored Agent와 `micro_ros_msgs`만 ROS 패키지로 빌드합니다.

아래 명령은 권장 워크스페이스인 `~/KAI_ws` 루트에서 실행합니다.

```bash
cd ~/KAI_ws
si

colcon build \
  --base-paths src/Control/firmware \
  --packages-select micro_ros_msgs micro_ros_agent

si
```

vendored upstream revision과 로컬 빌드 수정은
`firmware/micro_ros_agent/UPSTREAM.md`에 기록되어 있습니다.

## STM 보드 고정 장치 이름

NUCLEO-G474RE의 ST-LINK serial을 기준으로 다음 이름을 사용합니다. `ttyACM0/1/2`
순서는 재부팅이나 USB 재연결 때 바뀔 수 있으므로 보드 식별에 사용하지 않습니다.

이름은 실물 보드를 구분하려고 나눈 것입니다. HIL bench에 있는 보드는 `hil_`
이름을 쓰고, 나중에 실차에 다는 보드가 `stm_a`와 `stm_b`를 씁니다. bench의
`hil_a`와 `hil_b`는 실차 보드와 같은 STM_A·STM_B 펌웨어를 올린 복제본이므로
동작은 동일해야 합니다.

| 보드 | 올린 펌웨어 | ST-LINK/USB serial | 고정 장치 이름 |
|---|---|---|---|
| Vehicle STM_A | STM_A | `003D00433234511137333934` | `/dev/stm_a` |
| Vehicle STM_B | STM_B | `002E00253235510837333439` | `/dev/stm_b` |
| Retired STM_B | 사용 금지 (MCU 손상) | `004E003B3235510B37333439` | 미등록 |
| HIL bench A | STM_A | `0025002B3234511137333934` | `/dev/hil_a` |
| HIL bench B | STM_B | `003600283235510B37333439` | `/dev/hil_b` |
| HIL bench TEST | STM_TEST | `002000203235511537333439` | `/dev/hil_test` |
| HIL dashboard | Arduino UNO R4 | `3114101259323437A93033334B57305C` | `/dev/arduino_hil` |

규칙 원본은 `firmware/udev/99-kaiev26-stm.rules`입니다. 호스트에 설치한 뒤 규칙을
다시 읽고 현재 장치에 적용합니다.

```bash
sudo install -m 0644 firmware/udev/99-kaiev26-stm.rules \
  /etc/udev/rules.d/99-kaiev26-stm.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
readlink -f /dev/stm_a /dev/stm_b
readlink -f /dev/hil_a
```

2026-08-03 실보드 확인에서 STM_A `003D00433234511137333934`는 `/stm_a_node`,
당시 STM_B `004E003B3235510B37333439`는 `/stm_b_node`를 각각 생성했습니다.
2026-08-06 HIL bench에서 다시 대조해 같은 결과를 얻었습니다. 이 확인은 장치
이름뿐 아니라 실제 실행된 펌웨어 역할까지 대조한 결과입니다. 확인 방법은 각
장치에 micro-ROS Agent를 하나씩 붙여 `ros2 node list`로 올라온 node 이름을 보는
것입니다. Agent 없이 장치를 듣기만 하면
보드가 보내는 GET_INFO ping에는 session_id가 `0x80`이라 client key가 실리지
않아 두 보드의 frame이 완전히 같습니다. Agent 연결 뒤 나오는 client key는 세션마다
새로 뽑히므로 보드 식별에 쓰지 않습니다.

기존 HIL_TEST 보드 `002E00253235510837333439`는 2026-08-06에 STM_TEST의
`STATUS`와 `SET MODE AUTO` 응답까지 확인했지만, 2026-08-27 차량 STM_B 교체
보드로 전용했습니다. 2026-09-08 대체 보드 `002000203235511537333439`를
`/dev/hil_test`로 배정하고 STM_TEST 플래시 및 bench 연동을 확인했습니다.
결과와 남은 사항은 [HIL runbook](../docs/HIL_RUNBOOK.md#15-진행-상태와-남은-작업)에 있습니다.

2026-08-27 교체 보드 `002E00253235510837333439`에는 당시 direct-LCD `STM_B.elf`를
verified flash하고 reset한 뒤, 3.28 V·Device ID `0x469`와 flash readback의
`stm_b_node`, `/vehicle/command`, `/brake/status` 문자열을 확인했습니다. 이 결과는
전송 이미지의 STM_B 역할을 증명하지만 micro-ROS Agent 연결이나 실차 액추에이터
동작까지 검증한 것은 아닙니다. 해당 source는 현재
`archive/firmware/STM_B_DIRECT_LCD`에 보관합니다.

## Agent 실행

두 보드를 동시에 연결하려면 서로 다른 serial device에 Agent를 하나씩 실행합니다.
위 udev 규칙을 설치한 호스트에서는 고정 장치 이름을 사용합니다. 아래는 HIL
bench 이름 기준이며, 실차 보드에서는 `/dev/stm_a`와 `/dev/stm_b`를 씁니다.
STM_TEST는 micro-ROS를 쓰지 않으므로 Agent를 붙이지 않습니다.

터미널 1, STM_A:

```bash
cd ~/KAI_ws
si
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/hil_a -b 115200
```

터미널 2, STM_B:

```bash
cd ~/KAI_ws
si
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/hil_b -b 115200
```

연결 후에는 node 이름뿐 아니라 타입과 실제 endpoint를 함께 확인합니다.

```bash
ros2 node list
ros2 topic info -v /vehicle/command
ros2 topic info -v /drive/status
ros2 topic info -v /brake/status
ros2 topic echo /drive/status
ros2 topic echo /brake/status
```

정상 상태에서는 `/stm_a_node`와 `/stm_b_node`가 보이고, 두 보드 모두
`/vehicle/command`를 구독합니다. STM_A는 `/drive/status`, STM_B는
`/brake/status`를 발행합니다.

STM_TEST bridge와 GUI를 포함해 SSH 접속부터 HIL을 순서대로 실행하는 절차는
[`../docs/HIL_RUNBOOK.md`](../docs/HIL_RUNBOOK.md)를 따릅니다. 특히 device symlink는
virtual COM port 식별이며 SWD target 확인을 대신하지 않으므로, runbook의 현장 확인
전에는 이를 근거로 플래시하지 않습니다.

## 관련 문서

- 전체 시스템 구조: [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md)
- 통신 인터페이스: [`../docs/INTERFACES.md`](../docs/INTERFACES.md)
- 안전 설계: [`../docs/SAFETY.md`](../docs/SAFETY.md)
- STM_A 상세: [`STM_A/README.md`](STM_A/README.md)
- STM_B 상세: [`STM_B/README.md`](STM_B/README.md)
- STM_TEST 상세: [`STM_TEST/README.md`](STM_TEST/README.md)
- HIL 전체 운용: [`../docs/HIL_RUNBOOK.md`](../docs/HIL_RUNBOOK.md)
