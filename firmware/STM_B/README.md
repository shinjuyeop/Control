# STM_B Firmware

> 이 문서는 현재 작업 브랜치의 소스 코드를 기준으로 작성되었습니다.
> 코드 변경 시 핀, 주기, timeout, 통신 ID와 안전 조건을 함께 갱신해야 합니다.

## 1. 역할

STM_B는 STM32G474 기반의 물리 E-stop·브레이크 보드입니다. 세 E-stop 입력과 AUTO
Host brake를 조합해 최종 `BRAKE_REQUEST`를 만들고, 브레이크 서보를 직접 구동하며
같은 요청을 STM_A에 전달합니다. 표시 값은 Classical CAN으로 Arduino UNO R4에
보냅니다.

Dashboard CAN 오류나 Arduino 고장은 E-stop, brake PWM, STM_A safety UART와
micro-ROS 동작을 변경하지 않습니다. 이전 직접-SPI LCD 구현은
[`../../archive/firmware/STM_B_DIRECT_LCD`](../../archive/firmware/STM_B_DIRECT_LCD)에
보관합니다.

## 2. 담당 기능

- Dashboard/Side/RF 물리 E-stop 입력
- MANUAL/AUTO 정책에 따른 최종 `BRAKE_REQUEST` 계산
- TIM2 brake servo PWM 및 engage/release ramp
- STM_A USART1 vehicle/detail frame 수신과 safety frame 송신
- LPUART1 micro-ROS `/vehicle/command`, `/gnss/fix` 구독과 `/brake/status` 발행
- 기존 STM_B LCD가 사용한 값의 atomic snapshot 생성
- 250 kbps Classical CAN dashboard protocol v1 송신
- FDCAN automatic retransmission, bus-off 감시와 1초 backoff 재시작
- dashboard CAN 실패를 safety task와 격리한 저우선순위 처리

## 3. 보드와 MCU

| 항목 | 값 |
|---|---|
| 보드 | NUCLEO-G474RE, MB1367 |
| MCU | STM32G474RETx, Cortex-M4F |
| Flash / RAM | 512 KiB / 128 KiB |
| CubeMX 프로젝트 | `STM_B.ioc` |
| 빌드 target | `STM_B.elf` |
| FDCAN | 170 MHz kernel clock, Classical CAN 250 kbps |
| micro-ROS transport | LPUART1, 115200 8N1 |
| STM_A transport | USART1, 115200 8N1 |
| 고정 장치 이름 | 실차 `/dev/stm_b` |

`STM_B.ioc`에는 기존 SPI2와 LCD GPIO generated 설정이 남아 있지만 Application
build는 `display_app.c`/`lcd.c`가 아니라 `dashboard_can.c`와
`dashboard_can_protocol.c`를 사용합니다.

## 4. 핀맵

보드 커넥터 번호는 NUCLEO-G474RE ST morpho header 기준입니다. `CN7`과 `CN10`은
각각 2x19 header이며 홀수와 짝수 번호가 서로 다른 열입니다. GND는 `CN7-8`,
`CN7-19`, `CN7-20`, `CN7-22`, `CN10-9`, `CN10-20`에서 사용할 수 있습니다.

### 4.1 신호 핀

| MCU 핀 | Header | 기능 | 방향/설정 | 의미 |
|---|---|---|---|---|
| `PC0` | `CN7-38` (A5) | `ESTOP_DASH` | input, pull-up, EXTI0 | LOW=active |
| `PC1` | `CN7-36` (A4) | `ESTOP_SIDE` | input, pull-up, EXTI1 | LOW=active |
| `PB4` | `CN10-27` (D5) | `ESTOP_RF` | input, pull-up, EXTI4 | LOW=active |
| `PB3` | `CN10-31` (D3) | brake servo `TIM2_CH2` | PWM output, 50 Hz | 1305 us=engage (55°), 1665 us=release (120°) |
| `PC13` | `CN7-23` | status LED | push-pull output | 상태 표시 |

### 4.2 통신·주변장치 핀

| MCU 핀 | Header | Peripheral | 용도 |
|---|---|---|---|
| `PA12` | `CN10-12` | `FDCAN1_TX` | SN65HVD230 controller TX |
| `PA11` | `CN10-14` | `FDCAN1_RX` | SN65HVD230 controller RX |
| `PA2` | `CN10-35` | `LPUART1_TX` | micro-ROS Agent transport |
| `PA3` | `CN10-37` | `LPUART1_RX` | micro-ROS Agent transport |
| `PC4` | `CN10-34` | `USART1_TX` | STM_A `PC5`로 safety frame 송신 |
| `PC5` | `CN10-6` | `USART1_RX` | STM_A `PC4`에서 vehicle/detail 수신 |

기존 SPI2와 LCD GPIO generated 초기화는 CubeMX 호환을 위해 남아 있지만 production
application은 SPI transfer를 호출하지 않습니다. 직접-SPI LCD harness는 연결하지
않습니다.

### 4.3 디버그·미사용 핀

| MCU 핀 | Header | 기능 |
|---|---|---|
| `PA13` | `CN7-13` | SWDIO |
| `PA14` | `CN7-15` | SWCLK |
| `PB13/PB14/PB15` | `CN10-30/28/26` | generated SPI2, application 미사용 |
| `PB0/PB1/PB2/PB10` | `CN7-34`, `CN10-24/22/25` | 기존 LCD GPIO, application 미사용 |

### 4.4 배선 상대

| STM_B | 상대 연결 |
|---|---|
| `PC0`, `PC1`, `PB4` | Dashboard/Side/RF E-stop active-low output |
| `PB3` | brake servo signal |
| `PC4` TX | STM_A `PC5` RX |
| `PC5` RX | STM_A `PC4` TX |
| `PA12` TX | SN65HVD230 pin 1 `D/TXD` |
| `PA11` RX | SN65HVD230 pin 4 `R/RXD` |
| 3.3 V / GND | SN65HVD230 pin 3 / pin 2, pin 8 RS는 GND |
| SN65HVD230 pin 7/6 | CANH/CANL -> Arduino MCP2551 CANH/CANL |

두 CAN node가 cable 끝이면 양 끝에 CANH-CANL 120 ohm termination을 둡니다. 전원
OFF 상태에서 전체 bus의 CANH-CANL 합성 저항은 약 60 ohm이어야 합니다.

## 5. 소프트웨어 구조

| 경로 | 책임 |
|---|---|
| `Core/Src/app_freertos.c` | brake, E-stop, micro-ROS, STM_A UART task |
| `Core/Src/dashboard_can.c` | snapshot capture, frame scheduling, FDCAN health/recovery |
| `Core/Inc/dashboard_can_protocol.h` | ID, version, flag와 wire type 계약 |
| `Core/Src/dashboard_can_protocol.c` | 명시적 little-endian packing |
| `Core/Src/fdcan.c` | Classical CAN 250 kbps와 PA11/PA12 설정 |
| `Core/Src/tim.c` | TIM2 brake servo PWM |
| `Core/Src/usart.c` | LPUART1 micro-ROS와 USART1 STM_A transport |
| `Core/Src/globals.c` | UART, E-stop와 brake 공유 상태 |
| `../common/board_uart_protocol.c` | STM_A/STM_B UART framing과 CRC8 |
| `tests/dashboard_can_protocol_host_test.c` | protocol v1 golden-byte test |

## 6. FreeRTOS Task

| Task | 책임 | 주기/Trigger | 우선순위 (CMSIS 값) | Stack (words) |
|---|---|---|---|---:|
| `Brake` | `BRAKE_REQUEST`에 따른 PWM ramp | 기존 5 ms loop delay, 20 ms ramp step | `High1` (41) | 512 |
| `EStop` | 물리 E-stop과 AUTO command brake 통합 | EXTI flags + 10 ms timeout 재확인 | `Realtime` (48) | 256 |
| `uROS` | Agent 상태, command 수신, brake status 발행 | 상태 머신; running 후 5 ms delay | `Normal` (24) | 1280 |
| `Display` | `DashboardCan_Task()` 실행 | 기존 absolute 10 ms slot / 50 ms snapshot | `Low` (8) | 512 |
| `ARx` | vehicle/detail parse와 timeout | 작업 후 1 ms delay | `AboveNormal` (32) | 256 |
| `ATx` | STM_A safety frame 송신 | absolute 20 ms | `High` (40) | 256 |
| `PerfMon` | runtime snapshot 집계 | 약 1 Hz | `Low` (8) | 256 |

모두 `.ioc` baseline의 Dynamic task이며 1 word는 4 bytes입니다. 기존 README의
Brake/EStop priority·주기 표기를 실제 생성 코드와 일치시켰습니다. Task 설정 자체는
변경하지 않았습니다. Brake의 ramp와 EStop의 event/timeout 경로도 유지합니다.

ATx는 `vTaskDelayUntil()` 기반으로 정상 20 ms cadence를 유지합니다. 늦은 activation은
실제 시작 시점으로 주기를 다시 맞추고 작업이 한 주기를 소진하면 완료 후 한 주기를
기다립니다. 밀린 safety frame을 burst로 재전송하지 않습니다. blocking UART TX의
10 ms timeout과 buffer lifetime을 유지하며 DMA로 치환하지 않았습니다.

Display는 production에서 Dashboard CAN을 실행합니다. 기존 `vTaskDelayUntil()`과
slot 순서를 유지하므로 지연 시 기존 catch-up 동작도 그대로입니다. Low priority와
512-word stack을 유지하며 stack 축소 여부는 향후 HWM 실측으로 판단합니다.
Dashboard와 PerfMon은 높은 priority의 safety/UART task가 기다리는 mutex를 소유하지
않습니다. FDCAN stop/start는 Low task에서 수행되며 HAL timeout으로 bounded됩니다.

EStop callback→task dispatch 및 처리 duration, task activation/wall time,
stack HWM/heap, snapshot API와 물리 interrupt latency 측정 한계는
[공통 runtime metrics 문서](../common/README.md)를 참고하십시오. PerfMon은 ROS
publish를 하지 않습니다.

## 7. 외부 인터페이스

### Subscribers

| Topic | Type | 사용 필드 |
|---|---|---|
| `/vehicle/command` | `kaiev26_msgs/msg/ActuatorCommand` | AUTO의 `brake_engage` |
| `/gnss/fix` | `sensor_msgs/msg/NavSatFix` | `status.status`; 500 ms freshness |

### Publishers

| Topic | Type | 주기 | 내용 |
|---|---|---|---|
| `/brake/status` | `kaiev26_msgs/msg/BrakeActuatorStatus` | 50 ms 및 reason 변경 | brake command, reason bits, command age |

### Dashboard CAN

| ID | 이름 | 주 내용 |
|---:|---|---|
| `0x321` | STATUS | validity, mode, gear, E-stop, brake, pedal, GNSS, speed |
| `0x322` | POWER | bus voltage, bus current, phase current |
| `0x323` | THERMAL | controller와 motor temperature |
| `0x324` | FAULT | motor fault low 24 bits |

모든 frame은 standard ID, Classical CAN, DLC 8이고 byte 0은 protocol version `1`,
byte 1은 같은 snapshot의 `uint8` alive입니다. Multi-byte 값은 little-endian입니다.

## 8. 동작 상세

### 8.1 E-stop 입력 (`PC0`, `PC1`, `PB4`)

세 입력은 internal pull-up active-low입니다. EXTI callback은 입력 변화를 task에 알리고,
`EStop` task가 실제 GPIO를 다시 읽어 합성 `ESTOP_STATUS`를 계산합니다. 하나라도 LOW면
물리 E-stop이 active입니다.

### 8.2 브레이크 서보 (`PB3`)

`Brake` task는 최종 `BRAKE_REQUEST`를 읽고 engage/release pulse로 ramp합니다. 부팅
초기값은 brake 요청·적용 상태이며 실제 보드 플래시/reset 시 servo가 engage 방향으로
움직일 수 있습니다.

### 8.3 STM_A UART (`PC4`/`PC5`)

- STM_A -> STM_B vehicle status: mode, gear, signed speed, nominal 20 ms
- STM_A -> STM_B drive detail: voltage/current/temperature/fault, nominal 100 ms
- STM_B -> STM_A safety status: final brake request, nominal 20 ms
- vehicle validity timeout: 200 ms
- detail validity timeout: 300 ms

각 frame은 SOF, message type, payload length, sequence와 CRC8을 사용합니다. Dashboard는
validity가 false인 값을 `--`로 표시하도록 flag를 CAN에 함께 보냅니다.

### 8.4 안전 상태 결정

`BRAKE_REQUEST`는 다음 정책을 따릅니다.

- 물리 E-stop은 MANUAL/AUTO 모두 brake를 요청합니다.
- Host `brake_engage`는 STM_A가 전달한 mode가 AUTO일 때만 STM_B brake에 반영됩니다.
- Dashboard CAN 상태, Arduino alive와 LCD 상태는 brake 결정에 사용하지 않습니다.
- Dashboard CAN recovery 실패도 E-stop/brake/UART/micro-ROS 상태를 변경하지 않습니다.

### 8.5 Dashboard snapshot과 wire 단위

| 값 | Source | CAN type/unit |
|---|---|---|
| speed | `CURRENT_SPEED_FROM_A` | signed `int16`, 0.01 m/s |
| gear/mode | `GEAR_FROM_A`, `MODE_FROM_A` | status bits |
| vehicle/detail/Part I/Part II valid | UART validity와 detail flags | status bits |
| pedal | `pedal_percent` | `uint8`, percent |
| bus voltage | `bus_voltage_dv` | `uint16`, 0.1 V |
| bus/phase current | `bus_current_da`, `phase_current_da` | signed `int16`, 0.1 A |
| controller/motor temperature | `*_temperature_dc` | signed `int16`, 0.1 degC |
| motor fault | `motor_fault_bits` | low 24-bit bitmap |
| E-stop/brake | `ESTOP_STATUS`, `BRAKE_REQUEST` | status bits |
| GNSS | `/gnss/fix.status.status` | `uint8` dashboard GNSS state |

한 snapshot의 네 frame은 같은 alive를 사용합니다. 0/10/20/30 ms slot에서 frame을
하나씩 enqueue하고 40 ms slot은 비워 다음 snapshot은 50 ms마다 시작합니다.

#### `0x321` STATUS

| Byte | Type | 의미 |
|---:|---|---|
| 0 | `uint8` | version=1 |
| 1 | `uint8` | alive |
| 2 | bitfield | bit0 vehicle valid, 1 detail valid, 2 Part I, 3 Part II, 4 AUTO, 5 REV, 6 E-stop, 7 brake |
| 3 | bitfield | bit0 tractive feedback, 1 command received, 2 command timeout, 3 drive stop |
| 4 | `uint8` | pedal percent |
| 5 | `uint8` | GNSS: 0 unavailable/stale, 1 no fix, 2 valid fix, 3 RTK fixed |
| 6-7 | `int16` LE | speed, 0.01 m/s |

STM_B는 `/gnss/fix`를 best-effort로 구독하고 수신 시각을 monotonic tick으로
기록합니다. 새 메시지가 500 ms 안에 들어온 경우 `NavSatFix.status=2`만
`RTK fixed`, `status>=0`은 일반 `fix`, 음수는 `no fix`로 변환합니다. 500 ms가
지나거나 한 번도 수신하지 못했으면 `unavailable`을 전송합니다. GNSS 상태는 표시용이며
구동·제동·E-stop 판단에는 사용하지 않습니다.

#### `0x322` POWER

| Byte | Type | 의미 |
|---:|---|---|
| 0/1 | `uint8` | version/alive |
| 2-3 | `uint16` LE | bus voltage, 0.1 V |
| 4-5 | `int16` LE | bus current, 0.1 A |
| 6-7 | `int16` LE | phase current, 0.1 A |

#### `0x323` THERMAL

| Byte | Type | 의미 |
|---:|---|---|
| 0/1 | `uint8` | version/alive |
| 2-3 | `int16` LE | controller temperature, 0.1 degC |
| 4-5 | `int16` LE | motor temperature, 0.1 degC |
| 6-7 | reserved | zero |

#### `0x324` FAULT

| Byte | Type | 의미 |
|---:|---|---|
| 0/1 | `uint8` | version/alive |
| 2-4 | `uint24` LE | motor fault low 24 bits |
| 5-7 | reserved | zero |

Repository의 standard ID `0x20`-`0x22` battery 계획과 겹치지 않습니다. EZkontrol과
KEYA가 사용하는 ID는 extended frame입니다.

### 8.6 FDCAN 오류와 복구

- hardware automatic retransmission을 사용합니다.
- FIFO가 찬 경우 해당 slot을 버리고 오류 counter만 증가시킵니다.
- 100 ms마다 protocol status를 읽습니다.
- bus-off transition은 한 번만 `DASHBOARD_CAN_BUS_OFF_COUNT`에 기록합니다.
- bus-off 또는 start 실패 상태에서는 1초 backoff로 `HAL_FDCAN_Stop/Start`를
  시도합니다.
- recovery는 Low priority Display task 안에서만 수행합니다.
- recovery 성공/실패는 별도 counter에 기록하며 safety state를 변경하지 않습니다.

### 8.7 Arduino와 LCD

[`../UNO_R4_DASHBOARD`](../UNO_R4_DASHBOARD)는 네 frame의 version/alive가 일치해야
snapshot을 완성합니다. 새 alive의 완전한 snapshot이 300 ms 없으면 `CAN LOST`를
표시하고, 1초마다 CAN controller를 재초기화합니다. Fresh snapshot이 오면
`CAN RECOVERED` 후 UI 갱신을 재개합니다. Arduino -> STM heartbeat나 safety feedback은
없습니다.

### 8.8 모드별 동작

| 입력/상태 | MANUAL | AUTO |
|---|---|---|
| 물리 E-stop | brake engage | brake engage |
| Host `brake_engage` | STM_B brake에 미반영 | brake engage |
| Dashboard CAN/LCD lost | safety 영향 없음 | safety 영향 없음 |

## 9. 빌드

권장 workspace root에서 Release clean build합니다.

```bash
cd ~/KAI_ws
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh

cmake -S src/Control/firmware/STM_B --preset Release
cmake --build src/Control/firmware/STM_B/build/Release --clean-first
```

결과물은 `src/Control/firmware/STM_B/build/Release/STM_B.elf`입니다.
2026-09-04 GNSS dashboard build 결과는 Flash 99,664 byte(19.01%), RAM 96,040
byte(73.27%)였습니다. `_gettimeofday` linker warning은 현재 사용하지 않는 syscall에 대한
기존 non-fatal warning입니다.

Protocol packing test:

```bash
cd ~/KAI_ws/src/Control/firmware/STM_B
cc -std=c11 -Wall -Wextra -Werror \
  -I Core/Inc \
  tests/dashboard_can_protocol_host_test.c \
  Core/Src/dashboard_can_protocol.c \
  -o /tmp/dashboard_can_protocol_host_test
/tmp/dashboard_can_protocol_host_test
```

`kaiev26_msgs`가 바뀌면 Control root에서 `bash firmware/tools/regen_microros.sh`를
실행합니다. 이 도구는 STM_A와 STM_B 산출물을 함께 갱신합니다.

## 10. 플래시 및 실행

실보드 operation 전 차량 정지·중립, 가속 페달 해제, E-stop과 LV 상태를 확보합니다.
ST-LINK VCP 이름만 믿지 말고 SWD 전압·Device ID와 기존 firmware role string을 읽은 뒤
정확한 serial을 지정합니다. Reset 순간 brake engage 방향 동작 가능성이 있으므로 최초
bench에서는 servo 전원을 분리합니다.

```bash
STM32_Programmer_CLI \
  -c port=SWD sn=<VERIFIED_STLINK_SN> mode=UR \
  -w src/Control/firmware/STM_B/build/Release/STM_B.elf \
  -v -rst
```

`Download verified successfully`와 `MCU Reset`을 모두 확인합니다. 이 결과는 image
전송 증거이며 vehicle safety 준비 완료를 뜻하지 않습니다.

micro-ROS Agent:

```bash
source /opt/ros/humble/setup.bash
source ~/KAI_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/stm_b -b 115200
```

## 11. 상태 확인

Debugger에서 확인 가능한 dashboard 진단값:

| 변수 | 의미 |
|---|---|
| `DASHBOARD_CAN_SNAPSHOT_COUNT` | 생성한 50 ms snapshot 수 |
| `DASHBOARD_CAN_TX_COUNT` | FIFO enqueue 성공 frame 수 |
| `DASHBOARD_CAN_TX_ERROR_COUNT` | 전체 enqueue/started 실패 수 |
| `DASHBOARD_CAN_TX_FIFO_FULL_COUNT` | FIFO full로 버린 frame 수 |
| `DASHBOARD_CAN_START_ERROR_COUNT` | 최초 start 실패 수 |
| `DASHBOARD_CAN_BUS_OFF_COUNT` | bus-off transition 수 |
| `DASHBOARD_CAN_RECOVERY_COUNT` | stop/start 성공 수 |
| `DASHBOARD_CAN_RECOVERY_ERROR_COUNT` | protocol read 또는 restart 실패 수 |
| `DASHBOARD_CAN_LAST_HAL_STATUS` | 마지막 start/enqueue/recovery HAL status |
| `DASHBOARD_CAN_ALIVE` | 다음 snapshot alive |

정상 CAN에서는 `TX_COUNT`가 `SNAPSHOT_COUNT`의 약 네 배로 증가하고 error/bus-off/
recovery counter는 증가하지 않습니다. Arduino Serial의 `complete`는 증가하고
`ignored`, `duplicate`와 `reinit`은 안정 상태에서 증가하지 않아야 합니다.

실차 검증에서 확인할 중앙 경로:

1. `A-B OK`, mode/gear/speed 변화
2. Dashboard/Side/RF E-stop 각각과 brake reason
3. servo engage/release PWM 및 실제 동작
4. `/brake/status` typed decode와 command age
5. drive-detail voltage/current/temperature/fault
6. CAN 단선, 재연결과 controller recovery 중 safety/UART 지속

## 12. 알려진 제한사항

- Recovery code는 clean build됐지만 의도적인 hardware bus-off 주입은 아직 검증하지
  않았습니다.
- 2026-09-02 STM_A 재플래시 후 `detail=OK`, Part I/II `VALID`, HV 54.9~55.0 V와
  정지 상태 bus/phase current 0.0 A가 Arduino까지 전달되는 것을 확인했습니다.
- Arduino와 LCD는 표시 전용이며 STM_B safety feedback을 보내지 않습니다.
- `motor_fault_bits`는 기존 STM_B LCD와 동일하게 low 24 bits만 전송·표시합니다.
- Direct SPI LCD source는
  [`archive/firmware/STM_B_DIRECT_LCD`](../../archive/firmware/STM_B_DIRECT_LCD)에
  보관하며 production build에는 포함되지 않습니다.
- Firmware flash/reset 검증만으로 brake servo와 drivetrain safety를 증명할 수 없습니다.

## 13. 관련 문서

- 기존 direct-LCD STM_B: [`../../archive/firmware/STM_B_DIRECT_LCD/README.md`](../../archive/firmware/STM_B_DIRECT_LCD/README.md)
- Arduino receiver/UI: [`../UNO_R4_DASHBOARD/README.md`](../UNO_R4_DASHBOARD/README.md)
- 공통 firmware 개요: [`../README.md`](../README.md)
- 차량 interface: [`../../docs/INTERFACES.md`](../../docs/INTERFACES.md)
- safety 계약: [`../../docs/SAFETY.md`](../../docs/SAFETY.md)
- STM_A: [`../STM_A/README.md`](../STM_A/README.md)
