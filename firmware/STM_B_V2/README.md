# STM_B_V2 firmware

`STM_B_V2`는 기존 [`../STM_B`](../STM_B)를 보존한 채 LCD의 긴 SPI 배선을 제거하고,
별도 Arduino UNO R4 Minima dashboard로 기존 LCD 값을 CAN으로 전달하는 변형입니다.
UNO R4는 이번 단계에서 LCD UI를 그리지 않고 protocol v1 snapshot을 Serial Monitor에
출력합니다.

## 보존되는 기능

다음 경로는 기존 STM_B에서 그대로 복제했습니다.

- 물리 E-stop 세 입력과 최종 `BRAKE_REQUEST` 결정
- TIM2 brake servo PWM 및 ramp
- STM_A와 USART1 safety/vehicle/detail frame 교환
- LPUART1 micro-ROS transport와 `/vehicle/command`, `/brake/status`

기존 저우선순위 `Display` FreeRTOS task만 `DashboardCan_Task()` 호출로 바꿨습니다.
CAN start/enqueue와 FIFO 상태 확인은 모두 즉시 반환하는 HAL API만 사용합니다. 실패는
진단 counter에만 기록하며 retry 대기, `Error_Handler()`, brake/E-stop 변경을 하지
않습니다.

## Dashboard data source

CAN snapshot은 기존 STM_B
[`display_app.c`](../STM_B/Core/Src/display_app.c)의 `Dashboard_ReadState()`가 읽던 값만
복사합니다. LCD용 문자열이나 `lcd.c`의 rendering 상태를 전송하지 않습니다.

| 표시 값 | STM_B source | source type/unit | CAN 표현 |
|---|---|---|---|
| speed | `CURRENT_SPEED_FROM_A` | `float`, m/s. STM_A UART 원본은 signed `int16`, 0.01 m/s | signed `int16`, 0.01 m/s |
| gear | `GEAR_FROM_A` | `uint8`, `0=FWD`, `1=REV` | status bit |
| mode | `MODE_FROM_A` | `uint8`, `0=MANUAL`, `1=AUTO` | status bit |
| STM_A vehicle link | `VEHICLE_FRAME_VALID_FROM_A` | boolean | status bit |
| detail link | `DRIVE_DETAIL_FRAME_VALID_FROM_A` | boolean | status bit |
| pedal | `DRIVE_DETAIL_FROM_A.pedal_percent` | `uint8`, percent | unsigned byte |
| bus voltage | `bus_voltage_dv` | `uint16`, 0.1 V | unsigned `uint16`, 0.1 V |
| bus current | `bus_current_da` | `int16`, 0.1 A | signed `int16`, 0.1 A |
| phase current | `phase_current_da` | `int16`, 0.1 A | signed `int16`, 0.1 A |
| controller temperature | `controller_temperature_dc` | `int16`, 0.1 degC | signed `int16`, 0.1 degC |
| motor temperature | `motor_temperature_dc` | `int16`, 0.1 degC | signed `int16`, 0.1 degC |
| motor fault | `motor_fault_bits` | LCD가 표시하는 low 24 bits | unsigned 24-bit bitmap |
| E-stop | `ESTOP_STATUS` | 세 물리 E-stop의 합성 boolean | status bit |
| brake | `BRAKE_REQUEST` | 최종 brake 요청 boolean | status bit |
| drive status | `DRIVE_DETAIL_FROM_A.flags` | LCD가 사용하는 validity, tractive feedback, command와 drive-stop bits | status/drive bits |

속도는 source의 부호를 보존합니다. 기존 LCD에 표시하던 속도는 이 값의 절댓값에
`3.6`을 곱한 km/h입니다. 개별 E-stop 입력과 `BRAKE_COMMAND_APPLIED`는 기존 LCD
snapshot에 없으므로 protocol v1에 넣지 않습니다.

## CAN protocol v1

| 항목 | 값 |
|---|---|
| Peripheral | STM32G474 `FDCAN1` |
| Controller pins | `PA12=FDCAN1_TX`, `PA11=FDCAN1_RX` |
| Nucleo headers | `PA12=CN10-12`, `PA11=CN10-14` |
| Frame | Classical CAN, standard ID, DLC 8 |
| Bitrate | 250 kbps |
| Byte order | multi-byte integer little-endian |
| Snapshot period | 50 ms (20 Hz) |
| Frame slots | snapshot 후 0/10/20/30 ms, 40 ms는 빈 slot |
| Protocol version | `1` |
| Alive counter | `uint8`, snapshot마다 증가, 255 다음 0 |

네 frame의 byte 0은 protocol version, byte 1은 같은 snapshot의 alive입니다. UNO는
네 ID와 version/alive가 모두 일치할 때만 완전한 snapshot으로 사용합니다.

### `0x321` STATUS

| Byte | Type | Meaning |
|---:|---|---|
| 0 | `uint8` | protocol version = 1 |
| 1 | `uint8` | alive counter |
| 2 | bitfield | status flags |
| 3 | bitfield | drive flags |
| 4 | `uint8` | pedal percent |
| 5 | reserved | zero |
| 6-7 | `int16` LE | signed speed, 0.01 m/s |

Status flags byte 2:

| Bit | Meaning |
|---:|---|
| 0 | STM_A vehicle frame valid |
| 1 | drive-detail frame valid |
| 2 | detail Part I valid |
| 3 | detail Part II valid |
| 4 | AUTO (`0`은 MANUAL) |
| 5 | REVERSE (`0`은 FORWARD) |
| 6 | combined physical E-stop active |
| 7 | final brake request active |

Drive flags byte 3:

| Bit | Meaning |
|---:|---|
| 0 | tractive feedback RUN |
| 1 | AUTO command received |
| 2 | AUTO command timeout |
| 3 | drive stop |
| 4-7 | reserved, zero |

### `0x322` POWER

| Byte | Type | Meaning |
|---:|---|---|
| 0 | `uint8` | protocol version = 1 |
| 1 | `uint8` | alive counter |
| 2-3 | `uint16` LE | bus voltage, 0.1 V |
| 4-5 | `int16` LE | bus current, 0.1 A |
| 6-7 | `int16` LE | phase current, 0.1 A |

### `0x323` THERMAL

| Byte | Type | Meaning |
|---:|---|---|
| 0 | `uint8` | protocol version = 1 |
| 1 | `uint8` | alive counter |
| 2-3 | `int16` LE | controller temperature, 0.1 degC |
| 4-5 | `int16` LE | motor temperature, 0.1 degC |
| 6-7 | reserved | zero |

### `0x324` FAULT

| Byte | Type | Meaning |
|---:|---|---|
| 0 | `uint8` | protocol version = 1 |
| 1 | `uint8` | alive counter |
| 2-4 | `uint24` LE | `motor_fault_bits & 0x00FFFFFF` |
| 5-7 | reserved | zero |

Repository의 standard-ID 계획 `0x20`-`0x22`(LV battery gauge)와 겹치지 않습니다.
EZkontrol `0x0C01EFD0`, `0x1801D0EF`, `0x1802D0EF` 및 KEYA
`0x05800001`, `0x06000001`, `0x07000001`은 extended ID입니다. 기존 smoke ID
`0x321`은 STATUS frame으로 승격했고 `0x322`-`0x324`는 repository 검색상 다른 CAN
contract와 충돌하지 않습니다.

현재 [`STM_B_V2.ioc`](STM_B_V2.ioc)의 FDCAN clock은 170 MHz이고 nominal timing은
prescaler 40, TimeSeg1 13, TimeSeg2 3입니다.

```text
170 MHz / (40 * (1 + 13 + 3)) = 250,000 bit/s
```

따라서 FDCAN peripheral/pin 설정은 기존 STM_B 그대로 사용하며 CubeMX CAN 설정을
변경하지 않았습니다. 기존 SPI2 generated peripheral은 호환성을 위해 초기화 상태로
남아 있지만 LCD application source는 V2 build에서 제거됐고 SPI transfer를 호출하는
task도 없습니다. V2에는 LCD harness를 연결하지 않습니다.

`HAL_FDCAN_AddMessageToTxFifoQ()`는 frame을 Tx FIFO에 enqueue하고 즉시 반환합니다.
4개 frame을 한 번에 enqueue하지 않고 10 ms slot마다 하나씩 시도합니다. 상대 노드가
없거나 FIFO가 찬 경우 해당 frame을 버리고 다음 slot으로 진행하므로 dashboard task가
safety task 실행을 막지 않습니다. UNO는 불완전한 alive 세트를 출력하지 않습니다.

Debugger에서 확인 가능한 진단 변수:

- `DASHBOARD_CAN_TX_COUNT`
- `DASHBOARD_CAN_TX_ERROR_COUNT`
- `DASHBOARD_CAN_TX_FIFO_FULL_COUNT`
- `DASHBOARD_CAN_START_ERROR_COUNT`
- `DASHBOARD_CAN_LAST_HAL_STATUS`
- `DASHBOARD_CAN_SNAPSHOT_COUNT`
- `DASHBOARD_CAN_ALIVE`

## STM_B_V2 ↔ SN65HVD230 배선

| STM_B_V2 | SN65HVD230 | 설명 |
|---|---|---|
| `PA12`, `CN10-12` | pin 1 `D/TXD` | controller TX → transceiver input |
| `PA11`, `CN10-14` | pin 4 `R/RXD` | transceiver output → controller RX |
| 3.3 V | pin 3 `VCC` | transceiver supply |
| GND | pin 2 `GND` | logic/bus ground |
| GND | pin 8 `RS` | high-speed mode |
| — | pin 5 `Vref` | 사용하지 않음 |
| CANL | pin 6 `CANL` | differential bus low |
| CANH | pin 7 `CANH` | differential bus high |

SN65HVD230 VCC 가까이에 100 nF decoupling capacitor를 둡니다. STM32 PA11/PA12를
CANH/CANL에 직접 연결하면 안 됩니다.

## Build

```bash
cd ~/KAI_ws
source /etc/profile.d/cubeclt-bin-path_1.20.0.sh

cmake -S src/Control/firmware/STM_B_V2 --preset Release
cmake --build src/Control/firmware/STM_B_V2/build/Release --clean-first
```

결과물은 `build/Release/STM_B_V2.elf`입니다.

Protocol byte packing host test:

```bash
cd ~/KAI_ws/src/Control/firmware/STM_B_V2
cc -std=c11 -Wall -Wextra -Werror \
  -I Core/Inc \
  tests/dashboard_can_protocol_host_test.c \
  Core/Src/dashboard_can_protocol.c \
  -o /tmp/dashboard_can_protocol_host_test
/tmp/dashboard_can_protocol_host_test
```

Build와 host test는 compile 및 byte layout 증거이며 실제 CAN frame 송신이나 safety
hardware 동작 증거는 아닙니다.

2026-09-01 Release clean build 결과는 Flash 94,600 byte(18.04%), RAM 95,200
byte(72.63%)였습니다. Protocol host test도 warning 없이 통과했습니다.

## Bench smoke test

1. 차량 정지·중립, 가속 페달 해제, E-stop 상태를 확보하고 LV를 끕니다.
2. brake servo 전원과 기존 LCD SPI harness는 분리합니다.
3. 위 표대로 STM_B_V2와 SN65HVD230을 연결합니다.
4. UNO R4/MCP2551의 CANH-CANL과 연결하고 두 보드 GND를 공통으로 묶습니다.
5. 두 노드가 cable의 양 끝이라면 각 끝 CANH-CANL 사이에 120 ohm termination을
   하나씩 둡니다. 전원 OFF에서 CANH-CANL 합성 저항이 약 60 ohm인지 확인합니다.
6. UNO R4 sketch를 업로드하고 Serial Monitor를 115200 baud로 엽니다.
7. STM_B_V2를 flash한 뒤 두 CAN node를 함께 켭니다.
8. Serial Monitor에서 `DASH v1 alive=...`와 LINK/MOTION/SAFETY/DRIVE/POWER/THERMAL
   값이 약 200 ms마다 출력되는지 확인합니다. `complete`는 계속 증가하고 `ignored`는
   증가하지 않아야 합니다.
9. STM_A UART를 끊으면 `vehicle=LOST` 또는 `detail=LOST`와 해당 `--` 값이 표시되는지
   확인합니다. 이는 source validity 전달 시험이며 실차 구동 시험이 아닙니다.
10. dashboard CAN을 300 ms 넘게 끊으면 `CAN LOST`, 다시 연결하면 `CAN RECOVERED`가
    한 번씩 출력되는지 확인합니다.
11. 시험 후 전원을 끄고 Programmer/serial monitor 등 시작한 도구를 종료합니다.

이번 build/flash 검증만으로 brake safety나 실제 CAN 배선이 검증되는 것은 아닙니다.

## References

- [TI SN65HVD230 datasheet](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)
- [UNO R4 dashboard receiver](../UNO_R4_DASHBOARD/README.md)
