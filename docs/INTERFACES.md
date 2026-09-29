# Interfaces

> 이 문서는 현재 작업 브랜치의 소스 코드를 기준으로 작성되었습니다.
> 코드 변경 시 주기, timeout, 핀, 통신 ID와 안전 조건을 함께 갱신해야 합니다.

## 1. 문서 목적

이 문서는 현재 코드에서 확인되는 ROS 2 topic, STM_A↔STM_B UART, EZkontrol·KEYA
CAN, Dashboard CAN, RP2350 LV 계측·정비, Arduino LCD/touch와 HIL UNO USB의 데이터 계약과
주기·timeout을 정리하는 기준 문서입니다. 안전 조건의 결과는
[`SAFETY.md`](SAFETY.md), 구성요소의 책임은
[`ARCHITECTURE.md`](ARCHITECTURE.md)를 참고하십시오.

## 2. 표기 및 단위 규칙

- ROS 2 공통 단위: 속도 `m/s`, 조향각 `rad`(차량 좌회전 양수)
- Web HIL 화면: 속도 `km/h`, 조향 `degree`. 입력/표시 경계에서 변환하며 wire 단위는 유지
- STM 간 속도: signed `int16`, `0.01 m/s/bit`, little-endian
- EZkontrol CAN: 29-bit extended ID, Classic CAN, DLC 8, little-endian
- KEYA CAN: 29-bit extended ID, Classic CAN, DLC 8, multi-byte 값
  big-endian
- `주기`는 송신 또는 callback 실행의 nominal 설정값입니다. RTOS 지연과 blocking
  I/O 때문에 실제 wire interval과 다를 수 있습니다.
- 공통 메시지 필드의 원본은 같은 workspace의 별도 저장소
  `kaiev26_msgs/msg/*.msg`입니다. 이 문서에는 전체 정의를 복제하지 않습니다.

## 3. 인터페이스 요약

| 송신 | 수신 | 방식 | 이름/ID | 데이터 | 주기 | Timeout | 설명 |
|---|---|---|---|---|---:|---:|---|
| Planning 또는 시험 명령원 | `command_governor` | ROS 2 | `/planning/command` | `ActuatorCommand` | production은 별도 상위 패키지; 실차 시험·HIL은 20 Hz | 거버너 입력 게이트 200 ms | 실차 시험은 `vehicle_test_scenario_runner`, HIL은 `hil_command_generator` |
| `command_governor` | STM_A | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 500 ms | AUTO 속도·brake |
| `command_governor` | STM_B | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 적용 안 함 | brake만 저장 |
| `command_governor` | 조향 노드 | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 200 ms | ACTIVE 조향 freshness |
| STM_A | Host·조향·상태·진단 | ROS 2 | `/drive/status` | `DriveActuatorStatus` | 20 ms | 조향에서 1500 ms | mode·gear·구동 진단 |
| STM_A | Host·기록 | ROS 2 | `/drive/can_rx_diagnostics` | `std_msgs/UInt32MultiArray` | 1 s | 없음 | Part I/II RX 횟수·FDCAN RX 오류 관측 전용 |
| STM_B | Host·조향·상태·진단 | ROS 2 | `/brake/status` | `BrakeActuatorStatus` | 50 ms 및 원인 변경 시 | 조향에서 별도 timeout 없음 | brake·E-stop 원인 |
| 조향 노드 | Host·상태·진단 | ROS 2 | `/steering/status` | `SteeringActuatorStatus` | 20 ms | 상태 통합에서 별도 timeout 없음 | 조향 상태·진단 |
| 조향 노드 | AUTO dashboard summary | ROS 2 | `/steering/target_motor_position_count` | `std_msgs/Int32` | 20 ms | summary에서 300 ms | 기존 equivalent-angle→KEYA count 변환 결과 |
| AUTO dashboard summary | STM_B | ROS 2 | `/dashboard/auto_status` | `std_msgs/UInt8MultiArray`, 고정 25 byte | 100 ms | STM_B에서 300 ms | health·방위·목표속도·조향 표시 전용 |
| `vehicle_state_publisher_node` | ROS 2 consumers | ROS 2 | `/vehicle/state` | `VehicleState` | 50 ms | 없음 | 세 status의 마지막 값 합성 |
| `vehicle_state_publisher_node` | ESKF·기록 | ROS 2 | `/vehicle/wheel_twist` | `TwistWithCovarianceStamped` | 최대 50 ms | 자체 timeout 없음 | valid 구동·조향 관측의 Ackermann 근사 |
| `diagnostics_publisher` | Console·Foxglove·기록 | ROS 2 | `/diagnostics` | `DiagnosticArray` | 1 s | 입력 status 1 s | 링크·상태·제동 진단 요약 |
| STM_A | STM_B | USART1 | type `0x01` | mode, gear, speed | 20 ms | STM_B에서 200 ms | vehicle status |
| STM_A | STM_B | USART1 | type `0x03` | 구동 bus·온도·fault·command 상태 | 100 ms | STM_B에서 300 ms | dashboard drive detail |
| STM_B | STM_A | USART1 | type `0x02` | final brake request | 20 ms | STM_A에서 100 ms | safety status |
| RP2350 | STM_B | UART0 ↔ UART4, 115200 8N1 | status `0x01` | CRC32 보호 136-byte LV payload | 100 ms | STM_B에서 1 s | 계측·SOC·진단 |
| STM_B | RP2350 | UART4 ↔ UART0 | command `0x10` / ACK `0x11` | 12-byte command / 16-byte 저장 결과 ACK | event | TUI 1 s 재전송 | 정비용 request ID |
| STM_B | ROS 소비자 | ROS 2 | `/lv_battery/state` | `sensor_msgs/BatteryState` | 100 ms | UART 1 s | 방전 음수; 계측 무효는 NaN |
| STM_B ↔ 정비 TUI | 정비 TUI ↔ STM_B | ROS 2 | `/lv_battery/link/{status,command,ack}` | `std_msgs/UInt8MultiArray` | status 100 ms / 나머지 event | TUI freshness 판정 | 전체 이름은 아래 LV 절 참고 |
| STM_B | UNO R4 | Classical CAN | `0x321`~`0x328` | native status 4 frame + AUTO summary 4 frame | 50 ms | UNO에서 300 ms | 표시 전용 |
| UNO R4 | ST7796S LCD | SPI | MSP4030/MSP4031 | logical 480×320 화면 | 적용 상태 변경 시 | 없음 | local display output |
| FT6336U | UNO R4 | I2C | address `0x38` | 최대 2점 raw touch | 20 ms polling | read 실패 시 1 s 재시도 | local navigation input |
| HIL Console | UNO R4 | USB Serial | newline ASCII, 115200 | `HIL_LCD`, `HIL_TOUCH`, `VT D/M/U/C` | 보고 200/100 ms | Virtual Touch 1 s | HIL-only virtual HMI |
| STM_A | EZkontrol | FDCAN1 | `0x0C01EFD0` | handshake ACK 또는 command | 50 ms command | 명시적 TX timeout 없음 | AUTO Speed/Torque / MANUAL direct torque |
| EZkontrol | STM_A | FDCAN1 | `0x1801D0EF`, `0x1802D0EF` | Part I/II status | 외부 장치 설정 | Part I invalid 500 ms, Part II invalid 1 s | AUTO Torque는 둘 중 하나가 invalid면 PI reset·0 A/HALTED |
| 조향 노드 | KEYA | SocketCAN | `0x06000001` | enable/disable/position/torque/query | 기본 20 ms | position 500 ms / torque 위치 100 ms | node ID 기본값 1 |
| KEYA | 조향 노드 | SocketCAN | `0x05800001`, `0x07000001` | query response, heartbeat | 외부 장치 설정 | CAN·position 각 500 ms | ID는 node ID에 따라 변함 |
| Host Agent | 각 STM | LPUART1 serial, 460800 8N1 | micro-ROS XRCE-DDS | ROS 2 serialization | event-driven | Agent ping 500/100 ms | 보드별 Agent 1개 |

## 4. ROS 2 Topics

### 4.1 차량 제어와 상태

| Topic | Publisher | Subscriber | Type | Rate | QoS | 설명 |
|---|---|---|---|---:|---|---|
| `/planning/command` | Planning (production은 별도 상위 패키지), 실차 시험 `vehicle_test_scenario_runner`, HIL `hil_command_generator` 중 하나 | `command_governor` | `kaiev26_msgs/msg/ActuatorCommand` | 시험 명령원 20 Hz | default | Planning 의도 명령 |
| `/vehicle/command` | `command_governor` (유일 발행자) | STM_A, STM_B, `steering_actuator_node` | `kaiev26_msgs/msg/ActuatorCommand` | 입력 연동; HIL 20 Hz | default | 속도·조향·최종 brake 명령 |
| `/drive/status` | `/stm_a_node` | 조향 노드, `vehicle_state_publisher_node`, `diagnostics_publisher`, 관측 도구 | `kaiev26_msgs/msg/DriveActuatorStatus` | 50 Hz | default | mode·gear·구동 명령 및 EZkontrol 진단 |
| `/brake/status` | `/stm_b_node` | 조향 노드, `vehicle_state_publisher_node`, `diagnostics_publisher`, 관측 도구 | `kaiev26_msgs/msg/BrakeActuatorStatus` | 20 Hz + 변경 시 | default | 브레이크 command와 stop reason |
| `/steering/status` | `/steering_actuator_node` | `vehicle_state_publisher_node`, `diagnostics_publisher`, 관측 도구 | `kaiev26_msgs/msg/SteeringActuatorStatus` | 50 Hz | Keep Last 10, reliable, volatile | 조향 상태 머신·timeout·모터 진단 |
| `/vehicle/state` | `/vehicle_state_publisher` | 기록 및 차량 상태 consumer | `kaiev26_msgs/msg/VehicleState` | 20 Hz | Keep Last 10, reliable, volatile | 세 status가 모두 수신된 후 합성 |
| `/vehicle/wheel_twist` | `/vehicle_state_publisher` | `eskf_localization`, 기록 | `geometry_msgs/msg/TwistWithCovarianceStamped` | 최대 20 Hz | Keep Last 10, reliable, volatile | drive Part I·steering feedback valid일 때만 발행 |
| `/diagnostics` | `/diagnostics_publisher` 외 sensor/bridge 진단 노드 | Vehicle Console, Foxglove, 기록 | `diagnostic_msgs/msg/DiagnosticArray` | 차량 status 요약 1 Hz | Keep Last 10, reliable, volatile | publisher별 DiagnosticArray가 같은 topic에 공존 |

STM micro-ROS endpoint는 `rclc_*_init_default`를 사용합니다. 이는 코드상 default
QoS이며, 조향·상태 통합의 depth `10`도 rclcpp/rclpy default QoS를 사용합니다.
실제 RMW별 endpoint 호환성은 다음 명령으로 확인하십시오.

```bash
ros2 topic info -v /planning/command
ros2 topic info -v /vehicle/command
ros2 topic info -v /drive/status
ros2 topic info -v /brake/status
ros2 topic info -v /steering/status
ros2 topic info -v /vehicle/state
ros2 topic info -v /vehicle/wheel_twist
ros2 topic info -v /diagnostics
```

`vehicle_state_publisher` package는 세 독립 노드를 제공합니다. 상태 합성 노드는
`/vehicle/state`와 `/vehicle/wheel_twist`를 발행하고, `diagnostics_publisher`는
status 3종을 1 Hz `DiagnosticArray`로 요약합니다. `auto_dashboard_summary`는
센서·차량·planning 입력으로 `/dashboard/auto_status`를 10 Hz로 발행합니다. `/diagnostics`에는 STM_TEST,
INS 등 다른 publisher도 함께 쓸 수 있으므로 consumer는 배열 하나를 전체 시스템
상태로 간주하지 않고 `hardware_id`와 `name`으로 항목을 구분합니다.

| `hardware_id` | `name` | 입력 topic |
|---|---|---|
| `STM_A / EZkontrol` | `구동 링크` | `/drive/status` |
| `STM_A / EZkontrol` | `구동 상태` | `/drive/status` |
| `KEYA / can0` | `조향 링크` | `/steering/status` |
| `KEYA` | `조향 상태` | `/steering/status` |
| `STM_B` | `브레이크` | `/brake/status` |

이 identity는 NO DATA/STALE/LIVE와 severity가 바뀌어도 유지합니다. Topic명은
message/detail에 표시하고 identity로 대체하지 않습니다. Vehicle Console도
`(hardware_id, name)`을 cache key로 사용해 여러 `DiagnosticArray` publisher의
동명 항목을 구분합니다.

STM_A/B의 PC-facing micro-ROS transport는 각각 LPUART1 **460800 baud, 8N1**입니다.
Host launch의 `agent_baud` 기본값도 460800입니다. 보드 사이 USART1, STM_TEST ASCII,
UNO USB는 별개 인터페이스이며 115200을 유지합니다.

`/vehicle/command`의 발행자는 `command_governor` 하나입니다. 거버너는
`/planning/command`를 수신 즉시 정합성 게이트(stamp 0·음수, 나이 ±200 ms,
finite)를 거쳐 재-stamp 후 재발행하며, 자체 타이머와 keep-alive가 없으므로 발행률은
입력을 그대로 따릅니다. 따라서 상류(실차 시험은
`vehicle_test_scenario_runner`, HIL은 `hil_command_generator`, 모두 20 Hz)가
STM_A의 500 ms와 조향의 200 ms freshness 한계보다 충분히 빠르게 발행해야
합니다. 5 Hz는 조향 200 ms의 경계라 간헐 timeout이 납니다.
`/planning/command`의 production Planning publisher는 아직 구현되지 않았습니다.
실차 시험·HIL·production publisher 중 하나만 실행합니다.

### 시험 실행기와 label

`vehicle_test_scenario_runner` 실행 파일은 일반 호출 시 TUI(`/auto_sysid`)이며,
YAML launch는 headless `vehicle_test_scenario_node`(`/vehicle_test_scenario_runner`)를
사용한다. 두 경로 모두 시험 명령을 20 Hz로 `/planning/command`에 발행한다.
`/sysid/scenario`는 `std_msgs/msg/String` 기록용 label 토픽이다.
기존 `/sysid/scinario` 오타를 정정했으며 과거 MCAP의 토픽 이름은 그대로 둔다. Headless의 빈 `scenario_label`은 `scenario` 변경을 따라가고,
명시된 label은 그대로 유지된다. 고정 YAML label에 들어 있는 숫자는 식별자이며
현재 속도 설정을 뜻하지 않는다. 모든 실차 시나리오의 기본 속도·중심각은 0이다.
실행 중단 조건의 차이는 [Safety](SAFETY.md#실차-시나리오-실행기의-현재-제한)를 따른다.

### 4.2 조향 보조 topic

| Topic | Publisher | Type | Rate | 설명 |
|---|---|---|---:|---|
| `/steering/angle` | 조향 노드 | `std_msgs/msg/Float32` | position valid일 때 최대 50 Hz | 중앙 등가 실측 조향각, degree·좌회전 양수 |
| `/steering/enable_commanded` | 조향 노드 | `std_msgs/msg/Bool` | 50 Hz | KEYA에 명령한 enable 상태 |
| `/steering/enabled` | 조향 노드 | `std_msgs/msg/Bool` | 50 Hz | feedback enable; 현재 하드웨어에서는 validity 미지원 |
| `/steering/test/estop` | 벤치 노드 | `std_msgs/msg/UInt8` | 벤치 기본 20 Hz | `test_input_injection_enabled=true`에서만 구독 |

구현 위치:

- `firmware/STM_A/Core/Src/app_freertos.c`
- `firmware/STM_B/Core/Src/app_freertos.c`
- `software/src/lateral_ctrl/src/steering_actuator_node.cpp`
- `software/src/vehicle_state_publisher/vehicle_state_publisher/vehicle_state_publisher_node.py`

## 5. STM_A ↔ STM_B UART

### 5.1 공통 frame

- Peripheral: 두 보드 모두 `USART1`
- Pin: TX `PC4`, RX `PC5`
- 설정: `115200 baud`, 8 data bits, no parity, 1 stop bit, no flow control
- RX: Receive-to-Idle circular DMA → ISR ring buffer → Task parser
- TX: `HAL_UART_Transmit(..., 10 ms)` blocking 호출
- byte order: multi-byte payload는 little-endian
- SOF: `0xAA 0x55`
- sequence: 8-bit, 매 frame 1 증가하며 wrap 허용
- CRC-8: polynomial `0x07`, initial `0x00`, type부터 payload 마지막까지 계산

공통 layout:

| Byte | Field | Type | 설명 |
|---:|---|---|---|
| 0 | SOF1 | `uint8` | `0xAA` |
| 1 | SOF2 | `uint8` | `0x55` |
| 2 | message type | `uint8` | `0x01`, `0x02` 또는 `0x03` |
| 3 | payload length | `uint8` | payload byte 수 |
| 4 | sequence | `uint8` | 송신 때마다 증가 |
| 5.. | payload | bytes | message별 layout |
| 마지막 | CRC-8 | `uint8` | byte 2부터 payload 끝까지 |

CRC가 맞지 않는 frame은 폐기합니다. sequence gap은 error count만 증가시키며
CRC가 유효하면 payload는 반영합니다.

### 5.2 Vehicle Status Frame

- 송신: STM_A
- 수신: STM_B
- message type: `0x01`
- 주기: 20 ms (`VEHICLE_TX_PERIOD_MS`)
- 수신 timeout: 200 ms (`VEHICLE_COMM_TIMEOUT_MS`)
- frame length: 9 bytes
- payload length: 3 bytes
- 포함 데이터: mode, gear, 현재 추정 속도

| Byte | Field | Type | Unit | 설명 |
|---:|---|---|---|---|
| 0..4 | 공통 header | - | - | SOF, type, length, sequence |
| 5 bit 0 | mode | `uint1` | - | `0=MANUAL`, `1=AUTO` |
| 5 bit 1 | gear | `uint1` | - | `0=FORWARD`, `1=REVERSE` |
| 5 bit 2..7 | reserved | bits | - | 송신값 0 |
| 6 | speed LSB | `int16` 일부 | `0.01 m/s` | little-endian |
| 7 | speed MSB | `int16` 일부 | `0.01 m/s` | 범위 clamp 후 전송 |
| 8 | CRC-8 | `uint8` | - | byte 2..7 |

STM_B timeout은 `VEHICLE_FRAME_VALID_FROM_A`를 false로 만들어 Arduino LCD의 `A-B LOST`
badge와 `STM A COMM LOST` 배너를 표시하지만, 현재 `BRAKE_REQUEST` 계산에는 직접
사용되지 않습니다.

### 5.3 Drive Detail Frame

- 송신: STM_A
- 수신: STM_B
- message type: `0x03`
- 주기: 100 ms (`DRIVE_DETAIL_TX_PERIOD_MS`)
- 수신 timeout: 300 ms (`DRIVE_DETAIL_COMM_TIMEOUT_MS`)
- frame length: 22 bytes
- payload length: 16 bytes
- 용도: ROS 없이 STM_A가 이미 보유한 EZkontrol·command 상태를 STM_B dashboard로 전달

| Byte | Field | Type | Unit | 설명 |
|---:|---|---|---|---|
| 0..4 | 공통 header | - | - | SOF, type, length, type별 sequence |
| 5 bit 0 | Part I valid | `uint1` | - | bus voltage·bus/phase current 유효 |
| 5 bit 1 | Part II valid | `uint1` | - | 온도·fault·RUN feedback 유효 |
| 5 bit 2 | tractive commanded | `uint1` | - | STM_A가 CAN command를 실었음 |
| 5 bit 3 | tractive feedback | `uint1` | - | EZkontrol RUN feedback |
| 5 bit 4 | command received | `uint1` | - | Host command 수신 이력 있음 |
| 5 bit 5 | command timeout | `uint1` | - | AUTO command timeout |
| 5 bit 6 | drive stop | `uint1` | - | STM_A 최종 traction stop 요청 |
| 5 bit 7 | reserved | `uint1` | - | 송신값 0 |
| 6 | pedal percent | `uint8` | `%` | MANUAL 페달 0..100 |
| 7..8 | bus voltage | `uint16` | `0.1 V` | little-endian |
| 9..10 | bus current | `int16` | `0.1 A` | little-endian |
| 11..12 | phase current | `int16` | `0.1 A` | little-endian |
| 13..14 | controller temperature | `int16` | `0.1 °C` | little-endian |
| 15..16 | motor temperature | `int16` | `0.1 °C` | little-endian |
| 17..20 | motor fault bits | `uint32` | - | little-endian, 하위 24비트 사용 |
| 21 | CRC-8 | `uint8` | - | byte 2..20 |

`0x03` sequence는 `0x01`과 별도로 증가합니다. detail timeout이나 Part validity
저하는 dashboard의 해당 값을 `--`로 만들 뿐 `VEHICLE_FRAME_VALID_FROM_A`나 brake 판단을
바꾸지 않습니다. 따라서 새 STM_B와 구형 STM_A, 구형 STM_B와 새 STM_A 조합에서도
기존 mode·gear·speed·safety frame은 계속 동작합니다.

### 5.4 Safety Status Frame

- 송신: STM_B
- 수신: STM_A
- message type: `0x02`
- 주기: 20 ms (`SAFETY_TX_PERIOD_MS`)
- 수신 timeout: 100 ms (`STM_B_COMM_TIMEOUT_MS`)
- frame length: 7 bytes
- payload length: 1 byte
- 포함 데이터: STM_B의 최종 brake request

| Byte | Field | Type | 설명 |
|---:|---|---|---|
| 0..4 | 공통 header | - | SOF, type, length, sequence |
| 5 bit 0 | brake request | `uint1` | `1`이면 STM_A traction stop |
| 5 bit 1..7 | reserved | bits | 송신값 0 |
| 6 | CRC-8 | `uint8` | byte 2..5 |

첫 유효 frame 전과 100 ms timeout 동안 STM_A는 safety frame을 invalid로 보고
traction stop을 요청합니다.

구현 위치:

- `firmware/STM_A/Core/Src/app_freertos.c`
- `firmware/STM_A/Core/Src/usart.c`
- `firmware/STM_B/Core/Src/app_freertos.c`
- `firmware/STM_B/Core/Src/usart.c`
- `firmware/common/board_uart_protocol.{h,c}`

## 6. EZkontrol CAN

### 6.1 Bus와 frame

| 항목 | 현재 구현 |
|---|---|
| Peripheral | STM_A `FDCAN1`, RX `PA11`, TX `PA12` |
| Nominal bitrate | 250 kbit/s |
| 계산 | `170 MHz / (40 × (1 + 13 + 3))` |
| Frame | Classic CAN, 29-bit extended, DLC 8, BRS off |
| Mode | normal, auto retransmission enabled |
| Command ID | `0x0C01EFD0` |
| Status Part I ID | `0x1801D0EF` |
| Status Part II ID | `0x1802D0EF` |
| Endian | 16-bit command·Part I 값 little-endian |

### 6.2 Handshake

EZkontrol이 Part I ID로 `0x55` 8 bytes를 보내면 STM_A는 command ID로
`0xAA` 8 bytes를 보냅니다. ACK를 CAN TX FIFO에 넣는 데 성공해야
`s_motor_handshake_done`이 true가 되며, 그 전에는 traction command를 보내지
않습니다.

Handshake가 끝난 뒤 별도의 주기적 handshake나 명시적 재시작 timer는 없습니다.
다시 `0x55` request를 받으면 handshake 상태를 내리고 ACK를 재전송합니다.

### 6.3 Command frame

STM_A Control task가 handshake 이후 nominal 50 ms마다 송신합니다.

| Byte | Field | Encoding | 설명 |
|---:|---|---|---|
| 0..1 | phase-current target | `(A + 3200) / 0.1`, little-endian | AUTO·MANUAL 한계 480 A |
| 2..3 | motor RPM target/limit | `rpm + 32000`, little-endian | 전진 +4500, 후진 -1500 rpm clamp |
| 4 bit 0 | RUNNING | `1=RUN`, `0=HALTED` | 정지 조건에서 0 |
| 4 bit 1 | control mode | `1=speed`, `0=torque` | AUTO speed, MANUAL torque |
| 5..6 | reserved | 0 | - |
| 7 | life counter | `uint8` | command frame마다 증가, wrap 허용 |

### 6.4 Status Part I

| Byte | Field | Encoding |
|---:|---|---|
| 0..1 | DC bus voltage | unsigned × `0.1 V` |
| 2..3 | DC bus current | unsigned × `0.1 A - 3200 A` |
| 4..5 | phase current | unsigned × `0.1 A - 3200 A` |
| 6..7 | motor RPM | unsigned `- 32000 rpm` |

Part I motor RPM 원본은 `/drive/status.motor_rpm`에 그대로 남깁니다. `CURRENT_SPEED`와
`/drive/status.speed_mps`는 움직이던 중 정확히 `0 rpm`인 Part I 샘플을 직전
nonzero RPM으로 최대 2회 hold한 뒤, 타이어 지름 `0.47 m`, 감속비 `4.60`을 사용해
변환합니다. 세 번째 연속 `0 rpm`부터는 실제 정지로 반영합니다. 이 횟수는
`/drive/status` 발행 횟수가 아니라 Part I CAN frame 기준입니다.

### 6.5 Status Part II

| Byte | Field | Encoding |
|---:|---|---|
| 0 | controller temperature | `raw - 40 °C` |
| 1 | motor temperature | `raw - 40 °C` |
| 2 bit 0 | RUN state | `1=running` |
| 2 bit 1 | control mode | `0=torque`, `1=speed` |
| 3..5 | fault bits | 24-bit little-endian 원본 |
| 6..7 | 현재 코드에서 사용하지 않음 | - |

Part I은 마지막 수신 후 500 ms 미만, Part II는 1 s 미만일 때 valid입니다.
Part I이 끊긴 동안 STM_A의 마지막 속도값은 유지되며 AUTO Torque는 500 ms까지
그 값으로 PI를 계산합니다. 새 속도 관측으로 해석해서는 안 됩니다.
Part I 수신 간격이 200 ms 이상 벌어진 뒤 다시 붙으면 RPM의 짧은 0샘플 보정 이력은
새 stream으로 넘어가지 않도록 초기화합니다. 이는 500 ms 유효성 기준과 별개입니다.
Part II는 1 s부터 invalid로 표시하고 AUTO Torque를 차단합니다.
UNO와 Host 진단에는 Part II의 중간 경고가 없습니다.
STM_A의 `/drive/can_rx_diagnostics`는 1 Hz `UInt32MultiArray`이며 data index
0~6이 차례로 Part I 수신 누적, Part II 수신 누적, RX FIFO 손실 누적,
FIFO 읽기 실패 누적, 현재 RX error counter, 현재 TX error counter,
최대 255에서 포화하는 하드웨어 error logging counter입니다. 마지막 세 값의 `UINT32_MAX`는
하드웨어 counter 조회 실패를 뜻합니다.
수신 횟수는 STM_A가 받아 파싱한 프레임만 세므로, 송신 누락과 물리 버스 손실을
단독으로 구분하지는 못합니다.
AUTO Speed/MANUAL에서는 이 freshness가 기존 status validity에 반영됩니다.
AUTO Torque에서는 둘 중 하나라도 stale이면 PI history reset·0 A/HALTED로 inhibit하며,
두 Part의 fresh 복구를 확인한 뒤 받은 새 Host command가 필요합니다.

구현 위치:

- `firmware/STM_A/Core/Src/fdcan.c`
- `firmware/STM_A/Core/Src/app_freertos.c`

## 7. STM_B → UNO R4 Dashboard CAN

STM_B는 250 kbit/s Classical CAN standard frame 여덟 개로 표시 snapshot을
전송합니다. 각 frame은 protocol version과 같은 `alive` 값을 포함합니다. UNO는
`0x321` STATUS, `0x322` POWER, `0x323` THERMAL, `0x324` FAULT의 native 그룹과
`0x325` AUTO HEALTH, `0x326` AUTO MOTION, `0x327` STEER ACTUAL,
`0x328` STEER TARGET 그룹을 따로 조립합니다. snapshot 주기는 50 ms이고 fresh
native complete snapshot이 300 ms 없으면 UNO가 `CAN LOST`를 표시합니다.

AUTO 그룹은 PC의 `/dashboard/auto_status`를 STM_B가 표시용으로 중계합니다. PC
summary가 300 ms stale이면 STM_B가 `HOST_VALID`를 내리지만 native E-stop frame은
계속 보냅니다. 따라서 UNO의 `AUTO DATA LOST`는 STM_B↔UNO CAN 단절과 다릅니다.

세부 byte layout, scale, signed type과 recovery 절차는
[`STM_B README`](../firmware/STM_B/README.md#85-dashboard-snapshot과-wire-단위)를 따릅니다.
native `0x321`의 Part II valid bit는 1 s부터 내려갑니다.

Dashboard CAN은 STM_B에서 UNO로 차량 상태를 보내는 단방향 표시 경로입니다.
Arduino는 LCD 출력과 physical/HIL touch navigation을 처리하는 local HMI이며 차량
제어 또는 safety command source가 아닙니다. CAN enqueue 실패, bus-off, Arduino reset,
LCD/touch 장애는 STM_B E-stop/brake PWM, STM_A safety UART와 traction 판단에 입력되지
않고, gear, mode, accelerator, brake, E-stop 또는 actuator command를 만들지 않습니다.

### 7.1 UNO → LCD와 FT6336U → UNO

- UNO는 MSP4030/MSP4031의 ST7796S에 SPI로 logical 480×320 화면을 출력합니다.
- MSP4031의 FT6336U는 I2C 7-bit address `0x38`이며 20 ms마다 polling합니다.
- physical raw 320×480 좌표는 `screen_x = raw_y`,
  `screen_y = 319 - raw_x`로 480×320 screen 좌표에 정규화합니다.
- FT6336U INT는 diagnostic falling-edge counter만 유지합니다. 좌표를 얻는 기능
  경로는 INT가 아니라 polling입니다.
- physical touch는 UNO 내부 Gesture/Navigator와 local page navigation으로 이어지며
  ROS나 차량 actuator command를 거치지 않습니다.

I2C 전압·pinout, reset timing과 상세 touch report field는
[UNO README](../firmware/UNO_R4_DASHBOARD/README.md#ft6336u-터치-확인)를 따릅니다.

### 7.2 HIL UNO USB Serial과 Virtual Touch

UNO USB는 **115200 baud, newline-delimited ASCII**입니다. CAN/LCD/local touch는
USB 연결 없이도 동작하며, 아래 인터페이스는 HIL profile에서만 사용합니다.

| 명령/보고 | 방향 | 의미와 주기 |
|---|---|---|
| `HIL_LCD ON/OFF` | HIL → UNO | 적용 화면 보고 enable/disable |
| `HIL_LCD {JSON}` | UNO → HIL | 약 200 ms, 적용 표시값·page·CAN LOST·CAN age·snapshot 누적 |
| `HIL_TOUCH ON/OFF` | maintenance diagnostic client → UNO | physical raw touch diagnostic enable/disable |
| `HIL_TOUCH {JSON}` | UNO → maintenance diagnostic client | 약 100 ms, raw point·I2C/INT diagnostic·CAN age |
| `VT D x y`, `VT M x y`, `VT U x y` | HIL → UNO | 480×320 screen coordinate의 down/move/up |
| `VT C` | HIL → UNO | 현재 virtual gesture를 판정 없이 취소 |

Virtual Touch는 현재 HIL control owner만 사용할 수 있습니다. physical touch가
우선하며 physical release 뒤 150 ms guard를 둡니다. virtual 입력이 1초 끊기거나
연결·ownership·pointer 상태가 해제되면 gesture를 취소합니다. 이 입력은 dashboard
page navigation에만 연결되고 gear, mode, brake, E-stop, accelerator 또는 vehicle
actuator command를 만들지 않습니다.
현재 Web HIL worker는 `HIL_LCD`와 `VT`를 사용합니다. `HIL_TOUCH`는 Serial Monitor
등의 물리 touch 점검용이며 `HIL_LCD` 보고와 상호 배타적이므로 같은 UNO serial을
Web worker와 동시에 열지 않습니다.

```text
Web PointerEvent
→ vehicle_console HIL ownership/revision check
→ existing UNO USB connection
→ common UNO Gesture/Navigator
→ physical LCD page applied
→ HIL_LCD report
→ native renderer
→ Web mirror
```

Web은 요청 전송만으로 page를 optimistic하게 변경하지 않고 UNO가 실제 적용했다고
보고한 `HIL_LCD` page를 표시합니다. queue, MOVE coalescing과 상세 test 결과는
[UNO Virtual Touch](../firmware/UNO_R4_DASHBOARD/README.md#web-hil-virtual-touch-v1)와
[HIL Web Console](../software/src/vehicle_console/HIL_WEB.md#virtual-touch-사용)을
따릅니다.

Web의 토픽 Hz/주기는 PC ROS 수신 간격의 최근 5초 평균이며 개별 CAN 주기로
해석하지 않습니다. MCU가 보고한 CAN age와 PC ROS/USB 수신 age도 서로 다릅니다.

## 8. Steering Motor CAN

> 이 절의 software 관련 내용은 현재 브랜치 구현을 기준으로 한 스냅샷입니다.
> software 구조와 통합 실행 방식은 아직 정리 중이며 이후 변경될 수 있습니다.

현재 실차 경로는 Host의 `steering_actuator_node`가 SocketCAN `can0`을 직접
사용합니다. bitrate는 노드가 설정하지 않으며 실행 전에 외부에서 250 kbit/s로
올려야 합니다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set can0 up
```

`restart-ms`는 주지 않습니다. HIL bench의 USB-CAN은 gs_usb(candleLight 계열)이고
이 드라이버는 bus-off 자동 복구를 지원하지 않아 링크 설정 자체가 거부됩니다.
bus-off에서는 `down` 후 `up`으로 수동 복구합니다.

기본 node ID `1`일 때 ID는 다음과 같습니다.

| 방향 | ID 식 | 기본 ID | 형식 | 설명 |
|---|---|---|---|---|
| Host → KEYA | `0x06000000 + node_id` | `0x06000001` | extended, DLC 8 | command와 query |
| KEYA → Host | `0x05800000 + node_id` | `0x05800001` | extended, DLC 8 | query response |
| KEYA → Host | `0x07000000 + node_id` | `0x07000001` | extended, DLC 8 | heartbeat |

주요 command payload:

| 동작 | 8-byte payload |
|---|---|
| Enable | `23 0D 20 01 00 00 00 00` |
| Disable | `23 0C 20 01 00 00 00 00` |
| Position | `23 02 20 01 <count, big-endian 4 bytes>` |
| Torque | `23 01 20 01 <signed int32, big-endian 4 bytes>`, -10000..10000 |
| Encoder position query | `40 04 21 02 00 00 00 00` |
| Motor speed query | `40 03 21 01 00 00 00 00` |
| Motor current query | `40 00 21 01 00 00 00 00` |
| Fault query | `40 12 21 01 00 00 00 00` |
| Bus voltage query | `40 0D 21 02 00 00 00 00` |
| Driver temperature query | `40 0F 21 01 00 00 00 00` |

제조사 `KEYA KY170DJD01005-08G-V1 User Manual V1.1` 인쇄면 12~13쪽의
query response와 heartbeat 인코딩은 다음과 같습니다. multi-byte 값은 high
byte가 먼저 옵니다. 아래 byte 위치는 8-byte CAN payload의 0-based index입니다.

| 응답 | payload layout | 값 위치·폭 | 의미 |
|---|---|---|---|
| Encoder position query | `60 04 21 02 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)` | byte 4~7, 4 byte | position count (`10000 count/rev`) |
| Motor speed query | `60 03 21 01 DATA_H(h) DATA_H(l) DATA_L(h) DATA_L(l)` | byte 4~7, 4 byte | motor speed `[rpm]` |
| Motor current query | `60 00 21 01 00 00 00 DATA` | byte 7, 1 byte | working current `[A]` |
| Fault query | `60 12 21 01 DATA_H DATA_L 00 00` | byte 4~5, 2 byte | failure code |
| Bus voltage query | `60 0D 21 02 00 00 00 DATA` | byte 7, 1 byte | supply voltage `[V]` |
| Driver temperature query | `60 0F 21 01 00 00 DATA 00` | byte 6, 1 byte | driver temperature `[°C]` |
| Heartbeat | `ELE_H ELE_L SPEED_H SPEED_L CURRENT_H CURRENT_L ERR_H ERR_L` | position byte 0~1, speed 2~3, current 4~5, fault 6~7 | position `360 count/rev`, speed `[rpm]`, current `[A]`, failure code. 현재 driver는 heartbeat position을 사용하지 않음 |

control tick 20 ms마다 position을 조회합니다. `position` 모드의 ACTIVE에서는
rate-limit된 position command를, 현재 YAML의 `torque` 모드에서는 외부 PD의
torque command를 전송합니다. fault는 100 ms, speed·current는 200 ms,
bus voltage·driver temperature는 500 ms 간격으로 번갈아 조회합니다.

- CAN timeout: heartbeat 또는 feedback frame이 500 ms 동안 없을 때
- position feedback timeout: position 모드는 500 ms, torque 모드는 100 ms 동안
  유효 position response가 없을 때
- KEYA 매뉴얼은 speed·torque command 간격이 1000 ms를 넘으면 CAN disconnect
  error가 발생하고 재시작 시 Enable을 다시 보내야 한다고 규정. 현재 통합 경로의
  position command에도 같은 watchdog과 출력 차단이 적용되는지는 미확인
- startup-only `actuation_mode=position`은 KEYA 0020=3, `torque`는 0020=2가 필요.
  Host는 KEYA 내부 mode를 설정하거나 검증하지 않음
- motor-axis diagnostic scaling: `10000 count/rev`
- 조향 calibration은 raw KEYA count를 중앙 등가각으로 직접 환산하며
  `software/src/lateral_ctrl/config/steering_motor_calibration.yaml`이 소유
- 명령 범위: `-5300..5300 count`(nominal 약 `-28.33..28.03 deg`), feedback은 클램프하지 않음

Torque mode는 같은 canonical `/vehicle/command.steering_target_rad`를 직접 PD에
전달한다. `torque_control_period_ms` 기본20 ms, 위치 timeout은
`min(feedback_timeout_ms, torque_feedback_timeout_ms)` 기본100 ms다.
Driver는 kernel 수신 timestamp로 freshness와 위치 차분 간격을 계산한다.
ACTIVE 진입은 torque0→Enable, 종료는 torque0→Disable이며 position frame을 보내지
않는다. peak/sustained raw envelope는 Amp 변환이 아니다.

`kaiev26_msgs` ABI 변경 없이 torque 관측값을 `std_msgs` 보조 토픽에 제공한다.
`/steering/actuation_mode`는 String/transient-local(두 mode 공통)이고, torque mode에서
`/steering/requested_angle`, `/steering/angle_error`(rad), `/steering/rate`,
`/steering/unfiltered_rate`(rad/s), `/steering/torque_p_raw`, `/steering/torque_d_raw`,
`/steering/torque_unsaturated_raw`와 `/steering/torque_current_limit_command_a`는
Float32다. 후자는 KEYA Parameter 0003과 매뉴얼의 80% 식으로 계산한 signed 명령
전류 제한값이며 실제 입력/모터 전류가 아니다. `/steering/torque_command_raw`와
`/steering/torque_limit_raw`는 Int32, `torque_saturated`, `torque_peak_limited`,
`torque_settled`, `torque_rate_ready`는 같은 prefix의 Bool이다.
`/steering/torque_safety`(String)는 현재 Host torque feedback stop 원인을 보고하며,
정상 feedback에서 `none`으로 돌아간다.
상세 unit·설정·튜닝 계약은
[`TORQUE_CONTROL.md`](../software/src/lateral_ctrl/TORQUE_CONTROL.md)를 따른다.

구현 위치:

- `software/src/lateral_ctrl/src/keya_can_driver.cpp`
- `software/src/lateral_ctrl/src/steering_actuator_node.cpp`
- `software/src/lateral_ctrl/config/steering_actuator.yaml`

## 9. Timeout and Freshness

| 항목 | 값 | 근거 상수/설정 | 구현 위치 |
|---|---:|---|---|
| STM_A Control command | 50 ms | `RuntimePeriodic`의 50 ms 주기 | STM_A `app_freertos.c` |
| STM_A Host command timeout | 500 ms | `HOST_COMMAND_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM_B safety TX | 20 ms | `SAFETY_TX_PERIOD_MS` | STM_B `app_freertos.c` |
| STM_A vehicle TX | 20 ms | `VEHICLE_TX_PERIOD_MS` | STM_A `app_freertos.c` |
| STM_B→STM_A UART timeout | 100 ms | `STM_B_COMM_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM_A→STM_B UART timeout | 200 ms | `VEHICLE_COMM_TIMEOUT_MS` | STM_B `app_freertos.c` |
| Drive status | 20 ms | `DRIVE_STATUS_PERIOD_MS` | STM_A `app_freertos.c` |
| Brake status | 50 ms + 변경 시 | `BRAKE_STATUS_PERIOD_MS` | STM_B `app_freertos.c` |
| Dashboard CAN snapshot | 50 ms | `DASHBOARD_CAN_SLOT_PERIOD_MS × DASHBOARD_CAN_SLOTS_PER_SNAPSHOT` | STM_B `dashboard_can.c` |
| Arduino Dashboard CAN lost | 300 ms | `CAN_LOST_TIMEOUT_MS` | `UNO_R4_DASHBOARD.ino` |
| Dashboard CAN recovery backoff | 1000 ms | `DASHBOARD_CAN_RECOVERY_BACKOFF_MS`, `CAN_REINIT_PERIOD_MS` | STM_B `dashboard_can.c`, UNO firmware |
| EZkontrol Part I validity | 500 ms | `MOTOR_FEEDBACK_TIMEOUT_MS` | STM_A `app_freertos.c` |
| Part I RPM 보정 이력 초기화 | 200 ms 수신 간격 | `MOTOR_RPM_STREAM_RESET_MS` | STM_A `app_freertos.c`; traction 차단 아님 |
| EZkontrol Part II validity / AUTO Torque control gate | 1000 ms | `MOTOR_PART2_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM Agent ping | 500 ms, call timeout 100 ms | `AGENT_PING_*` | 두 STM `app_freertos.c` |
| 조향 control/status | 20 ms | `control_period_ms` | `steering_actuator.yaml` |
| 조향 command timeout | 200 ms | `steer_command_timeout_ms` | `steering_actuator.yaml` |
| 조향 CAN timeout | 500 ms | `can_timeout_ms` | `steering_actuator.yaml` |
| 조향 position timeout | 500 ms | `feedback_timeout_ms` | `steering_actuator.yaml` |
| Torque control/status | 20 ms | `torque_control_period_ms` | `steering_actuator.yaml` |
| Torque position timeout | 100 ms | `torque_feedback_timeout_ms` (기존 timeout과 작은 값) | `steering_actuator.yaml` |
| 조향 차속 feedback timeout | 500 ms | `drive_status_timeout_ms`; 최소속도 0에서도 적용 | `steering_actuator.yaml` |
| 조향 mode timeout | 1500 ms | `mode_timeout_ms` | `steering_actuator.yaml` |
| 거버너 입력 나이 게이트 | 200 ms | `input_timeout_ms` | `command_governor.yaml` |
| `/vehicle/state` publish | 50 ms | `publish_rate_hz=20.0` | vehicle state publisher |
| `/vehicle/wheel_twist` publish | 최대 50 ms | state publisher timer + valid feedback gate | vehicle state publisher |
| 차량 status `/diagnostics` publish | 1000 ms | `publish_rate_hz=1.0` | diagnostics publisher |
| diagnostics 입력 status stale | 1000 ms | `STATUS_STALE_SEC` | diagnostics publisher |

경계 비교는 구현마다 다릅니다. STM Host와 UART timeout은 `>=`, EZkontrol valid는
Part I valid는 `age < 500 ms`, Part II status valid는 `age < 1000 ms`입니다.
AUTO Torque는 둘 중 하나가 invalid면 차단합니다. 조향 timeout은 `age > 설정값`을 사용합니다.

### 9.1 Web HIL의 표시·조작 연결

| 항목 | 값 | 의미 |
|---|---:|---|
| Web 화면 갱신 | 200 ms | WebSocket 상태와 PC 생성 LCD 이미지 |
| UNO LCD 보고 | 200 ms | USB로 적용 상태만 보고; framebuffer 전송 없음 |
| Web ROS/UNO 보고 stale 표시 | 1000 ms | 표시용, MCU의 CAN timeout을 변경하지 않음 |
| Web 조작권 heartbeat | 3000 ms | 조작권 회수와 명령 발행 해제 요청 |
| Web 전용 generator watchdog | 3000 ms | Web heartbeat 소실 시 `enabled=false`; 기존 launch는 비활성 |

## 10. 관련 구현 파일

- STM_A: `firmware/STM_A/Core/Src/app_freertos.c`
- STM_A CAN: `firmware/STM_A/Core/Src/fdcan.c`
- STM_A serial: `firmware/STM_A/Core/Src/usart.c`
- STM_B: `firmware/STM_B/Core/Src/app_freertos.c`
- STM_B PWM: `firmware/STM_B/Core/Src/tim.c`
- STM_B dashboard CAN: `firmware/STM_B/Core/Src/dashboard_can.c`, `dashboard_can_protocol.c`
- Arduino dashboard: `firmware/UNO_R4_DASHBOARD/`
- 조향: `software/src/lateral_ctrl/src/`
- 명령 거버너: `software/src/command_governor/src/`
- 상태·wheel twist·diagnostics: `software/src/vehicle_state_publisher/`
- Foxglove bridge·layout: `software/src/foxglove/`
- 메시지 원본: 별도 workspace repository `kaiev26_msgs/msg/`

## 11. 확인 필요 항목

- Production `/planning/command` publisher (Planning)와 발행률.
  `/vehicle/command` 발행자는 `command_governor`로 확정됨
- STM_A→STM_B UART timeout을 브레이크 판단에 포함할지 여부
- EZkontrol Part II 장기 미수신을 traction stop 조건으로 포함할지 여부
- EZkontrol status 실제 송신 주기
- KEYA 내부 watchdog 시간과 현재 position command의 keep-alive 보장
- STM_A·STM_B PCB connector 기준 실제 배선 및 signal ground
- `BrakeActuatorStatus`와 `/vehicle/state` input freshness 정책


## Longitudinal calibration v2

STM_A AUTO longitudinal config/status uses `std_msgs/msg/Float32MultiArray` with
empty `layout.dim` and `layout.data_offset=0`. The bounded MCU buffers accept the exact
15-float request and 27-float status payload lengths. v1 speed/torque requests remain supported. Unsupported version/length/nonfinite/config ranges are rejected;
malformed DDS sequences that cannot deserialize have no acknowledgment. `DriveCurrentCommand` and regenerated micro-ROS libraries are required for external-current control.
`DriveActuatorStatus` retains its existing layout.

`/drive/longitudinal_config`: reliable, volatile (never transient-local), 15 floats:

| Index | Field |
|---|---|
| 0 | version=2 |
| 1 | request_id: integer 1…16777215 |
| 2 | expected_revision: integer 0…16777215 |
| 3 | control_mode: 0=speed, 1=torque PI, 2=external_current (different from EZ feedback bit!) |
| 4–14 | Kp, Ki, max_drive_current, integrator_limit, slew_up, slew_down, deadband, hysteresis, reapply_current, reapply_slew, motor_rpm_limit |

Current fields are phase amperes; Kp A/(m/s), Ki A/(m/s*s), slew A/s,
deadband/hysteresis m/s, RPM motor rpm. Field order is defined by
`control_tuning.longitudinal.CONFIG_KEYS` and `Long_EncodeConfig`.

STM_A validates the complete request, then hands it to a one-slot mailbox. The next
Control cycle checks MANUAL, fresh Part I <500 ms and Part II <1 s, |speed|≤0.05 m/s, pedal=0,
|commanded phase|<0.1 A and |actual phase|≤1 A. Under one FreeRTOS critical section,
expected_revision must equal applied revision; then all config fields/revision/source
commit together and PI resets. A rejected request never waits for future motion state.
Duplicate last-applied request ID is idempotent. Revision exhaustion requires reboot.
IDs/revisions use the exact integer range of float32; they are not authorization tokens.
Use the existing trusted vehicle ROS domain, not an untrusted network.

`/drive/longitudinal_status`: best effort, 5 Hz, 27 floats:

| Index | Field |
|---|---|
| 0 | version=2 |
| 1 | applied revision; boot=0 |
| 2 | last request ID; malformed request=0 |
| 3 | result: 0=idle, 1=applied, 2=invalid, 3=unsafe, 4=revision conflict, 5=busy |
| 4 | calibration source: 0=DEFAULT, 1=HOST_OVERRIDE |
| 5–16 | active config in CONFIG_KEYS order, including mode |
| 17–21 | speed error, P, I/integrator state, unsaturated current, final PI current |
| 22–25 | saturated, anti-windup, coasting, AUTO Torque control feedback usable (Part I <500 ms and Part II <1 s; 0/1) |
| 26 | inhibit: 0=OK, 1=disabled, 2=feedback loss, 3=wait fresh command, 4=reverse unsupported, 5=stop |

Read actual CAN-transmitted current/RPM and actual phase/bus current from `/drive/status`.
In speed mode its commanded_phase_current_a is a limit, not commanded torque.
PI status final current is the controller output before CAN quantization, not measured current.
MANUAL leaves the pedal path unchanged and resets PI. Safety gates bypass slew to zero/HALTED.
After Torque feedback recovery, a command received after recovery is required.
Agent reconnect preserves RAM calibration but clears host commands/pending requests;
MCU reboot restores source DEFAULT/revision 0/mode speed. Host clients do not auto-replay overrides.

The launch calibration helper compares the matching request ID, revision and full float32
readback before reporting success. The TUI only observes current settings and saves presets;
it does not send calibration or parameter writes. Saved values take effect on the next
control-stack launch with that preset. For calibration, test procedure,
range rationale and recorder coverage see
[the implementation report](reports/2026-09-21_longitudinal_torque_pi.md).


## External phase-current command

`/vehicle/drive_current_command` uses `kaiev26_msgs/msg/DriveCurrentCommand`:

| Field | Meaning |
|---|---|
| `header.stamp` | Publisher ROS clock, strictly advancing for accepted samples |
| `phase_current_a` | Forward motor phase-current demand in A, 0–480 protocol envelope |

Only `external_current` mode consumes this topic. The selected operating current/RPM envelope
may be lower than the protocol maximum. `/vehicle/command` remains governor-owned and independently
required: positive forward target, released brake and fresh host command. PC MPC, host reference PI
and current-identification scenarios are alternative producers of the current topic.

STM_A uses a separate 500 ms reception watchdog for current commands. A received current stamp must
be at least the most recent accepted target stamp and less than 500 ms ahead of it. Current source
stamps reject reordered commands and commands queued before a stop; HAL ticks measure reception age.
They do not imply that the MCU is synchronized to the PC's UTC clock. Invalid current samples do not
renew the watchdog. Brake, drive stop, mode changes, handshake, reconnect, invalid feedback and
valid nonzero motor fault clear stored current. Feedback recovery also retains the existing
requirement for a new upper-level command. Safety inhibits bypass current slew and send HALTED.

The MCU performs no speed PI in this mode. CAN current remains in phase amperes, torque mode is
bit1=0, and the configured motor RPM is a ceiling. Read actual CAN-transmitted current from
`/drive/status.commanded_phase_current_a`; calibration status elements 17–24 are PI diagnostics.
The new current message type, STM_A library/firmware and Host must be deployed together. New Host
configuration clients read v1/v2 status and refuse external-current selection on v1 firmware.

The current MPC model and all controller settings are fixed for a run. The Host publishes read-only
`model_A`, `model_B`, `model_c` and `model_dt` parameters containing the loaded coefficients.
The existing recorder captures these values when `/longitudinal_controller` is running.

## Control Tuning parameter names

TUI/preset keys use `steering_`, `drive_`, `governor_` prefixes with physical units.
[The complete mapping and migration contract](CONTROL_TUNING_PARAMETERS.md) defines
preset schema v3 and parameter snapshot schema v2. Existing v2 presets and v1 snapshots
remain readable through explicit name adapters; conflicting aliases fail instead of
silently selecting a value. All managed values remain required.

Node-scoped ROS parameter names and `/parameter_events` keep their native names/units.
The MCU array ABI remains v1. Its host decoder adds canonical `display_values` while
retaining the existing `config` object for old clients. The topic/message types remain unchanged. The naming adapter needs no firmware
rollout, but the shared AUTO Speed/Torque envelope requires the updated STM_A firmware. Control Tuning remains read-only with respect to runtime:
S saves the selected preset for the next control-stack launch.


### AUTO drive envelope semantics

The ABI v1 `max_drive_current_a` and `motor_rpm_limit` fields (canonical
`drive_current_limit_a`, `drive_motor_speed_limit_rpm`) apply to both AUTO modes.
Speed uses them as the EZ native current-limit field and symmetric signed target-RPM
envelope. Torque uses them as PI phase-current ceiling and EZ torque-mode RPM ceiling.
The final sender still clamps forward to 4500 rpm and reverse to 1500 rpm.
MANUAL pedal/gear bypasses this AUTO calibration envelope.

Airborne first-test candidate: Kp15/Ki2, 20 A, integral6 A, slew120/400 A/s,
reapply5 A at40 A/s, RPM600, deadband0.05/release0.12 m/s.
Driving remains 480 A/RPM4500 with unchanged gains and CAN baseline.
PI gains, integral, deadband/coast and slew/reapply remain Torque-only. Firmware
Part I freshness is 500 ms; Part II status becomes invalid at 1 s.
AUTO Torque resets/inhibits to 0 A/HALTED if either Part is invalid and requires a Host
command received after feedback recovery. Speed/MANUAL continue to report the valid flags.

## 12. RP2350 LV 배터리 계측·정비

RP2350 UART0 TX(`D6/GPIO0`) → STM_B UART4 RX(`PC11/CN7-2`),
RP2350 UART0 RX(`D7/GPIO1`) ← STM_B UART4 TX(`PC10/CN7-1`)이며 공통 GND를 사용합니다.
3.3 V, 115200 8N1의 양방향 CRC32 binary protocol입니다.
frame·단위·정비 command는 [LV 프로토콜](rp2350/lv_battery_gauge.md#9-battery-uart-protocol)을 따릅니다.

| Topic | 방향 | Type | 내용 |
|---|---|---|---|
| `/lv_battery/state` | STM_B → ROS | `sensor_msgs/msg/BatteryState` | 전압·전류·용량·SOC; 100 ms |
| `/lv_battery/link/status` | STM_B → TUI | `std_msgs/msg/UInt8MultiArray` | 136-byte status payload; 100 ms |
| `/lv_battery/link/command` | TUI → STM_B → RP2350 | `std_msgs/msg/UInt8MultiArray` | 12-byte 정비 command payload |
| `/lv_battery/link/ack` | RP2350 → STM_B → TUI | `std_msgs/msg/UInt8MultiArray` | 16-byte 저장 결과 ACK payload |

RP2350 내부 전류는 방전 양수, `BatteryState.current`는 방전 음수입니다.
UART status 수신 후 1초가 지나면 `present=false`이며 전압·전류는 NaN입니다.
유효 계측 여부와 SOC 설정 여부는 별도로 판정합니다. 온도는 배터리 센서가 없으므로
NaN이며 INA die 온도를 대신 넣지 않습니다.

LV 계기판은 기존 native `0x324` byte 5(flags), 6(0.1 V), 7(SOC %, unknown `0xFF`)를
사용합니다. 별도 CAN ID는 추가하지 않으며 UART 1초 단절 시 LV flags를 내립니다.
계측 상태는 ROS 상태 통합 3종이나 E-stop·traction 제어 입력에 포함되지 않습니다.

## 식별 기반 횡방향 MPC

`lateral_ctrl`의 `torque_controller=pd|mpc`는 같은 `/vehicle/command` 목표각과
같은 KEYA 송신·안전 처리를 사용한다. MPC 모델 파일은 `steering_model_file`로 지정하며,
`/steering/torque_command_raw`는 실제 전송한 signed KEYA raw 명령을 계속 발행한다.

경로 MPC는 기존 Decision의 `lateral_controller=mpc`로 선택한다.

| 토픽 | 타입 | 생산자 → 소비자 | 의미 |
| --- | --- | --- | --- |
| `/control/lateral/target` | `ackermann_msgs/AckermannDriveStamped` | C++ `lateral_path_mpc` → Decision motion-control | 계산 시각과 LEFT-positive 중앙 등가 목표각 `drive.steering_angle` [rad] |
| `/control/lateral/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 경로 MPC → Decision/recorder | 해 상태, 횡오차·방향오차, solve/compute 시간 |
| `/control/steering/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 조향 MPC → recorder | 해 상태, 반복 횟수, solve/compute 시간 |

Decision은 MPC 선택 시 위 목표각을 속도·제동 명령과 합친다. 실패 진단 또는 150 ms를 넘긴
해는 기존 정지 처리로 연결한다. 기존 `/planning/command` → governor → `/vehicle/command`
소유권을 유지한다. 경로 입력 `Centerline`의 좌표계는 `base_footprint`, 각도 단위는 rad다.

`path_model_file`에는 내부 조향 MPC를 켜고 측정한 폐루프 응답과 차량 yaw 응답이 들어간다.
목표각·목표각 변화속도는 식별된 입력 범위와 기존 차량 설정의 교집합으로 제한한다.
실제 조향 상태에는 차량의 각도 한계를 적용한다. 학습 입력 범위는 기계적 최대 능력의 측정값이 아니다.
