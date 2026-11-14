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
finite)를 거쳐 양수 목표의 하강률을 제한하고 재-stamp 후 재발행하며,
자체 타이머와 keep-alive가 없으므로 발행률은
입력을 그대로 따릅니다. 따라서 상류(실차 시험은
`vehicle_test_scenario_runner`, HIL은 `hil_command_generator`, 모두 20 Hz)가
STM_A의 500 ms와 조향의 200 ms freshness 한계보다 충분히 빠르게 발행해야
합니다. 5 Hz는 조향 200 ms의 경계라 간헐 timeout이 납니다.
감속 slew는 `deceleration_slew_mps2` (기본 0.5 m/s², 0이면 해제)입니다.
Speed 프리셋에서는 `acceleration_slew_mps2` (초기값 1.0 m/s², 0이면 해제)로
양수 목표 상승률도 제한합니다. Torque/외부 전류에서는 가속 제한을 해제합니다.
가속 제한이 켜져 있으면 첫 명령과 입력 공백 후 재개는 0부터 시작합니다.
음수 목표는 그대로 전달하고, 0 목표는 즉시 전달해 STM_A가 0 A/HALTED
Torque 명령으로 타력감속합니다. Speed의 0 RPM hold나 회생 ramp를 유지하지 않습니다.
`brake_engage`도 지연하지 않습니다. 유효 입력 공백이 `input_timeout_ms`를 넘으면
이전 slew 이력을 버립니다. Speed 선택은 정지 상태 설정 적용 후 CAN control-mode bit 1로 반영합니다.
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
- EZkontrol status 실제 송신 주기
- KEYA 내부 watchdog 시간과 현재 position command의 keep-alive 보장
- STM_A·STM_B PCB connector 기준 실제 배선 및 signal ground
- `BrakeActuatorStatus`와 `/vehicle/state` input freshness 정책


<a id="longitudinal-calibration-v5"></a>

## 종방향 캘리브레이션 v5

STM_A의 AUTO 종방향 설정·상태는 `std_msgs/msg/Float32MultiArray`를 사용하며,
`layout.dim`은 비우고 `layout.data_offset=0`으로 둡니다. MCU의 고정 크기 버퍼는
정확히 40개 float로 구성된 v5 요청을 받고, 78개 float로 구성된 v5 상태를 발행합니다.
v4(27개), v3(16개), v1/v2(15개) 요청도 수신할 수 있습니다. 구형 요청에 없는 필드는
아래 v5 부팅 기본값(회생 꺼짐)으로 채우며, v3의 인덱스 12 값은 주행 중 최소 구동 전류로 해석합니다.
현재 Host 클라이언트는 v5 요청만 보내고 구형 펌웨어는 거부합니다. 지원하지 않는 버전·길이,
유한하지 않은 값, 설정 범위 위반은 거부하며, 역직렬화할 수 없는 DDS sequence에는 응답하지 않습니다.
외부 전류 제어에는 `DriveCurrentCommand`와 이를 포함해 다시 생성한 micro-ROS 라이브러리가 필요합니다.
`DriveActuatorStatus`의 기존 레이아웃은 유지합니다.

`/drive/longitudinal_config`: reliable, volatile(transient-local 사용 안 함), float 40개:

| 인덱스 | 필드·의미 |
|---|---|
| 0 | version=5 |
| 1 | request_id: 정수 1…16777215 |
| 2 | expected_revision: 정수 0…16777215 |
| 3 | control_mode: 0=speed, 1=torque PI, 2=external_current. EZ 피드백의 모드 비트와는 다름 |
| 4–14 | Kp, Ki, max_drive_current, integrator_limit, slew_up, slew_down, deadband, hysteresis, reapply_current, reapply_slew, motor_rpm_limit |
| 15 | min_drive_current: 주행 중 최소 전진 구동 전류, 0…min(max_drive_current, 30). 0이면 16–19의 제어 효과를 사용하지 않음 |
| 16 | motor_waveform: 0=square, 1=sine. 운영자가 EZ의 구동 파형을 지정하며 STM_A는 수신값만 보고 |
| 17 | start_step: 0/1. 주행 유지가 꺼진 상태에서 전진 요청이 (0, min_drive_current)이면 min_drive_current로 올림 |
| 18 | hold_on_speed: 0.1…2 m/s |
| 19 | hold_off_speed: 0 … < hold_on_speed m/s |
| 20 | reverse_speed: 0.05…1 m/s |
| 21 | reverse_debounce_frames: 정수 1…10, frame당 50 ms |
| 22 | collapse_hold_frames: 정수 0…40. 0이면 관측·집계만 수행 |
| 23 | ff_current_gain: 0(feedforward 꺼짐) 또는 0.0005…0.2 (m/s²)/A |
| 24–26 | ff_rolling_resistance 0…2 m/s², ff_speed_damping 0…1 1/s, ff_accel_limit 0…5 m/s² |
| 27 | regen_max_current: 회생 전류 크기 한도 0…60 A, 송신 시 음수. **0이면 모든 회생 비활성화** |
| 28 | regen_slew: 증가·감소 모두 10…1000 A/s |
| 29–30 | regen_fade_speed 0.3…5 m/s; regen_cutoff_speed 0.1 … < fade m/s |
| 31–32 | regen_voltage_soft 40 … < hard V; regen_voltage_hard … 60 V |
| 33 | regen_bus_current_limit: 실측 충전전류 기준 1…39 A |
| 34 | regen_rpm_field: 0=기존처럼 RPM 필드에 0, 1=motor_rpm_limit를 양수 상한으로 사용 |
| 35 | pi_regen: 0/1. 속도 PI의 회생 요청 허용, AUTO Torque 전용 |
| 36–38 | regen_enter_error 0.05…2 m/s; regen_exit_error 0 … < enter; regen_kp 0…500 A/(m/s) |
| 39 | collapse_phase_low: −200…−10 A. 최근·직전 명령이 모두 ≥ 0인 Torque frame에서 상전류가 이 값보다 낮으면 붕괴로 판정 |

전류 필드의 단위는 모터 상전류 A입니다. Kp는 A/(m/s), Ki는 A/(m/s*s), slew는 A/s,
deadband·hysteresis는 m/s, RPM은 모터 rpm입니다. 필드 순서는
`control_tuning.longitudinal.WIRE_KEYS`와 `Long_EncodeConfig`가 정의합니다.
Host는 `min_drive_current`, `ff_current_gain`, `collapse_phase_low`를 파형별
(`*_square_*`, `*_sine_*`)로 저장하고 지정한 파형의 값을 보냅니다. 나머지 필드는 공통입니다.

부팅 기본값은 Torque·sine, 최소 전류 0(최소 전류 기반 보호 미적용), start step 켜짐입니다.
hold는 0.35/0.20 m/s, 역회전 판정은 0.30 m/s × 3 frame, 붕괴 대응과 feedforward는 0입니다.
회생 한도는 0(꺼짐), 회생 slew는 100 A/s, fade/cutoff는 1.5/0.8 m/s,
전압은 57/58 V, 충전전류는 30 A, RPM 필드는 0입니다.
PI 회생은 꺼짐이며 진입·해제 오차 0.3/0.1 m/s와 gain 0을 사용합니다. 붕괴 상전류 하한은 −60 A입니다.

STM_A는 전체 요청을 검증한 뒤 요청 하나를 담는 mailbox에 전달합니다. 다음 Control 주기에서
MANUAL, Part I 수신 경과 <500 ms, Part II <1 s, |속도|≤0.05 m/s, 페달=0,
|지령 상전류|<0.1 A, |실측 상전류|≤1 A를 확인합니다. 하나의 FreeRTOS critical section 안에서
`expected_revision`이 적용 중인 revision과 같은지 확인하고, 모든 설정 필드·revision·source를
함께 반영한 뒤 PI를 초기화합니다. 거부된 요청은 차량이 나중에 적용 가능 상태가 될 때까지 대기하지 않습니다.
마지막으로 적용한 request ID를 다시 받으면 중복 적용하지 않습니다. revision 범위를 모두 사용하면 재부팅이 필요합니다.
ID와 revision은 float32가 정확하게 표현할 수 있는 정수 범위를 사용하며 인증 토큰이 아닙니다.
신뢰할 수 있는 기존 차량 ROS domain에서 사용합니다.

`/drive/longitudinal_status`: best effort, 5 Hz, float 78개(v4: 57개, v3: 28개):

| 인덱스 | 필드·의미 |
|---|---|
| 0 | version=5 |
| 1 | 적용된 revision, 부팅 시 0 |
| 2 | 마지막 request ID, 잘못된 형식의 요청은 0 |
| 3 | 결과: 0=대기(idle), 1=적용(applied), 2=무효(invalid), 3=안전 조건 미충족(unsafe), 4=revision 충돌, 5=처리 중(busy) |
| 4 | 설정 출처: 0=DEFAULT, 1=HOST_OVERRIDE |
| 5–41 | 적용 중인 설정, 전송 순서 기준 요청 인덱스 3–39 |
| 42–47 | 속도 오차, P, I/적분기 상태, feedforward 전류, 포화 전 전류, 최종 PI 전류 |
| 48–51 | 포화, anti-windup, coasting, AUTO Torque 피드백 사용 가능 여부(Part I <500 ms 및 Part II <1 s; 0/1) |
| 52 | 차단 상태: 0=OK, 1=비활성, 2=피드백 소실, 3=새 명령 대기, 4=역회전, 5=정지, 6=역회전 판정 대기(0 A로 RUN) |
| 53 | 구동 경로: 0=HALTED, 1=MANUAL, 2=AUTO Speed, 3=AUTO Torque, 4=외부 전류, 5=회생, 6=0 A로 RUN |
| 54 | 주행 중 최소 전류 적용 후 최종 지령 상전류, 송신 부호 기준 A |
| 55–57 | 주행 유지 활성, 최소 전류가 출력 결정(요청 ≤ 최소 전류), start step 적용 여부(0/1) |
| 58 | 역회전 속도가 연속된 frame 수, debounce 진행 상태 |
| 59 | 부팅 이후 붕괴 사건 수. 정상 Part I frame 20개가 연속된 뒤에야 새 사건으로 집계 |
| 60 | 최근 사건의 붕괴 플래그: 1=RPM, 2=상전류 상한, 4=상전류 하한, 8=bus 전류 |
| 61 | 최근 사건 이후 경과 시간(s), 사건이 없으면 −1 |
| 62–66 | 최근 사건의 첫 frame: 지령, 상전류, bus 전류, 원본 RPM, 직전 원본 RPM |
| 67–69 | 최근 사건의 구동 경로, 당시 주행 유지 상태, 남은 AUTO 대응 frame 수 |
| 70–71 | 회생 요청(A ≥ 0, LT/외부 요청과 속도 PI 중 큰 값), 회생 한도(A ≥ 0) |
| 72–74 | 회생 속도 계수, 전압 계수, 충전전류 배율(1이면 bus 보호에 따른 감축 없음) |
| 75–77 | 송신 회생 전류(A ≤ 0), 회생 경로 활성(0/1), 속도 PI 회생 모드(0/1) |

회생은 AUTO Torque와 외부 전류 모드에서만 동작합니다. Torque에서는
LT의 `/vehicle/drive_current_command` ≤ 0, 외부 전류 모드에서는 같은 토픽의 ≤ 0 값이
회생 요청입니다. 외부 전류 모드의 양수 값은 회생 요청을 초기화하고 해당 값으로 구동 입력을 갱신합니다.
`pi_regen=1`이면 속도 PI도 회생을 요청할 수 있습니다. STM_A는 두 요청 중 큰 값을 선택하고
50 ms마다 다음을 계산합니다.

```
limit = regen_max × speed_factor(v) × voltage_factor(V) × bus_scale
speed_factor: 0 at v ≤ cutoff, linear to 1 at v ≥ fade
voltage_factor: 1 at V ≤ soft, linear to 0 at V ≥ hard
bus_scale: × 0.8 per frame while bus current < −regen_bus_current_limit, back at regen_slew
target = allowed ? −min(request, limit) : 0;  current = slew(target, regen_slew), then clamp to −limit
```

한도는 같은 frame에서 즉시 적용합니다. cutoff, 전압·충전 보호, Part I 만료 조건에서는
해당 한도에 맞추며, Part I이 만료되면 0으로 제한합니다. `allowed`는 AUTO 전류 경로,
차단 조건 없음, Part I/II 유효, fault 0, EZ Torque 모드를 모두 요구합니다.
이 조건이 해제되면 전류는 `regen_slew`에 따라 0으로 돌아갑니다. 목표 0, 브레이크, E-stop,
STM_B safety와 Host timeout은 기존 HALTED 경로를 유지하며 회생을 즉시 초기화합니다.
MANUAL과 AUTO Speed에서는 회생하지 않습니다.

회생이 활성인 동안에는 0 A로 돌아오는 frame까지 구동 PI 대신 회생 전류를 송신합니다.
PI 적분기와 모드 상태를 유지하고, 구동 출력은 0 A에서 재인가 구간을 거쳐 다시 시작해
한 frame 안에서 상전류 부호가 바뀌지 않게 합니다. 속도 PI 회생은 구동 출력이 0 A에 도달한 뒤
속도 오차가 −`regen_enter_error`보다 작을 때 진입합니다.
`regen_kp × (overspeed − regen_exit_error)`를 요청하고, 오차가 −`regen_exit_error`보다 커지면 해제합니다.

붕괴 판정은 RUN 중 Part I frame마다 수행하며 상수는 `longitudinal_controller.h`에 있습니다.
C는 최근·직전 송신 지령의 절댓값 중 큰 값입니다. 원본 RPM이 ≥400 rpm에서 ≤0으로 떨어지거나,
부호가 바뀌거나, 한 frame에서 max(200, 3·C) rpm보다 크게 변하면 RPM 붕괴로 판정합니다.
Torque 명령에서는 |상전류| > C+60 A, |bus 전류| > C+60 A, 또는 최근·직전 명령이 모두
음수(회생)가 아닌데 상전류가 `collapse_phase_low`보다 낮은 경우를 판정합니다.
이 기준은 2026-09-29/30 방형파 MCAP(붕괴 49건 중 48건 검출, 오탐 없음)과
2026-10-01 사인파 시험(FOC 0 A 잡음 −51 A까지, 350 A 출발 시 frame당 최대 600 rpm)을
근거로 정했습니다. [사인파 재구성 보고서](reports/2026-10-01_ezkontrol_sine_rework.md)와
[회생 재구성 보고서](reports/2026-10-02_regen_governor_rework.md)를 참고합니다.

실제 CAN 송신 전류·RPM과 실측 상전류·bus 전류는 `/drive/status`에서 확인합니다.
Speed 모드의 `commanded_phase_current_a`는 전류 한도이며 토크 지령이 아닙니다.
PI 상태의 최종 전류는 CAN 양자화 전 제어기 출력이며 실측 전류가 아닙니다.
MANUAL은 페달을 토크 요구량으로 사용하고 PI를 초기화합니다. 안전 차단은 slew를 우회해 0 A/HALTED를 명령합니다.

주행 중 최소 전류(rolling floor)는 구동 경로 분기 뒤의 공통 출력 단계에서 적용합니다.
주행 유지(rolling hold)가 켜져 있으면 모든 전진 구동 명령(MANUAL 전진, AUTO Torque,
외부 전류, AUTO 붕괴 대응)을 `min_drive_current` 이상으로 올립니다. 전진 기어·목표,
유효한 피드백, HALTED·회생·역회전 경로가 아닌 상태에서 속도 ≥ `hold_on_speed`이면
유지를 켜고, 속도 ≤ `hold_off_speed`이거나 조건 하나라도 해제되면 끕니다.
AUTO Torque는 이 하한을 PI에 전달해 slew 상태의 시작값으로 사용하고 하한에서는 적분기를 유지합니다.
유지 상태 밖에서 출력이 감소할 때는 (0, min_drive_current) 구간을 천천히 통과하지 않고 PI 값으로 바로 이동합니다.
유지 중 MANUAL은 페달 0…100 %를 최소 전류…전류 한도로 매핑합니다.
한도는 Torque/외부 전류 설정의 `max_drive_current`, Speed 설정의 480 A입니다.
정지·후진에서는 기존 매핑을 유지하며 페달 deadband(2 %)는 바꾸지 않습니다.
`min_drive_current = 0`인 사인파 기본값에서는 이 처리가 동작하지 않습니다.

역회전: 음수 목표는 즉시 차단합니다. 실측 속도가 −`reverse_speed`보다 낮은 상태가
`reverse_debounce_frames`만큼 이어지면 `LONG_REVERSE` → HALTED로 전환합니다.
그전까지는 0 A로 RUN(경로 6)을 유지하고 PI 이력을 보존합니다.
이전의 단일 샘플 −0.05 m/s 기준은 한 frame의 despike 잡음만으로 정지할 수 있었습니다.

붕괴 대응(`collapse_hold_frames > 0`, AUTO 전용): 새 사건 발생 후 지정한 frame 수 동안
AUTO 전류 경로는 최소 전류를 보내고(주행 유지가 꺼져 있으면 0 A) PI 상태를 유지합니다.
MANUAL 출력은 덮어쓰지 않으며, 목표 0·브레이크·E-stop·안전 차단이 우선합니다.
Torque 피드백 복구 후에는 복구 시점 이후 수신한 새 명령이 필요합니다.
Agent 재연결은 RAM 설정을 보존하고 Host 명령·대기 요청을 초기화합니다.
MCU 재부팅은 source DEFAULT/revision 0/Torque 모드로 돌아갑니다.
Host 클라이언트는 이전 override를 자동 재전송하지 않습니다.

launch의 calibration helper는 request ID, revision과 전체 float32 readback을 비교한 뒤
성공을 보고합니다. TUI는 현재 설정 관측과 프리셋 저장만 수행하며 calibration 요청이나
파라미터 쓰기를 보내지 않습니다. 저장값은 해당 프리셋으로 제어 스택을 다음에 실행할 때 적용합니다.
설정 적용·시험 절차, 범위 선정 근거와 녹화 항목은
[구현 보고서](reports/2026-09-21_longitudinal_torque_pi.md)를 참고합니다.

<a id="external-phase-current-command"></a>

## 외부 상전류 명령

`/vehicle/drive_current_command`는 `kaiev26_msgs/msg/DriveCurrentCommand`를 사용합니다.

| 필드 | 의미 |
|---|---|
| `header.stamp` | 발행자의 ROS 시각. 수락되는 샘플은 직전 수락 시각보다 커야 함 |
| `phase_current_a` | 모터 상전류 요청(A). 구동 요청은 0…480 A, 회생 요청은 −480…0 A이며 실제 출력에는 별도 설정 한도 적용 |

`external_current` 모드에서는 이 토픽을 구동·회생 입력으로 사용하고,
AUTO Torque 모드에서는 0 이하 값을 회생 요청으로 받습니다.
선택한 운용 전류·RPM 한도는 프로토콜 최대값보다 낮을 수 있습니다.
`/vehicle/command`는 계속 governor가 발행하며, 양수 전진 목표·브레이크 해제·Host 명령 유효성은
별도로 필요합니다. PC MPC, Host 참조 PI, 전류 식별 시나리오 중 선택한 구성에서 전류 토픽을 발행합니다.

STM_A는 외부 전류 명령에 별도의 500 ms 수신 watchdog을 적용합니다.
외부 구동 전류의 timestamp는 가장 최근에 수락한 목표 명령 시각 이상이고,
그보다 500 ms 미만으로 앞서 있어야 합니다. 전류 발행 시각으로 순서가 뒤바뀌거나 정지 전에
대기열에 쌓인 명령을 거부하고, 수신 후 경과 시간은 HAL tick으로 측정합니다.
이 검사는 MCU가 PC의 UTC 시계와 동기화됐다는 뜻이 아닙니다.
유효하지 않은 전류 샘플은 watchdog을 갱신하지 않습니다. 브레이크, 구동 정지, 모드 변경,
handshake, 재연결, 무효 피드백, 유효한 모터 fault 비트가 0이 아닌 경우에는 저장된 전류를 초기화합니다.
피드백 복구 뒤에는 기존과 같이 새 상위 명령을 요구합니다. 안전 차단은 전류 slew를 우회해 HALTED를 송신합니다.

외부 전류 모드에서 MCU는 속도 PI를 실행하지 않습니다. CAN 전류 단위는 모터 상전류 A이며,
Torque 모드는 bit1=0, 설정한 모터 RPM은 상한입니다. 실제 CAN 송신 전류는
`/drive/status.commanded_phase_current_a`에서 읽습니다. calibration 상태 인덱스 42–52는
PI 진단, 54는 주행 중 최소 전류 적용 후 최종 지령, 70–77은 회생 상태입니다.
새 전류 메시지 타입, STM_A 라이브러리·펌웨어와 Host를 함께 배포해야 합니다.
Host 설정 클라이언트는 기록 해석용으로 v1–v5 상태를 읽지만 요청은 v5만 보냅니다.

실행에 사용하는 MPC 모델과 모든 제어기 설정은 한 Run 동안 고정합니다. Host는 불러온 계수를
읽기 전용 `model_A`, `model_B`, `model_c`, `model_dt` 파라미터로 제공합니다.
기존 recorder는 `/longitudinal_controller`가 실행 중이면 이 값들을 함께 수집합니다.

<a id="control-tuning-parameter-names"></a>

## Control Tuning 파라미터 이름

TUI·프리셋 키는 물리 단위와 `steering_`, `drive_`, `governor_` 접두사를 사용합니다.
[전체 이름 연결·호환 계약](CONTROL_TUNING_PARAMETERS.md)은 preset schema v3와
parameter snapshot schema v2를 정의합니다. 기존 v2 프리셋과 v1 snapshot은 명시적인
이름 변환 어댑터로 읽을 수 있습니다. 별칭이 충돌하면 임의로 값을 고르지 않고 실패 처리합니다.
관리 대상 값은 모두 필수입니다.

노드별 ROS 파라미터와 `/parameter_events`는 기존 이름·단위를 유지합니다.
현재 MCU 배열 ABI는 v5이며 요청은 float 40개, 상태는 78개입니다.
Host decoder는 v1–v5를 읽고 기존 소비자용 `config`를 유지하면서 표준 이름의
`display_values`를 추가합니다. 토픽·메시지 타입은 유지하지만 v5 요청 레이아웃에 맞는
Host와 STM_A 펌웨어가 필요합니다. Control Tuning은 실행 중인 제어값을 읽기만 하며,
`S`는 다음 제어 스택 실행에 사용할 선택 프리셋을 저장합니다.

<a id="auto-drive-envelope-semantics"></a>

### AUTO 구동 한도 적용 방식

ABI v1부터 사용한 `max_drive_current_a`와 `motor_rpm_limit`
(표준 이름 `drive_current_limit_a`, `drive_motor_speed_limit_rpm`)은
AUTO Speed와 Torque에 모두 적용합니다. Speed에서는 EZ 내부 전류 한도와 양·음 대칭의
목표 RPM 한도로, Torque에서는 PI 상전류 상한과 EZ Torque 모드의 RPM 상한으로 사용합니다.
최종 송신부는 전진 4500 rpm, 후진 1500 rpm으로 별도 제한합니다.
MANUAL은 Torque/외부 전류 설정에서 전류·RPM 한도와 전류 slew를 공유하고,
Speed 설정에서는 기존 페달 0–480 A 직접 매핑을 유지합니다. MANUAL은 속도 PI를 실행하지 않습니다.

저장된 driving 프리셋은 Torque/Sine, 480 A/RPM4500, Kp/Ki 180/12, 적분 한도 120 A입니다.
PI 회생(30 A, 진입·해제 0.5/0.3 m/s, gain 15)을 켜고 governor 감속 slew는 0으로 두며,
feedforward는 지상 식별 전까지 끕니다. 이 조합의 지상 검증은 아직 수행하지 않았습니다.
저장된 airborne 프리셋은 Torque/Sine, 80 A/RPM4500, 적분 한도 4 A,
feedforward 0.12/1.05/0.12입니다. PI 회생(30 A, 진입·해제 0.5/0.3 m/s, gain 15)을 켜고
governor 감속 slew는 0으로 둡니다. Torque에서는 가속 slew를 사용하지 않습니다.
[RBS 튜닝 보고서](reports/2026-10-08_rbs_tuning_report.md)와
[현재 프리셋 값](../software/src/control_tuning/README.md#두-프리셋과-schema-v3)을 참고합니다.
이는 저장된 설정이며 현재 MCU에 적용된 readback을 뜻하지 않습니다.

Torque PI는 부호 있는 속도 오차를 사용하고 출력을 양수 구동 전류 또는 0으로 제한합니다.
음수 오차는 전류를 연속적으로 낮춥니다. 출력·slew 하한에 도달하면 학습한 부하 전류를 지우지 않고
적분기를 유지합니다. `coasting`은 과속 중 PI 요구가 주행 중 최소 전류로 제한된 상태를 보고하며,
출력을 강제로 고정하는 latch가 아닙니다. `hysteresis` / `drive_coast_release_error_mps` 필드는
구형 펌웨어·설정 호환을 위해 ABI에 남지만 연속 PI에서는 사용하지 않습니다.
재인가 slew는 0 A 이후 양수 전류 복구를 제한하며, 주행 중 최소 전류가 `reapply_current`
이상이면 느린 구간을 건너뜁니다. v3의 `coast_current_floor`(과속 전용)는 v4부터 위의
주행 중 최소 전류로 대체했습니다. 목표 0과 안전 차단은 전류 0을 명령합니다.

`pi_regen_enabled`와 양수 `regen_max_current_a`를 사용하면 별도 PI 회생 경로가
과속 중 음수 전류를 요청할 수 있습니다. 현재 저장된 두 프리셋은 이 기능을 켜며,
MCU 부팅 기본값에서는 꺼져 있습니다. LT와 외부 음수 전류 요청도 같은 v5 회생 한도를 적용합니다.

PI 게인·적분·deadband와 PI의 slew·재인가는 Torque 전용입니다.
펌웨어 Part I 유효시간은 500 ms이며, Part II 상태는 1 s부터 무효입니다.
AUTO Torque는 둘 중 하나라도 무효이면 PI를 초기화하고 0 A/HALTED로 차단하며,
피드백 복구 이후 수신한 Host 명령을 요구합니다. Speed/MANUAL은 기존처럼 유효성 플래그를 보고합니다.

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
