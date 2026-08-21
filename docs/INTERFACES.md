# Interfaces

> 이 문서는 현재 작업 브랜치의 소스 코드를 기준으로 작성되었습니다.
> 코드 변경 시 주기, timeout, 핀, 통신 ID와 안전 조건을 함께 갱신해야 합니다.

## 1. 문서 목적

이 문서는 현재 코드에서 확인되는 ROS 2 topic, STM_A↔STM_B UART, EZkontrol CAN,
KEYA 조향 CAN의 데이터 계약과 주기·timeout을 정리하는 기준 문서입니다. 안전
조건의 결과는 [`SAFETY.md`](SAFETY.md), 구성요소의 책임은
[`ARCHITECTURE.md`](ARCHITECTURE.md)를 참고하십시오.

## 2. 표기 및 단위 규칙

- ROS 2 공통 단위: 속도 `m/s`, 조향각 `rad`(차량 좌회전 양수)
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
| Planning 또는 시험 명령원 | `command_governor` | ROS 2 | `/planning/command` | `ActuatorCommand` | production 미구현; 실차 시험·HIL은 20 Hz | 거버너 입력 게이트 200 ms | 실차 시험은 `vehicle_test_scenario_runner`, HIL은 `hil_command_generator` |
| `command_governor` | STM_A | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 500 ms | AUTO 속도·brake |
| `command_governor` | STM_B | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 적용 안 함 | brake만 저장 |
| `command_governor` | 조향 노드 | ROS 2 | `/vehicle/command` | `ActuatorCommand` | 입력 연동 (수신 즉시 재발행) | 200 ms | ACTIVE 조향 freshness |
| STM_A | Host·조향·상태 통합 | ROS 2 | `/drive/status` | `DriveActuatorStatus` | 50 ms | 조향에서 1500 ms | mode·gear·구동 진단 |
| STM_B | Host·조향·상태 통합 | ROS 2 | `/brake/status` | `BrakeActuatorStatus` | 50 ms 및 원인 변경 시 | 조향에서 별도 timeout 없음 | brake·E-stop 원인 |
| 조향 노드 | Host·상태 통합 | ROS 2 | `/steering/status` | `SteeringActuatorStatus` | 20 ms | 상태 통합에서 별도 timeout 없음 | 조향 상태·진단 |
| 상태 통합 | ROS 2 consumers | ROS 2 | `/vehicle/state` | `VehicleState` | 50 ms | 없음 | 세 status의 마지막 값 합성 |
| STM_A | STM_B | USART1 | type `0x01` | mode, gear, speed | 20 ms | STM_B에서 200 ms | vehicle status |
| STM_B | STM_A | USART1 | type `0x02` | final brake request | 20 ms | STM_A에서 100 ms | safety status |
| STM_A | EZkontrol | FDCAN1 | `0x0C01EFD0` | handshake ACK 또는 command | 50 ms command | 명시적 TX timeout 없음 | AUTO speed / MANUAL torque |
| EZkontrol | STM_A | FDCAN1 | `0x1801D0EF`, `0x1802D0EF` | Part I/II status | 외부 장치 설정 | 200 ms validity | 진단 validity에만 사용 |
| 조향 노드 | KEYA | SocketCAN | `0x06000001` | enable/disable/position/query | position 20 ms | 500 ms | node ID 기본값 1 |
| KEYA | 조향 노드 | SocketCAN | `0x05800001`, `0x07000001` | query response, heartbeat | 외부 장치 설정 | CAN·position 각 500 ms | ID는 node ID에 따라 변함 |
| Host Agent | 각 STM | LPUART1 serial | micro-ROS XRCE-DDS | ROS 2 serialization | event-driven | Agent ping 500/100 ms | 보드별 Agent 1개 |

## 4. ROS 2 Topics

### 4.1 차량 제어와 상태

| Topic | Publisher | Subscriber | Type | Rate | QoS | 설명 |
|---|---|---|---|---:|---|---|
| `/planning/command` | Planning (production 미구현), 실차 시험 `vehicle_test_scenario_runner`, HIL `hil_command_generator` 중 하나 | `command_governor` | `kaiev26_msgs/msg/ActuatorCommand` | 시험 명령원 20 Hz | default | Planning 의도 명령 |
| `/vehicle/command` | `command_governor` (유일 발행자) | STM_A, STM_B, `steering_actuator_node` | `kaiev26_msgs/msg/ActuatorCommand` | 입력 연동; HIL 20 Hz | default | 속도·조향·최종 brake 명령 |
| `/drive/status` | `/stm_a_node` | 조향 노드, `vehicle_state_publisher` | `kaiev26_msgs/msg/DriveActuatorStatus` | 20 Hz | default | mode·gear·구동 명령 및 EZkontrol 진단 |
| `/brake/status` | `/stm_b_node` | 조향 노드, `vehicle_state_publisher` | `kaiev26_msgs/msg/BrakeActuatorStatus` | 20 Hz + 변경 시 | default | 브레이크 command와 stop reason |
| `/steering/status` | `/steering_actuator_node` | `vehicle_state_publisher`, 진단 도구 | `kaiev26_msgs/msg/SteeringActuatorStatus` | 50 Hz | Keep Last 10, reliable, volatile | 조향 상태 머신·timeout·모터 진단 |
| `/vehicle/state` | `/vehicle_state_publisher` | 현재 코드에서 확인되지 않음 | `kaiev26_msgs/msg/VehicleState` | 20 Hz | Keep Last 10, reliable, volatile | 세 status가 모두 수신된 후 합성 |

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
```

`/vehicle/command`의 발행자는 `command_governor` 하나입니다. 거버너는
`/planning/command`를 수신 즉시 정합성 게이트(stamp 0·음수, 나이 ±200 ms,
finite)를 거쳐 재-stamp 후 재발행하며, 자체 타이머와 keep-alive가 없으므로 발행률은
입력을 그대로 따릅니다. 따라서 상류(실차 시험은
`vehicle_test_scenario_runner`, HIL은 `hil_command_generator`, 모두 20 Hz)가
STM_A의 500 ms와 조향의 200 ms freshness 한계보다 충분히 빠르게 발행해야
합니다. 5 Hz는 조향 200 ms의 경계라 간헐 timeout이 납니다.
`/planning/command`의 production Planning publisher는 아직 구현되지 않았습니다.
실차 시험·HIL·production publisher 중 하나만 실행합니다.

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
| 2 | message type | `uint8` | `0x01` 또는 `0x02` |
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

STM_B timeout은 `VEHICLE_FRAME_VALID_FROM_A`를 false로 만들어 LCD에
`COMM FAULT`를 표시하지만, 현재 `BRAKE_REQUEST` 계산에는 직접 사용되지 않습니다.

### 5.3 Safety Status Frame

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
| 0..1 | phase-current target | `(A + 3200) / 0.1`, little-endian | AUTO 한계 300 A, MANUAL 한계 175 A |
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

Part I motor RPM은 타이어 지름 `0.47 m`, 감속비 `4.60`을 사용해
`CURRENT_SPEED`로 변환됩니다.

### 6.5 Status Part II

| Byte | Field | Encoding |
|---:|---|---|
| 0 | controller temperature | `raw - 40 °C` |
| 1 | motor temperature | `raw - 40 °C` |
| 2 bit 0 | RUN state | `1=running` |
| 2 bit 1 | control mode | `0=torque`, `1=speed` |
| 3..5 | fault bits | 24-bit little-endian 원본 |
| 6..7 | 현재 코드에서 사용하지 않음 | - |

Part I과 Part II는 마지막 수신 후 200 ms 미만일 때 각각 valid입니다. 이 timeout은
현재 `/drive/status` validity에만 반영되고 traction을 직접 HALTED로 바꾸지는
않습니다.

구현 위치:

- `firmware/STM_A/Core/Src/fdcan.c`
- `firmware/STM_A/Core/Src/app_freertos.c`

## 7. Steering Motor CAN

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
| Encoder position query | `40 04 21 02 00 00 00 00` |
| Motor speed query | `40 03 21 01 00 00 00 00` |
| Motor current query | `40 00 21 01 00 00 00 00` |
| Fault query | `40 12 21 01 00 00 00 00` |

control tick 20 ms마다 position을 조회하며 ACTIVE에서는 rate-limit된 position
command도 매 tick 전송합니다. fault는 100 ms, speed·current는 200 ms,
bus voltage·driver temperature는 500 ms 간격으로 번갈아 조회합니다.

- CAN timeout: heartbeat 또는 feedback frame이 500 ms 동안 없을 때
- position feedback timeout: 유효 position response가 500 ms 동안 없을 때
- KEYA 내부 watchdog 설정값: 현재 actuator 코드에서 설정하거나 확인하지 않음
- 제어 mode: 현재 통합 경로는 position command만 사용
- motor-axis diagnostic scaling: `10000 count/rev`
- 조향 calibration은 raw KEYA count를 중앙 등가각으로 직접 환산하며
  `software/src/lateral_ctrl/config/steering_motor_calibration.yaml`이 소유
- 명령 범위: `-3321..3321 count`(약 `-28.38..28.38 deg`), feedback은 클램프하지 않음

구현 위치:

- `software/src/lateral_ctrl/src/keya_can_driver.cpp`
- `software/src/lateral_ctrl/src/steering_actuator_node.cpp`
- `software/src/lateral_ctrl/config/steering_actuator.yaml`

## 8. Timeout and Freshness

| 항목 | 값 | 근거 상수/설정 | 구현 위치 |
|---|---:|---|---|
| STM_A Control command | 50 ms | `osDelay(50)` | STM_A `app_freertos.c` |
| STM_A Host command timeout | 500 ms | `HOST_COMMAND_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM_B safety TX | 20 ms | `SAFETY_TX_PERIOD_MS` | STM_B `app_freertos.c` |
| STM_A vehicle TX | 20 ms | `VEHICLE_TX_PERIOD_MS` | STM_A `app_freertos.c` |
| STM_B→STM_A UART timeout | 100 ms | `STM_B_COMM_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM_A→STM_B UART timeout | 200 ms | `VEHICLE_COMM_TIMEOUT_MS` | STM_B `app_freertos.c` |
| Drive status | 50 ms | `DRIVE_STATUS_PERIOD_MS` | STM_A `app_freertos.c` |
| Brake status | 50 ms + 변경 시 | `BRAKE_STATUS_PERIOD_MS` | STM_B `app_freertos.c` |
| EZkontrol status validity | 200 ms | `MOTOR_FEEDBACK_TIMEOUT_MS` | STM_A `app_freertos.c` |
| STM Agent ping | 500 ms, call timeout 100 ms | `AGENT_PING_*` | 두 STM `app_freertos.c` |
| 조향 control/status | 20 ms | `control_period_ms` | `steering_actuator.yaml` |
| 조향 command timeout | 200 ms | `steer_command_timeout_ms` | `steering_actuator.yaml` |
| 조향 CAN timeout | 500 ms | `can_timeout_ms` | `steering_actuator.yaml` |
| 조향 position timeout | 500 ms | `feedback_timeout_ms` | `steering_actuator.yaml` |
| 조향 mode timeout | 1500 ms | `mode_timeout_ms` | `steering_actuator.yaml` |
| 거버너 입력 나이 게이트 | 200 ms | `input_timeout_ms` | `command_governor.yaml` |
| `/vehicle/state` publish | 50 ms | `publish_rate_hz=20.0` | vehicle state publisher |

경계 비교는 구현마다 다릅니다. STM Host와 UART timeout은 `>=`, EZkontrol valid는
`age < 200 ms`, 조향 timeout은 `age > 설정값`을 사용합니다.

## 9. 관련 구현 파일

- STM_A: `firmware/STM_A/Core/Src/app_freertos.c`
- STM_A CAN: `firmware/STM_A/Core/Src/fdcan.c`
- STM_A serial: `firmware/STM_A/Core/Src/usart.c`
- STM_B: `firmware/STM_B/Core/Src/app_freertos.c`
- STM_B PWM: `firmware/STM_B/Core/Src/tim.c`
- 조향: `software/src/lateral_ctrl/src/`
- 명령 거버너: `software/src/command_governor/src/`
- 상태 통합: `software/src/vehicle_state_publisher/`
- 메시지 원본: 별도 workspace repository `kaiev26_msgs/msg/`

## 10. 확인 필요 항목

- Production `/planning/command` publisher (Planning)와 발행률.
  `/vehicle/command` 발행자는 `command_governor`로 확정됨
- STM_A→STM_B UART timeout을 브레이크 판단에 포함할지 여부
- EZkontrol Part I/II 손실을 traction stop 조건으로 포함할지 여부
- EZkontrol status 실제 송신 주기
- KEYA 내부 watchdog 시간과 현재 position command의 keep-alive 보장
- STM_A·STM_B PCB connector 기준 실제 배선 및 signal ground
- `BrakeActuatorStatus`와 `/vehicle/state` input freshness 정책
