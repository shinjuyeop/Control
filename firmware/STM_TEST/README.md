# STM_TEST HIL firmware

`STM_TEST`는 NUCLEO-G474RE를 사용해 차량 입력과 구동 장치를 에뮬레이션하는
1차 HIL 펌웨어다. STM_A의 mode/gear/pedal/EZkontrol 입력, STM_B의 E-stop과
brake servo PWM, Host의 KEYA steering CAN 경로를 실제 제어기 없이 시험할 수
있게 한다. micro-ROS는 사용하지 않으며 PC 제어는 LPUART1 ASCII protocol을
사용한다.

SSH/X11에서 STM_A/B Agent와 GUI까지 실행하는 전체 절차는
[`../../docs/HIL_RUNBOOK.md`](../../docs/HIL_RUNBOOK.md)를 따른다.
ROS bridge·topic·service와 GUI 계약은
[`../../software/src/kaiev26_hil/README.md`](../../software/src/kaiev26_hil/README.md)에서 설명한다.

## Pin map

| Pin | Function | Meaning |
|---|---|---|
| PC0 | `HIL_MODE_OUT` | HIGH=MANUAL, LOW=AUTO |
| PC1 | `HIL_GEAR_OUT` | HIGH=Forward, LOW=Reverse |
| PC2 | `HIL_ESTOP_DASH_OUT` | HIGH=Normal, LOW=Active |
| PC3 | `HIL_ESTOP_SIDE_OUT` | HIGH=Normal, LOW=Active |
| PC4 | `HIL_ESTOP_RF_OUT` | HIGH=Normal, LOW=Active |
| PA4 | `DAC1_OUT1` | STM_A accelerator ADC input |
| PA6 | `TIM3_CH1` | STM_B brake PWM input |
| PA2/PA3 | `LPUART1_TX/RX` | PC USB serial, 115200 8N1 |
| PA11/PA12 | `FDCAN1_RX/TX` | STM_A / EZkontrol emulator |
| PB12/PB13 | `FDCAN2_RX/TX` | Host USB-CAN / KEYA emulator |

두 CAN bus는 250 kbit/s Classic CAN을 사용한다. 물리 연결에는 각 bus별 CAN
transceiver와 올바른 120-ohm termination이 필요하다. MCU FDCAN pin을 CANH/CANL에
직접 연결하면 안 된다. 모든 보드는 signal ground를 공유해야 한다.

## Safe defaults

- MANUAL, Forward
- 세 E-stop 모두 Normal
- pedal 0% (`DAC=560`, STM_A의 `PEDAL_ADC_MIN`)
- vehicle speed, motor RPM, steering position 0
- EZkontrol/KEYA CAN enabled
- plant 비활성. speed/RPM은 주입한 값을 그대로 유지한다
- fault 및 TX drop 없음

## UART protocol

명령은 ASCII이며 LF 또는 CRLF로 끝난다. 성공은 `OK <command>`, 실패는
`ERR <reason>`으로 응답한다. `STATUS` 외에는 상태를 자동 출력하지 않는다.
명령어는 대소문자를 구분하지 않는다.

```text
SET MODE MANUAL
SET MODE AUTO
SET GEAR FORWARD
SET GEAR REVERSE
SET ESTOP DASH ON|OFF
SET ESTOP SIDE ON|OFF
SET ESTOP RF ON|OFF
SET PEDAL <0..100>
SET SPEED <-200..200 kph>
SET RPM <-32000..32000>
SET VOLTAGE <0..300 V>
SET CURRENT <-3200..3200 A>
SET MOTOR_TEMP <-40..210 C>
SET STEER_POS <-1800..1800 motor-deg>
SET EZ_CAN ON|OFF
SET KEYA_CAN ON|OFF
SET PLANT ON|OFF
FAULT EZ DROP_TX ON|OFF
FAULT KEYA DROP_TX ON|OFF
FAULT EZ ON|OFF
FAULT KEYA ON|OFF
STATUS
HELP
```

`SET SPEED`는 STM_A와 동일한 tire diameter 0.47 m, gear ratio 4.60을 사용해
EZkontrol motor RPM으로 환산한다. `SET RPM`은 반대로 diagnostic speed도 갱신한다.
`SET STEER_POS`는 KEYA motor shaft degree이며 실제 query/command의 10,000
count/rev와 변환된다.

`SET PLANT ON`은 아래 EZkontrol emulator의 plant를 켠다. plant가 켜져 있으면
speed와 RPM은 STM_A가 보낸 CAN 명령을 따라가므로 `SET SPEED`/`SET RPM`으로 넣은
값은 다음 plant 주기에 덮어써진다. plant를 끄면 그 시점의 motion을 그대로 두므로
운전자가 도달한 값에서 이어서 주입할 수 있다.

STATUS 응답 예시는 다음과 같다.

```text
STATUS protocol=2 mode=AUTO gear=FWD pedal=25.0 estop_dash=0 estop_side=0 estop_rf=0 speed=5.20 rpm=487 voltage=52.3 current=12.5 motor_temp=40.0 brake_period_us=20000 brake_us=1388 brake=RELEASED pwm_lost=0 steer=1.50 steer_target=2.00 ez_rx=10 ez_tx=20 keya_rx=20 keya_tx=21 ez_can=1 keya_can=1 ez_ok=1 keya_ok=1 ez_fault=0 keya_fault=0 plant=1 ez_target_rpm=1200 ez_running=1 ez_speed_control=1 ez_command_rpm=1200 ez_command_current=23.5 ez_rx_age_ms=12 keya_enabled=1 keya_rx_age_ms=15
```

`ez_ok`/`keya_ok`는 FDCAN 초기화 및 queue 상태이지 상대 controller 연결 판정이
아니다. 실제 link는 `*_rx`, `*_rx_age_ms`로 판단한다. 아직 유효 frame을 받지
않았거나 CAN을 껐다가 다시 켠 직후의 age는 `UINT32_MAX`(4294967295)다.
`protocol=2`는 plant 및 command-observation field가 포함된 STATUS 계약이다.

## EZkontrol emulator

FDCAN1은 extended command ID `0x0C01EFD0`만 FIFO0로 수신한다. 다른 standard,
extended, remote frame은 global filter에서 거부한다.

상태 머신:

1. `WAIT_HANDSHAKE`: `0x1801D0EF` / `55` x 8을 50 ms마다 송신한다.
2. STM_A의 `0x0C01EFD0` / `AA` x 8을 받으면 `ACTIVE`가 된다.
3. `ACTIVE`: Part I `0x1801D0EF`와 Part II `0x1802D0EF`를 50 ms마다 송신한다.
4. `FAULT`: 동일 status를 송신하되 Part II의 문서화된 software fault bit 21을
   설정한다.

Part I은 bus voltage, bus/phase current, motor RPM을 STM_A가 사용하는
little-endian offset/scale로 encode한다. 별도의 vehicle-speed CAN field는 없으므로
STM_A가 RPM으로 속도를 계산한다. Part II는 temperature, fault, life counter와
수신한 STM_A command의 RUN/control-mode bit를 반영한다. `DROP_TX`는 수신을
유지하고 handshake/status 송신만 버린다.

`ACTIVE`에서 `HIL_EZ_RX_TIMEOUT_MS`(500 ms) 동안 command frame이 오지 않으면
`WAIT_HANDSHAKE`로 돌아간다. STM_A는 handshake가 끝나기 전에는 CAN을 보내지
않고 emulator는 `WAIT_HANDSHAKE`에서만 `0x55`를 보내므로, 이 복귀가 없으면
STM_A만 재시작했을 때 양쪽이 서로를 영원히 기다린다.

### Plant

`SET PLANT ON`이면 emulator가 수신한 command에서 목표 motor RPM을 뽑아 1차 지연으로
`motor_rpm`을 갱신한다. 시정수는 `HIL_EZ_PLANT_TAU_S`(0.60 s)이고 차량 질량이나
주행 저항은 넣지 않는다. 속도 명령이 STM_A가 되읽는 RPM까지 도달하는지 확인하는
용도이며 실제 가속도를 재현하지 않는다.

- speed control(bit1=1): 목표 RPM은 command의 RPM field 그대로다
- torque control(bit1=0): RPM field는 상한이므로 phase current를
  `HIL_EZ_PLANT_TORQUE_FULL_SCALE_A`(350 A) 대비 비율로 곱한다. STM_A는 MANUAL
  pedal 0~100%를 0~175 A로 보내므로 pedal 100%가 상한의 절반이 된다
- RUN bit가 0이거나 handshake가 안 끝났으면 목표는 0이다

`ez_target_rpm`은 이렇게 계산한 목표값이며 STATUS로 관찰할 수 있다.
원본 command의 `ez_running`, `ez_speed_control`, `ez_command_rpm`,
`ez_command_current`도 함께 노출하므로 target 계산과 실제 RPM 추종을 분리해 볼 수
있다.

## KEYA emulator

기본 node ID는 1이다.

- Host command/query: `0x06000001`
- Emulator response: `0x05800001`
- Emulator heartbeat: `0x07000001`, 1 Hz

Enable, disable, absolute position command와 encoder position, motor speed/current,
fault, voltage, temperature query payload를 KEYA V1.1 manual 형식으로 처리한다.
Position은 signed big-endian 32-bit, 10,000 count/rev다. Enable 상태에서는 실제
position이 10 ms마다 최대 90 motor-deg/s로 target을 따라간다.

`keya_enabled`는 마지막 enable/disable command 결과이고 `keya_rx_age_ms`는 마지막
유효 command/query 이후 경과시간이다. FDCAN2 transceiver가 없는 구성에서도
emulator와 GUI는 동작하며, RX count 0/age UINT32_MAX가 물리 link 부재를 명확히
나타낸다. KEYA 추종 모델은 EZ의 `SET PLANT`와 독립적이다.

1차 state에는 별도의 KEYA supply/current/temperature 입력이 없으므로 해당 query는
0을 반환한다. `FAULT KEYA ON`은 매뉴얼의 CAN-disconnected fault bit 14로
표현한다. Disable 상태는 문서의 status bit 0으로 표현한다. 정의되지 않은 fault
bit는 만들지 않는다.

## Brake PWM monitor

TIM3은 1 us/count PWM Input mode다. CH1 rising/direct capture가 period를, CH2
falling/indirect capture가 pulse width를 제공한다.

- engaged: 1194 us ± 50 us
- released: 1388 us ± 50 us
- 그 외: UNKNOWN
- 100 ms 동안 유효 capture가 없으면 `pwm_lost=1`, UNKNOWN

ISR은 capture register와 tick만 기록하고 분류 및 공유 상태 갱신은 Monitor task가
50 ms마다 수행한다.

## Build and host logic tests

반드시 이 README가 있는 디렉터리에서 실행한다.

```bash
cmake --preset Debug
cmake --build --preset Debug
./tests/run_host_tests.sh
```

결과 ELF는 `build/Debug/STM_TEST.elf`다.

## Intended wiring

- FDCAN1 transceiver ↔ STM_A FDCAN1 bus
- FDCAN2 transceiver ↔ PC USB-CAN adapter
- PC USB serial ↔ NUCLEO LPUART1 pins through a suitable USB-UART path
- PA4 DAC output ↔ STM_A accelerator ADC input, with common ground
- PC0/PC1 ↔ STM_A mode/gear inputs
- PC2/PC3/PC4 ↔ STM_B dashboard/side/RF E-stop inputs
- STM_B brake servo PWM output ↔ PA6, with compatible logic voltage/common ground

실제 연결 전 GPIO output 간 충돌 여부, 3.3 V logic compatibility, DAC load, CAN
termination과 NUCLEO virtual COM routing을 회로도 기준으로 다시 확인해야 한다.
