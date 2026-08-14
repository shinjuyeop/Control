# STM_TEST Firmware

> 이 문서는 현재 작업 브랜치의 소스 코드를 기준으로 작성되었습니다.
> 코드 변경 시 핀, 주기, timeout, 통신 ID와 안전 조건을 함께 갱신해야 합니다.

## 1. 역할

STM_TEST는 STM32G474 기반의 bench 전용 HIL 에뮬레이터 보드입니다. 차량 입력과
구동 장치를 흉내 내어 STM_A의 mode/gear/pedal/EZkontrol 입력, STM_B의 E-stop과
brake servo PWM, Host의 KEYA steering CAN 경로를 실제 제어기 없이 시험할 수 있게
합니다. micro-ROS는 사용하지 않으며 PC 제어는 LPUART1 ASCII protocol을 씁니다.

## 2. 담당 기능

- STM_A의 mode·gear 스위치와 가속 페달을 GPIO·DAC로 대체
- STM_B의 dashboard·side·RF E-stop 스위치를 GPIO로 대체
- STM_B가 내보내는 brake servo PWM을 입력 캡처로 관측
- EZkontrol 모터 드라이버 CAN emulator와 1차 지연 plant
- KEYA 조향 모터 CAN emulator와 위치 추종 모델
- LPUART1 ASCII 명령으로 위 상태를 주입하고 `STATUS`로 관측
- CAN fault와 TX drop 주입

## 3. 보드와 MCU

| 항목 | 값 |
|---|---|
| 보드 | NUCLEO-G474RE |
| MCU | STM32G474RETx, Cortex-M4F |
| 시스템 clock | HSI 16 MHz → PLL(M=4, N=85, R=2) → 170 MHz, FLASH latency 4 |
| FLASH | origin `0x08000000`, 512 KB |
| RAM | origin `0x20000000`, 128 KB |
| 고정 장치 이름 | HIL bench `/dev/hil_test` |
| CubeMX 프로젝트 | `STM_TEST.ioc` |

STM_A·STM_B와 달리 실차에 올라가지 않는 bench 전용 펌웨어입니다.

## 4. 핀맵

핀 이름 정의는 `Core/Inc/main.h`, 초기화는 `Core/Src/main.c`의 `MX_GPIO_Init()`과
`Core/Src/stm32g4xx_hal_msp.c`에 있습니다. 아래 표는 두 곳을 대조해 정리한
것입니다.

보드 커넥터 번호는 NUCLEO-G474RE(MB1367)의 ST morpho 헤더 기준입니다. `CN7`과
`CN10`은 각각 2×19 헤더이고 홀수 번호와 짝수 번호가 각각 한 열을 이룹니다.
괄호 안의 `A2`·`D12`는 같은 신호가 나오는 Arduino Uno 헤더 실크스크린 이름이며,
표기가 없는 핀은 morpho 헤더에서만 나옵니다. 물리적 배치는 보드 매뉴얼 UM2505를
확인하십시오. GND는 `CN7-8`, `CN7-19`, `CN7-20`, `CN7-22`, `CN10-9`, `CN10-20`에
있습니다.

### 4.1 신호 핀

| MCU 핀 | 보드 커넥터 | 신호 이름 | 방향 | 전기적 설정 | 의미 |
|---|---|---|---|---|---|
| `PC0` | `CN7-38` (A5) | `HIL_MODE_OUT` | 출력 | push-pull, no pull | HIGH=MANUAL, LOW=AUTO |
| `PC1` | `CN7-36` (A4) | `HIL_GEAR_OUT` | 출력 | push-pull, no pull | HIGH=FORWARD, LOW=REVERSE |
| `PC2` | `CN7-35` | `HIL_ESTOP_DASH_OUT` | 출력 | push-pull, no pull | HIGH=Normal, LOW=Active |
| `PC3` | `CN7-37` | `HIL_ESTOP_SIDE_OUT` | 출력 | push-pull, no pull | HIGH=Normal, LOW=Active |
| `PC4` | `CN10-34` | `HIL_ESTOP_RF_OUT` | 출력 | push-pull, no pull | HIGH=Normal, LOW=Active |
| `PA4` | `CN7-32` (A2) | `DAC1_OUT1` (가속 페달) | 출력 (analog) | 12-bit right-aligned | `560`=0%, `3100`=100%, 부팅값 `560` |
| `PA6` | `CN10-13` (D12) | `TIM3_CH1` (brake PWM 관측) | 입력 (AF) | PWM Input mode, 1 µs/count | CH1 rising=period, CH2 falling=pulse |

다섯 GPIO 출력은 모두 상대 보드의 pull-up active-low 입력을 대신하므로 LOW가
"동작(AUTO·REVERSE·E-stop Active)"입니다. `VehicleIO` Task가 10 ms마다 다섯 GPIO와
DAC를 함께 갱신합니다.

### 4.2 통신·주변장치 핀

| MCU 핀 | 보드 커넥터 | Peripheral | 용도 | 설정 | 관련 파일 |
|---|---|---|---|---|---|
| `PA11` | `CN10-14` | `FDCAN1_RX` | STM_A / EZkontrol emulator | Classic CAN, extended ID, 250 kbit/s | `Core/Src/main.c` |
| `PA12` | `CN10-12` | `FDCAN1_TX` | 동일 bus | 동일 | `Core/Src/main.c` |
| `PB12` | `CN10-16` | `FDCAN2_RX` | Host USB-CAN / KEYA emulator | Classic CAN, extended ID, 250 kbit/s | `Core/Src/main.c` |
| `PB13` | `CN10-30` | `FDCAN2_TX` | 동일 bus | 동일 | `Core/Src/main.c` |
| `PA2` | `CN10-35` | `LPUART1_TX` | PC 제어 ASCII protocol | 115200 8N1, DMA | `Core/Src/main.c` |
| `PA3` | `CN10-37` | `LPUART1_RX` | 동일 | Receive-to-Idle DMA | `Core/Src/main.c` |

두 CAN bus는 각각 별도의 CAN transceiver와 올바른 120 Ω termination이 필요합니다.
MCU FDCAN 핀을 CANH/CANL에 직접 연결하면 안 됩니다. `PA2`/`PA3`은 NUCLEO의
ST-LINK virtual COM port와 연결된 핀이므로 morpho 자리에 다른 신호를 물리면 PC
제어 protocol과 충돌합니다.

### 4.3 디버그·예약 핀

| MCU 핀 | 보드 커넥터 | 신호 | 비고 |
|---|---|---|---|
| `PA13` | `CN7-13` | `SWDIO` | ST-LINK SWD |
| `PA14` | `CN7-15` | `SWCLK` | ST-LINK SWD |
| `PB3` | `CN10-31` (D3) | `SWO` | trace 출력 |
| `PF0`/`PF1` | `CN7-29`/`CN7-31` | `RCC_OSC_IN`/`RCC_OSC_OUT` | `.ioc`에만 잡혀 있고 실제 clock은 HSI PLL |
| `PC14`/`PC15` | `CN7-25`/`CN7-27` | `RCC_OSC32_IN`/`RCC_OSC32_OUT` | application 미사용 |

### 4.4 배선 상대

| STM_TEST 핀 | 연결 대상 | 대상 핀 |
|---|---|---|
| `PC0` `CN7-38` | STM_A 모드 입력 | `PA8` `CN10-23` (`SW_MODE`) |
| `PC1` `CN7-36` | STM_A 기어 입력 | `PA9` `CN10-21` (`SW_GEAR`) |
| `PC2` `CN7-35` | STM_B Dashboard E-stop 입력 | `PC0` `CN7-38` (`ESTOP_DASH`) |
| `PC3` `CN7-37` | STM_B Side E-stop 입력 | `PC1` `CN7-36` (`ESTOP_SIDE`) |
| `PC4` `CN10-34` | STM_B RF E-stop 입력 | `PB4` `CN10-27` (`ESTOP_RF`) |
| `PA4` `CN7-32` | STM_A 가속 페달 ADC 입력 | `PA0` `CN7-28` (`ADC1_IN1`) |
| `PA6` `CN10-13` | STM_B brake servo PWM 출력 | `PB3` `CN10-31` (`TIM2_CH2`) |
| `PA11`/`PA12` `CN10-14`/`CN10-12` | CAN transceiver → STM_A `FDCAN1` | `PA11`/`PA12` `CN10-14`/`CN10-12` |
| `PB12`/`PB13` `CN10-16`/`CN10-30` | CAN transceiver → Host USB-CAN adapter | 어댑터 CANH/CANL |
| `PA2`/`PA3` `CN10-35`/`CN10-37` | ST-LINK virtual COM port → PC (`/dev/hil_test`) | — |

실제 연결 전에 GPIO output 간 충돌 여부, 3.3 V logic compatibility, DAC load, CAN
termination과 NUCLEO virtual COM routing을 회로도 기준으로 다시 확인해야 합니다.
모든 보드는 signal ground를 공유해야 합니다.

## 5. 소프트웨어 구조

| 파일 | 역할 |
|---|---|
| `Core/Src/main.c` | clock·GPIO·DMA·FDCAN1/2·DAC1·LPUART1·TIM3 초기화와 Task 생성 |
| `Core/Src/app_freertos.c` | UART RxEvent, FDCAN RxFifo0, TIM input capture HAL callback |
| `Core/Src/hil_state.c` | mutex로 보호되는 공유 상태와 snapshot |
| `Core/Src/hil_logic.c` | 단위 변환, pedal·DAC, RPM·speed, brake 분류 등 순수 로직 |
| `Core/Src/hil_command.c` | LPUART1 ASCII protocol parse와 응답 |
| `Core/Src/hil_vehicle_io.c` | mode·gear·E-stop GPIO와 pedal DAC 출력 |
| `Core/Src/hil_ezkontrol.c` | EZkontrol CAN emulator와 plant |
| `Core/Src/hil_keya.c` | KEYA CAN emulator와 위치 추종 |
| `Core/Src/hil_monitor.c` | brake PWM input capture 분류 |
| `tests/` | `hil_logic.c`를 host에서 컴파일해 돌리는 logic test |

STM_A·STM_B와 달리 공유 상태를 critical section이 아니라 CMSIS-RTOS mutex
(`HilStateMutex`)로 보호하고, Task는 `HilState_GetSnapshot()`으로 값을 복사해
씁니다. micro-ROS는 사용하지 않습니다.

`hil_logic.c`는 HAL에 의존하지 않는 순수 함수만 모아 두었으므로 host에서 그대로
컴파일해 시험할 수 있습니다.

## 6. FreeRTOS Task

| Task | 역할 | 주기/Trigger | 우선순위 | 관련 파일 |
|---|---|---:|---|---|
| `HostCommand` | LPUART1 ASCII 명령 parse와 응답 | 2 ms loop | `Normal` | `hil_command.c` |
| `VehicleIO` | mode·gear·E-stop GPIO와 pedal DAC 출력 | 10 ms | `Normal` | `hil_vehicle_io.c` |
| `EZKontrol` | handshake·status 송신, 명령 수신, plant 갱신 | 10 ms loop, 50 ms 송신 | `AboveNormal` | `hil_ezkontrol.c` |
| `KEYA` | command/query 응답, 1 Hz heartbeat, 위치 추종 | 10 ms | `AboveNormal` | `hil_keya.c` |
| `Monitor` | brake PWM capture 분류와 timeout | 50 ms | `BelowNormal` | `hil_monitor.c` |

주기 Task는 `osDelayUntil()`로 고정 주기를 유지합니다. ISR은 capture register와
tick만 기록하고 판단은 Task에서 수행합니다.

## 7. 외부 인터페이스

micro-ROS Agent를 붙이지 않고 LPUART1(`PA2`/`PA3`, 115200 8N1)에서 ASCII 명령을
직접 받습니다. 명령은 LF 또는 CRLF로 끝납니다. 성공은 `OK <command>`, 실패는
`ERR <reason>`으로 응답합니다. `STATUS` 외에는 상태를 자동 출력하지 않습니다.
명령어는 대소문자를 구분하지 않습니다.

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

`SET MODE`, `SET GEAR`, `SET ESTOP`은 4.1의 GPIO 출력을, `SET PEDAL`은 `PA4` DAC
값을 바꿉니다.

`SET SPEED`는 STM_A와 동일한 tire diameter 0.47 m, gear ratio 4.60을 사용해
EZkontrol motor RPM으로 환산합니다. `SET RPM`은 반대로 diagnostic speed도
갱신합니다. `SET STEER_POS`는 KEYA motor shaft degree이며 실제 query/command의
10,000 count/rev와 변환됩니다.

`SET PLANT ON`은 아래 EZkontrol emulator의 plant를 켭니다. plant가 켜져 있으면
speed와 RPM은 STM_A가 보낸 CAN 명령을 따라가므로 `SET SPEED`/`SET RPM`으로 넣은
값은 다음 plant 주기에 덮어써집니다. plant를 끄면 그 시점의 motion을 그대로 두므로
운전자가 도달한 값에서 이어서 주입할 수 있습니다.

STATUS 응답 예시는 다음과 같습니다.

```text
STATUS protocol=2 mode=AUTO gear=FWD pedal=25.0 estop_dash=0 estop_side=0 estop_rf=0 speed=5.20 rpm=487 voltage=52.3 current=12.5 motor_temp=40.0 brake_period_us=20000 brake_us=1388 brake=RELEASED pwm_lost=0 steer=1.50 steer_target=2.00 ez_rx=10 ez_tx=20 keya_rx=20 keya_tx=21 ez_can=1 keya_can=1 ez_ok=1 keya_ok=1 ez_fault=0 keya_fault=0 plant=1 ez_target_rpm=1200 ez_running=1 ez_speed_control=1 ez_command_rpm=1200 ez_command_current=23.5 ez_rx_age_ms=12 keya_enabled=1 keya_rx_age_ms=15
```

`ez_ok`/`keya_ok`는 FDCAN 초기화 및 queue 상태이지 상대 controller 연결 판정이
아닙니다. 실제 link는 `*_rx`, `*_rx_age_ms`로 판단합니다. 아직 유효 frame을 받지
않았거나 CAN을 껐다가 다시 켠 직후의 age는 `UINT32_MAX`(4294967295)입니다.
`protocol=2`는 plant 및 command-observation field가 포함된 STATUS 계약입니다.

## 8. 동작 상세

### 8.1 부팅 기본값

- MANUAL, Forward
- 세 E-stop 모두 Normal
- pedal 0% (`DAC=560`, STM_A의 `PEDAL_ADC_MIN`)
- vehicle speed, motor RPM, steering position 0
- EZkontrol/KEYA CAN enabled
- plant 비활성. speed/RPM은 주입한 값을 그대로 유지
- fault 및 TX drop 없음

### 8.2 EZkontrol emulator (`FDCAN1`, `PA11`/`PA12`)

FDCAN1은 extended command ID `0x0C01EFD0`만 FIFO0로 수신합니다. 다른 standard,
extended, remote frame은 global filter에서 거부합니다.

상태 머신:

1. `WAIT_HANDSHAKE`: `0x1801D0EF` / `55` x 8을 50 ms마다 송신합니다.
2. STM_A의 `0x0C01EFD0` / `AA` x 8을 받으면 `ACTIVE`가 됩니다.
3. `ACTIVE`: Part I `0x1801D0EF`와 Part II `0x1802D0EF`를 50 ms마다 송신합니다.
4. `FAULT`: 동일 status를 송신하되 Part II의 문서화된 software fault bit 21을
   설정합니다.

Part I은 bus voltage, bus/phase current, motor RPM을 STM_A가 사용하는
little-endian offset/scale로 encode합니다. 별도의 vehicle-speed CAN field는 없으므로
STM_A가 RPM으로 속도를 계산합니다. Part II는 temperature, fault, life counter와
수신한 STM_A command의 RUN/control-mode bit를 반영합니다. `DROP_TX`는 수신을
유지하고 handshake/status 송신만 버립니다.

`ACTIVE`에서 `HIL_EZ_RX_TIMEOUT_MS`(500 ms) 동안 command frame이 오지 않으면
`WAIT_HANDSHAKE`로 돌아갑니다. STM_A는 handshake가 끝나기 전에는 CAN을 보내지
않고 emulator는 `WAIT_HANDSHAKE`에서만 `0x55`를 보내므로, 이 복귀가 없으면
STM_A만 재시작했을 때 양쪽이 서로를 영원히 기다립니다.

`SET PLANT ON`이면 emulator가 수신한 command에서 목표 motor RPM을 뽑아 1차 지연으로
`motor_rpm`을 갱신합니다. 시정수는 `HIL_EZ_PLANT_TAU_S`(0.60 s)이고 차량 질량이나
주행 저항은 넣지 않습니다. 속도 명령이 STM_A가 되읽는 RPM까지 도달하는지 확인하는
용도이며 실제 가속도를 재현하지 않습니다.

- speed control(bit1=1): 목표 RPM은 command의 RPM field 그대로입니다
- torque control(bit1=0): RPM field는 상한이므로 phase current를
  `HIL_EZ_PLANT_TORQUE_FULL_SCALE_A`(350 A) 대비 비율로 곱합니다. STM_A는 MANUAL
  pedal 0~100%를 0~175 A로 보내므로 pedal 100%가 상한의 절반이 됩니다
- RUN bit가 0이거나 handshake가 안 끝났으면 목표는 0입니다

`ez_target_rpm`은 이렇게 계산한 목표값이며 STATUS로 관찰할 수 있습니다.
원본 command의 `ez_running`, `ez_speed_control`, `ez_command_rpm`,
`ez_command_current`도 함께 노출하므로 target 계산과 실제 RPM 추종을 분리해 볼 수
있습니다.

### 8.3 KEYA emulator (`FDCAN2`, `PB12`/`PB13`)

기본 node ID는 1입니다.

- Host command/query: `0x06000001`
- Emulator response: `0x05800001`
- Emulator heartbeat: `0x07000001`, 1 Hz

Enable, disable, absolute position command와 encoder position, motor speed/current,
fault, voltage, temperature query payload를 KEYA V1.1 manual 형식으로 처리합니다.
Position은 signed big-endian 32-bit, 10,000 count/rev입니다. Enable 상태에서는 실제
position이 10 ms마다 최대 90 motor-deg/s로 target을 따라갑니다.

`keya_enabled`는 마지막 enable/disable command 결과이고 `keya_rx_age_ms`는 마지막
유효 command/query 이후 경과시간입니다. FDCAN2 transceiver가 없는 구성에서도
emulator와 GUI는 동작하며, RX count 0/age UINT32_MAX가 물리 link 부재를 명확히
나타냅니다. KEYA 추종 모델은 EZ의 `SET PLANT`와 독립적입니다.

1차 state에는 별도의 KEYA supply/current/temperature 입력이 없으므로 해당 query는
0을 반환합니다. `FAULT KEYA ON`은 매뉴얼의 CAN-disconnected fault bit 14로
표현합니다. Disable 상태는 문서의 status bit 0으로 표현합니다. 정의되지 않은 fault
bit는 만들지 않습니다.

### 8.4 Brake PWM monitor (`PA6`)

TIM3은 1 µs/count PWM Input mode입니다. CH1 rising/direct capture가 period를, CH2
falling/indirect capture가 pulse width를 제공합니다. 두 채널 모두 `PA6` 하나의
입력을 공유합니다.

- engaged: 1194 µs ± 50 µs
- released: 1388 µs ± 50 µs
- 그 외: UNKNOWN
- 100 ms 동안 유효 capture가 없으면 `pwm_lost=1`, UNKNOWN

ISR은 capture register와 tick만 기록하고 분류 및 공유 상태 갱신은 `Monitor` Task가
50 ms마다 수행합니다.

## 9. 빌드

권장 워크스페이스인 `~/KAI_ws` 루트에서 STM_TEST만 Release로 빌드합니다.
`arm-none-eabi-gcc`가 `PATH`에 있어야 합니다.

```bash
cd ~/KAI_ws
cmake -S src/Control/firmware/STM_TEST --preset Release
cmake --build src/Control/firmware/STM_TEST/build/Release
```

결과물은 `src/Control/firmware/STM_TEST/build/Release/STM_TEST.elf`입니다.

`hil_logic.c`의 host logic test는 별도로 실행합니다. 스크립트가 자기 경로를
기준으로 동작하므로 실행 위치는 상관없습니다.

```bash
src/Control/firmware/STM_TEST/tests/run_host_tests.sh
```

## 10. 플래시 및 실행

현재 저장소에는 ST-Link flash command나 board-specific flash script가 없습니다.
사용하는 ST-Link 환경에서 `STM_TEST.elf`를 bench의 STM_TEST 보드에 flash하십시오.

STM_TEST는 micro-ROS를 쓰지 않으므로 Agent를 붙이지 않습니다. PC에서는 serial
terminal로 직접 명령을 보냅니다.

```bash
screen /dev/hil_test 115200
```

ROS 2 bridge와 GUI로 운용할 때의 절차는
[`../../docs/HIL_RUNBOOK.md`](../../docs/HIL_RUNBOOK.md)를 따릅니다.

> **경고:** 실차 보드(`/dev/stm_a`, `/dev/stm_b`)에 STM_TEST 펌웨어를 올리지
> 마십시오. linker script의 FLASH origin은 `0x08000000`이지만, 사용하는 도구의
> flash·reset 명령은 현재 저장소에 없어 TBD입니다.

## 11. 상태 확인

serial terminal에서 다음을 확인합니다.

```text
STATUS
SET MODE AUTO
STATUS
```

`ez_rx`/`ez_rx_age_ms`와 `keya_rx`/`keya_rx_age_ms`로 상대 노드와의 실제 link를
확인하고, `brake_us`와 `pwm_lost`로 STM_B PWM 배선을, `mode`/`gear`/`estop_*`로
GPIO 출력 상태를 확인하십시오.

## 12. 알려진 제한사항

- FDCAN2 bus-off 복구 로직이 없습니다. CAN은 수신 노드가 ACK를 주어야 송신이
  완료되므로, 상대 노드가 없으면(transceiver 미연결, Host `can0` down, USB-CAN
  분리) TX FIFO가 3개에서 차고 FDCAN2가 에러 상태로 들어갑니다. **상대 노드가
  돌아와도 자동으로 회복되지 않으며 보드 리셋이 필요합니다.**
  - 증상: `keya_tx`가 정확히 `3`에서 멈춤. `keya_rx`는 0
  - 복구: Host `can0`을 먼저 올린 뒤 보드를 리셋합니다. 순서가 반대면 리셋 직후
    다시 같은 상태가 됩니다
  - `keya_ok`(`keya_can_healthy`)는 `s_initialized && !s_rx_overflow`이므로 초기화
    성공 여부만 나타내며 bus-off를 반영하지 않습니다. 판정에 쓰지 마십시오
- EZkontrol plant는 1차 지연만 모사하며 차량 질량과 주행 저항은 없습니다.
- KEYA supply voltage/current/temperature 입력이 없어 해당 query는 0을 반환합니다.
- brake PWM은 관측만 하며 STM_B로 되먹임하는 경로가 없습니다.
- MCU independent watchdog 사용은 현재 코드에서 확인되지 않습니다.
- flash 절차와 bench 하네스 connector pinout은 문서화 필요 상태입니다. 위 핀맵은
  MCU 핀 기준입니다.

## 13. 관련 문서

- 펌웨어 공통 빌드·장치 이름: [`../README.md`](../README.md)
- 상대 보드: [`../STM_A/README.md`](../STM_A/README.md),
  [`../STM_B/README.md`](../STM_B/README.md)
- ROS bridge·topic·service와 GUI 계약:
  [`../../software/src/kaiev26_hil/README.md`](../../software/src/kaiev26_hil/README.md)
- HIL 전체 운용 절차: [`../../docs/HIL_RUNBOOK.md`](../../docs/HIL_RUNBOOK.md)
- 통신 계약: [`../../docs/INTERFACES.md`](../../docs/INTERFACES.md)
