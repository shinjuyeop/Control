# UNO_R4_DASHBOARD

Arduino UNO R4 Minima의 내부 CAN controller로 STM_B dashboard protocol v1을 받아
Serial Monitor에 출력하는 firmware입니다. MCP2515는 사용하지 않습니다. RTOS 없이
Arduino `setup()`/`loop()` 구조와 `Arduino_CAN` API를 사용합니다.

기존 STM_B의 MSP4030/ST7796S 초기화 sequence와 480x320 dashboard UI의 배치, 문구,
색상, validity 표현을 Arduino SPI로 이식했습니다. 첫 화면을 전체 렌더한 뒤에는 CAN
snapshot에서 바뀐 문자와 badge만 갱신합니다.

## Local UI preview

[`dashboard_preview.py`](dashboard_preview.py)는 CAN/UNO/LCD hardware 없이 동일한
dashboard 값을 480x320 화면으로 확인하는 개발용 GUI입니다. 현재 펌웨어의 좌표,
RGB565 색과 5x7 font를 옮긴 BASELINE과 다음 8개 시안을 같은 상태로 비교합니다.
Python 3, Tkinter와 Pillow가 필요합니다.

- `GT3_VITALS`: 수치 진단을 위한 dark three-column grid
- `CUP_LIGHT`: battery / road / motor를 나눈 light cockpit grid
- `DUAL_DIAL`: bus-current와 phase-current 부하의 직접 비교
- `TAPE_GAUGES`: 좌우 눈금 tape에서 bus와 phase 위치를 직접 비교
- `HISTORY_SCOPE`: 최근 5초 current/temperature 궤적을 oscilloscope로 표시
- `LOAD_ENVELOPE`: bus-current와 phase-current 운전점 및 최근 trail
- `MANUAL_TFT`: 계기판 reference를 옮긴 2중 BUS/PHASE arc, 곡선형 F/R, speed,
  4열 PED/HV/MOTOR/CTRL gauge와 한 줄 상태 표시
- `AUTO_GT3`: HV/bus/controller, speed/host command, phase/motor를 나눈 AUTO 전용 grid

`MANUAL_TFT`와 `AUTO_GT3`는 mode page 한 쌍입니다. 드롭다운에서 각 page를 직접 열어
볼 수 있고, 둘 중 하나를 사용하던 중 CAN의 MANUAL/AUTO mode가 바뀌면 해당 page로
자동 전환합니다. 현재 CAN에 없는 steering 상태와 LV battery SOC는 빈 placeholder로
만들지 않습니다. GUI 기본 variant도 이 mode page 쌍입니다. BASELINE과 나머지 안은
드롭다운에서 비교할 수 있습니다.

GUI를 열면 검은 화면에서 KAI symbol의 흰 outline이 약한 haze와 함께 나타나는 1.4초
startup sequence가 한 번 재생됩니다. `Replay startup`으로 다시 볼 수 있으며 symbol은
이동하거나 채워지지 않고 TEAM K.A.I. 문구도 표시하지 않습니다. 수동 화면은 설치된
Blender Pro Medium/Bold/Heavy를 label/value/gear·speed 역할에 나눠 사용하고, 폰트가
없는 환경에서는 기존 condensed font로 fallback합니다.

수동 화면의 BUS/PHASE track은 수직 구간, 대각선 구간, 수평 구간과 두 전이곡선을
고정 antialias mask로 한 번만 만들고 그 안에서 진행 길이만 갱신합니다. 따라서 값이
변해도 곡선 외곽 픽셀이 프레임마다 다시 만들어지지 않습니다.

미리보기 구현은 시안 추가가 한 파일을 계속 키우지 않도록 다음처럼 분리했습니다.

- `dashboard_preview.py`: 기존 실행 명령을 보존하는 8줄 진입점
- `dashboard_preview/model.py`: CAN 값, scenario, history와 gauge dynamics
- `dashboard_preview/palette.py`: 공통 색상과 font
- `dashboard_preview/core.py`: semantic state 합성과 공통 drawing primitive
- `dashboard_preview/registry.py`: GUI/CLI에 노출할 variant 순서와 renderer 이름
- `dashboard_preview/variants/`: 기존 계기류와 semantic 시안군
- `dashboard_preview/app.py`: Tk GUI와 snapshot CLI

```bash
cd firmware/UNO_R4_DASHBOARD
python3 dashboard_preview.py
```

오른쪽 panel에서 scenario, 10/20 Hz telemetry cadence, MANUAL/AUTO, FWD/REV,
validity, E-stop/brake와 motor fault 24종을 바꿀 수 있습니다. 실제 LCD 해상도의 PNG만
저장하려면 GUI의 `Save 480x320 PNG`를 사용하거나 다음처럼 실행합니다.

```bash
python3 dashboard_preview.py \
  --variant manual_tft --scenario load --time 6 --snapshot /tmp/dashboard.png
```

Python GUI의 current/temperature threshold와 animation은 표시 시안일 뿐 CAN contract나
차량 safety limit이 아닙니다. 이 도구는 SocketCAN이나 serial device를 열지 않으며
STM/UNO firmware 상태를 변경하지 않습니다.

## CAN contract

| 항목 | 값 |
|---|---|
| Board | Arduino UNO R4 Minima |
| Controller | Renesas RA4M1 internal CAN |
| Controller pins | `D4=CANTX0`, `D5=CANRX0` |
| Transceiver | MCP2551 |
| Frame | Classical CAN, standard data frame, DLC 8 |
| Bitrate | 250 kbps |
| Accepted IDs | `0x321` STATUS, `0x322` POWER, `0x323` THERMAL, `0x324` FAULT |
| Protocol | version 1, little-endian, shared `uint8` alive |
| Snapshot | 4개 ID와 version/alive가 모두 일치해야 완성 |
| CAN LOST | 새 alive의 완전한 4-frame snapshot이 300 ms 없음 |
| CAN recovery | LOST 중 1초마다 `CAN.end()`/`CAN.begin()` 재시도 |
| Serial Monitor | 115200 baud |

`loop()`은 `CAN.available()`을 확인한 뒤 준비된 frame만 `CAN.read()`하므로 CAN 수신을
기다리며 blocking하지 않습니다. ID, DLC 또는 version이 다른 frame은 무시합니다.
같은 alive가 반복되면 duplicate로 세고 link health를 갱신하지 않습니다. 완성 snapshot은
200 ms마다 출력하여 Serial 출력이 CAN 수신을 방해하지 않게 합니다.
정확한 byte layout과 source type/unit은
[`../STM_B/README.md`](../STM_B/README.md#85-dashboard-snapshot과-wire-단위)가 canonical contract입니다.

## UNO R4 Minima ↔ MCP2551 배선

| UNO R4 Minima | MCP2551 | 설명 |
|---|---|---|
| `D4/CANTX0` | pin 1 `TXD` | controller TX → transceiver input |
| `D5/CANRX0` | pin 4 `RXD` | transceiver output → controller RX |
| 5 V | pin 3 `VDD` | transceiver supply |
| GND | pin 2 `VSS` | logic/bus ground |
| GND | pin 8 `RS` | high-speed mode |
| — | pin 5 `VREF` | 사용하지 않음 |
| CANL | pin 6 `CANL` | differential bus low |
| CANH | pin 7 `CANH` | differential bus high |

MCP2551 VDD 가까이에 100 nF decoupling capacitor를 둡니다. D4/D5를 CANH/CANL에
직접 연결하면 안 됩니다.

## CAN bus 연결

- SN65HVD230 `CANH` ↔ MCP2551 `CANH`
- SN65HVD230 `CANL` ↔ MCP2551 `CANL`
- STM_B GND ↔ UNO R4/MCP2551 GND
- CANH/CANL은 twisted pair 사용
- cable 양 끝에만 CANH-CANL 120 ohm termination
- 두 node만 있는 bench라면 각 transceiver 끝에 120 ohm 하나씩 사용

## Build

Arduino IDE:

1. Boards Manager에서 `Arduino UNO R4 Boards`를 설치합니다.
2. Board를 `Arduino UNO R4 Minima`로 선택합니다.
3. [`UNO_R4_DASHBOARD.ino`](UNO_R4_DASHBOARD.ino)를 열고 Verify/Upload합니다.

arduino-cli:

```bash
cd ~/KAI_ws/src/Control/firmware
arduino-cli core update-index
arduino-cli core install arduino:renesas_uno
arduino-cli compile --fqbn arduino:renesas_uno:minima UNO_R4_DASHBOARD
```

`Arduino_CAN`은 Renesas UNO core에 포함돼 있으므로 별도 MCP2515 library를 설치하지
않습니다.

Arduino compile은 sketch/API 호환성 증거이며 실제 MCP2551/CAN bus 수신 증거는
아닙니다.

2026-09-02 Arduino UNO R4 Boards core `1.6.0` recovery compile 결과는 Flash 64,656
byte(24%), global RAM 8,920 byte(27%)였습니다.

## Bench smoke test

1. 모든 전원을 끄고 STM_B/SN65HVD230과 UNO R4/MCP2551을 위 표대로 배선합니다.
2. CANH-CANL, 공통 GND, 양 끝 120 ohm termination을 확인합니다.
3. 전원 OFF에서 CANH-CANL 저항이 약 60 ohm인지 확인합니다.
4. 이 sketch를 UNO R4 Minima에 업로드합니다.
5. Serial Monitor를 115200 baud로 엽니다.
6. STM_B와 두 transceiver를 켭니다.
7. 다음 형태로 모든 dashboard 값이 출력되고 `complete`가 증가하며 `ignored=0`인지
   확인합니다.

```text
DASH v1 alive=42 complete=101 ignored=0 duplicate=0 reinit=0
  LINK vehicle=OK detail=OK part1=VALID part2=VALID
  MOTION mode=MANUAL gear=FWD speed=0.00 m/s display=0.0 km/h
  SAFETY estop=NO brake=NO
  DRIVE pedal=0% tractive=STOP command=WAIT drive_stop=YES
  POWER voltage=72.4 V bus_current=-18.3 A phase_current=256.7 A
  THERMAL controller=34.2 C motor=42.1 C fault=0x000000
```

8. CAN 연결을 300 ms 넘게 끊어 `CAN LOST`와 LCD의 `DASH CAN LOST`를 확인합니다.
9. 1초 뒤 `CAN controller reinitialized`가 출력되고 `reinit`이 증가하는지 확인합니다.
10. 다시 연결하여 새 alive의 완전 snapshot 뒤 `CAN RECOVERED`와 UI 갱신이
    재개되는지 확인합니다.

snapshot이 완성되지 않거나 `ignored`/`duplicate`/`reinit`이 안정 상태에서 증가하면
bitrate, ID/DLC/version, termination, CANH/CANL polarity, 공통 GND와 transceiver
전원을 먼저 확인합니다. LINK의 `vehicle/detail LOST`는 dashboard CAN 단절이 아니라
STM_A→STM_B UART source가 stale인 상태입니다.

## MSP4030/ST7796S LCD dashboard

대상은 기존 STM_B에 연결했던 QDtech MSP4030 4.0-inch SPI module입니다. 모듈 사양과
회로도에는 5V/3.3V MCU 호환 level conversion과 onboard 3.3V regulator가 있으며 VCC는
5V 연결을 권장합니다. 이 배선은 raw ST7796S panel/FPC에 적용하면 안 됩니다.

| MSP4030 header | UNO R4 Minima | 설명 |
|---|---|---|
| `VCC` | `5V` | module power; backlight 사양 전류 약 103 mA |
| `GND` | `GND` | CAN/STM과도 공통 ground |
| `LCD_CS` | `D10` | active-low chip select |
| `LCD_RST` | `D8` | active-low reset |
| `LCD_RS` | `D9` | command/data select (`DC`) |
| `SDI(MOSI)` | `D11/COPI` | SPI write data |
| `SCK` | `D13/SCK` | SPI clock |
| `LED` | `D7` | HIGH=backlight on |
| `SDO(MISO)` | 연결하지 않음 | 이번 write-only driver에서는 readback 미사용 |

`D4/D5`는 CAN TX/RX로 계속 사용합니다. `D12/MISO`, touch와 SD-card pin도 이번
단계에서는 사용하지 않습니다.

[`st7796_lcd.cpp`](st7796_lcd.cpp)는 기존 STM_B와 같은 MSP4030 vendor command와
landscape `MADCTL=0x28`, RGB565를 사용합니다. Arduino SPI는 mode 3, MSB first,
1 MHz로 설정했습니다. 초기화 순서는 다음과 같습니다.

1. backlight OFF
2. reset LOW 100 ms, HIGH 50 ms
3. MSP4030 vendor sequence
4. `NORON`, `SLPOUT`, 120 ms, `DISPON`, 20 ms
5. CAN controller 시작
6. 기존 STM_B와 같은 static layout과 현재 state를 전체 렌더
7. backlight ON
8. 완성된 CAN snapshot마다 변경된 문자와 badge만 갱신

Arduino SPI API는 panel readback을 사용하지 않으므로 Serial의
`LCD dashboard UI rendered`는 command와 pixel 전송 함수가 끝났다는 뜻이지 실제
panel 표시 증거는 아닙니다. STM_A가 연결되지 않은 bench에서는 다음 화면이 정상입니다.

- top badge: `MODE --`, `--`, `A-B LOST`, `SAFE`
- HV BUS, SPEED, MOTOR, CTRL, FAULT 값: `--`
- bottom bar: `STM A COMM LOST`, `VALUES STALE`

Dashboard CAN까지 끊기거나 alive가 진행하지 않으면 bottom bar는 `DASH CAN LOST`,
`VALUES STALE`로 바뀝니다. LOST 상태에서 controller 재초기화가 성공해도 fresh complete
snapshot이 들어오기 전에는 stale 화면을 유지합니다.
[`dashboard_ui.cpp`](dashboard_ui.cpp)가 기존 STM_B `display_app.c`의 좌표, badge 색상,
표시 우선순위와 5x7 font rendering을 담당합니다.

## References

- [Sitronix ST7796S Datasheet V1.0](<../../docs/hardware/ST7796S-Sitronix (display).pdf>)
- [Arduino UNO R4 Minima documentation](https://docs.arduino.cc/hardware/uno-r4-minima)
- [Arduino Renesas Minima CAN pin mapping](https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/MINIMA/pins_arduino.h)
- [Arduino_CAN non-blocking receive example](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_CAN/examples/CANRead/CANRead.ino)
- [Microchip MCP2551 datasheet](https://www.microchip.com/content/dam/mchp/documents/APID/ProductDocuments/DataSheets/20001667G.pdf)
