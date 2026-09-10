# UNO_R4_DASHBOARD

Arduino UNO R4 Minima의 내부 CAN controller로 STM_B dashboard protocol v1을 받아
Serial Monitor에 출력하는 firmware입니다. MCP2515는 사용하지 않습니다. RTOS 없이
Arduino `setup()`/`loop()` 구조와 `Arduino_CAN` API를 사용합니다.

기존 STM_B의 MSP4030/ST7796S 초기화 sequence와 480x320 dashboard UI의 배치, 문구,
색상, validity 표현을 Arduino SPI로 이식했습니다. 첫 화면을 전체 렌더한 뒤에는 CAN
snapshot에서 바뀐 문자와 badge만 갱신합니다. MSP4031 터치 모델의 FT6336U를
I2C로 읽고, `HIL_TOUCH ON`으로 raw 좌표와 INT 상태를 관찰할 수 있습니다.
부팅 시 기존 계기판에서 시작하며, 스와이프로 `L — 기존 계기판 — R` 사이를 이동합니다.
물리 터치와 Web Virtual LCD 입력은 같은 UNO gesture/navigation 로직을 사용합니다.
터치는 차량 명령에 연결하지 않습니다.

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

`MANUAL_TFT`와 `AUTO_GT3`는 preview의 mode page 한 쌍입니다. 드롭다운에서 각 page를 직접 열어
볼 수 있고, 둘 중 하나를 사용하던 중 CAN의 MANUAL/AUTO mode가 바뀌면 해당 page로
자동 전환합니다. 현재 CAN에 없는 steering 상태와 LV battery SOC는 빈 placeholder로
만들지 않습니다. GUI 기본 variant도 이 mode page 쌍입니다. BASELINE과 나머지 안은
드롭다운에서 비교할 수 있습니다.

GNSS 상태는 `MANUAL_TFT` 하단 `GNSS` 셀과 `AUTO_GT3` 상단 중앙에 표시합니다.
`RTK FIXED`, 일반 `GNSS FIX`, `NO FIX`, 미수신/500 ms stale을 서로 구분합니다.
실제 Arduino firmware는 MANUAL에서 `MANUAL_TFT`의 곡선형 BUS/PHASE track,
우측 4열 gauge와 하단 상태줄을 사용하고, AUTO에서는 기존 진단 grid를 유지합니다.
[`manual_tft_assets.h`](manual_tft_assets.h)는 preview renderer가 만든 정적 raster,
곡선 antialias/progress mask, Blender Pro glyph와 1.4초 startup keyframe입니다. 직접
편집하지 않고 다음 명령으로 다시 생성합니다.

```bash
python3 -m dashboard_preview.generate_firmware_assets
```

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
validity, E-stop/brake, GNSS 네 상태와 motor fault 24종을 바꿀 수 있습니다. 실제 LCD 해상도의 PNG만
저장하려면 GUI의 `Save 480x320 PNG`를 사용하거나 다음처럼 실행합니다.

```bash
python3 dashboard_preview.py \
  --variant manual_tft --scenario load --time 6 --snapshot /tmp/dashboard.png
```

Python GUI의 current/temperature threshold와 animation은 표시 시안일 뿐 CAN contract나
차량 safety limit이 아닙니다. 이 도구는 SocketCAN이나 serial device를 열지 않으며
STM/UNO firmware 상태를 변경하지 않습니다.

## Web Virtual LCD와 UNO 상태 보고

`dashboard_preview/native`는 실제 `dashboard_ui.cpp`, `manual_tft_ui.cpp`,
`st7796_lcd.cpp`를 PC용으로 빌드한다. Arduino/SPI 대신 PC 메모리의 RGB565 화면에
그리는 adapter만 교체하므로, MANUAL의 preview 생성 아셋과 AUTO의 기존 진단 grid를
모두 실제 C++ 코드로 재사용한다. Python 시안 `AUTO_GT3`를 실제 AUTO 화면으로
오인해 표시하지 않는다. 기존 Tk preview 기능은 별도로 유지한다.

```bash
# Control 저장소에서
cmake -S firmware/UNO_R4_DASHBOARD/dashboard_preview/native \
  -B firmware/UNO_R4_DASHBOARD/build/host -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/UNO_R4_DASHBOARD/build/host --parallel 2
cmake --install firmware/UNO_R4_DASHBOARD/build/host --prefix software/src/kaiev26_hil
```

USB serial 115200에서 `HIL_LCD ON\n`을 보내면 200 ms마다 한 줄의 상태 보고를
출력한다. `HIL_LCD OFF\n` 또는 USB 호스트 연결 해제로 보고를 끈다. 이 명령은
UI·CAN·차량 입력을 변경하지 않는다. 보고 중에는 기존 사람용 CAN snapshot 로그를
억제해 한 줄이 섞이지 않게 한다. USB가 없어도 기존 CAN/LCD 동작은 독립적이다.

예:

```text
HIL_LCD {"v":1,"uptime_ms":10000,"seq":42,"can_lost":false,"can_age_ms":12,"page":"MANUAL_TFT","values":[15,3,25,3,120,523,125,1200,320,400,0]}
```

- `v`: USB 상태 보고 protocol version 1. Dashboard CAN protocol version과 별도다.
- `uptime_ms`: UNO의 millis. `seq`: 완성된 새 CAN snapshot의 부팅 이후 누적 수.
- `can_lost`: **LCD drawing에 적용한** dashboard CAN LOST 상태.
- `can_age_ms`: 마지막 완성 CAN snapshot 이후 경과시간. 미수신은 -1.
- `page`: 실제 적용 페이지 `MANUAL_TFT`, `AUTO_BASELINE`, `SWIPE_L`, `SWIPE_R`.
- `vtouch`: 선택적 확장 필드. `1`은 아래 Virtual Touch USB protocol 지원을 뜻한다.
- `touch_errors`: 선택적 확장 필드. FT6336U의 누적 I2C/invalid sample 오류 수.
- `values`: LCD drawing을 완료한 `DashboardValues`의 아래 순서 원시 정수 값.

| Index | Field | 단위 / 의미 |
|---|---|---|
| 0 | status_flags | CAN 상태 비트 |
| 1 | drive_flags | 구동 상태 비트 |
| 2 | pedal_percent | % |
| 3 | gnss_state | 0~3 enum |
| 4 | speed_centi_mps | 0.01 m/s |
| 5 | bus_voltage_decivolt | 0.1 V |
| 6 | bus_current_deciamp | 0.1 A |
| 7 | phase_current_deciamp | 0.1 A |
| 8 | controller_temperature_decic | 0.1 °C |
| 9 | motor_temperature_decic | 0.1 °C |
| 10 | motor_fault_bits | 24-bit fault |

보고값은 `DashboardUi`가 보유한 적용 상태에서 가져온다. CAN이 끊겨도 UNO의 USB
보고는 계속되며, 실제 표시와 같은 CAN LOST 화면을 PC가 재구성할 수 있다.
USB 보고가 끊긴 것은 dashboard CAN LOST와 구분한다. PC는 마지막 이미지를 흐리게
표시하고 현재 상태를 확인할 수 없음을 알린다. 전체 framebuffer는 UNO에 추가하지 않았다.
512-byte TX 버퍼를 USB 가용 공간만큼 나누어 보내며 USB 전송을 기다리지 않는다.

2026-09-09 상태 보고 추가판 compile: Flash **217,880 B (83%)**, global RAM
**10,720 B (32%)**. 이는 빌드 결과이며 실제 업로드/패널 검증과 별도다.

같은 날 USB serial `3114101259323437A93033334B57305C`인 UNO R4 Minima에
이 바이너리를 DFU로 업로드했다. 업로드 후 `HIL_LCD` v1 보고, MANUAL_TFT 페이지,
CAN 정상 수신과 snapshot 누적 증가를 실제 USB에서 확인했다. PC native renderer와
Web에서 480×320 이미지 표시까지 확인했으며 실제 LCD 패널 자체의 광학 검증은 수행하지 않았다.

## CAN contract

| 항목 | 값 |
|---|---|
| Board | Arduino UNO R4 Minima |
| Controller | Renesas RA4M1 internal CAN |
| Controller pins | `D4=CANTX0`, `D5=CANRX0` |
| Transceiver | TJA1050 |
| Frame | Classical CAN, standard data frame, DLC 8 |
| Bitrate | 250 kbps |
| Accepted IDs | `0x321` STATUS, `0x322` POWER, `0x323` THERMAL, `0x324` FAULT |
| Protocol | version 1, little-endian, shared `uint8` alive |
| Snapshot | 4개 ID와 version/alive가 모두 일치해야 완성 |
| GNSS | STATUS byte 5: 0 unavailable, 1 no fix, 2 fix, 3 RTK fixed |
| CAN LOST | 새 alive의 완전한 4-frame snapshot이 300 ms 없음 |
| CAN recovery | LOST 중 1초마다 `CAN.end()`/`CAN.begin()` 재시도 |
| Serial Monitor | 115200 baud |

`loop()`은 `CAN.available()`을 확인한 뒤 준비된 frame만 `CAN.read()`하므로 CAN 수신을
기다리며 blocking하지 않습니다. ID, DLC 또는 version이 다른 frame은 무시합니다.
같은 alive가 반복되면 duplicate로 세고 link health를 갱신하지 않습니다. 완성 snapshot은
200 ms마다 출력하여 Serial 출력이 CAN 수신을 방해하지 않게 합니다.
정확한 byte layout과 source type/unit은
[`../STM_B/README.md`](../STM_B/README.md#85-dashboard-snapshot과-wire-단위)가 canonical contract입니다.

## UNO R4 Minima ↔ TJA1050 배선

| UNO R4 Minima | TJA1050 | 설명 |
|---|---|---|
| `D4/CANTX0` | pin 1 `TXD` | controller TX → transceiver input |
| `D5/CANRX0` | pin 4 `RXD` | transceiver output → controller RX |
| 5 V | pin 3 `VCC` | transceiver supply |
| GND | pin 2 `GND` | logic/bus ground |
| GND | pin 8 `S/RS` | high-speed mode |
| — | pin 5 `VREF` | 사용하지 않음 |
| CANL | pin 6 `CANL` | differential bus low |
| CANH | pin 7 `CANH` | differential bus high |

TJA1050 VCC 가까이에 100 nF decoupling capacitor를 둡니다. D4/D5를 CANH/CANL에
직접 연결하면 안 됩니다.

## CAN bus 연결

- SN65HVD230 `CANH` ↔ TJA1050 `CANH`
- SN65HVD230 `CANL` ↔ TJA1050 `CANL`
- STM_B GND ↔ UNO R4/TJA1050 GND
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

Arduino compile은 sketch/API 호환성 증거이며 실제 TJA1050/CAN bus 수신 증거는
아닙니다.

2026-09-04 Arduino UNO R4 Boards core `1.6.0` preview-asset MANUAL TFT compile
결과는 Flash 217,032 byte(82%), global RAM 10,344 byte(31%)였습니다.

## Bench smoke test

1. 모든 전원을 끄고 STM_B/SN65HVD230과 UNO R4/TJA1050을 위 표대로 배선합니다.
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
  GNSS state=RTK_FIXED
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

## MSP4030/MSP4031 ST7796S LCD dashboard

대상은 QDtech MSP4030(터치 없음) 및 MSP4031(FT6336U 정전식 터치) 4.0-inch
SPI module입니다. 모듈 사양과
회로도에는 5V/3.3V MCU 호환 level conversion과 onboard 3.3V regulator가 있으며 VCC는
5V 연결을 권장합니다. 이 배선은 raw ST7796S panel/FPC에 적용하면 안 됩니다.

| MSP4030/MSP4031 header | UNO R4 Minima | 설명 |
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
| `SD_CS` | 연결하지 않음 | SD card 미사용 |

### CTP touch 배선

| CTP touch header | UNO R4 Minima | 설명 |
|---|---|---|
| `CTP_SCL` | `A5/SCL` | I2C clock |
| `CTP_SDA` | `A4/SDA` | I2C data |
| `CTP_RST` | `D3` | touch controller reset |
| `CTP_INT` | `D2` | touch interrupt |

`D4/D5`는 CAN TX/RX로 계속 사용합니다. `D12/MISO`와 `SD_CS`는 연결하지 않습니다.
터치는 모듈의 공용 VCC/GND와 내부 3.3 V regulator에서 전원을 받습니다.
제조사 회로도에서 SDA/SCL은 BSS138 양방향 level conversion, RST는
74LVC245를 거칩니다. INT는 터치 칩에서 직접 연결되므로 D2는 `INPUT`으로
설정하며 5 V `INPUT_PULLUP`을 사용하지 않습니다.
UNO R4의 D2(P105)는 5 V 동작 시 HIGH 보장 입력이 4.0 V 이상입니다
([RA4M1 datasheet, Table 2.4](https://www.renesas.com/en/document/dst/ra4m1-group-datasheet)).
따라서 3.3 V CTP_INT 직결로는 HIGH 인식이 보장되지 않으며, INT를 사용할 경우
3.3 V 신호를 받아 5 V로 내보내는 level conversion이 필요합니다.
좌표 읽기는 INT에 의존하지 않는 polling으로 동작합니다.

### FT6336U 터치 확인

[`ft6336_touch.cpp`](ft6336_touch.cpp)는 제조사 예제의 I2C 7-bit 주소 `0x38`과
리셋 LOW 20 ms / HIGH 후 500 ms 대기를 사용합니다. UNO의 `Wire`를 100 kHz로
설정하고 20 ms마다 `0x02`부터 터치 수와 최대 두 점을 한 번에 읽습니다.
I2C timeout은 3000 µs이며 읽기 실패 후 재시도 주기는 1초입니다.
INT ISR은 falling edge 횟수만 셉니다. 좌표 읽기는 polling하므로 INT 배선이
없거나 펄스를 놓쳐도 누름/해제를 확인할 수 있습니다.

Arduino Serial Monitor(115200 baud, newline)에 다음을 입력합니다.
HIL 웹의 Arduino reader와 Serial Monitor를 동시에 열지 않습니다.

```text
HIL_TOUCH ON
```

100 ms마다 `HIL_TOUCH {JSON}` 한 줄을 출력합니다. `HIL_LCD ON`과는
상호 배타적이며, 기존 HIL LCD JSON 형식은 변경하지 않습니다.

- `addr`: 56 = `0x38`; `identified`: 칩 식별 레지스터 읽기 성공
- `vendor`, `chip`, `fw`: 실제 읽은 `0xA8`, `[0xA3,0x9F,0xA0]`, `0xA6`
- `valid`, `sample_ms`: 최근 좌표 읽기의 성공 여부와 마지막 성공 시각
- `irq`, `irq_edges`: D2 현재 레벨과 falling edge 누적 횟수
- `count`: 현재 터치 수, 0이면 해제. `points`의 앞 `count`개만 유효
- `points`: 각 점 `[id,event,raw_x,raw_y]`, event 0=down, 1=up, 2=contact
- `samples`, `errors`: 읽기 시도와 오류 누적 횟수
- `can_seq`, `can_age_ms`: 터치 확인 중에도 CAN 수신이 계속되는지 확인

`points`는 패널의 raw 320×480 축을 유지합니다. 현재 가로 LCD 방향에는
`screen_x = raw_y`, `screen_y = 319 - raw_x`로 대응합니다. 2026-09-10 사용자가
좌상 → 우상 → 좌하 → 우하 → 중앙 순서로 누른 것을 확인하고 두 차례의
터치 좌표로 방향을 대조했습니다. 임의의 5점 보정 계수는 적용하지 않습니다.

```text
HIL_TOUCH OFF
```

위 명령 또는 USB Serial Monitor 종료 시 터치 로그가 꺼집니다. CAN 수신과
계기판 렌더링은 터치 진단 중에도 유지됩니다.

2026-09-10 HIL Arduino(serial `3114101259323437A93033334B57305C`)에
업로드 후 `0x38`에서 vendor `0x11`, chip `0x64/0x26/0x02`, firmware `0xA4`를
읽었습니다. 실제 누름을 수집한 60초 동안 I2C errors=0, CAN seq 3563→4765였고
각 누름에서 count=1, 해제 시 count=0으로 복귀했습니다. 두 번째 5점의 대표값은
다음과 같습니다. 모서리의 정확한 경계값/픽셀 오차를 측정한 것은 아닙니다.

| 사용자가 누른 위치 | raw X/Y | 현재 화면 방향의 X/Y |
|---|---|---|
| 좌상 | 319 / 43 | 43 / 0 |
| 우상 | 303 / 445 | 445 / 16 |
| 좌하 | 26 / 70 | 70 / 293 |
| 우하 | 23 / 451 | 451 / 296 |
| 중앙 | 142 / 253 | 253 / 177 |

INT는 누름/해제 중에도 LOW, falling edge=0으로 관측했습니다. 터치 좌표는 polling으로
정상 수신되지만 INT 배선/전압 동작은 통과하지 못했습니다. 앞서 설명한 3.3 V→5 V
입력 레벨 문제와 물리 배선을 실측해야 하며, 이번에는 전압을 측정하지 않았습니다.
빌드 결과는 Flash 225,364 bytes(85%), global RAM 12,116 bytes(36%)입니다.

### L — 기존 계기판 — R 페이지 이동

부팅 시 중앙의 기존 계기판이 나오고 페이지 이동은 기본으로 켜집니다.
손가락으로 페이지를 끌어 넘기는 방향을 사용합니다.

| 현재 화면 | 오른쪽으로 밀기 | 왼쪽으로 밀기 |
|---|---|---|
| L | L 유지 | 기존 계기판 |
| 기존 계기판 | L | R |
| R | 기존 계기판 | R 유지 |

중앙은 기존 `MANUAL_TFT`/`AUTO_BASELINE` 코드 그대로 사용합니다.
L/R 화면에서도 CAN 최신 값과 모드를 계속 받아 두고, 중앙으로 돌아오면 최신
값으로 다시 그립니다. 한 번의 스와이프로 한 페이지만 이동하며 양 끝은 순환하지 않습니다.

115200 baud Serial Monitor에서 다음을 보내면 중앙으로 돌아오며 페이지 이동을 켭니다.

```text
SWIPE_TEST ON
```

손가락 하나의 가로 이동이 60픽셀 이상이고 세로 이동보다 클 때, 손을 떼는 시점에 한 번
판정합니다. 짧은 탭이나 세로 이동은 화면을 바꾸지 않습니다. 두 손가락 터치나
I2C 읽기 실패가 끼어든 동작은 취소하고 완전히 손을 뗀 뒤 다시 받습니다.
이 표시는 방향 확인용이며 기어 선택이나 차량 제어 명령이 아닙니다.

```text
SWIPE_TEST OFF
```

OFF는 최신 CAN 값의 계기판으로 복귀하고 페이지 이동을 끕니다.
설정은 Serial Monitor를 닫아도 유지되며, Arduino reset 시 중앙/ON으로 돌아옵니다.

웹 Virtual LCD는 중앙의 기존 page 값과 `page=SWIPE_R/SWIPE_L` 보고를 받아
같은 C++ 그리기 코드로 표시합니다. 위 native renderer 빌드/설치와 Python
`dashboard_display.py` 갱신을 함께 적용해야 합니다. `HIL_LCD ON/OFF`는
보고만 켜고 끄며 스와이프 테스트 모드는 바꾸지 않습니다.
이전 단일 방향 테스트의 `SWIPE_WAIT`는 호스트 호환용으로만 유지합니다.

2026-09-10 페이지 이동 방식으로 갱신한 빌드는 Flash 226,124 bytes(86%),
global RAM 12,152 bytes(37%)이며 기존 LCD 보고 테스트 6개를 통과했습니다.
HIL Arduino에 복구 업로드를 완료했고, 부팅 시 `MANUAL_TFT`와 오른쪽 스와이프 후
`SWIPE_L` 보고를 확인했습니다. 업로드 중 사용자 reset으로 전송이 한 번 중단됐고,
복구 업로드 뒤 흰 화면은 Arduino/LCD 전원을 완전히 껐다 켠 후 정상 계기판으로
돌아왔다고 사용자가 확인했습니다. 이후 60초 동안 CAN seq 572→1774,
CAN LOST 보고 0건을 확인했습니다. 전원 재연결 후 사용자가 실제 LCD에서
오른쪽 → 왼쪽 → 왼쪽 → 오른쪽 입력에 따라 `L → 기존 계기판 → R → 기존 계기판`이
요청한 순서대로 표시됨을 확인했습니다(사용자 실물 LCD 검증 확인).

### Web HIL Virtual Touch v1

기존 `SwipeTest`는 FT6336U 상태를 직접 읽어 gesture 판정과 페이지 변경을 함께
수행했습니다. 이제 [`touch_navigation.h`](touch_navigation.h)의 `Gesture`와
`Navigator`는 Arduino API 없이 동작하고, `swipe_test.cpp`는 물리 입력 정규화와
`DashboardUi::showSwipeTest()` 호출만 연결합니다. 기존 Serial 명령 이름은 유지합니다.

```text
FT6336U polling → screen TouchSample ─┐
                                    ├→ Gesture / Navigator → DashboardUi → physical LCD
Web PointerEvent → WebSocket → USB ─┘                               │
                 Web Virtual LCD ← native renderer ← HIL_LCD report ┘
```

공통 `TouchSample`은 `phase`(DOWN/MOVE/UP/CANCEL), `source`(PHYSICAL/VIRTUAL),
`x/y`, `finger`, `contacts`를 담습니다. 좌표는 항상 **480×320**, x=0..479,
y=0..319입니다. 물리 raw 320×480 좌표는 검증된 `screen_x=raw_y`,
`screen_y=319-raw_x`로 변환합니다. calibration 계수는 없습니다.

FT6336U 주소 `0x38`, A4=SDA/A5=SCL/D3=RST/D2=INT 배선과 **20 ms polling**을
그대로 사용합니다. **INT는 기능 경로에서 의도적으로 사용하지 않습니다.** 기존 ISR은
진단용 edge counter만 유지하며, INT 전압 문제를 interrupt 구조 변경으로 우회하지 않습니다.
`HIL_TOUCH ON/OFF`의 physical raw diagnostics JSON과 보고 주기는 유지합니다.

USB 115200 baud, newline 구분 ASCII protocol:

```text
VT D 120 180
VT M 160 181
VT M 220 181
VT U 300 182
VT C
```

D=누름, M=접촉 이동, U=해제, C=판정 없이 취소입니다. D/M/U는 screen 좌표
정수 두 개가 필요하며 음수·소수·범위 초과·추가 token을 거부합니다. C에는 좌표가
없습니다. 잘못된 VT 명령이나 줄 overflow는 virtual gesture를 취소합니다.
16-byte 고정 입력 버퍼와 loop당 최대 32-byte 입력 처리를 유지하며 동적 할당이나
입력 대기가 없습니다. 출력은 기존 가용 USB TX 공간만큼 나누어 보내고 CAN loop로
돌아갑니다. `HIL_LCD ON/OFF`, `HIL_TOUCH ON/OFF`, `SWIPE_TEST ON/OFF`는 유지합니다.

**Physical touch wins.** 물리 손가락이 있거나 읽기 오류/다중 터치로 해제를 기다리는
동안 virtual 입력을 무시합니다. virtual drag 도중 물리 입력이 시작되면 virtual
동작을 취소하고 물리 입력을 받습니다. 물리 해제 후 **150 ms** 동안 virtual DOWN을
받지 않으며, guard 중 무시된 DOWN의 뒤늦은 MOVE/UP으로 gesture가 시작되지 않습니다.
물리 idle polling은 진행 중인 virtual gesture에 UP으로 전달되지 않습니다.
두 source의 좌표를 한 gesture에 섞지 않습니다.

두 입력 모두 가로 거리 **60 px 이상**, `abs(dx)>abs(dy)`, 손가락 하나, **UP에서
한 번** 판정하며 L↔Dashboard↔R을 한 단계씩 이동합니다. 경계는 순환하지 않습니다.
짧은 탭·세로 drag·다중 터치·invalid 좌표·finger/source 변경·C는 이동하지 않습니다.
virtual 입력이 **1초** 동안 끊기거나 USB 연결이 닫히면 판정 없이 취소합니다.
물리 polling 경로의 timeout이나 차량 safety semantics를 바꾸지 않습니다.

PC는 `DashboardDisplay`의 기존 exclusive USB connection과 worker 하나를 사용합니다.
Web 요청은 lock으로 보호된 최대 **32개** command queue에 들어갑니다. 대기 MOVE만
최신 좌표로 합치고 USB MOVE는 최대 50 Hz로 보냅니다. DOWN/UP 순서는 보존하고,
UP의 최종 좌표가 직전 대기 MOVE를 대신합니다. 포화·보고 stale·pointer timeout·취소 시
gesture 전체를 폐기하고 C를 보냅니다. 하나의 edge만 버리고 계속 진행하지 않습니다.
연결을 열 때 부분 line을 종료하고 C를 먼저 전송하며 pending gesture를 재생하지 않습니다.
connection generation으로 재연결 전 브라우저 요청을 거부합니다.

Web은 image content rect를 480×320으로 변환하므로 bezel·1×·2×·responsive 크기에
영향받지 않습니다. Pointer Events와 capture로 mouse/touch를 함께 처리합니다.
MOVE를 약 20 ms마다 합치고 정지한 pointer도 200 ms마다 접촉을 갱신합니다.
`pointercancel`, capture 상실, tab 숨김, window blur, 페이지 이탈은 C로 끝납니다.
화면 밖 release 좌표는 화면 경계로 제한합니다.

**현재 Web control owner만** 입력할 수 있습니다. 기존 lease/revision과 transition
검사를 거치며, release·heartbeat 만료·disconnect·takeover 시작에 queued gesture를
취소합니다. 이전 owner의 지연 요청이나 C가 새 owner를 취소하지 못합니다.
Virtual Touch는 차량 ARM이나 STM_TEST 상태 수신을 요구하지 않습니다. 기존 차량
명령·takeover의 disarm/reset/heartbeat 계약은 그대로입니다.

`VTOUCH READY`는 Web online, UNO serial open, 1초 미만의 지원 LCD report, 현재
control owner, 인계 중이 아닌 조건에서만 표시합니다. viewer=`VIEW ONLY`, stale=
`UNO STALE`, 구버전 firmware=`VTOUCH UNAVAILABLE`입니다. queue 등록 응답은 페이지
적용 확인이 아닙니다. **React는 페이지를 미리 변경하지 않습니다.** UNO가 LCD drawing을
마친 뒤 보고한 `page`와 표시값만 native renderer에 전달합니다.

이 기능은 **dashboard navigation only**입니다. 기어·mode·브레이크·E-stop·가속·actuator
command나 L/R 차량 기능 UI를 추가하지 않습니다. STM_A/B/TEST firmware, Dashboard CAN,
micro-ROS RuntimeStats 및 vehicle command semantics는 변경하지 않습니다.

공통 로직 회귀 시험:

```bash
cmake -S firmware/UNO_R4_DASHBOARD/dashboard_preview/native \
  -B firmware/UNO_R4_DASHBOARD/build/host -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/UNO_R4_DASHBOARD/build/host --parallel 2
ctest --test-dir firmware/UNO_R4_DASHBOARD/build/host --output-on-failure
```

`touch_navigation_test`는 양쪽 source의 모든 이동/경계, 정확히 60 px,
release 전 이동 금지, tap/vertical/multitouch/invalid/finger/source cancel,
physical priority/guard, virtual cancel/timeout, malformed VT를 확인합니다.
Python serial PTY 시험과 실제 Chrome PointerEvent 시험은
[Web HIL 검증](../../software/src/kaiev26_hil/WEB_CONSOLE.md#검증)을 따릅니다.

#### 2026-09-10 구현·자동 검증 결과

시작 checkout은 깨끗한 `main`, HEAD `14c38b1ff87362e27db9522a5c0a9d9b2ce88a8e`였습니다.
원격 fetch 후 `origin/Shin`도 같은 HEAD임을 확인하고 `Shin`으로 전환해 작업했습니다.
Arduino UNO R4 Boards core **1.6.0**, `arduino:renesas_uno:minima`로 이전 touch/swipe
코드와 이번 코드를 각각 직접 compile했습니다.

| 항목 | 이전 touch/swipe | Virtual Touch v1 | 증가 |
|---|---:|---:|---:|
| Flash / 262,144 B | 226,124 B (86.26%) | 226,840 B (86.53%) | 716 B |
| Global RAM / 32,768 B | 12,152 B (37.08%) | 12,160 B (37.11%) | 8 B |

- 공통 C++ gesture/navigation/VT parser CTest: **PASS**.
- 기존 테스트를 포함한 `kaiev26_hil` 전체: **131 passed**, 67.03초.
- 첫 test/build 이후 refinement: 유효하지 않은 Web 좌표 입력도 진행 중 gesture를
  취소하도록 보완하고, USB 초기 명령의 partial write를 실패 처리했습니다.
- refinement 후 관련 serial/Web/browser 시험: **17 passed**, 11.62초.
  실제 Chrome에서 1×/2×/responsive mapping, 500개 MOVE burst의 coalescing,
  pointercancel, 화면 밖 release/capture, 두 번째 손가락 취소, owner takeover,
  stale UNO 입력 거부 및 **새 보고 전 mirror 불변**을 확인했습니다.
- PTY serial 시험에서 reconnect의 C 선행·이전 DOWN/MOVE 폐기·이전 generation 거부와
  터치 송신 중 HIL_LCD 수신/native PNG 생성을 확인했습니다.
- TypeScript/Vite production build, native renderer build/install,
  `colcon build --symlink-install --packages-select kaiev26_hil`: **PASS**.
- 기존 Starlette/httpx·anyio deprecation warning 2개가 있었으며 실패는 없었습니다.

**이번 변경의 실제 보드 flash/HIL은 아직 수행하지 않았습니다.** `/dev/arduino_hil`의
USB serial `3114101259323437A93033334B57305C`까지 재확인했으며, 현재 대상 LCD와
구동계·액추에이터 분리 bench 상태는 사용자 확인 대기입니다. 테스트의 PTY는 명시된
보고값을 공급하는 peer이며 실제 FT6336U, CAN 또는 LCD 패널을 모사·검증하지 않습니다.
따라서 위 이전 physical swipe 실물 결과를 이번 virtual 입력의 실물 통과로 간주하지 않습니다.

남은 실보드 확인은 Web 오른쪽/왼쪽/왼쪽/오른쪽 drag에 대한 실제 패널의
L→Dashboard→R→Dashboard, 그 뒤 UNO page report와 Web mirror의 일치,
1~2분 반복 중 CAN `seq` 증가·CAN LOST 없음·`touch_errors` 증가 없음·USB 보고 지속입니다.
물리 패널 표시, CAN continuity 및 LCD/CAN latency 악화 여부는 아직 미검증입니다.

### LCD 초기화

[`st7796_lcd.cpp`](st7796_lcd.cpp)는 기존 STM_B와 같은 MSP4030 vendor command와
landscape `MADCTL=0x28`, RGB565를 사용합니다. Arduino SPI는 mode 3, MSB first,
12 MHz로 설정했습니다. ST7796S의 4-SPI write cycle 최소값 66 ns(약 15.15 MHz
상한)보다 낮고, UNO와 LCD 사이의 짧은 로컬 harness를 전제로 합니다. 초기화 순서는
다음과 같습니다.

1. backlight OFF
2. reset LOW 100 ms, HIGH 50 ms
3. MSP4030 vendor sequence
4. `NORON`, `SLPOUT`, 120 ms, `DISPON`, 20 ms
5. CAN controller 시작
6. 현재 mode의 static layout과 state를 전체 렌더
7. backlight ON
8. 완성된 CAN snapshot마다 변경된 속도·track·gauge·상태 영역만 갱신

Arduino SPI API는 panel readback을 사용하지 않으므로 Serial의
`LCD dashboard UI rendered`는 command와 pixel 전송 함수가 끝났다는 뜻이지 실제
panel 표시 증거는 아닙니다. STM_A가 연결되지 않은 bench에서는 다음 화면이 정상입니다.

- top badge: `MODE --`, `--`, `A-B LOST`, `SAFE`
- HV BUS, SPEED, MOTOR, CTRL, FAULT 값: `--`
- bottom bar: `STM A COMM LOST`, `VALUES STALE`

Dashboard CAN까지 끊기거나 alive가 진행하지 않으면 bottom bar는 `DASH CAN LOST`,
`VALUES STALE`로 바뀝니다. LOST 상태에서 controller 재초기화가 성공해도 fresh complete
snapshot이 들어오기 전에는 stale 화면을 유지합니다.
정상 bottom bar는 MANUAL에서 `PEDAL | TRACT | GNSS`, AUTO에서
`CMD | TRACT | GNSS` 세 칸으로 표시합니다. GNSS 칸은 `RTK FIXED`, `GNSS FIX`,
`GNSS NOFIX`, `GNSS --` 중 하나이며, 앞의 전체 경고가 활성화되면 경고 문구가
bottom bar를 우선 점유합니다.
[`dashboard_ui.cpp`](dashboard_ui.cpp)가 기존 STM_B `display_app.c`의 좌표, badge 색상,
표시 우선순위와 5x7 font rendering을 담당합니다.

## References

- [Sitronix ST7796S Datasheet V1.0](<../../docs/hardware/ST7796S-Sitronix (display).pdf>)
- [Arduino UNO R4 Minima documentation](https://docs.arduino.cc/hardware/uno-r4-minima)
- [Arduino Renesas Minima CAN pin mapping](https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/MINIMA/pins_arduino.h)
- [Arduino_CAN non-blocking receive example](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_CAN/examples/CANRead/CANRead.ino)
- [NXP TJA1050 datasheet](https://www.nxp.com/docs/en/data-sheet/TJA1050.pdf)
