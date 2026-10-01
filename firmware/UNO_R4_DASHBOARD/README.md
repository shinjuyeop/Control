# UNO_R4_DASHBOARD

Arduino UNO R4 Minima의 내부 CAN controller로 STM_B dashboard protocol v1을 받아
MSP4030/ST7796S에 표시하는 firmware입니다. MCP2515는 사용하지 않습니다. RTOS 없이
Arduino `setup()`/`loop()` 구조와 `Arduino_CAN` API를 사용합니다.

MANUAL은 기존 KTM 스타일 화면을 유지합니다. AUTO는 상단 health 8개, 방위, KEYA
motor-axis 조향각, 실제/목표 속도와 최근 5초 조향 전류를 표시합니다. 두 화면 모두
480픽셀 scanline과 압축된 정적 asset을 사용하며 전체 framebuffer를 만들지 않습니다.

```text
UNO_R4_DASHBOARD.ino       Arduino sketch entry point
src/*.cpp                  firmware implementation
src/include/               hand-written firmware headers
src/assets/                generated raster and font headers
simulator/                 PC simulator and native host runtime
tools/                     asset generation and timing calibration
README.md                  usage and hardware contract
```

`src/`는 Arduino가 재귀 컴파일하므로 PC 전용 C++은 `simulator/native/`에 둡니다.
`.ino`의 `src/include/...`와 `src` 구현의 `include/...`·`assets/...` 상대 include를
명시하므로 추가 include 빌드 옵션 없이 기존 Arduino CLI 명령을 사용합니다.
[Arduino CLI sketch specification](https://docs.arduino.cc/arduino-cli/sketch-specification/)을
따르는 구조입니다.

## Local UI preview

`python3 -m simulator`는 CAN/UNO/LCD hardware 없이 실제
`.ino` 수신 loop, C++ page renderer와 ST7796 SPI driver를 host 공유라이브러리로
실행합니다. MANUAL, AUTO, startup은 SPI byte가 해당 가상시각까지 전송한 RGB565
pixel만 보입니다.
Python 3, g++, Tkinter, Pillow와 PyYAML이 필요합니다.

- `MANUAL_TFT`: 계기판 reference를 옮긴 2중 BUS/PHASE arc, 곡선형 F/R, speed,
  5열 PED/HV/MTR/CTRL/LV gauge, 한 줄 상태 표시와 작은 자동 dino runner
- `AUTO_GT3`: 480x320 전체를 사용하는 AUTO cockpit. 상단 2행 상태 박스,
  좌측 차량 방위·목표 방향, 우측 실제·목표 조향각, 중앙 상태 메시지·속도,
  하단 조향 상태·전류·최근 추이로 구성합니다.

화면의 주요 영역은 고정입니다. 상단은 120x28 박스 4개씩 2행이며, 하단 전류 영역은
좌우·아래 끝까지 사용합니다. 중앙 상태 문구는 속도 위의 흰 배경에 표시합니다.
수동/AUTO 스위치는 별도 대기 없이 한 행씩 전송합니다. 한 서비스에서 최대 4행을
처리하며, 2ms가 지나면 현재 행을 마친 뒤 수신 처리로 복귀합니다. MANUAL→AUTO는
아래에서 위, AUTO→MANUAL은 위에서 아래로 진행하고 중간 반전은 실제 전송한
경계에서 방향만 바꿉니다.

## AUTO 상태 표시의 의미와 소스

PC 모의 입력의 상태 변환은 `simulator/backend.py`에 있고, 실차용 생산자는
`vehicle_state_publisher/auto_dashboard_summary.py`입니다. 생산자는 토픽별 실제 도착 시각을
monotonic clock으로 기록해 freshness를 판정하고 `/dashboard/auto_status`를 10 Hz로
발행합니다. 명령 두절과 모터 피드백 유효성은 액추에이터가 보고하는 기존 필드를
따릅니다. Preview의 입력은 계속 모의 데이터이며 ROS에 연결되지 않습니다.

GNSS는 수동·자동 모두 표시합니다. 수동은 STM_B가 `/gnss/fix`에서 만든 native
STATUS frame의 `gnss_state`를 사용해 `FIXED/FIX/NOFIX/--`를 표시합니다.
현재 UM980 driver는 RTK fixed일 때 NavSatStatus 2를 발행합니다.
자동은 `/gnss/status`의 `FIXED/FLOAT/FIX/NO FIX`를 사용하고, AUTO 요약이 없거나
GPS가 WAIT일 때 fresh native STATUS의 GNSS를 사용합니다. CAN 자체가 끊기면
수신 전의 FIXED를 유지하지 않고 미확인 상태로 표시합니다.

실차 launch는 `vehicle_state_publisher`의 `auto_dashboard_summary`와
`diagnostics_publisher`를 실행합니다. 현재 STM_A/B LPUART firmware와 맞는
Agent 기본 baud는 460800입니다. AUTO 상세는 STM_B의 `0x325`~`0x328` 중계와
PC summary가 모두 필요합니다.

| 박스 | 소스 | 표시 의미 |
|---|---|---|
| GPS | `/gnss/status.fix_mode` | FIXED 초록 / FLOAT 노랑 / 일반 FIX 주황 / NO FIX 빨강 |
| LIDAR | `/ouster/points` | 실제 PointCloud2 메시지 수신 / 중단 / 미수신 |
| CAMERA L | `/perception/camera/left/source/image_raw/compressed` | 좌측 USB 카메라 이미지 수신 상태 |
| CAMERA R | `/perception/camera/right/source/image_raw/compressed` | 우측 USB 카메라 이미지 수신 상태 |
| DRIVE | `/drive/status` (STM_A) | 상태 수신, 명령 유효성, EZkontrol Part I/II·fault·RUN 피드백을 함께 판단 |
| STEER | `/steering/status` (PC lateral_ctrl + KEYA CAN) | 상태·명령·모드 freshness, CAN·위치 피드백, motor/tracking fault, 출력 허용·대기 이유 |
| E-STOP | 기존 STM_B dashboard CAN의 물리 E-stop 상태 | ON / OFF / dashboard 수신 두절 시 미확인 |
| BRAKE CMD | `/brake/status` (STM_B) | STM_B가 적용한 제동 출력 요청과 명령 무효 상태 |

DRIVE는 STM_A 상태가 계속 도착해도 모터 피드백이 없으면 `NO FEEDBACK`, RUN 요청과
유효 피드백이 다르면 `RUN MISMATCH`, 명령이 끊기면 `CMD LOST`입니다. 일치하는
정지 요청은 `HOLD`이고 고장과 구분합니다. STEER는 위치 피드백이 살아 있어도
reported tracking fault가 있으면 `TRACK FAULT`입니다. ACTIVE는 유효한 피드백과
출력 요청을 동반한 제어 상태이며 모터의 독립적인 Enable 확인을 뜻하지 않습니다.

브레이크는 실제 작동 피드백이 없습니다. `BRAKE CMD ON`은
`BrakeActuatorStatus.brake_commanded`에 따른 소프트웨어 출력 상태입니다.
`STOP_REASON_COMMAND_INVALID`는 `CMD LOST`로 표시하고, `actual_valid=false`를
제동 성공으로 바꾸지 않습니다. 물리 E-stop은 desktop 요약이 끊겨도 dashboard CAN이
살아 있으면 표시합니다. 실제 제동·구동 안전 동작은 계기판 표시가 제어하지 않습니다.

현재 UM980 드라이버에서 RTK FIXED는 NavSatFix status 2, NO FIX는 -1입니다.
FLOAT와 일반 FIX는 모두 status 0일 수 있어 `fix_mode`로 구분합니다.
LiDAR·카메라 RX는 토픽 입력의 수신이며 인지 결과의 정확도를 뜻하지 않습니다.

## AUTO 방위·조향 계기

배경은 순백색이고 실제값은 파랑, 목표값은 보라입니다. 좌측은 북쪽 0°, 시계방향의
나침반입니다. 파란 차량 실루엣이 현재 방위, 보라색 화살표가 목표 방향입니다.
실차 연동 시 유효한 ENU yaw를 `(90 - degrees(yaw)) % 360`으로 변환하고 목표 방향도
같은 좌표 기준으로 변환합니다. 현재 방위는 `/localization/odometry`, 목표 방위는
유효한 `/planning/route_context.heading_error + current yaw`입니다. 미수신인
방위·바늘은 그리지 않습니다.

우측은 KEYA encoder 영점 기준의 실제 스티어링 모터/스티어링휠 각도이며 좌회전
양수입니다. ACTUAL은 `/steering/status.motor_position_count`를 그대로 전달한 뒤
`count * 360 / 10000`으로 표시합니다. TARGET은 `target_wheel_angle_rad`를
`SteeringAngleConverter::equivalentDegToCount()`로 바꾼
`/steering/target_motor_position_count`이며, 같은 식으로 motor degree를 표시합니다.
따라서 중앙 등가 바퀴각과 원시 motor angle을 직접 겹치지 않습니다. 활성 권위에는
motor shaft와 steering wheel 사이의 별도 배율이 없어 임의 배율을 추가하지 않았습니다.
CAN에는 `int32` count를 넣어 표시 전송에서 회전수나 encoder 영점을 잃지 않습니다.

하단은 조향 전류, 출력 요청, 피드백 상태, 최근 5초 수신 전류의 추이·최대값입니다.
KEYA 전류의 현재 정수 A 분해능을 따르며 0–25 A는 그래프 눈금입니다. 전류 0 A만으로
정상·고장을 판단하거나 그래프 눈금으로 과전류를 판정하지 않습니다.

`AUTO` 탭에서 정상 상태와 모터 피드백 단절·RUN 불일치·추종 fault·센서 입력 중단을
선택할 수 있습니다. `auto_low_speed`·`auto_launch`의 저속 Enable 대기와
`auto_steering_trip`의 20 A → 0 A·motor fault는 설명을 위한 모의 시나리오이며
실제 bag 재생이나 제어 로직이 아닙니다.

## MANUAL dino runner

MANUAL의 우측 세로 gauge 아래, 하단 상태 panel 위 `x=281..470`, `y=206..257`
여백에는 흐린 회색 pixel dino와 지면·장애물을 표시합니다. 실제 유효 차량 속도의
절댓값을 누적해 발 모양, 지면 점선과 장애물 이동을 함께 진행시키므로 속도가 0이면
모두 멈춥니다. Dashboard CAN이나 vehicle frame이 stale이면 이전 속도로 계속
움직이지 않고 현재 위치에서 멈춥니다.

장애물이 공룡 앞에 도달하면 이동 거리 기준 포물선으로 자동 점프합니다. 입력, 점수,
충돌 실패, game over와 menu는 없습니다. 기존 MANUAL 경고가 활성화되면 이 영역의
애니메이션을 숨기고 경고를 우선합니다. Firmware는 이 영역만 4 scanline씩 갱신해
각 chunk 사이에 CAN 수신으로 복귀합니다.

실제 Arduino firmware는 MANUAL에서 `MANUAL_TFT`의 곡선형 BUS/PHASE track,
우측 5열 gauge와 하단 상태줄을 표시하고, AUTO에서는 선택된 `AUTO_GT3` cockpit을
사용합니다. [`manual_tft_assets.h`](src/assets/manual_tft_assets.h)는 정적 레이아웃
생성기가 만든 정적 raster, 곡선 antialias/progress mask와 Blender Pro glyph입니다.
[`startup_tft_assets.h`](src/assets/startup_tft_assets.h)는
고정 KAI outline 8-bit alpha와 16단계 haze mask를 행별로 저장합니다. 직접
편집하지 않습니다. [`auto_tft_assets.h`](src/assets/auto_tft_assets.h)는 AUTO의 흰 배경, 고정
label과 계기 눈금을 담습니다. 둘 다 다음 명령으로 다시 생성합니다.

```bash
python3 -m tools.generate_assets
```

Flash를 줄이기 위해 asset은 다음 형식으로 저장하며, 디코더는
[`asset_codec.h`](src/include/asset_codec.h)와 각 UI 파일에 있습니다.

| Asset | 형식 | 손실 |
|---|---|---|
| MANUAL/AUTO 정적 배경 | RGB565 palette index. 행마다 같은 색 연속 또는 윗줄 유지 명령이며 8행마다 기준 행 | 없음 |
| BUS/PHASE track | 0/255/임의 값 연속·원본 값 나열로 된 alpha 구간, 진행도는 첫 값과 픽셀당 +0/+1 bit | 없음 |
| Startup outline/haze | outline은 track과 같은 alpha 구간, haze는 1 byte에 단계 4 bit·길이 4 bit | 없음 |
| Glyph | 폰트별 16단계 alpha 표를 쓰는 4-bit 값. 두 값씩 묶거나 0/255 연속을 줄인 형식 중 작은 쪽 | 4-bit alpha |

Glyph의 16단계 중 0과 255는 원래 값 그대로이고, 나머지 14단계는 폰트별 alpha
분포에서 제곱 오차가 가장 작도록 고릅니다. 전체 시뮬레이터 화면 155장 기준으로 바뀐
픽셀은 1.02%이며 그중 99.5%는 RGB565 한 단계 이내입니다.

GUI를 열면 검은 화면에서 KAI symbol의 흰 outline이 약한 haze와 함께 나타나는 1.4초
startup sequence가 한 번 재생됩니다. `Replay startup`으로 다시 볼 수 있으며 symbol은
이동하거나 채워지지 않고 TEAM K.A.I. 문구도 표시하지 않습니다. 자산을 재생성할 때는
설치된 Blender Pro Medium/Bold/Heavy를 label/value/gear·speed 역할에 나눠 사용하고,
폰트가 없는 환경에서는 기존 condensed font로 fallback합니다. 실행 중 펌웨어와
시뮬레이터는 생성된 동일 glyph를 사용합니다.

수동 화면의 BUS/PHASE track은 수직 구간, 대각선 구간, 수평 구간과 두 전이곡선을
고정 antialias mask로 한 번만 만들고 그 안에서 진행 길이만 갱신합니다. 따라서 값이
변해도 곡선 외곽 픽셀이 프레임마다 다시 만들어지지 않습니다.

실제 화면 동작은 C++ 펌웨어가 담당하고, Python 앱은 입력과 창·PNG 출력을 담당합니다.
`simulator/app.py`는 창·PNG 출력, `simulator/model.py`와 `simulator/auto_model.py`는
가상 입력 시나리오, `simulator/backend.py`는 가상 CAN 입력과 RGB565 화면 연결을
담당합니다. `simulator/native/`는 Arduino/CAN/SPI 호환 실행부이고 시간 프로파일은
`simulator/timing_profile.json`에 둡니다. 펌웨어 자산은 `tools/generate_assets.py`와
`tools/manual_layout.py`·`tools/auto_layout.py`가 생성하며, 보드 계측·시간 보정은
`tools/calibrate.py`에서 실행합니다.

```bash
cd firmware/UNO_R4_DASHBOARD
python3 -m simulator
```

LV 5열 화면은 생산 C++ renderer에 구현되어 있습니다. 네 게이지와 속도/단위를
26px 왼쪽으로 옮기고, 원래 CTRL 자리 (`x=454..473`, 값 중심 `x=463`)에 LV를
표시합니다. 막대 폭 20px, 간격 6px와 기존 glyph를 유지합니다. 원래 4열 static
asset은 보존하고 scanline 디코딩 시 위치를 옮기며 LV 라벨은 기존 L/V raster를
재사용합니다. 따라서 현재 머신에 없는 Blender Pro 파일을 대체 폰트로 재생성할
필요가 없습니다. LV 막대는 SOC, 숫자는 전압만 표시합니다.

`python3 -m simulator --manual-lv --lv-soc 73 --lv-voltage 13.3`은 실제 UNO CAN
수신기와 렌더러에 LV 모의 프레임을 입력합니다. `--lv-soc -1`은 SOC 미설정,
음수 전압은 무효 전압 표시입니다. SOC 미설정은 빈 막대, 유효 전압은 계속 표시하며
CAN 또는 LV status가 끊기면 `--.-V`와 빈 막대를 표시합니다. 기존 시간 프로파일은
변경 전 측정값으로, 추가 LV 렌더링의 실제 보드 시간은 별도 측정 대상입니다.

오른쪽 panel에서 scenario, 10/20 Hz telemetry cadence, MANUAL/AUTO, FWD/REV,
validity, E-stop/brake, GNSS 네 상태와 motor fault 24종을 바꿀 수 있습니다. 실제 LCD 해상도의 PNG만
저장하려면 GUI의 `Save 480x320 PNG`를 사용하거나 다음처럼 실행합니다.

```bash
python3 -m simulator \
  --variant manual_tft --scenario load --time 6 --snapshot /tmp/dashboard.png
```

AUTO 화면을 바로 열려면:

```bash
python3 -m simulator --variant auto_gt3 --scenario auto_healthy
```

다른 상태는 `--scenario auto_rtk_float`, `auto_steering_lost`, `auto_host_lost`
등으로 지정하거나 GUI의 `AUTO` 탭에서 선택합니다. `--variant startup --time 0.7`
처럼 부팅 중간 시각도 저장할 수 있습니다. `simulator/timing_profile.json`의
기본 시간 프로파일은 보드 실측값을 사용하며, SPI 전송시간과 CPU 상태 준비·행 합성·전송
부대비용을 가상 시각에 반영합니다. 다른 측정값은 `--timing-profile`로 지정합니다.
GUI는 monotonic wall/virtual origin의 절대시각으로 native clock을 진행하고 60 Hz
표시 deadline을 사용합니다. 10/20 Hz CAN sample tick은 GUI callback과 독립적으로
모두 공급되며 Pause는 시나리오 시간만 멈춥니다.

일반 프리뷰는 PC의 가상 CAN 입력을 사용하며 실제 CAN·serial 장치를 열지 않습니다.
아래 보정 도구는 사용자가 지정한 보드의 USB serial을 사용합니다.

## Web Virtual LCD와 UNO 상태 보고

`dashboard_preview/native`는 현재 `src/`의 MANUAL·AUTO C++ UI를 Web HIL용 shared
library로 빌드합니다. UNO에서 `HIL_LCD ON`을 받으면 200 ms마다 실제 적용한 native와
AUTO 표시값을 USB serial로 보고하며, Web HIL은 같은 C++ UI로 480×320 화면을
재구성합니다. 렌더러는 UNO가 보고한 page를 따르며, mode 전환 도중 수신한 mode bit로
화면을 앞서 전환하지 않습니다. 이 경로는 표시 전용이며 CAN이나 차량 명령을 만들지 않습니다.

```bash
cmake -S firmware/UNO_R4_DASHBOARD/dashboard_preview/native \
  -B firmware/UNO_R4_DASHBOARD/build/host -DCMAKE_BUILD_TYPE=Release
cmake --build firmware/UNO_R4_DASHBOARD/build/host --parallel 2
cmake --install firmware/UNO_R4_DASHBOARD/build/host \
  --prefix software/src/vehicle_console
```

보고 protocol v1의 기존 native `values`는 유지하고 `auto` 배열과
`lv=[flags, voltage_decivolt, soc_percent]`를 확장 필드로 제공합니다. Web HIL은
확장 필드가 없는 기존 UNO 보고도 해당 값이 미수신인 상태로 처리합니다.
USB 보고 중에는 일반 사람용 serial 로그를 잠시 보류하고, `HIL_LCD OFF` 또는 USB
연결 해제 뒤 다시 출력합니다. USB 보고 단절과 dashboard CAN LOST는 별도 상태입니다.

MSP4031 physical touch와 Web Virtual Touch는 같은 navigation을 사용합니다.
`VT D/M/U x y`와 `VT C`는 480×320 화면의 이동·취소 입력이며 차량 명령을 만들지
않습니다. `HIL_LCD`에는 `vtouch=1`과 `touch_errors`도 포함됩니다.
스와이프로 L/R 화면으로 이동한 동안에도 최신 CAN 상태를 저장하고,
복귀 시 최신 차량 모드와 GNSS 상태로 계기판을 다시 그립니다.

## CAN contract

| 항목 | 값 |
|---|---|
| Board | Arduino UNO R4 Minima |
| Controller | Renesas RA4M1 internal CAN |
| Controller pins | `D4=CANTX0`, `D5=CANRX0` |
| Transceiver | MCP2551 |
| Frame | Classical CAN, standard data frame, DLC 8 |
| Bitrate | 250 kbps |
| Native IDs | `0x321` STATUS, `0x322` POWER, `0x323` THERMAL, `0x324` FAULT |
| AUTO IDs | `0x325` HEALTH, `0x326` MOTION, `0x327` STEER ACTUAL, `0x328` STEER TARGET |
| Protocol | version 1, little-endian, shared `uint8` alive |
| Snapshot | native 4개와 AUTO 4개를 각각 같은 version/alive로 조립 |
| GNSS | STATUS byte 5: 0 unavailable, 1 no fix, 2 fix, 3 RTK fixed |
| CAN LOST | 새 native alive의 완전한 4-frame snapshot이 300 ms 없음 |
| CAN recovery | LOST 중 1초마다 `CAN.end()`/`CAN.begin()` 재시도 |
| Serial Monitor | 115200 baud |

`loop()`은 `CAN.available()`을 확인한 뒤 준비된 frame만 `CAN.read()`하므로 CAN 수신을
기다리며 blocking하지 않습니다. ID, DLC 또는 version이 다른 frame은 무시합니다.
같은 native alive가 반복되면 duplicate로 세고 link health를 갱신하지 않습니다. 완성
snapshot은 최신 값으로 합친 뒤 200 ms cadence에서 고정 buffer로 만듭니다. 모든
snapshot/event/timing 출력은 매 loop `availableForWrite()`가 허용한 byte만
`Serial.write()`하고 남은 offset을 다음 loop에서 이어 보냅니다.
Part II age가 1 s에 도달하면 Part II valid bit가 내려가 MANUAL 화면의
`THERMAL DATA LOST`와 serial의 `INVALID`로 표시됩니다. AUTO 화면은
`DRIVE MOTOR FEEDBACK LOST`로 표시합니다. 200 ms 중간 경고는 없습니다.
정확한 byte layout과 source type/unit은
[`../STM_B/README.md`](../STM_B/README.md#85-dashboard-snapshot과-wire-단위)가 canonical contract입니다.

PC 생산자는 고정 25-byte `std_msgs/msg/UInt8MultiArray`를
`/dashboard/auto_status`로 발행합니다. byte 0은 summary version이고 나머지 24 byte는
각 AUTO CAN frame의 data byte 2–7 네 묶음입니다. STM_B는 이 ROS message가 300 ms
이내 도착했을 때만 `0x325`의 `HOST_VALID` bit를 세웁니다. 이 bit가 꺼진 상태에서도
`0x324`의 byte5는 LV flags(bit0=present, bit1=measurement valid, bit2=SOC known),
byte6은 전압(0.1 V), byte7은 SOC(0–100%, unknown=255)입니다. LV는 STM_B가
RP2350 UART에서 직접 받은 값이고 PC/ROS 없이도 전송합니다. UART status가
1초 동안 없으면 STM_B가 flags를 지우며, CAN 전체가 300ms 끊기면 UNO가
LV를 포함한 stale 값을 숨깁니다. 이전 STM_B의 zero flags는 LV 미수신입니다.

STM_B의 native E-stop과 0x321–0x324는 계속 전송되므로 UNO는 `AUTO DATA LOST`와
`DASH CAN LOST`를 구분합니다.

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

Flash/RAM 사용량은 해당 소스를 빌드한 결과로 확인합니다. UNO R4 Minima의 이 구성에서
실제 앱 Flash 영역은 245,760바이트입니다. 선택적 계측 빌드는 필요한 행·전송 통계를
RAM에 모으고 실행 구간 종료 후 출력하며, 정상 빌드에는 계측 집계가 포함되지 않습니다.

## Display-only board capture

`DASHBOARD_BENCH_INPUT` 빌드는 실제 CAN controller를 입력에 섞지 않고 USB serial의
시간표 레코드를 기존 `accept_dashboard_frame()`에 넣습니다. `B/C/E` 제어와 timing
응답은 모두 bounded serial queue를 사용하며 차량 CAN으로 송신하지 않습니다.

```bash
python3 -m tools.calibrate \
  --board-capture manual --board-host kai \
  --board-port /dev/serial/by-id/usb-Arduino_UNO_R4_Minima_31142C1F593234372E0233334B57305C-if00 \
  --duration 10 --scenario road --capture-output manual-road.json
```

`startup`, `manual`, `auto`를 각각 수집할 수 있고 `--held-steps`는 준비 구간 1.2초 뒤
1.7초 간격의 단계 입력을 사용합니다. capture JSON에는 보드 raw timing, 동일 CAN 시간표와 같은
profile로 재생한 host event가 함께 저장됩니다. 외부 후보 profile은
`--timing-profile`로 지정합니다.

보정 도구의 전체 옵션은 `python3 -m tools.calibrate --help`로 확인합니다.

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

중앙은 kgm의 `MANUAL_TFT`/`AUTO_GT3` 화면을 사용합니다.
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
[Web HIL 검증](../../software/src/vehicle_console/HIL_WEB.md#검증)을 따릅니다.

[`st7796_lcd.cpp`](src/st7796_lcd.cpp)는 기존 STM_B와 같은 MSP4030 vendor command와
landscape `MADCTL=0x28`, RGB565를 사용합니다. Arduino SPI는 mode 3, MSB first,
12 MHz로 설정했습니다. ST7796S의 4-SPI write cycle 최소값 66 ns(약 15.15 MHz
상한)보다 낮고, UNO와 LCD 사이의 짧은 로컬 harness를 전제로 합니다. 초기화 순서는
다음과 같습니다.

1. backlight OFF
2. reset LOW 100 ms, HIGH 50 ms
3. MSP4030 vendor sequence
4. `NORON`, `SLPOUT`, 120 ms, `DISPON`, 20 ms
5. 현재 mode의 static layout과 state를 전체 렌더
6. CAN controller 시작
7. backlight ON
8. 완성된 CAN snapshot마다 변경된 영역만 갱신. AUTO dirty region은 4행씩 전송

MANUAL→AUTO는 아래에서 위로, AUTO→MANUAL은 위에서 아래로 16행씩 reveal합니다.
보통 18 ms 간격 20단계이므로 목표는 약 360 ms입니다. E-stop, dashboard CAN 두절,
조향 통신/피드백/fault/추종 fault가 들어오면 간격을 없애되 한 단계 뒤 CAN 수신으로
돌아옵니다. ST7796S hardware scroll은 사용하지 않아 landscape 물리축을 가정하지
않습니다.

Arduino SPI API는 panel readback을 사용하지 않으므로 Serial의
`LCD dashboard UI rendered`는 command와 pixel 전송 함수가 끝났다는 뜻이지 실제
panel 표시 증거는 아닙니다. STM_A가 연결되지 않은 bench에서는 다음 화면이 정상입니다.

- top badge: `MODE --`, `--`, `A-B LOST`, `SAFE`
- HV BUS, SPEED, MOTOR, CTRL, FAULT 값: `--`
- bottom bar: `STM A COMM LOST`, `VALUES STALE`

Dashboard CAN까지 끊기거나 native alive가 진행하지 않으면 현재 mode 화면에서
`DASH CAN LOST`를 표시합니다. LOST 상태에서 controller 재초기화가 성공해도 fresh
native snapshot이 들어오기 전에는 stale 상태를 유지합니다. MANUAL의 상태줄 동작은
기존 그대로이고, AUTO는 경고가 생겨도 상단 8개 박스·중앙 상태·하단 조향 정보의
좌표와 크기를 바꾸지 않습니다. [`dashboard_ui.cpp`](src/dashboard_ui.cpp)는 모드/와이프를,
[`auto_tft_ui.cpp`](src/auto_tft_ui.cpp)는 AUTO 합성과 incremental rendering을 담당합니다.

## References

- [Sitronix ST7796S Datasheet V1.0](<../../docs/hardware/ST7796S-Sitronix (display).pdf>)
- [Arduino UNO R4 Minima documentation](https://docs.arduino.cc/hardware/uno-r4-minima)
- [Arduino Renesas Minima CAN pin mapping](https://github.com/arduino/ArduinoCore-renesas/blob/main/variants/MINIMA/pins_arduino.h)
- [Arduino_CAN non-blocking receive example](https://github.com/arduino/ArduinoCore-renesas/blob/main/libraries/Arduino_CAN/examples/CANRead/CANRead.ino)
- [Microchip MCP2551 datasheet](https://www.microchip.com/content/dam/mchp/documents/APID/ProductDocuments/DataSheets/20001667G.pdf)
