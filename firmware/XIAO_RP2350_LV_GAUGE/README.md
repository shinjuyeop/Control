# XIAO RP2350 LV battery gauge

이 디렉터리는 XIAO RP2350이 INA228 측정, 전력/Ah/Wh 적산, SOC와 상태 저장을
독립적으로 수행하는 LV 적산계 펌웨어와 정비 TUI입니다. XIAO는 ROS나 PC가 없어도
측정·적산합니다. TUI 기본 경로는 STM_B micro-ROS↔UART4이며 XIAO USB는 정비용
fallback입니다. 두 경로는 같은 CRC 보호 payload와 transaction ID를 사용합니다.

## 현재 하드웨어 계약

- XIAO `3V3` → INA228 `VIN`, 공통 `GND`
- XIAO `D4/GPIO6` → INA228 `SDA`
- XIAO `D5/GPIO7` → INA228 `SCL`
- XIAO `D6/GPIO0/UART0_TX` → STM_B `PC11/CN7-2/UART4_RX`
- XIAO `D7/GPIO1/UART0_RX` ← STM_B `PC10/CN7-1/UART4_TX`
- XIAO의 `Wire1`을 사용하고 SDA/SCL을 코드에서 명시합니다.
- FL-2C 100 A/75 mV 외장 션트: `0.00075 ohm`
- 션트의 배터리 쪽(오른쪽 작은 나사) → `IN+`/`VIN+`
- 션트의 부하 쪽(왼쪽 작은 나사) → `IN-`/`VIN-`
- INA228 `VBUS`는 `VIN+`에 연결된 현재 점퍼 구성을 전제로 합니다.

이 방향에서는 배터리에서 부하로 흐르는 방전 전류가 양수이고, 부하에서 배터리로
흐르는 충전 전류가 음수입니다. 주전류는 INA228 모듈을 통과하지 않고 션트를
통과해야 하며, `IN+`/`IN-`는 션트 Kelvin sense 선이어야 합니다.

펌웨어는 INA228의 가능한 주소 `0x40`–`0x4F`를 시작할 때 검색하고, `MANUFACTURER_ID`
`0x5449`와 `DIEID` `0x228`을 확인한 주소를 사용합니다. INA228 설정은 다음과
같습니다.

- `ADCRANGE=0`: `+/-163.84 mV`
- `CURRENT_LSB=0.0002 A/bit`
- `SHUNT_CAL=1966` (`0.00075 ohm` 기준, 반올림)
- VBUS/VSHUNT/DIETEMP 연속 변환, 각 50 us, averaging 1회 (`ADC_CONFIG=0xF000`)

## 적산과 저장 의미

INA228 `CNVRF`가 선행된 변환 완료 결과만 유효 샘플로 사용합니다. 설정 레지스터가
`CONFIG=0x0000`, `ADC_CONFIG=0xF000`, `SHUNT_CAL=0x07AE`인지 매번 확인하고,
센서 reset·재설정·설정 불일치 뒤에는 새 변환 완료 결과를 받을 때까지 적산을
중단합니다. I2C ACK만으로 유효 계측이라고 판단하지 않습니다. I2C 응답 실패,
센서 재시작, XIAO 재시작 또는 연결 두절은 0 A 샘플로 바꾸지 않으며 해당 구간의
Ah/Wh와 계측 가동시간을 추가하지 않습니다. 트립 시간은 전원 OFF 시간을 포함하지
않는 `계측 가동 누적시간`입니다.

초기 상태는 `capacity=100 Ah`, `SOC=미설정`, `remaining=미설정`입니다. `f` 정비
명령은 실제 완충을 사용자가 확인한 뒤에만 SOC를 100%와 용량으로 설정합니다.
전압만으로 SOC를 보정하지 않습니다. `t`는 충전/방전 Ah·Wh와 계측 가동 누적시간만
지우며 SOC, 용량, 영점 보정을 변경하지 않습니다. `z`는 자동 실행되지 않고, 실제로
외부 부하와 충전기를 분리해 션트 전류가 0 A인 조건에서만 사용자가 확인한 뒤
변환 완료된 INA228 current raw count 64개를 수집해 평균을 저장합니다. 고정 시간마다
같은 값을 반복 읽어 독립 샘플로 세지 않으며 임의의 deadband는 사용하지 않습니다.

Ah는 연속 모드 INA228 `CHARGE`의 signed 증가량에 영점 보정을 적용해 적산합니다.
Wh는 같은 구간의 전하량에 구간 양끝 VBUS의 평균을 곱해 적산합니다.
`ΔWh = V평균 × ΔAh`이며, 같은 ΔAh 부호로 두 단위의 충전·방전을 함께 분류합니다.
전압은 수집 구간 양끝의 측정값으로 근사하므로 구간 내부의 빠른 전압 변화까지
복원하는 것은 아닙니다. 또한 한 수집 구간에서 방향이 바뀌면 CHARGE 증가량은
그 구간의 순전하입니다. 1 mWs 미만의 에너지는 방향별로 이월하여 저전류에서
반복 반올림으로 사라지지 않게 합니다. 이 소수 잔량은 RAM에만 있으며 재시작 시
방향별 1 mWs 미만이 소실될 수 있습니다.

INA228 재설정 뒤 첫 완료 샘플은 전하·전압 기준점만 만들고 적산하지 않습니다.
40비트 CHARGE rollover는 modulo delta로 처리합니다. 트립 초기화는 기존 `t`
명령으로 수행하며 펌웨어 업데이트나 모듈 교체만으로 누적값을 지우지 않습니다.
이전 펌웨어로 누적된 Ah·Wh는 과거 측정 이력이 없어 재계산할 수 없습니다.

상태는 60초마다 LittleFS journal에 저장하며 매 샘플 flash를 쓰지 않습니다. 84-byte
레코드를 두 파일에 번갈아 최대 512개씩 append하고, CRC/generation이 유효한 최신
레코드를 부팅 때 선택합니다. 새 파일을 완성하고 읽기 확인한 뒤에 활성 journal을
바꾸므로 쓰기 중 정전 시 이전 파일의 유효 상태가 남습니다. 정비 명령은 이 저장과
읽기 확인이 끝난 뒤에만 성공 ACK를 보냅니다. 동일 request ID를 재수신하면 이미
성공한 명령의 ACK만 재전송해 트립 초기화가 중복 적용되지 않습니다.

저장 주기 60초의 상한은 100 A가 계속 흐른다고 가정할 때
`100 A × 60 s / 3600 = 1.667 Ah`이며, 14.6 V에서 전력 적산은
`100 A × 14.6 V × 60 s / 3600 = 24.333 Wh`입니다.
계측 가동시간 손실 상한은 60초입니다. 논리 레코드 쓰기는 상태가 계속 변하면
연간 `365 × 24 × 60 = 525,600`회입니다. LittleFS의 실제 erase block 분산은
Arduino-Pico core의 LittleFS 설정은 page 256 byte, erase block 4096 byte이고,
256 KB FS에는 64개 erase block이 있습니다. 다만 현재 XIAO 보드 자료와 설치된
보드 패키지만으로 실제 flash 부품의 endurance를 확정할 수 없고, LittleFS logical
record 수를 erase cycle 수로 직접 환산할 수도 없습니다. 따라서 60초는 1.667 Ah와
24.333 Wh의 정전 손실을 제한하기 위한 선택이며 flash 수명 보증이나 차량 운용
승인값이 아닙니다. 실제 endurance와 전원 차단 복구는 장치 시험에서 확인해야 합니다.

## 빌드 환경

현재 준비한 공식 도구는 다음과 같습니다.

- Arduino CLI `1.5.1`
- Arduino-Pico core `rp2040:rp2040` `6.1.0`
- FQBN: `rp2040:rp2040:seeed_xiao_rp2350`
- Flash option: `flash=2097152_262144` (`2MB`, sketch `1792KB`, FS `256KB`)
- Arduino-Pico Boards Manager URL:
  `https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json`

공식 release checksum
`28a8e119c498a25607821c36cb2dc49e8463941b261a0d99091baa7bc692dd2b`와 대조한
Arduino CLI를 사용했습니다. 새 셸에서 `arduino-cli`가 PATH에 없다면 설치한 실행
파일의 디렉터리를 PATH에 추가합니다.

깨끗한 환경에서는 아래처럼 준비합니다.

```bash
arduino-cli config add board_manager.additional_urls \
  https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
arduino-cli core update-index
arduino-cli core install rp2040:rp2040
```

## 빌드

`Control` 저장소 루트에서 실행합니다.

```bash
arduino-cli compile \
  --fqbn rp2040:rp2040:seeed_xiao_rp2350 \
  --board-options flash=2097152_262144 \
  --warnings all \
  --export-binaries \
  --build-path firmware/XIAO_RP2350_LV_GAUGE/build \
  firmware/XIAO_RP2350_LV_GAUGE
```

빌드가 끝나면 `firmware/XIAO_RP2350_LV_GAUGE/build/`에
`XIAO_RP2350_LV_GAUGE.ino.bin`, `.elf`, `.map`, `.uf2`가 생깁니다. `build/`는
저장소에서 무시되는 생성물입니다.

## 고속 측정과 소프트웨어 LPF

ADC는 3항목을 150 us마다 변환합니다(명목 약 6.67 kHz). I2C는 fast-mode
400 kHz이며 고정 2 ms 대기를 없애 완료값을 연속 수집합니다. 실제 호스트 수집률은
레지스터 읽기 시간에 제한되며 ADC의 모든 변환을 수집한다는 뜻은 아닙니다.

RP2350은 수집 시각의 `micros()` 간격으로 1차 LPF를 적용합니다.
`alpha = 1 - exp(-dt / 100ms)`이며 차단 주파수는 약 1.59 Hz입니다.
기본 V/A/W와 플롯은 LPF 결과입니다. 진단의 VBUS/VDIFF는 raw 레지스터에서
환산한 미필터 값이고, VDIFF LPF·전류 LPF·영점 보정을 별도로 표시합니다.
연결 복구 후 첫 유효 샘플로 필터를 초기화합니다. 적산은 LPF 표시값이 아니라
INA228 CHARGE 증가량과 구간 전압으로 계산합니다.

## Rich TUI

TUI와 XIAO status는 10 Hz로 동작하고, INA228의 실제 완료 변환/적산 주기와
분리되어 있습니다. 기본 TUI는 STM_B가 발행하는 ROS payload에 연결하고, `--usb`
정비 모드는 USB by-id 경로를 찾아 직접 연결·재연결합니다. 화면은 host monotonic
수신시각 기준으로 최근 60초의
유효 V/A/W를 session memory에 제한 길이로 보관합니다. 연결 끊김·stale·무효 샘플은
0이나 직전값으로 메우지 않고 gap으로 표시합니다. 화면 갱신 중에도 키 입력을 받아
정비 명령을 전송하고, XIAO의 저장 완료 ACK 뒤에만 성공을 표시합니다.
Rich alternate-screen을 사용하지 않으므로 VS Code·원격 터미널에서도 전체 화면을
한 Panel로 잘못 확장하지 않고, 종료 후 마지막 화면이 터미널에 남습니다.

기본 화면에는 최근 60초 전압 V, 부호 있는 전류 A와 전력 W의 선 그래프가 있습니다.
왼쪽에는 값 눈금, 아래에는 −60s/−45s/−30s/−15s/now 시간 눈금을 표시하며,
전류·전력에는 0 기준선을 표시합니다. 같은 화면 열에 들어가는 샘플은 구간 평균으로
표현하여 겹친 점 대신 하나의 연결선을 그립니다. 연결이 끊긴 구간은 선으로 잇지
않습니다. 터미널 폭·높이에 따라 세 그래프를 가로나 세로로 배치합니다.

```bash
python3 -m pip install rich
source /opt/ros/humble/setup.zsh
source ~/KAI_ws/install/setup.zsh
python3 firmware/XIAO_RP2350_LV_GAUGE/tools/lv_battery_gauge_tui.py
```

기본 TUI는 `/lv_battery/link/status|command|ack`을 사용하므로 XIAO USB를 연결하지
않습니다. STM_B micro-ROS Agent는 실행 중이어야 합니다. XIAO 직접 정비가 필요할
때만 외부 `5V/VBUS`를 분리하고 다음처럼 USB fallback을 사용합니다.

```bash
python3 firmware/XIAO_RP2350_LV_GAUGE/tools/lv_battery_gauge_tui.py --usb
```

기본 화면은 연결/센서 상태, V/A/W와 충전·방전 방향, SOC/잔여 Ah, 충전·방전
트립 Ah/Wh, 계측 가동 누적시간을 표시합니다. `d`로 진단 화면을 열면 실제
INA228 ID, CONFIG/ADC_CONFIG/SHUNT_CAL, ADC 범위·변환 시간·평균화, 진단 플래그,
`INA die` 온도를 표시합니다. `INA die`는 배터리 온도가 아닙니다. `VBUS / VIN+`는
공통 GND(배터리 음극) 기준의 동일한 VBUS 실측값으로 표시하고, `VDIFF`는
`VIN+ - VIN−`인 VSHUNT 실측값으로 부호를 유지합니다. `VIN−`는
`VBUS - VDIFF`로 계산한 추정값이라고 명시합니다. 유효한 fresh 측정이 없으면
세 전압과 raw 값 모두 현재값처럼 표시하지 않습니다.

정비 키는 `f` 완충 기준점, `t` 트립 초기화, `z` 영점 보정, `c` 용량 설정입니다.
각 기록 변경 전 대상과 영향이 화면에 나오며 `y` 확인 뒤 실행됩니다.

## USB/UART protocol

프레임은 `[A5 5A][version u8][type u8][length u16 LE][sequence u32 LE]`
`payload` `CRC32 u32 LE` 순서입니다. CRC는 version부터 payload까지 계산합니다.
동일 프레임을 USB CDC와 `Serial1` UART0(`115200 8N1`) 양쪽으로 보내며, 명령도
두 transport에서 같은 형식으로 받습니다.
`0x01` status는 100 ms마다 전송하고, `0x10` command와 `0x11` persisted-result ACK를
사용합니다. Status payload의 정수 단위는 bus `uV`, shunt `nV`, current `uA`,
power `mW`, INA die temperature `mC`, trip charge는 `uAs`/`mWs`입니다. SOC와
remaining은 미설정일 때 각각 `-1`입니다. 진단에는 `VBUS/VSHUNT/CURRENT`의
raw 24-bit register 값도 포함하며, VSHUNT/CURRENT는 bits 23:4를 signed 20-bit로
해석합니다. `bus_voltage_uV`, `shunt_voltage_nV`, `current_uA`, `power_mW`는
RP2350 LPF 결과이고 raw 레지스터 값은 미필터입니다. 이 payload 하나가 XIAO 내부
적산 상태와 STM_B ROS TUI·USB fallback의 공통 상태입니다.

## 업로드 절차

최초 업로드 또는 bootloader를 통한 복구 절차입니다. USB 정비 시 외부
`5V/VBUS` 전원을 분리합니다.

1. USB를 분리하고 XIAO의 `BOOT` 버튼을 누른 상태로 USB를 연결해 RP2350 ROM
   bootloader(UF2 드라이브)로 진입합니다.
2. `arduino-cli board list`에서 `UF2_Board`가 나타나는지 확인합니다.
3. 다음 명령을 실행합니다. `UF2_Board`는 Arduino-Pico의 UF2 업로드 포트 이름입니다.

```bash
arduino-cli upload \
  --fqbn rp2040:rp2040:seeed_xiao_rp2350 \
  --board-options flash=2097152_262144 \
  --port UF2_Board \
  --input-file firmware/XIAO_RP2350_LV_GAUGE/build/XIAO_RP2350_LV_GAUGE.ino.uf2
```

4. 재부팅 후 `/dev/serial/by-id/usb-Seeed_XIAO_RP2350_*-if00` 장치가 나타나면
   USB 정비 TUI를 엽니다. serial suffix는 보드마다 다릅니다.

```bash
python3 firmware/XIAO_RP2350_LV_GAUGE/tools/lv_battery_gauge_tui.py --usb
```

TUI에서 출력되는 상태는 다음 의미를 갖습니다.

```text
INA address=0x40 manufacturer=0x5449 device=0x2281
bus_voltage=... V; shunt_voltage=... mV; current=+... A; power=+... W
```

STM_B UART4 수신, `/lv_battery/state` 및 정비 payload 중계, UNO의 LV 전압·SOC
표시는 코드에 구현되어 있습니다. 펌웨어 빌드나 센서 ID 확인만으로 실제 전압·전류
정확도와 정전 복구가 검증되는 것은 아닙니다. 장착 구성과 확인 항목은
[LV 배터리 구성 문서](../../docs/rp2350/lv_battery_gauge.md)를 참고합니다.
