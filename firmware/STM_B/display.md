# STM_B LCD 디스플레이 동작 문서

대상 파일: `Core/Src/spi.c`, `Core/Inc/spi.h`, `Core/Src/lcd.c`, `Core/Inc/lcd.h`,
`Core/Src/display_app.c`, `Core/Inc/display_app.h`

목적: 지지직거림(깜빡임/노이즈) 및 화면 180도 회전 현상의 원인을 좁히기 위해
각 계층이 무엇을 하는지, 어떤 파라미터가 어떤 증상과 연결될 수 있는지 정리.

---

## 1. 계층 구조

```
display_app.c  (Dashboard_Task - UI 로직, 50ms 주기)
      |
     lcd.c      (ST7796S/MSP4030 패널 드라이버 - 커맨드/픽셀 전송)
      |
     spi.c      (SPI2 HAL 초기화 - 클럭/DMA/GPIO)
      |
   STM32G4 HAL (HAL_SPI_Transmit / HAL_SPI_Transmit_DMA)
```

패널은 MSP4030 모듈(ST7796S 컨트롤러로 추정, TN 패널)이며 `lcd.c`의
`msp4030_init[]` 테이블이 벤더 초기화 시퀀스다.

---

## 2. spi.c — SPI2 저수준 설정

### 2.1 `MX_SPI2_Init()` 의 각 필드가 의미하는 것

| 필드 | 현재 값 | 의미 / 바꾸면 생기는 일 |
|---|---|---|
| `Mode` | `SPI_MODE_MASTER` | MCU가 마스터. 바꿀 이유 없음 |
| `Direction` | `SPI_DIRECTION_2LINES` | MOSI+MISO 풀듀플렉스. LCD_ReadPowerMode가 MISO로 읽기 때문에 필요 |
| `DataSize` | `SPI_DATASIZE_8BIT` | 8비트 단위 전송. lcd.c의 모든 코드가 byte 배열 기준이라 16비트로 바꾸면 lcd.c 전체 재작성 필요 |
| `CLKPolarity` / `CLKPhase` | `HIGH` / `2EDGE` (SPI 모드 3) | **패널이 요구하는 모드와 정확히 일치해야 함.** 모드가 틀리면 커맨드/데이터 비트가 한 클럭씩 밀려 읽혀서 임의의 커맨드로 오인식됨 → 화면 깨짐/회전/색 반전 등 "장거리 노이즈"와 똑같은 증상을 만들 수 있음 |
| `NSS` | `SPI_NSS_SOFT` | CS(LCD_CS)를 SPI 하드웨어가 아니라 GPIO로 수동 제어(`LCD_Select`/`LCD_Unselect`). SPI 클럭과 CS 엣지 사이의 타이밍이 하드웨어 NSS보다 느슨함 — 긴 케이블에서 클럭 엣지 근처에서 CS 전이가 흔들리면(리플렉션) 첫/마지막 비트가 깨질 여지가 있음 |
| `BaudRatePrescaler` | `SPI_BAUDRATEPRESCALER_128` | SPI 클럭 = APB1클럭 / 128. 아래 2.2절 참고. **가장 먼저 건드려볼 파라미터** |
| `FirstBit` | `SPI_FIRSTBIT_MSB` | 패널 스펙과 일치해야 함 |
| `CRCCalculation` | `DISABLE` | 하드웨어 CRC 체크 없음 → SPI 라인에서 비트가 깨져도 MCU는 알 방법이 없음 (LCD_IsHealthy로 사후 확인만 가능) |

### 2.2 SPI2 클럭 유도 (`main.c`의 `SystemClock_Config`)

- HSI 16MHz → PLLM=/4 (4MHz) → PLLN×85 (340MHz VCO) → PLLR=/2 → **SYSCLK 170MHz**
- AHB(HCLK) = SYSCLK /1 = 170MHz
- APB1(PCLK1) = HCLK /1 = 170MHz ← **SPI2는 APB1 버스에 있음**
- 현재 Prescaler 128 → **SPI2 클럭 ≈ 170MHz / 128 ≈ 1.33 MHz**

Prescaler를 바꾸면:

| Prescaler | SPI2 클럭 | 비고 |
|---|---|---|
| 64 | ≈2.66 MHz | 현재보다 빠름 — 프레임 전송 시간은 줄지만 장거리 케이블에서 신호 무결성은 더 나빠질 가능성 |
| 128 (현재) | ≈1.33 MHz | |
| 256 | ≈0.66 MHz | 더 느리게 — 지지직거림이 "케이블 리플렉션/크로스토크" 문제라면 이 값을 낮췄을 때 증상이 줄어드는지가 진단 포인트 |

> `SPI_BAUDRATEPRESCALER_*` 값을 바꾸는 실험은 케이블 문제인지 아닌지를 가르는
> 가장 저렴한 실험이다. 낮춰서(느리게) 증상이 사라지면 신호 무결성(장거리/반사/EMI) 문제,
> 그대로면 다른 원인(전원, 접지, 소프트웨어 타이밍)을 봐야 한다.

### 2.3 GPIO / DMA

- SCK/MISO/MOSI = PB13/PB14/PB15, AF5(SPI2), Speed = `GPIO_SPEED_FREQ_HIGH`
  - CS/DC/RST/BL(PB0/PB1/PB2/PB10)는 SPI AF가 아니라 **일반 GPIO 출력**으로 `lcd.c`에서 직접 토글 (main.c에서 별도 초기화, 이 파일들엔 없음)
- DMA1_Channel5, `MEMORY_TO_PERIPH`, `MINC_ENABLE`, `Priority = LOW`
  - 픽셀 데이터(대량)만 DMA로 전송(`HAL_SPI_Transmit_DMA`), 커맨드/소량 데이터는 블로킹 전송(`HAL_SPI_Transmit`)
  - DMA 우선순위가 LOW이므로, 다른 DMA(CAN, UART 등)와 경합 시 SPI DMA가 밀릴 수 있음 — 필요시 우선순위를 올려서 지지직거림 빈도가 바뀌는지 확인 가능

---

## 3. lcd.c — LCD 드라이버

### 3.1 커맨드/데이터 전송 원시 함수

- `LCD_Select()` / `LCD_Unselect()` — CS(PB0) Low/High
- `LCD_WriteCommand(cmd)` — DC(PB1)=Low(커맨드 모드) → CS Low → `HAL_SPI_Transmit` 1바이트, 타임아웃 `LCD_SPI_TIMEOUT_MS`(100ms) → CS High
- `LCD_WriteData(data, len)` — DC=High(데이터 모드) → 블로킹 `HAL_SPI_Transmit`
- `LCD_WriteCommandData()` — 위 둘을 묶은 헬퍼 (초기화 시퀀스에서 사용)
- `LCD_TransmitPixels(data, len)` — **DMA 경로**. `HAL_SPI_Transmit_DMA` 시작 후
  `dma_complete`/`dma_error` 플래그를 100ms 타임아웃까지 1ms 간격(`osDelay(1)`)으로 폴링.
  타임아웃 시 `HAL_SPI_Abort` 호출 후 `LCD_ERROR_DMA_TIMEOUT` 기록.
  완료 콜백은 파일 하단의 `HAL_SPI_TxCpltCallback` / `HAL_SPI_ErrorCallback`(hspi2 필터링)에서 옴.

> DC 핀은 **명령/데이터 전송 도중에는 바뀌지 않고**, 각 전송 전에 한 번만 세팅된다.
> 즉 만약 노이즈로 SPI 클럭에 글리치가 생겨 비트가 하나 밀리면, MCU 쪽은 "정상적으로
> 보낸 척"하지만 패널은 엉뚱한 바이트를 명령/데이터로 해석 — 이게 화면 깨짐이나
> 색상 오류, MADCTL 커맨드가 깨질 경우 **회전/미러 반전**까지 만들 수 있는 경로다.
> 이 코드에는 전송 성공 여부(HAL 리턴값)만 확인하고, 패널이 실제로 그 값을 받았는지
> 리드백 검증은 하지 않는다 (MADCTL 등 대부분의 write-only 커맨드는 원래 리드백 불가).

### 3.2 초기화/회전 로직 — `LCD_Init()` (180도 회전 현상의 핵심)

```
MADCTL(0x36) 값 결정:
  portrait  (320x480, ROTATION_PORTRAIT)  → BGR | MX   (0x08 | 0x40)
  landscape (480x320, ROTATION_LANDSCAPE) → BGR | MV   (0x08 | 0x20)
```

- `display_app.c`는 항상 `LCD_ROTATION_LANDSCAPE`(480x320)로 `LCD_Init`을 호출하므로,
  **정상 동작 시 회전 모드는 코드상 고정**되어 있다. 즉 소프트웨어가 스스로 180도를
  선택하는 경로는 없다 — "180도로 나온다"는 것은 곧 **MADCTL 바이트(0x36 커맨드 뒤에
  오는 1바이트 데이터)가 SPI 상에서 손상되어 패널에 다른 값이 들어갔다**는 뜻일 가능성이
  가장 크다. (MY 비트가 우연히 세팅되면 상하 반전, MX+MY가 같이 틀어지면 180도 반전 조합이 나옴)
- 재현 확인법: `Dashboard_Recover()`가 호출될 때마다(=SPI 에러로 인한 자동 복구,
  또는 아래 3.5의 기어 제스처) `LCD_Init`이 다시 실행되며 MADCTL도 재전송된다.
  **회전이 랜덤하게 나타났다가 다음 복구 사이클에 정상으로 돌아온다면, MADCTL 전송
  타이밍에 노이즈가 탄 것으로 사실상 확정**할 수 있다. 반대로 재부팅해도 계속 180도라면
  배선(MADCTL 계산 로직이 아니라 패널 자체/배선 극성) 문제를 봐야 한다.
- 초기화 시퀀스: `LCD_SetBacklight(false)` → RST Low 100ms → RST High 50ms →
  `msp4030_init[]` 순차 전송 → `NORON` → `SLPOUT` → 120ms delay → `DISPON` → 20ms delay.
  이 지연시간들은 패널 데이터시트 스펙이므로 임의로 줄이면 초기화 자체가 실패하거나
  불안정해질 수 있음 (건드릴 파라미터 아님).

### 3.3 화면 갱신 경로 — 두 가지

**(A) 즉시 그리기 (non-compose)** — `LCD_FillRect`/`LCD_DrawCharScaled`가
`compose_band_active == 0` 일 때 타는 경로:
1. 매 호출마다 `LCD_SetAddressWindow`(CASET+RASET+RAMWR, 커맨드 3번 블로킹 전송)
2. `pixel_buffer`를 채우고 `LCD_TransmitPixels`로 DMA 전송, 필요시 버퍼 크기(`LCD_PIXEL_BUFFER_BYTES`)만큼 chunk 반복
3. 작은 영역(배지, 텍스트 한 글자 등)마다 **주소창 설정 + DMA 전송**이 각각 따로 발생 → 작은 SPI 트랜잭션이 매우 많음

**(B) 컴포즈 밴드 (compose band)** — `LCD_BeginComposeBand(y, height)` ~
`LCD_EndComposeBand()` 사이에서 호출된 `FillRect`/`DrawCharScaled`는 실제 SPI 전송 없이
RAM의 `pixel_buffer`에만 그려지고, `LCD_EndComposeBand()`에서 **한 번의 주소창 설정 +
한 번의 DMA 전송**으로 `height`줄(최대 `LCD_COMPOSE_BAND_MAX_ROWS`=4줄) 전체 폭을
한꺼번에 내려보낸다.

`display_app.c`는 매 틱(50ms)마다 **(A)로 바뀐 필드만 그리고, 동시에 (B)로 화면을
4줄씩 위에서 아래로 훑으며 계속 다시 그린다** (아래 4.3절). 이 이중 경로가
증상 분석에서 중요한 포인트다 (5절 참고).

### 3.4 `pixel_buffer` 크기

```
LCD_PIXEL_BUFFER_BYTES = LCD_LANDSCAPE_WIDTH(480) * LCD_COMPOSE_BAND_MAX_ROWS(4) * 2 = 3840 bytes
```

- `LCD_COMPOSE_BAND_MAX_ROWS`를 늘리면: 한 번의 DMA 전송이 커져서 전송당 오버헤드(주소창
  설정 등)는 줄지만, 한 번의 전송 시간이 길어져 그 전송 도중 노이즈에 노출되는 시간도
  길어짐 + RAM 사용량 증가. 줄이면 반대.
- 이 값은 `LCD_BeginComposeBand`에서 `height > LCD_COMPOSE_BAND_MAX_ROWS`면 실패하도록
  강제되어 있음.

### 3.5 에러/헬스 체크

- `LCD_ReadPowerMode` / `LCD_IsHealthy` — RDDPM(0x0A) 레지스터를 읽어 SLEEP_OUT(bit4)+DISPLAY_ON(bit2)이 모두 세팅되어 있는지 확인. 이 함수 자체가 SPI MISO 라인을 쓰는 유일한 경로이므로, MISO 배선/노이즈를 의심할 때 이 결과가 흔들리는지 보면 됨.
- 실패는 전부 `LCD_Fail()`(lcd.c:57-62)을 거쳐 **전역 카운터 4개**에 누적된다. 의미는 3.5.1절 참고.

#### 3.5.1 에러 카운터 4개가 의미하는 것

**① `LCD_ERROR_COUNT`** (lcd.c) — SPI/LCD 계층에서 실패한 **개별 전송** 누적 횟수. 아래 함수 중 하나라도
실패하면 `LCD_Fail()`이 호출되어 +1. 화면이 눈에 띄게 깨졌는지와 무관하게, 아주 짧은 전송 실패도 여기 잡힘.

**② `LCD_LAST_ERROR`** (lcd.c, `enum LCD_Error_t`, lcd.h:36-47) — 가장 최근 실패의 **종류**. 폴링 주기가
길면 중간에 다른 종류 에러가 섞여도 마지막 것만 보이므로(덮어쓰기), 짧은 간격으로 확인해야 한다.

| 코드 | 값 | 의미 | 발생 함수 |
|---|---|---|---|
| `LCD_ERROR_NONE` | 0 | 실패 없음(초기값) | — |
| `LCD_ERROR_COMMAND_TX` | 1 | 커맨드 1바이트 블로킹 전송 실패 | `LCD_WriteCommand` |
| `LCD_ERROR_DATA_TX` | 2 | 파라미터 데이터 블로킹 전송 실패 (초기화 시퀀스, **MADCTL 포함**) | `LCD_WriteData` |
| `LCD_ERROR_DMA_START` | 3 | 픽셀 DMA 전송이 아예 시작을 못함 (채널 busy 등) | `LCD_TransmitPixels` |
| `LCD_ERROR_DMA_TIMEOUT` | 4 | DMA 시작은 됐는데 100ms 안에 완료 콜백이 안 옴 | `LCD_TransmitPixels` |
| `LCD_ERROR_DMA_CALLBACK` | 5 | SPI 페리페럴이 자체 에러(overrun/mode fault 등) 플래그를 냄 | `HAL_SPI_ErrorCallback` |
| `LCD_ERROR_READ_COMMAND` | 6 | RDDPM 커맨드 전송 실패 | `LCD_ReadPowerMode` |
| `LCD_ERROR_READ_DATA` | 7 | MISO 응답 읽기 실패 | `LCD_ReadPowerMode` |
| `LCD_ERROR_HEALTH_STATUS` | 8 | 전송은 성공했는데 읽어온 Power Mode 값이 "정상 동작 중" 조합이 아님(패널이 슬립/리셋된 상태) | `LCD_IsHealthy` |

진단 포인트: **1(COMMAND_TX)/2(DATA_TX)면 초기화 커맨드 손상**(180도 회전 의심 케이스와 직결, 3.2절),
**4(DMA_TIMEOUT)면 픽셀 데이터 전송 중 멈춤**, **5(DMA_CALLBACK)면 SPI 페리페럴이 스스로 이상을 감지**한 것으로
원인 계열이 갈린다.

**③ `DISPLAY_RECOVERY_COUNT`** (display_app.c) — "화면이 멈춰서 풀 재초기화 모드로 넘어간" 횟수.
`display_app.c:935`, `:950`에서만 증가 — 즉 **정상 동작 중(`display_ready==1`)이다가 처음 실패를 감지해
`display_ready=0`으로 떨어뜨린 순간**만 센다. 이후 1초 간격(`DISPLAY_RECOVERY_RETRY_MS`) 재시도 루프
(`display_app.c:906-920`)에서 계속 실패해도 이 값은 더 오르지 않는다.

**④ `DISPLAY_LAST_RECOVERY_ERROR`** (display_app.c) — 복구가 트리거된 그 순간의 `LCD_LAST_ERROR` 스냅샷
(`display_app.c:936`, `:951`). "**왜** 다운됐는지"를 알려준다.

**4개를 같이 읽는 법**:

- 항상 `LCD_ERROR_COUNT ≥ DISPLAY_RECOVERY_COUNT` — 복구 1회당 최소 1개의 LCD 에러가 선행해야 하므로.
- `LCD_ERROR_COUNT`는 계속 늘어나는데 `DISPLAY_RECOVERY_COUNT`는 멈춰 있다 → 화면이 다운된 채로
  1초마다 `LCD_Init` 재시도를 계속 하고 있는데 **매번 실패** 중(지속적 다운, 일시적 글리치 아님).
  백라이트도 계속 꺼진 채일 것.
- `DISPLAY_RECOVERY_COUNT`가 짧은 시간에 여러 번 오른다 → 매번 잠깐 복구됐다가 다시 떨어지는
  **간헐적** 문제. `DISPLAY_LAST_RECOVERY_ERROR`가 매번 같은 코드인지가 원인 분석에 중요.
- `LCD_ERROR_COUNT`는 그대로인데 화면이 이상해 보인다 → SPI 전송은 HAL 레벨에서 "성공" 처리됐지만
  내용이 손상된 경우(비트 하나 밀림 등)로, **이 4개 카운터로는 못 잡는 사각지대**다. MADCTL 회전
  문제가 여기 해당할 가능성이 있다 — 전송은 성공했다고 나오는데 패널이 받은 비트는 틀렸을 수 있음.

---

## 4. display_app.c — 대시보드 애플리케이션 로직

### 4.1 상수

| 매크로 | 값 | 의미 |
|---|---|---|
| `DISPLAY_UPDATE_MS` | 50 | 메인 갱신 주기. 줄이면 갱신은 빨라지지만 SPI 트래픽/CPU 부하 증가 → 노이즈 유발 빈도가 늘 수 있음 |
| `DISPLAY_RECOVERY_RETRY_MS` | 1000 | 초기화/복구 실패 시 재시도 간격 |
| `DISPLAY_GEAR_GESTURE_WINDOW_MS` | 1000 | 아래 4.4절 제스처 인식 윈도우 |

### 4.2 `Dashboard_ReadState` / 값 포맷팅

CAN(STM_A)에서 받은 값(`globals.h`의 `CURRENT_SPEED_FROM_A` 등)을 `taskENTER/EXIT_CRITICAL`로 보호하며 스냅샷. 이후 `FormatTenthsSigned/Unsigned`, `FormatSpeed` 등이 텍스트로 변환. **이 부분은 화면 표시 내용에만 관여하고 SPI 타이밍과는 무관** — "값 갱신이 잦을 때만 깨진다"는 관찰이 나오면 4.3절의 diff-draw 경로(A)가 원인일 확률이 높아진다.

### 4.3 매 틱의 두 갱신 경로 (증상 재현에 가장 중요)

`Dashboard_Task`의 메인 루프(각 `DISPLAY_UPDATE_MS`마다):

1. **`Dashboard_DrawChanges(current, previous)`** — 이전 상태와 비교해 배지/HV값/속도/온도/하단바 중 **바뀐 필드만** 즉시 경로(A)로 그림. 값이 안 바뀌면 이 단계는 거의 아무것도 안 그림.
2. **`Dashboard_RefreshBand(current, refresh_y, refresh_rows)`** — 화면 전체(0~320행)를 4행씩 위에서 아래로 **매 틱마다 계속** 컴포즈 밴드(B)로 재그리기. `refresh_y`가 320을 넘으면 0으로 wrap. 즉 값이 하나도 안 바뀌어도 **화면 전체가 320/4=80틱 = 약 4초에 한 바퀴씩 계속 다시 그려지고 있다.**

→ **이 구조 때문에 "정적 화면(값 변화 없음)"이어도 SPI 트래픽은 절대 0이 아니다.**
사용자가 계획한 1번 실험("CAN 케이블 분리, 정적 화면")에서도 이 롤링 리프레시 때문에
DMA 전송은 계속 발생한다 — 즉 1번 실험에서 지지직거림이 사라지지 않는다고 해서
"SPI 자체는 무죄"라고 결론 내릴 수 없고, 오히려 **정적 화면에서도 노이즈가 난다면
그게 SPI/배선/전원 자체의 문제라는 강한 증거**가 된다 (모터/CAN 트래픽과 무관하게
재현되므로).

또한 컴포즈 밴드 전송은 **화면 폭 전체(480px)를 4줄 단위 가로 띠**로 한 번의 DMA로
내려보내므로, **만약 전송 도중에만 노이즈가 타서 깨진다면 그 결과물은 정확히 가로줄
형태**로 나타난다 (사용자가 나열한 "가로줄 vs 랜덤" 구분과 정확히 대응됨).
반대로 즉시 경로(A)에서 텍스트 한 글자 단위로 깨진다면 특정 숫자/글자 영역만
깨지는 형태로 보일 것이다.

### 4.4 기어 리셋 제스처 → 강제 전체 재초기화

`GearResetGesture_Update`: 1초 이내에 기어가 FWD→REV→FWD→REV→FWD(전이 4회) 패턴이 들어오면
`Dashboard_Recover()`를 강제 호출 — `LCD_Init` 재실행 + 백라이트 끔/켬 + 전체 화면 재작성.
**진단 실험 중 "지금 180도로 돌았다"를 보면, 이 제스처(또는 자동 복구)로 강제 재초기화를 유도해
회전이 정상으로 돌아오는지 확인하면 "MADCTL 전송 시점의 일시적 노이즈"인지 "상시 재현되는
배선/극성 문제"인지 즉시 구분할 수 있다.**

### 4.5 자동 복구 (`Dashboard_Recover`)

`LCD_FillRect`/`DrawChar`/`EndComposeBand` 등 SPI 계열 함수가 하나라도 실패(false 리턴, 즉 `LCD_ERROR_COUNT` 증가)하면:
1. `DISPLAY_RECOVERY_COUNT++`, `DISPLAY_LAST_RECOVERY_ERROR = LCD_LAST_ERROR` 기록
2. `LCD_SetBacklight(false)` — **백라이트를 끔**
3. 다음 루프에서 `display_ready == 0` 이므로 `Dashboard_Recover` → `LCD_Init` 풀 재초기화 → 전체 화면 재작성 → `LCD_SetBacklight(true)`

→ **사용자가 "깜빡임"이라 보는 현상 중 일부는 실제로는 SPI 전송 실패 → 자동 복구 사이클이
백라이트를 껐다 켜는 것일 수 있다.** 이 경우 진짜 원인은 화면 렌더링 노이즈가 아니라
"SPI 전송이 100ms 타임아웃 안에 실패해서 복구 루틴이 도는 것"이다.
**`DISPLAY_RECOVERY_COUNT`가 실측 중 계속 증가하는지 디버거로 확인하는 것이 1순위 체크포인트.**

### 4.6 화면에 어떤 데이터가, 어디서 와서, 어떻게 표시되는가

값의 원천은 전부 **STM_A → STM_B UART**(`board_uart_protocol.c/h`, `globals.h`)로 수신된
전역변수이며, `Dashboard_ReadState()`(4.2절)가 매 틱 스냅샷을 뜬다. 화면 레이아웃은
`Dashboard_DrawStaticLayout()`이 그리는 고정 틀(카드/테두리/라벨) 위에, 아래 각 값이
올라가는 구조다. 좌표는 480×320(landscape) 기준.

**전체 레이아웃 (좌상단 원점 기준)**

```
y=6~40    [MODE]badge_x0  [FWD/REV]badge_x1  [A-B OK/LOST]badge_x2  [SAFE/E-STOP/BRAKE]badge_x3
y=50~196  ┌─ HV BUS 카드(x8~300) ──────────┐  ┌─ SPEED 카드(x308~472) ─┐
          │ VOLT ...                        │  │  (큰 숫자, 5배 스케일)  │
          │ BUS  ...                        │  │  KM/H                  │
          │ PHS  ...                        │  │                        │
y=204~266 ┌MOTOR(x8~156)┐ ┌CTRL(x166~314)┐ ┌FAULT(x324~472)┐
y=274~312 ────────── 하단 상태바 (좌: left_text / 우: right_text) ──────────
```

**상단 배지 4개** (`BuildBadgeViews`, `Dashboard_DrawTopChanges`/`Dashboard_DrawTop`)

| 배지 | 소스 | 표시값 | 색상 로직 |
|---|---|---|---|
| MODE | `MODE_FROM_A` (`state->mode`) | `state->vehicle_valid`=0이면 `"MODE --"`(회색 `UI_BORDER`) / `mode==1`→`"AUTO"`(파랑 `UI_AUTO`) / 그 외 `"MANUAL"`(주황 `UI_MANUAL`) |
| 기어 | `GEAR_FROM_A` (`state->gear`) | `vehicle_valid`=0이면 `"--"` / `gear==1`→`"REV"` / 그 외 `"FWD"`. 배경은 항상 `UI_BORDER`(회색) |
| 통신 | `VEHICLE_FRAME_VALID_FROM_A` (`state->vehicle_valid`) | `"A-B OK"`(초록)/`"A-B LOST"`(빨강) — STM_A↔STM_B UART 링크 자체의 생사 표시 |
| 안전 | `ESTOP_STATUS`, `BRAKE_REQUEST` | E-stop 있으면 `"E-STOP"`, 없고 brake면 `"BRAKE"`, 둘 다 없으면 `"SAFE"`(초록/빨강) |

**HV BUS 카드** (`BuildHvText`, `Dashboard_DrawHv`/`Dashboard_DrawHvChanges`) — 소스: `DRIVE_DETAIL_FROM_A`(`BoardUartDriveDetail_t`), `BOARD_UART_DETAIL_PART1_VALID` 플래그가 서야 유효값. 유효하지 않으면 전부 `"--.- V"`/`"--.- A"` 표시(더미값 아님, 데이터 무효 표시).

| 라벨 | 필드 | 단위 스케일 | 표시 |
|---|---|---|---|
| VOLT | `bus_voltage_dv` (uint16) | deci-volt(0.1V 단위) | 9999 초과 시 `">999 V"`, 아니면 `xx.x V` |
| BUS | `bus_current_da` (int16, 부호) | deci-amp | `±xx.x A` |
| PHS | `phase_current_da` (int16, 부호) | deci-amp | `±xx.x A` |

**SPEED 카드** (`BuildSpeedText`, `Dashboard_DrawSpeed`) — 소스: `CURRENT_SPEED_FROM_A`(m/s, float). `vehicle_valid`=0이면 `"--"`. `speed_mps × 3.6` = km/h로 변환해 소수 1자리(`FormatSpeed`)로 표시, 9999(=999.9km/h) 초과면 `">999"`. 5배 스케일 폰트로 가장 크게 그려지는 값.

**MOTOR / CTRL 카드** (`BuildMetricText`, `Dashboard_DrawMetrics`/`Changes`) — 소스: `DRIVE_DETAIL_FROM_A`, `BOARD_UART_DETAIL_PART2_VALID` 플래그 필요.

| 라벨 | 필드 | 단위 | 표시 |
|---|---|---|---|
| MOTOR | `motor_temperature_dc` (int16, 부호) | deci-celsius | `±xx.x C` |
| CTRL | `controller_temperature_dc` (int16, 부호) | deci-celsius | `±xx.x C` |

**FAULT 카드** — 소스: `motor_fault_bits` (uint32, 실사용은 하위 24비트). 0이면 `"OK"`(초록 `UI_OK`), 아니면 `"0X부호6자리"` 헥사(빨강 `UI_FAULT`) — 어떤 비트가 섰는지는 이 화면만으로는 알 수 없고 값만 보임.

**하단 상태바** (`BuildBottomView`, `Dashboard_DrawBottom`) — 여러 조건을 **우선순위 순으로** 검사해 좌/우 텍스트와 배경색을 결정 (`display_app.c:577-664`), 먼저 만족하는 조건이 이김:

1. `ESTOP_STATUS` → `"E-STOP ACTIVE" / "BRAKE ENGAGED"` (빨강)
2. `VEHICLE_FRAME_VALID_FROM_A`=0 → `"STM A COMM LOST" / "VALUES STALE"` (빨강) — 위 화면의 다른 값들이 전부 정지된 옛 값이라는 경고
3. `motor_fault_bits != 0` → `"MOTOR FAULT" / "0X헥사"` (빨강)
4. `BOARD_UART_DETAIL_DRIVE_STOP` 플래그 → `"DRIVE STOP" / "TRACTIVE OFF"` (빨강)
5. `BRAKE_REQUEST` → `"BRAKE ENGAGED" / "TRACTIVE STOP"` (빨강)
6. `mode==1`(AUTO) → 좌: `COMMAND_RECEIVED`/`COMMAND_TIMEOUT` 플래그로 `"AUTO CMD OK/WAIT/TIMEOUT"`, 우: `TRACTIVE_FEEDBACK` 플래그로 `"TRACTIVE RUN/STOP"` (파랑, `UI_AUTO`)
7. 그 외(MANUAL 정상) → 좌: `"PEDAL xx%"`(`pedal_percent`), 우: `TRACTIVE RUN/STOP` (주황, `UI_MANUAL`)

**진단에 참고할 점**: 위 값 중 `VOLT/BUS/PHS/MOTOR/CTRL/FAULT`는 `PART1_VALID`/`PART2_VALID` 플래그가
꺼지면 그리는 문자가 통째로 `"--.- V"` 류로 바뀐다 — 즉 **화면상 값이 갑자기 "--"로 튀는 건 지지직거림이
아니라 STM_A→STM_B UART 링크(별도 프로토콜, SPI/LCD와 무관)의 유효성 플래그가 꺼진 것**이므로,
LCD/SPI 문제와 UART 링크 문제를 혼동하지 않아야 한다. 반대로 문자 자체가 깨진 모양(글리프가
이상하거나 부분적으로 다른 색)으로 나오면 그건 SPI/LCD 렌더링 쪽 문제다.

---

## 5. 실험 계획 (사용자 제공, 원인 매핑 포함)

### 5.1 증상 조건별 분리 체크리스트

- [ ] 화면 **전체**가 깜빡이는가, **특정 영역/텍스트**만 깨지는가
  → 전체: 백라이트 토글(복구 사이클, 4.5절) 또는 전원 문제 의심.
    특정 영역만: 즉시 경로(A)의 특정 위젯(4.3절) 또는 컴포즈 밴드 특정 y구간(전송 중 노이즈, 4.3절) 의심
- [ ] 값이 **갱신되는 순간**에만 발생하는가, 상시 발생하는가
  → 상시 발생: 롤링 리프레시(4.3절)가 정적 상태에서도 SPI를 계속 쏘고 있으므로
    "상시"도 SPI 관련일 수 있음 — CAN 분리 실험(5.2의 1번)으로 구분
- [ ] 깨진 픽셀이 **가로줄** 형태인가, 랜덤한가
  → 가로줄: 컴포즈 밴드 DMA 전송(4행×480px) 도중 노이즈 (3.3-B, 4.3절) 가능성 높음.
    랜덤: 개별 커맨드/데이터 블로킹 전송(3.1절, `LCD_WriteCommand`/`LCD_WriteData`) 쪽 의심

### 5.2 실험 순서과 확인할 텔레메트리

1. **CAN 케이블 분리, 정적 화면만**
   - 확인 대상: 롤링 리프레시(4.3-2)는 계속 돌고 있으므로 SPI 자체 문제라면 여기서도 재현되어야 함
   - 재현되면 → CAN/모터와 무관, SPI 배선/전원/타이밍(2절, 3.2절 MADCTL) 문제로 좁혀짐
   - 재현 안 되면 → 아래 2, 3번으로 진행
   - 이때 `LCD_ERROR_COUNT`, `DISPLAY_RECOVERY_COUNT`를 시작 전/후로 기록해 비교
2. **CAN 연결 + 모터 ON** (값 변화 적음 / 값 변화 잦음 비교)
   - 값 변화가 잦을 때만 악화되면 → 즉시 경로(A)의 트랜잭션 수 증가가 원인 (4.3-1절) — 즉 SPI 자체보다 "트랜잭션 빈도"와 상관
   - 값 변화와 무관하게 모터 ON 시에만 악화되면 → 모터 드라이버발 EMI가 SPI 라인에 유도되는 것 (배선 근접/차폐 문제)
3. **모터 드라이버 전원만 차단, MCU/디스플레이 유지**
   - 노이즈가 사라지면 → 모터 드라이버가 노이즈원(EMI/그라운드 바운스)임을 확정
   - 안 사라지면 → 모터와 무관한 상시 요인(케이블 길이/임피던스 미스매치, 전원 리플, SPI 클럭 자체)

### 5.3 우선 시도해볼 만한 파라미터 변경 (실험과 별개로 코드 레벨 대응)

- `SPI_BAUDRATEPRESCALER_128` → `256`으로 낮춰서 지지직거림 빈도 변화 확인 (2.2절)
- `LCD_COMPOSE_BAND_MAX_ROWS`를 4 → 2로 낮춰 한 번의 DMA 전송 길이를 줄이고 가로줄 폭이 좁아지는지 확인 (3.4절)
- 회전 문제 재현 시 `Dashboard_Recover`(자동 복구, 4.5절)나 기어 제스처(4.4절)로 강제 재초기화가 되는지, 재초기화 후 정상으로 돌아오는지로 "일시적 비트 손상 vs 상시 배선 문제" 구분 (3.2절)

---

## 6. 요약: 코드에서 확인한 핵심 사실

1. 소프트웨어는 항상 landscape(MV) 고정 회전만 요청한다 — 180도 반전은 코드 로직이 아니라
   **MADCTL 커맨드 바이트의 SPI 전송 손상**이 가장 유력한 설명.
2. 화면은 값이 하나도 안 바뀌어도 **4초에 한 바퀴씩 전체가 계속 재전송**되고 있다(컴포즈 밴드
   롤링 리프레시). "정적 화면=SPI 트래픽 없음"이 아니므로 실험 설계 시 유의.
3. 컴포즈 밴드 전송은 4행×480px 단위의 가로 띠 DMA 전송이라, 전송 중 노이즈는 **가로줄 형태**로
   나타나는 것이 구조적으로 자연스럽다.
4. SPI 전송 실패는 `LCD_ERROR_COUNT`/`LCD_LAST_ERROR`로, 그로 인한 자동 복구(백라이트 토글 포함)는
   `DISPLAY_RECOVERY_COUNT`/`DISPLAY_LAST_RECOVERY_ERROR`로 각각 카운트되므로, 실측 중 이 4개
   전역변수를 관찰하면 "언제/얼마나" 문제가 났는지 정량화할 수 있다.
5. 현재 SPI2 클럭은 약 1.33MHz(APB1 170MHz / 128)이며, 이는 실험적으로 낮춰볼 수 있는 값이다.

마지막 수정 날짜 : 2026.8.28
신규 변경 or 검증 사항이 있다면 날짜 update 할 것. 