# RP2350 LCD 이관 참고 노트

**상태: 아직 RP2350 펌웨어 자체는 존재하지 않음.** 이 문서는 STM_B에서 LCD를 직접
구동하던 시절(`firmware/STM_B/Core/Src/lcd.c`, `spi.c`, `display_app.c`)에 조사·구현·
검증했다가, 아키텍처가 바뀌면서 STM_B에서는 되돌린(rollback) 내용을 나중에 RP2350
펌웨어를 만들 때 그대로 재사용할 수 있도록 남겨둔 기록이다.

## 1. 왜 이 문서가 존재하는가

목표 아키텍처:

```
STM_B → CAN → RP2350 → LCD (SPI)
```

- LCD(패널: MSP4030 모듈, 컨트롤러: ST7796S, 480×320 landscape)를 물리적으로 구동하는
  주체가 STM_B에서 **RP2350으로 옮겨갈 예정**이다. STM_B는 CAN으로 표시할 데이터만
  RP2350에 보내고, 실제 SPI 커맨드/픽셀 전송과 **MISO(SDO) 기반 read/에러 검출은
  RP2350의 몫**이 된다.
- RP2350은 LCD 구동 하나만 담당하는 단일 목적 보드로 계획 중이라 **RTOS 없이(bare
  metal 또는 단순 super-loop) 구현 가능할 수도 있다**는 게 현재(2026-08-28) 방향이다.
  이 부분은 아직 설계로 확정된 건 아니고, 이 문서에도 더 이상 파고들지 않았다.
- 아래 내용은 STM_B의 `lcd.c`/`lcd.h`에 실제로 구현해서 **host 단위 테스트(`tests/run_lcd_host_test.sh`)
  통과, `arm-none-eabi-gcc`로 STM32G4 타겟 크로스컴파일까지 경고 없이 확인**한 뒤,
  아키텍처가 RP2350으로 넘어가는 게 확정되어 STM_B에서는 롤백했다(2026-08-28,
  `git diff`로 `lcd.c`/`lcd.h`가 커밋 시점과 완전히 동일함을 확인함). 즉 **아이디어와
  코드는 검증됐고, 위치만 잘못됐던 것**이다.

## 2. 배경 — STM_B에서 있었던 SPI/디스플레이 아키텍처

`firmware/STM_B/display.md`에 STM_B가 LCD를 어떻게 구동했는지(SPI2 설정, compose-band
렌더링, 화면 레이아웃, 에러 카운터 체계 등) 상세히 정리되어 있다. RP2350 구현 시
이 문서도 함께 참고할 것 — SPI 프로토콜 자체(ST7796S 커맨드셋)는 어느 MCU에서
구동하든 동일하다.

- 패널: MSP4030 모듈 / ST7796S 컨트롤러, 480×320 landscape
- 인터페이스: **4-line SPI**(별도 D/CX 핀, 8-bit 데이터), STM_B에서는 SCK/MOSI/MISO
  + GPIO로 CS/DC/RST/BL 제어
- STM_B에서 쓰던 SPI 클럭: ~1.33 MHz (참고용. RP2350은 자체적으로 클럭을 새로 정하면 됨 — 3절의 read 타이밍 한도 참고)

## 3. 데이터시트 조사 결과 (ST7796S-Sitronix (display).pdf)

### 3.1 🎯 가장 중요한 발견 — IFMODE(0xB0) SPI_EN 버그

기존 벤더 초기화 시퀀스(`msp4030_init[]`)가 다음을 보낸다:

```c
{0xF0U, 1U, {0xC3U}},   // CSCON: enable command 2 part I   (§9.3.23, p.239)
{0xF0U, 1U, {0x96U}},   // CSCON: enable command 2 part II
...
{0xB0U, 1U, {0x80U}},   // IFMODE (§9.3.1, p.212) — 이게 문제
...
{0xF0U, 1U, {0x3CU}},   // CSCON: disable command 2 part I
{0xF0U, 1U, {0x69U}},   // CSCON: disable command 2 part II
```

`0xF0`에 `0xC3`→`0x96`을 순서대로 쓰는 게 정확히 확장 명령어("Command 2", §9.3.x 전체)
잠금 해제 절차이고, 그 안에서 보내는 `0xB0`은 **IFMODE (Interface Mode Control)**이다.
1st parameter의 **bit7 = `SPI_EN`**:

> **SPI_EN = '0'**: DIN과 DOUT 핀 둘 다 3/4-wire 시리얼 인터페이스에 사용됨
> **SPI_EN = '1'**: DIN/SDA 핀만 사용되고 **DOUT 핀은 사용 안 함**

기존 코드가 보내는 값 `0x80` = `1000_0000b` → **SPI_EN=1** → **DOUT(SDO/MISO) 핀을 끔.**

**→ RP2350 구현 시 반드시 `0xB0`에 `{0x00}`(SPI_EN=0)을 보낼 것.** 그래야 DOUT이 켜져서
MISO read가 실제 데이터를 실어 나른다. (STM_B README에는 "실차 하네스의 SDO/MISO
readback이 부동값이라 못 믿는다"고 적혀 있었는데, 케이블/배선 노이즈가 아니라 이
초기화 버그가 원인이었을 가능성이 높다는 게 이번 조사의 결론이었다 — 다만 STM_B에서
롤백했기 때문에 **실차 검증까지 가진 못했다.** RP2350에서 처음 구현할 때 이 수정을
반영한 상태로 바로 검증하면 된다.)

부수적으로 `SPIRC`(0xFBh, §9.3.24, p.240)이라는 레지스터에 `SPI_REN`(SPI read enable)
비트가 있다. 이것도 read를 켜는 별도 게이트일 수 있는데, 데이터시트에 리셋 기본값이
"N/A"로만 나와 있어 **불확실하다.** RP2350 bring-up 때 RDDPM/RDID 등을 읽어보고 값이
안 나오면 이 레지스터도 의심할 것.

### 3.2 4-line SPI read 프로토콜

- 커맨드 전송 → 방향 전환(D/CX=high) → **1바이트 dummy** → 실제 데이터 바이트, 순서.
  (§8.4.2.2, §9.2.6 RDDPM 표의 "1st parameter"가 dummy, "2nd parameter"가 실데이터)
- 컨트롤러는 SCL 상승 엣지에 SDA(입력)를 샘플링, SCL 하강 엣지에 SDO(출력)를 시프트.
  즉 MCU는 SCL 상승 엣지에서 읽으면 됨.
- 타이밍(§7.4.3, p.54, 4-SPI Interface Characteristics):
  - `TSCYCR`(read 클럭 사이클) 최소 150ns → **read 클럭 최대 ~6.67 MHz**
  - `TSCYCW`(write) 최소 66ns → write 최대 ~15.15 MHz
  - **read는 CS setup(`TCSS`)이 write보다 빡빡함: write 15ns vs read 60ns**, hold(`TSCC`)도 read만 65ns 명시.
    소프트웨어 CS(GPIO)로 제어한다면 CS assert부터 첫 클럭까지 이 여유를 확보할 것.

### 3.3 오류 검출용 read 커맨드 정리

| 커맨드 | 실시간 사용 가능? | 뭘 검출하는가 | 비고 |
|---|---|---|---|
| **RDDPM (0Ah)** | ✅ (1+1 byte) | Sleep/Display On, Booster 등 전원·모드 상태 | STM_B에도 이미 있던 함수(`LCD_ReadPowerMode`/`LCD_IsHealthy`) |
| **RDDMADCTL (0Bh)** | ✅ (1+1 byte) | **MY/MX/MV**를 직접 readback — 회전 방향 그 자체 | STM_B에선 미구현. **180도 회전류 버그를 가장 직접적으로 검증하는 커맨드** — RP2350에서는 꼭 고려 |
| **RDDST (09h)** | ✅ 이지만 무거움 (1+4 byte) | RDDPM+RDDMADCTL을 포함한 32비트 종합 상태 | 위 둘의 상위호환, 트랜잭션이 큼 |
| **RDDSDR (0Fh)** | ✅ (1+1 byte) | register loading detection(D7) / functionality detection(D6) / **checksum comparison(D0)** | STM_B에 구현·검증함(아래 4절 코드). **D0만 데이터시트에 값 정의가 있음**(`0`=일치,`1`=불일치). D7/D6은 값 정의가 없어 판정에 안 씀 — 필요하면 실측으로 폴리서티 확인 |
| RDFCS (Aah) / RDCFCS (Afh) | ❌ 실시간 불가 | 레지스터+프레임 메모리 전체 checksum | 마지막 write 후 **150ms/300ms**를 기다려야 read 가능(스펙 명시). RP2350이 유휴 시간에 가끔 한 번씩 도는 "전체 무결성 감사"용으로는 쓸 수 있음 |
| RDID1/2/3 (Dah/DBh/DCh) | ⚠️ 조건부 | 제조사/버전/모듈 ID | 문서에 고정 상수로 안 나옴(패널 OTP값). "MISO 경로가 살아있는가" bring-up sanity check로 유용 — 정상일 때 한 번 캡처해서 기준값으로 삼을 것 |
| RDDSM (0Eh) | ❌ 무관 | HS/VS/PixelClk/DataEn/ErrorDSI | RGB·DSI 인터페이스 전용 신호. SPI-only 구현에는 의미 없음 |

## 4. STM_B에 구현했다가 롤백한 실제 코드 (RP2350에서 그대로 재사용 가능)

### 4.1 초기화 시퀀스 수정

```c
/* msp4030_init[] 안, MADCTL/COLMOD 다음 줄 */
{0xB0U, 1U, {0x00U}},   /* SPI_EN=0 — DOUT/MISO 유지. 원래 {0x80U}였음(SPI_EN=1, MISO 꺼짐) */
```

### 4.2 RDDSDR 읽기 함수 (`LCD_ReadPowerMode`와 동일 구조)

```c
#define LCD_CMD_RDDSDR  0x0FU
#define LCD_SELF_DIAG_CHECKSUM_MISMATCH (1U << 0)

bool LCD_ReadSelfDiagResult(uint8_t *result)
{
  uint8_t command = LCD_CMD_RDDSDR;
  uint8_t tx_data[2] = {0U, 0U};
  uint8_t rx_data[2] = {0U, 0U};

  if (result == NULL)
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  LCD_Select();
  if (HAL_SPI_Transmit(&hspi2, &command, 1U, LCD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    LCD_Unselect();
    return LCD_Fail(LCD_ERROR_READ_COMMAND);
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  if (HAL_SPI_TransmitReceive(&hspi2,
                              tx_data,
                              rx_data,
                              sizeof(tx_data),
                              LCD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    LCD_Unselect();
    return LCD_Fail(LCD_ERROR_READ_DATA);
  }
  LCD_Unselect();
  *result = rx_data[1];   /* rx_data[0]은 dummy read, 버림 */
  return true;
}
```

### 4.3 `LCD_IsHealthy()`에 통합

```c
bool LCD_IsHealthy(void)
{
  uint8_t power_mode;
  uint8_t self_diag;

  if (!LCD_ReadPowerMode(&power_mode))
  {
    return false;
  }
  if ((power_mode & LCD_POWER_MODE_REQUIRED) != LCD_POWER_MODE_REQUIRED)
  {
    return LCD_Fail(LCD_ERROR_HEALTH_STATUS);
  }
  if (!LCD_ReadSelfDiagResult(&self_diag))
  {
    return false;
  }
  if ((self_diag & LCD_SELF_DIAG_CHECKSUM_MISMATCH) != 0U)
  {
    return LCD_Fail(LCD_ERROR_SELF_DIAG_STATUS);   /* enum에 새 값 추가 필요 */
  }
  return true;
}
```

`LCD_Error_t`에 `LCD_ERROR_SELF_DIAG_STATUS = 9U`를 추가해서 썼다(기존 값 0~8은
`LCD_ERROR_NONE`부터 `LCD_ERROR_HEALTH_STATUS`까지 그대로).

## 5. RP2350 bring-up 시 검증 순서 제안

1. 위 4.1 IFMODE 수정을 반영한 채로 초기화.
2. `LCD_ReadPowerMode` 먼저 읽어서 예상값(SLEEP_OUT+DISPLAY_ON 비트)이 나오는지 확인 —
   여기서부터 막히면 MISO 배선/타이밍(3.2절) 자체를 재점검.
3. RDID1/2/3을 한 번 읽어서 고정 상수로 캡처(3.3절) — 이후 재현성 확인 기준으로 사용.
4. RDDSDR을 읽어서 D0(checksum) 비트 확인. D7/D6 값도 같이 로그로 남겨서, 여러 번
   반복 측정해 이 두 비트가 실제로 어떤 조건에서 바뀌는지 실측으로 폴리서티를 확정할 것
   (데이터시트에 값 정의가 없었으므로).
5. 필요하면 RDDMADCTL(0Bh)도 추가해서 MY/MX/MV 회전 상태까지 직접 검증 — STM_B에서는
   시간 관계상 범위에서 뺐지만(사용자 요청: RDDPM+RDDSDR만), RP2350은 이 부분에 시간을
   더 쓸 수 있으면 우선순위 높음.

마지막 수정 날짜 : 2026.8.28
신규 변경 or 검증 사항이 있다면 날짜 update 할 것. 