뜯어봤습니다. 결론부터 말하면 **이건 리눅스에서 완전히 대체할 수 있습니다.** 생각보다 프로토콜도 단순합니다. V4.3 EXE가 .NET 어셈블리이고 PDB까지 같이 들어 있어서 `Connect`, `Write`, `Burn`, 수신 파서까지 거의 그대로 복원됐습니다.

그리고 지금 올려주신 2026-09-13 스크린샷에서 선택된 **`Not 170 Three generations`** 모드가 우리가 써야 할 **8바이트 설정 프로토콜**입니다.

### 핵심 프로토콜

RS232는 데이터시트 그대로 **115200, 8N1, parity 없음**입니다. 설정 포트도 TX/RX/GND를 쓰는 것이 맞습니다. 

Windows 프로그램에서 `Connect`를 누르면:

```text
FA 00 00 00 00 00 00 00
```

을 보냅니다.

그러면 KEYA가 설정값을 계속 다음 **4바이트 레코드**로 토해냅니다.

```text
[index] [data_hi] [data_lo] [bank]
```

`bank`는 실제 EXE 코드를 따라가 보니:

```text
0 = Param
1 = RAM
2 = ROM(EEPROM)
```

입니다.

즉 예를 들어 이런 응답이 왔다면:

```text
0D 00 96 02
```

해석은

```text
index 0x0D = 0013 Acceleration Time
value = 0x0096 = 150
bank = 2 = ROM
```

따라서 **EEPROM에 Acceleration=150 → 15.0초가 저장되어 있다**는 뜻입니다.

---

### 파라미터 하나를 RAM에 쓰는 명령

이 부분도 정확히 찾았습니다.

```text
F0 [index] [value_hi] [value_lo] 05 00 00 00
```

예를 들어 지금 사진의 Accel 150은:

```text
F0 0D 00 96 05 00 00 00
```

Accel을 `5 = 0.5초`로 바꾸려면:

```text
F0 0D 00 05 05 00 00 00
```

Accel을 `1 = 0.1초`로:

```text
F0 0D 00 01 05 00 00 00
```

Decel은 index 30(decimal) = `0x1E`이므로 현재 200은:

```text
F0 1E 00 C8 05 00 00 00
```

Decel 5로 만들면:

```text
F0 1E 00 05 05 00 00 00
```

입니다.

중요한 게 **여기까지는 RAM만 변경**합니다. Windows 프로그램의 각 행 옆 `Write` 버튼이 정확히 이 작업입니다.

---

### `Burn` 버튼도 찾았습니다

EEPROM에 영구 저장시키는 프레임은:

```text
FA 00 00 00 08 00 00 00
```

입니다.

즉 Windows에서 하던 과정이 사실상:

```text
Connect
FA 00 00 00 00 00 00 00

0013 = 5
F0 0D 00 05 05 00 00 00

0030 = 5
F0 1E 00 05 05 00 00 00

Burn
FA 00 00 00 08 00 00 00

V4.3 프로그램 기준 5초 대기 후 ROM readback 확인

전원 OFF → ON
```

이게 전부입니다.

체크섬도 없고 암호화도 없고 별도의 인증도 없습니다.

Disconnect는 그냥:

```text
00 00 00 00 00 00 00 00
```

을 보냅니다.

---

## PID 값의 인코딩도 풀렸습니다

여기서 아주 중요한 차이가 있습니다.

정수 파라미터와 PID 같은 소수 파라미터를 EXE가 다르게 처리합니다.

`0005~0012` PID 계수 등은 **×1000 fixed point**입니다.

따라서:

```text
Position Kp = 0.500

0.500 × 1000 = 500
500 = 0x01F4

→ F0 09 01 F4 05 00 00 00
```

Current Kp=2.000이라면:

```text
2.000 × 1000 = 2000 = 0x07D0

→ F0 05 07 D0 05 00 00 00
```

반면 Accel=150 같은 것은 그냥 integer `150`입니다.

EXE 내부 데이터 타입 테이블도 정확히 뽑혔습니다.

```text
정수 uint16:
0000~0004
0013, 0014
0017~0030

0.001 fixed-point:
0005~0012
0015, 0016
0031, 0032
0041, 0063
```

이건 상당히 큰 수확입니다.

---

## 그리고 `Rated Speed = 1000`도 하나 해결됐습니다

전에 제가 `1000`이 혹시 프로그램 내부에서 `/10` 해서 100 rpm인 게 아닐까 했는데, **그건 아닙니다.**

V4.3 소스를 보면 `0002 Rated Speed`는 명백히 **정수형 그대로 표시**합니다.

즉 스크린샷의

```text
Rated Speed
RAM = 1000
ROM = 1000
```

은 정말 파라미터 값 **1000**입니다.

다만 이것이 **출력축 물리속도 1000 rpm이라는 뜻은 아닙니다.**

오히려 실제 KY170 출력 사양과 맞지 않으므로, 이 값은 내부 BLDC rotor/reference speed이거나 감속 전 기준일 가능성이 상당히 있습니다. 이 부분은 펌웨어/감속비 관계를 더 확인해야 합니다.

하지만 적어도:

> GUI에서 1000을 띄우지만 실질적으로 100.0이라는 단순 표시 스케일링

은 아닙니다.

---

## `0014`도 코드에서 이름을 찾았습니다

영문 리소스에는 정확히:

```text
0014
Postion Err_H
Slow Down in Advance 20-100
```

이라고 박혀 있습니다.

그러니까 단순한 'Position'이라는 이름보다 정확히는 **Position Err_H**, 즉 위치오차가 일정 수준에 접근했을 때 미리 감속시키는 threshold 계열로 보는 게 자연스럽습니다.

다만 **20~100의 물리 단위가 무엇인지는 EXE에서도 정의하지 않습니다.** 중국어 리소스도 advance deceleration을 말할 뿐이라 여기서 임의로 도(deg)나 회전수로 단정하면 안 됩니다.

여러분 값은 `20`, 즉 UI상 최솟값입니다.

따라서 현재 설정에는 다음과 같은 긴 가감속 시간이 들어 있습니다.

```text
0013 Accel = 150 → 15.0 s
0030 Decel = 200 → 20.0 s
```

다만 이 값은 매뉴얼상 0 rpm과 정격속도 사이의 시간입니다. 절대위치 모드의 실제 조향 응답 지연 원인인지는 RAM 변경과 위치 응답 측정으로 따로 확인해야 합니다.

---

## 프로그램 안에 숨어 있는 설정 항목도 정리됐습니다

이 V4.3 GUI가 실제로 관리하는 주소는:

```text
0000 ~ 0032
0041
0063
```

입니다.

반면 여러분 정확한 모터 매뉴얼에는 `0033 steering zero`, `0034 CAN heartbeat`도 나오죠. 

그런데 **V4.3 Setup GUI 코드에서는 0033/0034를 설정 파라미터로 등록하지 않습니다.**

따라서 당장은 0033/0034에

```text
F0 21 ...
F0 22 ...
```

같은 것을 억지로 보내면 안 됩니다. 프로토콜 형식상 그럴 듯해 보여도 펌웨어가 같은 주소공간으로 처리한다고 확인된 게 아닙니다.

특히 0033은 매뉴얼도 수정 금지입니다.

---

## Windows 프로그램이 `Burn 성공`을 확인하는 방법도 좀 웃깁니다

EEPROM Burn에 별도의 제대로 된 ACK 검증 로직이 없습니다.

대략:

```text
Burn frame 전송
        ↓
버튼 빨갛게
        ↓
timer
        ↓
5초 타이머
        ↓
"Burned successfully"
```

입니다.

매뉴얼은 Burn에 약 3초가 걸린다고 설명하지만, 이 V4.3 EXE의 `WriteDevtimer.Interval`은 5000 ms입니다. 타이머 콜백은 ACK를 검사하지 않고 완료 메시지를 표시합니다.

리눅스 버전은 다음처럼 확인합니다.

```text
Write
→ RAM readback 확인

Burn
→ 5초 대기
→ ROM readback 확인

→ power cycle
→ 다시 ROM 확인
```

으로 하는 게 좋습니다.

---

# 리눅스 CLI도 만들어놨습니다

바로 테스트할 수 있게 프로토타입을 만들었습니다.

[KEYA V4.3 Linux 설정 CLI](./keya_v43_config.py)와 [PySide6/QML GUI](./keya_v43_gui.py)를 같은 통신 코어 위에 구현했습니다. 사용법은 [README](./README.md)에 정리되어 있습니다.

`pyserial`만 필요합니다.

```bash
sudo apt install python3-serial
```

USB-RS232를 꽂으면 보통:

```bash
ls /dev/ttyUSB*
```

로 `/dev/ttyUSB0`가 잡힐 겁니다.

제조사 매뉴얼은 전원 투입 순간 설정을 읽으면 파라미터가 초기값으로 리셋될 수 있다고 경고합니다. 컨트롤러의 정상 기동이 끝난 뒤 설정 세션을 시작해야 하며, 읽기만 했더라도 세션 종료 후 전원을 껐다 켜야 합니다.

먼저 정상 기동을 확인한 뒤 dump합니다.

```bash
python3 keya_v43_config.py \
  --port /dev/ttyUSB0 \
  dump
```

그러면 목표는 이런 식으로 뜨는 겁니다.

```text
  No  Name                                      Param          RAM          ROM
--------------------------------------------------------------------------------
0005  Current Kp                                2.000        2.000        2.000
0006  Current Ki                                0.010        0.010        0.010
...
0013  Acceleration Time                           150          150          150
0014  Position Err_H                               20           20           20
...
0030  Deceleration Time                           200          200          200
```

**이 dump가 스크린샷과 일치하면 사실상 프로토콜 복제 성공입니다.**

그다음 Accel을 RAM에서만 5로 시험하려면:

```bash
python3 keya_v43_config.py \
  --port /dev/ttyUSB0 \
  write accel 5
```

이 명령은 **EEPROM에는 쓰지 않습니다.**

실제로 송신되는 건:

```text
F0 0D 00 05 05 00 00 00
```

이고 프로그램이 RAM readback까지 확인합니다.

Decel도:

```bash
python3 keya_v43_config.py \
  --port /dev/ttyUSB0 \
  write decel 5
```

영구 저장을 정말 하려면 별도로:

```bash
python3 keya_v43_config.py \
  --port /dev/ttyUSB0 \
  burn --yes
```

해야 합니다.

일부러 `write`와 `burn`을 분리해 놨습니다. PID 잘못 하나 썼다가 바로 EEPROM까지 태우는 일을 피하려고요.

---

### 물리 연결에서 이것만 주의하시면 됩니다

이건 **TTL UART가 아니라 진짜 RS232**입니다.

따라서:

```text
USB-TTL CP2102
FT232 TTL
```

을 바로 KEYA TX/RX에 박으면 안 됩니다.

**USB ↔ RS232 레벨 변환기**를 쓰셔야 합니다.

매뉴얼 기준:

```text
KEYA TX  → DB9 pin 2
KEYA RX  → DB9 pin 3
KEYA GND → DB9 pin 5
```

입니다. 

---

`Control Ways=2(CAN)` 상태에서도 스크린샷의 RS232 설정 화면이 데이터를 읽은 사실은 확인됩니다. 다만 이것만으로 주행 중 CAN 제어와 RS232 설정을 동시에 사용해도 된다고 입증되지는 않습니다. 설정 세션은 정차 상태에서 별도 작업으로 다룹니다.

첫 장치 시험은 정상 기동 후 `dump`로 Param/RAM/ROM을 읽고, 설정 세션 종료와 전원 재투입까지 수행하는 것입니다. Windows 프로그램 대체 여부는 RAM 쓰기 readback, Burn 뒤 ROM readback, 전원 재투입 후 유지까지 각각 확인해야 합니다.
