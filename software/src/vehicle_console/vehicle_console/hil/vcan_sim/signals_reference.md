# vcan SIL Signals Reference (교육용 요약)

이 표는 `can_io.py`의 software-only 모델을 설명한다. 현재 조향 모델은
enable·position 입력과 단순 heartbeat를 사용하며, production driver의 encoder query
응답이나 외부 torque 제어를 재현하지 않는다. 실차 CAN 계약은
[Interfaces](../../../../../../docs/INTERFACES.md#8-steering-motor-can)를 따른다.

## 1. Motor command `0x0C01EFD0` (EFF, 8 bytes)

| Byte | 의미 | 엔디안 | 변환 |
|---|---|---|---|
| 0-1 | phase current limit | little | `A = raw * 0.1 - 3200` |
| 2-3 | target speed rpm | little | `rpm = raw - 32000` |
| 4 | control bits | - | bit0 = RUN, bit1 = SPEED |
| 5 | reserved | - | 0 |
| 6 | reserved | - | 0 |
| 7 | reserved | - | 모델의 명령 생성기는 0을 송신 |

## 2. Motor status1 `0x1801D0EF`

### handshake 단계

- payload = `55 55 55 55 55 55 55 55`

### normal status 단계

| Byte | 의미 | 엔디안 | 변환 |
|---|---|---|---|
| 0-1 | DC bus voltage | little | `V = raw * 0.1` |
| 2-3 | bus current | little | `A = raw * 0.1 - 3200` |
| 4-5 | phase current | little | `A = raw * 0.1 - 3200` |
| 6-7 | actual motor rpm | little | `rpm = raw - 32000` |

## 3. Motor status2 `0x1802D0EF`

| Byte | 의미 | 변환 |
|---|---|---|
| 0 | controller temp | `°C = raw - 40` |
| 1 | motor temp | `°C = raw - 40` |
| 2 | status bits | bit0 = RUN, bit1 = SPEED |
| 3 | error_group low byte | 24-bit fault 값의 bits 0..7 |
| 4 | error_group middle byte | bits 8..15 |
| 5 | error_group high byte | bits 16..23 |
| 6 | reserve | 0 |
| 7 | reserve | 0 |

## 4. Steering command `0x06000001`

### enable

```text
23 0D 20 01 00 00 00 00
```

### disable

```text
23 0C 20 01 00 00 00 00
```

### position write

```text
23 02 20 01 [scaled int32, big-endian]
```

여기서 `scaled = handle_deg / 360 * 10000`

## 5. Steering feedback `0x07000001`

- 0-1 byte: signed int16, big-endian
- 이 교육용 시뮬레이터에서는 **handle angle degree ≈ raw count** 로 보냄
- `lateral_ctrl`의 캘리브레이션 다항식이 적용되면 최종 wheel angle이 나옴
