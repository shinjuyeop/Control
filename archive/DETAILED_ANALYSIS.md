# Control Stack 상세 기술 분석 문서

## 개요
본 문서는 자율주행 차량 Control Stack의 각 패키지를 상세하게 분석한 기술 문서입니다. 실제 코드베이스와 README.md 간의 차이점, 각 패키지의 구조, 로직, 입출력을 명확히 정리했습니다.

## 주요 발견사항

### README와 실제 코드베이스 차이점
1. **steering_monitor_cpp 패키지 존재**: README에 문서화되지 않았으나 실제로 존재하는 모니터링 유틸리티
2. **EV.ino 펌웨어 존재**: README에 ArduinoMEGA.ino만 언급되었으나 EV.ino도 존재
3. **토픽명 변경**: 일관된 네이밍 체계로 업데이트됨

### 토픽 네이밍 변경사항
- `/target_rpm_auto` → `/cmd/rpm` (자율주행 RPM 명령)
- `/current_speed` → `/ctrl/speed` (현재 속도 피드백, 이제 m/s 단위)
- `/steering_command` → `/cmd/steer` (조향 명령)
- `/encoder_angle` → `/ctrl/steer` (조향 피드백)
- `/throttle_data` 유지 (변경 없음)
- `target_rpm` 유지 (슬래시 없음, rpm_mux 출력)

---

## 패키지별 상세 분석

## 1. throttle_msgs 패키지

### 개요
- **역할**: ROS2 커스텀 메시지 정의 패키지
- **타입**: 메시지 인터페이스 패키지
- **빌드 시스템**: ament_cmake + rosidl

### 구조
```
throttle_msgs/
├── msg/
│   └── ThrottleData.msg
├── CMakeLists.txt
└── package.xml
```

### 메시지 정의 (ThrottleData.msg)
```
int16 rpm    # 현재 RPM (-2000 ~ 2000)
bool estop   # 비상정지 상태 (True = 활성)
bool asms    # AUTO/MANUAL 스위치 (True = Manual)
```

### 특징
- **단순한 구조**: 3개 필드만 정의
- **부호있는 RPM**: 음수값으로 후진 표현
- **이중 용도 플래그**: asms가 MANUAL 모드를 나타냄

### 의존성
- std_msgs (ROS2 기본 메시지)
- rosidl_default_generators (메시지 생성)

---

## 2. throttle_serial_py 패키지

### 개요
- **역할**: Arduino와 ROS2 간 시리얼 통신 브릿지
- **언어**: Python
- **주요 노드**: throttle_node, rpm_mux

### 2.1 throttle_node.py

#### 기능
- Arduino에서 시리얼 패킷 수신 및 파싱
- ThrottleData 메시지로 변환하여 발행

#### 입력
- **시리얼 포트**: /dev/ttyACM1 (하드코딩) → 자동 탐지로 개선됨
- **패킷 형식**: [0xAA][RPM_HIGH][RPM_LOW][FLAGS]
- **통신 속도**: 115200 bps

#### 출력
- **토픽**: `/throttle_data` (ThrottleData)
- **주기**: 100Hz

#### 핵심 로직
```python
# 포트 자동 탐지 (개선된 버전)
1. /dev/serial/by-id/* 우선 탐색
2. Arduino Mega, CH340, CP210x 등 우선순위 부여
3. 부트로더 대기 (2초)
4. 헤더(0xAA) 프로브로 올바른 포트 확인

# 패킷 파싱
- 16비트 RPM: Big-endian 형식
- FLAGS 비트맵:
  - bit 0: Manual 모드
  - bit 1: E-STOP
  - bit 2: Reverse
```

#### 주요 개선사항
- **포트 자동 탐지**: VID/PID 기반 우선순위
- **재연결 로직**: 6초 간격으로 재시도
- **데드밴드 적용**: -2 < RPM < 2 → 0 처리

### 2.2 rpm_mux.py

#### 기능
- AUTO/MANUAL/ESTOP 모드에 따른 RPM 선택
- 멀티플렉싱 로직 구현

#### 입력
- `/throttle_data` (ThrottleData): 수동 RPM + 모드 정보
- `/cmd/rpm` (Int32): 자율주행 목표 RPM (**업데이트된 토픽명**)

#### 출력
- `/target_rpm` (Int32): 최종 목표 RPM
- **주기**: 100Hz (README에는 20Hz로 오기)

#### 핵심 로직
```python
if ESTOP:
    output = 0
elif MANUAL:
    output = manual_rpm
else:
    output = auto_rpm
```

---

## 3. motor_control_cpp 패키지

### 개요
- **역할**: CAN 통신 기반 모터 제어
- **언어**: C++
- **노드**: speed_publisher, can_speed_sender, can_speed_receiver

### 3.1 speed_publisher.cpp

#### 기능
- 자율주행 목표 RPM 생성 및 발행

#### 출력
- **토픽**: `/cmd/rpm` (Int32) (**업데이트된 토픽명**)
- **주기**: 100Hz (10ms 타이머)
- **기본값**: 500 RPM

#### 특징
- **동적 파라미터**: `rpm` 파라미터 실시간 변경 가능
- **콜백 기반**: SetParametersCallback 사용

### 3.2 can_speed_sender.cpp

#### 기능
- MCU와 CAN 통신으로 모터 제어 명령 전송

#### 입력
- **토픽**: `/target_rpm` (Int32)
- **CAN**: MCU 핸드셰이크 (0x1801D0EF)

#### 출력
- **CAN ID**: 0x0C01EFD0 (EFF)
- **주기**: 50ms

#### CAN 프로토콜
```cpp
// 핸드셰이크
MCU → ROS: 0x1801D0EF [0x55 × 8]
ROS → MCU: 0x0C01EFD0 [0xAA × 8]

// 제어 프레임 (8 바이트)
data[0:1]: Phase Current (int16, Little-endian)
          - 전진: +180A → 0x9470
          - 후진: -180A → 0x6B90
data[2:3]: RPM (int16, Little-endian, +32000 오프셋)
data[4]:   Mode (0x03 = RUNNING + SPEED)
data[5:6]: Reserved (0x00)
data[7]:   Life Counter (순증)
```

#### 핵심 로직
- **위상 전류 부호 결정**: RPM ≥ 0 ? +180A : -180A
- **변환식**: 
  - phase_val = (current_a - (-3200)) / 0.1
  - rpm_val = rpm - (-32000)

### 3.3 can_speed_receiver.cpp (**주요 업데이트**)

#### 기능
- MCU로부터 실시간 피드백 수신 및 파싱

#### 입력
- **CAN ID**: 
  - 0x1801D0EF (Part1: 전압/전류/RPM)
  - 0x1802D0EF (Part2: 온도/상태/에러)

#### 출력 (**중요 변경사항**)
- **토픽**: `/ctrl/speed` (Float32) - **RPM에서 m/s로 변경**
- **로그**: 전압, 전류, 온도, 에러 상태

#### 핵심 변환 로직 (**신규 기능**)
```cpp
// m/s 변환 (Arduino와 동일한 파라미터 사용)
const float kTireDiamCm = 47.0f;   // cm
const float kGearRatio  = 4.60f;   // 감속비

// wheel_rpm = motor_rpm / kGearRatio
const float wheel_rpm  = 2.0f * static_cast<float>(rpm) / kGearRatio;
// v [m/s] = wheel_rpm * (π * D_m) / 60
const float speed_mps  = wheel_rpm * (3.1415926f * D_m) / 60.0f;
```

#### 피드백 파싱
```cpp
// Part1 (0x1801D0EF)
data[0:1]: Bus Voltage (uint16, 0.1V/bit)
data[2:3]: Bus Current (int16, 0.1A/bit, +3000 오프셋)
data[4:5]: Phase Current (int16, 0.1A/bit, +3000 오프셋)
data[6:7]: Speed (int16, 1rpm/bit, +32000 오프셋)

// Part2 (0x1802D0EF)
data[0]: Controller Temp (°C = raw - 40)
data[1]: Motor Temp (°C = raw - 40)
data[2]: Status (bit0: RUN, bit1: SpeedMode)
data[3]: Error bits
data[4]: Fault1 bits
data[5]: Fault2 bits
```

---

## 4. steering_control 패키지 (**주요 업데이트**)

### 개요
- **역할**: 조향 모터 제어 및 엔코더 피드백
- **언어**: C++
- **CAN 인터페이스**: can1 (기본값)

### 4.1 steering_control_node.cpp (**안전 기능 강화**)

#### 기능
- 조향 명령을 CAN 신호로 변환하여 전송
- **신규**: ESTOP/MANUAL 모드 처리 및 명령 버퍼링

#### 입력
- **토픽**: `/cmd/steer` (Float32, degrees) (**업데이트된 토픽명**)
- **토픽**: `/throttle_data` (ThrottleData): ESTOP/모드 상태

#### 출력
- **CAN ID**: 0x06000001 (EFF)
- **프로토콜**: SDO (Service Data Object)

#### 핵심 안전 로직 (**신규 기능**)
```cpp
// ESTOP/MANUAL 중 명령 버퍼링
if (estop_ || is_manual_) {
    pending_scaled_ = calculated_value;
    has_pending_ = true;
    return; // 전송 차단
}

// MANUAL → AUTO 전환 시
if (!is_manual_ && last_is_manual_) {
    enable_motor(); // 모터 재활성화
    nanosleep(50ms); // 안정화 대기
    if (has_pending_) {
        send_position_scaled(pending_scaled_); // 버퍼된 명령 전송
    }
}
```

#### 핵심 변환 로직
```cpp
// 2차 역함수 (y: 입력각도 → x: 모터각도)
x = -0.00125f * y² + 3.635f * y - 3.875f

// 스케일링 (360도 = 10000 단위)
scaled_val = (x / 360.0f) * 10000.0f

// CAN 프레임
data[0:3]: [0x23, 0x02, 0x20, 0x01]  // SDO 헤더
data[4:7]: scaled_val (int32, Big-endian)
```

#### 초기화
- Enable 명령: `[0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00]`

### 4.2 steering_feedback_node.cpp (**구성 가능 엔코더**)

#### 기능
- 조향 모터 엔코더 값 수신 및 각도 변환
- **신규**: 다중 CAN 인터페이스 및 구성 가능한 엔코더 파싱

#### 입력
- **CAN ID**: 0x07000001 (Heartbeat/Encoder)
- **인터페이스**: can1,can0 (기본값, 구성 가능)

#### 출력
- **토픽**: `/ctrl/steer` (Float32, degrees) (**업데이트된 토픽명**)
- **주기**: 20ms (50Hz)

#### 구성 가능한 엔코더 파싱 (**신규 기능**)
```cpp
// 파라미터 (런타임 구성 가능)
can_ifaces: "can1,can0"          // 다중 인터페이스
feedback_id: 0x07000001          // CAN ID
value_offset: 0                  // 데이터 시작 위치
value_bits: 16                   // 16/32-bit 선택
big_endian: true                 // 엔디안 선택
counts_per_rev: 360              // 해상도

// 16/32-bit 지원
if (value_bits_ == 16) {
    uint16_t u = value_big_endian_ ? 
        (frame.data[offset] << 8) | frame.data[offset+1] :
        (frame.data[offset+1] << 8) | frame.data[offset];
    out_counts = static_cast<int16_t>(u);
}
```

### 4.3 steering_command_publisher_node.cpp

#### 기능
- 테스트용 조향 명령 생성

#### 출력
- **토픽**: `/cmd/steer` (Float32) (**업데이트된 토픽명**)
- **시퀀스**: [0°, 10°, -15°, 20°, 0°] 순환
- **주기**: 1Hz

---

## 5. speed_monitor_cpp 패키지 (문서화 누락)

### 개요
- **역할**: 속도 모니터링 유틸리티
- **언어**: C++
- **노드**: speed_monitor_node

### 기능
- RPM/속도 토픽 구독 및 콘솔 출력
- 디버깅 및 모니터링 용도

### 입력
- **토픽**: `/ctrl/speed` (Float32) (**업데이트된 토픽명, m/s**)
- **파라미터**: `topic` (토픽명 변경 가능)

### 출력
- **콘솔 로그**: "Speed: {value} m/s"

---

## 6. steering_monitor_cpp 패키지 (**신규 패키지**)

### 개요
- **역할**: 조향각 모니터링 유틸리티 (speed_monitor_cpp와 유사)
- **언어**: C++
- **노드**: steering_monitor_node

### 기능
- 조향각 피드백 및 명령 모니터링
- 통계 분석 (최소/최대값, 이동평균, 주기)

### 입력
- **토픽**: `/ctrl/steer` (Float32) - 피드백 모니터링
- **토픽**: `/cmd/steer` (Float32) - 명령 모니터링

### 출력
- **콘솔 로그**: "Angle={현재값} deg | Avg={평균} deg | Min={최소} Max={최대} | ~{주기} Hz"

### 특징
- **이동평균 계산**: 9개 샘플 윈도우
- **주기 추정**: 지수 평활화 (0.9 * 이전값 + 0.1 * 현재값)
- **쓰로틀링**: 500ms 간격으로 출력 제한
- **통계 추적**: 최소/최대값 지속 추적

---

## 7. firmware 패키지

### 7.1 ArduinoMEGA.ino

#### 개요
- **타겟**: Arduino Mega 2560
- **역할**: VCU (Vehicle Control Unit)
- **기능**: 센서 입력, 속도 계산, 시리얼 통신, VESS 제어

#### 핀 매핑
```cpp
THROTTLE_PIN  A7   // 0-5V 아날로그 입력
ESTOP1_PIN    52   // E-STOP 1
ESTOP2_PIN    53   // E-STOP 2
ASMS_PIN      A8   // AUTO/MANUAL 스위치
GEAR_PIN      A9   // 기어 레버 (FWD/REV)
SERVO_PIN     46   // 브레이크 서보
VESS_PIN      9    // 가상 엔진음 PWM
RF_ESTOP_PIN  A12  // 무선 E-STOP
HALL_PIN      2    // Hall 센서 (속도)
```

#### 시리얼 패킷 포맷
```cpp
[0xAA][RPM_HIGH][RPM_LOW][FLAGS]
- 헤더: 0xAA
- RPM: 16비트 Big-endian (0-2000)
- FLAGS:
  bit 0: Manual (1=MANUAL, 0=AUTO)
  bit 1: E-STOP (1=활성)
  bit 2: Reverse (1=후진)
```

#### 핵심 로직
1. **스로틀 변환**:
   - ADC 읽기 (8회 평균)
   - 전압 변환: 0.8V~4.4V → 0~2000 RPM
   - 데드밴드: < 1.5 RPM → 0

2. **속도 계산**:
   - Hall 센서 인터럽트 카운트
   - PPR=6, 타이어 직경=47cm, 기어비=4.60
   - 500ms 간격으로 km/h 계산

3. **VESS 제어**:
   - 속도 비례 PWM (1600~2000μs)
   - 0 km/h → 1600μs (아이들)
   - 100 km/h → 2000μs (최대)

4. **안전 기능**:
   - PCINT 인터럽트로 즉각 응답
   - 비상정지 시 브레이크 서보 작동 (45°)

### 7.2 EV.ino

#### 개요
- **타겟**: Arduino (범용)
- **역할**: 속도계 + 랩타이머 + VESS
- **특징**: 스탠드얼론 동작

#### 주요 기능
1. **속도 측정**: ArduinoMEGA와 동일한 방식
2. **랩타이머**: 
   - 버튼으로 시작/랩 기록
   - 3초 길게 누르면 리셋
   - 베스트 랩 추적

3. **LCD 디스플레이** (20x4 I2C):
   ```
   SPEED: XX.X km/h
   TIME : MM:SS
   BEST : MM:SS(+XX)
   LAP  : N
   ```

4. **백라이트 효과**: 랩 기록 시 깜빡임

---

## 8. 실행 순서 검증

### README.md 실행 명령 검토

#### 올바른 부분
모든 실행 명령이 `ros2 run` 형식으로 올바르게 작성됨:
```bash
ros2 run throttle_serial_py throttle_node          ✓
ros2 run motor_control_cpp speed_publisher         ✓
ros2 run throttle_serial_py rpm_mux                ✓
ros2 run motor_control_cpp can_speed_sender        ✓
ros2 run motor_control_cpp can_speed_receiver      ✓
ros2 run steering_control steering_control_node    ✓
ros2 run steering_control steering_feedback_node   ✓
# 추가 (문서화되지 않음)
ros2 run speed_monitor_cpp speed_monitor_node      ✓
ros2 run steering_monitor_cpp steering_monitor_node ✓
```

#### Launch 파일 부재
- Launch 파일이 없어 각 노드를 개별 실행해야 함
- 개선 제안: launch 파일 생성으로 일괄 실행 가능

### 의존성 순서 분석

#### 올바른 실행 순서
1. **throttle_node** (먼저): Arduino 데이터 수신
2. **speed_publisher** (동시 가능): 자율 RPM 생성
3. **rpm_mux** (이후): 1,2의 데이터 필요
4. **can_speed_sender** (마지막): 3의 출력 필요

#### 조향 제어 (독립적)
- steering_control_node와 steering_feedback_node는 독립 실행 가능

---

## 9. 주요 이슈 및 개선 제안

### 발견된 이슈

#### 1. 문서화 불일치
- **누락된 패키지**: speed_monitor_cpp, steering_monitor_cpp가 README에 없음
- **주기 불일치**: rpm_mux 실제 100Hz, 문서 20Hz

#### 2. 토픽 네이밍 일관성 개선
- **개선됨**: 일관된 `/cmd/` 및 `/ctrl/` 네임스페이스 적용
- **명령 토픽**: `/cmd/rpm`, `/cmd/steer`
- **제어 피드백**: `/ctrl/speed`, `/ctrl/steer`

#### 3. 하드코딩 문제 (부분 해결)
- ~~시리얼 포트 /dev/ttyACM1 하드코딩~~ → 자동 탐지로 개선됨
- CAN 인터페이스 기본값 can0, can1 (파라미터로 변경 가능)

### 개선 제안

#### 1. Launch 파일 생성
```python
# motor_control.launch.py 예시
def generate_launch_description():
    return LaunchDescription([
        Node(package='throttle_serial_py', executable='throttle_node'),
        Node(package='motor_control_cpp', executable='speed_publisher'),
        Node(package='throttle_serial_py', executable='rpm_mux'),
        Node(package='motor_control_cpp', executable='can_speed_sender'),
    ])
```

#### 2. 파라미터 파일 활용
```yaml
# config/motor_control.yaml
throttle_node:
  ros__parameters:
    port: 'auto'
    baud: 115200
    boot_wait: 2.0

speed_publisher:
  ros__parameters:
    rpm: 500
```

#### 3. 네임스페이스 분리
```bash
ros2 run --ros-args -r __ns:=/motor motor_control_cpp can_speed_sender
ros2 run --ros-args -r __ns:=/steering steering_control steering_control_node
```

#### 4. 상태 모니터링 노드 추가
- 시스템 상태 종합 모니터링
- 에러 검출 및 알림
- 데이터 로깅

---

## 10. 하드웨어 연결 정보

### Arduino Mega 2560 핀 연결

#### 아날로그 입력
| 핀 | 기능 | 전압 범위 | 설명 |
|-----|------|-----------|------|
| A7 | THROTTLE_PIN | 0-5V | 스로틀 페달 전압 입력 |
| A8 | ASMS_PIN | 0-5V | AUTO/MANUAL 모드 스위치 (HIGH=Manual) |
| A9 | GEAR_PIN | 0-5V | 기어 레버 (HIGH=Reverse) |
| A12 | RF_ESTOP_PIN | 0-5V | 무선 비상정지 (LOW=활성) |

#### 디지털 입력
| 핀 | 기능 | 풀업 | 설명 |
|-----|------|------|------|
| D2 | HALL_PIN | 내부 풀업 | Hall 센서 (속도 측정, RISING 인터럽트) |
| D52 | ESTOP1_PIN | 내부 풀업 | 비상정지 버튼 1 (LOW=활성) |
| D53 | ESTOP2_PIN | 내부 풀업 | 비상정지 버튼 2 (LOW=활성) |

#### PWM 출력
| 핀 | 기능 | PWM 범위 | 설명 |
|-----|------|-----------|------|
| D9 | VESS_PIN | 1000-2000μs | 가상 엔진음 서보 (ESS-DUAL+) |
| D46 | SERVO_PIN | 각도 제어 | 브레이크 서보 (45°=제동, 80°=해제) |

#### 통신
| 인터페이스 | 핀 | 설정 | 용도 |
|------------|-----|------|------|
| Serial | TX0/RX0 | 115200 bps | ROS2 통신 |
| I2C | SDA/SCL | 0x27 | LCD 디스플레이 (20x4) |

### CAN 버스 연결

#### 물리적 연결
```
CAN0 (모터 제어)
├── CANH ─── 120Ω 종단저항 ─── MCU CANH
├── CANL ─── 120Ω 종단저항 ─── MCU CANL
└── GND  ─────────────────────── MCU GND

CAN1 (조향 제어)  
├── CANH ─── 120Ω 종단저항 ─── Steering Motor CANH
├── CANL ─── 120Ω 종단저항 ─── Steering Motor CANL
└── GND  ─────────────────────── Steering Motor GND
```

#### CAN 설정
- **비트레이트**: 250 kbps
- **종단저항**: 양 끝단에 120Ω (총 60Ω)
- **케이블**: 차폐 트위스트 페어
- **최대 길이**: 40m @ 250kbps

### 전원 요구사항

| 컴포넌트 | 전압 | 전류 | 비고 |
|----------|------|------|------|
| Arduino Mega | 7-12V | 200mA | USB 또는 배럴잭 |
| LCD I2C | 5V | 50mA | Arduino 5V 핀 |
| Hall 센서 | 5V | 10mA | Arduino 5V 핀 |
| 서보 모터 | 5-6V | 500mA | 별도 전원 권장 |
| CAN 트랜시버 | 3.3V/5V | 70mA | 시스템 전원 |

---

## 11. 파라미터 상세 목록

### throttle_serial_py 패키지

#### throttle_node 파라미터
| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| port | string | 'auto' | 시리얼 포트 ('auto' 또는 '/dev/ttyACM0' 등) |
| baud | int | 115200 | 시리얼 통신 속도 |
| boot_wait | float | 2.0 | 포트 오픈 후 부트로더 대기 시간 (초) |
| probe_window | float | 3.0 | 헤더 탐지 대기 시간 (초) |
| read_timeout | float | 0.05 | 일반 읽기 타임아웃 (초) |

**파라미터 설정 예시:**
```bash
ros2 run throttle_serial_py throttle_node --ros-args \
  -p port:=/dev/ttyACM1 \
  -p baud:=115200 \
  -p boot_wait:=3.0
```

### motor_control_cpp 패키지

#### speed_publisher 파라미터
| 파라미터 | 타입 | 기본값 | 범위 | 설명 |
|----------|------|--------|------|------|
| rpm | int | 500 | 0-2000 | 자율주행 목표 RPM |

**동적 파라미터 변경:**
```bash
# 실행 시 설정
ros2 run motor_control_cpp speed_publisher --ros-args -p rpm:=800

# 실행 중 변경
ros2 param set /speed_publisher rpm 1000
```

### steering_control 패키지 (**신규 파라미터**)

#### steering_feedback_node 파라미터
| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| can_ifaces | string | "can1,can0" | 모니터링할 CAN 인터페이스 목록 |
| feedback_id | int64 | 0x07000001 | 엔코더 피드백 CAN ID |
| extended_id | bool | true | 확장 프레임 사용 여부 |
| value_offset | int | 0 | 엔코더 값 바이트 오프셋 |
| value_bits | int | 16 | 엔코더 해상도 (16/32-bit) |
| big_endian | bool | true | 엔디안 설정 |
| counts_per_rev | int | 360 | 회전당 카운트 수 |
| publish_topic | string | "/ctrl/steer" | 발행 토픽명 |
| poll_period_ms | int | 20 | 폴링 주기 (ms) |

### speed_monitor_cpp 패키지

#### speed_monitor_node 파라미터
| 파라미터 | 타입 | 기본값 | 설명 |
|----------|------|--------|------|
| topic | string | '/ctrl/speed' | 모니터링할 토픽명 |

**사용 예시:**
```bash
ros2 run speed_monitor_cpp speed_monitor_node --ros-args \
  -p topic:=/cmd/rpm
```

### Arduino 펌웨어 상수

#### 속도 계산 파라미터
| 상수 | 값 | 단위 | 설명 |
|------|-----|------|------|
| PPR | 6 | pulses/rev | Hall 센서 회전당 펄스 |
| TIRE_DIAM_CM | 47.0 | cm | 타이어 직경 |
| TRANS_RATIO | 4.60 | - | 기어비 |
| SPD_INTERVAL | 500 | ms | 속도 계산 주기 |

#### 스로틀 매핑
| 상수 | 값 | 단위 | 설명 |
|------|-----|------|------|
| V_MIN | 0.8 | V | 최소 스로틀 전압 |
| V_MAX | 4.4 | V | 최대 스로틀 전압 |
| RPM_MIN | 0.0 | RPM | 최소 RPM |
| RPM_MAX | 2000.0 | RPM | 최대 RPM |

#### VESS (가상 엔진음)
| 상수 | 값 | 단위 | 설명 |
|------|-----|------|------|
| SERVO_IDLE_US | 1600 | μs | 0 km/h PWM |
| SERVO_MAX_US | 2000 | μs | 최고속 PWM |
| KMH_MAX | 100.0 | km/h | 최대 속도 |

---

## 12. 에러 코드 및 상태 정의

### MCU 에러 비트 (0x1802D0EF data[3])

| 비트 | 에러명 | 설명 | 대응 방법 |
|------|--------|------|-----------|
| bit 0 | Overcurrent | 과전류 감지 | 부하 감소, 모터 점검 |
| bit 1 | Overload | 과부하 상태 | 가속 프로파일 조정 |
| bit 2 | Overvoltage | 과전압 | 전원 공급 점검 |
| bit 3 | Undervoltage | 저전압 | 배터리 충전, 전원 점검 |
| bit 4 | Controller Overheat | 컨트롤러 과열 | 냉각 대기, 방열판 점검 |
| bit 5 | Motor Overheat | 모터 과열 | 냉각 대기, 부하 감소 |
| bit 6 | Motor Stalled | 모터 정지 | 기계적 장애물 제거 |
| bit 7 | Motor Out of phase | 위상 손실 | 모터 케이블 연결 점검 |

### MCU Fault1 비트 (0x1802D0EF data[4])

| 비트 | 결함명 | 설명 | 심각도 |
|------|--------|------|---------|
| bit 0 | Motor Sensor | 모터 센서 오류 | 높음 |
| bit 1 | Motor AUX Sensor | 보조 센서 오류 | 중간 |
| bit 2 | Encoder Misaligned | 엔코더 정렬 불량 | 높음 |
| bit 3 | Anti-Runaway Engaged | 폭주 방지 작동 | 높음 |
| bit 4 | Main Accelerator | 메인 가속기 오류 | 높음 |
| bit 5 | AUX Accelerator | 보조 가속기 오류 | 낮음 |
| bit 6 | Pre-charge | 프리차지 실패 | 높음 |
| bit 7 | DC Contactor | DC 접촉기 오류 | 높음 |

### MCU Fault2 비트 (0x1802D0EF data[5])

| 비트 | 결함명 | 설명 | 심각도 |
|------|--------|------|---------|
| bit 0 | Power valve | 파워 밸브 오류 | 중간 |
| bit 1 | Current Sensor | 전류 센서 오류 | 높음 |
| bit 2 | Auto-tune | 자동 튜닝 실패 | 낮음 |
| bit 3 | RS485 | RS485 통신 오류 | 중간 |
| bit 4 | CAN | CAN 통신 오류 | 높음 |
| bit 5 | Software | 소프트웨어 오류 | 높음 |
| bit 6-7 | Reserved | 예약됨 | - |

### MCU 상태 비트 (0x1802D0EF data[2])

| 비트 | 상태 | 값 | 설명 |
|------|------|-----|------|
| bit 0 | RUN | 0=정지, 1=동작 | 모터 구동 상태 |
| bit 1 | SpeedMode | 0=토크, 1=속도 | 제어 모드 |
| bit 2-7 | Reserved | - | 미사용 |

### Arduino 모드 상태

| 모드 | enum 값 | 조건 | 동작 |
|------|---------|------|------|
| AS_EMERGENCY | 2 | ESTOP 활성 | 브레이크 작동, RPM=0 |
| AS_MANUAL | 1 | ASMS=HIGH | 수동 제어, 페달 입력 |
| AS_AUTO | 0 | ASMS=LOW | 자율 제어, ROS2 명령 |

### 시리얼 패킷 FLAGS 비트

| 비트 | 플래그 | 값 | 의미 |
|------|--------|-----|------|
| bit 0 | Manual | 1=Manual, 0=Auto | 제어 모드 |
| bit 1 | E-STOP | 1=활성, 0=비활성 | 비상정지 상태 |
| bit 2 | Reverse | 1=후진, 0=전진 | 기어 상태 |
| bit 3-7 | Reserved | 0 | 미사용 |

### CAN 프레임 Life Counter

| 필드 | 위치 | 범위 | 용도 |
|------|------|------|------|
| life_counter | data[7] | 0-255 | 통신 정상 확인, 순환 증가 |

---

## 13. 업데이트된 토픽 맵

### 명령 토픽 (`/cmd/`)
| 토픽명 | 타입 | 발행자 | 구독자 | 설명 |
|--------|------|--------|--------|------|
| `/cmd/rpm` | Int32 | speed_publisher | rpm_mux | 자율주행 RPM 명령 |
| `/cmd/steer` | Float32 | 외부 플래너 | steering_control_node | 조향 명령 (degrees) |

### 제어 피드백 토픽 (`/ctrl/`)
| 토픽명 | 타입 | 발행자 | 구독자 | 설명 |
|--------|------|--------|--------|------|
| `/ctrl/speed` | Float32 | can_speed_receiver | speed_monitor_node | 현재 속도 (m/s) |
| `/ctrl/steer` | Float32 | steering_feedback_node | steering_monitor_node | 현재 조향각 (degrees) |

### 내부 토픽 (네임스페이스 없음)
| 토픽명 | 타입 | 발행자 | 구독자 | 설명 |
|--------|------|--------|--------|------|
| `throttle_data` | ThrottleData | throttle_node | rpm_mux, steering_control_node | Arduino 센서 데이터 |
| `target_rpm` | Int32 | rpm_mux | can_speed_sender | 최종 RPM 명령 |

---

## 결론

본 Control Stack은 기능적으로 완성도가 높으며, 하드웨어와 소프트웨어 통합이 잘 구현되어 있습니다. 

### 주요 업데이트 사항
1. **새 패키지 추가**: steering_monitor_cpp 모니터링 유틸리티
2. **토픽 네이밍 일관성**: `/cmd/` 및 `/ctrl/` 네임스페이스 적용
3. **단위 표준화**: 속도 피드백이 m/s로 통일
4. **안전성 강화**: steering_control에서 ESTOP/MANUAL 모드 처리
5. **구성 가능성 개선**: steering_feedback에서 엔코더 파싱 설정 가능

### 패키지 요약 (총 7개)
1. **throttle_msgs**: 메시지 정의
2. **throttle_serial_py**: Arduino 통신 브릿지
3. **motor_control_cpp**: 모터 제어 및 CAN 통신
4. **steering_control**: 조향 제어 및 피드백
5. **speed_monitor_cpp**: 속도 모니터링 유틸리티
6. **steering_monitor_cpp**: 조향 모니터링 유틸리티 (**신규**)
7. **firmware**: Arduino 펌웨어

### 문서 완성도
- **하드웨어 연결**: 모든 핀 매핑과 전기적 사양 문서화 완료
- **파라미터**: 모든 ROS2 노드 파라미터와 Arduino 상수 정의 완료
- **에러/상태**: MCU 통신 프로토콜의 모든 비트 정의 완료
- **토픽 맵**: 업데이트된 토픽 네이밍 체계 완전 반영

### 개선 제안
1. **자동화**: Launch 파일로 복잡한 실행 과정 단순화
2. **모니터링**: 에러 비트 자동 감지 및 알림 시스템
3. **안전성**: Fault 발생 시 자동 복구 로직 추가

전체적으로 실제 배포에 필요한 모든 기술 정보가 문서화되었으며, 최신 코드베이스 변경사항이 완전히 반영되었습니다.