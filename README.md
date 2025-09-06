# Control Stack - 자율주행 차량 통합 제어 시스템

## 목차
- [시스템 개요](#시스템-개요)
- [아키텍처](#아키텍처)
- [주요 컴포넌트](#주요-컴포넌트)
- [데이터 플로우](#데이터-플로우)
- [설치 및 설정](#설치-및-설정)
- [실행 가이드](#실행-가이드)
- [트러블슈팅](#트러블슈팅)

## 시스템 개요

본 Control Stack은 자율주행 차량의 **추진(Throttle)**과 **조향(Steering)**을 통합 제어하는 ROS2 기반 시스템입니다. 하드웨어 수준의 안전 제어(Arduino)와 소프트웨어 수준의 지능형 제어(ROS2)를 결합하여 안전하고 정밀한 차량 제어를 구현합니다.

### 핵심 기능
- **실시간 차량 제어**: 50ms 주기의 정밀한 모터 제어
- **안전 우선 설계**: 하드웨어/소프트웨어 이중화 비상정지 시스템
- **모드 전환 지원**: AUTO/MANUAL/EMERGENCY 실시간 전환
- **CAN 통신 기반**: 산업 표준 통신 프로토콜 사용
- **모듈러 아키텍처**: ROS2 기반 확장 가능한 구조

## 아키텍처

```
┌──────────────────────────────────────────────────────────────┐
│                     Control Stack Architecture                │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐  │
│  │   Arduino   │────▶│   ROS2      │────▶│   Motors    │  │
│  │    (VCU)    │USB  │   Nodes     │CAN  │  (MCU/SCU)  │  │
│  └─────────────┘     └─────────────┘     └─────────────┘  │
│        ▲                    ▲                    ▲         │
│        │                    │                    │         │
│   ┌─────────┐         ┌─────────┐         ┌─────────┐    │
│   │ Sensors │         │  Auto   │         │Feedback │    │
│   │& Inputs │         │  Stack  │         │Encoders │    │
│   └─────────┘         └─────────┘         └─────────┘    │
│                                                              │
└──────────────────────────────────────────────────────────────┘
```

## 디렉토리 구조

```
Control/
├── firmware/
│   ├── ArduinoMEGA/
│   │   └── ArduinoMEGA.ino      # Arduino Mega VCU 펌웨어
│   └── EV/
│       └── EV.ino               # 스탠드얼론 속도계/랩타이머
├── motor_control_cpp/           # C++ 모터 제어 패키지
├── throttle_serial_py/          # Python 시리얼 통신 패키지
├── throttle_msgs/               # 커스텀 메시지 정의
├── steering_control/            # 조향 제어 패키지
├── speed_monitor_cpp/           # 속도 모니터링 유틸리티
├── README.md                    # 본 문서
└── DETAILED_ANALYSIS.md        # 상세 기술 분석 문서
```

## 주요 컴포넌트

### 1. Arduino VCU (Vehicle Control Unit)
**위치**: `firmware/ArduinoMEGA.ino`

#### 주요 기능
- **입력 처리**: 스로틀(0-5V), ASMS 스위치, ESTOP, 기어 레버
- **속도 측정**: Hall 센서 기반 실시간 속도 계산
- **안전 제어**: 하드웨어 레벨 비상정지 및 브레이크 제어
- **통신**: 100Hz 시리얼 패킷 전송 (헤더: 0xAA + RPM16 + FLAGS)
- **시리얼 포트**: 자동 탐지 (우선순위: /dev/serial/by-id/*, Arduino Mega, CH340, CP210x)
- **부가 기능**: LCD 디스플레이, VESS 서보 제어

#### 시리얼 패킷 포맷
```
[0xAA] [RPM_HIGH] [RPM_LOW] [FLAGS]
- RPM: 0-2000 범위, Big-endian
- FLAGS: bit2=Reverse, bit1=E-STOP, bit0=Manual
```

### 2. Motor Control (ROS2)

모터 제어를 위한 ROS2 패키지들로 구성되어 있으며, C++ 노드와 Python 노드가 협력하여 동작합니다.

#### 주요 노드 상세

##### `speed_publisher` (C++)
- **기능**: 자율 주행 목표 RPM 생성
- **토픽**: `target_rpm_auto` (std_msgs/Int32)
- **주기**: 50ms
- **파라미터**: `rpm` (동적 변경 가능, 기본 500)

##### `can_speed_sender` (C++)
- **기능**: MCU와 CAN 통신으로 모터 제어
- **입력**: `target_rpm` 토픽 구독
- **프로토콜**: 
  - 핸드셰이크: MCU(0x1801D0EF) ↔ Node(0x0C01EFD0)
  - 제어 프레임: 위상전류(RPM 부호에 따라 ±180A) + RPM + 모드(0x03) + life counter
  - **피드백**: MCU data[6:7]에서 RPM 추출 (Little-endian)
- **주기**: 50ms

##### `throttle_node` (Python)
- **기능**: Arduino 시리얼 패킷 파싱
- **출력**: `throttle_data` (ThrottleData)
- **포트**: 자동 탐지 또는 ROS 파라미터 `port` 지정 (115200 bps)
- **파라미터**: `port` (기본: 'auto'), `baud`, `boot_wait`, `probe_window`

##### `rpm_mux` (Python)
- **기능**: AUTO/MANUAL/ESTOP 모드 선택
- **로직**:
  ```
  if ESTOP: output = 0
  elif MANUAL: output = manual_rpm
  else: output = auto_rpm
  ```
- **출력**: `target_rpm` @ 100Hz

##### `can_speed_receiver` (C++)
- **기능**: MCU로부터 실시간 피드백 수신
- **입력**: CAN ID 0x1801D0EF (Part1), 0x1802D0EF (Part2)
- **출력**: `/current_speed` (Int32) - 실제 RPM
- **피드백 데이터**:
  - Part1: 전압, 전류, 위상전류, RPM
  - Part2: 온도, 상태, 에러, 결함

### 3. Speed Monitor (ROS2)
**위치**: `speed_monitor_cpp/`

#### 노드 구성

##### `speed_monitor_node` (C++)
- **기능**: RPM 토픽 모니터링 유틸리티
- **입력**: `/current_speed` (Int32) 또는 파라미터 지정 토픽
- **출력**: 콘솔 로그 ("RPM: {value}")
- **파라미터**: `topic` (모니터링할 토픽명)

### 4. Steering Control (ROS2)
**위치**: `steering_control/`

#### 노드 구성

##### `steering_control_node` (C++)
- **기능**: 조향 모터 위치 제어
- **입력**: `/steering_command` (Float32, degrees)
- **CAN 프로토콜**:
  - Enable: ID 0x06000001, Data: `23 0D 20 01 00 00 00 00`
  - Position: ID 0x06000001, Data: `23 02 20 01 [32bit_position]`
  ```cpp
  x = -0.00125*y² + 3.635*y - 3.875
  scaled_val = (x/360) * 10000
  ```

##### `steering_feedback_node` (C++)
- **기능**: 엔코더 피드백 수신
- **출력**: `/encoder_angle` (Float32, degrees)
- **CAN ID**: 0x07000001 (Heartbeat)

## 데이터 플로우

### 추진 제어 흐름
```
Arduino ──[Serial]──▶ throttle_node
                            │
                            ▼
                      ThrottleData 토픽
                            │
         ┌──────────────────┴──────────────────┐
         ▼                                     ▼
    Manual RPM                            Auto RPM
         │                                     │
         └──────────▶ rpm_mux ◀───────────────┘
                            │
                            ▼
                      target_rpm 토픽
                            │
                            ▼
                    can_speed_sender
                            │
                      [CAN 0x0C01EFD0]
                            ▼
                      Motor Controller
```

### 조향 제어 흐름
```
steering_command 토픽 ──▶ steering_control_node
                                │
                          [CAN 0x06000001]
                                ▼
                          Steering Motor
                                │
                          [CAN 0x07000001]
                                ▼
                      steering_feedback_node
                                │
                                ▼
                        encoder_angle 토픽
```

## 설치 및 설정

### 1. 의존성 설치
```bash
# ROS2 패키지
sudo apt install ros-humble-*

# Python 패키지
pip install python-can pyserial

# CAN 도구
sudo apt install can-utils
```

### 2. CAN 인터페이스 설정
```bash
# CAN 인터페이스 활성화 (250kbps)
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000

# 권한 설정
sudo usermod -a -G dialout $USER  # 시리얼 포트
```

### 3. Arduino 설정
- Arduino MEGA 2560 보드 사용
- 라이브러리: Servo, LiquidCrystal
- 포트 권한 확인: `ls -l /dev/ttyACM*`

## 실행 가이드

### 전체 시스템 실행

#### 1. 워크스페이스 빌드
```bash
# 전체 워크스페이스 빌드
cd ~/ROS2_Workspace/KAI_ws
colcon build
source install/setup.bash
```

#### 2. Motor Control 실행
```bash
# 터미널 1: Arduino 통신
ros2 run throttle_serial_py throttle_node

# 터미널 2: 자율 RPM 생성
ros2 run motor_control_cpp speed_publisher --ros-args -p rpm:=800

# 터미널 3: RPM MUX
ros2 run throttle_serial_py rpm_mux

# 터미널 4: CAN 송신
ros2 run motor_control_cpp can_speed_sender

# (옵션) 터미널 5: CAN 수신 로거
ros2 run motor_control_cpp can_speed_receiver

# (옵션) 터미널 6: 속도 모니터
ros2 run speed_monitor_cpp speed_monitor_node
```

#### 3. Steering Control 실행
```bash

# 터미널 1: 조향 제어
ros2 run steering_control steering_control_node

# 터미널 2: 엔코더 피드백
ros2 run steering_control steering_feedback_node

# (테스트) 터미널 3: 명령 생성
ros2 run steering_control steering_command_publisher_node
```

### 모니터링

#### 토픽 확인
```bash
# 토픽 리스트
ros2 topic list

# 실시간 모니터링
ros2 topic echo /throttle_data
ros2 topic echo /target_rpm
ros2 topic echo /steering_command
ros2 topic echo /encoder_angle
```

#### CAN 모니터링
```bash
# CAN 트래픽 모니터링
candump can0

# 특정 ID 필터링
candump can0,0x1801D0EF:0xFFFFFFFF
```

## 주요 토픽 및 메시지

### 커스텀 메시지: `ThrottleData`
```
int16 rpm      # 현재 RPM (-2000 ~ 2000)
bool estop     # 비상정지 상태
bool asms      # AUTO/MANUAL 스위치 (True=Manual)
```

### 표준 토픽
| 토픽명 | 타입 | 설명 | 주기 |
|--------|------|------|------|
| `/throttle_data` | ThrottleData | Arduino 센서 데이터 | 100Hz |
| `/target_rpm_auto` | Int32 | 자율 목표 RPM | 100Hz |
| `/target_rpm` | Int32 | 최종 목표 RPM | 100Hz |
| `/current_speed` | Int32 | MCU 피드백 실제 RPM | - |
| `/steering_command` | Float32 | 목표 조향각 (deg) | - |
| `/encoder_angle` | Float32 | 현재 조향각 (deg) | 10Hz |

## CAN 메시지 사양

### Motor Control CAN IDs
| ID (EFF) | 방향 | 설명 |
|----------|------|------|
| 0x1801D0EF | MCU→ROS | 핸드셰이크/피드백 |
| 0x0C01EFD0 | ROS→MCU | 제어 명령 |

### Steering Control CAN IDs
| ID (EFF) | 방향 | 설명 |
|----------|------|------|
| 0x06000001 | ROS→Motor | Enable/Position |
| 0x07000001 | Motor→ROS | Heartbeat/Encoder |

## ⚠️ 주요 이슈 및 주의사항

### 🔶 CAN 인터페이스 설정
- **기본값**: can_speed_sender/receiver는 `can0`, steering_control은 `can1` 사용
- **변경 방법**: 실행 시 명령줄 인수로 지정 가능
- **예시**: `ros2 run motor_control_cpp can_speed_sender can1`

### 📊 개선 사항
- **시리얼 포트 자동 탐지**: throttle_node가 Arduino를 자동으로 찾아 연결
- **동적 파라미터**: speed_publisher의 RPM을 실행 중 변경 가능
- **에러 복구**: 시리얼/CAN 연결 끊김 시 자동 재연결

## 트러블슈팅

### 일반적인 문제 해결

#### 시리얼 포트 접근 오류
```bash
# 자동 탐지가 실패할 경우 수동 지정
ros2 run throttle_serial_py throttle_node --ros-args -p port:=/dev/ttyACM0

# 권한 추가
sudo usermod -a -G dialout $USER
# (로그아웃 후 재로그인 필요)

# 디버깅: 연결된 Arduino 확인
ls -l /dev/serial/by-id/
```

#### CAN 인터페이스 오류
```bash
# 인터페이스 상태 확인
ip link show can0

# 재설정
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 250000

# 통계 확인
ip -details -statistics link show can0
```

#### RPM이 0으로 고정
1. ESTOP 상태 확인: `ros2 topic echo /throttle_data`
2. ASMS 모드 확인 (Manual/Auto)
3. CAN 통신 상태 확인: `candump can0`

#### 조향이 응답하지 않음
1. Enable 프레임 전송 확인
2. 엔코더 피드백 확인: `ros2 topic echo /encoder_angle`
3. CAN 보레이트 확인 (250kbps)
4. CAN 인터페이스 확인 (기본 can1)

### 디버깅 팁

#### 로그 레벨 설정
```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
```

#### 파라미터 동적 변경
```bash
# RPM 파라미터 변경
ros2 param set /speed_publisher rpm 1000
```

## 참고 자료

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [SocketCAN Documentation](https://www.kernel.org/doc/Documentation/networking/can.txt)
- [Arduino Serial Communication](https://www.arduino.cc/reference/en/language/functions/communication/serial/)

---

## 향후 개선 사항

- [x] 워크스페이스 통합 완료
- [ ] 중복 코드 제거 및 리팩토링
- [ ] 단위 테스트 추가
- [ ] CI/CD 파이프라인 구축
- [ ] 시뮬레이션 환경 구축
- [ ] 성능 최적화 (레이턴시 감소)

---

**마지막 업데이트**: 2025-08-19  
**버전**: 1.1.0  
**관리자**: TeamKAI-DL
