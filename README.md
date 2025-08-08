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
KAI_ws/src/Control/
├── firmware/
│   └── ArduinoMEGA.ino          # Arduino 펌웨어
├── motor_control_cpp/           # C++ 모터 제어 패키지
├── throttle_serial_py/          # Python 시리얼 통신 패키지
├── throttle_msgs/               # 커스텀 메시지 정의
├── steering_control/            # 조향 제어 패키지
├── position_control/            # 위치 제어 패키지
├── position_control_ex/         # 위치 제어 실험용
└── README.md                    # 본 문서
```

## 주요 컴포넌트

### 1. Arduino VCU (Vehicle Control Unit)
**위치**: `firmware/ArduinoMEGA.ino`

#### 주요 기능
- **입력 처리**: 스로틀(0-5V), ASMS 스위치, ESTOP, 기어 레버
- **속도 측정**: Hall 센서 기반 실시간 속도 계산
- **안전 제어**: 하드웨어 레벨 비상정지 및 브레이크 제어
- **통신**: 100Hz 시리얼 패킷 전송 (헤더: 0xAA + RPM16 + FLAGS)
- **실제 시리얼 포트**: `/dev/ttyACM1` (기본 설정과 다를 수 있음)
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
  - **피드백**: MCU data[7:6]에서 RPM 추출 (문서 data[6:7]과 다름)
- **주기**: 50ms

##### `throttle_node` (Python)
- **기능**: Arduino 시리얼 패킷 파싱
- **출력**: `throttle_data` (ThrottleData)
- **포트**: `/dev/ttyACM1` (115200 bps) - 하드코딩됨, 파라미터화 필요
- **파일명**: throttle_node.py (실제 파일명)

##### `rpm_mux` (Python)
- **기능**: AUTO/MANUAL/ESTOP 모드 선택
- **로직**:
  ```
  if ESTOP: output = 0
  elif MANUAL: output = manual_rpm
  else: output = auto_rpm
  ```
- **출력**: `target_rpm` @ 100Hz (실제 동작 주파수, 문서에는 20Hz로 표기된 경우 있음)

### 3. Steering Control (ROS2)
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
- **⚠️ 주의**: position_control 패키지와 동일한 토픽 발행 (충돌 가능성)

### 4. Position Control (Python)
**위치**: `position_control/`

- **`control_node`**: Python-CAN 기반 실험용 제어
- **`position_control_test1`**: 목표 각도 시퀀스 테스트
- **⚠️ CAN 충돌 위험**: steering_control과 동일한 CAN ID (0x06000001) 사용
- **⚠️ 토픽 충돌**: `/encoder_angle` 토픽을 steering_feedback_node와 중복 발행 가능

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
| `/target_rpm_auto` | Int32 | 자율 목표 RPM | 20Hz |
| `/target_rpm` | Int32 | 최종 목표 RPM | 100Hz |
| `/steering_command` | Float32 | 목표 조향각 (deg) | - |
| `/encoder_angle` | Float32 | 현재 조향각 (deg) | - |

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

### 🔴 CAN 충돌 위험
- **ID 0x06000001**: `steering_control`과 `position_control` 패키지가 동일한 CAN ID 사용
- **위험**: 두 패키지 동시 실행 시 명령 충돌로 인한 예측 불가능한 동작
- **해결방안**: 한 번에 하나의 조향 제어 패키지만 실행 권장

### 🔶 토픽 충돌 가능성
- **토픽 `/encoder_angle`**: `steering_feedback_node`와 `position_control` 패키지에서 동시 발행 가능
- **영향**: 토픽 구독자가 혼재된 데이터 수신 가능
- **해결방안**: 토픽명 변경 또는 네임스페이스 분리 필요

### 🔶 하드코딩된 설정
- **시리얼 포트**: `throttle_node.py`에서 `/dev/ttyACM1` 하드코딩
- **문제**: 포트 변경 시 코드 수정 필요
- **개선 필요**: ROS2 파라미터로 포트 설정 가능하도록 수정

### 📊 문서-코드 불일치
- **rpm_mux 주기**: 실제 100Hz 동작, 일부 문서에는 20Hz 표기
- **CAN 피드백**: 실제 data[7:6] 사용, 문서에는 data[6:7] 표기
- **phase_current**: RPM 부호에 따라 ±180A 설정, 문서에 상세 설명 부족

## 트러블슈팅

### 일반적인 문제 해결

#### 시리얼 포트 접근 오류
```bash
# 포트 확인 (ACM0 또는 ACM1 확인 필요)
ls -l /dev/ttyACM*

# Arduino 실제 포트가 /dev/ttyACM1일 수 있음
# throttle_node.py에서 하드코딩된 포트 확인 필요

# 권한 추가
sudo chmod 666 /dev/ttyACM0  # 또는 /dev/ttyACM1
# 또는
sudo usermod -a -G dialout $USER
# (로그아웃 후 재로그인 필요)
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
4. **CAN ID 충돌 확인**: steering_control과 position_control 동시 실행 여부

#### 토픽 데이터가 이상함
1. **중복 발행자 확인**: `ros2 topic info /encoder_angle` 으로 발행자 수 확인
2. **네임스페이스 분리**: 필요시 `--ros-args -r __ns:=/steering` 사용
3. **토픽 리매핑**: `--ros-args -r encoder_angle:=/steering/encoder_angle`

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

**마지막 업데이트**: 2025-08-08  
**버전**: 1.0.0  
**관리자**: TeamKAI-DL