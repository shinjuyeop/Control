# Team KAI CAN Lab

`teamkai_can_lab`는 `vcan0 / vcan1` 기반으로 모터/조향 CAN 제어를 빠르게 확인하는 교육용 시뮬레이터입니다.

## 구성

- `teamkai_can_lab.py`
  - 실행 진입점
- `can_io.py`
  - SocketCAN 입출력, CAN frame 인코딩/디코딩
- `physics.py`
  - 모터/조향 상태와 교육용 차량/조향 plant 계산
- `app.py`
  - 통신부와 물리부를 연결하는 runtime
- `hmi.py`
  - Tkinter + Matplotlib 디버그 화면
- `signals_reference.md`
  - 교육용 프로토콜 요약

## 현재 에뮬레이션하는 프레임

### Motor / drive

- `0x1801D0EF` : MCU -> VCU status1 / handshake
- `0x1802D0EF` : MCU -> VCU status2
- `0x0C01EFD0` : VCU -> MCU command

### Steering

- `0x06000001` : VCU -> steering actuator command
- `0x07000001` : steering actuator -> VCU feedback

## vcan 준비

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link add dev vcan1 type vcan
sudo ip link set up vcan0
sudo ip link set up vcan1
```

이미 있으면 `ip link show vcan0` / `vcan1`로 확인하면 됩니다.

## 실행

### GUI

```bash
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py --motor-iface vcan0 --steer-iface vcan1
```

### Headless demo

```bash
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py --motor-iface vcan0 --steer-iface vcan1 --no-gui --headless-demo --duration 5
```

`--no-gui` 모드도 실제 `vcan` 인터페이스가 올라와 있어야 합니다.

## GUI에서 볼 수 있는 것

- 상단 motor/steering 요약 상태
- 우상단 인터페이스/진단 요약 (`tx`, `rx`, `cb_err`, `listener_err`)
- motor 계기와 steering 계기
- 최근 30초 기준 `20 Hz` 갱신 플롯
- raw CAN manual TX
- preset 버튼
- raw CAN frame log

## runtime / plant 특성

- runtime main loop는 `100 Hz`, motor status 송신은 `20 Hz`, steering feedback 송신은 `50 Hz`, motor handshake frame 송신은 `2 Hz`입니다.
- bus 계층은 송신/수신 frame 수와 callback 예외 수를 누적하고, socket recv 실패와 listener 예외는 rate-limit 로그로 남깁니다.
- motor plant는 `기어비`, `차량 질량`, `바퀴 반경`, `구름저항`, `공력저항`, `경사 부하`, `구동 효율`을 사용해 속도와 전류/온도를 계산합니다.
- steering plant는 `최대 조향 속도`, `backlash`, `static friction`, `damping`이 있는 응답으로 핸들각을 갱신합니다.

## 최소 실습 시나리오

### Motor handshake ACK

```bash
cansend vcan0 0C01EFD0#AAAAAAAAAAAAAAAA
```

### Motor +1000 rpm

```bash
cansend vcan0 0C01EFD0#8890E88003000000
```

### Steering enable

```bash
cansend vcan1 06000001#230D200100000000
```

### Steering +10 deg

```bash
cansend vcan1 06000001#23022001FFFFFB3F
```

## ROS 2와 같이 쓰기

추후 `software/src/control_launch/launch/` 아래에 종방향/횡방향 제어 노드를 묶는 launch 파일을 추가하면, 이 시뮬레이터를 `vcan0`/`vcan1` 상대역으로 함께 사용할 수 있습니다.

종방향:

```bash
ros2 topic pub /target_rpm std_msgs/msg/Int32 "{data: 1000}" -r 20
```

횡방향:

```bash
ros2 topic pub -r 50 /vehicle/command kaiev26_msgs/msg/ActuatorCommand \
  "{steering_target_rad: -0.1745, speed_target_mps: 0.0, brake_engage: false}"
```

위 명령의 `-0.1745 rad`는 기존 벤치 표기의 우조향 `+10 deg`에 해당합니다.
`lateral_ctrl`을 함께 실행한다면 STM_A 또는 시험 노드가 AUTO
`/drive/status`를 발행하고 KEYA feedback도 들어와야 모터가 활성화됩니다.

## 주의

- 이건 교육용 plant model입니다. 정밀 물리 모델이 아닙니다.
- motor command는 일정 시간 안 들어오면 timeout으로 목표 RPM을 놓습니다.
- steering은 enable 후 position command를 주는 흐름입니다.
