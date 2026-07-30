# Control

## 프로젝트 소개

`Control`은 ROS 2 Humble 기반 차량 제어 소프트웨어와 STM32G4 펌웨어를 함께
관리하는 저장소입니다. 공통 ROS 2 메시지 계약은 별도 저장소
`kaiev26_msgs`에 있으며, 두 저장소를 같은 ROS 2 워크스페이스의 `src/` 아래에
배치해 함께 빌드할 수 있습니다.

이 문서는 프로젝트의 입구와 문서 지도 역할을 합니다. 펌웨어 통신 규약과 안전
동작의 상세 내용은 아래의 기준 문서를 따릅니다.

## 시스템 구성

- Host / Control PC: 상위 ROS 2 제어 명령 전달, 조향 모터 제어, 상태 통합
- STM_A: 가속 페달·모드·기어 입력과 EZkontrol 구동 모터 제어
- STM_B: 세 물리 E-stop 입력, 브레이크 서보, 최종 브레이크 요청 결정
- micro-ROS Agent: 각 STM의 LPUART1 serial transport와 ROS 2 graph 연결

전체 데이터 흐름과 책임 경계는
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md)를 참고하십시오.

## 저장소 구조

```text
Control/
├── docs/                    # 시스템·인터페이스·안전 기준 문서
├── firmware/
│   ├── STM_A/               # 구동 및 차량 입력 펌웨어
│   ├── STM_B/               # E-stop 및 브레이크 펌웨어
│   └── micro_ros_agent/     # vendored micro-ROS Agent
└── software/
    └── src/
        ├── lateral_ctrl/    # KEYA 조향 액추에이터 제어
        ├── longitudinal_ctrl/
        ├── vehicle_state_publisher/
        ├── control_launch/
        ├── can_lab/
        ├── gazebo/
        └── visualization/
```

## 주요 구성요소

| 구성요소 | 현재 역할 |
|---|---|
| `firmware/STM_A` | `/vehicle/command`의 속도·브레이크 명령, 물리 입력, EZkontrol CAN, `/drive/status` |
| `firmware/STM_B` | `/vehicle/command`의 브레이크 명령, 물리 E-stop, 브레이크 PWM, `/brake/status` |
| `software/src/lateral_ctrl` | `/vehicle/command`와 STM 상태를 사용한 KEYA 조향 모터 제어 및 `/steering/status` |
| `software/src/vehicle_state_publisher` | 액추에이터 status 3종을 `/vehicle/state`로 통합 |
| `software/src/control_launch` | 전체 제어 launch 파일용 패키지이며 현재 통합 launch는 없음 |
| `software/src/longitudinal_ctrl` | 현재 패키지 골격만 있고 실행 노드는 없음 |
| `software/src/can_lab` | SocketCAN 기반 구동·조향 CAN 시험 도구 |
| `software/src/gazebo` | PID 및 SILS 관련 시뮬레이션 자원 |

## 빠른 빌드

권장 워크스페이스 배치는 다음과 같습니다.

```text
~/kai_ws/
└── src/
    ├── Control/
    └── kaiev26_msgs/
```

ROS 2 패키지는 워크스페이스 루트에서 빌드합니다.

```bash
cd ~/kai_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/kai_ws/install/setup.bash
```

STM 펌웨어 빌드와 보드별 micro-ROS Agent 실행 절차는
[`firmware/README.md`](firmware/README.md)를 따릅니다.

## 문서

| 문서 | 내용 |
|---|---|
| [Architecture](docs/ARCHITECTURE.md) | 전체 시스템 구성, 역할과 데이터 흐름 |
| [Interfaces](docs/INTERFACES.md) | ROS 2, STM 간 UART, EZkontrol·KEYA CAN 계약 |
| [Safety](docs/SAFETY.md) | E-stop, timeout, fail-safe와 복구 동작 |
| [Firmware](firmware/README.md) | 공통 STM 빌드와 micro-ROS Agent 실행 |
| [STM_A](firmware/STM_A/README.md) | STM_A 내부 구조 및 동작 |
| [STM_B](firmware/STM_B/README.md) | STM_B 내부 구조 및 동작 |
| [Lateral control](software/src/lateral_ctrl/README.md) | 현재 KEYA 조향 패키지 사용법 |

## 개발 상태

펌웨어의 현재 구현과 인터페이스·안전 동작은 위 기준 문서에 정리되어 있습니다.
`software` 패키지 전체의 구조와 실행 절차는 아직 정리 중이며, 코드 구조가
안정된 뒤 패키지별 README와 통합 실행 문서를 별도 작성할 예정입니다.

현재 `control_launch`에는 통합 launch가 없고 `longitudinal_ctrl`에는 실행 노드가
없으므로, 문서에 없는 전체 차량 실행 명령을 추정해서 사용하지 마십시오.

현재 유지되는 개별 개발 진입점은 다음과 같습니다.

```bash
# PID Gazebo
ros2 launch pid_gazebo pid_gazebo.launch.py

# SILS bag 기록 포함
ros2 launch pid_gazebo sils.launch.py

# CAN lab
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py \
  --motor-iface vcan0 --steer-iface vcan1
```

개인 작업은 각자 브랜치에서 진행하고, `build/`, `install/`, `log/`, rosbag,
MCAP, 분석 CSV와 plot 산출물은 커밋하지 않습니다. 커밋 전에는 변경 범위와
빌드 결과를 확인하십시오.

- ROS 2 패키지는 `software/src/` 아래에 둡니다.
- Gazebo 코드는 `software/src/gazebo/` 아래에 둡니다.
- 전체 제어 launch는 `software/src/control_launch/launch/` 아래에 둡니다.
- 패키지 단독 시험 launch는 해당 패키지의 `launch/` 아래에 둡니다.
- 다른 작업자의 branch를 덮어쓰거나 force push해야 할 때는 먼저 합의합니다.

```bash
git status
git diff
```
