# Control

ROS 2 기반 차량 제어 소프트웨어와 STM32 펌웨어를 함께 관리하는 저장소입니다.

## Directory Layout

```text
Control/
├── firmware/
│   ├── STM_A/
│   ├── STM_B/
│   └── micro_ros_agent/
└── software/
    └── src/
        ├── can_lab/
        │   ├── teamkai_can_lab/
        │   └── tests/
        ├── control_launch/
        │   └── launch/
        ├── gazebo/
        │   ├── assets/
        │   └── pid_gazebo/
        │       ├── config/
        │       ├── launch/
        │       ├── matlab/
        │       ├── pid_gazebo/
        │       └── scripts/
        ├── lateral_ctrl/
        ├── longitudinal_ctrl/
        └── visualization/
```

## ROS 2 Packages

- `control_launch`: 전체 제어 스택을 묶는 launch 파일을 두는 패키지입니다.
- `can_lab/teamkai_can_lab`: `vcan` 기반 종방향/횡방향 CAN 테스트용 시뮬레이터입니다.
- `lateral_ctrl`: 횡방향 제어 노드를 위한 C++ 패키지입니다.
- `longitudinal_ctrl`: 종방향 제어 노드를 위한 C++ 패키지입니다.
- `gazebo/pid_gazebo`: Gazebo에서 PID 제어를 단독 테스트하는 Python 패키지입니다.
- `gazebo/assets`: Gazebo SDF, STL 등 시뮬레이션 리소스를 모아둔 폴더입니다.
- `visualization`: Foxglove 등 시각화 보조 노드를 위한 Python 패키지입니다.

`can_lab/teamkai_can_lab`는 실제 제어 노드가 CAN으로 내보내는 명령을 로컬 `vcan`에서 확인하기 위한 도구입니다. 종방향 제어는 `vcan0`, 횡방향 제어는 `vcan1`에 연결하는 흐름을 기본으로 둡니다.

```text
longitudinal_ctrl -> vcan0 -> teamkai_can_lab motor sim
lateral_ctrl      -> vcan1 -> teamkai_can_lab steering sim
```

`gazebo/pid_gazebo` 내부는 역할별로 나눕니다.

- `config/`: PID gain 등 실행 설정 파일을 둡니다.
- `launch/`: `pid_gazebo` 단독 실행과 SILS 실행 launch 파일을 둡니다.
- `matlab/`: PID 튜닝용 MATLAB 스크립트를 둡니다.
- `pid_gazebo/`: 실제 ROS 2 Python 노드 코드를 둡니다.
- `scripts/`: bag 분석, 자동 실험, plot 생성 같은 개발 보조 스크립트를 둡니다.

펌웨어의 보드 역할, 안전 경로, STM 빌드와 micro-ROS Agent 실행 절차는
[`firmware/README.md`](firmware/README.md)를 따릅니다.

## Build

ROS 2 Humble 환경을 source한 뒤 `software` 워크스페이스에서 빌드합니다.

```bash
source /opt/ros/humble/setup.bash
cd software
colcon build
source install/setup.bash
```

빌드 산출물인 `software/build`, `software/install`, `software/log`는 Git에 올리지 않습니다. 필요하면 지워도 되고, `colcon build`를 다시 실행하면 재생성됩니다.

## Launch

PID Gazebo 단독 테스트:

```bash
ros2 launch pid_gazebo pid_gazebo.launch.py
```

SILS bag 기록을 포함한 PID Gazebo 실행:

```bash
ros2 launch pid_gazebo sils.launch.py
```

추후 실제 차량 제어 스택 launch 파일은 `software/src/control_launch/launch/` 아래에 추가합니다.

CAN lab 시뮬레이터:

```bash
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py --motor-iface vcan0 --steer-iface vcan1
```

GUI 없이 빠르게 확인:

```bash
python3 software/src/can_lab/teamkai_can_lab/teamkai_can_lab.py --motor-iface vcan0 --steer-iface vcan1 --no-gui --headless-demo --duration 5
```

## Generated Data

주행 로그, rosbag, MCAP, 분석 CSV, plot 이미지 등은 Git 추적 대상에서 제외합니다. 주요 ignore 대상은 다음과 같습니다.

```text
*.bag
*.db3
*.mcap
software/build/
software/install/
software/log/
software/src/gazebo/pid_gazebo/bags/
software/src/gazebo/pid_gazebo/analysis/
software/src/gazebo/pid_gazebo/plots/
```

## Development Rules

- 개인 작업은 각자 브랜치에서 진행합니다.
- 다른 사람 브랜치를 덮어쓰거나 force push해야 하는 경우 반드시 사전에 합의합니다.
- 하나의 커밋에는 하나의 목적만 담습니다.
- 로그 삭제, 폴더 이동, 기능 추가, 버그 수정은 가능하면 분리해서 커밋합니다.
- ROS 2 패키지는 `software/src/` 아래에 둡니다.
- Gazebo 관련 코드는 `software/src/gazebo/` 아래에 둡니다.
- 실제 제어 스택 launch 파일은 `software/src/control_launch/launch/` 아래에 둡니다.
- 단일 패키지 테스트 launch 파일은 해당 패키지의 `launch/` 폴더에 둡니다.
- `build/`, `install/`, `log/`, `__pycache__/`는 커밋하지 않습니다.
- rosbag, MCAP, DB3, 분석 CSV, plot 이미지는 커밋하지 않습니다.

커밋 전에는 다음을 확인합니다.

```bash
git status
git diff
cd software && colcon build
```

## Collaboration Workflow

작업이 끝나거나 공유할 업데이트가 생기면 본인 브랜치에 push합니다.

```bash
git add .
git commit -m "message"
git push origin <my-branch>
```

push 후에는 카카오톡으로 팀원에게 변경 내용을 간단히 공유합니다.

```text
브랜치:
주요 변경:
확인 필요한 부분:
```

다른 팀원의 업데이트가 생기면, 자신의 작업을 이어가기 전에 최신 변경을 merge 또는 rebase해서 반영합니다.

```bash
git fetch origin
git checkout <my-branch>
git merge origin/<updated-branch>
```

충돌이 생기면 임의로 덮어쓰지 말고, 변경한 사람과 확인한 뒤 해결합니다.

## Local Setup

개인 환경용 `setup.bash`는 저장소 루트에 둘 수 있지만 Git에는 올리지 않습니다. 필요한 경우 로컬에서 다음처럼 사용합니다.

```bash
source /home/shinjuyeop/Control/setup.bash
```
