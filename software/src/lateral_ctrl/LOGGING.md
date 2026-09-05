# 토픽 로깅 (mcap 녹화 + CSV 변환)

실험할 때마다 `ros2 bag record`를 손으로 치는 대신, 로깅 전용 launch 하나로 실차
sysid에 필요한 토픽만 mcap으로 녹화하고 분석할 때 토픽당 CSV 1개로 변환하는 구성.
압축 카메라 영상과 카메라 정보, Ouster LiDAR·IMU 패킷 및 메타데이터도 함께 녹화한다.

- 녹화: `launch/mcap_logging.launch.py`
- 변환: `scripts/mcap_to_csv.py` (`ros2 run`으로 실행)
- 원본은 항상 mcap — CSV는 파생물이라 나중에 필요한 필드가 생겨도 원본에서 다시 뽑으면 됨.
  mcap은 Foxglove에서 바로 열림.

## 1. 녹화

```bash
# 센서·Control을 먼저 띄우고 필요한 publisher와 QoS를 확인한다.
ros2 topic info -v /steering/status
ros2 topic info -v /imu/data
ros2 topic info -v /gnss/fix

# 실차 제어컴 기본 경로는 /home/kai/KAI_ws/record다.
ros2 launch lateral_ctrl mcap_logging.launch.py

# 개발 랩탑에서 외장 SSD를 쓸 때는 경로와 mount 강제를 함께 명시한다.
findmnt -T ~/Storage/ROS2_Workspace_offload
df -h ~/Storage/ROS2_Workspace_offload
ros2 launch lateral_ctrl mcap_logging.launch.py \
  output_base_dir:=~/Storage/ROS2_Workspace_offload/gazebo_sitl/vehicle_sysid \
  require_external_storage:=true
```

- `/vehicle/command`, actuator status, raw IMU/GNSS, ESKF, 좌우 hub IMU, TF와
  diagnostics와 위 카메라·Ouster 토픽을 명시해 녹화한다. 발행되지 않는 선택 토픽은 recorder 경고만 남고,
  발행을 시작하면 discovery를 통해 구독한다.
- raw CAN은 같은 host에서 interface별 `candump -L`로 별도 기록한다.
- 종료는 로깅 터미널에서 **Ctrl+C** — rosbag2가 SIGINT를 받아 mcap을 정상 마감하고
  `metadata.yaml`을 쓴다. 이게 없으면 변환 스크립트가 거부하니 강제 kill 금지.

### 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `output_base_dir` | `/home/kai/KAI_ws/record` | 실차 제어컴의 local 기록 경로. 다른 host에서는 명시적으로 덮어쓴다 |
| `require_external_storage` | `false` | `true`면 출력이 외장 artifact root 아래 별도 mount인지 확인한다 |

### 출력 구조

```
/home/kai/KAI_ws/record/
└── rosbag2_2026_09_05-14_00_00/
    ├── rosbag2_2026_09_05-14_00_00_0.mcap
    ├── metadata.yaml
    └── csv/                       ← 변환 후 생김 (2절)
```

## 2. CSV 변환

**워크스페이스가 소싱된 터미널에서** 실행해야 함 — custom 메시지
(`SteeringActuatorStatus` 등)의 정의가 필요하기 때문.

```bash
source install/setup.bash  # gazebo_sitl workspace root에서 실행

# 전체 토픽 변환 → <bag>/csv/에 토픽당 1파일
BAG_DIR=/home/kai/KAI_ws/record/rosbag2_2026_09_05-14_00_00
ros2 run lateral_ctrl mcap_to_csv.py "$BAG_DIR"

# 특정 토픽만 / 출력 위치 지정
ros2 run lateral_ctrl mcap_to_csv.py <bag_dir> \
  -t /steering/status /vehicle/command \
  -o /tmp/csv_out
```

### CSV 형식

- 파일명: 토픽 이름의 `/`를 `__`로 치환 (`/steering/status` → `steering__status.csv`)
- 첫 컬럼 `recv_time_sec`: bag이 그 메시지를 받은 시각 (epoch 초, float)
- 중첩 메시지 필드는 점 표기로 평탄화: `header.stamp.sec`, `motor_current_a`, ...
- 배열 필드는 세미콜론으로 이어 한 셀에 저장

pandas 예:

```python
import pandas as pd
df = pd.read_csv("csv/steering__status.csv")
df.plot(x="recv_time_sec", y=["motor_current_a", "motor_angle_deg"])
```

## 3. Foxglove로 볼 때

변환 없이 mcap을 그대로 연다:

1. Foxglove Studio → Open local file → `..._0.mcap` 선택
2. custom 메시지 정의는 mcap 안에 포함돼 있어서 별도 설정 불필요
3. Plot 패널에 `/steering/status.motor_current_a` 같은 필드 경로를 직접 입력

## 4. 자주 겪는 문제

| 증상 | 원인 / 해결 |
|---|---|
| `file 'mcap_logging.launch.py' was not found` | 그 터미널이 다른(또는 옛) install을 소싱함. 현재 `gazebo_sitl/install/setup.bash`를 소싱하고 재시도 |
| `The message type '...' is invalid` (echo/변환 시) | 같은 원인 — 워크스페이스 소싱 안 됨 |
| `external ROS artifact root is unavailable/not mounted separately` | `require_external_storage:=true` 실행에서 외장 SSD mount를 복구한 뒤 재시도. 내부 SSD나 `/tmp`로 fallback하지 않음 |
| 변환 스크립트가 `metadata.yaml이 없습니다`로 거부 | 녹화를 Ctrl+C로 안 끝내고 강제 종료했거나, bag 디렉토리가 아닌 mcap 파일 자체를 지정함. 디렉토리를 지정할 것 |
| 원하는 토픽이 CSV에 없음 | 움직임 전에 해당 publisher와 QoS를 `ros2 topic info -v`로 확인하고 recorder가 subscription을 만든 뒤 시작할 것 |
| 특정 토픽 `타입 로드 실패` 경고 | 그 메시지 패키지가 소싱 안 된 환경에서 변환함. 해당 워크스페이스 소싱 후 재실행 |
