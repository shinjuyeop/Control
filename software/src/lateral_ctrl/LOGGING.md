# 토픽 로깅 (mcap 녹화 + CSV 변환)

실험할 때마다 `ros2 bag record`를 손으로 치는 대신, 로깅 전용 launch 하나로 **모든 토픽을
mcap으로 자동 녹화**하고, 분석할 때 **토픽당 CSV 1개**로 변환하는 구성.

- 녹화: `launch/mcap_logging.launch.py`
- 변환: `scripts/mcap_to_csv.py` (`ros2 run`으로 실행)
- 원본은 항상 mcap — CSV는 파생물이라 나중에 필요한 필드가 생겨도 원본에서 다시 뽑으면 됨.
  mcap은 Foxglove에서 바로 열림.

## 1. 녹화

```bash
# 터미널 1 — 로깅 (습관은 "로깅 먼저, 실험 나중")
ros2 launch lateral_ctrl mcap_logging.launch.py session_name:=deadzone_sweep

# 터미널 2 — 실험 (예)
ros2 launch lateral_ctrl steering_bench_test.launch.py
```

- `--all`로 녹화하므로 **녹화 시작 이후에 새로 생기는 토픽도 자동 구독**됨.
  로깅을 먼저 켜고 실험 launch를 나중에 띄워도 다 잡힌다. (반대로 실험을 먼저 띄우면
  로깅 시작 전 메시지는 유실)
- 종료는 로깅 터미널에서 **Ctrl+C** — rosbag2가 SIGINT를 받아 mcap을 정상 마감하고
  `metadata.yaml`을 쓴다. 이게 없으면 변환 스크립트가 거부하니 강제 kill 금지.

### 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `session_name` | `session` | bag 디렉토리 이름 접두어. 실험 이름을 넣을 것 (`deadzone_sweep`, `bench`, ...) |
| `output_base_dir` | `~/KAI_26/logs/rosbag` | bag들이 쌓일 상위 경로 (repo 밖이라 git에 안 섞임) |

### 출력 구조

```
~/KAI_26/logs/rosbag/
└── deadzone_sweep_20260722_193157/   ← <session_name>_<YYYYmmdd_HHMMSS>
    ├── deadzone_sweep_20260722_193157_0.mcap
    ├── metadata.yaml
    └── csv/                          ← 변환 후 생김 (2절)
```

## 2. CSV 변환

**워크스페이스가 소싱된 터미널에서** 실행해야 함 — custom 메시지
(`SteeringActuatorStatus` 등)의 정의가 필요하기 때문.

```bash
source ~/KAI_26/workspace/Control/software/install/setup.bash

# 전체 토픽 변환 → <bag>/csv/에 토픽당 1파일
ros2 run lateral_ctrl mcap_to_csv.py ~/KAI_26/logs/rosbag/deadzone_sweep_20260722_193157

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
| `file 'mcap_logging.launch.py' was not found` | 그 터미널이 다른(또는 옛) install을 소싱함. `source ~/KAI_26/workspace/Control/software/install/setup.bash` 후 재시도 |
| `The message type '...' is invalid` (echo/변환 시) | 같은 원인 — 워크스페이스 소싱 안 됨 |
| 변환 스크립트가 `metadata.yaml이 없습니다`로 거부 | 녹화를 Ctrl+C로 안 끝내고 강제 종료했거나, bag 디렉토리가 아닌 mcap 파일 자체를 지정함. 디렉토리를 지정할 것 |
| 원하는 토픽이 CSV에 없음 | 녹화 당시 그 토픽을 퍼블리시하는 노드가 안 떠 있었음. 노드가 떠 있어야 토픽이 존재한다 |
| 특정 토픽 `타입 로드 실패` 경고 | 그 메시지 패키지가 소싱 안 된 환경에서 변환함. 해당 워크스페이스 소싱 후 재실행 |
