# Control tools

제어기의 시험·측정·녹화·분석 도구를 모은 패키지입니다. 실제 조향 제어기는
`lateral_ctrl`, 설정 편집은 `control_tuning`, 시나리오 TUI는 `vehicle_test_scenarios`가 담당합니다.

## 실행 도구

| 목적 | 실행 |
|---|---|
| 독립 MCAP 녹화 | `ros2 launch control_tools mcap_logging.launch.py` |
| MCAP → 토픽별 CSV | `ros2 run control_tools mcap_to_csv.py <bag>` |
| 종방향 Speed/Torque 비교 | `ros2 run control_tools longitudinal_analysis <bag>` |
| 조향 응답·관성 분석 | `ros2 run control_tools torque_sysid_metrics.py step <bag>` |
| 조향 단독 벤치 | `ros2 launch control_tools steering_bench_test.launch.py` |
| 데드존 측정 | `ros2 run control_tools deadzone_measure_node can0` |
| 바퀴 각도 측정 환경 | `ros2 launch control_tools wheel_steering_measurement.launch.py` |
| T0/T1/T2 토크 식별 | `ros2 launch control_tools torque_sysid_bench.launch.py test:=t0` |

이전 `lateral_ctrl`의 위 실행 명령과 `control_tuning.longitudinal_analysis`는
이 패키지로 이동했습니다. 조향 노드·ROS 제어 토픽·설정 프리셋은 유지합니다.

## 빌드

워크스페이스 루트에서 ROS 환경을 불러온 뒤 실행합니다.

```bash
cd ~/KAI_ws
colcon build --symlink-install --packages-select lateral_ctrl control_tuning control_tools vehicle_test_scenarios
```

빌드 후 같은 터미널에서 `source install/setup.bash`를 실행합니다(zsh는 `setup.zsh`).
측정 노드는 C++, 공통 녹화·분석은 Python이며 `ament_cmake_python`으로 함께 설치합니다.
MCAP에는 ROS의 `rosbag2_py`, `rosbag2_storage_mcap`, 분석에는 NumPy가 필요합니다.

## 녹화와 분석

[녹화 상세](LOGGING.md)에 두 프로필, 초기 파라미터 수집과 저장 결과를 정리했습니다.
독립 녹화와 시나리오 TUI는 `control_tools.topics`의 Control 토픽과
`control_tools.recording`의 MCAP `zstd_fast` 설정을 공유합니다.
각 경로의 기존 센서 선택은 별도로 유지하며 카메라·LiDAR 토픽은 변경하지 않았습니다.
TUI의 Run·Stop, 시험 조건과 결과 관리는 계속 시나리오 패키지에 있습니다.

```bash
ros2 run control_tools longitudinal_analysis <bag> --start 0 --end 10.5 --target 6 --pi-csv /tmp/pi.csv
```

`--pi-csv`는 같은 calibration revision 안에서도 **수신된 모든 PI 상태**를 내보냅니다.
MCAP 수신 시각, revision, 속도 오차, P·I, 제한 전·후 전류, 포화·anti-windup·coasting,
feedback validity·inhibit가 들어갑니다. 파라미터 변경 이력은 별도 snapshot으로 유지합니다.
STM_A PI 상세의 기존 5 Hz 발행률과 펌웨어 ABI는 바꾸지 않습니다.

## 조향 단독 벤치

벤치 노드는 `/drive/status`와 각도 시퀀스를 직접 발행합니다. 현재 기본 시험
status는 유효한 구동 속력을 제공하지 않으므로 조향은 Disable 상태를 유지합니다.
최저속도 설정이 0이어도 `part1_valid`와 fresh한 속도 피드백은 필요합니다.

> **HIL bench와 동시에 실행하지 마십시오.** HIL 스택의 `command_governor`와
> 이 노드가 모두 `/vehicle/command`를 발행하므로 명령이 서로 덮어써집니다.
> 이 노드는 governor를 우회하는 조향 단독 벤치 전용입니다. HIL bench에서
> 조향을 시험할 때는 `hil_command_generator`(governor 경유)만 사용합니다.
> 확인은 다음과 같이 합니다.
>
> ```bash
> ros2 topic info -v /vehicle/command   # publisher가 하나인지
> ```

다음 launch는 actuator 노드와 벤치 명령 노드를 함께 실행합니다.

```bash
ros2 launch control_tools steering_bench_test.launch.py
```

기본 벤치 노드는 다음을 발행합니다.

| 토픽 | 내용 | 주기 |
| --- | --- | ---: |
| `/drive/status` | 시험용 AUTO 상태 | 100 ms |
| `/vehicle/command` | 절대 중앙 등가각 `+5/-5 deg`, 속도 0, 브레이크 해제 | 20 ms |
| `/steering/test/estop` | 시험 E-stop 해제 | 50 ms |

`steer_sequence_deg`는 중앙 등가각 degree·좌회전 양수입니다. 벤치 노드는 단위만
canonical radian으로 바꾸어 발행합니다. 명령은 현재각 offset이 아닌 절대
중앙 등가각입니다. 바퀴가 중앙에서 벗어난 상태에서는 첫 명령이 큰 이동을 만들 수
있습니다.

벤치 시퀀스의 기본값은 다음과 같습니다.

```yaml
steer_command_publish_period_ms: 20
steer_target_change_interval_ms: 3000
steer_sequence_start_delay_sec: 3.0
steer_sequence_repeat: true
```

시험 E-stop은 다음처럼 조작합니다.

```bash
ros2 param set /steering_bench_test_node test_estop_active true
ros2 param set /steering_bench_test_node test_estop_active false
```

STM_A와 STM_B의 실제 모드·E-stop을 사용하고 벤치 노드만 조향 명령을 발행하게
하려면 다음처럼 실행합니다.

```bash
ros2 launch control_tools steering_bench_test.launch.py \
  drive_mode_publish_enabled:=false \
  safety_injection_enabled:=false
```

이 혼합 벤치에서는 STM_A가 `/drive/status`, STM_B가 `/brake/status`, 벤치 노드가
`/vehicle/command`를 각각 발행합니다. 시험 E-stop publisher는 생성되지 않습니다.


## 토크 식별 T0–T2

`torque_sysid_bench.launch.py`는 T0 부호, T1 마찰, T2 관성 시험 중 하나만 실행한다.
읽기 전용 조향 노드와 실제 torque를 보내는 측정 노드를 함께 띄우므로
`vehicle.launch.py`나 다른 KEYA 송신원과 동시에 실행하면 안 된다. 바퀴를 띄우고
차량 정지·MANUAL/중립·가속 페달 해제·E-stop 준비, 대상 CAN과 KEYA 0020=2를 확인한 뒤
T0부터 진행한다. 이 문서의 코드·테스트 결과는 물리 부호나 안전 한계의 증명이 아니다.

```bash
ros2 launch control_tools torque_sysid_bench.launch.py test:=t0 direction:=1
ros2 launch control_tools torque_sysid_bench.launch.py test:=t1 direction:=1
ros2 launch control_tools torque_sysid_bench.launch.py test:=t2 level:=300
```

기록은 기본적으로 외장 저장소
`~/Storage/ROS2_Workspace_offload/Control/torque_sysid`에 생성된다. 외장 저장소가 별도
mount가 아니면 launch가 기록 전에 중단한다. T2를 두 개 이상의 raw level로 기록한 뒤:

```bash
ros2 run control_tools torque_sysid_metrics.py inertia \
  <level300-bag>:300 <level500-bag>:500 --friction-raw <T1-result>
ros2 run control_tools torque_sysid_metrics.py step <step-bag> --csv <output.csv>
```

측정 도구는 조향 기구의 마찰·관성 특성을 분석하는 용도이며 PD 제어 설정을 변경하지 않는다.

## 바퀴 각도·데드존 측정

`wheel_steering_measurement.launch.py`는 외부 `ebimu_driver`의 좌우 IMU와
읽기 전용 `lateral_ctrl`를 함께 실행합니다. 모터를 구동하는 launch가 아닙니다.
데드존 측정은 [측정 절차와 과거 결과](DEADZONE_MEASUREMENT.md)를 따릅니다.

## 구성

- `src/`: 벤치·데드존 C++ 노드
- `launch/`, `config/`: 단독 녹화 및 측정 실행 설정
- `control_tools/topics.py`: 공통 Control 토픽과 경로별 센서·시나리오 토픽
- `control_tools/parameter_capture.py`: 실행 중 조향·거버너 파라미터 및 STM_A readback 조회
- `control_tools/recording.py`, `record_mcap.py`: 공통 MCAP 저장과 독립 녹화 생명주기
- `scripts/`, `control_tools/longitudinal_analysis.py`: CSV 변환과 분석
