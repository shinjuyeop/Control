# MCAP 녹화와 분석

독립 녹화는 `control_tools`, 시험에 연동된 녹화는 `vehicle_test_scenarios` TUI로 실행합니다.
둘을 같은 시험에 중복 실행하지 않습니다. 원본은 MCAP이며 CSV는 분석용 파생물입니다.

## 독립 녹화

구동 스택을 먼저 실행한 뒤 별도 터미널에서 시작합니다.

```bash
ros2 launch control_tools mcap_logging.launch.py output_base_dir:="$HOME/KAI_ws/records"
```

`녹화 준비 완료`를 확인한 뒤 시험을 시작합니다. `Ctrl+C`로 MCAP을 정상 마감합니다.
초기 제어 설정을 조회하지 못하거나 필수 토픽이 기록되지 않으면 결과를 `incomplete`로 남깁니다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `output_base_dir` | `~/KAI_ws/records` | 실행 사용자의 기록 부모 경로 |
| `require_external_storage` | `false` | true면 기존 외장 `~/Storage/ROS2_Workspace_offload` mount·하위 경로 검사 |
| `capture_parameters` | `true` | 실제 조향·거버너 설정 snapshot을 YAML과 MCAP으로 저장 |
| `require_longitudinal` | `true` | 새 STM_A의 유효하고 fresh한 longitudinal readback 요구 |

제어기가 없는 센서 전용 기록은 `capture_parameters:=false`를 명시합니다.
STM_A 없는 벤치에서 조향 파라미터만 수집하려면 `require_longitudinal:=false`를 사용합니다.
이 경우 생략된 조회는 결과 파일에 드러나며 실제 설정을 추정해 채우지 않습니다.

```text
records/<시간>_independent/
├── rosbag/                   # MCAP + metadata.yaml
├── control_parameters.yaml  # 실제 설정 snapshot (수집 활성화 시)
└── recording.yaml           # 프로필·토픽·압축·완료 여부·실제 메시지 수
```

## 시나리오 TUI 녹화

```bash
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

`MCAP 녹화 ON`에서 `R`로 준비하고 `S`로 정리합니다. TUI는 recorder 연결과
실제 설정 조회가 끝난 뒤 시험을 진행합니다. 실차 모드에서는 Speed/Torque 구분 없이
`/drive/longitudinal_status`를 필수 구독·기록 대상으로 검사해 모드와 적용값을 확인합니다.
외부 Gazebo 진입점의 `require_ezkontrol_status=False`에서는 STM_A readback을 요구하지 않습니다.
녹화 OFF는 recorder와 파라미터 snapshot을 생성하지 않습니다.
시험 설정·콘솔 로그·종료 사유는 기존 `scenario.yaml`, `console.log`에 남습니다.

## 공통 데이터와 프로필 차이

두 경로 모두 MCAP `zstd_fast`를 사용합니다. 새 토픽은
[`control_tools/topics.py`](control_tools/topics.py)에서 관리합니다.

| 공통 Control 항목 | 용도 |
|---|---|
| `/planning/command`, `/vehicle/command` | 상위·최종 명령 |
| `/drive/status` | 속도·RPM·명령/실제 전류·전압·유효성·fault |
| `/drive/longitudinal_status`, `/drive/longitudinal_config` | 종방향 적용 설정·PI 상태와 설정 요청 |
| `/steering/status`, 조향 torque 상세 토픽 | 조향 응답·PD·제한·전류·전압 |
| `/brake/status`, `/vehicle/state`, `/vehicle/wheel_twist` | 제동·차량 상태 |
| `/diagnostics`, `/rosout`, STM_A/B `runtime_stats` | fault 복구·경고·실행 지연 |
| `/lv_battery/state`, `/lv_battery/link/status`, `/lv_battery/link/command`, `/lv_battery/link/ack` | LV 계측·정비 기록 |
| `/sysid/scenario`, `/sysid/control_parameters`, `/parameter_events` | 시험 라벨·초기 설정·변경 이벤트 |

기존 `/sysid/scinario` 오타는 publisher·config·Foxglove를 함께 `/sysid/scenario`로 통일했습니다.
과거 MCAP의 토픽 이름은 변경하지 않습니다.
독립 프로필은 기존 Planning/localization·좌우 IMU를, TUI는 기존 `/sysid/*` 경로 정보를 유지합니다.
카메라·LiDAR 선택은 기존 프로필과 동일합니다(독립의 `/lidar_points`, 두 경로의 Ouster 원시 패킷 등).
발행되지 않는 선택 토픽은 기록되지 않습니다. Position 모드의 torque 상세,
장착하지 않은 RP2350, 실제 변경 없는 `/parameter_events` 등이 해당합니다.

## 튜닝값 해석

- 조향·거버너: 녹화 연결 후 실행 노드의 전체 ROS 파라미터를 읽어 저장합니다.
- 종방향: `/drive/longitudinal_status`의 readback이 적용값입니다. config 토픽은 요청입니다.
- Speed의 명령 전류는 상한, Torque의 명령 전류는 목표 전류입니다.
- MCU PI 상세는 현재 5 Hz이며 bag 수신 시각 기준입니다. 매 제어 tick의 기록은 아닙니다.
- KEYA·EZkontrol 내부 장치 설정은 Host 파라미터와 별개이며 이 snapshot으로 읽지 않습니다.

## 분석

```bash
ros2 run control_tools mcap_to_csv.py <bag-directory> -o /tmp/control_csv
ros2 run control_tools torque_sysid_metrics.py step <bag-directory> --csv /tmp/steering.csv
ros2 run control_tools longitudinal_analysis <bag-directory> --pi-csv /tmp/pi.csv
```

종방향 분석은 실제 속도·전류와 calibration 이력, 수집된 PI 표본 수를 보고합니다.
`--pi-csv`는 동일 revision의 연속 PI 표본도 보존합니다. 원본 MCAP은 변경하지 않습니다.
