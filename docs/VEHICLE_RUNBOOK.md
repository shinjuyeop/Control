# 실차 운용

설치·빌드·장치 설정을 마친 차량 PC 기준입니다.
구조는 [Architecture](ARCHITECTURE.md), 장치 준비는 [Firmware](../firmware/README.md)를 참고합니다.

## 1. 준비

각 터미널에서 workspace 환경을 불러옵니다. 아래 명령은 **zsh 기준**입니다.
이미 설정된 `si` 별칭을 사용해도 됩니다.

```zsh
cd ~/KAI_ws
source install/setup.zsh
```

전원·E-stop·CAN 배선을 확인하고 **MANUAL·정지 상태**에서 시작합니다.
조향 출력은 기본 허용 상태입니다.

튜닝값을 변경할 때는 구동 스택을 켜기 전에 실행합니다.

```zsh
ros2 run control_tuning control_tuning_tui
```

시험에 쓸 **주행 / 공중** 프리셋을 선택하고 `S`로 저장합니다.
2번에서 같은 프리셋으로 실행해야 적용됩니다. 구동 중 값을 변경했다면
MANUAL·정지 상태에서 구동 스택을 재시작합니다. [튜닝 상세](../software/src/control_tuning/README.md)

현재 저장된 주행·공중 프리셋은 모두 Speed·사인파이고 회생·feedforward는 꺼져 있습니다.
값은 [프리셋 표](../software/src/control_tuning/README.md#두-프리셋과-schema-v3)를 기준으로 확인합니다.
현재 Host는 STM_A longitudinal ABI v5 설정을 보내므로 STM_A firmware도 v5가 필요합니다.
Git 갱신이나 Host 빌드만으로 STM_A firmware가 갱신되지는 않습니다.

## 2. 구동 스택

```zsh
ros2 launch control_launch vehicle.launch.py
```

기본은 주행 프리셋 `driving`입니다. 공중 시험은 다음 명령으로 **대신** 실행합니다.

```zsh
ros2 launch control_launch vehicle.launch.py tuning_preset:=airborne
```

STM_A calibration 적용·readback의 `HOST_OVERRIDE`와 선택 프리셋 값을 확인합니다. 실패하면 MANUAL·정지·페달 해제를
확인하고 스택을 다시 실행합니다. [launch 상세](../software/src/control_launch/README.md)

## 3. 관측

각각 별도 터미널에서 실행합니다. 아래 접속 링크는 `Ctrl+클릭`으로 브라우저에서 엽니다.

### Foxglove bridge

PC당 하나만 실행합니다. 이미 켜져 있으면 접속만 합니다.

```zsh
ros2 launch foxglove foxglove_bridge.launch.py
```

- 로컬: [Foxglove 열기 · 127.0.0.1:8765](https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2F127.0.0.1%3A8765&openIn=web)
- Tailscale: [Foxglove 열기 · 100.112.72.27:8765](https://app.foxglove.dev/~/view?ds=foxglove-websocket&ds.url=ws%3A%2F%2F100.112.72.27%3A8765&openIn=web)

레이아웃은 [`control.json`](../software/src/foxglove/control.json)을 import합니다.
[Foxglove 상세](../software/src/foxglove/README.md)

### Vehicle Console

```zsh
ros2 launch vehicle_console vehicle.launch.py \
  allowed_hosts:=100.112.72.27 \
  web_python:="$HOME/KAI_ws/src/Control/software/src/vehicle_console/.venv/bin/python"
```

- 로컬: [http://127.0.0.1:8766/](http://127.0.0.1:8766/)
- Tailscale: [http://100.112.72.27:8766/](http://100.112.72.27:8766/)

`100.112.72.27`은 기존 차량 PC 주소입니다. 주소가 달라지면 링크와 `allowed_hosts`를
함께 바꿉니다. Console 원격 접속은 기존 Tailscale Serve 설정이 필요합니다.
[Console 설치·원격 접속](../software/src/vehicle_console/README.md#원격-접속)

## 4. 녹화

**시험 TUI를 쓸 경우 5번에서 녹화 ON을 선택하고 이 단계는 건너뜁니다.**
수동 주행·실제 Planning 시험에서는 별도 터미널에서 독립 녹화를 실행합니다.

```zsh
ros2 launch control_tools mcap_logging.launch.py output_base_dir:="$HOME/KAI_ws/records"
```

`녹화 준비 완료`를 확인하고 다음 단계로 넘어갑니다. 종료 시 `Ctrl+C`를 눌러 마감을 기다립니다.
MCAP과 실제 튜닝값은 `~/KAI_ws/records/` 아래 시험별 폴더에 저장됩니다.
[녹화 옵션·분석 명령](../software/src/control_tools/LOGGING.md)

## 5. 명령원 선택

수동 주행은 명령원을 실행하지 않고 6번으로 넘어갑니다.
실제 Planning 또는 시험 TUI 중 하나만 사용합니다. `/planning/command` 발행자가 중복되면 안 됩니다.
Planning 없이 시험할 때는 다음 TUI를 실행합니다.

```zsh
ros2 run vehicle_test_scenarios vehicle_test_scenario_runner
```

시험·속도·조향 진폭·시험 조건을 확인하고 **MCAP 녹화 ON**을 선택합니다.
`R`을 눌러 녹화·설정 조회를 마친 뒤 AUTO 대기 상태에서 6번을 확인합니다.
`S` / `Space`는 속도 0·`brake_engage=false`로 정상 종료하고 기록을 마감합니다. `Q`는 종료입니다.
정상 종료는 토크컷·타력감속이며 물리 브레이크 정지를 뜻하지 않습니다.
기본 속도는 0이지만 조향 진폭은 별도입니다.
[시나리오·키 조작 상세](../software/src/vehicle_test_scenarios/README.md)

## 6. 실행 확인

```zsh
ls -l /dev/stm_a /dev/stm_b
ip -details link show can0
ros2 topic info /planning/command          # 수동 주행 0개 / Planning·TUI 실행 중 1개
ros2 topic info /vehicle/command           # governor 1개
ros2 topic echo /drive/status --once
ros2 topic echo /drive/longitudinal_status --once
ros2 topic echo /steering/status --once
ros2 topic echo /brake/status --once
```

장치 연결·CAN 상태·명령 수신·피드백 validity·차단 사유를 확인합니다.
종방향 설정은 요청값이 아니라 `longitudinal_status` readback으로 판단합니다.
[상태 해석](INTERFACES.md)

자율주행·시나리오 시험은 준비가 끝나면 물리 스위치를 AUTO로 전환합니다.
시험 TUI는 카운트다운 후 시험을 시작합니다. 수동 주행은 MANUAL을 유지합니다.

## 7. 정지·종료

1. 시험 TUI는 `S` / `Space`로 정지 요청합니다. 비상 상황은 물리 E-stop을 사용합니다.
2. 차량 정지 후 MANUAL로 전환합니다.
3. 녹화 마감 완료를 확인합니다. 독립 recorder는 `Ctrl+C`로 마감합니다.
4. TUI `Q`, Planning·구동 스택·관측 화면 순서로 종료합니다.

문제가 있으면 [안전·복구 기준](SAFETY.md),
보드를 교체하면 [펌웨어 교체 절차](../firmware/README.md#보드-교체-절차)를 따릅니다.
