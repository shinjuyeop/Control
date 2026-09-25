# Longitudinal MPC

PC가 목표속도를 모터 상전류로 변환하는 ROS 2 C++ 제어기입니다. Gazebo와 실차가
같은 전류 명령·상태 인터페이스를 사용하며, 주행 기록에서 식별한 모델 YAML을 읽습니다.

## 구성

- `src/current_mpc.cpp`: ROS 독립 OSQP QP, 예측 행렬과 solver 메모리를 초기화 때 구성합니다.
- `src/controller_node.cpp`: 상태 수신, 20 Hz 제어, 명령 발행과 실행시간 기록입니다.
- `scripts/fit_longitudinal`: MCAP 식별·독립 예측 검증입니다.
- `scripts/evaluate_longitudinal`: 차체 속도·전류·가속도·jerk와 계산시간 비교입니다.
- `longitudinal_reference`: STM_A의 `longitudinal_controller.c`를 호스트에서도 빌드합니다.
  PI 비교와 SITL의 설정·전류 수신 판정에 실제 펌웨어 계산 코드를 재사용합니다.

C++ 의존성은 ROS Humble의 `osqp_vendor`, Eigen3, yaml-cpp입니다. 오프라인 도구는
ROS Python, NumPy, SciPy, Matplotlib, PyYAML을 사용합니다.

## 입출력과 구동

| 입력 | 역할 |
|---|---|
| `/vehicle/command` | governor가 전달한 현재 목표속도·브레이크 |
| `/vehicle/state` | 제어에 사용하는 속도 |
| `/drive/status` | 실제 상전류, CAN 적용 전류, 모드, fault와 측정 유효성 |
| `/drive/longitudinal_status` | STM_A에 적용된 외부 전류 모드와 한계 |

출력은 `/vehicle/drive_current_command`의 `kaiev26_msgs/DriveCurrentCommand`입니다.
`header.stamp`는 ROS 시각, `phase_current_a`는 **모터 상전류 A**입니다. CAN 적용 전류와
PC 지령은 별도로 기록합니다. 식별 모델의 입력은 MPC가 실제로 출력하는 **PC 지령**입니다.
이 경계를 맞춰 PC→CAN 전달과 모터 응답을 모델에 포함합니다.

구동은 기존 차량 launch가 맡습니다.

```bash
ros2 launch control_launch vehicle.launch.py \
  longitudinal_controller:=mpc longitudinal_model_file:=/absolute/path/model.yaml
```

`longitudinal_controller` 선택은 다음과 같습니다.

| 값 | 실행 |
|---|---|
| `firmware` | 기존 프리셋의 Speed 또는 STM_A Torque PI |
| `sysid` | 외부 전류 모드 설정 후 기존 시나리오 러너가 전류를 발행 |
| `pi` | PC에서 STM_A와 같은 C PI 계산을 실행하는 비교 모드 |
| `mpc` | PC의 OSQP MPC |

외부 전류 모드는 MANUAL·정지·무전류 상태에서 적용 readback을 확인한 뒤 governor와
제어기를 시작합니다. 설정 적용 실패에는 launch가 종료됩니다. 구동 중 설정은 고정합니다.
실차 적용에는 새 메시지/라이브러리와 STM_A 펌웨어, Host를 함께 배포해야 합니다.

Gazebo의 동일 경로는 Simulation의 차량 launch로 실행합니다.

```bash
ros2 run kaiev26_gazebo_bringup gazebo_launcher
```

GUI에서 차량 프로파일을 `SITL CAN`으로 고르고 하위 제어의 종방향 PI 또는 MPC를
선택합니다. 가상 CAN 인터페이스는 `simulation.yaml`의 `sitl_can.can_interface`를
사용하며, MPC 모델은 `longitudinal_mpc` 패키지 설정을 사용합니다.
GUI는 `/simulation/driver/control_input`으로 MANUAL/AUTO를 전환합니다.
목표 명령은 별도 판단 또는 공용 러너가 `/planning/command`로 발행하며,
차량 환경의 command_governor가 `/vehicle/command`로 중계합니다.

## 식별과 비교

전류 Step과 코스트다운은 기존 시나리오 TUI 또는 `vehicle_test_scenario_node`를 사용합니다.
기본 전류와 차속은 0이며, 전류 시험에는 `external_current` readback이 필요합니다.
속도 상한에 도달하면 전류를 0으로 내립니다. MPC/PI와 전류 시험은 한 번에 하나만 실행합니다.

```bash
ros2 run vehicle_test_scenarios vehicle_test_scenario_node --ros-args \
  -p use_sim_time:=true -p enabled:=true -p wait_for_auto_mode:=true \
  -p auto_start_delay_sec:=3.0 -p scenario:=coastdown \
  -p current_high_a:=200.0 -p current_hold_sec:=8.0 \
  -p coast_duration_sec:=16.0 -p cruise_speed_kph:=30.0 -p scenario_repetitions:=2
```

기록은 기존 Control 도구를 사용합니다. Gazebo에서는 입력·관측과 같은 시계로 기록합니다.

```bash
ros2 run control_tools record_mcap --use-sim-time --require-external-storage \
  --output-base-dir ~/Storage/ROS2_Workspace_offload/longitudinal_mpc/run

ros2 run longitudinal_mpc fit_longitudinal \
  --train /path/train1/rosbag /path/train2/rosbag \
  --validation /path/held_out/rosbag --output /path/identified

ros2 run longitudinal_mpc evaluate_longitudinal \
  --pi /path/pi/rosbag --mpc /path/mpc/rosbag --output /path/comparison
```

실차에서는 `use_sim_time`을 사용하지 않습니다. MCU 상태의 ROS 발행 횟수를 독립적인
CAN 측정 횟수로 간주하지 않으며, MCAP의 공통 기록 시각으로 신호를 정렬합니다.
식별·튜닝·평가 기록은 주행 단위로 분리합니다. 데이터 파일과 그림은 외장 SSD에 저장합니다.

## 모델과 최적화

상태는 `x = [v, I_phase]`, 입력은 PC의 요구 전류 `u`입니다.

`x[k+1] = A x[k] + B u[k] + c`, `dt = 0.05 s`

전류 응답은 측정 전류와 PC 요구 전류로 추정하고, 속도 계수는 2초 궤적 오차를
최소화해 구합니다. 단일 샘플의 속도를 미분해 회귀하는 과정은 사용하지 않습니다.
이동 중 affine 저항 모델의 식별 범위는 0.2–8.33 m/s이며, 출발은 폐루프 시험으로 확인합니다.
각 계수는 통신과 관측을 포함한 유효 계수입니다. 질량·토크 상수 등의 개별 물리량을
이 기록만으로 각각 식별했다고 해석하지 않습니다.

기본 예측 구간은 40 step/2초입니다. 최소화하는 비용은 다음과 같습니다.

`Σ [ qv (v[k+1] - r)² + rd (u[k] - u[k-1])² + ru (u[k] - u_equilibrium)² ]`

- 전류는 `0 ≤ u ≤ current_max_a`로 제한합니다.
- 매 step 전류 변화는 `-slew_down × dt ≤ Δu ≤ slew_up × dt`입니다.
- 초기 `u[k-1]`은 직전에 PC가 발행한 전류입니다.
- 목표속도는 현재 판단 출력으로 예측 구간 동안 유지합니다.
- `P`와 제약 행렬은 고정하고 매 주기 선형 비용과 경계 벡터를 갱신합니다.
  OSQP는 이전 해를 재사용합니다.
- solver 실패나 입력 상실에는 0 A를 발행합니다. 브레이크/E-stop은 변화율 제한보다 우선합니다.

MPC가 0 A보다 낮은 전류를 요청하지 않으므로 감속 능력은 타력 감속으로 제한됩니다.
급격한 목표 하향의 초기 오차와 정지 요청은 이 제약을 고려해 평가해야 합니다.
정지·긴급제동 명령과 최종 브레이크 동작은 기존 경로가 담당합니다.

## 측정 범위

`/control/longitudinal/diagnostics`는 solver 시간, 상태 검사부터 전류 발행까지의 시간,
callback 간격, 50 ms 계산 기한 초과 횟수와 solver 상태를 기록합니다.
관측 지연과 운영체제 스케줄링은 별도이므로 측정된 짧은 계산시간을 hard real-time 보장으로
표현하지 않습니다. 본 구현은 설치된 OSQP C API를 직접 사용합니다.

가속도와 jerk는 `/gnss/fix_velocity`의 차체 속도에서 같은 0.55초/3차
Savitzky–Golay 방법으로 계산합니다. 바퀴 속도와 차체 속도 차이도 함께 제시합니다.
전류 변화율은 승차감 지표와 구분합니다. 정착시간은 목표 ±0.2 m/s를 2초 유지하는
첫 구간으로 정의하고, 관측창에서 정착하지 않으면 미관측으로 기록합니다.

실차 이식은 같은 관측·입력 계약과 식별 모델이 유효한 운용 구간을 전제로 합니다.
실차의 RPM 이상, 전류 응답, 경사·하중·SOC 영향은 실차 시험에서 확인해야 합니다.
모델·전류 한계·지연·가중치가 실차에 적합한지는 해당 실차 기록으로 판단합니다.

## 이론 참고

- [Rawlings, Mayne, Diehl — Model Predictive Control: Theory, Computation, and Design](https://sites.engineering.ucsb.edu/~jbraw/mpc/)
- [Stellato et al. — OSQP, Mathematical Programming Computation, 2020](https://web.stanford.edu/~boyd/papers/osqp.html)
- [OSQP의 선형 MPC 구성 예제](https://osqp.org/docs/examples/mpc.html)
