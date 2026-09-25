# steering_mpc

실차 SYSID bag에서 KEYA 토크 입력→실제 조향각 모델을 식별하고, 기존 `lateral_ctrl` 안에서 50 Hz 조향 토크 MPC를 실행한다. C++ solver는 `mpc_common`을 사용한다. 모델 식별·비교용 Python은 `scripts/`에 설치된다. 패키지 빌드는 `ament_cmake`다.

`lateral_ctrl`가 CAN 출력, Enable/Disable, E-stop, 피드백 만료, peak/sustained 토크 한계를 계속 담당한다. MPC는 기존 PD와 같은 안전 출력을 사용한다.

## 조향 모델 식별

ROS 환경을 source한 뒤 실행한다. 원본 bag은 보존하고 결과는 외장 SSD의 프로젝트 폴더에 저장한다.

```bash
ros2 run steering_mpc fit_lateral \
  --bags <SYSID_campaign> --calibration <steering_motor_calibration.yaml> \
  --output <artifact_root>/identified
```

입력은 LEFT-positive KEYA raw 명령을 10000으로 정규화한 값이다. 현재 시험의 `torque_direction_sign=1`을 사용한다.
출력은 원시 encoder count를 wheel-hub 보정 다항식으로 환산한 중앙 등가각이다.
모터 전류 telemetry를 토크 명령으로 대신 사용하지 않는다. raw 단위는 Nm나 실측 A가 아니다.

각도·각속도를 상태로 하는 모델은 다음과 같다.

`x[k+1] = A x[k] + B u[k-d] + c`, `x = [δ, δ̇]`

연속 모델 `δ̈ = gain·u − damping·δ̇ − stiffness·δ + bias`를 정확히 이산화한다.
`config/kaiev26_260919_8bags_mpc.yaml`은 2026-09-19의 지정한 8개 bag에서
앞의 두 기록을 시작 후 23초로 제한하고 AUTO/ACTIVE 이동 구간만 사용한 0.6초
예측 모델이다. `config/kaiev26_260919_8bags_plant.yaml`은 같은 자료의
연속 재생에 맞춘 Gazebo KEYA 토크 플랜트용 모델이다. 두 파일의 입력 단위와
상태 정의는 같으며, 각각 MPC와 플랜트에 사용한다. 큰 스텝의 진동은 두 모델이
충분히 재현하지 못한다. 구간과 오차는
`~/Storage/ROS2_Workspace_offload/vehicle_sysid_analysis/20260924/eight_lateral_bags/REPORT.md`에 있다.

명령과 피드백은 MCAP 수신 시각에 정렬하며, AUTO/ACTIVE/유효 피드백 구간을 사용한다.
수신 공백과 보정 범위 밖은 분리한다. 파일 이름순 매 세 번째 run은 검증용으로 남겨 학습에서 제외한다.
0.6초 다단계 예측오차로 적합하며, 검증의 상태 초기화에 사용하는 각속도는 오프라인 평활 미분이다.
이는 기존 PD 폐루프 기록에서 추정한 경험 모델이다. 물리 계수의 불편 추정이나 실차 전 범위 유효성을 보장하지 않는다.
검증 그림은 매 0.1초 초기화한 예측 창이며 연속 자유응답으로 해석하지 않는다.

## 조향 MPC 실행

기존 차량 bringup에서 선택한다.

```bash
ros2 launch control_launch vehicle.launch.py \
  actuation_mode:=torque torque_controller:=mpc steering_model_file:=<identified/model.yaml>
```

이 차량 launch는 실제 CAN 출력을 포함한다. 모델 재식별·HIL·차량별 한계 확인을 마친 환경에서 사용한다.
Gazebo에서는 기존 Simulation bringup이 동일한 조향 노드와 가상 CAN을 실행한다.

```bash
ros2 run kaiev26_gazebo_bringup gazebo_launcher
```

GUI에서 차량 프로파일을 `SITL CAN`으로 고르고 하위 제어의 종방향 PI/MPC와
조향 PD/MPC를 선택한다. 조향 모델은 `steering_mpc` 패키지 설정을 사용한다.
가상 CAN 인터페이스는 `simulation.yaml`의 `sitl_can.can_interface`를 사용한다.

Simulation의 토크 플랜트는 KEYA 브리지에서 지연·각도·각속도를 적분한다. 이 경로의 downstream
`steering_response_delay_sec`와 `steering_response_time_constant_sec`는 0으로 설정해 액추에이터 지연을 중복 적용하지 않는다.
GUI는 위 8개 bag의 연속 재생 모델을 KEYA 브리지 플랜트에,
0.6초 예측 모델을 조향 MPC에 전달한다.

조향 MPC는 20 ms, horizon 50을 기본으로 한다. 비용은 각도 오차·각속도·정규화 토크·토크 변화량이다.
`mpc_angle_weight=4000`, `mpc_rate_weight=0.1`, `mpc_delta_torque_weight=0.02`를 기본값으로 사용한다.
`torque_peak_limit_raw`, `torque_sustained_limit_raw`, peak 시간 예산과 보정 각도 한계를 공유한다.
OSQP는 희소 구조와 workspace를 재사용하며 warm start한다.
절대/상대 허용오차는 `1e-4`/`1e-3`, 조향 solve 시간 예산은 16 ms이다.
`solved inaccurate`는 OSQP가 정의한 근사해로 받아들이고 WARN으로 기록한다. 반환할 첫 입력은
입력 한계와 변화량 한계의 교집합으로 제한한다. 시간 초과·비실현 가능·해 없음은 정지 처리한다.
[OSQP 상태 정의](https://osqp.org/docs/release-0.6.3/interfaces/status_values.html)를 따른다. 실시간 보장은 실행 호스트에서 따로 측정해야 한다.

## 비교

```bash
ros2 run steering_mpc evaluate_lateral --pd <PD_bag> --mpc <MPC_bag> \
  --calibration <steering_motor_calibration.yaml> --output <comparison_dir>
```

가제보에서 측정한 모델·주행 성능과 실차 bag의 예측 성능은 [비교 보고서](../../../docs/reports/2026-09-24_lateral_mpc.md)에 구분해 기록했다.
9월 19일 지정 8개 bag 모델을 적용한 SITL CAN 조향 PD/MPC 결과는
`~/Storage/ROS2_Workspace_offload/vehicle_sysid_analysis/20260924/gazebo_steering_comparison/REPORT.md`에 있다.
