# path_mpc

`path_controller`는 Decision의 목표 경로와 차량 상태를 받아 20 Hz 목표 조향각을 계산한다. C++/OSQP QP 계산은 `mpc_common`을 사용하며, 실행은 기존 Decision launch가 담당한다. 패키지 빌드는 `ament_cmake`다.

`/control/lateral/target`은 `ackermann_msgs/AckermannDriveStamped`로 계산 시각과 LEFT-positive 목표 조향각을 전달한다. Decision이 기존 속도·제동 명령과 합쳐 최종 명령을 발행한다. 해가 실패하거나 150 ms를 넘기면 기존 정지 처리가 동작한다.

## 경로 MPC와 식별 모델

조향 MPC를 켜고 기존 스텝·사인을 실행한 뒤 폐루프 응답을 적합한다.

```bash
ros2 run path_mpc fit_lateral_response \
  --actuator-bag <steering_MPC_bag> --vehicle-bags <yaw_fit_bags> \
  --validation-bag <separate_yaw_bag> --calibration <steering_motor_calibration.yaml> \
  --output <artifact_root>/path_model.yaml
```

경로 모델 상태는 횡오차·방향오차·yaw rate·실제 조향각이다. 속도와 경로 곡률로 예측식을 갱신한다.
측정한 내부 조향 응답의 gain/bias/시정수를 사용하고 조향 목표의 학습 범위를 적용 범위로 제한한다.
측정 범위를 넓히려면 해당 범위의 새 조향 시험으로 모델을 다시 만든다. 이 범위는 기계적 최대 능력이 아니다.
20 ms 기록에서 그보다 짧게 적합된 yaw lag는 시간 해상도 이하로 표시하며 정확한 지연 식별이라고 해석하지 않는다.

```bash
ros2 launch kaiev26_decision decision_test.launch.py \
  case:=1 platform:=simulation lateral_controller:=mpc \
  path_model_file:=<path_model.yaml> route_config:=<route.yaml> speed_mps:=2.5
```

Decision TUI에서도 `L`로 MPC를 선택하고 `M`으로 모델 경로를 입력한다. 별도의 MPC launch 실행은 필요 없다.
`/control/lateral/target`은 `ackermann_msgs/AckermannDriveStamped`이고 사용 필드는 timestamp와
`drive.steering_angle`(rad, 좌회전 양수)이다. 경로 입력 좌표계는 `base_footprint`이다.

## 경로 평가

```bash
ros2 run path_mpc evaluate_path --run PP_PD=<bag1> --run PP_MPC=<bag2> \
  --run MPC_MPC=<bag3> --route <route.yaml> --output <path_comparison_dir>
```

MCAP 경로 비교와 모델 정확도의 한계는 [비교 보고서](../../../docs/reports/2026-09-24_lateral_mpc.md)에 정리했다.
