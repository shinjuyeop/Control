# C++ 횡방향 MPC 구현·Gazebo 비교 (2026-09-24)

조향 토크 MPC(50 Hz)와 경로 MPC(20 Hz)를 구현하고, 기존 CAN 경로와 시나리오러너로 Gazebo에서 실행했다. 실제 차량 MPC 주행은 수행하지 않았다.

| 약 100 m, 2.5 m/s 경로 | 횡오차 RMSE | 최대 절대오차 |
| --- | ---: | ---: |
| PP + PD | 0.195 m | 0.342 m |
| PP + 조향 MPC | 0.105 m | 0.187 m |
| 경로 MPC + 조향 MPC | 0.264 m | 0.677 m |

최종 사인 시험 RMSE는 PD 0.459°, MPC 0.471°였다. 현재 가장 좋은 경로 조합은 PP+조향 MPC다. 경로 MPC의 우위나 승차감 개선을 주장하지 않는다.

실차 SYSID bag에서 raw torque와 보정 엔코더 각도로 2상태 경험 모델을 적합했다. 별도 큰 스텝 run의 0.6초 예측 RMSE는 2.246°와 3.809°로, 이 모델을 실차 전 범위에 적용하기에는 추가 식별이 필요하다. 다른 판단 주행 bag을 이용한 yaw 예측 오차도 별도 기록했다.

최종 경로 시험의 solver 실패는 0회였다. 조향 근사해 경고는 각 주행에서 3회/2회였다. MPC 함수 계산시간의 최대는 조향 16.04 ms, 경로 2.94 ms이며 전체 OS/DDS/CAN 주기의 hard real-time 보장은 아니다.

- [조향 토크 MPC·SYSID](../../software/src/steering_mpc/README.md)
- [경로 MPC·경로 평가](../../software/src/path_mpc/README.md)
- [공통 OSQP 계산부](../../software/src/mpc_common/README.md)
- 외장 SSD 결과: `~/Storage/ROS2_Workspace_offload/lateral_mpc/20260924/`
- `REPORT.md`: 모델·이론·조건·검증 한계와 원본 bag 경로
- `PORTFOLIO.pdf`: 지원서용 4쪽 자료
- `SELF_INTRO_DRAFT.md`: 실제 수행 범위에 맞춘 기술 경험 초안
- `steering_comparison/`, `path_comparison/`: 최종 PNG/SVG 및 지표 JSON

실차 적용 전에는 모델 잔차와 토크 요동을 개선하고 동일 러너로 검증해야 한다. 종방향 STM_A 재부팅 후 Speed 모드 복귀 경로도 별도 배포 차단 항목으로 남아 있다.
