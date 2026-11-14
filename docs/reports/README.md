# 시험·분석 기록

[프로젝트 개요](../../README.md) · [문서 지도](../README.md)

날짜별 분석, 시험 계획과 구현 결과를 보관합니다. 문서 기준일과 실제 시험일은 다를 수 있으며,
내용은 해당 시점의 기록입니다. 현재 동작은 [아키텍처](../ARCHITECTURE.md)·[인터페이스](../INTERFACES.md)·
[안전 문서](../SAFETY.md), 실행 절차는 [실차 runbook](../VEHICLE_RUNBOOK.md)을 따릅니다.

## 주제별 읽기 순서

| 관심 주제 | 근거와 이어 읽을 문서 |
|---|---|
| 실차 조향 문제와 복구 판단 | [무응답 원인](2026-09-18_steering_fault.md) → [실주행·SYSID 분석](2026-09-19_control_tuning_regen_fault_review.md) → [후속 PD 비교 계획](2026-09-20_lateral_pd_experiments.md) |
| 종방향 제어 전환 | [Speed 실측과 Torque PI 도입](2026-09-21_longitudinal_torque_pi.md) → [feedforward 설계](2026-09-29_longitudinal_feedforward.md) → [사인파 기준 재구성](2026-10-01_ezkontrol_sine_rework.md) |
| 회생제동과 부하 차이 | [ABI v5 구현 기록](2026-10-02_regen_governor_rework.md) → [공중 RBS 실측](2026-10-08_rbs_tuning_report.md) → [지상 기준선·시뮬레이션·후속 시험 계획](2026-10-09_ground_longitudinal_tuning_plan.md) |
| 모델 기반 횡방향 제어 | [실차 bag 식별과 Gazebo PD/MPC 비교](2026-09-24_lateral_mpc.md) → [조향 MPC](../../software/src/steering_mpc/README.md) · [경로 MPC](../../software/src/path_mpc/README.md) |

공중 RBS 보고서는 바퀴 무부하 시험입니다. 지상 튜닝 지침의 10/03 기록 분석은 실측에 근거하지만,
후보 파라미터의 개선 수치는 시뮬레이션 예측입니다. 횡방향 MPC 보고서는 Gazebo 비교이며 실차 MPC 주행을 수행하지 않았습니다.
계획 문서의 시험표나 host 테스트 통과를 실제 차량 시험 완료로 읽지 않습니다.

## 날짜별 기록

| 기준일 | 구분 | 문서 |
|---|---|---|
| 2026-09-18 | 원인 분석·수정 기록 | [ASMS 전환 후 조향 무응답](2026-09-18_steering_fault.md) |
| 2026-09-19 | 실주행·SYSID 기록 분석 | [조향 튜닝·종방향 감속·회생·fault](2026-09-19_control_tuning_regen_fault_review.md) |
| 2026-09-20 | 기록·설정 검토 | [EZkontrol 감속 추종과 내부 설정](2026-09-20_ezkontrol_decel_tuning.md) |
| 2026-09-20 | 시험 계획 | [현장 시험 순서](2026-09-20_field_test_plan.md) |
| 2026-09-20 | 기존 측정 분석·시험 제안 | [횡방향 PD 비교와 차속별 게인 검토](2026-09-20_lateral_pd_experiments.md) |
| 2026-09-21 | 구현·host 검증 기록 | [조향 복구 정책](2026-09-21_steering_availability.md) |
| 2026-09-21 | 구현·기존 실측·시험 후보 | [AUTO Torque PI와 실차 기준선](2026-09-21_longitudinal_torque_pi.md) |
| 2026-09-21 | 9/20 MCAP 분석 | [종방향·전류 피크·RPM 이상](2026-09-21_0920_longitudinal_review.md) |
| 2026-09-24 | 식별·Gazebo 비교 | [C++ 횡방향 MPC](2026-09-24_lateral_mpc.md) |
| 2026-09-29 | 구현·식별·튜닝 후보 | [Torque feedforward와 MANUAL 페달 envelope](2026-09-29_longitudinal_feedforward.md) |
| 2026-10-01 | 공중 로그 분석·구현·시험 계획 | [EZkontrol 사인파 기준 재구성](2026-10-01_ezkontrol_sine_rework.md) |
| 2026-10-02 | 구현·host/ARM 검사·시험 계획 | [회생제동 ABI v5와 Governor 변경 이력](2026-10-02_regen_governor_rework.md) |
| 2026-10-08 | 공중 무부하 실측 | [PI 회생·feedforward·재인가 튜닝](2026-10-08_rbs_tuning_report.md) |
| 2026-10-09 | 지상 실측 분석·시뮬레이션·시험 계획 | [지상 종방향 구동·회생 튜닝 지침](2026-10-09_ground_longitudinal_tuning_plan.md) |

10/02 문서의 Governor slew 제거안은 문서 상단의 10/07 통합 메모에서 채택하지 않았다고 명시합니다.
현재 코드에는 slew 기능이 있으며, 저장 프리셋에서의 활성 여부와는 별개입니다.
현재값은 [Governor](../../software/src/command_governor/README.md)와
[Control Tuning](../../software/src/control_tuning/README.md)에서 확인합니다.

## HIL·소프트웨어 검증 기록

| 기록 | 범위 |
|---|---|
| [HIL runbook](../HIL_RUNBOOK.md#15-진행-상태와-남은-작업) | 보드 연결·명령·E-stop·PWM·CAN 시험과 재플래시 기록 |
| [Web HIL 검증](../../software/src/vehicle_console/HIL_WEB.md#검증) | Web 조작, 시나리오, UNO 가상 LCD와 실제 패널 검증의 구분 |
| [Console 검증](../../software/src/vehicle_console/CONSOLE_VALIDATION.md) | 통합 당시 소프트웨어 검사, 실제 ROS pub/sub 및 관측 범위 |
| [STM runtime metrics](../../firmware/common/README.md) | 주기·wall time·지터의 의미, 460800-baud board-only HIL 관측 |

## 자료 범위와 과거 설계 메모

원본 MCAP, 분석 CSV·plot과 일부 상세 결과는 보고서에 적힌 로컬·외장 저장소 경로에 있으며 Git에 포함되지 않습니다.
재분석에는 해당 Run의 원본 기록, 파라미터 snapshot/readback과 분석 도구가 필요합니다.
Git에 있는 요약만으로 원본 실험 전체를 재현할 수 있는 것은 아닙니다. [기록 도구와 형식](../../software/src/control_tools/LOGGING.md)

다음 문서는 당시 설계·시험 논의의 배경입니다. 현행 실행 절차나 검증 완료 목록으로 사용하지 않습니다.

- [9/29 Torque 시험 시나리오·튜닝 가이드](../260929_Long_TorqueTest.md)
- [EZkontrol 토크 제어 재구성 지침](../EZKONTROL_TORQUE_CONTROL_REWORK.md)
- [회생·Governor 변경 지침과 통합 메모](../EZKONTROL_REGEN_AND_GOVERNOR_REWORK.md)
- [10/08~09 작업 대화 요약](../conversation_1009.md)

새 기록은 `YYYY-MM-DD_topic_in_snake_case.md`로 작성하고, 시험 조건·사용 자료·수행한 검증과 남은 범위를 함께 적습니다.
