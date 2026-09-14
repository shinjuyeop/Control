# longitudinal_mpc

`/vehicle/command`의 현재 목표속도를 horizon 전체의 상수 reference로 사용해
`/vehicle/drive_current_command`에 양의 상전류를 발행하는 host-side MPC다.
solver는 MCU가 아니라 PC에서 실행한다.

## Solver

ROS Humble의 `osqp_vendor 0.2.0`이 설치한 OSQP C API 0.6.2를 직접 사용한다.
OSQP의 라이선스는 Apache-2.0이다. P와 constraint matrix, solver workspace는
초기화 뒤 재사용하고 각 tick에는 q/l/u와 warm start만 갱신한다. `polish=1`과
OSQP 기본 adaptive-rho를 사용하므로 solver 내부에서는 필요할 때 재분해할 수 있다.

상태는 `x=[vehicle speed m/s, actual phase current/100 A]`, 입력은
`u=commanded phase current/100 A`다. 20 Hz, 2 s horizon, 4-step control block,
0..300 A, 정상 slew 300 A/s를 사용한다.

## Physical parameterization

속도 상태는 다음 식으로 생성한다.

```text
dv/dt = effective_torque_scale * efficiency * final_drive * Kt
        / (mass * tire_radius) * phase_current
        - speed_damping * v - resistance_accel
```

기본값 `mass=285 kg`, `Kt=0.11 Nm/A`, `G=4.6`, `eta=0.95`,
`radius=0.2374765591 m`에서 nominal gain은 0.007102455388 m/s²/A다.
`effective_torque_scale=0.6839881619421034`는 2026-09-03 bag에서 얻은 기존
유효 gain 0.004857995406 m/s²/A를 기준점에서 보존한다. 질량과 Kt를 bag만으로
독립 식별했다는 뜻이 아니며, Kt는 48 V 카탈로그 추정이고 phase current가
정확한 dq torque current인지도 미확인이다.

SITL 기본 반경은 plant와 speed feedback의 지름 0.4749531182 m에 맞춘 값이다.
실제 STM_A의 0.47 m 지름을 쓰는 HILS에서는 `tire_radius_m=0.235`로 별도
override해야 한다.

질량이나 Kt를 바꿔도 `speed_damping_per_s`와 `resistance_accel_mps2`는 기존
bag에서 얻은 경험적 가속도 계수로 고정한다. 두 저항항까지 물리력으로 다시
scaling하는 모델은 아니다.

## Safety

brake, E-stop, drive stop request, valid drive fault, MANUAL 전환,
command/feedback timeout, solver 실패에서 current와 OSQP warm start를 0으로
reset한다. 해제 뒤 첫 current는 15 A부터 다시 시작한다. solver가 끝난 뒤에도
freshness를 재확인해 만료된 입력으로 계산한 결과를 새 timestamp로 발행하지 않는다.
