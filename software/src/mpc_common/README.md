# mpc_common

`steering_mpc`와 `path_mpc`가 사용하는 C++ `LinearMpc` QP 계산 라이브러리다. 상태 예측과 비용행렬 구성, OSQP workspace 재사용과 warm start, 입력·변화량·상태 제약을 제공한다. 차량 토픽과 모델 파일은 각 제어 패키지가 소유한다.
