# foxglove

Foxglove bridge와 KAI 차량 engineering layout만 제공한다. 차량 상태 요약
`/diagnostics`는 `vehicle_state_publisher/diagnostics_publisher`가 발행하며,
브라우저 기반 운영 화면은 `vehicle_console`이 담당한다.

```bash
ros2 launch foxglove foxglove_bridge.launch.py
```

노트북 Foxglove 앱에서 `ws://100.112.72.27:8765`로 연결하고
[`control.json`](control.json)을 import한다. bridge는 PC당 하나만 실행한다.
하나의 bridge가 ROS graph 전체를 전달하고 여러 클라이언트가 접속할 수 있다.

추천 항목은 다음과 같다.

- `/vehicle/command`: speed, steering, brake 목표
- `/drive/status`: 속도, RPM, 전압·전류·온도, timeout과 fault
- `/brake/status`: brake command와 stop reason
- `/steering/status`: 목표·실제 조향, state, timeout과 fault
- `/diagnostics`: 상태 요약과 source별 상세

Foxglove 기본 포트는 8765, Vehicle Console은 8766, HIL Console은 8767이다.
세 기능은 서로 다른 기본 포트를 사용한다.

```bash
colcon build --symlink-install --packages-select foxglove
```

기존 Qt dashboard는
[`archive/software/legacy_vehicle_dashboard`](../../../archive/software/legacy_vehicle_dashboard)에
보관한다. Console 기능 대응과 검증 기록은
[`vehicle_console/CONSOLE_PARITY.md`](../vehicle_console/CONSOLE_PARITY.md)와
[`vehicle_console/CONSOLE_VALIDATION.md`](../vehicle_console/CONSOLE_VALIDATION.md)에 있다.
