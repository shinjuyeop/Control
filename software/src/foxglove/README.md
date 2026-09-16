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
`foxglove_bridge.launch.py`는 diagnostics publisher를 시작하지 않는다. 실차에서는
`control_launch/vehicle.launch.py`가 함께 시작한 publisher를 사용하고, bridge만
단독 실행했다면 필요할 때 다음 노드를 별도로 실행한다.

```bash
ros2 run vehicle_state_publisher diagnostics_publisher
```

추천 항목은 다음과 같다.

- `/vehicle/command`: speed, steering, brake 목표
- `/drive/status`: 속도, RPM, 전압·전류·온도, timeout과 fault
- `/brake/status`: brake command와 stop reason
- `/steering/status`: 목표·실제 조향, state, timeout과 fault
- `/diagnostics`: 구동 링크·구동 상태·조향 링크·조향 상태·브레이크 요약과
  source별 상세. sensor/bridge 등 다른 publisher의 진단도 같은 토픽에 공존한다.

[`control.json`](control.json)은 다음 여섯 탭을 제공한다. 모든 탭 아래에는
기어·모드·E-stop·브레이크·페달 공통 상태 바가 계속 보인다.

- **공통**: 속도와 명령 age, ROS 파라미터
- **Control**: 속도·조향 추종과 전원·상태·고장 상세
- **조향 Torque**: 요청·soft-limit 목표·actual·error, P/D·unsaturated·KEYA raw,
  filtered/unfiltered wheel rate, current/speed, saturation·peak·settle, mode/safety.
  단위·튜닝 순서는 [TORQUE_CONTROL.md](../lateral_ctrl/TORQUE_CONTROL.md)를 따른다.
- **종방향 테스트**: `/planning/command`부터 `/vehicle/command`, 실제 속도까지의
  추종 관계와 모터 RPM, 상전류·버스 전류, 통신 age, 안전 이벤트
- **조향 테스트**: 시험 입력부터 변화율 제한·PID 보정·실측 조향각까지의 명령 체인과
  추종 오차, 차량 속도, 모터 전류, 통신 age, 상태·timeout 이벤트
- **Diagnostics**: `/diagnostics` 요약과 선택 항목 상세

시험 탭의 그래프와 상태 전이는 receive/log time으로 동기화되어 MCAP 재생 커서를
함께 따른다. `/sysid/scinario`는 현재 시험 label을 표시하며, 이 철자는 현재 시험
runner의 토픽 계약을 그대로 사용한다. 이 레이아웃은 토픽을 관측할 뿐 차량 제어 또는
safety 판단에 사용하지 않는다.

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
