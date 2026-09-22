# foxglove

Foxglove bridge와 KAI 차량 engineering layout만 제공한다. 차량 상태 요약
`/diagnostics`는 `vehicle_state_publisher/diagnostics_publisher`가 발행하며,
브라우저 기반 운영 화면은 `vehicle_console`이 담당한다.

```bash
ros2 launch foxglove foxglove_bridge.launch.py
```

Foxglove 앱에서 로컬은 `ws://127.0.0.1:8765`, 원격은 차량 PC 주소로 연결하고
(기존 차량 PC Tailscale 예: `ws://100.112.72.27:8765`)
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
  명령 전류 제한·KEYA 전류 진단, filtered/unfiltered wheel rate, speed,
  saturation·peak·settle, mode/safety.
  단위·튜닝 순서는 [TORQUE_CONTROL.md](../lateral_ctrl/TORQUE_CONTROL.md)를 따른다.
- **종방향 테스트**: `/planning/command`부터 `/vehicle/command`, 실제 속도까지의
  추종 관계와 모터 RPM, 상전류·버스 전류, 통신 age, 안전 이벤트
- **조향 테스트**: 시험 입력·목표·실측 조향각의 명령 체인과
  추종 오차, 차량 속도, 모터 전류, 통신 age, 상태·timeout 이벤트
- **Diagnostics**: `/diagnostics` 요약과 선택 항목 상세

시험 탭의 그래프와 상태 전이는 receive/log time으로 동기화되어 MCAP 재생 커서를
함께 따른다. `/sysid/scenario`는 현재 시험 label을 표시하며, 토픽 이름은 현재 시험
runner의 토픽 계약을 그대로 사용한다. 이 레이아웃은 토픽을 관측할 뿐 차량 제어 또는
safety 판단에 사용하지 않는다.

현재 실차 YAML은 torque 모드다. position의 목표각 rate limit은 torque 모드에
적용되지 않는다. torque의 실제각 속도는 D항과 settle 판정에 사용하고 속도 자체로
FAULT를 만들지 않는다. feedback stop과 자동 복구는
[조향 운용 문서](../lateral_ctrl/TORQUE_CONTROL.md#안전-동작)를 따른다.
시나리오 기본 목표속도·중심각은 0이므로 시험 설정 전 그래프가 0인 것은 정상이다.

Foxglove 기본 포트는 8765, Vehicle Console은 8766, HIL Console은 8767이다.
세 기능은 서로 다른 기본 포트를 사용한다.

```bash
colcon build --symlink-install --packages-select foxglove
```

기존 Qt dashboard는 Vehicle Console로 대체되어 보관본을 정리했다. Console 기능 대응과 검증 기록은
[`vehicle_console/CONSOLE_PARITY.md`](../vehicle_console/CONSOLE_PARITY.md)와
[`vehicle_console/CONSOLE_VALIDATION.md`](../vehicle_console/CONSOLE_VALIDATION.md)에 있다.
