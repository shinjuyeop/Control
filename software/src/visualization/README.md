# visualization

차량 상태를 `/diagnostics`로 요약하고 Foxglove의 상세 분석 연결을 제공한다.
운영자용 브라우저 관측은 [KAI Vehicle Console](../vehicle_console/README.md)이
담당한다. Console은 빠른 상태 확인과 원격/mobile monitoring, Foxglove는 plot,
raw messages, state transition과 상세 진단을 담당한다.

이 패키지는 command generator, governor 또는 액추에이터 제어기가 아니다.
기존 Qt dashboard는 [legacy archive](../../../archive/software/legacy_vehicle_dashboard)에
source/tests/launch/docs와 함께 보관한다. [parity matrix](../../../docs/CONSOLE_PARITY.md)에
Web 대체 범위가 있다.

## Foxglove

Foxglove는 custom message 내부 필드를 직접 표시할 수 있으므로 필드마다
별도 scalar 토픽을 만들지 않는다. Plot이나 Raw Messages 패널에서 다음과
같은 message path를 바로 선택한다.

- `/vehicle/command.speed_target_mps`
- `/vehicle/command.steering_target_rad`
- `/vehicle/command.brake_engage`
- `/drive/status.speed_mps`
- `/drive/status.motor_rpm`
- `/drive/status.bus_voltage_v`
- `/drive/status.bus_current_a`
- `/drive/status.phase_current_a`
- `/brake/status.brake_commanded`
- `/brake/status.stop_reason_bits`
- `/steering/status.target_wheel_angle_rad`
- `/steering/status.actual_wheel_angle_rad`
- `/steering/status.state`
- `/steering/status.fault_bits`

추천 확인 항목은 target/actual speed, target/actual steering, bus
voltage/current, motor temperature, brake reason, steering state, message
timeout과 age다.

bridge는 `foxglove_bridge.launch.py`로 띄우며 **PC당 하나만** 실행한다 —
하나가 ROS graph 전체를 서빙하고 클라이언트 여러 대가 동시 접속한다. launch가
기본값과 다른 `send_buffer_limit`(100 MB)을 고정하므로 `ros2 run`으로 직접
띄우지 않는다.

## diagnostics_publisher

status 3종(`/drive/status`, `/brake/status`, `/steering/status`)을 판정해
`/diagnostics`(`diagnostic_msgs/DiagnosticArray`)로 1 Hz 요약하는 headless
노드다. Foxglove Diagnostics 패널이 등급순으로 정렬하므로 문제 항목만 위로
올라온다.

실차에서는 관찰이 아니라 구동 스택 소속이라
`control_launch/vehicle.launch.py`가 띄운다 — Foxglove를 켜지 않아도
발행되어 bag에 남는다. 단독 실행은 `ros2 run visualization
diagnostics_publisher`. 구독하는 status가 없으면 STALE로 표시되므로, 조향
없는 최소 HIL bench에는 포함하지 않는다.

## Build / run

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select visualization
source install/setup.bash
ros2 launch visualization foxglove_bridge.launch.py
```

Foxglove 기본 포트는 8765, Vehicle Console은 8766이므로 동시에 실행할 수 있다.
HIL Web은 기존 8765를 쓰므로 HIL과 함께 실행할 때는 Foxglove `port:=8767`을 준다.
`control.json`은 기존 engineering layout이며 패키지 share에도 설치된다.
Qt/PyQt5/control_gui_common은 active visualization의 dependency가 아니다.
