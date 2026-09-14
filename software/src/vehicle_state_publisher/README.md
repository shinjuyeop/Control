# vehicle_state_publisher

Canonical drive, brake and steering status를 차량 단위 상태와 진단으로 변환하는
headless ROS 2 package다. 각 기능은 별도 노드로 유지한다.

| Node | Input | Output | Rate |
|---|---|---|---:|
| `vehicle_state_publisher_node` | `/drive/status`, `/brake/status`, `/steering/status` | `/vehicle/state`, `/vehicle/wheel_twist` | 20 Hz |
| `diagnostics_publisher` | `/drive/status`, `/brake/status`, `/steering/status` | `/diagnostics` | 1 Hz |
| `auto_dashboard_summary` | 차량 status, GNSS·LiDAR·camera, localization·planning | `/dashboard/auto_status` | 10 Hz |

`vehicle_state_publisher_node`는 세 status가 모두 한 번 이상 들어온 뒤
`/vehicle/state`를 발행한다. 유효한 drive Part I과 steering feedback이 있을 때만
Ackermann 근사 `/vehicle/wheel_twist`를 발행한다.

`diagnostics_publisher`는 각 status의 수신 단절과 메시지가 보고한 timeout, valid,
fault, brake reason을 `diagnostic_msgs/msg/DiagnosticArray`로 요약한다. 진단 등급은
관측과 표시를 위한 값이며 MCU 또는 조향 제어기의 차단 조건을 실행하지 않는다.
판정 기준은 [SAFETY.md](../../../docs/SAFETY.md), topic 계약은
[INTERFACES.md](../../../docs/INTERFACES.md)를 따른다.

`auto_dashboard_summary`는 STM_B가 UNO 계기판으로 중계하는 표시용 25-byte payload를
발행한다. 상태 합성이나 제어 노드의 입력으로 사용하지 않는다. 표시값과 freshness는
[UNO README](../../../firmware/UNO_R4_DASHBOARD/README.md#auto-상태-표시의-의미와-소스)를 따른다.

실차에서는 `control_launch/vehicle.launch.py`가 세 노드를 모두 실행한다.
진단만 별도로 확인할 때:

```bash
ros2 run vehicle_state_publisher diagnostics_publisher
```

```bash
colcon build --symlink-install --packages-select vehicle_state_publisher
colcon test --packages-select vehicle_state_publisher
```
