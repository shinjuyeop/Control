#!/usr/bin/env python3
"""판단이 낸 원하는 운동을 실차 actuator 명령으로 바꿔 내보내는 노드.

분할 전 `kaiev26_control`의 control command gate에서 제어 소유의 절반이다.
경로 추종과 속도 계획은 판단 저장소의 `kaiev26_motion_control`이 갖는다.

이 노드가 갖는 것은 셋이다.

- `/vehicle/actuator_command`의 단독 발행자 자격
- E-stop과 AEB에 의한 정지 override
- 입력 단절 시의 처신

입력 단절은 **발행 중단**으로 처리한다. 오래된 운동 명령에 새 stamp를 붙여
내보내면 하위 계층이 그것을 신선한 명령으로 읽는다. 분할 전에도 게이트 노드가
죽으면 발행이 멈췄으므로 이것은 새로 만든 정지 게이트가 아니라 같은 실패
양상의 보존이다.

주의: 실차 STM_A는 `fw_current`에서 command deadman이 없다. 발행이 멈춰도
마지막 목표를 계속 보낸다. 그래서 planning stale처럼 **예상되는** 단절에는
`kaiev26_motion_control`이 명시적 정지 명령을 내고 이 노드는 그것을 그대로
통과시킨다. 발행 중단은 상위 노드가 사라진 **예상 밖** 경우에만 해당한다.
"""
from __future__ import annotations

from kaiev26_msgs.msg import ActuatorCommand, AEBState, VehicleState
import rclpy
from rclpy.node import Node
from rclpy.time import Time as RosTime
from std_msgs.msg import String


INF_AGE_SEC = 1.0e6


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def stamp_age_sec(now, stamp) -> float:
    if stamp.sec == 0 and stamp.nanosec == 0:
        return INF_AGE_SEC
    age = now.nanoseconds - RosTime.from_msg(stamp).nanoseconds
    return max(0.0, age * 1.0e-9)


class ActuatorCommandBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("kaiev26_actuator_command_bridge")
        self.declare_parameter("motion_command_topic", "/control/motion_command")
        self.declare_parameter("vehicle_state_topic", "/vehicle/state")
        self.declare_parameter("aeb_state_topic", "/safety/aeb_state")
        self.declare_parameter("command_topic", "/vehicle/actuator_command")
        self.declare_parameter("debug_topic", "/debug/control_gate")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("motion_stale_sec", 0.35)
        self.declare_parameter("max_steering_rad", 0.4363)

        self.motion_stale_sec = float(self.get_parameter("motion_stale_sec").value)
        self.max_steering_rad = float(self.get_parameter("max_steering_rad").value)
        self.publish_rate_hz = max(
            1.0, float(self.get_parameter("publish_rate_hz").value)
        )

        self.latest_motion: ActuatorCommand | None = None
        self.latest_vehicle_state: VehicleState | None = None
        self.latest_aeb_state: AEBState | None = None

        # TODO(Control safety contract): vehicle_state와 aeb_state는 현재 수신 시각과
        # 상위 source health를 검증하지 않고 마지막 값을 계속 사용한다. AEB의
        # active=false heartbeat가 끊기거나 vehicle_state_publisher가 단절된 actuator
        # status를 새 stamp로 재발행하면 통신 단절을 정상 상태로 오인할 수 있다.
        # 실차 적용 전에 제어팀이 freshness 기준, fault latch와 reset 절차, fault 시
        # steering과 brake 동작을 확정하고 이 입력 경계에 반영해야 한다.
        self.command_pub = self.create_publisher(
            ActuatorCommand,
            str(self.get_parameter("command_topic").value),
            10,
        )
        self.debug_pub = self.create_publisher(
            String,
            str(self.get_parameter("debug_topic").value),
            10,
        )
        self.create_subscription(
            ActuatorCommand,
            str(self.get_parameter("motion_command_topic").value),
            self.on_motion_command,
            10,
        )
        self.create_subscription(
            VehicleState,
            str(self.get_parameter("vehicle_state_topic").value),
            self.on_vehicle_state,
            10,
        )
        self.create_subscription(
            AEBState,
            str(self.get_parameter("aeb_state_topic").value),
            self.on_aeb_state,
            10,
        )
        self.create_timer(1.0 / self.publish_rate_hz, self.on_publish_timer)

    def on_motion_command(self, msg: ActuatorCommand) -> None:
        self.latest_motion = msg

    def on_vehicle_state(self, msg: VehicleState) -> None:
        self.latest_vehicle_state = msg

    def on_aeb_state(self, msg: AEBState) -> None:
        self.latest_aeb_state = msg

    @property
    def estop_active(self) -> bool:
        # E-stop 상태는 통합 차량 상태에서 읽는다 (별도 legacy 토픽 없음)
        return (
            self.latest_vehicle_state is not None
            and bool(self.latest_vehicle_state.estop_active)
        )

    def motion_is_stale(self, now) -> bool:
        if self.latest_motion is None:
            return True
        age = stamp_age_sec(now, self.latest_motion.header.stamp)
        return age > self.motion_stale_sec

    def on_publish_timer(self) -> None:
        now = self.get_clock().now()

        if self.motion_is_stale(now):
            # 오래된 운동 명령에 새 stamp를 붙여 세탁하지 않는다. 분할 전
            # 게이트 노드가 죽었을 때와 같은 결과다.
            self.publish_debug(None, "motion stale")
            return

        command, reason = self.compose_command(now)
        self.command_pub.publish(command)
        self.publish_debug(command, reason)

    def compose_command(self, now) -> tuple[ActuatorCommand, str]:
        assert self.latest_motion is not None
        command = ActuatorCommand()
        command.header.stamp = now.to_msg()
        command.gear = ActuatorCommand.GEAR_FORWARD

        if self.estop_active:
            reason = "estop"
            self.apply_stop_command(command, hold_steering=False)
        elif self.latest_aeb_state is not None and self.latest_aeb_state.active:
            reason = f"aeb:{self.latest_aeb_state.state}"
            self.apply_stop_command(command, hold_steering=True)
        else:
            reason = "normal"
            command.enable = bool(self.latest_motion.enable)
            command.steering_target_rad = float(self.latest_motion.steering_target_rad)
            command.speed_target_mps = float(self.latest_motion.speed_target_mps)
            command.brake_engage = bool(self.latest_motion.brake_engage)
            command.gear = self.latest_motion.gear

        return command, reason

    def apply_stop_command(self, command: ActuatorCommand, hold_steering: bool) -> None:
        if hold_steering and self.latest_vehicle_state is not None:
            steering = clamp(
                float(self.latest_vehicle_state.steering_rad),
                -self.max_steering_rad,
                self.max_steering_rad,
            )
        else:
            steering = 0.0

        command.enable = True
        command.steering_target_rad = float(steering)
        command.speed_target_mps = 0.0
        command.brake_engage = True
        command.gear = ActuatorCommand.GEAR_FORWARD

    def publish_debug(self, command: ActuatorCommand | None, reason: str) -> None:
        msg = String()
        if command is None:
            msg.data = f"reason={reason}"
        else:
            msg.data = (
                f"reason={reason} speed={command.speed_target_mps:.2f} "
                f"steer={command.steering_target_rad:.3f} brake={command.brake_engage}"
            )
        self.debug_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ActuatorCommandBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
