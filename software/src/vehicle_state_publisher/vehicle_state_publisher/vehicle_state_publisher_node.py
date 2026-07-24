#!/usr/bin/env python3
"""서브시스템 status 3종을 합성해 통합 차량 상태 /vehicle/state 를 발행한다.

/vehicle/state 의 필드는 세 프로세서(STM_A 구동, STM_B 브레이크, 조향 노드)에
나뉘어 있으므로 차량 측에서 한 번 합성이 필요하다. legacy 토픽은 사용하지 않고
canonical status만 구독한다.
"""
from __future__ import annotations

from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
    VehicleState,
)
import rclpy
from rclpy.node import Node


def fuse_vehicle_state(
    steering: SteeringActuatorStatus,
    drive: DriveActuatorStatus,
    brake: BrakeActuatorStatus,
) -> VehicleState:
    state = VehicleState()
    state.speed_mps = float(drive.speed_mps)
    state.steering_rad = float(steering.actual_wheel_angle_rad)
    state.gear = int(drive.gear)
    state.mode = int(drive.mode)
    state.estop_active = bool(
        brake.stop_reason_bits & BrakeActuatorStatus.STOP_REASON_ESTOP
    )
    return state


class VehicleStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_state_publisher")
        self.declare_parameter("steering_status_topic", "/steering/status")
        self.declare_parameter("drive_status_topic", "/drive/status")
        self.declare_parameter("brake_status_topic", "/brake/status")
        self.declare_parameter("state_topic", "/vehicle/state")
        self.declare_parameter("publish_rate_hz", 20.0)

        self.latest_steering: SteeringActuatorStatus | None = None
        self.latest_drive: DriveActuatorStatus | None = None
        self.latest_brake: BrakeActuatorStatus | None = None

        self.state_pub = self.create_publisher(
            VehicleState, str(self.get_parameter("state_topic").value), 10
        )
        self.create_subscription(
            SteeringActuatorStatus,
            str(self.get_parameter("steering_status_topic").value),
            self.on_steering,
            10,
        )
        self.create_subscription(
            DriveActuatorStatus,
            str(self.get_parameter("drive_status_topic").value),
            self.on_drive,
            10,
        )
        self.create_subscription(
            BrakeActuatorStatus,
            str(self.get_parameter("brake_status_topic").value),
            self.on_brake,
            10,
        )
        rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self.on_timer)

    def on_steering(self, msg: SteeringActuatorStatus) -> None:
        self.latest_steering = msg

    def on_drive(self, msg: DriveActuatorStatus) -> None:
        self.latest_drive = msg

    def on_brake(self, msg: BrakeActuatorStatus) -> None:
        self.latest_brake = msg

    def on_timer(self) -> None:
        # 세 status가 모두 최소 한 번 수신된 뒤에만 상태를 발행한다.
        # 없는 값을 지어내지 않는다.
        if (
            self.latest_steering is None
            or self.latest_drive is None
            or self.latest_brake is None
        ):
            return
        state = fuse_vehicle_state(
            self.latest_steering, self.latest_drive, self.latest_brake
        )
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = "base_footprint"
        self.state_pub.publish(state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VehicleStatePublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
