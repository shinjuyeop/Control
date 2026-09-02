#!/usr/bin/env python3
"""서브시스템 status 3종을 합성해 통합 차량 상태 /vehicle/state 를 발행한다.

/vehicle/state 의 필드는 세 프로세서(STM_A 구동, STM_B 브레이크, 조향 노드)에
나뉘어 있으므로 차량 측에서 한 번 합성이 필요하다. legacy 토픽은 사용하지 않고
canonical status만 구독한다.
"""
from __future__ import annotations

import math

from geometry_msgs.msg import TwistWithCovarianceStamped
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


def make_wheel_twist(
    steering: SteeringActuatorStatus,
    drive: DriveActuatorStatus,
    stamp,
    *,
    frame_id: str,
    wheelbase_m: float,
    speed_scale: float,
    steering_scale: float,
    steering_offset_rad: float,
    yaw_rate_scale: float,
    longitudinal_velocity_variance: float,
    lateral_velocity_variance: float,
    yaw_rate_variance: float,
) -> TwistWithCovarianceStamped | None:
    if not drive.part1_valid or not steering.feedback_valid:
        return None

    speed_mps = float(drive.speed_mps) * speed_scale
    steering_rad = (
        float(steering.actual_wheel_angle_rad) * steering_scale
        + steering_offset_rad
    )
    if not math.isfinite(speed_mps) or not math.isfinite(steering_rad):
        return None

    msg = TwistWithCovarianceStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.twist.twist.linear.x = speed_mps
    msg.twist.twist.angular.z = (
        yaw_rate_scale * speed_mps * math.tan(steering_rad) / wheelbase_m
    )
    msg.twist.covariance[0] = longitudinal_velocity_variance
    msg.twist.covariance[7] = lateral_velocity_variance
    msg.twist.covariance[35] = yaw_rate_variance
    return msg


class VehicleStatePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("vehicle_state_publisher")
        self.declare_parameter("steering_status_topic", "/steering/status")
        self.declare_parameter("drive_status_topic", "/drive/status")
        self.declare_parameter("brake_status_topic", "/brake/status")
        self.declare_parameter("state_topic", "/vehicle/state")
        self.declare_parameter("wheel_twist_topic", "/vehicle/wheel_twist")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("wheelbase_m", 1.2991017929)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("steering_scale", 1.0)
        self.declare_parameter("steering_offset_rad", 0.0)
        self.declare_parameter("yaw_rate_scale", 1.0)
        self.declare_parameter("longitudinal_velocity_variance", 0.016)
        self.declare_parameter("lateral_velocity_variance", 0.25)
        self.declare_parameter("yaw_rate_variance", 0.0016)

        self.wheelbase_m = float(self.get_parameter("wheelbase_m").value)
        self.speed_scale = float(self.get_parameter("speed_scale").value)
        self.steering_scale = float(self.get_parameter("steering_scale").value)
        self.steering_offset_rad = float(
            self.get_parameter("steering_offset_rad").value
        )
        self.yaw_rate_scale = float(self.get_parameter("yaw_rate_scale").value)
        self.longitudinal_velocity_variance = float(
            self.get_parameter("longitudinal_velocity_variance").value
        )
        self.lateral_velocity_variance = float(
            self.get_parameter("lateral_velocity_variance").value
        )
        self.yaw_rate_variance = float(
            self.get_parameter("yaw_rate_variance").value
        )

        self.latest_steering: SteeringActuatorStatus | None = None
        self.latest_drive: DriveActuatorStatus | None = None
        self.latest_brake: BrakeActuatorStatus | None = None

        self.state_pub = self.create_publisher(
            VehicleState, str(self.get_parameter("state_topic").value), 10
        )
        self.wheel_twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            str(self.get_parameter("wheel_twist_topic").value),
            10,
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
        stamp = self.get_clock().now().to_msg()
        if self.latest_steering is not None and self.latest_drive is not None:
            wheel_twist = make_wheel_twist(
                self.latest_steering,
                self.latest_drive,
                stamp,
                frame_id="base_footprint",
                wheelbase_m=self.wheelbase_m,
                speed_scale=self.speed_scale,
                steering_scale=self.steering_scale,
                steering_offset_rad=self.steering_offset_rad,
                yaw_rate_scale=self.yaw_rate_scale,
                longitudinal_velocity_variance=(
                    self.longitudinal_velocity_variance
                ),
                lateral_velocity_variance=self.lateral_velocity_variance,
                yaw_rate_variance=self.yaw_rate_variance,
            )
            if wheel_twist is not None:
                self.wheel_twist_pub.publish(wheel_twist)

        # /vehicle/state는 세 status가 모두 최소 한 번 수신된 뒤에만 발행한다.
        if self.latest_steering is None or self.latest_drive is None or self.latest_brake is None:
            return
        state = fuse_vehicle_state(
            self.latest_steering, self.latest_drive, self.latest_brake
        )
        state.header.stamp = stamp
        state.header.frame_id = "base_footprint"
        self.state_pub.publish(state)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VehicleStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
