"""Test-only publisher standing in for Planning on /planning/command.

The command chain is /planning/command -> command_governor -> /vehicle/command.
This node fills the Planning slot on the bench; the governor remains the only
/vehicle/command publisher.
"""

from __future__ import annotations

import math

from kaiev26_msgs.msg import ActuatorCommand
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class HilCommandGenerator(Node):
    def __init__(self) -> None:
        super().__init__("hil_command_generator")
        self.declare_parameter("enabled", False)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("speed_target_mps", 0.0)
        self.declare_parameter("steering_target_rad", 0.0)
        self.declare_parameter("brake_engage", False)
        self.declare_parameter("command_topic", "/planning/command")
        self._publisher = self.create_publisher(
            ActuatorCommand, str(self.get_parameter("command_topic").value), 10
        )
        self._callback = self.add_on_set_parameters_callback(self._validate_parameters)
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._publish)
        self.get_logger().warning(
            "HIL TEST SOURCE: this node stands in for Planning on /planning/command. "
            "Ensure no real Planning publisher is active at the same time. "
            "/vehicle/command is owned by command_governor."
        )

    @staticmethod
    def _validate_parameters(parameters):
        result = SetParametersResult(successful=True)
        for parameter in parameters:
            if parameter.name in {"speed_target_mps", "steering_target_rad"}:
                if not math.isfinite(float(parameter.value)):
                    result.successful = False
                    result.reason = f"{parameter.name} must be finite"
                    return result
            elif parameter.name == "publish_rate_hz" and float(parameter.value) <= 0.0:
                result.successful = False
                result.reason = "publish_rate_hz must be positive (restart required to change timer rate)"
                return result
        return result

    def _publish(self) -> None:
        if not bool(self.get_parameter("enabled").value):
            return
        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.speed_target_mps = float(self.get_parameter("speed_target_mps").value)
        message.steering_target_rad = float(self.get_parameter("steering_target_rad").value)
        message.brake_engage = bool(self.get_parameter("brake_engage").value)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HilCommandGenerator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
