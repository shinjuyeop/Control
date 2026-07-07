#!/usr/bin/env python3
"""
Test setpoint publisher for pid_gazebo.

Publishes:
  /setpoint/rpm        (std_msgs/Float64)  – longitudinal target RPM
  /setpoint/steer_deg  (std_msgs/Float64)  – heading target [degrees]

Parameters (ros2 run or launch):
  rpm       : target RPM (default 300.0)
  steer_deg : target heading degrees (default 0.0)
  hz        : publish rate (default 10.0)
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SetpointPublisher(Node):

    def __init__(self) -> None:
        super().__init__('setpoint_publisher')

        self.declare_parameter('rpm', 10.0)
        self.declare_parameter('steer_deg', 0.0)
        self.declare_parameter('hz', 10.0)

        self._rpm_pub = self.create_publisher(Float64, '/setpoint/rpm', 10)
        self._str_pub = self.create_publisher(Float64, '/setpoint/steer_deg', 10)

        hz = self.get_parameter('hz').value
        self.create_timer(1.0 / max(hz, 1.0), self._publish)

        self.get_logger().info(
            f'setpoint_publisher: rpm={self.get_parameter("rpm").value} '
            f'steer={self.get_parameter("steer_deg").value}° hz={hz}'
        )

    def _publish(self) -> None:
        rpm_msg = Float64()
        rpm_msg.data = float(self.get_parameter('rpm').value)
        self._rpm_pub.publish(rpm_msg)

        str_msg = Float64()
        str_msg.data = float(self.get_parameter('steer_deg').value)
        self._str_pub.publish(str_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SetpointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
