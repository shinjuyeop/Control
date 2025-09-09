#!/usr/bin/env python3
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class AebPublisher(Node):
    def __init__(self):
        super().__init__('aeb_pub')
        self.declare_parameter('topic', '/aeb')
        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Int32, topic, 10)

        self.last_value = 0  # 기본 값
        self.timer = self.create_timer(0.5, self._timer_publish)  # 0.5초마다 반복 발행

        self.get_logger().info(
            f"AEB publisher ready. Publishing to '{topic}'.\n"
            "Enter 0 or 1 then Enter. 'q' to quit.\n"
            "현재 값은 0으로 주기 발행 중."
        )

    def _timer_publish(self):
        self.pub.publish(Int32(data=self.last_value))

    def input_loop(self):
        while rclpy.ok():
            try:
                line = input('aeb (0/1, q=quit)> ').strip()
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info("Input ended.")
                rclpy.shutdown()
                break
            if line.lower() in ('q', 'quit', 'exit'):
                self.get_logger().info("Shutdown requested.")
                rclpy.shutdown()
                break
            if line in ('0', '1'):
                self.last_value = int(line)
                self.get_logger().info(f"Set AEB to {self.last_value} (publishing every 0.5s)")
            else:
                self.get_logger().warn("Please enter 0, 1, or q")


def main(args=None):
    rclpy.init(args=args)
    node = AebPublisher()

    t = threading.Thread(target=node.input_loop, daemon=True)
    t.start()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

