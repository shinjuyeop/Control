#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class AebPublisher(Node):
    def __init__(self):
        super().__init__('aeb_pub')
        self.declare_parameter('topic', '/aeb')
        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Int32, topic, 10)

        self.get_logger().info(
            f"AEB publisher ready. Publishing to '{topic}'.\n"
            "Enter 0 or 1 then Enter. 'q' to quit."
        )

    def prompt_and_publish(self):
        while rclpy.ok():
            try:
                line = input('aeb (0/1, q=quit)> ').strip()
            except (EOFError, KeyboardInterrupt):
                print()
                break
            if line.lower() in ('q', 'quit', 'exit'):
                break
            if line in ('0', '1'):
                val = int(line)
                self.pub.publish(Int32(data=val))
                self.get_logger().info(f"Published AEB: {val}")
            else:
                self.get_logger().warn("Please enter 0, 1, or q")


def main(args=None):
    rclpy.init(args=args)
    node = AebPublisher()
    try:
        node.prompt_and_publish()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

