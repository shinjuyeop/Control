"""Run the existing waypoint planner and controller on SYSID-private topics."""

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from kaiev26_decision.test_tracking import ConstantRoutePlanNode
from kaiev26_decision.route_zone_manager_node import RouteZoneManagerNode
from kaiev26_motion_control.motion_control_node import MotionControlNode


class RouteInput(Node):
    def __init__(self, planner, controller):
        super().__init__('route_input')
        self.planner = planner
        self.controller = controller
        self.target_speed = planner.constant_speed_mps
        planner.constant_speed_mps = 0.0
        self.create_subscription(Bool, '/sysid/route_enabled', self.on_enabled, 10)

    def on_enabled(self, message):
        self.planner.constant_speed_mps = self.target_speed if message.data else 0.0
        if not message.data:
            self.controller.commanded_speed_mps = 0.0


def main():
    rclpy.init()
    executor = SingleThreadedExecutor()
    nodes = []
    try:
        nodes.append(RouteZoneManagerNode())
        nodes.append(ConstantRoutePlanNode())
        nodes.append(MotionControlNode())
        nodes.append(RouteInput(nodes[1], nodes[2]))
        for node in nodes:
            executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        for node in reversed(nodes):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
