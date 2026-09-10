"""Canonical status -> DiagnosticArray remains usable without either Qt GUI."""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from kaiev26_msgs.msg import BrakeActuatorStatus, DriveActuatorStatus, SteeringActuatorStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from visualization.diagnostics_publisher import DiagnosticsPublisher


def test_canonical_status_to_diagnostics_without_command_publishers():
    rclpy.init()
    node = DiagnosticsPublisher()
    peer = Node("diagnostics_observer_test")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(peer)
    received = []
    peer.create_subscription(DiagnosticArray, "/diagnostics", received.append, 10)
    drive = peer.create_publisher(DriveActuatorStatus, "/drive/status", 10)
    brake = peer.create_publisher(BrakeActuatorStatus, "/brake/status", 10)
    steering = peer.create_publisher(SteeringActuatorStatus, "/steering/status", 10)
    try:
        assert {p.topic_name for p in node.publishers} <= {"/diagnostics", "/parameter_events"}
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and not received:
            executor.spin_once(timeout_sec=0.05)
        assert received and all(s.level == DiagnosticStatus.STALE for s in received[-1].status)
        received.clear()
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            drive.publish(DriveActuatorStatus(part1_valid=True, part2_valid=True,
                                             tractive_feedback_valid=True, fault_bits_valid=True))
            brake.publish(BrakeActuatorStatus(brake_commanded=True, stop_reason_bits=4))
            steering.publish(SteeringActuatorStatus(state=3, communication_ok=True, feedback_valid=True))
            executor.spin_once(timeout_sec=0.02)
            if received and any(s.name == "브레이크" and s.level == DiagnosticStatus.ERROR for s in received[-1].status):
                break
        statuses = {s.name: s for s in received[-1].status}
        assert statuses["구동 링크"].level == DiagnosticStatus.OK
        assert statuses["브레이크"].level == DiagnosticStatus.ERROR
        assert "ESTOP" in statuses["브레이크"].message
    finally:
        executor.remove_node(peer)
        executor.remove_node(node)
        peer.destroy_node()
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
