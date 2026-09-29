"""Canonical status -> DiagnosticArray remains usable without either Qt GUI."""

import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from kaiev26_msgs.msg import BrakeActuatorStatus, DriveActuatorStatus, SteeringActuatorStatus
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from vehicle_state_publisher.diagnostics_publisher import DiagnosticsPublisher


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


def test_diagnostic_identity_is_stable_across_waiting_live_stale_live():
    rclpy.init()
    node = DiagnosticsPublisher()
    now = [100.0]
    node._now_sec = lambda: now[0]
    getters = (
        node._drive_link,
        node._drive_state,
        node._steer_link,
        node._steer_state,
        node._brake_group,
    )
    expected = (
        ("STM_A / EZkontrol", "구동 링크"),
        ("STM_A / EZkontrol", "구동 상태"),
        ("KEYA / can0", "조향 링크"),
        ("KEYA", "조향 상태"),
        ("STM_B", "브레이크"),
    )

    def identities():
        statuses = (getter() for getter in getters)
        return tuple((status.hardware_id, status.name) for status in statuses)

    try:
        assert identities() == expected
        node._on_drive(DriveActuatorStatus(
            part1_valid=True, part2_valid=True,
            tractive_feedback_valid=True, fault_bits_valid=True,
        ))
        node._on_steering(SteeringActuatorStatus(
            state=SteeringActuatorStatus.STATE_ACTIVE,
            communication_ok=True, feedback_valid=True,
        ))
        node._on_brake(BrakeActuatorStatus())
        assert identities() == expected
        now[0] += 1.01
        assert identities() == expected
        node._on_drive(DriveActuatorStatus())
        node._on_steering(SteeringActuatorStatus())
        node._on_brake(BrakeActuatorStatus())
        assert identities() == expected
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_ezkontrol_feedback_stays_normal_until_invalid():
    rclpy.init()
    node = DiagnosticsPublisher()
    try:
        node._on_drive(DriveActuatorStatus(
            command_age_ms=0.0,
            part1_age_ms=300.0, part1_valid=True,
            part2_age_ms=800.0, part2_valid=True,
        ))
        status = node._drive_link()
        assert status.level == DiagnosticStatus.OK

        node._on_drive(DriveActuatorStatus(
            command_age_ms=0.0,
            part1_age_ms=500.0, part1_valid=False,
            part2_age_ms=800.0, part2_valid=True,
        ))
        assert node._drive_link().level == DiagnosticStatus.ERROR

        node._on_drive(DriveActuatorStatus(
            command_age_ms=0.0,
            part1_age_ms=25.0, part1_valid=True,
            part2_age_ms=1000.0, part2_valid=False,
        ))
        assert node._drive_link().level == DiagnosticStatus.ERROR
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_manual_without_commands_is_normal_and_recovery_is_distinct():
    rclpy.init()
    node = DiagnosticsPublisher()
    try:
        node._on_steering(SteeringActuatorStatus(
            state=1, disable_reason=3, communication_ok=True, feedback_valid=True,
            steer_command_age_ms=float('inf')))
        assert node._steer_link().level == DiagnosticStatus.OK
        assert node._steer_state().level == DiagnosticStatus.OK
        assert 'MANUAL' in node._steer_state().message
        node._steering.state = 2
        node._steering.disable_reason = 10
        assert 'RECOVERING' in node._steer_state().message
        node._steering.state = 8
        assert node._steer_state().level == DiagnosticStatus.ERROR
        assert 'HARD FAULT' in node._steer_state().message
    finally:
        node.destroy_node()
        rclpy.shutdown()
