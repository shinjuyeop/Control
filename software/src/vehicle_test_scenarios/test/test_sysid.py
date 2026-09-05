"""Field input, mode transition, and real ROS-to-MCAP tests."""

import time
from pathlib import Path
from unittest.mock import Mock

import pytest
import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped
from kaiev26_msgs.msg import ActuatorCommand

from vehicle_test_scenarios.sysid_profiles import (
    CENTER_DEG, RunSettings, baseline, evaluate, segments,
)
from vehicle_test_scenarios.sysid_runtime import SysidNode, TOPICS, FOLLOW_TOPICS
from vehicle_test_scenarios import sysid_processes


@pytest.mark.parametrize('scenario', [1, 2, 3, 4])
@pytest.mark.parametrize('speed', [5.0, 15.0, 36.0])
def test_profiles_respect_selected_target_limit_and_finish_at_zero(scenario, speed):
    settings = RunSettings(scenario, speed)
    schedule = segments(settings)
    assert sum(s['duration_sec'] for s in schedule) == settings.duration
    for index in range(int(settings.duration * 100) + 1):
        output, _ = evaluate(schedule, index / 100)
        assert 0 <= output.speed_kph <= speed
        if settings.steering_enabled:
            assert abs(output.steering_deg - CENTER_DEG) <= 1.0 + 1e-12
        else:
            assert output.steering_deg is None
        assert not output.brake_engage
    assert evaluate(schedule, settings.duration)[0] == baseline(
        steering_enabled=settings.steering_enabled)


@pytest.mark.parametrize('speed', [0, -1, float('nan'), float('inf')])
def test_invalid_operator_speed_is_rejected(speed):
    with pytest.raises(ValueError):
        RunSettings(1, speed)


def test_speed_steps_and_trapezoid_are_different_inputs():
    steps = segments(RunSettings(1, 15))
    assert [evaluate(steps, t)[0].speed_kph for t in (0, 3, 6, 8, 10, 13, 15, 17)] == [
        5, 0, 5, 10, 15, 10, 5, 0]
    ramp = segments(RunSettings(2, 15))
    assert [evaluate(ramp, t)[0].speed_kph for t in (0, 1.5, 3, 6, 8, 10)] == [
        0, 7.5, 15, 15, 7.5, 0]


def test_chirp_crossings_get_closer_and_end_at_center():
    schedule = segments(RunSettings(4, 10))
    crossings = []
    previous = 0
    for k in range(1, 10000):
        t = k / 1000
        value = evaluate(schedule, t)[0].steering_deg - CENTER_DEG
        if previous < 0 <= value:
            crossings.append(t)
        previous = value
    assert len(crossings) >= 4
    periods = [b - a for a, b in zip(crossings, crossings[1:])]
    assert all(a > b for a, b in zip(periods, periods[1:]))
    assert evaluate(schedule, 10)[0].steering_deg == CENTER_DEG


def telemetry():
    result = {topic: kind() for topic, kind in TOPICS.items()}
    drive = result['/drive/status']
    drive.part1_valid = drive.part2_valid = True
    drive.fault_bits_valid = True
    steering = result['/steering/status']
    steering.communication_ok = steering.feedback_valid = True
    fix = result['/gnss/fix']
    fix.status.status = 2
    fix.latitude, fix.longitude = 37.24, 126.77
    for child in ('gps', 'ebimu_imu_link'):
        transform = TransformStamped()
        transform.header.frame_id = 'base_link'
        transform.child_frame_id = child
        transform.transform.rotation.w = 1.0
        result['/tf_static'].transforms.append(transform)
    return result


@pytest.fixture
def controlled():
    rclpy.init()
    clock = [100.0]
    node = SysidNode(clock=lambda: clock[0])
    node.steering_control.ready = Mock(return_value=True)
    data = telemetry()
    node.graph_errors = []

    def tick(dt=0, auto=False, speed=0):
        clock[0] += dt
        data['/drive/status'].mode = int(auto)
        data['/drive/status'].speed_mps = speed / 3.6
        for topic, message in data.items():
            node._receive(topic, message)
        node._receive('/vehicle/command', ActuatorCommand())
        node._tick()

    tick()
    yield node, tick, clock
    node.destroy_node()
    rclpy.shutdown()


def begin(node, tick, scenario=1):
    node.request_run(RunSettings(scenario, 10))
    tick()
    node.commands.put(('record_ready', None))
    tick(auto=True)
    assert node.state == 'COUNTDOWN'


def test_run_waits_for_auto_then_five_seconds_and_manual_cancels(controlled):
    node, tick, _ = controlled
    node.request_run(RunSettings())
    tick()
    node.commands.put(('record_ready', None))
    tick()
    assert node.state == 'WAIT_AUTO'
    assert node.output.speed_kph == 0
    tick(auto=True)
    tick(4.99, auto=True)
    assert node.output.speed_kph == 0
    tick(0.01, auto=True)
    assert node.state == 'RUNNING'
    assert node.output.speed_kph > 0
    tick(auto=False)
    assert node.state == 'TAIL'
    assert node.output == baseline()
    tick(2.1, auto=True)
    assert node.state == 'CLOSING'
    assert node.output.speed_kph == 0


def test_duplicate_run_does_not_restart_recording_or_change_active_settings(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    node.request_run(RunSettings(4, 15))
    tick(auto=True)
    assert node.settings == RunSettings(1, 10)
    assert node.process_actions.qsize() == 1


@pytest.mark.parametrize('scenario', [5, 6])
def test_route_command_waits_for_countdown_and_stale_input_stops(controlled, scenario):
    node, tick, clock = controlled
    begin(node, tick, scenario=scenario)
    tick(4.9, auto=True)
    assert node.output.speed_kph == 0
    clock[0] += 0.1
    data = {topic: kind() for topic, kind in FOLLOW_TOPICS.items()}
    data['/sysid/route_context'].route_projection_valid = True
    data['/sysid/route_context'].distance_to_finish = 100.0
    data['/sysid/route_command'].speed_target_mps = 1.0
    data['/sysid/route_command'].steering_target_rad = 0.1
    for topic, message in data.items():
        node._receive(topic, message)
    tick(auto=True)
    assert node.state == 'RUNNING'
    assert node.output.speed_kph == pytest.approx(3.6)
    tick(0.21, auto=True)
    assert node.output.brake_engage
    assert node.terminal_state == 'ERROR'


def test_cruise_requires_actual_speed_and_timeout_ramps_to_zero(controlled):
    node, tick, _ = controlled
    begin(node, tick, scenario=3)
    tick(5, auto=True, speed=2)
    assert node.state == 'CRUISE'
    tick(15, auto=True, speed=2)
    assert node.state == 'DECEL'
    tick(2, auto=True, speed=2)
    assert node.output.speed_kph == pytest.approx(5)
    tick(2, auto=True)
    assert node.output == baseline(steering_enabled=True)
    assert node.terminal_state == 'ABORTED'


def test_cruise_starts_steering_after_one_second_near_target(controlled):
    node, tick, _ = controlled
    begin(node, tick, scenario=3)
    tick(5, auto=True, speed=10)
    tick(1, auto=True, speed=10)
    assert node.state == 'RUNNING'
    tick(1, auto=True, speed=10)
    assert node.output.steering_deg == CENTER_DEG + 1


def test_stop_and_recording_failure_request_brake(controlled):
    node, tick, _ = controlled
    begin(node, tick)
    node.commands.put(('record_failed', 'disk full'))
    tick(auto=True)
    assert node.state == 'CLOSING'
    assert node.output.brake_engage
    node.commands.put(('record_closed', '/record'))
    tick(auto=True)
    assert node.state == 'ERROR'
    assert not node.record_open


def test_motor_setting_wait_blocks_input_but_not_stop(controlled):
    node, tick, _ = controlled
    node.steering_control.ready.return_value = False
    node.command_publisher = Mock()
    node.request_run(RunSettings(1, 10))
    tick(auto=True)
    node.command_publisher.publish.assert_not_called()
    node.steering_control.ready.assert_called_with(False)
    node.request_stop()
    tick(auto=True)
    assert node.command_publisher.publish.call_args.args[0].brake_engage


def test_lost_telemetry_stops_active_input(controlled):
    node, tick, clock = controlled
    begin(node, tick)
    tick(5, auto=True)
    clock[0] += 1.1
    node._tick()
    assert node.output.brake_engage
    assert node.terminal_state == 'ERROR'


def test_missing_or_lost_imu_does_not_block_or_stop_run(controlled):
    node, tick, clock = controlled
    node.received.pop('/imu/data_raw')
    node.sensor_errors.pop('/imu/data_raw')
    node.scan_graph = lambda: None
    assert not any('/imu/data_raw' in issue for issue in node.issues())
    begin(node, tick)
    tick(5, auto=True)
    clock[0] += 1.1
    for topic, (_, message) in list(node.received.items()):
        if topic != '/imu/data_raw':
            node.received[topic] = (clock[0], message)
    node._tick()
    assert node.state == 'RUNNING'
    assert not node.output.brake_engage


@pytest.mark.parametrize('extra_planning', [True, False])
def test_unrelated_command_publisher_prevents_run_and_command_output(controlled, extra_planning):
    node, tick, _ = controlled
    other = Node('other_planner')
    if extra_planning:
        other.create_publisher(ActuatorCommand, '/planning/command', 10)
    other.create_publisher(ActuatorCommand, '/vehicle/command', 10)
    messages = []
    observer = Node('command_observer')
    observer.create_subscription(ActuatorCommand, '/planning/command', messages.append, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(other)
    executor.add_node(observer)
    try:
        deadline = time.monotonic() + 2
        while len(node.get_publishers_info_by_topic('/planning/command')) < 1 + int(extra_planning):
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=0.02)
        node.scan_graph()
        node.request_run(RunSettings())
        tick()
        deadline = time.monotonic() + 0.2
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert node.state == 'IDLE'
        assert node.process_actions.empty()
        assert not messages
    finally:
        executor.shutdown()
        observer.destroy_node()
        other.destroy_node()


def test_real_governor_and_mcap_record_mode_and_input(tmp_path):
    # Existing ROS transports are used; the test supplies telemetry instead of hardware.
    rclpy.init()
    node = SysidNode()
    source = Node('steering_actuator_node')
    source.declare_parameter('steering_enabled', True)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(source)
    messages = telemetry()
    publishers = {}
    for topic, kind in TOPICS.items():
        qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic == '/tf_static' else 10)
        publishers[topic] = source.create_publisher(kind, topic, qos)
    received = []
    source.create_subscription(ActuatorCommand, '/vehicle/command',
                               lambda m: received.append(m), 10)
    manager = sysid_processes.ProcessManager(node, records_root=tmp_path)
    last_publish = [0.0]
    static_sent = [False]

    def spin_until(condition, seconds=15):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - last_publish[0] >= 0.05:
                for topic, message in messages.items():
                    if topic != '/tf_static' or not static_sent[0]:
                        publishers[topic].publish(message)
                last_publish[0] = now
                static_sent[0] = True
            executor.spin_once(timeout_sec=0.03)
            if condition():
                return
        raise AssertionError((node.state, node.issues(), list(node.logs)[-8:]))

    governor = None
    try:
        governor = sysid_processes.OwnedProcess('test governor', [
            str(Path(get_package_prefix('command_governor')) /
                'lib/command_governor/command_governor_node'),
            '--ros-args', '--params-file',
            str(Path(get_package_share_directory('command_governor')) /
                'config/command_governor.yaml'),
        ], node.log)
        manager.start()
        spin_until(lambda: not node.issues())
        assert not received
        node.request_run(RunSettings(2, 5))
        spin_until(lambda: node.state == 'WAIT_AUTO')
        assert not source.get_parameter('steering_enabled').value
        assert all(m.speed_target_mps == 0 for m in received)
        messages['/drive/status'].mode = 1
        spin_until(lambda: node.state == 'COUNTDOWN')
        spin_until(lambda: node.state == 'RUNNING')
        spin_until(lambda: node.state == 'TAIL', seconds=16)
        assert node.state == 'TAIL'
        assert any(m.speed_target_mps > 0.5 for m in received)
        assert all(m.speed_target_mps <= 5 / 3.6 + 1e-6 for m in received)
        spin_until(lambda: node.state in ('COMPLETE', 'ERROR'))
        assert node.state == 'COMPLETE', list(node.logs)
        assert node.output == baseline()
        import rosbag2_py
        meta = rosbag2_py.Info().read_metadata(str(Path(node.run_dir) / 'rosbag'), 'mcap')
        counts = {entry.topic_metadata.name: entry.message_count for entry in meta.topics_with_message_count}
        assert counts['/planning/command'] > 0
        assert counts['/vehicle/command'] > 0
        assert counts['/drive/status'] > 0
        assert (Path(node.run_dir) / 'scenario.yaml').is_file()
        node.request_stop()
        spin_until(lambda: received[-1].brake_engage)
        manager.records_root = Path('/dev/null')
        node.request_run(RunSettings(1, 5))
        spin_until(lambda: node.state == 'ERROR')
        assert node.output.brake_engage
        assert not node.record_open
        assert manager.recorder is None
    finally:
        node.request_stop()
        executor.spin_once(timeout_sec=0.1)
        manager.close()
        if governor is not None:
            governor.stop()
        executor.shutdown()
        source.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
        if governor is not None:
            assert not sysid_processes.group_members(governor.pgid)
