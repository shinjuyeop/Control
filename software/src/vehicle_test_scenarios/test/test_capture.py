from pathlib import Path
from queue import SimpleQueue
import time
import math
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import yaml

from vehicle_test_scenarios.sysid_capture import ParameterCapture, TORQUE_TOPICS
from vehicle_test_scenarios import sysid_processes
from control_tuning.schema import STEERING, GOVERNOR


@pytest.fixture
def parameter_nodes(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '221')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    reader, steering, governor = Node('snapshot_test'), Node(STEERING), Node(GOVERNOR)
    packages = Path(__file__).resolve().parents[2]
    values = yaml.safe_load((packages / 'lateral_ctrl/config/steering_actuator.yaml').read_text())[STEERING]['ros__parameters']
    values.update(yaml.safe_load((packages / 'lateral_ctrl/config/steering_motor_calibration.yaml').read_text()))
    values.update(steering_enabled=True, torque_kd=1723.0, torque_direction_sign=-1)
    for key, value in values.items():
        steering.declare_parameter(key, value)
    for key, value in dict(input_topic='/planning/command', output_topic='/vehicle/command', input_timeout_ms=234.).items():
        governor.declare_parameter(key, value)
    executor = SingleThreadedExecutor()
    for node in (reader, steering, governor):
        executor.add_node(node)

    def capture(condition='airborne'):
        snapshot = ParameterCapture(reader, condition)
        try:
            deadline = time.monotonic() + 7
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=.01)
                if snapshot.tick():
                    return snapshot.document
            pytest.fail('snapshot service timed out')
        finally:
            snapshot.close()

    try:
        yield reader, steering, governor, capture
    finally:
        executor.shutdown()
        for node in (reader, steering, governor):
            node.destroy_node()
        rclpy.shutdown()


def test_capture_reads_actual_values_all_names_and_clock(parameter_nodes):
    _, steering, _, capture = parameter_nodes
    steering.declare_parameter('operator_note', 'actual runtime parameter')
    document = capture()
    assert document['status'] == 'complete'
    actual = document['nodes']['/steering_actuator_node']
    assert document['schema_version'] == 2
    assert actual['ros__parameters']['torque_kd'] == 1723.
    assert actual['display_values']['steering_kd_a_per_deg_s'] == pytest.approx(1723 * math.pi / 180 * .0016)
    assert actual['display_units']['steering_kd_a_per_deg_s'] == 'A/(deg/s)'
    assert actual['display_values']['steering_torque_feedback_timeout_ms'] == 100
    assert document['nodes']['/command_governor']['display_values']['governor_command_timeout_ms'] == 234.
    assert actual['ros__parameters']['torque_direction_sign'] == -1
    assert actual['ros__parameters']['operator_note'] == 'actual runtime parameter'
    assert actual['received_ros_time_ns'] >= document['requested_ros_time_ns'] > 0
    assert document['test_condition']['key'] == 'airborne'
    assert document['nodes']['/command_governor']['ros__parameters']['input_timeout_ms'] == 234.


def test_tui_adopts_running_launch_preset_and_preserves_operator_choice(parameter_nodes):
    from vehicle_test_scenarios.sysid_runtime import SysidNode
    from vehicle_test_scenarios.sysid_tui import MenuState
    _, steering, _, capture = parameter_nodes
    steering.declare_parameter('tuning_preset', 'airborne')
    document = capture('stationary')
    assert document['test_condition']['key'] == 'stationary'
    assert document['nodes']['/steering_actuator_node']['ros__parameters']['tuning_preset'] == 'airborne'
    node = SysidNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(steering)
    menu = MenuState()
    try:
        deadline = time.monotonic() + 6
        while node.launch_preset == 'unspecified' and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        assert node.launch_preset == 'airborne'
        menu.adopt_launch_preset(node.snapshot()['launch_preset'])
        assert menu.run_settings().test_condition == 'airborne'
        menu.test_condition = 'stationary'
        menu.adopt_launch_preset('airborne')
        assert menu.run_settings().test_condition == 'stationary'
    finally:
        executor.remove_node(steering)
        executor.shutdown()
        node.destroy_node()


def test_missing_parameter_is_explicit_not_filled_from_yaml(parameter_nodes):
    _, steering, _, capture = parameter_nodes
    steering.undeclare_parameter('torque_kd')
    document = capture()
    assert document['status'] == 'incomplete'
    actual = document['nodes']['/steering_actuator_node']
    assert actual['status'] == 'partial'
    assert 'torque_kd' in actual['missing_parameters']
    assert 'torque_kd' not in actual['ros__parameters']


def test_snapshot_unavailable_node_times_out_and_cleans_clients(parameter_nodes):
    reader, _, _, _ = parameter_nodes
    capture = ParameterCapture(reader, 'stationary', timeout=0)
    capture.started -= 1
    assert capture.tick()
    assert capture.document['status'] == 'incomplete'
    capture.close()
    assert not tuple(reader.clients)


@pytest.mark.parametrize('complete', [True, False])
def test_recorder_only_announces_ready_after_snapshot_saved(tmp_path, monkeypatch, complete):
    node = Mock()
    node.required_topics = ('/drive/status',)
    node.commands = SimpleQueue()
    node.get_subscriptions_info_by_topic.return_value = [SimpleNamespace(node_name='rosbag2_recorder', endpoint_gid=b'new')]
    manager = sysid_processes.ProcessManager(node, tmp_path)
    manager.run_dir = tmp_path
    manager.run_document = {'test_condition': {'key': 'airborne'}}
    manager.subscribers_before_recording = {t: set() for t in (*manager.record_topics_required, '/parameter_events')}
    manager.recorder = Mock()
    manager.recorder.process.poll.return_value = None
    capture = Mock()
    capture.tick.return_value = True
    capture.document = {
        'status': 'complete' if complete else 'incomplete',
        'capture_stage': 'recorder_ready_before_run', 'requested_ros_time_ns': 1,
        'nodes': {'/steering_actuator_node': {'status': 'complete' if complete else 'failed'}},
    }
    monkeypatch.setattr(sysid_processes, 'ParameterCapture', Mock(return_value=capture))
    manager._finish_recording = Mock()

    def published(_message):
        assert (tmp_path / 'control_parameters.yaml').is_file()
        assert yaml.safe_load((tmp_path / 'scenario.yaml').read_text())['control_parameters']['status'] == 'complete'

    node.control_parameters_publisher.publish.side_effect = published
    manager._record_status()
    action, _ = node.commands.get_nowait()
    assert action == ('record_ready' if complete else 'record_failed')
    assert manager.recorder_ready is complete
    capture.close.assert_called_once()
    if complete:
        node.control_parameters_publisher.publish.assert_called_once()
    else:
        manager._finish_recording.assert_called_once_with('parameter_snapshot_failed')
        node.control_parameters_publisher.publish.assert_not_called()


def test_torque_record_topics_are_complete_and_not_duplicated():
    assert len(sysid_processes.RECORD_TOPICS) == len(set(sysid_processes.RECORD_TOPICS))
    assert {'/parameter_events', '/sysid/control_parameters', *TORQUE_TOPICS} <= set(sysid_processes.RECORD_TOPICS)
