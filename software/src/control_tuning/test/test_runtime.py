import time
from types import SimpleNamespace

import pytest
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float32MultiArray

from control_tuning.runtime import TuningNode
from control_tuning.schema import STEERING, GOVERNOR, READ_NAMES
from control_tuning.longitudinal import LONGITUDINAL, STATUS_TOPIC, CONFIG_TOPIC, DEFAULT, config_wire


@pytest.fixture
def ros_pair(store, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '218')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    server = Node(STEERING, enable_rosout=False)
    governor = Node(GOVERNOR, enable_rosout=False)
    for node, name in ((server, STEERING), (governor, GOVERNOR)):
        for key in READ_NAMES[name]:
            node.declare_parameter(key, store.saved_native[name][key])
    client = TuningNode()
    executor = SingleThreadedExecutor()
    for node in (client, server, governor):
        executor.add_node(node)
    status_pub = server.create_publisher(Float32MultiArray, STATUS_TOPIC, 10)
    status = Float32MultiArray(data=[1., 0., 0., 0., 0., *config_wire(DEFAULT), *([0.] * 10)])
    writes = []

    def record(parameters):
        writes.append(parameters)
        return SetParametersResult(successful=True)

    server.add_on_set_parameters_callback(record)

    def pump(until):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            status_pub.publish(status)
            executor.spin_once(timeout_sec=.01)
            client.tick()
            if until():
                return
        pytest.fail(f'ROS read timed out: {client.errors}')

    try:
        pump(lambda: all(client.fresh_values(n) for n in (STEERING, GOVERNOR, LONGITUDINAL)))
        yield SimpleNamespace(client=client, server=server, governor=governor, pump=pump,
                              writes=writes, status=status)
    finally:
        executor.shutdown()
        for node in (client, server, governor):
            node.destroy_node()
        rclpy.shutdown()


def test_tui_save_does_not_change_running_settings(ros_pair, store, editor):
    pair = ros_pair
    editor.draft.update(steering_kp_a_per_deg=40., governor_command_timeout_ms=250., drive_control_mode='torque')
    for group in ('1', '3', '5'):
        editor.key(group)
        editor.key('a', pair.client)
        assert editor.modal is None
    editor.key('s', pair.client)
    saved = editor.presets.load('airborne')
    assert saved['steering_kp_a_per_deg'] == 40.
    assert saved['drive_control_mode'] == 'torque'
    pair.client.tick(force=True)
    pair.pump(lambda: not pair.client.requests)
    assert not pair.writes
    assert pair.server.get_parameter('torque_full_scale_error_deg').value == .5
    assert pair.governor.get_parameter('input_timeout_ms').value == 200.
    assert pair.client.fresh_values(LONGITUDINAL)['control_mode'] == 'speed'
    assert all(c.srv_name.endswith('/get_parameters') for c in pair.client.clients)
    assert not {CONFIG_TOPIC, '/vehicle/command', '/planning/command'} & {
        p.topic_name for p in pair.client.publishers}


def test_current_values_follow_external_changes_and_expire(ros_pair, editor):
    pair = ros_pair
    pair.server.set_parameters_atomically([Parameter('torque_full_scale_error_deg', value=.4)])
    pair.client.tick(force=True)
    pair.pump(lambda: pair.client.fresh_values(STEERING)['torque_full_scale_error_deg'] == .4)
    assert editor.live_values(pair.client)['steering_kp_a_per_deg'] == 40.
    assert pair.client.longitudinal_status()['source'] == 'DEFAULT'
    pair.client.received[STEERING] -= 5
    pair.client.received[LONGITUDINAL] -= 1
    assert pair.client.fresh_values(STEERING) is None
    assert pair.client.longitudinal_status() is None


def test_invalid_readback_is_not_shown_as_current(ros_pair):
    pair = ros_pair
    pair.status.data = []
    pair.pump(lambda: pair.client.fresh_values(LONGITUDINAL) is None)
    assert LONGITUDINAL in pair.client.errors
