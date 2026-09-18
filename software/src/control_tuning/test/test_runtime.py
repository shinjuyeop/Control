import time
from types import SimpleNamespace

import pytest
import rclpy
from kaiev26_msgs.msg import DriveActuatorStatus, SteeringActuatorStatus
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from control_tuning.runtime import TuningNode, stop_reason
from control_tuning.schema import STEERING, GOVERNOR, READ_NAMES, LIVE_NAMES


@pytest.fixture
def ros_context(monkeypatch):
    # Separate from the vehicle ROS graph; publish diagnostic status only.
    monkeypatch.setenv('ROS_DOMAIN_ID', '218')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    try:
        yield
    finally:
        rclpy.shutdown()


@pytest.fixture
def ros_pair(store, ros_context):
    server = Node(STEERING, enable_rosout=False)
    governor = Node(GOVERNOR, enable_rosout=False)
    for node, name in ((server, STEERING), (governor, GOVERNOR)):
        for key in READ_NAMES[name]:
            node.declare_parameter(key, store.saved_native[name][key])
    client = TuningNode()
    executor = SingleThreadedExecutor()
    for node in (client, server, governor):
        executor.add_node(node)
    drive_pub = server.create_publisher(DriveActuatorStatus, '/drive/status', 10)
    steer_pub = server.create_publisher(SteeringActuatorStatus, '/steering/status', 10)
    drive = DriveActuatorStatus(mode=DriveActuatorStatus.MODE_MANUAL, speed_mps=0., part1_valid=True)
    steering = SteeringActuatorStatus(state=SteeringActuatorStatus.STATE_DISABLED)
    writes = []
    reject = []

    def record(parameters):
        writes.append(parameters)
        return SetParametersResult(successful=not reject, reason='test rejection' if reject else '')

    server.add_on_set_parameters_callback(record)

    def pump(until, timeout=5, publish=True):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publish:
                drive_pub.publish(drive)
                steer_pub.publish(steering)
            executor.spin_once(timeout_sec=.01)
            client.tick()
            if until():
                return
        pytest.fail(f'ROS condition timed out: {client.errors}, {client.result}')

    try:
        pump(lambda: client.fresh_values(STEERING) and client.fresh_values(GOVERNOR)
             and not client.apply_reason(store.saved_native[STEERING]))
        yield SimpleNamespace(client=client, server=server, governor=governor, pump=pump,
                              drive=drive, steering=steering, writes=writes, reject=reject)
    finally:
        executor.shutdown()
        for node in (client, server, governor):
            node.destroy_node()


def test_atomic_apply_readback_keeps_startup_values_and_file(ros_pair, store, editor):
    pair = ros_pair
    editor.draft.update(p_gain_a_per_deg=40., d_gain_a_per_deg_s=.05,
                        peak_current_command_a=12., minimum_vehicle_speed_kph=1.,
                        governor_input_timeout_ms=250.)
    editor.key('a', pair.client)
    assert editor.modal['action'] == 'apply'
    assert len(editor.modal['rows']) == 3
    editor.key('enter', pair.client)
    pair.pump(lambda: pair.client.result is not None)
    assert pair.client.result[0], pair.client.result
    assert len(pair.writes) == 1
    assert {p.name for p in pair.writes[0]} == set(LIVE_NAMES)
    assert pair.server.get_parameter('torque_full_scale_error_deg').value == .3
    assert pair.server.get_parameter('torque_peak_limit_raw').value == 7500
    assert pair.server.get_parameter('minimum_steering_speed_kph').value == 0.
    assert pair.governor.get_parameter('input_timeout_ms').value == 200.
    assert not store.path.exists()


@pytest.mark.parametrize('change', ['external_parameter', 'moving', 'auto', 'active', 'enabled'])
def test_preflight_rejects_changes_without_writing(ros_pair, store, editor, change):
    pair = ros_pair
    editor.draft['p_gain_a_per_deg'] = 40.
    editor.key('a', pair.client)
    assert editor.modal
    if change == 'external_parameter':
        pair.server.set_parameters_atomically([Parameter('torque_kd', value=1600.)])
        pair.writes.clear()
    else:
        if change == 'moving':
            pair.drive.speed_mps = 1.
        elif change == 'auto':
            pair.drive.mode = DriveActuatorStatus.MODE_AUTO
        elif change == 'active':
            pair.steering.state = SteeringActuatorStatus.STATE_ACTIVE
        else:
            pair.steering.enabled_commanded = True
        pair.client._drive(pair.drive)
        pair.client._steering(pair.steering)
    editor.key('enter', pair.client)
    if pair.client.operation:
        pair.pump(lambda: pair.client.result is not None)
        assert not pair.client.result[0]
    else:
        assert editor.error
    assert not pair.writes
    assert pair.server.get_parameter('torque_full_scale_error_deg').value == .5


def test_server_rejection_is_reported_and_no_partial_write(ros_pair, store):
    pair = ros_pair
    pair.reject.append(True)
    pair.client.begin_apply({**store.saved, 'p_gain_a_per_deg': 40},
                            store.saved_native[STEERING], pair.client.fresh_values(STEERING))
    pair.pump(lambda: pair.client.result is not None)
    assert not pair.client.result[0]
    assert 'test rejection' in pair.client.result[1]
    assert pair.server.get_parameter('torque_full_scale_error_deg').value == .5
    pair.pump(lambda: not pair.client.uncertain)


def test_runtime_guards_and_stale_values(ros_pair, store):
    pair = ros_pair
    context = store.saved_native[STEERING]
    pair.client.drive = (time.monotonic() - 1, pair.drive)
    assert '오래된' in pair.client.apply_reason(context)
    pair.client._drive(pair.drive)
    pair.client.values[STEERING]['actuation_mode'] = 'position'
    assert 'Position' in pair.client.apply_reason(context)
    pair.client.values[STEERING]['actuation_mode'] = 'torque'
    assert '환산' in pair.client.apply_reason({**context, 'keya_protection_current_a': 10})
    pair.client.received[STEERING] -= 5
    assert pair.client.fresh_values(STEERING) is None


def test_stop_requires_valid_finite_stationary_status():
    drive = DriveActuatorStatus(mode=DriveActuatorStatus.MODE_MANUAL, part1_valid=True)
    steer = SteeringActuatorStatus(state=SteeringActuatorStatus.STATE_DISABLED)
    assert not stop_reason((1., drive), (1., steer), 1.1)
    for speed in (float('nan'), float('inf'), -.1, .1):
        drive.speed_mps = speed
        assert stop_reason((1., drive), (1., steer), 1.1)
    drive.speed_mps = 0.
    drive.part1_valid = False
    assert stop_reason((1., drive), (1., steer), 1.1)
    assert stop_reason(None, (1., steer), 1.1)
    assert stop_reason((1., drive), None, 1.1)
