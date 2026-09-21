import math
import struct
import time
from types import SimpleNamespace

import pytest
import rclpy
from kaiev26_msgs.msg import DriveActuatorStatus
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray

from control_tuning.longitudinal import (
    DEFAULT, AIRBORNE, CONFIG_KEYS, CONFIG_TOPIC, STATUS_TOPIC,
    config_wire, decode_status, request_data, validate_config, wire_equal,
)
from control_tuning.longitudinal_runtime import LongitudinalClient


def status_data(config, revision=0, request_id=0, result=0, source=0):
    return [1., float(revision), float(request_id), float(result), float(source),
            *config_wire(config), *([0.] * 8), 1., 1.]


def test_candidate_roundtrip_and_wire_precision():
    for config in (DEFAULT, AIRBORNE):
        data = status_data(config)
        data = struct.unpack('<27f', struct.pack('<27f', *data))
        actual = decode_status(data)
        assert wire_equal(config, actual['config'])
        assert actual['source'] == 'DEFAULT'
        assert actual['feedback_valid'] == 1
    assert DEFAULT['max_drive_current_a'] == 480
    assert AIRBORNE['max_drive_current_a'] < DEFAULT['max_drive_current_a']


@pytest.mark.parametrize('key,value', [
    ('speed_kp_a_per_mps', math.nan), ('speed_ki_a_per_mps_s', -1.),
    ('max_drive_current_a', 481.), ('integrator_limit_a', 481.),
    ('current_slew_up_a_s', 0.), ('current_slew_down_a_s', math.inf),
    ('speed_deadband_mps', .3), ('speed_hysteresis_mps', .01),
    ('reapply_current_a', -1.), ('reapply_slew_a_s', 3000.), ('motor_rpm_limit', 5000.),
    ('control_mode', 'regen'),
])
def test_invalid_request_rejected_at_host_boundary(key, value):
    with pytest.raises(ValueError):
        request_data({**DEFAULT, key:value}, 1, 0)


def test_ros_apply_waits_for_matching_mcu_readback(monkeypatch):
    # No vehicle command or CAN interfaces. Deterministic in-process DDS endpoints.
    monkeypatch.setenv('ROS_DOMAIN_ID', '217')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    rclpy.init(args=[])
    server = Node('stm_a_node', enable_rosout=False)
    node = Node('longitudinal_test_client', enable_rosout=False)
    executor = SingleThreadedExecutor()
    executor.add_node(server); executor.add_node(node)
    client = LongitudinalClient(node)
    status_pub = server.create_publisher(Float32MultiArray, STATUS_TOPIC, 10)
    drive_pub = server.create_publisher(DriveActuatorStatus, '/drive/status', 10)
    requests = []
    server.create_subscription(Float32MultiArray, CONFIG_TOPIC, requests.append, 10)
    drive = DriveActuatorStatus(mode=0, part1_valid=True, part2_valid=True)
    data = status_data(DEFAULT)

    def pump(predicate):
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            status_pub.publish(Float32MultiArray(data=data))
            drive_pub.publish(drive)
            executor.spin_once(timeout_sec=.01)
            client.tick()
            if predicate():
                return
        pytest.fail('DDS readback did not arrive')

    try:
        pump(lambda: not client.reason())
        client.begin(AIRBORNE, 0)
        pump(lambda: bool(requests))
        assert client.result is None  # sent is not applied
        request = requests[0].data
        data = status_data(AIRBORNE, 1, int(request[1]), 1, 1)
        pump(lambda: client.result is not None)
        assert client.result[0]
        assert client.fresh()['config']['control_mode'] == 'torque'
        with pytest.raises(ValueError):
            client.begin(DEFAULT, 0)  # changed revision since UI review
        drive.mode = 1
        pump(lambda: client.drive[1].mode == 1)
        with pytest.raises(ValueError, match='MANUAL'):
            client.begin(DEFAULT, 1)
        drive.mode = 0
        pump(lambda: not client.reason())
        client.begin(DEFAULT, 1)
        pump(lambda: len(requests) == 2)
        data = status_data(AIRBORNE, 1, int(requests[-1].data[1]), 3, 1)
        pump(lambda: client.result is not None)
        assert not client.result[0]  # board rechecks stop gate and rejects
        assert client.fresh()['config']['control_mode'] == 'torque'
        # MCU reboot readback explicitly restores source DEFAULT/revision 0;
        # the host observes it but never auto-replays the old override.
        data = status_data(DEFAULT)
        pump(lambda: client.fresh()['source'] == 'DEFAULT')
        assert len(requests) == 2
        client.status = (time.monotonic()-1, client.status[1])
        with pytest.raises(ValueError, match='readback'):
            client.begin(DEFAULT, 0)
    finally:
        executor.shutdown()
        node.destroy_node(); server.destroy_node()
        rclpy.shutdown()


def test_longitudinal_preset_edit_save_apply_review(editor, store):
    editor.key('5')
    editor.commit('torque')
    editor.key('s')
    saved = editor.presets.load('airborne')
    assert saved['control_mode'] == 'torque'
    class Runtime:
        operation = None
        longitudinal = SimpleNamespace(reason=lambda:'', fresh=lambda:dict(revision=3,config=DEFAULT))
        def begin_longitudinal_apply(self, config, revision):
            self.applied = (config,revision)
    runtime = Runtime()
    editor.key('a',runtime)
    assert editor.modal['action'] == 'apply_longitudinal'
    editor.key('enter',runtime)
    assert runtime.applied[0]['control_mode'] == 'torque'
    assert runtime.applied[1] == 3
