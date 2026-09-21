import math
import struct
import time

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
        assert actual['display_values']['drive_current_limit_a'] == config['max_drive_current_a']
        assert actual['display_values']['drive_speed_kp_a_per_mps'] == config['speed_kp_a_per_mps']
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


def test_longitudinal_preset_edits_are_saved_without_applying(editor, store):
    editor.key('3')
    editor.commit('torque')
    editor.key('a')
    assert editor.modal is None
    editor.key('s')
    saved = editor.presets.load('airborne')
    assert saved['drive_control_mode'] == 'torque'
    assert '다음 구동 스택 실행' in editor.message


def test_checked_in_drive_envelopes_and_launch_mapping():
    from control_tuning.storage import SettingsStore
    from control_tuning.presets import PresetStore, load_launch_overrides
    from control_tuning.longitudinal import LONGITUDINAL
    settings = SettingsStore()
    presets = PresetStore(settings)
    airborne = presets.load('airborne')
    expected = {
        'drive_control_mode':'torque', 'drive_speed_kp_a_per_mps':15.,
        'drive_speed_ki_a_per_mps_s':2., 'drive_speed_deadband_mps':.05,
        'drive_coast_release_error_mps':.12, 'drive_current_limit_a':20.,
        'drive_integrator_limit_a':6., 'drive_current_slew_up_a_s':120.,
        'drive_current_slew_down_a_s':400., 'drive_reapply_current_band_a':5.,
        'drive_reapply_current_slew_a_s':40., 'drive_motor_speed_limit_rpm':600.,
    }
    assert {k:airborne[k] for k in expected} == expected
    native = load_launch_overrides(settings.path,LONGITUDINAL,'airborne')
    assert native == AIRBORNE
    assert request_data(native,1,0)[6] == 20.
    assert request_data(native,1,0)[14] == 600.
    driving = load_launch_overrides(settings.path,LONGITUDINAL,'driving')
    assert driving == {**DEFAULT,'control_mode':'torque'}
