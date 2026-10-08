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
    DEFAULT, CONFIG_KEYS, CONFIG_TOPIC, STATUS_TOPIC, STATE_KEYS, VERSION, WIRE_KEYS,
    collapse_flags, config_wire, decode_status, request_data, resolve, status_data as v5_status,
    validate_config, wire_equal,
)
from control_tuning.longitudinal_runtime import LongitudinalClient


# Deliberate test input for the Torque path, independent of operator presets.
TORQUE_CONFIG = {**DEFAULT, 'control_mode': 'torque', 'speed_kp_a_per_mps': 25.,
                 'speed_ki_a_per_mps_s': 2., 'max_drive_current_a': 40., 'integrator_limit_a': 6.,
                 'current_slew_up_a_s': 120., 'current_slew_down_a_s': 400.,
                 'reapply_current_a': 5., 'reapply_slew_a_s': 20.}


def status_data(config, revision=0, request_id=0, result=0, source=0):
    return v5_status(config, revision=revision, request_id=request_id, result=result, source=source)


def test_candidate_roundtrip_and_wire_precision():
    count = 5 + len(WIRE_KEYS) + len(STATE_KEYS)
    assert count == 78
    for config in (DEFAULT, TORQUE_CONFIG):
        data = status_data(config)
        assert len(data) == count and data[0] == VERSION == 5
        data = struct.unpack(f'<{count}f', struct.pack(f'<{count}f', *data))
        actual = decode_status(data)
        assert wire_equal(config, actual['config'])
        assert actual['display_values']['drive_current_limit_a'] == config['max_drive_current_a']
        assert actual['display_values']['drive_speed_kp_a_per_mps'] == config['speed_kp_a_per_mps']
        assert actual['display_values']['drive_motor_waveform'] == 'sine'
        assert actual['source'] == 'DEFAULT'
        assert actual['feedback_valid'] == 1 and actual['collapse_age_s'] == -1
    assert DEFAULT['max_drive_current_a'] == 480
    assert TORQUE_CONFIG['max_drive_current_a'] < DEFAULT['max_drive_current_a']
    assert len(request_data(DEFAULT, 1, 0)) == 3 + 37
    # Older firmware cannot take the regen limits: refuse rather than reinterpret.
    for version in (1, 2, 3, 4):
        with pytest.raises(ValueError, match='v5'):
            request_data(DEFAULT, 1, 0, version=version)


def test_legacy_readback_still_decodes():
    """MCAP analysis reads v1~v3 readbacks; their slot 12 is the coast-only floor."""
    head = [3., 2., 7., 1., 1., 1., 25., 2., 40., 6., 120., 50., .05, .12, 5., 20., 4500., 12.]
    v3 = decode_status(head + [.1, 1., 2., 3., 3., 0., 0., 0., 1., 0.])
    assert v3['version'] == 3 and v3['config']['coast_current_floor_a'] == 12.
    assert v3['display_values']['drive_coast_current_floor_a'] == 12.
    assert v3['current_command_a'] == 3. and 'rolling_hold' not in v3
    v2 = decode_status([2.] + head[1:-1] + [.1, 1., 2., 3., 3., 0., 0., 0., 1., 0.])
    assert v2['config']['coast_current_floor_a'] == 0
    with pytest.raises(ValueError):
        decode_status([4.] + head[1:] + [0.] * 10)
    # 2026-10-01 sine recordings are v4: 24 config + 28 state floats.
    v4 = [4., 6., 9., 1., 1., *config_wire(TORQUE_CONFIG)[:24], *([0.] * 28)]
    v4[5 + 24 + 9] = 1.
    decoded = decode_status(v4)
    assert decoded['version'] == 4 and decoded['config']['motor_waveform'] == 'sine'
    assert 'regen_max_current_a' not in decoded['config'] and 'regen_current_a' not in decoded
    assert decoded['feedback_valid'] == 1. and decoded['collapse_events'] == 0.


def test_waveform_specific_values_are_resolved_for_the_wire():
    square = {**TORQUE_CONFIG, 'motor_waveform': 'square', 'ff_current_gain_square_mps2_per_a': .064}
    wire = request_data(square, 1, 0)[3:]
    assert wire[WIRE_KEYS.index('min_drive_current_a')] == 12.
    assert wire[WIRE_KEYS.index('motor_waveform')] == 0.
    assert wire[WIRE_KEYS.index('ff_current_gain_mps2_per_a')] == pytest.approx(.064)
    sine = {**square, 'motor_waveform': 'sine'}
    wire = request_data(sine, 1, 0)[3:]
    assert wire[WIRE_KEYS.index('min_drive_current_a')] == 0.
    assert wire[WIRE_KEYS.index('motor_waveform')] == 1.
    assert wire[WIRE_KEYS.index('ff_current_gain_mps2_per_a')] == 0.
    # Readback names the active slot only; the other waveform's value is not on the wire.
    readback = decode_status(v5_status(square))['config']
    assert readback['min_drive_current_square_a'] == 12. and 'min_drive_current_sine_a' not in readback
    assert wire_equal(square, readback) and not wire_equal(sine, readback)
    assert wire_equal(square, {**square, 'min_drive_current_sine_a': 3.})
    assert not wire_equal(square, {**square, 'min_drive_current_square_a': 10.})
    assert resolve(readback)['min_drive_current_a'] == 12.
    # Integers and switches are carried exactly.
    custom = {**sine, 'start_step': 'disabled', 'reverse_debounce_frames': 5, 'collapse_hold_frames': 6}
    back = decode_status(v5_status(custom))['config']
    assert back['start_step'] == 'disabled' and back['reverse_debounce_frames'] == 5
    assert back['collapse_hold_frames'] == 6 and wire_equal(custom, back)


@pytest.mark.parametrize('key,value', [
    ('speed_kp_a_per_mps', math.nan), ('speed_ki_a_per_mps_s', -1.),
    ('max_drive_current_a', 481.), ('integrator_limit_a', 481.),
    ('current_slew_up_a_s', 0.), ('current_slew_down_a_s', math.inf),
    ('speed_deadband_mps', .3), ('speed_hysteresis_mps', .01),
    ('reapply_current_a', -1.), ('reapply_slew_a_s', 3000.), ('motor_rpm_limit', 5000.),
    ('control_mode', 'regen'), ('motor_waveform', 'trapezoid'), ('start_step', True),
    ('min_drive_current_square_a', -1.), ('min_drive_current_square_a', 31.),
    ('min_drive_current_sine_a', 30.5), ('hold_on_speed_mps', .05), ('hold_on_speed_mps', 2.5),
    ('hold_off_speed_mps', -.1), ('hold_off_speed_mps', .35), ('reverse_speed_mps', .01),
    ('reverse_speed_mps', 1.5), ('reverse_debounce_frames', 0), ('reverse_debounce_frames', 11),
    ('reverse_debounce_frames', 2.5), ('collapse_hold_frames', -1), ('collapse_hold_frames', 41),
    ('ff_current_gain_square_mps2_per_a', .0001), ('ff_current_gain_sine_mps2_per_a', .3),
    ('ff_rolling_resistance_mps2', 2.5), ('ff_speed_damping_per_s', -.1), ('ff_accel_limit_mps2', 6.),
    ('regen_max_current_a', -1.), ('regen_max_current_a', 61.), ('regen_slew_a_s', 5.),
    ('regen_fade_speed_mps', .2), ('regen_cutoff_speed_mps', 1.5), ('regen_voltage_soft_v', 58.),
    ('regen_voltage_hard_v', 60.5), ('regen_bus_current_limit_a', 40.), ('regen_rpm_field_mode', 'max'),
    ('pi_regen_enabled', 'on'), ('regen_enter_error_mps', .1), ('regen_exit_error_mps', .3),
    ('regen_kp_a_per_mps', 501.), ('collapse_phase_low_sine_a', -5.), ('collapse_phase_low_square_a', 30.),
])
def test_invalid_request_rejected_at_host_boundary(key, value):
    with pytest.raises(ValueError):
        request_data({**DEFAULT, 'control_mode':'torque', key:value}, 1, 0)


def test_regen_fields_reach_the_wire_and_default_off():
    assert DEFAULT['regen_max_current_a'] == 0 and DEFAULT['pi_regen_enabled'] == 'disabled'
    regen = {**TORQUE_CONFIG, 'regen_max_current_a': 20., 'regen_rpm_field_mode': 'ceiling',
             'pi_regen_enabled': 'enabled', 'regen_kp_a_per_mps': 40., 'motor_waveform': 'square'}
    wire = request_data(regen, 1, 0)[3:]
    assert wire[WIRE_KEYS.index('regen_max_current_a')] == 20.
    assert wire[WIRE_KEYS.index('regen_rpm_field_mode')] == 1. and wire[WIRE_KEYS.index('pi_regen_enabled')] == 1.
    assert wire[WIRE_KEYS.index('collapse_phase_low_a')] == -30.   # square threshold
    assert request_data({**regen, 'motor_waveform': 'sine'}, 1, 0)[3:][-1] == -60.
    back = decode_status(v5_status(regen, dict(regen_current_a=-12., regen_active=1.)))
    assert wire_equal(regen, back['config']) and back['regen_current_a'] == -12.
    assert back['config']['collapse_phase_low_square_a'] == -30.
    assert back['display_values']['drive_regen_max_current_a'] == 20.


def test_collapse_rules_mirror_firmware():
    # Square 00:11 spike, sine 0 A noise, sine 350 A launch, regen spike.
    assert collapse_flags(12, 12, 257.6, 171.7, 802, 948, True, True, -30) == 2 | 8
    assert collapse_flags(0, 0, -51.3, 2.6, 2354, 2318, True, True, -60) == 0
    assert collapse_flags(0, 0, -51.3, 2.6, 2354, 2318, True, True, -30) == 4
    assert collapse_flags(353.5, 426.8, 341.4, 77.2, 959, 578, True, True, -60) == 0
    assert collapse_flags(-20, -20, -150, -30, 1200, 1190, True, True, -60) == 2
    assert collapse_flags(-20, -15, -22, -12, 1200, 1190, True, True, -60) == 0
    # The frame after regen is cut to 0 A still shows the regen current.
    assert collapse_flags(0, -40, -38, -12, 260, 300, True, True, -30) == 0
    assert collapse_flags(0, 0, -38, -12, 260, 300, True, True, -30) == 4


def test_speed_current_limit_does_not_depend_on_inactive_pi_settings():
    config = {**DEFAULT, 'max_drive_current_a':10.}
    assert request_data(config, 1, 0)[6] == 10.
    with pytest.raises(ValueError):
        request_data({**config, 'control_mode':'torque'}, 1, 0)
    # The square floor (12 A) is above a 10 A limit; it only matters when square is active.
    with pytest.raises(ValueError, match='최소 구동 전류'):
        request_data({**config, 'motor_waveform':'square'}, 1, 0)


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
        client.begin(TORQUE_CONFIG, 0)
        pump(lambda: bool(requests))
        assert client.result is None  # sent is not applied
        request = requests[0].data
        data = status_data(TORQUE_CONFIG, 1, int(request[1]), 1, 1)
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
        data = status_data(TORQUE_CONFIG, 1, int(requests[-1].data[1]), 3, 1)
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
    for condition in ('airborne', 'driving'):
        assert presets.is_current(condition)
        values = presets.load(condition)
        # Sine (FOC) basis: no floor; square keeps the logged 12 A and has no feedforward.
        assert values['drive_motor_waveform'] == 'sine'
        assert values['drive_min_current_sine_a'] == 0. and values['drive_min_current_square_a'] == 12.
        assert values['drive_ff_current_gain_square_mps2_per_a'] == 0.
        assert (values['drive_hold_on_speed_mps'], values['drive_hold_off_speed_mps']) == (.35, .2)
        assert (values['drive_reverse_speed_mps'], values['drive_reverse_debounce_frames']) == (.3, 3)
        assert values['drive_collapse_hold_frames'] == 0 and values['drive_start_step'] == 'enabled'
        # Driving ships regeneration and feedforward off; airborne carries the 2026-10-08 tuning.
        regen_max = 30. if condition == 'airborne' else 0.
        assert values['drive_regen_max_current_a'] == regen_max
        assert values['drive_pi_regen'] == ('enabled' if condition == 'airborne' else 'disabled')
        assert values['drive_ff_current_gain_sine_mps2_per_a'] == (.12 if condition == 'airborne' else 0.)
        assert (values['drive_regen_voltage_soft_v'], values['drive_regen_voltage_hard_v']) == (57., 58.)
        assert values['drive_regen_bus_current_limit_a'] == 30. and values['drive_regen_rpm_field'] == 'zero'
        assert (values['drive_collapse_phase_low_square_a'], values['drive_collapse_phase_low_sine_a']) == (-30., -60.)
        native = load_launch_overrides(settings.path, LONGITUDINAL, condition)
        validate_config(native)
        wire = request_data(native, 1, 0)[3:]
        assert wire[WIRE_KEYS.index('min_drive_current_a')] == 0.
        assert wire[WIRE_KEYS.index('motor_waveform')] == 1.
        assert wire[WIRE_KEYS.index('regen_max_current_a')] == regen_max
        assert request_data({**native, 'motor_waveform': 'square'}, 1, 0)[3:][12] == 12.
    airborne = presets.load('airborne')
    # Slow 50 A/s cuts crawled through the square dead zone at speed (P0-5).
    assert airborne['drive_current_slew_down_a_s'] == 400.
    assert airborne['drive_current_limit_a'] == 80.
    # 2026-10-08 airborne RBS tuning: feedforward carries the load, I only the residual.
    assert airborne['drive_control_mode'] == 'torque' and airborne['drive_integrator_limit_a'] == 4.
    assert (airborne['drive_ff_rolling_resistance_mps2'], airborne['drive_ff_speed_damping_per_s']) == (1.05, .12)
    assert (airborne['drive_regen_enter_error_mps'], airborne['drive_regen_exit_error_mps']) == (.5, .3)
    assert airborne['drive_regen_kp_a_per_mps'] == 15. and airborne['drive_reapply_current_slew_a_s'] == 60.
