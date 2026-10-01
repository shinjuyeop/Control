"""STM_A longitudinal ABI v4: named host representation of fixed Float32 arrays.

Keep field order in sync with longitudinal_controller.h and docs/INTERFACES.md.
Configuration is RAM-only and conditional on the STM_A revision and stop gate.
The host keeps square/sine values side by side; STM_A receives the value for the
declared motor waveform in one slot and only echoes the declaration.
"""
import math
import struct

from .naming import canonical_values

VERSION = 4
MODES = ('speed', 'torque', 'external_current')
WAVEFORMS = ('square', 'sine')
SWITCH = ('disabled', 'enabled')
LONGITUDINAL = 'stm_a_longitudinal'
CONFIG_TOPIC = '/drive/longitudinal_config'
STATUS_TOPIC = '/drive/longitudinal_status'
# STM_A wire order after version/request_id/revision.
WIRE_KEYS = (
    'control_mode', 'speed_kp_a_per_mps', 'speed_ki_a_per_mps_s',
    'max_drive_current_a', 'integrator_limit_a', 'current_slew_up_a_s',
    'current_slew_down_a_s', 'speed_deadband_mps', 'speed_hysteresis_mps',
    'reapply_current_a', 'reapply_slew_a_s', 'motor_rpm_limit', 'min_drive_current_a',
    'motor_waveform', 'start_step', 'hold_on_speed_mps', 'hold_off_speed_mps',
    'reverse_speed_mps', 'reverse_debounce_frames', 'collapse_hold_frames',
    'ff_current_gain_mps2_per_a', 'ff_rolling_resistance_mps2', 'ff_speed_damping_per_s',
    'ff_accel_limit_mps2',
)
# v1..v3 readbacks: slot 12 was the coast-only floor (v3 only).
LEGACY_KEYS = WIRE_KEYS[:12] + ('coast_current_floor_a',)
# Values that depend on the EZ commutation, stored per waveform (WAVEFORMS order).
SPECIFIC = {
    'min_drive_current_a': ('min_drive_current_square_a', 'min_drive_current_sine_a'),
    'ff_current_gain_mps2_per_a': ('ff_current_gain_square_mps2_per_a',
                                   'ff_current_gain_sine_mps2_per_a'),
}
CONFIG_KEYS = tuple(name for key in WIRE_KEYS for name in SPECIFIC.get(key, (key,)))
CHOICES = {'control_mode': MODES, 'motor_waveform': WAVEFORMS, 'start_step': SWITCH}
INTEGER_KEYS = ('reverse_debounce_frames', 'collapse_hold_frames')
MIN_CURRENT_CAP_A = 30.
STATE_KEYS = (
    'speed_error_mps', 'p_current_a', 'i_current_a', 'ff_current_a', 'unsaturated_current_a',
    'current_command_a', 'saturated', 'anti_windup', 'coasting', 'feedback_valid', 'inhibit',
    'drive_path', 'output_current_a', 'rolling_hold', 'floor_applied', 'start_stepped',
    'reverse_frames', 'collapse_events', 'collapse_flags', 'collapse_age_s',
    'collapse_command_a', 'collapse_phase_current_a', 'collapse_bus_current_a',
    'collapse_rpm', 'collapse_previous_rpm', 'collapse_path', 'collapse_hold',
    'collapse_reaction_frames',
)
LEGACY_STATE_KEYS = ('speed_error_mps', 'p_current_a', 'i_current_a', 'unsaturated_current_a',
                     'current_command_a', 'saturated', 'anti_windup', 'coasting',
                     'feedback_valid', 'inhibit')
PATHS = ('HALT', 'MANUAL', 'SPEED', 'TORQUE', 'EXTERNAL', 'REGEN', 'ZERO')
COLLAPSE_FLAGS = {1: 'rpm', 2: 'phase_high', 4: 'phase_low', 8: 'bus'}
# Sine (FOC) is the basis: no floor and no feedforward until identified. The
# square-wave floor is kept ready for a controller switched back to square.
PROTECTION = dict(
    min_drive_current_square_a=12., min_drive_current_sine_a=0., motor_waveform='sine',
    start_step='enabled', hold_on_speed_mps=.35, hold_off_speed_mps=.2,
    reverse_speed_mps=.3, reverse_debounce_frames=3, collapse_hold_frames=0,
    ff_current_gain_square_mps2_per_a=0., ff_current_gain_sine_mps2_per_a=0.,
    ff_rolling_resistance_mps2=0., ff_speed_damping_per_s=0., ff_accel_limit_mps2=0.)
DEFAULT = {**dict(zip(WIRE_KEYS[:12], ('speed', 180., 12., 480., 120., 2400., 4800.,
                                        .05, .12, 20., 200., 4500.))), **PROTECTION}
AIRBORNE = {**dict(zip(WIRE_KEYS[:12], ('torque', 25., 2., 40., 6., 120., 400.,
                                         .05, .12, 5., 20., 4500.))), **PROTECTION}
RESULTS = ('대기', '적용 완료', '잘못된 설정', 'MANUAL 정지·무전류 조건 불충족',
           '다른 설정이 먼저 적용됨', '다른 요청 처리 중')
# Same rules as LONG_COLLAPSE_* in longitudinal_controller.h, for MCAPs recorded
# before the firmware counter existed (square-wave baseline) and for FOC runs.
COLLAPSE_RULES = dict(rpm_min=400., rpm_step=200., phase_margin_a=60., phase_negative_a=30.,
                      bus_margin_a=60., rearm_frames=20)


def collapse_flags(command, phase, bus, rpm, previous_rpm, running, torque_command):
    """Bitmask of one Part I frame, mirroring Long_CollapseFlags."""
    r = COLLAPSE_RULES
    flags = 0
    if not running:
        return 0
    if abs(previous_rpm) >= r['rpm_min'] and (rpm * previous_rpm <= 0 or abs(rpm - previous_rpm) > r['rpm_step']):
        flags |= 1
    if torque_command and command >= 0:
        flags |= (2 if phase > command + r['phase_margin_a'] else 0)
        flags |= (4 if phase < -r['phase_negative_a'] else 0)
        flags |= (8 if bus > command + r['bus_margin_a'] else 0)
    return flags


def resolve(config):
    """Wire values: each waveform-specific slot takes the declared waveform's value.

    Accepts the host form (both waveforms) and the readback form (active only).
    """
    wire = {key: config[key] for key in WIRE_KEYS if key not in SPECIFIC}
    index = WAVEFORMS.index(config['motor_waveform'])
    for key, names in SPECIFIC.items():
        wire[key] = config[names[index]] if names[index] in config else config[key]
    return wire


def validate_config(config):
    for key, choices in CHOICES.items():
        if config[key] not in choices:
            raise ValueError(f'종방향 {key}: {" / ".join(choices)} 중 하나가 필요합니다.')
    for key in CONFIG_KEYS:
        if key in CHOICES:
            continue
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f'{key}: 유한한 숫자가 필요합니다.')
    bounds = {
        'speed_kp_a_per_mps': (0, 1000), 'speed_ki_a_per_mps_s': (0, 200),
        'max_drive_current_a': (0, 480), 'integrator_limit_a': (0, config['max_drive_current_a']),
        'current_slew_up_a_s': (0, 9600), 'current_slew_down_a_s': (0, 9600),
        'speed_deadband_mps': (0, .2), 'speed_hysteresis_mps': (config['speed_deadband_mps'], .5),
        'reapply_current_a': (0, config['max_drive_current_a']),
        'reapply_slew_a_s': (0, config['current_slew_up_a_s']), 'motor_rpm_limit': (100, 4500),
    }
    inclusive = {'speed_ki_a_per_mps_s', 'integrator_limit_a', 'speed_deadband_mps',
                 'reapply_current_a', 'motor_rpm_limit'}
    for key, (lo, hi) in bounds.items():
        if config['control_mode'] != 'torque' and key not in ('max_drive_current_a', 'motor_rpm_limit'):
            continue
        if not (lo <= config[key] <= hi if key in inclusive else lo < config[key] <= hi):
            raise ValueError(f'{key}: {lo}~{hi} 범위를 확인하세요.')
    # Mirrors Long_DecodeRequest: the floor and the reverse debounce also act in
    # MANUAL and outside Torque, so they are checked for every control mode.
    for key in INTEGER_KEYS:
        if int(config[key]) != config[key]:
            raise ValueError(f'{key}: 정수가 필요합니다.')
    wire = resolve(config)
    cap = min(config['max_drive_current_a'], MIN_CURRENT_CAP_A)
    for name in SPECIFIC['min_drive_current_a']:
        if not 0 <= config[name] <= MIN_CURRENT_CAP_A:
            raise ValueError(f'{name}: 0~{MIN_CURRENT_CAP_A:g} A 범위를 확인하세요.')
    if not 0 <= wire['min_drive_current_a'] <= cap:
        raise ValueError(f'회전 중 최소 구동 전류: 0~{cap:g} A (구동 전류 한도 이하)를 확인하세요.')
    checks = {
        'hold_on_speed_mps': .1 <= config['hold_on_speed_mps'] <= 2,
        'hold_off_speed_mps': 0 <= config['hold_off_speed_mps'] < config['hold_on_speed_mps'],
        'reverse_speed_mps': .05 <= config['reverse_speed_mps'] <= 1,
        'reverse_debounce_frames': 1 <= config['reverse_debounce_frames'] <= 10,
        'collapse_hold_frames': 0 <= config['collapse_hold_frames'] <= 40,
        'ff_rolling_resistance_mps2': 0 <= config['ff_rolling_resistance_mps2'] <= 2,
        'ff_speed_damping_per_s': 0 <= config['ff_speed_damping_per_s'] <= 1,
        'ff_accel_limit_mps2': 0 <= config['ff_accel_limit_mps2'] <= 5,
    }
    for name in SPECIFIC['ff_current_gain_mps2_per_a']:
        checks[name] = config[name] == 0 or .0005 <= config[name] <= .2
    for key, ok in checks.items():
        if not ok:
            raise ValueError(f'{key}: 허용 범위를 확인하세요.')


def _number(key, value):
    return float(CHOICES[key].index(value)) if key in CHOICES else float(value)


def config_wire(config, version=VERSION):
    validate_config(config)
    if version != VERSION:
        raise ValueError(f'STM_A longitudinal ABI v{VERSION} 펌웨어가 필요합니다 (현재 v{version}). '
                         'STM_A와 host를 함께 갱신하세요.')
    wire = resolve(config)
    return [_number(key, wire[key]) for key in WIRE_KEYS]


def request_data(config, request_id, revision, version=VERSION):
    return [float(version), float(request_id), float(revision), *config_wire(config, version)]


def status_data(config, state=None, *, revision=1, request_id=1, result=1, source=1):
    """v4 status array as STM_A publishes it; for emulators, tests and tools."""
    state = {'feedback_valid': 1., 'collapse_age_s': -1., **(state or {})}
    return [float(VERSION), float(revision), float(request_id), float(result), float(source),
            *config_wire(config), *[float(state.get(key, 0.)) for key in STATE_KEYS]]


def decode_status(data):
    layouts = {1: (12, LEGACY_STATE_KEYS), 2: (12, LEGACY_STATE_KEYS),
               3: (13, LEGACY_STATE_KEYS), 4: (len(WIRE_KEYS), STATE_KEYS)}
    if (not data or data[0] not in layouts or not all(math.isfinite(x) for x in data) or
            len(data) != 5 + layouts[int(data[0])][0] + len(layouts[int(data[0])][1])):
        raise ValueError('STM_A longitudinal status ABI v1/v2/v3/v4가 필요합니다.')
    version = int(data[0])
    count, state_keys = layouts[version]
    if data[5] not in (range(3) if version >= 2 else range(2)) or data[4] not in (0, 1) or data[3] not in range(len(RESULTS)):
        raise ValueError('STM_A longitudinal status enum 오류')
    raw = list(data[5:5 + count])
    if version == VERSION:
        if raw[13] not in (0, 1) or raw[14] not in (0, 1):
            raise ValueError('STM_A longitudinal status enum 오류')
        wire = dict(zip(WIRE_KEYS, raw))
        wire.update({key: CHOICES[key][int(wire[key])] for key in CHOICES})
        wire.update({key: int(wire[key]) for key in INTEGER_KEYS})
        # Readback names the active waveform's slot; the other one is not on the wire.
        index = WAVEFORMS.index(wire['motor_waveform'])
        config = {}
        for key in WIRE_KEYS:
            config[SPECIFIC[key][index] if key in SPECIFIC else key] = wire[key]
    else:
        config = dict(zip(LEGACY_KEYS, [MODES[int(raw[0])], *raw[1:]]))
        config.setdefault('coast_current_floor_a', 0.)
    # Float32 readback is compared in wire precision; never claim send == apply.
    return dict(version=version, revision=int(data[1]), request_id=int(data[2]), result=int(data[3]),
                source='HOST_OVERRIDE' if data[4] else 'DEFAULT', config=config,
                display_values=canonical_values(config),
                **dict(zip(state_keys, data[5 + count:])))


def wire_equal(config, other):
    def pack(c):
        wire = resolve(c)
        return struct.pack(f'<{len(WIRE_KEYS)}f', *[_number(key, wire[key]) for key in WIRE_KEYS])
    return pack(config) == pack(other)
