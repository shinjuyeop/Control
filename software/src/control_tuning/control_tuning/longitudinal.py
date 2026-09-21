"""STM_A longitudinal ABI v1: named host representation of fixed Float32 arrays.

Keep field order in sync with longitudinal_controller.h and docs/INTERFACES.md.
Configuration is RAM-only and conditional on the STM_A revision and stop gate.
"""
import math
import struct

from .naming import canonical_values

LONGITUDINAL = 'stm_a_longitudinal'
CONFIG_TOPIC = '/drive/longitudinal_config'
STATUS_TOPIC = '/drive/longitudinal_status'
CONFIG_KEYS = (
    'control_mode', 'speed_kp_a_per_mps', 'speed_ki_a_per_mps_s',
    'max_drive_current_a', 'integrator_limit_a', 'current_slew_up_a_s',
    'current_slew_down_a_s', 'speed_deadband_mps', 'speed_hysteresis_mps',
    'reapply_current_a', 'reapply_slew_a_s', 'motor_rpm_limit',
)
STATE_KEYS = ('speed_error_mps', 'p_current_a', 'i_current_a', 'unsaturated_current_a',
              'current_command_a', 'saturated', 'anti_windup', 'coasting',
              'feedback_valid', 'inhibit')
DEFAULT = dict(zip(CONFIG_KEYS, ('speed', 180., 12., 480., 120., 2400., 4800.,
                               .05, .12, 20., 200., 4500.)))
AIRBORNE = dict(zip(CONFIG_KEYS, ('torque', 15., 2., 20., 6., 120., 400.,
                                .05, .12, 5., 40., 600.)))
RESULTS = ('대기', '적용 완료', '잘못된 설정', 'MANUAL 정지·무전류 조건 불충족',
           '다른 설정이 먼저 적용됨', '다른 요청 처리 중')


def validate_config(config):
    if config['control_mode'] not in ('speed', 'torque'):
        raise ValueError('종방향 control_mode: speed / torque가 필요합니다.')
    for key in CONFIG_KEYS[1:]:
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
        if not (lo <= config[key] <= hi if key in inclusive else lo < config[key] <= hi):
            raise ValueError(f'{key}: {lo}~{hi} 범위를 확인하세요.')


def config_wire(config):
    validate_config(config)
    return [float(config['control_mode'] == 'torque'), *[float(config[k]) for k in CONFIG_KEYS[1:]]]


def request_data(config, request_id, revision):
    return [1., float(request_id), float(revision), *config_wire(config)]


def decode_status(data):
    if len(data) != 27 or data[0] != 1 or not all(math.isfinite(x) for x in data):
        raise ValueError('STM_A longitudinal status ABI v1이 필요합니다.')
    if data[5] not in (0, 1) or data[4] not in (0, 1) or data[3] not in range(len(RESULTS)):
        raise ValueError('STM_A longitudinal status enum 오류')
    config = dict(zip(CONFIG_KEYS, ['torque' if data[5] else 'speed', *data[6:17]]))
    # Float32 readback is compared in wire precision; never claim send == apply.
    return dict(revision=int(data[1]), request_id=int(data[2]), result=int(data[3]),
                source='HOST_OVERRIDE' if data[4] else 'DEFAULT', config=config,
                display_values=canonical_values(config),
                **dict(zip(STATE_KEYS, data[17:])))


def wire_equal(config, other):
    def pack(c):
        return struct.pack('<12f', float(c['control_mode'] == 'torque'),
                           *[c[k] for k in CONFIG_KEYS[1:]])
    return pack(config) == pack(other)
