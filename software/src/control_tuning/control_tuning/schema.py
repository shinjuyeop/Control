"""User-facing units and the existing actuator parameter contract."""

from dataclasses import dataclass
import math
from .longitudinal import LONGITUDINAL, validate_config

STEERING = 'steering_actuator_node'
GOVERNOR = 'command_governor'
GROUPS = ('조향응답', '조향출력', '구동응답', '구동출력', '수신제한')


@dataclass(frozen=True)
class Field:
    key: str
    label: str
    unit: str
    group: str
    native: str
    help: str
    step: float = 0.0
    node: str = STEERING
    integer: bool = False
    choices: tuple = ()


FIELDS = (
    Field('steering_control_mode', '조향 제어 방식', '', GROUPS[0],
          'actuation_mode', '재시작 필요. KEYA 내부 0020도 torque=2 / position=3으로 별도 설정해야 합니다.',
          choices=('torque', 'position')),
    Field('steering_kp_a_per_deg', '조향 P 게인', 'A/deg', GROUPS[0],
          'torque_full_scale_error_deg',
          '높일수록 목표각을 더 강하게 따라갑니다. 최대 전류 명령을 바꿔도 P 게인은 유지됩니다.', 1.6),
    Field('steering_kd_a_per_deg_s', '조향 D 게인', 'A/(deg/s)', GROUPS[0],
          'torque_kd', '높일수록 조향 움직임을 더 강하게 억제합니다. 너무 크면 목표각을 따라가는 속도가 느려질 수 있습니다.',
          1500 * math.pi / 180 * 0.0016 * 0.05),
    Field('steering_settle_error_deg', '조향 정착 오차', 'deg', GROUPS[0],
          'torque_settle_angle_deg', '각도 오차와 조향속도가 모두 판정 기준 미만이면 목표에 도달한 것으로 보고 전류 명령을 0으로 합니다.', 0.01),
    Field('steering_settle_rate_deg_s', '조향 정착 각속도', 'deg/s', GROUPS[0],
          'torque_settle_rate_deg_s', '목표각 변화율과 필터 적용 전·후 실제 조향속도가 모두 이 값보다 작아야 합니다.', 0.1),
    Field('steering_rate_filter_hz', '조향 각속도 필터', 'Hz', GROUPS[0],
          'torque_rate_filter_hz', '낮출수록 속도 측정 잡음을 더 줄이지만, 실제 움직임을 늦게 반영합니다.', 0.5),
    Field('steering_peak_current_limit_a', '조향 최대 전류 한도', 'A', GROUPS[1],
          'torque_peak_limit_raw', '순간적으로 허용할 전류 명령의 상한입니다. 실제 측정 전류나 모터 정격을 설정하는 값은 아닙니다.', 0.1),
    Field('steering_sustained_current_limit_a', '조향 지속 전류 한도', 'A', GROUPS[1],
          'torque_sustained_limit_raw', '고출력 허용 시간을 모두 사용한 뒤 적용할 전류 명령의 상한입니다.', 0.1),
    Field('steering_peak_duration_ms', '조향 고출력 허용시간', 'ms', GROUPS[1],
          'torque_peak_duration_ms', '지속 전류 한도를 넘긴 시간을 누적합니다. 목표각을 바꿔도 사용한 시간은 초기화되지 않습니다.', 100, integer=True),
    Field('steering_peak_recovery_ms', '조향 고출력 회복대기', 'ms', GROUPS[1],
          'torque_peak_recovery_ms', '지속 전류 한도의 50% 이하 출력을 이 시간 동안 유지하면 고출력을 다시 사용할 수 있습니다.', 100, integer=True),
    Field('steering_angle_margin_deg', '조향 한계각 여유', 'deg', GROUPS[1],
          'torque_soft_limit_margin_deg', '측정된 조향 범위 안쪽으로 목표를 제한하고, 끝 부근의 바깥 방향 출력을 줄입니다.', 0.1),
    Field('steering_min_vehicle_speed_kph', '조향 허용 최소 차속', 'km/h', GROUPS[1],
          'minimum_steering_speed_kph', '0이면 정지 상태에서도 조향합니다. 최근에 수신한 유효한 차속 피드백은 계속 필요합니다.', 0.1),
    Field('governor_command_timeout_ms', '거버너 명령 유효시간', 'ms', GROUPS[4],
          'input_timeout_ms', '/planning/command의 생성 시각을 검사합니다. 너무 오래되거나 미래 시각인 명령을 중계하지 않습니다.',
          10, node=GOVERNOR),
    Field('steering_command_timeout_ms', '조향 명령 수신 제한', 'ms', GROUPS[4],
          'steer_command_timeout_ms', '/vehicle/command가 이 시간 이상 오래되면 조향 명령을 만료시킵니다.', 10, integer=True),
    Field('steering_can_timeout_ms', '조향 CAN 수신 제한', 'ms', GROUPS[4],
          'can_timeout_ms', 'KEYA CAN frame 수신 단절 기준입니다.', 10, integer=True),
    Field('steering_position_feedback_timeout_ms', '조향 위치 피드백 제한', 'ms', GROUPS[4],
          'feedback_timeout_ms', '위치 피드백 수신 제한입니다. torque에서는 torque 피드백 제한과 작은 값이 적용됩니다.', 10, integer=True),
    Field('steering_torque_feedback_timeout_ms', '조향 토크모드 피드백 제한', 'ms', GROUPS[4],
          'torque_feedback_timeout_ms', '조향 Torque 모드의 위치 피드백·샘플 간격·제어 지연 제한입니다. 구동 STM_A에는 적용되지 않습니다. 제어 주기의 2배 이상, 200 ms 이하.', 10, integer=True),
    Field('steering_vehicle_mode_timeout_ms', '조향용 차량모드 수신 제한', 'ms', GROUPS[4],
          'mode_timeout_ms', '조향 허용 판단에 사용하는 /drive/status의 MANUAL/AUTO 차량모드 수신 단절 기준입니다.', 50, integer=True),
    Field('steering_vehicle_speed_timeout_ms', '조향용 차속 수신 제한', 'ms', GROUPS[4],
          'drive_status_timeout_ms', '조향 허용 판단에 사용하는 차속의 수신 단절 기준입니다.', 10, integer=True),

)
FIELDS += (
    Field('drive_control_mode', '구동 제어 방식', '', GROUPS[2], 'control_mode',
          'S로 저장하고 선택한 프리셋으로 구동 스택을 재시작합니다. Torque 값은 초기 시험 후보입니다.',
          node=LONGITUDINAL, choices=('speed', 'torque')),
    Field('drive_speed_kp_a_per_mps', '구동 속도 P 게인', 'A/(m/s)', GROUPS[2], 'speed_kp_a_per_mps',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 10, node=LONGITUDINAL),
    Field('drive_speed_ki_a_per_mps_s', '구동 속도 I 게인', 'A/(m/s*s)', GROUPS[2], 'speed_ki_a_per_mps_s',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 1, node=LONGITUDINAL),
    Field('drive_speed_deadband_mps', '구동 속도 데드밴드', 'm/s', GROUPS[2], 'speed_deadband_mps',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 0.01, node=LONGITUDINAL),
    Field('drive_coast_release_error_mps', '구동 타력 해제 오차', 'm/s', GROUPS[2], 'speed_hysteresis_mps',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 0.01, node=LONGITUDINAL),
    Field('drive_current_limit_a', '구동 전류 한도', 'A', GROUPS[3], 'max_drive_current_a',
          'Speed/Torque 공통 구동 한도입니다. Torque에서는 PI 상전류 명령 상한, Speed에서는 EZ 내부 속도제어의 전류 한도입니다.', 10, node=LONGITUDINAL),
    Field('drive_integrator_limit_a', '구동 적분 전류 한도', 'A', GROUPS[3], 'integrator_limit_a',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 5, node=LONGITUDINAL),
    Field('drive_current_slew_up_a_s', '구동 전류 상승률', 'A/s', GROUPS[3], 'current_slew_up_a_s',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 100, node=LONGITUDINAL),
    Field('drive_current_slew_down_a_s', '구동 전류 하강률', 'A/s', GROUPS[3], 'current_slew_down_a_s',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 100, node=LONGITUDINAL),
    Field('drive_reapply_current_band_a', '구동 재인가 전류 구간', 'A', GROUPS[3], 'reapply_current_a',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 1, node=LONGITUDINAL),
    Field('drive_reapply_current_slew_a_s', '구동 재인가 전류 상승률', 'A/s', GROUPS[3], 'reapply_slew_a_s',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 20, node=LONGITUDINAL),
    Field('drive_motor_speed_limit_rpm', '구동 모터 회전수 한도', 'rpm', GROUPS[3], 'motor_rpm_limit',
          'Speed/Torque 공통 RPM 한도입니다. Torque에서는 EZ 모터 회전수 상한, Speed에서는 목표 RPM 범위를 제한합니다. 최종 전진/후진 물리 한도는 펌웨어에 별도로 적용됩니다.', 100, node=LONGITUDINAL),
    Field('drive_ff_current_gain_mps2_per_a', '구동 전류당 가속도', 'm/s^2/A', GROUPS[2],
          'ff_current_gain_mps2_per_a',
          '구동 Torque 모드의 feedforward 기울기입니다. 0이면 feedforward를 쓰지 않고 PI만 동작합니다. 2026-09-20 기록 식별값이며 실차 검증값은 아닙니다.',
          0.0001, node=LONGITUDINAL),
    Field('drive_ff_rolling_resistance_mps2', '구동 주행저항 가속도', 'm/s^2', GROUPS[2],
          'ff_rolling_resistance_mps2',
          '정지 상태에서 움직이기 시작하는 데 필요한 감속 성분입니다. feedforward 정상주행 전류 = (이 값 + 속도항) / 전류당 가속도입니다.',
          0.01, node=LONGITUDINAL),
    Field('drive_ff_speed_damping_per_s', '구동 속도 저항 계수', '1/s', GROUPS[2],
          'ff_speed_damping_per_s',
          '차속에 비례하는 주행저항입니다. 높이면 고속에서 feedforward 전류가 더 커집니다.',
          0.001, node=LONGITUDINAL),
    Field('drive_ff_accel_limit_mps2', '구동 feedforward 가속 한도', 'm/s^2', GROUPS[2],
          'ff_accel_limit_mps2',
          '목표 속도 변화율에서 얻는 feedforward 가속 항의 상한입니다. 상위 목표가 급히 바뀌어도 이 값 이상으로 전류를 미리 내지 않습니다. 0이면 가속 항을 쓰지 않습니다.',
          0.1, node=LONGITUDINAL),
)
BY_KEY = {f.key: f for f in FIELDS}
OWNED = {node: {f.native for f in FIELDS if f.node == node} for node in (STEERING, GOVERNOR, LONGITUDINAL)}
# Read these as well so conversions, limits and concurrent edits use actual node values.
CONTEXT_NAMES = (
    'keya_protection_current_a', 'torque_control_period_ms', 'count_scale',
    'equivalent_k1_deg', 'equivalent_k2_deg', 'equivalent_k3_deg',
    'equivalent_k4_deg', 'equivalent_k5_deg', 'command_count_limit',
)
READ_NAMES = {STEERING: tuple(sorted(OWNED[STEERING])) + CONTEXT_NAMES,
              GOVERNOR: tuple(sorted(OWNED[GOVERNOR]))}


def equivalent(a, b):
    if type(a) is bool or type(b) is bool:
        return type(a) is type(b) and a == b
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-10)
    return a == b


def current_per_raw(steering):
    reference = float(steering['keya_protection_current_a'])
    if not math.isfinite(reference) or reference <= 0:
        raise ValueError('KEYA 전류 환산 기준은 양수여야 합니다.')
    return reference * 0.8 / 10000.0


def decode_node(node, native):
    values = {f.key: native[f.native] for f in FIELDS if f.node == node}
    if node == STEERING:
        scale = current_per_raw(native)
        values['steering_kp_a_per_deg'] = native['torque_peak_limit_raw'] * scale / native['torque_full_scale_error_deg']
        values['steering_kd_a_per_deg_s'] = native['torque_kd'] * math.pi / 180.0 * scale
        values['steering_peak_current_limit_a'] = native['torque_peak_limit_raw'] * scale
        values['steering_sustained_current_limit_a'] = native['torque_sustained_limit_raw'] * scale
    return values


def decode(native):
    return {k: v for node, params in native.items() for k, v in decode_node(node, params).items()}


def encode(values, context):
    """Convert amperes to existing ROS parameters; preserve P when changing peak."""
    result = {STEERING: {}, GOVERNOR: {}, LONGITUDINAL: {}}
    for f in FIELDS:
        value = values[f.key]
        if f.choices:
            if value not in f.choices:
                raise ValueError(f'{f.label}: {" / ".join(f.choices)} 중 선택하십시오.')
        else:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f'{f.label}: 유한한 숫자가 필요합니다.')
            if f.integer and int(value) != value:
                raise ValueError(f'{f.label}: 정수를 입력하십시오.')
            if f.integer and not 0 < value <= 2147483647:
                raise ValueError(f'{f.label}: 1~2147483647 사이 정수가 필요합니다.')
            value = int(value) if f.integer else float(value)
        result[f.node][f.native] = value
    s = result[STEERING]
    scale = current_per_raw(context)
    if values['steering_kp_a_per_deg'] <= 0:
        raise ValueError('P 게인은 0보다 커야 합니다.')
    for key, native in (('steering_peak_current_limit_a', 'torque_peak_limit_raw'),
                        ('steering_sustained_current_limit_a', 'torque_sustained_limit_raw')):
        amps = values[key]
        if not 0 < amps <= scale * 10000:
            raise ValueError(f'{BY_KEY[key].label}: 0 초과, {scale * 10000:g} A 이하입니다.')
        s[native] = round(amps / scale)
    # Integer wire quantization is reflected in the converted full-scale error too.
    s['torque_full_scale_error_deg'] = s['torque_peak_limit_raw'] * scale / values['steering_kp_a_per_deg']
    s['torque_kd'] = values['steering_kd_a_per_deg_s'] / scale * 180.0 / math.pi
    validate_native({**context, **s}, result[GOVERNOR])
    validate_config(result[LONGITUDINAL])
    return result


def validate_native(s, governor):
    positive = ('torque_full_scale_error_deg', 'torque_settle_angle_deg',
                'torque_settle_rate_deg_s', 'torque_rate_filter_hz',
                'torque_soft_limit_margin_deg',
                'can_timeout_ms', 'feedback_timeout_ms', 'mode_timeout_ms',
                'drive_status_timeout_ms', 'steer_command_timeout_ms')
    if any(not math.isfinite(s[k]) or s[k] <= 0 for k in positive):
        raise ValueError('각도·주파수·수신 시간 제한은 0보다 큰 유한한 값이어야 합니다.')
    if s['torque_kd'] < 0 or not math.isfinite(s['torque_kd']):
        raise ValueError('D 게인은 0 이상의 유한한 값이어야 합니다.')
    if not math.isfinite(s['minimum_steering_speed_kph']) or s['minimum_steering_speed_kph'] < 0:
        raise ValueError('조향 허용 최소 차속은 0 이상입니다.')
    peak, continuous = s['torque_peak_limit_raw'], s['torque_sustained_limit_raw']
    if not 1 <= continuous <= peak <= 10000:
        raise ValueError('지속 명령은 최대 명령 이하여야 하며, 최소 1 raw에 해당해야 합니다.')
    period = s['torque_control_period_ms']
    if not 10 <= period <= 50:
        raise ValueError('기본 YAML의 torque 제어 주기는 10~50 ms여야 합니다.')
    if not period <= s['torque_peak_duration_ms'] <= 5000:
        raise ValueError(f'고출력 허용 시간은 {period}~5000 ms입니다.')
    if s['torque_peak_recovery_ms'] < s['torque_peak_duration_ms']:
        raise ValueError('회복 시간은 고출력 허용 시간 이상이어야 합니다.')
    if not period * 2 <= s['torque_feedback_timeout_ms'] <= 200:
        raise ValueError(f'Torque 피드백 timeout은 {period * 2}~200 ms입니다.')
    if s['torque_rate_filter_hz'] > 500.0 / period:
        raise ValueError(f'필터 주파수는 {500.0 / period:g} Hz 이하입니다.')
    if s['torque_settle_angle_deg'] >= s['torque_full_scale_error_deg']:
        raise ValueError('목표 도달 오차는 P항이 최대 명령에 도달하는 오차보다 작아야 합니다.')
    bounds = []
    for sign in (-1, 1):
        v = sign * s['command_count_limit'] / s['count_scale']
        bounds.append(sum(s[f'equivalent_k{i}_deg'] * v ** i for i in range(1, 6)))
    if not 0 < s['torque_soft_limit_margin_deg'] < min(-bounds[0], bounds[1]):
        raise ValueError('조향 한계 여유각이 캘리브레이션 범위를 벗어납니다.')
    if not math.isfinite(governor['input_timeout_ms']) or governor['input_timeout_ms'] <= 0:
        raise ValueError('상위 명령 유효시간은 0보다 큰 유한한 값이어야 합니다.')


def range_hint(field, values, context):
    """Controller acceptance bounds, not experimentally verified tuning guidance."""
    if field.node == LONGITUDINAL:
        return {
            'drive_control_mode': 'speed / torque; 저장 후 재시작',
            'drive_speed_kp_a_per_mps': '0 초과 ~ 1000', 'drive_speed_ki_a_per_mps_s': '0~200',
            'drive_current_limit_a': '0 초과 ~ 480', 'drive_integrator_limit_a': '0~최대 구동 전류',
            'drive_current_slew_up_a_s': '0 초과 ~ 9600', 'drive_current_slew_down_a_s': '0 초과 ~ 9600',
            'drive_speed_deadband_mps': '0~0.2', 'drive_coast_release_error_mps': 'deadband 초과 ~ 0.5',
            'drive_reapply_current_band_a': '0~최대 구동 전류', 'drive_reapply_current_slew_a_s': '0 초과 ~ 상승률',
            'drive_motor_speed_limit_rpm': '100~4500',
            'drive_ff_current_gain_mps2_per_a': '0(사용 안 함) 또는 0.0005~0.05',
            'drive_ff_rolling_resistance_mps2': '0~2', 'drive_ff_speed_damping_per_s': '0~1',
            'drive_ff_accel_limit_mps2': '0~5',
        }[field.key]
    peak = values['steering_peak_current_limit_a']
    period = context['torque_control_period_ms']
    hints = {
        'steering_kp_a_per_deg': f'0 초과, {peak / values["steering_settle_error_deg"]:g} 미만' if values['steering_settle_error_deg'] > 0 else '0 초과',
        'steering_kd_a_per_deg_s': '0 이상 · 코드상 고정 상한 없음',
        'steering_peak_current_limit_a': f'지속 명령 이상, {current_per_raw(context) * 10000:g} 이하',
        'steering_sustained_current_limit_a': f'{current_per_raw(context):g} 이상, 최대 명령 이하',
        'steering_peak_duration_ms': f'{period}~5000',
        'steering_peak_recovery_ms': '고출력 허용 시간 이상',
        'steering_rate_filter_hz': f'0 초과, {500 / period:g} 이하',
        'steering_settle_error_deg': f'0 초과, {peak / values["steering_kp_a_per_deg"]:g} 미만' if values['steering_kp_a_per_deg'] > 0 else '0 초과',
        'steering_settle_rate_deg_s': '0 초과',
        'steering_angle_margin_deg': '0 초과, 측정된 좌우 한계각 안쪽',
        'steering_torque_feedback_timeout_ms': f'{period * 2}~200',
        'steering_min_vehicle_speed_kph': '0 이상',
        'steering_control_mode': 'Torque / Position',
    }
    return hints.get(field.key, '0 초과')


def format_value(value):
    if value is None:
        return '—'
    if isinstance(value, str):
        return {'torque': 'Torque', 'position': 'Position'}.get(value, value)
    return f'{value:.5g}'
