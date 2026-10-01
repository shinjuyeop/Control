"""User-facing units and the existing actuator parameter contract."""

from dataclasses import dataclass
import math
from .longitudinal import LONGITUDINAL, validate_config

STEERING = 'steering_actuator_node'
GOVERNOR = 'command_governor'
GROUPS = ('조향응답', '조향출력', '구동응답', '구동출력', '구동보호', '수신제한')


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
    # Waveform-specific value: shown and sent only for this motor waveform.
    waveform: str = ''
    # Integer settings where 0 is meaningful (for example "reaction off").
    zero: bool = False


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
    Field('governor_command_timeout_ms', '거버너 명령 유효시간', 'ms', GROUPS[5],
          'input_timeout_ms', '/planning/command의 생성 시각을 검사합니다. 너무 오래되거나 미래 시각인 명령을 중계하지 않습니다.',
          10, node=GOVERNOR),
    Field('steering_command_timeout_ms', '조향 명령 수신 제한', 'ms', GROUPS[5],
          'steer_command_timeout_ms', '/vehicle/command가 이 시간 이상 오래되면 조향 명령을 만료시킵니다.', 10, integer=True),
    Field('steering_can_timeout_ms', '조향 CAN 수신 제한', 'ms', GROUPS[5],
          'can_timeout_ms', 'KEYA CAN frame 수신 단절 기준입니다.', 10, integer=True),
    Field('steering_position_feedback_timeout_ms', '조향 위치 피드백 제한', 'ms', GROUPS[5],
          'feedback_timeout_ms', '위치 피드백 수신 제한입니다. torque에서는 torque 피드백 제한과 작은 값이 적용됩니다.', 10, integer=True),
    Field('steering_torque_feedback_timeout_ms', '조향 토크모드 피드백 제한', 'ms', GROUPS[5],
          'torque_feedback_timeout_ms', '조향 Torque 모드의 위치 피드백·샘플 간격·제어 지연 제한입니다. 구동 STM_A에는 적용되지 않습니다. 제어 주기의 2배 이상, 200 ms 이하.', 10, integer=True),
    Field('steering_vehicle_mode_timeout_ms', '조향용 차량모드 수신 제한', 'ms', GROUPS[5],
          'mode_timeout_ms', '조향 허용 판단에 사용하는 /drive/status의 MANUAL/AUTO 차량모드 수신 단절 기준입니다.', 50, integer=True),
    Field('steering_vehicle_speed_timeout_ms', '조향용 차속 수신 제한', 'ms', GROUPS[5],
          'drive_status_timeout_ms', '조향 허용 판단에 사용하는 차속의 수신 단절 기준입니다.', 10, integer=True),

)
FIELDS += (
    Field('drive_control_mode', '구동 제어 방식', '', GROUPS[2], 'control_mode',
          'S로 저장하고 선택한 프리셋으로 구동 스택을 재시작합니다. Torque 값은 초기 시험 후보입니다.',
          node=LONGITUDINAL, choices=('speed', 'torque', 'external_current')),
    Field('drive_motor_waveform', '구동 모터 파형', '', GROUPS[2], 'motor_waveform',
          'EZkontrol PC 설정의 정류 방식과 같게 맞춥니다. STM_A는 판별하지 못하고 기록만 합니다. '
          '선택한 파형의 최소 전류·feedforward 이득만 보내며 나머지 값은 공통입니다.',
          node=LONGITUDINAL, choices=('sine', 'square')),
    Field('governor_deceleration_slew_mps2', '거버너 감속 slew', 'm/s²', GROUPS[2], 'deceleration_slew_mps2',
          '양수 속도 목표를 낮추는 최대 변화율입니다. 작을수록 천천히 감속합니다. '
          '0.5이면 5→2 m/s에 6초. 목표 0·브레이크는 즉시 처리합니다. 0이면 제한 해제.',
          0.1, node=GOVERNOR),
    Field('governor_acceleration_slew_mps2', '거버너 가속 slew', 'm/s²', GROUPS[2], 'acceleration_slew_mps2',
          'Speed 전용 목표속도 상승률입니다. 1이면 0→5 m/s에 5초. '
          '첫 명령과 수신 재개 시 0부터 시작합니다. 0이면 제한 해제. 저장 후 스택 재시작에 반영됩니다.',
          0.1, node=GOVERNOR),
    Field('drive_speed_kp_a_per_mps', '구동 속도 P 게인', 'A/(m/s)', GROUPS[2], 'speed_kp_a_per_mps',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 10, node=LONGITUDINAL),
    Field('drive_speed_ki_a_per_mps_s', '구동 속도 I 게인', 'A/(m/s*s)', GROUPS[2], 'speed_ki_a_per_mps_s',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 1, node=LONGITUDINAL),
    Field('drive_speed_deadband_mps', '구동 속도 데드밴드', 'm/s', GROUPS[2], 'speed_deadband_mps',
          '구동 Torque 모드 전용입니다. MCAP 기반 초기 시험 후보이며 S 저장 후 구동 스택을 재시작하고 현재값을 확인합니다.', 0.01, node=LONGITUDINAL),
    Field('drive_coast_release_error_mps', '구동 타력 해제 오차 (구형 펌웨어)', 'm/s', GROUPS[2], 'speed_hysteresis_mps',
          '이전 타력 래치 제어용입니다. 연속 PI 펌웨어는 이 값을 사용하지 않으며 기존 설정 호환을 위해 보존합니다.', 0.01, node=LONGITUDINAL),
    Field('drive_ff_current_gain_square_mps2_per_a', '구동 FF 전류 이득 (방형파)', '(m/s²)/A', GROUPS[2],
          'ff_current_gain_square_mps2_per_a',
          '식별 플랜트 a = 이득·I − 구름저항 − 감쇠·v의 이득입니다. 0이면 feedforward를 끕니다. '
          '같은 상전류의 토크가 파형마다 달라 파형별로 따로 식별합니다.', 0.0005,
          node=LONGITUDINAL, waveform='square'),
    Field('drive_ff_current_gain_sine_mps2_per_a', '구동 FF 전류 이득 (사인파)', '(m/s²)/A', GROUPS[2],
          'ff_current_gain_sine_mps2_per_a',
          '식별 플랜트 a = 이득·I − 구름저항 − 감쇠·v의 이득입니다. 0이면 feedforward를 끕니다. '
          '사인파(FOC) 일정 전류 가속 시험으로 식별하기 전에는 0으로 둡니다.', 0.0005,
          node=LONGITUDINAL, waveform='sine'),
    Field('drive_ff_rolling_resistance_mps2', '구동 FF 구름저항', 'm/s²', GROUPS[2], 'ff_rolling_resistance_mps2',
          '파형 공통 기계 저항입니다. 전류 이득이 0이면 쓰지 않습니다.', 0.01, node=LONGITUDINAL),
    Field('drive_ff_speed_damping_per_s', '구동 FF 속도 감쇠', '1/s', GROUPS[2], 'ff_speed_damping_per_s',
          '파형 공통 속도 비례 저항입니다. 전류 이득이 0이면 쓰지 않습니다.', 0.001, node=LONGITUDINAL),
    Field('drive_ff_accel_limit_mps2', '구동 FF 가속 항 한도', 'm/s²', GROUPS[2], 'ff_accel_limit_mps2',
          '목표속도 변화율에서 가져오는 가속도 항의 크기 한도입니다. 목표 계단이 1/dt로 커지지 않게 합니다.',
          0.1, node=LONGITUDINAL),
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
    Field('drive_min_current_square_a', '회전 중 최소 구동 전류 (방형파)', 'A', GROUPS[4], 'min_drive_current_square_a',
          '굴러가는 동안 MANUAL 전진·AUTO Torque·외부 전류의 최종 명령을 이 값 이상으로 유지합니다. '
          '방형파 EZ의 0 A·저전류 재인가 붕괴 회피용이며 0이면 끕니다. 2026-09-29/30 공중 시험 기준 12 A.',
          0.5, node=LONGITUDINAL, waveform='square'),
    Field('drive_min_current_sine_a', '회전 중 최소 구동 전류 (사인파)', 'A', GROUPS[4], 'min_drive_current_sine_a',
          '사인파(FOC)는 0 A도 능동 제어라 기본 0(꺼짐)입니다. FOC에서도 붕괴가 감지될 때만 시험적으로 올립니다.',
          0.5, node=LONGITUDINAL, waveform='sine'),
    Field('drive_start_step', '출발 전류 스텝', '', GROUPS[4], 'start_step',
          '정지 상태에서 0과 최소 전류 사이의 명령을 최소 전류로 올려 데드존을 천천히 지나지 않게 합니다. '
          '최소 전류가 0이면 효과가 없습니다.',
          node=LONGITUDINAL, choices=('enabled', 'disabled')),
    Field('drive_hold_on_speed_mps', '최소 전류 유지 시작 속도', 'm/s', GROUPS[4], 'hold_on_speed_mps',
          '전진·유효 피드백에서 이 속도 이상이면 회전 중 최소 전류를 켭니다. 0.35 m/s ≈ 65 motor rpm.',
          0.05, node=LONGITUDINAL),
    Field('drive_hold_off_speed_mps', '최소 전류 유지 해제 속도', 'm/s', GROUPS[4], 'hold_off_speed_mps',
          '이 속도 이하면 끕니다. 시작 속도보다 작아야 하며 그 사이가 히스테리시스입니다.',
          0.05, node=LONGITUDINAL),
    Field('drive_reverse_speed_mps', '역회전 판정 속도', 'm/s', GROUPS[4], 'reverse_speed_mps',
          'AUTO Torque/외부 전류에서 측정 속도가 이 값보다 더 음수일 때 역회전 후보로 셉니다. 목표 음수는 즉시 정지합니다.',
          0.05, node=LONGITUDINAL),
    Field('drive_reverse_debounce_frames', '역회전 확정 프레임', '50 ms', GROUPS[4], 'reverse_debounce_frames',
          '연속 프레임 수가 이 값에 도달해야 HALTED합니다. 그 전에는 RUN을 유지하고 0 A를 보냅니다.',
          1, node=LONGITUDINAL, integer=True),
    Field('drive_collapse_hold_frames', '붕괴 감지 반응 프레임', '50 ms', GROUPS[4], 'collapse_hold_frames',
          'STM_A가 상전류·RPM 붕괴를 감지하면 AUTO에서 이 프레임 동안 최소 전류(없으면 0 A)에 머물고 PI를 멈춥니다. '
          '0이면 관측·집계만 합니다.',
          1, node=LONGITUDINAL, integer=True, zero=True),
)
BY_KEY = {f.key: f for f in FIELDS}


def applies_to_drive_mode(field, mode, waveform=None):
    if field.waveform and waveform is not None and field.waveform != waveform:
        return False
    if field.key == 'governor_acceleration_slew_mps2':
        return mode == 'speed'
    if field.key == 'governor_deceleration_slew_mps2':
        return mode in ('speed', 'torque')
    if field.node != LONGITUDINAL:
        return True
    # MANUAL applies the rolling floor whatever the calibration mode is.
    if field.native in ('control_mode', 'max_drive_current_a', 'motor_rpm_limit', 'motor_waveform',
                        'min_drive_current_square_a', 'min_drive_current_sine_a', 'start_step',
                        'hold_on_speed_mps', 'hold_off_speed_mps'):
        return True
    if field.native in ('reverse_speed_mps', 'reverse_debounce_frames', 'collapse_hold_frames'):
        return mode in ('torque', 'external_current')
    # Retain the old hysteresis setting in presets/ABI; continuous PI does not use it.
    return mode == 'torque' and field.native != 'speed_hysteresis_mps'


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
    if node == LONGITUDINAL:
        # Readback names only the active waveform's slot (and v1~v3 fewer fields).
        return {f.key: native[f.native] for f in FIELDS if f.node == node and f.native in native}
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
            if f.integer and not (0 if f.zero else 1) <= value <= 2147483647:
                raise ValueError(f'{f.label}: {0 if f.zero else 1}~2147483647 사이 정수가 필요합니다.')
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
    if not math.isfinite(governor['deceleration_slew_mps2']) or governor['deceleration_slew_mps2'] < 0:
        raise ValueError('거버너 감속 slew는 0 이상의 유한한 값이어야 합니다. 0이면 제한 해제입니다.')
    if not math.isfinite(governor['acceleration_slew_mps2']) or governor['acceleration_slew_mps2'] < 0:
        raise ValueError('거버너 가속 slew는 0 이상의 유한한 값이어야 합니다. 0이면 제한 해제입니다.')


def range_hint(field, values, context):
    """Controller acceptance bounds, not experimentally verified tuning guidance."""
    if field.node == LONGITUDINAL:
        return {
            'drive_control_mode': 'speed / torque / external_current; 저장 후 재시작',
            'drive_speed_kp_a_per_mps': '0 초과 ~ 1000', 'drive_speed_ki_a_per_mps_s': '0~200',
            'drive_current_limit_a': '0 초과 ~ 480', 'drive_integrator_limit_a': '0~최대 구동 전류',
            'drive_current_slew_up_a_s': '0 초과 ~ 9600', 'drive_current_slew_down_a_s': '0 초과 ~ 9600',
            'drive_speed_deadband_mps': '0~0.2', 'drive_coast_release_error_mps': 'deadband 초과 ~ 0.5',
            'drive_reapply_current_band_a': '0~최대 구동 전류', 'drive_reapply_current_slew_a_s': '0 초과 ~ 상승률',
            'drive_motor_speed_limit_rpm': '100~4500',
            'drive_motor_waveform': 'sine / square; EZ 설정과 같게',
            'drive_min_current_square_a': '0~30, 사용 파형이면 구동 전류 한도 이하',
            'drive_min_current_sine_a': '0~30, 사용 파형이면 구동 전류 한도 이하',
            'drive_start_step': 'enabled / disabled',
            'drive_hold_on_speed_mps': '0.1~2', 'drive_hold_off_speed_mps': '0 ~ 시작 속도 미만',
            'drive_reverse_speed_mps': '0.05~1', 'drive_reverse_debounce_frames': '1~10',
            'drive_collapse_hold_frames': '0~40',
            'drive_ff_current_gain_square_mps2_per_a': '0 또는 0.0005~0.2',
            'drive_ff_current_gain_sine_mps2_per_a': '0 또는 0.0005~0.2',
            'drive_ff_rolling_resistance_mps2': '0~2', 'drive_ff_speed_damping_per_s': '0~1',
            'drive_ff_accel_limit_mps2': '0~5',
        }[field.key]
    peak = values['steering_peak_current_limit_a']
    period = context['torque_control_period_ms']
    hints = {
        'governor_deceleration_slew_mps2': '0 이상 · 0이면 제한 해제',
        'governor_acceleration_slew_mps2': '0 이상 · 0이면 제한 해제',
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
