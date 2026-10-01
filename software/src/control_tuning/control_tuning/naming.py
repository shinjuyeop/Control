"""Canonical tuning names and lossless adapters for existing presets/MCAPs.

These names describe physical-unit calibration. Node-scoped ROS parameter names
and the STM_A v1 array positions remain their existing transport contracts.
"""

KEY_ALIASES = {
    'actuation_mode': 'steering_control_mode',
    'p_gain_a_per_deg': 'steering_kp_a_per_deg',
    'd_gain_a_per_deg_s': 'steering_kd_a_per_deg_s',
    'settle_error_deg': 'steering_settle_error_deg',
    'settle_rate_deg_s': 'steering_settle_rate_deg_s',
    'rate_filter_cutoff_hz': 'steering_rate_filter_hz',
    'peak_current_command_a': 'steering_peak_current_limit_a',
    'continuous_current_command_a': 'steering_sustained_current_limit_a',
    'peak_budget_ms': 'steering_peak_duration_ms',
    'peak_recovery_hold_ms': 'steering_peak_recovery_ms',
    'angle_limit_margin_deg': 'steering_angle_margin_deg',
    'minimum_vehicle_speed_kph': 'steering_min_vehicle_speed_kph',
    'governor_input_timeout_ms': 'governor_command_timeout_ms',
    'steering_command_timeout_ms': 'steering_command_timeout_ms',
    'can_timeout_ms': 'steering_can_timeout_ms',
    'position_feedback_timeout_ms': 'steering_position_feedback_timeout_ms',
    'torque_feedback_timeout_ms': 'steering_torque_feedback_timeout_ms',
    'mode_timeout_ms': 'steering_vehicle_mode_timeout_ms',
    'drive_status_timeout_ms': 'steering_vehicle_speed_timeout_ms',
    'control_mode': 'drive_control_mode',
    'speed_kp_a_per_mps': 'drive_speed_kp_a_per_mps',
    'speed_ki_a_per_mps_s': 'drive_speed_ki_a_per_mps_s',
    'speed_deadband_mps': 'drive_speed_deadband_mps',
    'speed_hysteresis_mps': 'drive_coast_release_error_mps',
    'coast_current_floor_a': 'drive_coast_current_floor_a',
    'max_drive_current_a': 'drive_current_limit_a',
    'integrator_limit_a': 'drive_integrator_limit_a',
    'current_slew_up_a_s': 'drive_current_slew_up_a_s',
    'current_slew_down_a_s': 'drive_current_slew_down_a_s',
    'reapply_current_a': 'drive_reapply_current_band_a',
    'reapply_slew_a_s': 'drive_reapply_current_slew_a_s',
    'motor_rpm_limit': 'drive_motor_speed_limit_rpm',
    # STM_A ABI v4. The v3 coast floor above keeps its name for old MCAPs; the
    # rolling minimum current is a different rule and is stored per waveform.
    'motor_waveform': 'drive_motor_waveform',
    'min_drive_current_square_a': 'drive_min_current_square_a',
    'min_drive_current_sine_a': 'drive_min_current_sine_a',
    'start_step': 'drive_start_step',
    'hold_on_speed_mps': 'drive_hold_on_speed_mps',
    'hold_off_speed_mps': 'drive_hold_off_speed_mps',
    'reverse_speed_mps': 'drive_reverse_speed_mps',
    'reverse_debounce_frames': 'drive_reverse_debounce_frames',
    'collapse_hold_frames': 'drive_collapse_hold_frames',
    'ff_current_gain_square_mps2_per_a': 'drive_ff_current_gain_square_mps2_per_a',
    'ff_current_gain_sine_mps2_per_a': 'drive_ff_current_gain_sine_mps2_per_a',
    'ff_rolling_resistance_mps2': 'drive_ff_rolling_resistance_mps2',
    'ff_speed_damping_per_s': 'drive_ff_speed_damping_per_s',
    'ff_accel_limit_mps2': 'drive_ff_accel_limit_mps2',
}


def canonical_values(values):
    """Rename supplied values/units, reject conflicting aliases, never fill defaults."""
    if not isinstance(values, dict):
        raise ValueError('제어 설정은 이름과 값의 mapping이어야 합니다.')
    result = {}
    for key, value in values.items():
        name = KEY_ALIASES.get(key, key)
        if name in result and (result[name] != value or
                               (isinstance(result[name], bool) != isinstance(value, bool))):
            raise ValueError(f'{name}: 이전 이름과 새 이름의 값이 서로 다릅니다.')
        result[name] = value
    return result


def canonical_capture(document):
    """Read v1/v2 parameter snapshots without modifying raw node parameters."""
    if not isinstance(document, dict) or document.get('schema_version') not in (1, 2):
        raise ValueError('control_parameters snapshot schema v1/v2가 필요합니다.')
    nodes = {}
    for name, entry in document.get('nodes', {}).items():
        entry = dict(entry)
        if name in ('/steering_actuator_node', '/command_governor', '/stm_a_node'):
            for key in ('display_values', 'display_units'):
                if key in entry:
                    entry[key] = canonical_values(entry[key])
        nodes[name] = entry
    return {**document, 'schema_version': 2, 'nodes': nodes}
