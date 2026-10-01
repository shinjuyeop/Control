"""Read-only MCAP Speed/Torque comparison. Receive-time alignment; no bag writes.

Use identical event windows and target sequences for A/B. Phase-current command
is a LIMIT in native speed mode. GNSS supplies body acceleration when available.
"""
import argparse
import json
import csv
import math

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from control_tuning.longitudinal import (COLLAPSE_PHASE_LOW, COLLAPSE_RULES, STATUS_TOPIC,
                                         collapse_flags, decode_status)
from control_tuning.naming import canonical_capture

TOPICS = ('/drive/status', '/vehicle/command', '/brake/status', '/gnss/fix_velocity',
          '/parameter_events', '/sysid/control_parameters', STATUS_TOPIC)
PI_KEYS = ('speed_error_mps', 'p_current_a', 'i_current_a', 'ff_current_a', 'unsaturated_current_a',
           'current_command_a', 'saturated', 'anti_windup', 'coasting', 'feedback_valid', 'inhibit',
           'drive_path', 'output_current_a', 'rolling_hold', 'floor_applied', 'start_stepped',
           'reverse_frames', 'collapse_events', 'collapse_flags', 'regen_request_a', 'regen_limit_a',
           'regen_speed_factor', 'regen_voltage_factor', 'regen_bus_scale', 'regen_current_a',
           'regen_active', 'pi_regen_mode')


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(TOPICS)))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    rows = {t: [] for t in TOPICS}
    snapshots, parameters, captures = [], {}, []
    for_topic = {
        '/drive/status': ('mode', 'gear', 'speed_mps', 'motor_rpm', 'phase_current_a',
                          'bus_current_a', 'bus_voltage_v', 'commanded_phase_current_a',
                          'commanded_motor_rpm', 'controller_control_mode', 'part1_valid',
                          'part2_valid', 'part1_age_ms', 'stop_request', 'tractive_commanded'),
    }
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic])
        row = {'t': stamp / 1e9}
        if topic in for_topic:
            row.update({key: getattr(msg, key) for key in for_topic[topic]})
        elif topic == '/vehicle/command':
            row.update(target=msg.speed_target_mps, brake=msg.brake_engage, steer=msg.steering_target_rad)
        elif topic == '/brake/status':
            row['brake_actual'] = msg.brake_commanded
        elif topic == '/gnss/fix_velocity':
            row['gnss'] = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        elif topic == STATUS_TOPIC:
            status = decode_status(msg.data)
            # v4 adds feedforward, rolling hold and collapse fields; v1~v3 lack them.
            rows[topic].append({'t': row['t'], **{key: status[key] for key in PI_KEYS if key in status},
                'revision': status['revision']})
            if not snapshots or status['revision'] != snapshots[-1]['revision']:
                snapshots.append({'t': row['t'], **status})
            continue
        elif topic == '/parameter_events':
            for p in list(msg.new_parameters) + list(msg.changed_parameters):
                if any(key in p.name for key in ('accel', 'decel', 'speed')):
                    value = p.value
                    parameters[msg.node + ':' + p.name] = {2: value.integer_value,
                        1: value.bool_value, 3: value.double_value, 4: value.string_value}.get(value.type, str(value))
            continue
        else:
            captures.append(canonical_capture(json.loads(msg.data)))
            parameters['control_parameters_snapshot_present'] = True
            continue
        rows[topic].append(row)
    arrays = {topic: {key: np.array([row[key] for row in values]) for key in values[0]}
              for topic, values in rows.items() if values}
    if '/drive/status' not in arrays or '/vehicle/command' not in arrays:
        raise ValueError('MCAP needs /drive/status and /vehicle/command')
    drive = arrays['/drive/status']
    for topic in TOPICS[1:4]:
        if topic not in arrays:
            continue
        data = arrays[topic]
        index = np.searchsorted(data['t'], drive['t'], side='right') - 1
        valid = (index >= 0) & (drive['t'] - data['t'][np.maximum(index, 0)] < .25)
        for key, values in data.items():
            if key != 't':
                drive[key] = np.where(valid, values[np.maximum(index, 0)], np.nan)
    return drive, dict(calibration=snapshots, parameters=parameters, parameter_captures=captures,
                       pi_samples=rows[STATUS_TOPIC])


def phase_low_of(calibration):
    """Collapse phase-low threshold the firmware used (v5) or its waveform default."""
    for snapshot in reversed(calibration or []):
        config = snapshot.get('config', {})
        for key in ('collapse_phase_low_sine_a', 'collapse_phase_low_square_a'):
            if key in config:
                return config[key]
        if 'motor_waveform' in config:
            return COLLAPSE_PHASE_LOW[config['motor_waveform']]
    return COLLAPSE_PHASE_LOW['square']  # recorded before v4: square-wave EZ


def summarize(d, start=0., end=None, target=None, phase_low=COLLAPSE_PHASE_LOW['square']):
    auto = (d['mode'] == 1) & (d['target'] > .1)
    if not auto.any():
        raise ValueError('No AUTO positive-target samples')
    origin = d['t'][auto][0]
    valid = ((d['mode'] == 1) & d['part1_valid'] & d['part2_valid'] &
             ~d['stop_request'] & (d['brake'] == 0) & np.isfinite(d['target']))
    if 'brake_actual' in d:
        valid &= d['brake_actual'] == 0
    valid &= d['t'] >= origin + start
    if end is not None:
        valid &= d['t'] <= origin + end
    # Resample once at 10 Hz: duplicate ROS publishes are not independent CAN measurements.
    grid = np.arange(origin + start, min(origin + end if end is not None else d['t'][-1], d['t'][-1]), .1)
    idx = np.maximum(0, np.searchsorted(d['t'], grid, side='right') - 1)
    ok = valid[idx] & (grid - d['t'][idx] < .1)
    speed_key = 'gnss' if 'gnss' in d else 'speed_mps'
    speed = d[speed_key][idx]; current = d['phase_current_a'][idx]
    ok &= np.isfinite(speed)
    if not ok.any():
        raise ValueError('No fresh feedback-valid brake-off AUTO samples in window')
    error = speed - d['target'][idx]
    q = lambda x: np.quantile(x, [.5, .95, 1]).tolist()
    result = dict(origin_epoch_s=float(origin), window_s=[start, end],
                  acceleration_source=speed_key, sample_count=int(ok.sum()),
                  speed_mps_median_p95_max=q(speed[ok]),
                  phase_a_median_p95_max=q(current[ok]),
                  phase_a_min=float(current[ok].min()), phase_a_rms=float(np.sqrt(np.mean(current[ok]**2))),
                  bus_a_min_median_max=np.quantile(d['bus_current_a'][idx][ok], [0,.5,1]).tolist(),
                  error_rmse_mps=float(np.sqrt(np.mean(error[ok]**2))),
                  max_above_instantaneous_target_mps=float(max(0,error[ok].max())),
                  max_below_instantaneous_target_mps=float(max(0,-error[ok].min())),
                  feedback_modes=np.unique(d['controller_control_mode'][idx][ok]).tolist())
    consecutive = np.convolve(ok.astype(int), np.ones(6, dtype=int), 'valid') == 6
    acceleration = (speed[5:] - speed[:-5]) / .5
    good = consecutive & np.isfinite(acceleration)
    if good.any():
        result['acceleration_halfsec_mean_p95_max'] = [float(acceleration[good].mean()),
                                                      *np.quantile(acceleration[good],[.95,1]).tolist()]
    if speed_key == 'gnss':
        disagreement = abs(d['speed_mps'][idx] - speed)
        result['rpm_gnss_difference_abs_p95_max'] = np.quantile(disagreement[ok],[.95,1]).tolist()
        straight = ok & (abs(d['steer'][idx]) < math.radians(3)) & (disagreement < .5)
        fit_ok = (np.convolve(straight.astype(int), np.ones(6, dtype=int), 'valid') == 6) & good
        averaged_current = np.convolve(current, np.ones(6)/6, 'valid')
        fit_ok &= (averaged_current > 20) & (acceleration > .2)
        result['plant_samples'] = int(fit_ok.sum())
        if fit_ok.sum() >= 3:
            a, i = acceleration[fit_ok], averaged_current[fit_ok]
            gain, offset = np.linalg.lstsq(np.column_stack((i, np.ones(len(i)))), a, rcond=None)[0]
            result['net_acceleration_per_amp_p10_median_p90'] = np.quantile(a/i,[.1,.5,.9]).tolist()
            result['exploratory_fit_accel_equals_gain_current_plus_offset'] = [float(gain),float(offset)]
    finish = origin + end if end is not None else d['t'][-1]
    result['collapse'] = collapse_summary(d, origin + start, finish, phase_low)
    result['regen'] = regen_summary(d, origin + start, finish)
    # Reapplication/coast from actual current only, not speed-mode current limit.
    coast = ok & (abs(current) < 5)
    result['coast_seconds_below_5a'] = float(coast.sum() * .1)
    reapplications, was_coasting = 0, False
    for valid, is_coast, amps in zip(ok, coast, current):
        if not valid:
            was_coasting = False
        elif is_coast:
            was_coasting = True
        elif was_coasting and amps > 20:
            reapplications += 1
            was_coasting = False
    result['current_reapplications_above_20a'] = reapplications
    if target is not None:
        v0 = float(speed[ok][0]); delta = target - v0
        first = lambda mask: float(grid[np.flatnonzero(mask)[0]] - (origin+start)) if mask.any() else None
        sign = 1 if delta >= 0 else -1
        t10 = first(ok & (sign*(speed-v0) >= abs(delta)*.1))
        t90 = first(ok & (sign*(speed-v0) >= abs(delta)*.9))
        result['rise_10_90_s'] = t90-t10 if t90 is not None and t10 is not None else None
        result['target_first_crossing_s'] = first(ok & (sign*(speed-target) >= 0))
        result['target_overshoot_mps'] = float(max(0,(sign*(speed[ok]-target)).max()))
        inside = ok & (abs(speed-target) <= .2)
        # Settled only if all remaining samples are valid/in-band for >=2s.
        bad = np.flatnonzero(~inside); after = int(bad[-1]+1) if len(bad) else 0
        result['settling_within_0p2_mps_s'] = (float(grid[after]-(origin+start))
            if after < len(grid) and len(grid)-after >= 20 else None)
    return result


def part1_frames(d, begin, finish):
    """One /drive/status sample per 50 ms Part I frame inside the window."""
    window = (d['t'] >= begin) & (d['t'] <= finish)
    fresh = np.r_[True, np.diff(d['part1_age_ms']) < 0] & window & d['part1_valid'].astype(bool)
    return np.flatnonzero(fresh)


def collapse_summary(d, begin, finish, phase_low=COLLAPSE_PHASE_LOW['square']):
    """EZ estimator collapses with the STM_A detector rules, one count per episode.

    Works on any MCAP: square-wave logs predate the firmware counter, and the
    same rules then compare square and sine (FOC) runs on equal terms.
    """
    if 'tractive_commanded' not in d or not ((d['t'] >= begin) & (d['t'] <= finish)).any():
        return None
    index = part1_frames(d, begin, finish)
    events, frames, quiet, rolling, first_flags = 0, 0, COLLAPSE_RULES['rearm_frames'], 0, []
    command = d['commanded_phase_current_a']
    for before, previous, current in zip(index[:-2], index[1:-1], index[2:]):
        rpm, prior = float(d['motor_rpm'][current]), float(d['motor_rpm'][previous])
        running = bool(d['tractive_commanded'][current])
        rolling += running and abs(prior) >= COLLAPSE_RULES['rpm_min']
        flags = collapse_flags(float(command[current]), float(command[previous]),
                               float(d['phase_current_a'][current]), float(d['bus_current_a'][current]),
                               rpm, prior, running, running and d['controller_control_mode'][current] == 0,
                               phase_low)
        if not flags:
            quiet += 1
            continue
        frames += 1
        if quiet >= COLLAPSE_RULES['rearm_frames']:
            events += 1
            first_flags.append(int(flags))
        quiet = 0
    minutes = rolling * .05 / 60
    return dict(events=events, anomalous_frames=frames, rolling_minutes=round(minutes, 3),
                events_per_rolling_minute=round(events / minutes, 3) if minutes else None,
                first_frame_flags=first_flags, phase_low_a=phase_low, rules=COLLAPSE_RULES)


def regen_summary(d, begin, finish):
    """AUTO frames that commanded negative phase current (LT, external or PI regen)."""
    index = part1_frames(d, begin, finish)
    if not len(index):
        return None
    command = d['commanded_phase_current_a'][index]
    regen = (d['mode'][index] == 1) & (command < -.05)
    if not regen.any():
        return dict(seconds=0.)
    t, speed = d['t'][index], d['speed_mps'][index]
    rate = np.gradient(speed, t) if len(t) > 2 else np.zeros(len(t))
    phase, bus = d['phase_current_a'][index][regen], d['bus_current_a'][index][regen]
    return dict(seconds=round(float(regen.sum()) * .05, 2),
                command_min_a=float(command[regen].min()),
                deceleration_mean_mps2=round(float(-rate[regen].mean()), 3),
                bus_current_min_a=float(bus.min()), bus_voltage_max_v=float(d['bus_voltage_v'][index][regen].max()),
                phase_minus_command_mean_a=round(float((phase - command[regen]).mean()), 2),
                charging_fraction=round(float((bus < 0).mean()), 3))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('--start', type=float, default=0, help='Seconds from first positive AUTO command')
    parser.add_argument('--end', type=float)
    parser.add_argument('--pi-csv', help='Export every received PI sample at its bag receive time')
    parser.add_argument('--target', type=float, help='Nominal event target for 10–90%%/arrival/settling')
    args = parser.parse_args()
    data, provenance = read_bag(args.bag)
    samples = provenance.pop('pi_samples')
    provenance['pi_sample_count'] = len(samples)
    counters = [row['collapse_events'] for row in samples if 'collapse_events' in row]
    if counters:
        provenance['firmware_collapse_events_in_bag'] = int(counters[-1] - counters[0])
    if args.pi_csv:
        if not samples:
            raise ValueError('No longitudinal PI samples to export')
        with open(args.pi_csv, 'w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(samples[0]))
            writer.writeheader()
            writer.writerows(samples)
    print(json.dumps({'bag': args.bag, **summarize(data, args.start, args.end, args.target,
                                                    phase_low_of(provenance['calibration'])),
                      **provenance}, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
