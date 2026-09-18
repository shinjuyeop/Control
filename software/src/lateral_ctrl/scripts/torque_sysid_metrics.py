#!/usr/bin/env python3
"""Extract torque-steering step and inertia metrics from MCAP recordings."""

import argparse
import csv
import math
import os
import sys

import numpy as np


STATE_ACTIVE = 3
STATUS_TOPIC = '/steering/status'


def read_status(bag_dir):
    """Return stamp, target, actual and ACTIVE arrays from /steering/status."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
        rosbag2_py.ConverterOptions('', ''),
    )
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    if STATUS_TOPIC not in types:
        raise RuntimeError(f'{bag_dir}: {STATUS_TOPIC} topic is missing')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[STATUS_TOPIC]))
    message_type = get_message(types[STATUS_TOPIC])
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        message = deserialize_message(data, message_type)
        if not message.feedback_valid:
            continue
        rows.append((
            message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            message.target_wheel_angle_rad,
            message.actual_wheel_angle_rad,
            message.state == STATE_ACTIVE,
        ))
    if not rows:
        raise RuntimeError(f'{bag_dir}: no status samples with valid feedback')
    stamp, target, actual, active = (np.array(column) for column in zip(*rows))
    order = np.argsort(stamp, kind='stable')
    return stamp[order], target[order], actual[order], active[order].astype(bool)


def zeta_from_overshoot(overshoot_ratio):
    if overshoot_ratio <= 0.0:
        return 1.0
    logarithm = math.log(overshoot_ratio)
    return -logarithm / math.sqrt(math.pi ** 2 + logarithm ** 2)


def step_metrics(stamp, target, actual, active, min_step_rad, settle_band_rad):
    """Split ACTIVE samples into constant-target segments and score real steps."""
    results = []
    index = 0
    previous_target = None
    while index < len(stamp):
        if not active[index]:
            previous_target = None
            index += 1
            continue
        end = index
        while (end + 1 < len(stamp) and active[end + 1] and
               abs(target[end + 1] - target[index]) < 1e-7):
            end += 1
        segment = slice(index, end + 1)
        start_angle = actual[index]
        step = target[index] - (
            previous_target if previous_target is not None else start_angle)
        if abs(step) >= min_step_rad and end > index:
            times = stamp[segment] - stamp[index]
            angles = actual[segment]
            goal = target[index]
            direction = math.copysign(1.0, step)
            travel = goal - start_angle
            overshoot = max(0.0, float(np.max((angles - goal) * direction)))
            ratio = overshoot / abs(travel) if abs(travel) > 1e-9 else float('nan')
            progress = ((angles - start_angle) / travel if abs(travel) > 1e-9
                        else np.zeros_like(angles))
            rise = float('nan')
            above_10 = np.nonzero(progress >= 0.1)[0]
            above_90 = np.nonzero(progress >= 0.9)[0]
            if above_10.size and above_90.size:
                rise = times[above_90[0]] - times[above_10[0]]
            outside = np.nonzero(np.abs(angles - goal) > settle_band_rad)[0]
            settle = 0.0 if not outside.size else (
                times[outside[-1] + 1]
                if outside[-1] + 1 < len(times) else float('nan'))
            tail = angles[int(len(angles) * 0.75):]
            results.append({
                'start_sec': stamp[index],
                'duration_sec': times[-1],
                'target_deg': math.degrees(goal),
                'step_deg': math.degrees(step),
                'overshoot_pct': ratio * 100.0,
                'zeta': zeta_from_overshoot(ratio) if math.isfinite(ratio) else float('nan'),
                'rise_10_90_sec': rise,
                'settle_sec': settle,
                'final_error_deg': math.degrees(float(np.mean(tail - goal))),
            })
        previous_target = target[index]
        index = end + 1
    return results


def motion_onsets(stamp, angle, onset_rad, still_sec):
    onsets = []
    anchor = 0
    for index in range(1, len(stamp)):
        if abs(angle[index] - angle[anchor]) < onset_rad:
            continue
        if stamp[index - 1] - stamp[anchor] >= still_sec:
            onsets.append(index)
        anchor = index
    return onsets


def initial_acceleration(stamp, angle, onset, fit_sec):
    start = max(onset - 1, 0)
    stop = start
    while stop + 1 < len(stamp) and stamp[stop + 1] - stamp[start] <= fit_sec:
        stop += 1
    count = stop - start + 1
    if count < 4:
        return float('nan'), count
    times = stamp[start:stop + 1] - stamp[start]
    coefficients = np.polyfit(times, angle[start:stop + 1], 2)
    return abs(2.0 * coefficients[0]), count


def write_csv(path, rows):
    if not path:
        return
    with open(path, 'w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(f'CSV -> {path}')


def run_step(args):
    rows = []
    for bag in args.bags:
        stamp, target, actual, active = read_status(bag)
        for result in step_metrics(
                stamp, target, actual, active, math.radians(args.min_step_deg),
                math.radians(args.settle_band_deg)):
            rows.append({'bag': os.path.basename(os.path.normpath(bag)), **result})
    if not rows:
        print('No target step was found in an ACTIVE interval.', file=sys.stderr)
        return 1
    print(f"{'bag':<40} {'step deg':>8} {'OS%':>7} {'zeta':>6} "
          f"{'rise s':>7} {'settle s':>8} {'err deg':>8}")
    for result in rows:
        print(f"{result['bag'][:40]:<40} {result['step_deg']:>8.2f} "
              f"{result['overshoot_pct']:>7.1f} {result['zeta']:>6.2f} "
              f"{result['rise_10_90_sec']:>7.3f} {result['settle_sec']:>8.3f} "
              f"{result['final_error_deg']:>8.3f}")
    write_csv(args.csv, rows)
    return 0


def run_inertia(args):
    samples = []
    for specification in args.runs:
        bag, separator, torque = specification.rpartition(':')
        if not separator:
            raise SystemExit(f'{specification!r}: expected BAG:TORQUE_RAW')
        torque = abs(float(torque))
        stamp, _, actual, _ = read_status(bag)
        for onset in motion_onsets(
                stamp, actual, math.radians(args.onset_deg), args.still_sec):
            acceleration, count = initial_acceleration(
                stamp, actual, onset, args.fit_ms / 1000.0)
            if math.isfinite(acceleration) and acceleration > 0.0:
                samples.append({
                    'bag': os.path.basename(os.path.normpath(bag)),
                    'onset_sec': stamp[onset],
                    'torque_raw': torque,
                    'alpha_rad_s2': acceleration,
                    'fit_samples': count,
                })
    if not samples:
        print('No motion onset was found.', file=sys.stderr)
        return 1
    print(f"{'bag':<40} {'torque':>7} {'alpha rad/s2':>12} {'n':>3}")
    for sample in samples:
        print(f"{sample['bag'][:40]:<40} {sample['torque_raw']:>7.0f} "
              f"{sample['alpha_rad_s2']:>12.2f} {sample['fit_samples']:>3}")
    torque = np.array([sample['torque_raw'] for sample in samples])
    acceleration = np.array([sample['alpha_rad_s2'] for sample in samples])
    if args.friction_raw is not None:
        net = torque - args.friction_raw
        inertia = float(np.dot(net, acceleration) / np.dot(acceleration, acceleration))
        print(f'\nJ_ff (F_c={args.friction_raw:g}) = {inertia:.3f} raw/(rad/s2)')
    if len(set(torque)) >= 2:
        inertia, friction = np.polyfit(acceleration, torque, 1)
        print(f'J_ff (torque = J*alpha + F) = {inertia:.3f} raw/(rad/s2), '
              f'F = {friction:.1f} raw')
    else:
        print('At least two torque levels are needed to fit J and F together.')
    write_csv(args.csv, samples)
    return 0


def main():
    parser = argparse.ArgumentParser(description='Torque steering SYSID metrics')
    commands = parser.add_subparsers(dest='command', required=True)
    step = commands.add_parser('step', help='step response metrics')
    step.add_argument('bags', nargs='+')
    step.add_argument('--min-step-deg', type=float, default=0.5)
    step.add_argument('--settle-band-deg', type=float, default=0.1)
    step.add_argument('--csv')
    step.set_defaults(func=run_step)
    inertia = commands.add_parser('inertia', help='initial acceleration and J_ff fit')
    inertia.add_argument('runs', nargs='+', metavar='BAG:TORQUE_RAW')
    inertia.add_argument('--friction-raw', type=float)
    inertia.add_argument('--onset-deg', type=float, default=0.05)
    inertia.add_argument('--still-sec', type=float, default=0.3)
    inertia.add_argument('--fit-ms', type=float, default=100.0)
    inertia.add_argument('--csv')
    inertia.set_defaults(func=run_inertia)
    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
