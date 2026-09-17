#!/usr/bin/env python3
# ============================================================================
# torque_sysid_metrics.py
#   docs/Experiment_RUNBOOK.md의 토크 모드 실험 MCAP에서 판정 지표를 뽑는다.
#   입력은 /steering/status 하나다(canonical LEFT+ 바퀴각, header.stamp 시각).
#   전류는 1 A 양자화라 쓰지 않고, motor_speed_rpm 이상치 문제를 피하려고
#   속도·가속도도 각도에서 직접 구한다.
#
#   step    — T3: ACTIVE 구간의 목표 step마다 %OS, ζ, 10→90% 상승시간,
#             정착시간, 정착오차.
#               ros2 run lateral_ctrl torque_sysid_metrics.py step <bag> [<bag> ...]
#   inertia — T2: 정지 후 움직이기 시작하는 순간마다 θ(t)를 2차 다항식으로 맞춰
#             초기 각가속도 α를 구하고, 레벨별 지령 토크와 회귀해 J_ff·F_c를 낸다.
#               ros2 run lateral_ctrl torque_sysid_metrics.py inertia \
#                 <bag>:<torque_raw> [<bag>:<torque_raw> ...] [--friction-raw 106]
#
#   rpm/전류를 쓰지 않으므로 워크스페이스 소싱(kaiev26_msgs 타입)만 필요하다.
# ============================================================================
import argparse
import csv
import math
import os
import sys

import numpy as np

STATE_ACTIVE = 3  # lateral_ctrl::SteeringState::ACTIVE
STATUS_TOPIC = '/steering/status'


def read_status(bag_dir):
    """Return stamp[s], target[rad], actual[rad], active[bool] arrays from one bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if STATUS_TOPIC not in types:
        raise RuntimeError(f'{bag_dir}: {STATUS_TOPIC} 토픽이 없습니다')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[STATUS_TOPIC]))
    cls = get_message(types[STATUS_TOPIC])
    rows = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        m = deserialize_message(data, cls)
        if not m.feedback_valid:
            continue
        rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                     m.target_wheel_angle_rad, m.actual_wheel_angle_rad,
                     m.state == STATE_ACTIVE))
    if not rows:
        raise RuntimeError(f'{bag_dir}: feedback_valid인 status가 없습니다')
    t, target, actual, active = (np.array(c) for c in zip(*rows))
    order = np.argsort(t, kind='stable')
    return t[order], target[order], actual[order], active[order].astype(bool)


def zeta_from_overshoot(overshoot_ratio):
    if overshoot_ratio <= 0.0:
        return 1.0
    ln = math.log(overshoot_ratio)
    return -ln / math.sqrt(math.pi ** 2 + ln ** 2)


def step_metrics(t, target, actual, active, min_step_rad, settle_band_rad):
    """Split ACTIVE samples into constant-target segments and score each real step."""
    results = []
    n = len(t)
    i = 0
    previous_target = None
    while i < n:
        if not active[i]:
            previous_target = None
            i += 1
            continue
        j = i
        while j + 1 < n and active[j + 1] and abs(target[j + 1] - target[i]) < 1e-7:
            j += 1
        seg = slice(i, j + 1)
        # The first ACTIVE segment has no previous target; step from where the wheel was.
        start_angle = actual[i]
        step = target[i] - (previous_target if previous_target is not None else start_angle)
        if abs(step) >= min_step_rad and j > i:
            ts, ys, goal = t[seg] - t[i], actual[seg], target[i]
            direction = math.copysign(1.0, step)
            travel = goal - start_angle
            overshoot = max(0.0, float(np.max((ys - goal) * direction)))
            ratio = overshoot / abs(travel) if abs(travel) > 1e-9 else float('nan')
            progress = (ys - start_angle) / travel if abs(travel) > 1e-9 else np.zeros_like(ys)
            rise = float('nan')
            above10 = np.nonzero(progress >= 0.1)[0]
            above90 = np.nonzero(progress >= 0.9)[0]
            if above10.size and above90.size:
                rise = ts[above90[0]] - ts[above10[0]]
            outside = np.nonzero(np.abs(ys - goal) > settle_band_rad)[0]
            settle = 0.0 if not outside.size else (
                ts[outside[-1] + 1] if outside[-1] + 1 < len(ts) else float('nan'))
            tail = ys[int(len(ys) * 0.75):]
            results.append({
                'start_sec': t[i], 'duration_sec': ts[-1],
                'target_deg': math.degrees(goal), 'step_deg': math.degrees(step),
                'overshoot_pct': ratio * 100.0,
                'zeta': zeta_from_overshoot(ratio) if math.isfinite(ratio) else float('nan'),
                'rise_10_90_sec': rise, 'settle_sec': settle,
                'final_error_deg': math.degrees(float(np.mean(tail - goal))),
            })
        previous_target = target[i]
        i = j + 1
    return results


def motion_onsets(t, angle, onset_rad, still_sec):
    """Indices where the wheel leaves a still window of at least still_sec."""
    onsets = []
    anchor = 0
    for k in range(1, len(t)):
        if abs(angle[k] - angle[anchor]) < onset_rad:
            continue
        if t[k - 1] - t[anchor] >= still_sec:
            onsets.append(k)
        # Any movement restarts the still window, so only a fresh start from rest counts.
        anchor = k
    return onsets


def initial_acceleration(t, angle, onset, fit_sec):
    """Quadratic LSQ fit from the last still sample through fit_sec: returns |alpha|, samples."""
    start = max(onset - 1, 0)
    stop = start
    while stop + 1 < len(t) and t[stop + 1] - t[start] <= fit_sec:
        stop += 1
    if stop - start + 1 < 4:
        return float('nan'), stop - start + 1
    ts = t[start:stop + 1] - t[start]
    coeffs = np.polyfit(ts, angle[start:stop + 1], 2)
    return abs(2.0 * coeffs[0]), stop - start + 1


def run_step(args):
    rows = []
    for bag in args.bags:
        t, target, actual, active = read_status(bag)
        for r in step_metrics(t, target, actual, active, math.radians(args.min_step_deg),
                              math.radians(args.settle_band_deg)):
            rows.append({'bag': os.path.basename(os.path.normpath(bag)), **r})
    if not rows:
        print('ACTIVE 구간에서 step을 찾지 못했습니다.', file=sys.stderr)
        return 1
    print(f"{'bag':<40} {'step°':>7} {'OS%':>7} {'ζ':>6} {'rise s':>7} {'settle s':>8} {'err°':>7}")
    for r in rows:
        print(f"{r['bag'][:40]:<40} {r['step_deg']:>7.2f} {r['overshoot_pct']:>7.1f} "
              f"{r['zeta']:>6.2f} {r['rise_10_90_sec']:>7.3f} {r['settle_sec']:>8.3f} "
              f"{r['final_error_deg']:>7.3f}")
    write_csv(args.csv, rows)
    return 0


def run_inertia(args):
    samples = []
    for spec in args.runs:
        bag, sep, torque = spec.rpartition(':')
        if not sep:
            raise SystemExit(f'"{spec}": <bag>:<torque_raw> 형식이어야 합니다')
        torque = abs(float(torque))
        t, _, actual, _ = read_status(bag)
        for k in motion_onsets(t, actual, math.radians(args.onset_deg), args.still_sec):
            alpha, count = initial_acceleration(t, actual, k, args.fit_ms / 1000.0)
            if math.isfinite(alpha) and alpha > 0.0:
                samples.append({'bag': os.path.basename(os.path.normpath(bag)),
                                'onset_sec': t[k], 'torque_raw': torque,
                                'alpha_rad_s2': alpha, 'fit_samples': count})
    if not samples:
        print('움직임 시작 구간을 찾지 못했습니다.', file=sys.stderr)
        return 1
    print(f"{'bag':<40} {'torque':>7} {'α rad/s²':>10} {'n':>3}")
    for s in samples:
        print(f"{s['bag'][:40]:<40} {s['torque_raw']:>7.0f} {s['alpha_rad_s2']:>10.2f} "
              f"{s['fit_samples']:>3}")
    tau = np.array([s['torque_raw'] for s in samples])
    alpha = np.array([s['alpha_rad_s2'] for s in samples])
    if args.friction_raw is not None:
        net = tau - args.friction_raw
        j = float(np.dot(net, alpha) / np.dot(alpha, alpha))
        print(f'\nJ_ff (F_c={args.friction_raw:g} 고정) = {j:.3f} raw/(rad/s²)')
    if len(set(tau)) >= 2:
        j, friction = np.polyfit(alpha, tau, 1)
        print(f'J_ff (τ = J·α + F 회귀) = {j:.3f} raw/(rad/s²), 절편 F = {friction:.1f} raw')
    else:
        print('토크 레벨이 하나뿐이라 J·F 동시 회귀는 생략합니다(레벨 2개 이상 필요).')
    write_csv(args.csv, samples)
    return 0


def write_csv(path, rows):
    if not path:
        return
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f'CSV -> {path}')


def main():
    p = argparse.ArgumentParser(description='토크 모드 조향 SYSID 지표 (Experiment_RUNBOOK T2/T3)')
    sub = p.add_subparsers(dest='command', required=True)
    s = sub.add_parser('step', help='T3 step 응답: %%OS, ζ, 상승/정착시간, 정착오차')
    s.add_argument('bags', nargs='+')
    s.add_argument('--min-step-deg', type=float, default=0.5)
    s.add_argument('--settle-band-deg', type=float, default=0.1)
    s.add_argument('--csv')
    s.set_defaults(func=run_step)
    i = sub.add_parser('inertia', help='T2 계단 토크: 초기 α와 J_ff 회귀')
    i.add_argument('runs', nargs='+', metavar='BAG:TORQUE_RAW')
    i.add_argument('--friction-raw', type=float, default=None,
                   help='T1의 F_c. 주면 원점 통과 회귀로 J만 구한다')
    i.add_argument('--onset-deg', type=float, default=0.05)
    i.add_argument('--still-sec', type=float, default=0.3)
    i.add_argument('--fit-ms', type=float, default=100.0)
    i.add_argument('--csv')
    i.set_defaults(func=run_inertia)
    args = p.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
