"""Decode MCAP at its recording boundary and align telemetry on a common clock."""
from pathlib import Path
import json
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

TOPICS = ('/drive/status', '/vehicle/state', '/vehicle/command',
          '/vehicle/drive_current_command', '/brake/status', '/control/longitudinal/diagnostics', '/gnss/fix_velocity')


def read_run(path, dt=.05):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    rows = {t: [] for t in types}
    diagnostics = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg = deserialize_message(data, types[topic]); t = stamp*1e-9
        if topic == '/drive/status':
            row = [t, msg.speed_mps, msg.phase_current_a, msg.commanded_phase_current_a,
                   msg.mode, msg.part1_valid and msg.part2_valid and msg.part1_age_ms < 200 and msg.part2_age_ms < 200,
                   msg.stop_request or (msg.fault_bits_valid and msg.fault_bits != 0), msg.controller_control_mode]
        elif topic == '/gnss/fix_velocity':
            row = [t, np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)]
        elif topic == '/vehicle/state':
            row = [t, msg.speed_mps]
        elif topic == '/vehicle/command':
            row = [t, msg.speed_target_mps, msg.brake_engage]
        elif topic == '/vehicle/drive_current_command':
            row = [t, msg.phase_current_a, msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9]
        elif topic == '/brake/status':
            row = [t, msg.brake_commanded]
        else:
            diagnostics.append(dict(t=t, **json.loads(msg.data)))
            continue
        rows[topic].append(row)
    for topic in ('/drive/status', '/vehicle/state', '/vehicle/command', '/vehicle/drive_current_command'):
        if not rows.get(topic):
            raise ValueError(f'{path}: missing required samples on {topic}')
    start = max(rows[t][0][0] for t in ('/drive/status', '/vehicle/state', '/vehicle/command', '/vehicle/drive_current_command'))
    end = min(rows[t][-1][0] for t in ('/drive/status', '/vehicle/state', '/vehicle/command', '/vehicle/drive_current_command'))
    grid = np.arange(start, end, dt)
    result = dict(t=grid-start, name=str(Path(path).resolve()), diagnostics=diagnostics)
    fields = {
        '/drive/status': ('wheel_speed', 'i', 'u', 'mode', 'feedback_valid', 'stop', 'motor_mode'),
        '/gnss/fix_velocity': ('body_speed',),
        '/vehicle/state': ('v',), '/vehicle/command': ('reference', 'brake'),
        '/vehicle/drive_current_command': ('requested_current', 'request_stamp'), '/brake/status': ('brake_actual',)}
    for topic, keys in fields.items():
        if not rows.get(topic):
            continue
        a = np.asarray(rows[topic], float)
        order = np.argsort(a[:, 0], kind='stable'); a = a[order]
        idx = np.searchsorted(a[:, 0], grid, side='right')-1
        valid = (idx >= 0) & (grid-a[np.maximum(idx, 0), 0] < .2)
        for j, name in enumerate(keys, 1):
            values = np.interp(grid,a[:,0],a[:,j]) if name=='body_speed' else a[np.maximum(idx,0),j]
            result[name] = np.where(valid, values, np.nan)
    request_rows = np.asarray(rows['/vehicle/drive_current_command'],float)
    result['request_intervals_ros_ms'] = np.diff(request_rows[:,2])*1000
    result['valid'] = ((result['feedback_valid'] == 1) & (result['mode'] == 1)
        & (result['motor_mode'] == 0) & (result['stop'] == 0) & (result['brake'] == 0)
        & np.isfinite(result['v']) & np.isfinite(result['i']) & np.isfinite(result['u']))
    if 'brake_actual' in result:
        result['valid'] &= result['brake_actual'] == 0
    return result


def intervals(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1))
