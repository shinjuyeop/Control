"""Recorded commands and encoder feedback on MCAP reception time."""
from pathlib import Path
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_run(path, calibration, dt=.02):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    names = ['/steering/status', '/steering/torque_command_raw', '/vehicle/command',
             '/steering/actuation_mode', '/gnss/fix_velocity', '/imu/data']
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in names}
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(types)))
    rows = {t: [] for t in types}
    while reader.has_next():
        topic, raw, stamp = reader.read_next()
        m = deserialize_message(raw, types[topic]); t = stamp * 1e-9
        if topic == names[0]:
            row = [t, m.motor_position_count, m.state == 3 and m.drive_mode == 1 and
                   m.feedback_valid and not m.estop_active and not m.command_brake_active,
                   m.actual_wheel_angle_rad]
        elif topic == names[1]: row = [t, m.data / 10000.]
        elif topic == names[2]: row = [t, m.steering_target_rad, m.brake_engage]
        elif topic == names[3]: row = [t, m.data == 'torque']
        elif topic == names[4]: row = [t, np.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)]
        else: row = [t, m.angular_velocity.z]
        rows[topic].append(row)
    if not rows.get(names[0]) or not rows.get(names[2]):
        raise ValueError(f'{path}: steering status and vehicle command are required')
    start = max(rows[n][0][0] for n in names[:3] if rows.get(n))
    end = min(rows[n][-1][0] for n in names[:3] if rows.get(n))
    grid = np.arange(start, end, dt)
    result = {'t': grid-start, 'name': str(Path(path).resolve()), 'dt': dt}
    fields = [('count','active','recorded_angle'), ('u',), ('reference','brake'),
              ('torque_mode',), ('speed',), ('yaw_rate',)]
    for topic, keys in zip(names, fields):
        if not rows.get(topic): continue
        a = np.asarray(rows[topic]); order = np.argsort(a[:,0],kind='stable'); a=a[order]
        idx = np.searchsorted(a[:,0], grid, side='right')-1; idx = np.maximum(idx,0)
        fresh = (grid >= a[0,0]) & (grid-a[idx,0] < .1)
        for j,key in enumerate(keys,1): result[key] = np.where(fresh,a[idx,j],np.nan)
    c = yaml.safe_load(Path(calibration).read_text())
    v = result['count']/c['count_scale']
    result['angle'] = np.deg2rad(sum(c[f'equivalent_k{k}_deg']*v**k for k in range(1,6)))
    result['valid'] = ((result['active']==1) & (result['brake']==0) &
        (np.abs(result['count']) <= c['command_count_limit']) & np.isfinite(result['reference']))
    if 'u' in result:
        result['valid'] &= np.isfinite(result['u']) & (result.get('torque_mode',0)==1)
    return result


def intervals(mask):
    edge = np.diff(np.r_[False,mask,False].astype(int))
    return zip(np.flatnonzero(edge==1),np.flatnonzero(edge==-1))


def timing(path):
    from diagnostic_msgs.msg import DiagnosticArray
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path),storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/control/steering/diagnostics','/control/lateral/diagnostics']))
    controllers={}
    while reader.has_next():
        _,raw,_=reader.read_next()
        for s in deserialize_message(raw,DiagnosticArray).status:
            data=controllers.setdefault(s.name,dict(failed_status_count=0,solver_failure_count=0,reasons={},fields={}))
            level=int.from_bytes(s.level,'little') if isinstance(s.level,bytes) else s.level
            data['failed_status_count']+=level>=2
            data['solver_failure_count']+=level>=2 and s.message.startswith('solver_')
            data['reasons'][s.message]=data['reasons'].get(s.message,0)+1
            row={kv.key:float(kv.value) for kv in s.values}
            if row.get('solve_ms',0)>0:
                for key,value in row.items(): data['fields'].setdefault(key,[]).append(value)
    return {name:dict(failed_status_count=d['failed_status_count'],solver_failure_count=d['solver_failure_count'],status_counts=d['reasons'],**{
        k:dict(p95=float(np.quantile(v,.95)),p99=float(np.quantile(v,.99)),maximum=float(np.max(v)))
        for k,v in d['fields'].items() if v}) for name,d in controllers.items()}
