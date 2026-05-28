#!/usr/bin/env python3
"""
SILS + HILS bag 분석 스크립트
- 토픽이 없으면 NaN으로 채워서 CSV 저장
"""

from mcap_ros2.reader import read_ros2_messages
import pandas as pd
import glob
import sys
import os

os.makedirs('analysis', exist_ok=True)

# 마지막 bag analyze
bags = sorted(glob.glob('/home/seo/KAI_26/workspace/Control/pid_gazebo/bags/sils_bag_*/*.mcap'))
if not bags:
    print("bag 파일 없음")
    sys.exit(1)

bag_path = bags[-1]
bag_name = os.path.basename(os.path.dirname(bag_path))
print(f"읽는 파일: {bag_path}")

records = {
    '/setpoint/rpm':    [],
    '/setpoint/steer_deg': [],
    '/odom':            [],
    '/cmd_vel':         [],
    '/cmd/speed':       [],
    '/current/speed':   [],
    '/target/rpm':      [],
    '/cmd/steer':       [],
    '/current/steer':   [],
    '/gear':            [],
    '/mode':            [],
    '/aeb':             [],
    '/estop/status':    [],
    '/estop/rf':        [],
    '/pedal':           [],
    '/brake/target':    [],
    '/brake/current':   [],
}

for msg in read_ros2_messages(bag_path):
    topic = msg.channel.topic
    t = msg.log_time.timestamp()
    m = msg.ros_msg

    if topic not in records:
        continue

    if topic == '/odom':
        records[topic].append({'t': t, 'odom_vx': m.twist.twist.linear.x, 'odom_yaw': 0.0})
    elif topic == '/cmd_vel':
        records[topic].append({'t': t, 'cmd_linear_x': m.linear.x, 'cmd_angular_z': m.angular.z})
    else:
        # 토픽 마지막 부분을 컬럼명으로 사용 (/setpoint/rpm → rpm)
        col = topic.split('/')[-1]
        records[topic].append({'t': t, col: float(m.data)})

# 기준 시간 잡기
all_times = [r[0]['t'] for r in records.values() if r]
t0 = min(all_times)

# DataFrame 변환
dfs = {}
for topic, data in records.items():
    if not data:
        continue
    df = pd.DataFrame(data)
    df['t'] -= t0
    df = df.sort_values('t')
    # 컬럼명에 토픽 prefix
    name = topic.replace('/', '_').strip('_')
    dfs[topic] = df

base_key = '/setpoint/rpm' if '/setpoint/rpm' in dfs else list(dfs.keys())[0]
merged = dfs[base_key]
for topic, df in dfs.items():
    if topic == base_key:
        continue
    merged = pd.merge_asof(merged, df, on='t', direction='nearest')

out_path = f'analysis/{bag_name}.csv'
merged.to_csv(out_path, index=False)
print(f"\nCSV 저장: {out_path}")
print(f"rows: {len(merged)}, columns: {list(merged.columns)}")
