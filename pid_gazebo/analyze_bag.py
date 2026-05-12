#!/usr/bin/env python3
"""
SILS bag 분석 스크립트
- /setpoint/rpm vs /odom 실제 속도 비교
- /cmd_vel 출력 확인
- 결과를 analysis/ 폴더에 단일 CSV로 저장
"""

from mcap_ros2.reader import read_ros2_messages
import pandas as pd
import glob
import sys
import os

# 저장 폴더 생성
os.makedirs('analysis', exist_ok=True)

# 가장 최근 bag 자동 선택
bags = sorted(glob.glob('sils_bag_*/*.mcap'))
if not bags:
    print("bag 파일 없음")
    sys.exit(1)

bag_path = bags[-1]
bag_name = bag_path.split('/')[0]
print(f"읽는 파일: {bag_path}")

records = {
    'setpoint_rpm': [],
    'odom_vel': [],
    'cmd_vel': [],
}

for msg in read_ros2_messages(bag_path):
    topic = msg.channel.topic
    t = msg.log_time.timestamp()

    if topic == '/setpoint/rpm':
        records['setpoint_rpm'].append({'t': t, 'setpoint_rpm': msg.ros_msg.data})
    elif topic == '/odom':
        records['odom_vel'].append({'t': t, 'odom_vx': msg.ros_msg.twist.twist.linear.x})
    elif topic == '/cmd_vel':
        records['cmd_vel'].append({'t': t, 'cmd_linear_x': msg.ros_msg.linear.x})

df_sp   = pd.DataFrame(records['setpoint_rpm'])
df_odom = pd.DataFrame(records['odom_vel'])
df_cmd  = pd.DataFrame(records['cmd_vel'])

# 시작 시간 normalize
t0 = min(df_sp['t'].min(), df_odom['t'].min(), df_cmd['t'].min())
for df in [df_sp, df_odom, df_cmd]:
    df['t'] -= t0

# timestamp 기준 merge_asof (nearest)
df_sp   = df_sp.sort_values('t')
df_odom = df_odom.sort_values('t')
df_cmd  = df_cmd.sort_values('t')

df_merged = pd.merge_asof(df_sp, df_odom, on='t', direction='nearest')
df_merged = pd.merge_asof(df_merged, df_cmd, on='t', direction='nearest')

# 저장
out_path = f'analysis/{bag_name}.csv'
df_merged.to_csv(out_path, index=False)
print(f"\nCSV 저장 완료: {out_path}")
print(f"rows: {len(df_merged)}, columns: {list(df_merged.columns)}")
print(df_merged.describe())