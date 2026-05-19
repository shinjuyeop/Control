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

WORK_DIR = '/home/junhyeon/workspace/Control/pid_gazebo'

# 저장 폴더 생성
os.makedirs(os.path.join(WORK_DIR, 'analysis'), exist_ok=True)

# 가장 최근 bag 자동 선택
#bags = sorted(glob.glob('sils_bag_*/*.mcap'))
#bags = sorted(glob.glob('sils_bag_step4/*.mcap'))
#if not bags:
 #   print("bag 파일 없음")
 #   sys.exit(1)

#bag_path = bags[-1]
#bag_name = bag_path.split('/')[0]

# 수정 - 인자로 bag 절대경로 받으면 해당 bag 분석, 없으면 최신 bag 자동 선택
if len(sys.argv) > 1:
    bag_abs_path = sys.argv[1]  # 절대경로로 받음
    bag_name = os.path.basename(bag_abs_path)  # 폴더명만 추출
    # 폴더 안에서 mcap 파일 자동 찾기 (파일명이 폴더명과 다를 수 있음)
    mcap_files = glob.glob(f'{bag_abs_path}/*.mcap')
    if not mcap_files:
        print(f"mcap 파일 없음: {bag_abs_path}")
        sys.exit(1)
    bag_path = mcap_files[0]
else:
    # 절대경로로 최신 bag 자동 선택
    bags = sorted(glob.glob(os.path.join(WORK_DIR, 'sils_bag_run_*', '*.mcap')))
    if not bags:
        print("bag 파일 없음")
        sys.exit(1)
    bag_path = bags[-1]
    bag_name = bag_path.split('/')[-2]  # 폴더명 추출

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

# 저장 (절대경로 + bag 이름으로 구분)
out_path = os.path.join(WORK_DIR, 'analysis', f'{bag_name}.csv')
df_merged.to_csv(out_path, index=False)
print(f"\nCSV 저장 완료: {out_path}")
print(f"rows: {len(df_merged)}, columns: {list(df_merged.columns)}")
print(df_merged.describe())