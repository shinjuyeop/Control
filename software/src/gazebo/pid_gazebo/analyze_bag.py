#!/usr/bin/env python3
"""
SILS + HILS bag 분석 스크립트
- 토픽이 없으면 NaN으로 채워서 CSV 저장
"""

from mcap_ros2.reader import read_ros2_messages
import pandas as pd
import sys
from pathlib import Path

PACKAGE_DIR = Path(__file__).resolve().parent
ANALYSIS_DIR = PACKAGE_DIR / 'analysis'
BAGS_DIR = PACKAGE_DIR / 'bags'

ANALYSIS_DIR.mkdir(exist_ok=True)


def _resolve_bag_path() -> Path | None:
    if len(sys.argv) > 1:
        candidate = Path(sys.argv[1]).expanduser().resolve()
        if candidate.is_file():
            return candidate
        if candidate.is_dir():
            files = sorted(candidate.glob('*.mcap'))
            return files[-1] if files else None
        return None

    bags = sorted(BAGS_DIR.glob('sils_bag_*/*.mcap'))
    return bags[-1] if bags else None


bag_path = _resolve_bag_path()
if bag_path is None:
    print("bag 파일 없음")
    sys.exit(1)

bag_name = bag_path.parent.name
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

for msg in read_ros2_messages(str(bag_path)):
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

out_path = ANALYSIS_DIR / f'{bag_name}.csv'
merged.to_csv(out_path, index=False)
print(f"\nCSV 저장: {out_path}")
print(f"rows: {len(merged)}, columns: {list(merged.columns)}")
