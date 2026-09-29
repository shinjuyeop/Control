#!/usr/bin/env python3
# ============================================================================
# mcap_to_csv.py
#   rosbag2 mcap 녹화본을 토픽당 CSV 1개로 변환한다.
#   중첩 메시지 필드는 점 표기(header.stamp.sec, motor_current_a ...)로
#   컬럼 평탄화되어 pandas에서 바로 읽을 수 있다.
#
#   사용 (워크스페이스 소싱된 터미널에서 — custom msg 정의가 필요하므로):
#     ros2 run control_tools mcap_to_csv.py <bag 디렉토리>
#     ros2 run control_tools mcap_to_csv.py <bag 디렉토리> -o /tmp/csv_out
#     ros2 run control_tools mcap_to_csv.py <bag 디렉토리> \
#       -t /steering/status /vehicle/command
#
#   출력: <bag 디렉토리>/csv/<토픽이름(슬래시→__)>.csv
#   각 행 첫 컬럼 recv_time_sec = 수신 시각(bag 기록 시각, epoch 초, float)
# ============================================================================
import argparse
import csv
import os
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py


def flatten(msg, prefix=''):
    """ROS 메시지를 {점표기 필드명: 스칼라 값} dict로 재귀 평탄화."""
    out = {}
    for name in msg.get_fields_and_field_types():
        val = getattr(msg, name)
        key = f'{prefix}{name}'
        if hasattr(val, 'get_fields_and_field_types'):
            out.update(flatten(val, key + '.'))
        elif isinstance(val, (str, bytes)):
            out[key] = val if isinstance(val, str) else val.hex()
        else:
            try:
                # 배열류(list/tuple/numpy)는 세미콜론으로 이어 한 셀에 저장
                out[key] = ';'.join(str(v) for v in val)
            except TypeError:
                out[key] = val
    return out


def convert(bag_dir, out_dir, topics_filter):
    storage = rosbag2_py.StorageOptions(uri=bag_dir, storage_id='mcap')
    converter = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage, converter)

    type_by_topic = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topics_filter:
        missing = [t for t in topics_filter if t not in type_by_topic]
        if missing:
            print(f'경고: bag에 없는 토픽 무시: {missing}', file=sys.stderr)
        type_by_topic = {k: v for k, v in type_by_topic.items() if k in topics_filter}

    os.makedirs(out_dir, exist_ok=True)
    writers = {}   # topic -> (csv.DictWriter, file handle, fieldnames)
    msg_cls = {}
    rows = {t: 0 for t in type_by_topic}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in type_by_topic:
            continue
        if topic not in msg_cls:
            try:
                msg_cls[topic] = get_message(type_by_topic[topic])
            except (ModuleNotFoundError, AttributeError) as e:
                print(f'경고: {topic} 타입({type_by_topic[topic]}) 로드 실패, 건너뜀: {e}',
                      file=sys.stderr)
                type_by_topic.pop(topic)
                continue
        msg = deserialize_message(data, msg_cls[topic])
        row = {'recv_time_sec': t_ns / 1e9}
        row.update(flatten(msg))

        if topic not in writers:
            path = os.path.join(out_dir, topic.strip('/').replace('/', '__') + '.csv')
            f = open(path, 'w', newline='')
            w = csv.DictWriter(f, fieldnames=list(row.keys()), extrasaction='ignore')
            w.writeheader()
            writers[topic] = (w, f)
        writers[topic][0].writerow(row)
        rows[topic] += 1

    for topic, (_, f) in writers.items():
        f.close()
        print(f'{topic}: {rows[topic]} rows -> {f.name}')
    if not writers:
        print('변환된 토픽이 없습니다.', file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description='rosbag2 mcap -> 토픽당 CSV')
    p.add_argument('bag_dir', help='rosbag2 디렉토리 (metadata.yaml이 있는 곳)')
    p.add_argument('-o', '--out-dir', default=None,
                   help='CSV 출력 디렉토리 (기본: <bag_dir>/csv)')
    p.add_argument('-t', '--topics', nargs='+', default=None,
                   help='이 토픽들만 변환 (기본: 전부)')
    args = p.parse_args()

    bag_dir = os.path.abspath(os.path.expanduser(args.bag_dir))
    if not os.path.isfile(os.path.join(bag_dir, 'metadata.yaml')):
        p.error(f'{bag_dir}에 metadata.yaml이 없습니다 — rosbag2 디렉토리를 지정하세요')
    out_dir = args.out_dir or os.path.join(bag_dir, 'csv')
    convert(bag_dir, out_dir, args.topics)


if __name__ == '__main__':
    main()
