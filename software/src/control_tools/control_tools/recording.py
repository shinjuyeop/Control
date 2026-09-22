"""Shared MCAP storage options and worker for both recording entrypoints."""
import json
import sys

STORAGE_PRESET = 'zstd_fast'


def storage_options(uri):
    import rosbag2_py
    options = rosbag2_py.StorageOptions(uri=str(uri), storage_id='mcap')
    options.storage_preset_profile = STORAGE_PRESET
    return options


def record(uri, topics):
    import rosbag2_py
    options = rosbag2_py.RecordOptions()
    options.topics = list(topics)
    options.rmw_serialization_format = 'cdr'
    try:
        rosbag2_py.Recorder().record(storage_options(uri), options)
    except KeyboardInterrupt:
        pass


def worker_main():
    record(sys.argv[1], json.loads(sys.argv[2]))


def recorded_counts(uri):
    import rosbag2_py
    metadata = rosbag2_py.Info().read_metadata(str(uri), 'mcap')
    return {item.topic_metadata.name: item.message_count
            for item in metadata.topics_with_message_count}


if __name__ == '__main__':
    worker_main()
