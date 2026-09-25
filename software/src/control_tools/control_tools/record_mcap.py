"""Independent passive recorder with actual control-parameter capture."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import signal
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
import yaml

from control_tuning.storage import SettingsStore
from .parameter_capture import ParameterCapture
from .process import OwnedProcess
from .recording import STORAGE_PRESET, recorded_counts
from .topics import recording_topics


def output_base(value, require_external_storage=False):
    base = Path(value).expanduser().resolve()
    if require_external_storage:
        root = (Path.home() / 'Storage/ROS2_Workspace_offload').resolve()
        if not root.is_dir() or root.stat().st_dev == Path('/').stat().st_dev:
            raise ValueError(f'external artifact mount unavailable: {root}')
        if not base.is_relative_to(root):
            raise ValueError(f'output_base_dir must stay under {root}')
    return base


def save_yaml(path, document):
    SettingsStore._replace(path, yaml.safe_dump(document, sort_keys=False,
                           allow_unicode=True).encode('utf-8'))


def request_stop(_signum, _frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-base-dir', default=str(Path.home() / 'KAI_ws/records'))
    parser.add_argument('--require-external-storage', action='store_true')
    parser.add_argument('--no-capture-parameters', action='store_true')
    parser.add_argument('--no-require-longitudinal', action='store_true')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args(argv)
    base = output_base(args.output_base_dir, args.require_external_storage)
    run = base / datetime.now().strftime('%Y%m%d_%H%M%S_%f_independent')
    run.mkdir(parents=True, exist_ok=False)
    topics = recording_topics('independent')
    document = dict(profile='independent', topics=list(topics), storage_preset=STORAGE_PRESET,
                    status='starting', parameter_capture='disabled' if args.no_capture_parameters else 'pending')
    save_yaml(run / 'recording.yaml', document)
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Node('control_tools_recorder', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', value=args.use_sim_time)])
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    publisher = node.create_publisher(String, '/sysid/control_parameters', 10)
    recorder = capture = None
    code = 0
    required = ('/sysid/control_parameters', '/parameter_events')
    if not args.no_capture_parameters and not args.no_require_longitudinal:
        required += ('/drive/longitudinal_status',)
    previous = {topic: {bytes(info.endpoint_gid) for info in node.get_subscriptions_info_by_topic(topic)}
                for topic in required}
    started = time.monotonic()
    snapshot_sent = False
    previous_sigterm = signal.signal(signal.SIGTERM, request_stop)
    try:
        recorder = OwnedProcess('MCAP', [sys.executable, '-u', '-m', 'control_tools.recording',
                                str(run / 'rosbag'), json.dumps(topics), str(args.use_sim_time).lower()], print)
        node.get_logger().info(f'MCAP: {run}')
        while True:
            executor.spin_once(timeout_sec=.05)
            if recorder.process.poll() is not None:
                raise RuntimeError('MCAP recorder exited before stop')
            if snapshot_sent:
                continue
            connected = all(any(info.node_name == 'rosbag2_recorder' and
                                bytes(info.endpoint_gid) not in previous[topic]
                                for info in node.get_subscriptions_info_by_topic(topic))
                            for topic in required)
            if capture is None:
                if connected:
                    if args.no_capture_parameters:
                        snapshot_sent = True
                    else:
                        capture = ParameterCapture(node, require_longitudinal=not args.no_require_longitudinal)
                elif time.monotonic() - started > 15:
                    raise RuntimeError('MCAP required subscriptions did not connect')
            if capture is not None and capture.tick():
                save_yaml(run / 'control_parameters.yaml', capture.document)
                document['parameter_capture'] = capture.document['status']
                if capture.document['status'] != 'complete':
                    raise RuntimeError('Actual control parameter capture incomplete')
                publisher.publish(String(data=json.dumps(capture.document, ensure_ascii=False)))
                snapshot_sent = True
                capture.close()
                capture = None
            if snapshot_sent:
                document['status'] = 'recording'
                save_yaml(run / 'recording.yaml', document)
                detail = '파라미터 수집 생략' if args.no_capture_parameters else '실제 제어 설정 저장 완료'
                node.get_logger().info(f'녹화 준비 완료 · {detail} · Ctrl+C로 마감')
    except KeyboardInterrupt:
        pass
    except Exception as error:
        code = 1
        document['error'] = str(error)
        node.get_logger().error(str(error))
    finally:
        if capture is not None:
            capture.document.update(status='incomplete', error='Recording stopped before capture completed')
            save_yaml(run / 'control_parameters.yaml', capture.document)
            capture.close()
        try:
            if recorder is not None:
                exit_code = recorder.stop()
                document['recorder_exit_code'] = exit_code
                document['recorded_messages'] = counts = recorded_counts(run / 'rosbag')
                missing = [] if args.no_capture_parameters else [t for t in required if t != '/parameter_events' and not counts.get(t)]
                if exit_code != 0 or missing or not snapshot_sent:
                    raise RuntimeError(f'MCAP exit={exit_code}, missing={missing}, snapshot={snapshot_sent}')
        except Exception as error:
            code = 1
            document['recording_error'] = str(error)
        document['status'] = 'complete' if code == 0 else 'incomplete'
        save_yaml(run / 'recording.yaml', document)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        signal.signal(signal.SIGTERM, previous_sigterm)
    return code
