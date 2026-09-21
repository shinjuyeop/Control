"""Observe external ROS drivers and manage the rosbag2 MCAP recorder."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from zoneinfo import ZoneInfo

import yaml
from std_msgs.msg import String
from control_tuning.storage import SettingsStore
from .sysid_capture import ParameterCapture, TORQUE_TOPICS


DEFAULT_RECORDS_ROOT = next((
    ancestor / 'records'
    for ancestor in Path(__file__).resolve().parents
    if (ancestor / 'src/Control/software/src/vehicle_test_scenarios/package.xml').is_file()
), Path.home() / 'KAI_ws/records')


RECORD_TOPICS = (
    '/planning/command', '/vehicle/command', '/sysid/scinario',
    '/drive/status', "/drive/longitudinal_status", "/drive/longitudinal_config", '/steering/status', '/steering/angle', '/brake/status',
    '/vehicle/state', '/vehicle/wheel_twist',
    '/gnss/fix', '/gnss/fix_velocity', '/gnss/status', '/gnss/status_verbose',
    '/imu/data', '/imu/data_raw', '/ebimu/status', '/tf', '/tf_static', '/diagnostics',
    '/perception/camera/left/source/camera_info',
    '/perception/camera/left/source/image_raw/compressed',
    '/perception/camera/right/source/camera_info',
    '/perception/camera/right/source/image_raw/compressed',
    '/ouster/imu_packets', '/ouster/lidar_packets', '/ouster/metadata',
    '/sysid/odometry', '/sysid/odometry_gnss', '/sysid/route_context',
    '/sysid/target_path', '/sysid/target_speed', '/sysid/route_command',
    '/sysid/route_enabled', '/sysid/tf',
    '/parameter_events', '/sysid/control_parameters', *TORQUE_TOPICS,
)
RECORDER_CODE = """
import sys, json
import rosbag2_py
storage = rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='mcap')
options = rosbag2_py.RecordOptions()
options.topics = json.loads(sys.argv[2])
options.rmw_serialization_format = 'cdr'
try:
    rosbag2_py.Recorder().record(storage, options)
except KeyboardInterrupt:
    pass
"""


def group_members(pgid):
    result = []
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            fields = path.read_text().rsplit(')', 1)[1].split()
            if int(fields[2]) == pgid and fields[0] != 'Z':
                result.append(int(path.parent.name))
        except (OSError, ValueError, IndexError):
            continue
    return result


class OwnedProcess:
    """Direct executables each own a session; no ROS CLI daemon is started."""

    def __init__(self, name, argv, emit):
        self.name, self.emit = name, emit
        self.last_output = ''
        self.process = subprocess.Popen(
            argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace',
            start_new_session=True, bufsize=1,
        )
        self.pgid = self.process.pid
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        for line in self.process.stdout:
            self.last_output = line.rstrip()
            self.emit(f'{self.name}: {self.last_output}')
        self.process.stdout.close()

    def stop(self):
        for sig, timeout in ((signal.SIGINT, 5), (signal.SIGTERM, 2), (signal.SIGKILL, 2)):
            if not group_members(self.pgid):
                break
            try:
                os.killpg(self.pgid, sig)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.process.poll()
                if not group_members(self.pgid):
                    break
                time.sleep(0.05)
        self.process.wait(timeout=1)
        self.reader.join(timeout=1)
        remaining = group_members(self.pgid)
        if remaining:
            raise RuntimeError(f'{self.name}: remaining owned PIDs {remaining}')
        return self.process.returncode


@dataclass(frozen=True)
class Component:
    key: str
    node: str
    topic: str = ''


COMPONENTS = (
    Component('차량 TF', 'vehicle_robot_state_publisher'),
    Component('STM A', 'stm_a_node', '/drive/status'),
    Component('STM B', 'stm_b_node', '/brake/status'),
    Component('명령 중계', 'command_governor', '/vehicle/command'),
    Component('조향', 'steering_actuator_node', '/steering/status'),
    Component('차량 상태', 'vehicle_state_publisher', '/vehicle/state'),
    Component('차량 진단', 'diagnostics_publisher'),
    Component('GNSS', 'um980_driver', '/gnss/fix'),
    Component('IMU', 'ebimu_driver', '/imu/data_raw'),
)


class ProcessManager:
    def __init__(self, node, records_root=DEFAULT_RECORDS_ROOT):
        self.node = node
        self.record_topics_required = node.required_topics + ('/planning/command', '/vehicle/command')
        self.records_root = Path(records_root)
        self.messages = {}
        self.recorder = None
        self.route = None
        self.run_dir = None
        self.run_document = None
        self.run_log = None
        self.recorder_ready = False
        self.parameter_capture = None
        self.cleanup_errors = []
        self.record_started_at = 0.0
        self.subscribers_before_recording = {}
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._work, name='sysid-processes')

    def start(self):
        self.thread.start()

    def _say(self, key, message):
        if self.messages.get(key) != message:
            self.messages[key] = message
            self.node.log(f'{key}: {message}')

    def _drivers(self):
        names = {name for name, namespace in self.node.get_node_names_and_namespaces()
                 if namespace == self.node.get_namespace()}
        with self.node.lock:
            received = dict(self.node.received)
        now = self.node.clock()
        rows = []
        for component in COMPONENTS:
            sample = received.get(component.topic)
            stream_live = sample is not None and now - sample[0] < 1
            external = (component.node in names or stream_live
                        or (bool(component.topic) and bool(
                            self.node.get_publishers_info_by_topic(component.topic))))
            if component.key == '차량 TF':
                with self.node.lock:
                    external |= {'gps', 'ebimu_imu_link'}.issubset(self.node.static_frames)
            state = '감지됨' if external else '외부 실행 대기'
            rows.append((component.key, state))
            self._say(component.key, state)
        with self.node.lock:
            self.node.driver_rows = rows

    def _start_recording(self, settings):
        if self.recorder is not None:
            return
        self.run_dir = None
        self.record_topics_required = self.node.required_topics_for(settings.follows_route) + (
            '/planning/command', '/vehicle/command', '/sysid/control_parameters')
        if settings.follows_route:
            from .sysid_routes import RouteProcesses
            self.route = RouteProcesses(self.node, settings)
        if not settings.record_mcap:
            self.node.commands.put(('record_ready', None))
            return
        stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
        speed = f'{settings.max_speed_kph:g}'.replace('.', 'p')
        self.run_dir = self.records_root / (
            f'{stamp}_{settings.record_slug}_v{speed}'
        )
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.run_log = (self.run_dir / 'console.log').open('a', buffering=1)
        self.run_document = settings.document()
        self.run_document.update({
            'started_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'topics': list(RECORD_TOPICS),
            'control_parameters': {'file': 'control_parameters.yaml', 'status': 'pending'},
        })
        self._save_document()
        self.subscribers_before_recording = {
            topic: {bytes(info.endpoint_gid) for info in
                    self.node.get_subscriptions_info_by_topic(topic)
                    if info.node_name == 'rosbag2_recorder'}
            for topic in (*self.record_topics_required, '/parameter_events')
        }
        self.recorder = OwnedProcess('MCAP', [
            sys.executable, '-u', '-c', RECORDER_CODE, str(self.run_dir / 'rosbag'),
            json.dumps(RECORD_TOPICS),
        ], self.node.log)
        self.record_started_at = time.monotonic()
        self.recorder_ready = False
        self.node.commands.put(('record_started', str(self.run_dir)))

    def _save_document(self):
        SettingsStore._replace(self.run_dir / 'scenario.yaml',
                               yaml.safe_dump(self.run_document, sort_keys=False, allow_unicode=True).encode('utf-8'))

    def _finish_recording(self, reason):
        if self.parameter_capture is not None:
            capture, self.parameter_capture = self.parameter_capture, None
            try:
                if not capture.finished:
                    capture.document.update(status='cancelled', error=reason)
                    self._save_capture(capture.document)
            finally:
                capture.close()
        if self.recorder is None:
            self._close_route()
            self.node.commands.put(('record_closed', ''))
            return
        if self.run_document['control_parameters']['status'] == 'pending':
            self.run_document['control_parameters'].update(status='cancelled', error=reason)
        child, self.recorder = self.recorder, None
        self.recorder_ready = False
        code = child.stop()
        self.run_document.update({
            'finished_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'finish_reason': reason, 'recorder_exit_code': code,
        })
        try:
            import rosbag2_py
            metadata = rosbag2_py.Info().read_metadata(str(self.run_dir / 'rosbag'), 'mcap')
            counts = {item.topic_metadata.name: item.message_count
                      for item in metadata.topics_with_message_count}
            self.run_document['recorded_messages'] = counts
            missing = [topic for topic in self.record_topics_required if counts.get(topic, 0) == 0]
            if code != 0 or missing:
                raise RuntimeError(f'MCAP 종료 코드 {code}, 미기록 필수 토픽 {missing}')
        except Exception as error:
            self.run_document['recording_error'] = str(error)
            self.node.commands.put(('record_failed', str(error)))
        finally:
            self._save_document()
            self._flush_logs()
            self.run_log.close()
            self.run_log = None
            self.node.commands.put(('record_closed', str(self.run_dir)))
            self._close_route()

    def _close_route(self):
        if self.route is not None:
            route, self.route = self.route, None
            route.close()

    def _flush_logs(self):
        while not self.node.file_logs.empty():
            line = self.node.file_logs.get()
            if self.run_log:
                self.run_log.write(line + '\n')

    def _record_status(self):
        if self.recorder is None:
            return
        if self.recorder.process.poll() is not None:
            self.node.commands.put(('record_failed', 'MCAP recorder가 종료되었습니다'))
            self._finish_recording('recorder_failed')
            return
        if not self.recorder_ready:
            topics = (*self.record_topics_required, '/parameter_events')
            subscribed = all(any(
                info.node_name == 'rosbag2_recorder'
                and bytes(info.endpoint_gid) not in self.subscribers_before_recording[topic]
                for info in self.node.get_subscriptions_info_by_topic(topic)
            ) for topic in topics)
            if subscribed:
                if self.parameter_capture is None:
                    self.parameter_capture = ParameterCapture(
                        self.node, self.run_document['test_condition']['key'])
                    self.node.log('실행 중인 조향·거버너 파라미터를 기록합니다.')
                if self.parameter_capture.tick():
                    capture, self.parameter_capture = self.parameter_capture, None
                    try:
                        self._save_capture(capture.document)
                    finally:
                        capture.close()
                    if capture.document['status'] != 'complete':
                        self.node.commands.put(('record_failed', '실제 제어 파라미터 조회 실패 · control_parameters.yaml 확인'))
                        self._finish_recording('parameter_snapshot_failed')
                        return
                    self.node.control_parameters_publisher.publish(String(data=json.dumps(capture.document, ensure_ascii=False)))
                    self.recorder_ready = True
                    self.node.commands.put(('record_ready', None))
            elif time.monotonic() - self.record_started_at > 15:
                self.node.commands.put(('record_failed', 'MCAP 필수 토픽 구독이 연결되지 않았습니다'))
                self._finish_recording('subscription_timeout')

    def _save_capture(self, document):
        SettingsStore._replace(self.run_dir / 'control_parameters.yaml',
                               yaml.safe_dump(document, sort_keys=False, allow_unicode=True).encode('utf-8'))
        self.run_document['control_parameters'] = {
            'file': 'control_parameters.yaml', 'status': document['status'],
            'capture_stage': document['capture_stage'],
            'requested_ros_time_ns': document['requested_ros_time_ns'],
            'nodes': {name: entry['status'] for name, entry in document['nodes'].items()},
        }
        self._save_document()

    def _work(self):
        next_scan = 0.0
        try:
            while not self.stop_event.wait(0.1):
                while not self.node.process_actions.empty():
                    action, value = self.node.process_actions.get()
                    if action == 'record':
                        try:
                            self._start_recording(value)
                        except Exception as error:
                            self.node.commands.put(('record_failed', str(error)))
                            self._close_route()
                            if self.run_log:
                                self.run_log.close()
                                self.run_log = None
                            self.node.commands.put(('record_closed', str(self.run_dir or '')))
                    elif action == 'close':
                        self._finish_recording(value)
                self._record_status()
                if self.route is not None:
                    error = self.route.failure()
                    if error:
                        self.node.commands.put(('route_failed', error))
                        self._close_route()
                self._flush_logs()
                now = time.monotonic()
                if now >= next_scan:
                    self.node.scan_graph()
                    self._drivers()
                    next_scan = now + 1
        except Exception as error:
            self.node.commands.put(('manager_failed', str(error)))
            self.node.log(f'프로세스 관리 오류: {error}')
        finally:
            try:
                self._finish_recording('application_exit')
            except Exception as error:
                self.cleanup_errors.append(str(error))
                self.node.log(f'MCAP 종료 오류: {error}')
            finally:
                try:
                    self._close_route()
                except Exception as error:
                    self.cleanup_errors.append(str(error))
                    self.node.log(f'경로 추종 종료 오류: {error}')

    def close(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join()
        return list(self.cleanup_errors)
