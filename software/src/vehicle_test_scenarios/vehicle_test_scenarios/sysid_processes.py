"""Owned driver processes and the standard rosbag2 MCAP recorder."""

from __future__ import annotations

from dataclasses import dataclass
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
from zoneinfo import ZoneInfo

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml


RECORD_TOPICS = (
    '/planning/command', '/vehicle/command', '/sysid/scinario',
    '/drive/status', '/steering/status', '/steering/angle', '/brake/status',
    '/vehicle/state', '/vehicle/wheel_twist',
    '/gnss/fix', '/gnss/fix_velocity', '/gnss/status', '/gnss/status_verbose',
    '/imu/data', '/imu/data_raw', '/ebimu/status', '/tf', '/tf_static', '/diagnostics',
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
            self.emit(f'{self.name}: {line.rstrip()}')
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
    package: str
    executable: str
    node: str
    topic: str = ''
    device: str = ''


COMPONENTS = (
    Component('차량 TF', 'robot_state_publisher', 'robot_state_publisher', 'vehicle_robot_state_publisher'),
    Component('STM A', 'micro_ros_agent', 'micro_ros_agent', 'stm_a_node', '/drive/status', '/dev/stm_a'),
    Component('STM B', 'micro_ros_agent', 'micro_ros_agent', 'stm_b_node', '/brake/status', '/dev/stm_b'),
    Component('명령 중계', 'command_governor', 'command_governor_node', 'command_governor', '/vehicle/command'),
    Component('조향', 'lateral_ctrl', 'steering_actuator_node', 'steering_actuator_node', '/steering/status', 'can0'),
    Component('차량 상태', 'vehicle_state_publisher', 'vehicle_state_publisher_node', 'vehicle_state_publisher', '/vehicle/state'),
    Component('차량 진단', 'visualization', 'diagnostics_publisher', 'diagnostics_publisher'),
    Component('GNSS', 'um980_driver', 'um980_driver_node', 'um980_driver', '/gnss/fix', '/dev/gps'),
    Component('IMU', 'ebimu_driver', 'ebimu_node', 'ebimu_driver', '/imu/data_raw', '/dev/imu'),
)


class ProcessManager:
    def __init__(self, node, records_root=Path('/home/kai/records')):
        self.node = node
        self.record_topics_required = node.required_topics + ('/planning/command', '/vehicle/command')
        self.records_root = Path(records_root)
        self.temporary = tempfile.TemporaryDirectory(prefix='auto_sysid_')
        self.temp_root = Path(self.temporary.name)
        self.children = {}
        self.retry_after = {}
        self.messages = {}
        self.recorder = None
        self.run_dir = None
        self.run_document = None
        self.run_log = None
        self.recorder_ready = False
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

    def _params(self, key, node_name, values):
        path = self.temp_root / f'{key}.yaml'
        path.write_text(yaml.safe_dump({node_name: {'ros__parameters': values}}))
        return str(path)

    def _command(self, component):
        prefix = Path(get_package_prefix(component.package))
        executable = prefix / 'lib' / component.package / component.executable
        if not executable.is_file():
            raise FileNotFoundError(str(executable))
        if component.package == 'micro_ros_agent':
            return [str(executable), 'serial', '--dev', component.device, '-b', '115200']
        args = [str(executable), '--ros-args', '-r', f'__node:={component.node}']
        share = Path(get_package_share_directory(component.package))
        values = {}
        if component.key == '차량 TF':
            import xacro
            description = Path(get_package_share_directory('kaiev26_description'))
            xml = xacro.process_file(
                str(description / 'urdf/kaiev26.urdf.xacro'),
                mappings={'lidar_output_mode': 'none', 'debug_driver_camera_enabled': 'false'},
            ).toxml()
            values = {'robot_description': xml}
        elif component.key == '명령 중계':
            args += ['--params-file', str(share / 'config/command_governor.yaml')]
        elif component.key == '조향':
            spec = importlib.util.spec_from_file_location(
                'sysid_steering_launch', share / 'launch/steering_actuator.launch.py')
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            values = module.load_steering_motor_calibration(
                str(share / 'config/steering_motor_calibration.yaml'))
            values.update({'can_interface': 'can0', 'hardware_can_enabled': True,
                           'actuator_output_enabled': True, 'tracking_control_enabled': False,
                           'test_input_injection_enabled': False, 'use_sim_time': False})
            args += ['--params-file', str(share / 'config/steering_actuator.yaml')]
        elif component.key == 'GNSS':
            args += ['--params-file', str(share / 'config/um980_20hz.yaml')]
            values = {'configure_on_start': True, 'save_config': False, 'mode': 'survey_20hz',
                      'ntrip_enabled': True,
                      'ntrip_private_yaml': str(share / 'config/ntrip_private.yaml')}
        elif component.key == 'IMU':
            values = {'port': '/dev/imu', 'baudrate': 460800, 'output_mode': 'binary',
                      'output_interval_ms': 2, 'orientation_source': 'quaternion',
                      'enable_gyro': True, 'enable_accel': True, 'enable_magnetometer': False,
                      'enable_temperature': False, 'enable_timestamp': True,
                      'frame_id': 'ebimu_imu_link', 'publish_data_raw': True,
                      'publish_internal_static_tf': False}
        if values:
            args += ['--params-file', self._params(component.package, component.node, values)]
        return args

    @staticmethod
    def _external_agent(device):
        for path in Path('/proc').glob('[0-9]*/cmdline'):
            try:
                args = path.read_bytes().decode(errors='replace').split('\0')
                if any(Path(arg).name == 'micro_ros_agent' for arg in args[:2]):
                    if '--dev' in args and args[args.index('--dev') + 1] == device:
                        return True
            except (OSError, IndexError):
                continue
        return False

    @staticmethod
    def _device_available(component):
        if not component.device:
            return True
        if component.device == 'can0':
            try:
                return bool(int(Path('/sys/class/net/can0/flags').read_text().strip(), 16) & 1)
            except OSError:
                return False
        return Path(component.device).exists()

    def _drivers(self):
        names = {name for name, namespace in self.node.get_node_names_and_namespaces()
                 if namespace == self.node.get_namespace()}
        with self.node.lock:
            received = dict(self.node.received)
        now = time.monotonic()
        rows = []
        for component in COMPONENTS:
            child = self.children.get(component.key)
            if child and child.process.poll() is not None:
                child.stop()
                del self.children[component.key]
                self.retry_after[component.key] = now + 3
                self._say(component.key, f'프로세스 종료 ({child.process.returncode})')
                child = None
            sample = received.get(component.topic)
            stream_live = sample is not None and now - sample[0] < 1
            external = (component.node in names or stream_live
                        or (bool(component.topic) and bool(
                            self.node.get_publishers_info_by_topic(component.topic))))
            if component.key == '차량 TF':
                with self.node.lock:
                    external |= {'gps', 'ebimu_imu_link'}.issubset(self.node.static_frames)
            if child:
                state = '실행 중'
            elif external or (component.package == 'micro_ros_agent' and self._external_agent(component.device)):
                state = '기존 프로세스 사용'
            elif self.node.busy:
                state = '시험 종료 후 시작'
            elif not self._device_available(component):
                state = component.device + ' 연결 대기'
            elif now < self.retry_after.get(component.key, 0):
                state = '재시작 대기'
            else:
                try:
                    self.children[component.key] = OwnedProcess(
                        component.key, self._command(component), self.node.log)
                    state = '시작'
                except Exception as error:
                    state = str(error)
                    self.retry_after[component.key] = now + 3
            rows.append((component.key, state))
            self._say(component.key, state)
        with self.node.lock:
            self.node.driver_rows = rows

    def _start_recording(self, settings):
        if self.recorder is not None:
            return
        stamp = datetime.now(ZoneInfo('Asia/Seoul')).strftime('%Y%m%d_%H%M%S_%f')
        speed = f'{settings.max_speed_kph:g}'.replace('.', 'p')
        self.run_dir = self.records_root / f'{stamp}_scenario_{settings.scenario:02d}_v{speed}'
        self.run_dir.mkdir(parents=True, exist_ok=False)
        self.run_log = (self.run_dir / 'console.log').open('a', buffering=1)
        self.run_document = settings.document()
        self.run_document.update({
            'started_at': datetime.now(ZoneInfo('Asia/Seoul')).isoformat(),
            'topics': list(RECORD_TOPICS),
        })
        self._save_document()
        self.subscribers_before_recording = {
            topic: {bytes(info.endpoint_gid) for info in
                    self.node.get_subscriptions_info_by_topic(topic)
                    if info.node_name == 'rosbag2_recorder'}
            for topic in self.record_topics_required
        }
        self.recorder = OwnedProcess('MCAP', [
            sys.executable, '-u', '-c', RECORDER_CODE, str(self.run_dir / 'rosbag'),
            json.dumps(RECORD_TOPICS),
        ], self.node.log)
        self.record_started_at = time.monotonic()
        self.recorder_ready = False
        self.node.commands.put(('record_started', str(self.run_dir)))

    def _save_document(self):
        (self.run_dir / 'scenario.yaml').write_text(
            yaml.safe_dump(self.run_document, sort_keys=False, allow_unicode=True))

    def _finish_recording(self, reason):
        if self.recorder is None:
            return
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
            topics = self.record_topics_required
            subscribed = all(any(
                info.node_name == 'rosbag2_recorder'
                and bytes(info.endpoint_gid) not in self.subscribers_before_recording[topic]
                for info in self.node.get_subscriptions_info_by_topic(topic)
            ) for topic in topics)
            if subscribed:
                self.recorder_ready = True
                self.node.commands.put(('record_ready', None))
            elif time.monotonic() - self.record_started_at > 15:
                self.node.commands.put(('record_failed', 'MCAP 필수 토픽 구독이 연결되지 않았습니다'))
                self._finish_recording('subscription_timeout')

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
                            if self.run_log:
                                self.run_log.close()
                                self.run_log = None
                            self.node.commands.put(('record_closed', str(self.run_dir or '')))
                    elif action == 'close':
                        self._finish_recording(value)
                self._record_status()
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
            if self.children:
                with ThreadPoolExecutor(max_workers=len(self.children)) as pool:
                    results = [pool.submit(child.stop) for child in self.children.values()]
                    for result in results:
                        try:
                            result.result()
                        except Exception as error:
                            self.cleanup_errors.append(str(error))
                            self.node.log(str(error))
            self.temporary.cleanup()

    def close(self):
        self.stop_event.set()
        if self.thread.ident is not None:
            self.thread.join()
        else:
            self.temporary.cleanup()
        return list(self.cleanup_errors)
