"""ROS telemetry and the finite Run lifecycle for the field TUI."""

from collections import deque
from datetime import datetime
import math
from queue import SimpleQueue
import threading
import time

from geometry_msgs.msg import TwistWithCovarianceStamped
from kaiev26_msgs.msg import (
    ActuatorCommand, BrakeActuatorStatus, DriveActuatorStatus,
    SteeringActuatorStatus, VehicleState,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from .scenario_runner import ScenarioSetpoint
from .sysid_profiles import COUNTDOWN_SEC, CENTER_DEG, TAIL_SEC, RunSettings, baseline, evaluate, segments


TOPICS = {
    '/drive/status': DriveActuatorStatus,
    '/steering/status': SteeringActuatorStatus,
    '/brake/status': BrakeActuatorStatus,
    '/vehicle/state': VehicleState,
    '/gnss/fix': NavSatFix,
    '/gnss/fix_velocity': TwistWithCovarianceStamped,
    '/imu/data_raw': Imu,
    '/tf_static': TFMessage,
}
OPTIONAL_TOPICS = {'/imu/data_raw'}
BUSY_STATES = {'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN', 'CRUISE', 'RUNNING', 'DECEL', 'TAIL', 'CLOSING'}
STATE_LABELS = {
    'IDLE': '시험 선택', 'RECORD_START': 'MCAP 연결 중', 'WAIT_AUTO': 'AUTO 전환 대기',
    'COUNTDOWN': '출발 카운트다운', 'CRUISE': '정속 준비', 'RUNNING': '시험 중',
    'DECEL': '감속 중', 'TAIL': '후행 기록 중', 'CLOSING': 'MCAP 저장 중',
    'COMPLETE': '시험 입력 완료 · 목표속도 0 유지',
    'ABORTED': '시험 중단', 'ERROR': '확인 필요',
}


class SysidNode(Node):
    required_topics = tuple(topic for topic in TOPICS if topic not in OPTIONAL_TOPICS)

    def __init__(self, *, clock=time.monotonic, **kwargs):
        super().__init__('auto_sysid', **kwargs)
        self.clock = clock
        self.lock = threading.RLock()
        self.commands = SimpleQueue()
        self.process_actions = SimpleQueue()
        self.file_logs = SimpleQueue()
        self.logs = deque(maxlen=200)
        self.received = {}
        self.arrivals = {topic: deque(maxlen=1000) for topic in TOPICS}
        self.sensor_errors = {}
        self.static_frames = set()
        self.driver_rows = []
        self.graph_errors = ['ROS 그래프 확인 중']
        self.manager_ok = True
        self.state = 'IDLE'
        self.settings = RunSettings()
        self.schedule = segments(self.settings)
        self.output = baseline()
        self.output_active = False
        self.run_dir = ''
        self.finish_reason = ''
        self.terminal_state = 'COMPLETE'
        self.phase_started = 0.0
        self.input_started = 0.0
        self.stable_since = None
        self.segment = 0
        self.countdown = None
        self.auto_seen = False
        self.record_open = False
        self.record_error = ''
        self.command_publisher = self.create_publisher(ActuatorCommand, '/planning/command', 10)
        self.label_publisher = self.create_publisher(String, '/sysid/scinario', 10)
        self.subscriptions_owned = []
        for topic, message_type in TOPICS.items():
            qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/tf_static' else qos_profile_sensor_data)
            self.subscriptions_owned.append(self.create_subscription(
                message_type, topic, lambda message, topic=topic: self._receive(topic, message), qos))
        self.subscriptions_owned.append(self.create_subscription(
            ActuatorCommand, '/vehicle/command',
            lambda message: self._receive('/vehicle/command', message), qos_profile_sensor_data))
        self.create_timer(0.05, self._tick)

    @property
    def busy(self):
        return self.state in BUSY_STATES

    def log(self, message):
        line = datetime.now().astimezone().isoformat(timespec='milliseconds') + ' ' + message
        with self.lock:
            self.logs.append(line)
        self.file_logs.put(line)

    def _receive(self, topic, message):
        # Interpret external telemetry at receipt; the controller uses this cached result.
        error = ''
        if topic == '/drive/status':
            if not math.isfinite(message.speed_mps):
                error = '차속이 유효하지 않음'
            elif not message.part1_valid or not message.part2_valid:
                error = 'EZkontrol 상태 미수신'
            elif message.fault_bits_valid and message.fault_bits:
                error = f'구동 fault 0x{message.fault_bits:x}'
            elif message.gear != DriveActuatorStatus.GEAR_FORWARD:
                error = '전진 기어 필요'
            elif message.mode not in (DriveActuatorStatus.MODE_MANUAL, DriveActuatorStatus.MODE_AUTO):
                error = '유효하지 않은 drive mode'
        elif topic == '/steering/status':
            if message.estop_active:
                error = '조향 E-stop'
            elif message.fault_bits & ~1:
                error = f'조향 fault 0x{message.fault_bits:x}'
            elif not message.communication_ok or not message.feedback_valid:
                error = '조향 통신/피드백 미수신'
        elif topic == '/brake/status':
            if message.stop_reason_bits & BrakeActuatorStatus.STOP_REASON_ESTOP:
                error = '물리 E-stop'
        elif topic == '/gnss/fix':
            if message.status.status < 0 or not (
                math.isfinite(message.latitude) and math.isfinite(message.longitude)
                and -90 <= message.latitude <= 90 and -180 <= message.longitude <= 180
            ):
                error = '유효한 GNSS fix 없음'
        elif topic == '/gnss/fix_velocity':
            v = message.twist.twist.linear
            if not math.isfinite(v.x) or not math.isfinite(v.y):
                error = 'GNSS 속도 유효하지 않음'
        elif topic == '/imu/data_raw':
            vectors = (message.angular_velocity, message.linear_acceleration)
            if not all(math.isfinite(value) for v in vectors for value in (v.x, v.y, v.z)):
                error = 'IMU 측정값 유효하지 않음'
        with self.lock:
            now = self.clock()
            self.received[topic] = (now, message)
            if topic in self.arrivals:
                self.arrivals[topic].append(now)
                while self.arrivals[topic] and self.arrivals[topic][0] < now - 2:
                    self.arrivals[topic].popleft()
            if topic == '/tf_static':
                self.static_frames.update(t.child_frame_id for t in message.transforms)
                if 'gps' not in self.static_frames:
                    error = '차량 GNSS TF 대기'
            self.sensor_errors[topic] = error

    def scan_graph(self):
        errors = []
        planning = self.get_publishers_info_by_topic('/planning/command')
        vehicle = self.get_publishers_info_by_topic('/vehicle/command')
        if len(planning) != 1 or planning[0].node_name != self.get_name():
            errors.append('/planning/command에 다른 명령 발행자가 있음')
        if len(vehicle) != 1:
            errors.append('/vehicle/command 중계 발행자 확인 필요')
        elif not any(
            info.node_name == vehicle[0].node_name
            and info.node_namespace == vehicle[0].node_namespace
            for info in self.get_subscriptions_info_by_topic('/planning/command')
        ):
            errors.append('/vehicle/command 발행자가 /planning/command와 연결되어 있지 않음')
        with self.lock:
            self.graph_errors = errors

    def gate_rows(self, now=None, *, command_flow=False):
        now = self.clock() if now is None else now
        rows = [(
            '명령 발행·중계', not self.graph_errors,
            '정상' if not self.graph_errors else '; '.join(self.graph_errors),
        ), (
            '프로세스 관리자', self.manager_ok,
            '정상' if self.manager_ok else '프로세스 관리자 종료',
        )]
        for topic in self.required_topics:
            sample = self.received.get(topic)
            if sample is None:
                rows.append((topic, False, f'{topic} 미수신'))
            elif topic != '/tf_static' and now - sample[0] > 1.0:
                rows.append((topic, False, f'{topic} 수신 중단'))
            elif self.sensor_errors.get(topic):
                rows.append((topic, False, f'{topic}: {self.sensor_errors[topic]}'))
            else:
                rows.append((topic, True, '정상'))
        drive = self.received.get('/drive/status')
        brake = self.received.get('/brake/status')
        if drive and drive[1].stop_request:
            host_brake = brake and (
                brake[1].stop_reason_bits & BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE)
            if not host_brake:
                rows.append(('구동 정지 요청', False, '차량 구동 정지 요청'))
            else:
                rows.append(('구동 정지 요청', True, '브레이크 반영'))
        else:
            rows.append(('구동 정지 요청', True, '없음'))
        if command_flow:
            command = self.received.get('/vehicle/command')
            if command is None or now - command[0] > 0.2:
                rows.append(('/vehicle/command', False, '명령 중계 수신 중단'))
            else:
                rows.append(('/vehicle/command', True, '실시간 중계 중'))
        return rows

    def issues(self, now=None, *, command_flow=False):
        return [detail for _, go, detail in self.gate_rows(
            now, command_flow=command_flow) if not go]

    def request_run(self, settings):
        self.commands.put(('run', settings))

    def request_stop(self):
        self.commands.put(('stop', None))

    def _finish(self, reason, now, *, brake=False, terminal='COMPLETE'):
        self.finish_reason = reason
        self.terminal_state = terminal
        self.output = baseline(brake)
        self.output_active = True
        self.state = 'TAIL' if self.record_open else terminal
        self.phase_started = now
        self.log(reason)

    def _actions(self, now):
        while not self.commands.empty():
            action, value = self.commands.get()
            if action == 'run':
                if self.busy:
                    self.log('현재 Run이 끝난 뒤 다시 실행하세요')
                    continue
                problems = self.issues(now)
                if problems:
                    self.log('Run 대기: ' + '; '.join(problems))
                    continue
                self.settings = value
                self.schedule = segments(value)
                self.output = baseline()
                self.output_active = True
                self.auto_seen = False
                self.countdown = None
                self.segment = 0
                self.run_dir = ''
                self.record_open = True
                self.record_error = ''
                self.terminal_state = 'COMPLETE'
                self.state = 'RECORD_START'
                self.phase_started = now
                self.process_actions.put(('record', value))
                self.log(f'시나리오 {value.scenario} · 최고 목표속도 {value.max_speed_kph:g} km/h')
            elif action == 'stop':
                if self.state in ('TAIL', 'CLOSING'):
                    self.output = baseline(True)
                    self.terminal_state = 'ABORTED'
                    self.finish_reason = '사용자 Stop'
                else:
                    self._finish('사용자 Stop', now, brake=True, terminal='ABORTED')
            elif action == 'record_started':
                self.run_dir = value
                self.log(f'MCAP: {value}')
            elif action == 'record_ready' and self.state == 'RECORD_START':
                self.state = 'WAIT_AUTO'
                self.log('녹화 연결 완료 · AUTO 대기')
            elif action == 'record_closed':
                self.record_open = False
                self.state = self.terminal_state
                if self.record_error:
                    self.log(f'MCAP 오류로 종료: {self.record_error}')
                else:
                    self.log(f'MCAP 저장 완료: {value}')
            elif action == 'record_failed':
                self.record_error = str(value)
                self.finish_reason = f'MCAP 오류: {value}'
                self.terminal_state = 'ERROR'
                self.output = baseline(True)
                self.output_active = True
                self.state = 'CLOSING'
                self.log(self.finish_reason)
            elif action == 'manager_failed':
                self.manager_ok = False
                self._finish(f'프로세스 관리 오류: {value}', now, brake=True, terminal='ERROR')

    def _tick(self):
        with self.lock:
            now = self.clock()
            self._actions(now)
            if self.state in {'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN', 'CRUISE', 'RUNNING', 'DECEL'}:
                active_input = self.state in {'CRUISE', 'RUNNING', 'DECEL'}
                problems = self.issues(now, command_flow=active_input)
                if problems:
                    self._finish('시험 중단: ' + '; '.join(problems), now,
                                 brake=True, terminal='ERROR')
                else:
                    mode = self.received['/drive/status'][1].mode
                    auto = mode == DriveActuatorStatus.MODE_AUTO
                    if self.auto_seen and not auto:
                        self._finish('MANUAL 전환으로 Run 종료', now, terminal='ABORTED')
                    elif self.state == 'WAIT_AUTO' and auto:
                        self.auto_seen = True
                        self.state = 'COUNTDOWN'
                        self.phase_started = now
                        self.log('자동 확인!')
            if self.state == 'COUNTDOWN':
                left = max(0, math.ceil(COUNTDOWN_SEC - (now - self.phase_started)))
                if left != self.countdown and left > 0:
                    self.countdown = left
                    self.log(f'{left}초 후 시작')
                if now - self.phase_started >= COUNTDOWN_SEC:
                    problems = self.issues(now, command_flow=True)
                    if problems:
                        self._finish('출발 취소: ' + '; '.join(problems), now,
                                     brake=True, terminal='ERROR')
                    else:
                        self.phase_started = now
                        self.input_started = now
                        self.stable_since = None
                        self.state = 'CRUISE' if self.settings.needs_cruise else 'RUNNING'
                        self.log('정속 준비' if self.settings.needs_cruise else '시험 입력 시작')
            if self.state == 'CRUISE':
                speed = self.received['/drive/status'][1].speed_mps * 3.6
                self.output = ScenarioSetpoint(self.settings.max_speed_kph, CENTER_DEG)
                if abs(speed - self.settings.max_speed_kph) <= 0.5:
                    if self.stable_since is None:
                        self.stable_since = now
                    elif now - self.stable_since >= 1.0:
                        self.state = 'RUNNING'
                        self.input_started = now
                        self.log('정속 확인 · 조향 입력 시작')
                else:
                    self.stable_since = None
                if self.state == 'CRUISE' and now - self.phase_started >= 15:
                    self.state = 'DECEL'
                    self.phase_started = now
                    self.log('정속 준비 시간 초과 · 감속')
            if self.state == 'DECEL':
                fraction = max(0, 1 - (now - self.phase_started) / 4)
                self.output = ScenarioSetpoint(self.settings.max_speed_kph * fraction, CENTER_DEG)
                if fraction == 0:
                    self._finish('정속 미도달로 Run 종료', now, terminal='ABORTED')
            if self.state == 'RUNNING':
                elapsed = now - self.input_started
                self.output, segment = evaluate(self.schedule, elapsed)
                if segment != self.segment:
                    self.segment = segment
                    self.log(f'입력 구간 {segment}/{len(self.schedule)}')
                if elapsed >= self.settings.duration:
                    self._finish('시험 입력 완료 · 목표속도 0 유지', now)
            if self.state == 'TAIL' and now - self.phase_started >= TAIL_SEC:
                self.state = 'CLOSING'
                self.process_actions.put(('close', self.finish_reason))
            if self.output_active:
                message = ActuatorCommand()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = 'base_link'
                message.speed_target_mps = self.output.speed_kph / 3.6
                message.steering_target_rad = math.radians(self.output.steering_deg)
                message.brake_engage = self.output.brake_engage
                self.command_publisher.publish(message)
                self.label_publisher.publish(String(
                    data=f'scenario_{self.settings.scenario:02d}_v{self.settings.max_speed_kph:g}'))

    def snapshot(self):
        with self.lock:
            now = self.clock()
            rows = []
            for topic in TOPICS:
                sample = self.received.get(topic)
                error = self.sensor_errors.get(topic, '')
                age = now - sample[0] if sample else None
                times = self.arrivals[topic]
                rate = ((len(times) - 1) / (times[-1] - times[0])
                        if len(times) > 1 and times[-1] > times[0] else 0)
                optional = topic in OPTIONAL_TOPICS
                if sample is None and optional:
                    symbol, text = '🟡', '선택 기록 · 미수신'
                elif sample is None:
                    symbol, text = '❌', '미수신'
                elif error and optional:
                    symbol, text = '🟡', '선택 기록 · ' + error
                elif error:
                    symbol, text = '❌', error
                elif topic == '/tf_static':
                    symbol, text = '✅', '정적 TF 수신'
                elif age > 1 and optional:
                    symbol, text = '🟡', f'선택 기록 · {age:.1f}초 수신 중단'
                elif age > 1:
                    symbol, text = '❌', f'{age:.1f}초 수신 중단'
                elif age > 0.3:
                    symbol, text = '🟡', f'{rate:.1f} Hz · {age:.2f}초 전'
                else:
                    symbol, text = '✅', f'{rate:.1f} Hz'
                rows.append((topic, symbol, text))
            drive = self.received.get('/drive/status')
            steering = self.received.get('/steering/status')
            mode = ('미수신' if not drive or now - drive[0] > 1 else
                    'AUTO' if drive[1].mode == DriveActuatorStatus.MODE_AUTO else 'MANUAL')
            command_flow = self.state in {'CRUISE', 'RUNNING', 'DECEL'}
            return {
                'state': self.state, 'label': STATE_LABELS[self.state], 'mode': mode,
                'speed': drive[1].speed_mps * 3.6 if drive else None,
                'rpm': drive[1].motor_rpm if drive else None,
                'current': drive[1].phase_current_a if drive else None,
                'steering': math.degrees(steering[1].actual_wheel_angle_rad) if steering else None,
                'output': self.output, 'rows': rows, 'drivers': list(self.driver_rows),
                'issues': self.issues(now), 'logs': list(self.logs)[-7:],
                'run_dir': self.run_dir, 'recording': self.record_open,
                'settings': self.settings,
                'countdown': self.countdown if self.state == 'COUNTDOWN' else None,
                'elapsed': max(0, now - self.input_started) if self.state == 'RUNNING' else None,
                'gates': self.gate_rows(now, command_flow=command_flow),
                'finish_reason': self.finish_reason,
                'terminal_state': self.terminal_state,
            }
