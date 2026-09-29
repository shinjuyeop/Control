"""ROS telemetry and the finite Run lifecycle for the field TUI."""

from collections import deque
from datetime import datetime
import math
from queue import SimpleQueue
import threading
import time

from geometry_msgs.msg import TwistWithCovarianceStamped
from kaiev26_msgs.msg import (
    ActuatorCommand, BrakeActuatorStatus, DriveActuatorStatus, DriveCurrentCommand,
    SteeringActuatorStatus, VehicleState, RouteContext,
)
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
from control_tuning.conditions import CONDITION_KEYS
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Bool, String, Float32MultiArray
from control_tuning.longitudinal import decode_status
from tf2_msgs.msg import TFMessage

from .scenario_runner import ScenarioSetpoint, CURRENT_SCENARIOS, current_setpoint, external_current_ready
from .steering_control import SteeringControl
from .steering_fault_advice import steering_fault_advice
from .sysid_profiles import (
    COUNTDOWN_SEC, TAIL_SEC, ROUTE_FINISH_DISTANCE_M, ROUTE_STOP_SPEED_MPS,
    RunSettings, baseline, default_run_settings, input_setpoint,
)


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
FOLLOW_TOPICS = {
    '/sysid/odometry': Odometry,
    '/sysid/route_context': RouteContext,
    '/sysid/route_command': ActuatorCommand,
}
EZKONTROL_PART1_INVALID_STOP_SEC = 0.5
BUSY_STATES = {
    'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN', 'ROUTE_ARMING',
    'CRUISE', 'RUNNING', 'TAIL', 'CLOSING',
}
STATE_LABELS = {
    'IDLE': '시험 선택', 'RECORD_START': '녹화·제어 설정 기록 중', 'WAIT_AUTO': 'AUTO 전환 대기',
    'COUNTDOWN': '출발 카운트다운', 'ROUTE_ARMING': '경로 출발 명령 대기',
    'CRUISE': '정속 준비', 'RUNNING': '시험 중',
    'TAIL': '후행 기록 중', 'CLOSING': 'MCAP 저장 중',
    'COMPLETE': '시험 입력 완료 · 목표속도 0 유지',
    'ABORTED': '시험 중단', 'ERROR': '확인 필요',
}


class SysidNode(Node):
    required_topics = (
        '/drive/status', '/steering/status', '/brake/status', '/vehicle/state',
    )
    route_required_topics = (
        '/gnss/fix', '/gnss/fix_velocity', '/tf_static', *FOLLOW_TOPICS,
    )

    def __init__(self, *, clock=None, require_ezkontrol_status=True, **kwargs):
        super().__init__('auto_sysid', **kwargs)
        self.clock = clock or (lambda: self.get_clock().now().nanoseconds*1e-9)
        self.require_ezkontrol_status = require_ezkontrol_status
        self.lock = threading.RLock()
        self.commands = SimpleQueue()
        self.process_actions = SimpleQueue()
        self.file_logs = SimpleQueue()
        self.logs = deque(maxlen=200)
        self.received = {}
        self.arrivals = {topic: deque(maxlen=1000) for topic in TOPICS | FOLLOW_TOPICS}
        self.sensor_errors = {}
        self.ezkontrol_part1_invalid_since = None
        self.static_frames = set()
        self.driver_rows = []
        self.graph_errors = ['ROS 그래프 확인 중']
        self.manager_ok = True
        self.state = 'IDLE'
        self.settings = default_run_settings()
        self.output = baseline(settings=self.settings)
        self.output_active = False
        self.run_dir = ''
        self.finish_reason = ''
        self.run_warning = ''
        self.terminal_state = 'COMPLETE'
        self.phase_started = 0.0
        self.input_started = 0.0
        self.stable_since = None
        self.segment = 0
        self.countdown = None
        self.auto_seen = False
        self.record_open = False
        self.session_open = False
        self.record_error = ''
        self.command_publisher = self.create_publisher(ActuatorCommand, '/planning/command', 10)
        self.current_publisher = None
        self.longitudinal_sample = None
        self.create_subscription(Float32MultiArray, '/drive/longitudinal_status', self._receive_longitudinal,
                                 qos_profile_sensor_data)
        self.steering_control = SteeringControl(self)
        self.label_publisher = self.create_publisher(String, '/sysid/scenario', 10)
        self.control_parameters_publisher = self.create_publisher(
            String, '/sysid/control_parameters', 10)
        self.route_enable_publisher = self.create_publisher(Bool, '/sysid/route_enabled', 10)
        self.subscriptions_owned = []
        for topic, message_type in (TOPICS | FOLLOW_TOPICS).items():
            qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/tf_static' else qos_profile_sensor_data)
            self.subscriptions_owned.append(self.create_subscription(
                message_type, topic, lambda message, topic=topic: self._receive(topic, message), qos))
        self.subscriptions_owned.append(self.create_subscription(
            ActuatorCommand, '/vehicle/command',
            lambda message: self._receive('/vehicle/command', message), qos_profile_sensor_data))
        self.create_timer(0.05, self._tick)
        self.launch_preset = 'unspecified'
        self.preset_client = self.create_client(
            GetParameters, '/steering_actuator_node/get_parameters')
        self.preset_future = None
        self.preset_requested_at = 0.0
        self.preset_timer = self.create_timer(1.0, self._read_launch_preset)

    def _receive_longitudinal(self, message):
        try:
            self.longitudinal_sample = (self.clock(), decode_status(message.data))
        except ValueError:
            self.longitudinal_sample = None

    def _read_launch_preset(self):
        if self.preset_future is not None:
            if self.preset_future.done():
                try:
                    values = self.preset_future.result().values
                    preset = values[0].string_value if values else ''
                    if preset in CONDITION_KEYS:
                        with self.lock:
                            self.launch_preset = preset
                        self.preset_timer.cancel()
                except Exception as error:
                    self.get_logger().debug(f'Launch preset query: {error}')
                self.preset_future = None
            elif time.monotonic() - self.preset_requested_at >= 2.0:
                self.preset_client.remove_pending_request(self.preset_future)
                self.preset_future.cancel()
                self.preset_future = None
            return
        if self.preset_client.service_is_ready():
            self.preset_requested_at = time.monotonic()
            self.preset_future = self.preset_client.call_async(
                GetParameters.Request(names=['tuning_preset']))

    @property
    def busy(self):
        return self.state in BUSY_STATES

    def required_topics_for(self, route_input=False):
        return self.required_topics + (self.route_required_topics if route_input else ())

    def log(self, message):
        line = datetime.now().astimezone().isoformat(timespec='milliseconds') + ' ' + message
        with self.lock:
            self.logs.append(line)
        self.file_logs.put(line)

    def _receive(self, topic, message):
        # Interpret external telemetry at receipt; the controller uses this cached result.
        now = self.clock()
        error = ''
        if topic == '/drive/status':
            part1_invalid = not message.part1_valid
            if self.require_ezkontrol_status and part1_invalid:
                if self.ezkontrol_part1_invalid_since is None:
                    self.ezkontrol_part1_invalid_since = now
            else:
                self.ezkontrol_part1_invalid_since = None
            part1_timed_out = (
                self.ezkontrol_part1_invalid_since is not None
                and now - self.ezkontrol_part1_invalid_since
                >= EZKONTROL_PART1_INVALID_STOP_SEC
            )
            if not math.isfinite(message.speed_mps):
                error = '차속이 유효하지 않음'
            elif part1_timed_out:
                error = 'EZkontrol Part I 미수신 500 ms 지속'
            elif self.require_ezkontrol_status and not message.part2_valid:
                error = 'EZkontrol Part II 미수신 1 s'
            elif message.fault_bits_valid and message.fault_bits:
                error = f'구동 fault 0x{message.fault_bits:x}'
            elif message.gear != DriveActuatorStatus.GEAR_FORWARD:
                error = '전진 기어 필요'
            elif message.mode not in (DriveActuatorStatus.MODE_MANUAL, DriveActuatorStatus.MODE_AUTO):
                error = '유효하지 않은 drive mode'
        elif topic == '/steering/status':
            if message.estop_active:
                error = '조향 E-stop'
            elif (message.state == SteeringActuatorStatus.STATE_FAULT
                  or message.disable_reason == SteeringActuatorStatus.REASON_MOTOR_FAULT):
                error = '조향 제어 안전 정지'
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
        elif topic == '/sysid/route_context':
            if not message.route_projection_valid:
                error = '현재 위치와 경로 연결 대기'
        elif topic == '/sysid/route_command':
            if not (math.isfinite(message.speed_target_mps)
                    and message.speed_target_mps >= 0
                    and math.isfinite(message.steering_target_rad)):
                error = '유효하지 않은 경로 추종 명령'
        with self.lock:
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

    def gate_rows(self, now=None, *, route_input=False):
        now = self.clock() if now is None else now
        rows = [(
            '명령 발행·중계', not self.graph_errors,
            '정상' if not self.graph_errors else '; '.join(self.graph_errors),
        ), (
            '프로세스 관리자', self.manager_ok,
            '정상' if self.manager_ok else '프로세스 관리자 종료',
        )]
        if route_input:
            topic = '/sysid/route_command'
            sample = self.received.get(topic)
            error = self.sensor_errors.get(topic)
            if sample is None:
                rows.append((topic, False, f'{topic} 미수신'))
            elif now - sample[0] > 0.2:
                rows.append((topic, False, f'{topic} 수신 중단'))
            elif error:
                rows.append((topic, False, f'{topic}: {error}'))
            else:
                rows.append((topic, True, '정상'))
        return rows

    def issues(self, now=None, *, route_input=False):
        return [detail for _, go, detail in self.gate_rows(
            now, route_input=route_input) if not go]

    def request_run(self, settings):
        self.commands.put(('run', settings))

    def request_stop(self):
        self.commands.put(('stop', None))

    def _finish(self, reason, now, *, terminal='COMPLETE'):
        if self.terminal_state != 'ERROR':
            self.finish_reason = reason
            self.terminal_state = terminal
        self.output = baseline(settings=self.settings)
        self.output_active = True
        if self.record_open:
            self.state = 'TAIL'
        elif self.session_open:
            self.state = 'CLOSING'
            self.process_actions.put(('close', reason))
        else:
            self.state = self.terminal_state
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
                if value.scenario in CURRENT_SCENARIOS:
                    if (not self.longitudinal_sample or self.clock()-self.longitudinal_sample[0]>.7 or
                        self.longitudinal_sample[1]['config']['control_mode'] != 'external_current'):
                        self.log('전류 시험은 STM_A external_current 설정과 readback이 필요합니다.')
                        continue
                    if not self.current_publisher:
                        self.current_publisher = self.create_publisher(DriveCurrentCommand, '/vehicle/drive_current_command', 1)
                elif self.current_publisher:
                    self.destroy_publisher(self.current_publisher)
                    self.current_publisher = None
                self.settings = value
                for topic in FOLLOW_TOPICS:
                    self.received.pop(topic, None)
                    self.sensor_errors.pop(topic, None)
                    self.arrivals[topic].clear()
                self.output = baseline(settings=value, steering_enabled=False)
                self.output_active = True
                self.auto_seen = False
                self.countdown = None
                self.segment = 0
                self.run_dir = ''
                self.finish_reason = ''
                self.run_warning = ''
                self.record_open = value.record_mcap
                self.session_open = True
                self.record_error = ''
                self.terminal_state = 'COMPLETE'
                self.state = 'RECORD_START'
                self.phase_started = now
                self.process_actions.put(('record', value))
                self.log(f'{value.name} · {value.description()}')
            elif action == 'stop':
                if self.state in ('TAIL', 'CLOSING'):
                    self.output = baseline(settings=self.settings)
                elif self.busy:
                    self._finish('사용자 종료', now, terminal='ABORTED')
            elif action == 'record_started':
                self.run_dir = value
                self.log(f'MCAP: {value}')
            elif action == 'record_ready' and self.state == 'RECORD_START':
                self.state = 'WAIT_AUTO'
                self.log(('녹화 연결 완료' if self.settings.record_mcap else '녹화 OFF')
                         + ' · AUTO 대기')
            elif action == 'record_closed':
                self.record_open = False
                if self.state == 'RECORD_START':
                    self.state = 'WAIT_AUTO'
                elif self.state in ('TAIL', 'CLOSING'):
                    self.session_open = False
                    self.state = self.terminal_state
                if self.record_error:
                    self.log(f'MCAP 오류 기록: {self.record_error}')
                elif self.settings.record_mcap:
                    self.log(f'MCAP 저장 완료: {value}')
                else:
                    self.log('시험 정리 완료 · 녹화 OFF')
            elif action == 'record_failed':
                message = ('MCAP 오류: ' if self.settings.record_mcap else '시험 준비 오류: ') + str(value)
                if not self.record_error:
                    self.record_error = str(value)
                self.record_open = False
                self.run_warning = message
                self.log(message)
            elif action == 'manager_failed':
                self.manager_ok = False
                self._finish(f'프로세스 관리 오류: {value}', now, terminal='ERROR')
            elif action == 'route_failed':
                self._finish(f'경로 추종 종료: {value}', now, terminal='ERROR')

    def _tick(self):
        with self.lock:
            now = self.clock()
            self._actions(now)
            countdown_complete = (self.state == 'COUNTDOWN' and
                                  now - self.phase_started >= COUNTDOWN_SEC)
            steering_requested = self.settings.steering_enabled and (
                self.state not in {'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN'} or
                countdown_complete)
            steering_ready = not self.output_active or self.steering_control.ready(
                steering_requested)
            if not steering_ready:
                if self.state in {'RECORD_START', 'WAIT_AUTO'} or (
                        self.state == 'COUNTDOWN' and not countdown_complete):
                    self.phase_started = now
                    return
                if self.state == 'COUNTDOWN':
                    self.countdown = 0
                    self._publish_output()
                    return
            if self.state in {
                    'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN', 'ROUTE_ARMING',
                    'CRUISE', 'RUNNING'}:
                problems = self.issues(
                    now, route_input=self.state == 'RUNNING' and self.settings.follows_route)
                if problems:
                    self._finish('시험 중단: ' + '; '.join(problems), now,
                                 terminal='ERROR')
                else:
                    drive = self.received.get('/drive/status')
                    mode_fresh = drive is not None and now - drive[0] <= 1.0
                    auto = mode_fresh and drive[1].mode == DriveActuatorStatus.MODE_AUTO
                    if self.auto_seen and mode_fresh and not auto:
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
                    problems = self.issues(now)
                    if problems:
                        self._finish('출발 취소: ' + '; '.join(problems), now,
                                     terminal='ERROR')
                    else:
                        self.phase_started = now
                        self.stable_since = None
                        if self.settings.follows_route:
                            self.state = 'ROUTE_ARMING'
                            self.log('경로 출발 명령 대기')
                        else:
                            self.input_started = now
                            self.state = 'CRUISE' if self.settings.needs_cruise else 'RUNNING'
                            self.log('정속 준비' if self.settings.needs_cruise else '시험 입력 시작')
            if self.state == 'ROUTE_ARMING':
                route_command = self.received.get('/sysid/route_command')
                if (route_command and route_command[0] > self.phase_started and
                        not route_command[1].brake_engage):
                    self.state = 'RUNNING'
                    self.input_started = now
                    self.log('시험 입력 시작')
            if self.state == 'CRUISE':
                drive = self.received.get('/drive/status')
                speed = (drive[1].speed_mps * 3.6
                         if drive and now - drive[0] <= 1.0 else None)
                self.output = ScenarioSetpoint(
                    self.settings.max_speed_kph,
                    self.settings.steering_center_deg,
                )
                if speed is not None and abs(speed - self.settings.max_speed_kph) <= 0.5:
                    if self.stable_since is None:
                        self.stable_since = now
                    elif now - self.stable_since >= 1.0:
                        self.state = 'RUNNING'
                        self.input_started = now
                        self.log('정속 확인 · 조향 입력 시작')
                else:
                    self.stable_since = None
                if self.state == 'CRUISE' and now - self.phase_started >= 15:
                    self.run_warning = (
                        '정속 판정 실패 · 15초 내 목표속도 ±0.5 km/h를 '
                        '1초 유지하지 못함 · 조향 입력 계속')
                    self.state = 'RUNNING'
                    self.input_started = now
                    self.log(self.run_warning)
            if self.state == 'RUNNING':
                if self.settings.follows_route:
                    command = self.received['/sysid/route_command'][1]
                    route = self.received.get('/sysid/route_context')
                    self.output = ScenarioSetpoint(
                        command.speed_target_mps * 3.6,
                        math.degrees(command.steering_target_rad), command.brake_engage)
                    drive = self.received.get('/drive/status')
                    if (route and now - route[0] <= 1.0
                            and route[1].route_projection_valid
                            and route[1].distance_to_finish <= ROUTE_FINISH_DISTANCE_M
                            and command.brake_engage
                            and drive and now - drive[0] <= 1.0
                            and abs(drive[1].speed_mps) <= ROUTE_STOP_SPEED_MPS):
                        self._finish('경로 종료점 도착', now)
                else:
                    elapsed = now - self.input_started
                    self.output, complete = input_setpoint(self.settings, elapsed)
                    if complete:
                        self._finish('시험 입력 완료 · 목표속도 0 유지', now)
            if self.state == 'TAIL' and now - self.phase_started >= TAIL_SEC:
                self.state = 'CLOSING'
                self.process_actions.put(('close', self.finish_reason))
            self.route_enable_publisher.publish(Bool(
                data=self.settings.follows_route
                and self.state in {'ROUTE_ARMING', 'RUNNING'}))
            self._publish_output()

    def _publish_output(self):
        if not self.output_active:
            return
        message = ActuatorCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.speed_target_mps = self.output.speed_kph / 3.6
        message.steering_target_rad = (
            0.0 if self.output.steering_deg is None
            else math.radians(self.output.steering_deg))
        message.brake_engage = self.output.brake_engage
        if self.current_publisher:
            if not external_current_ready(self.received.get('/drive/status'), self.longitudinal_sample, self.clock()):
                message.speed_target_mps = 0.0
                message.brake_engage = True
            demand = DriveCurrentCommand()
            demand.header.stamp = message.header.stamp
            demand.phase_current_a = current_setpoint(self, self.output,
                self.received.get('/drive/status'), self.longitudinal_sample,
                message.brake_engage, now=self.clock())
            self.current_publisher.publish(demand)
        self.command_publisher.publish(message)
        self.label_publisher.publish(String(
            data=(
                f'{self.settings.record_slug}'
                f'_v{self.settings.max_speed_kph:g}'
            )))

    def snapshot(self):
        with self.lock:
            now = self.clock()
            rows = []
            route_observation = self.settings.follows_route and self.busy
            route_input = self.settings.follows_route and self.state == 'RUNNING'
            topics = TOPICS | FOLLOW_TOPICS if route_observation else TOPICS
            for topic in topics:
                sample = self.received.get(topic)
                error = self.sensor_errors.get(topic, '')
                age = now - sample[0] if sample else None
                times = self.arrivals[topic]
                rate = ((len(times) - 1) / (times[-1] - times[0])
                        if len(times) > 1 and times[-1] > times[0] else 0)
                if sample is None:
                    symbol, text = '—', '없음'
                elif topic != '/tf_static' and age > (0.2 if topic == '/sysid/route_command' else 1):
                    symbol, text = '—', f'{age:.1f}초 수신 중단'
                elif error:
                    symbol, text = '—', error
                elif topic == '/tf_static':
                    symbol, text = '✅', '정적 TF 수신'
                else:
                    symbol, text = '✅', f'{rate:.1f} Hz'
                rows.append((topic, symbol, text))
            drive = self.received.get('/drive/status')
            steering = self.received.get('/steering/status')
            mode = ('미수신' if not drive or now - drive[0] > 1 else
                    'AUTO' if drive[1].mode == DriveActuatorStatus.MODE_AUTO else 'MANUAL')
            return {
                'launch_preset': self.launch_preset,
                'state': self.state, 'label': (
                    '시험 준비 중' if self.state == 'RECORD_START' and not self.settings.record_mcap
                    else '시험 정리 중' if self.state == 'CLOSING' and not self.settings.record_mcap
                    else STATE_LABELS[self.state]
                ), 'mode': mode, 'busy': self.busy,
                'speed': drive[1].speed_mps * 3.6 if drive else None,
                'rpm': drive[1].motor_rpm if drive else None,
                'current': drive[1].phase_current_a if drive else None,
                'steering': math.degrees(steering[1].actual_wheel_angle_rad) if steering else None,
                'steering_advice': steering_fault_advice(
                    steering[1] if steering else None,
                    fresh=bool(steering and now - steering[0] <= 1.0)),
                'output': self.output, 'rows': rows, 'drivers': list(self.driver_rows),
                'optional_topics': set(topics) - set(self.required_topics_for(route_input)),
                'issues': self.issues(now) + (
                    [self.steering_control.error]
                    if self.output_active and self.state in {'RECORD_START', 'WAIT_AUTO', 'COUNTDOWN'}
                    and self.steering_control.error else []),
                'logs': list(self.logs)[-7:],
                'run_dir': self.run_dir, 'recording': self.record_open,
                'settings': self.settings,
                'countdown': self.countdown if self.state == 'COUNTDOWN' else None,
                'elapsed': max(0, now - self.input_started) if self.state == 'RUNNING' else None,
                'gates': self.gate_rows(now, route_input=route_input),
                'finish_reason': self.finish_reason,
                'run_warning': self.run_warning,
                'terminal_state': self.terminal_state,
            }
