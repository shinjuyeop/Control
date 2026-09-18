"""Observe nodes; change torque tuning atomically only at a fresh manual stop."""

import math
import time

from kaiev26_msgs.msg import DriveActuatorStatus, SteeringActuatorStatus
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.node import Node
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data

from .schema import (STEERING, GOVERNOR, READ_NAMES, LIVE_NAMES,
                     equivalent, encode, validate_native)


def stop_reason(drive, steering, now):
    if drive is None or now - drive[0] > 0.5:
        return '차량 상태 수신 대기 / 오래된 상태'
    if steering is None or now - steering[0] > 0.5:
        return '조향 상태 수신 대기 / 오래된 상태'
    msg = drive[1]
    if msg.mode != DriveActuatorStatus.MODE_MANUAL:
        return 'MANUAL 전환 후 적용할 수 있습니다.'
    if not msg.part1_valid or not math.isfinite(msg.speed_mps) or abs(msg.speed_mps) > 0.05:
        return '유효한 정지 차속 확인이 필요합니다. (0.18 km/h 이하)'
    if (steering[1].state == SteeringActuatorStatus.STATE_ACTIVE or
            steering[1].enabled_commanded):
        return '조향 ACTIVE 해제를 기다리고 있습니다.'
    return ''


class TuningNode(Node):
    def __init__(self):
        super().__init__('control_tuning', enable_rosout=False)
        self.values = {}
        self.received = {}
        self.errors = {}
        self.requests = {}
        self.drive = self.steering = None
        self.last_poll = -10.0
        self.operation = None
        self.result = None
        self.uncertain = False
        self.read_clients = {name: self.create_client(GetParameters, f'/{name}/get_parameters')
                        for name in READ_NAMES}
        self.writer = self.create_client(SetParametersAtomically, f'/{STEERING}/set_parameters_atomically')
        self.create_subscription(DriveActuatorStatus, '/drive/status',
                                 self._drive, qos_profile_sensor_data)
        self.create_subscription(SteeringActuatorStatus, '/steering/status',
                                 self._steering, qos_profile_sensor_data)

    def _drive(self, msg):
        self.drive = (time.monotonic(), msg)

    def _steering(self, msg):
        self.steering = (time.monotonic(), msg)

    def fresh_values(self, node):
        if time.monotonic() - self.received.get(node, -10) > 4.0:
            return None
        return self.values.get(node)

    def apply_reason(self, context):
        values = self.fresh_values(STEERING)
        if self.operation:
            return '적용 처리 중입니다.'
        if self.uncertain:
            return '이전 적용 결과 확인 중입니다. 다시 읽기를 기다려 주십시오.'
        if not values:
            return self.errors.get(STEERING, '조향 노드 연결 대기')
        if values['actuation_mode'] != 'torque':
            return '현재 노드가 Position 모드입니다. Torque로 저장·재시작 후 적용하십시오.'
        if not equivalent(values['keya_protection_current_a'], context['keya_protection_current_a']):
            return '실행 노드와 저장 설정의 KEYA 전류 환산 기준이 다릅니다.'
        count = sum(name == STEERING and namespace == '/'
                    for name, namespace in self.get_node_names_and_namespaces())
        if count != 1:
            return '조향 노드가 정확히 하나인지 확인해야 합니다.'
        return stop_reason(self.drive, self.steering, time.monotonic())

    def tick(self, force=False):
        now = time.monotonic()
        if self.operation:
            self._advance(now)
            return
        for name, (future, started) in list(self.requests.items()):
            if future.done():
                del self.requests[name]
                try:
                    self._accept(name, future.result())
                    if name == STEERING:
                        self.uncertain = False
                except Exception as error:
                    self.errors[name] = str(error)
                    self.received.pop(name, None)
            elif now - started > 3.0:
                self.read_clients[name].remove_pending_request(future)
                future.cancel()
                del self.requests[name]
                self.errors[name] = '파라미터 응답 시간 초과'
                self.received.pop(name, None)
        if force or now - self.last_poll >= 1.5:
            self.last_poll = now
            for name, client in self.read_clients.items():
                if name not in self.requests and client.service_is_ready():
                    self.requests[name] = (self._read(name), now)

    def _read(self, name):
        return self.read_clients[name].call_async(GetParameters.Request(names=list(READ_NAMES[name])))

    def _accept(self, name, response):
        if response is None or len(response.values) != len(READ_NAMES[name]):
            raise ValueError('파라미터 응답이 불완전합니다.')
        values = {key: parameter_value_to_python(value)
                  for key, value in zip(READ_NAMES[name], response.values)}
        if any(value is None for value in values.values()):
            raise ValueError('지원하는 파라미터가 없습니다. 현재 Control 노드를 빌드·재시작하십시오.')
        self.values[name] = values
        self.received[name] = time.monotonic()
        self.errors.pop(name, None)
        return values

    def begin_apply(self, draft, context, expected):
        reason = self.apply_reason(context)
        if reason:
            raise ValueError(reason)
        if not self.writer.service_is_ready():
            raise ValueError('조향 일괄 적용 서비스가 없습니다.')
        # Ignore any earlier polling responses once the preflight starts.
        for name, (future, _) in self.requests.items():
            self.read_clients[name].remove_pending_request(future)
            future.cancel()
        self.requests.clear()
        self.result = None
        self.operation = dict(stage='preflight', future=self._read(STEERING),
                              started=time.monotonic(), draft=dict(draft),
                              context=dict(context), expected=dict(expected))

    def _advance(self, now):
        op = self.operation
        future = op['future']
        if not future.done():
            if now - op['started'] > 3.0:
                client = self.writer if op['stage'] == 'write' else self.read_clients[STEERING]
                client.remove_pending_request(future)
                future.cancel()
                self.uncertain = op['stage'] != 'preflight'
                self._finish(False, '응답 시간 초과. 적용 여부를 다시 읽어 확인합니다. 자동 재적용하지 않습니다.')
            return
        try:
            response = future.result()
            if op['stage'] == 'preflight':
                fresh = self._accept(STEERING, response)
                if any(not equivalent(fresh[k], op['expected'][k]) for k in READ_NAMES[STEERING]):
                    raise ValueError('검토 이후 조향 설정이 변경되었습니다. 최신 값을 확인하고 다시 적용하십시오.')
                reason = stop_reason(self.drive, self.steering, time.monotonic())
                if reason:
                    raise ValueError(reason)
                if sum(name == STEERING and namespace == '/'
                       for name, namespace in self.get_node_names_and_namespaces()) != 1:
                    raise ValueError('조향 노드가 정확히 하나인지 확인해야 합니다.')
                if fresh['actuation_mode'] != 'torque':
                    raise ValueError('현재 조향 노드가 Torque 모드가 아닙니다.')
                if not equivalent(fresh['keya_protection_current_a'], op['context']['keya_protection_current_a']):
                    raise ValueError('전류 환산 기준이 변경되었습니다.')
                batch = encode(op['draft'], fresh)
                native = batch[STEERING]
                # Startup-only values remain active until restart. Validate that combination too.
                validate_native({**fresh, **{k: native[k] for k in LIVE_NAMES}},
                                batch[GOVERNOR])
                op['target'] = {k: native[k] for k in LIVE_NAMES}
                parameters = [Parameter(k, value=v).to_parameter_msg()
                              for k, v in op['target'].items()]
                op.update(stage='write', started=now, future=self.writer.call_async(
                    SetParametersAtomically.Request(parameters=parameters)))
            elif op['stage'] == 'write':
                if response is None or not response.result.successful:
                    raise ValueError('적용 거부: ' + (response.result.reason if response else '응답 없음'))
                op.update(stage='readback', started=now, future=self._read(STEERING))
            else:
                fresh = self._accept(STEERING, response)
                if any(not equivalent(fresh[k], v) for k, v in op['target'].items()):
                    raise ValueError('적용 후 값이 일치하지 않습니다. 외부 변경 여부를 확인하십시오.')
                self._finish(True, '조향 튜닝 적용·재조회 완료. 저장은 S로 별도 실행하십시오.')
        except Exception as error:
            self.uncertain = op['stage'] in ('write', 'readback')
            self._finish(False, str(error))

    def _finish(self, success, message):
        self.result = (success, message)
        self.operation = None
        self.last_poll = -10.0
