"""Read-only current settings for the preset editor."""

import time

from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from .longitudinal import LONGITUDINAL, STATUS_TOPIC, decode_status
from .schema import READ_NAMES


class TuningNode(Node):
    def __init__(self):
        super().__init__('control_tuning', enable_rosout=False)
        self.values = {}
        self.received = {}
        self.errors = {}
        self.requests = {}
        self.last_poll = -10.0
        self._longitudinal = None
        self.read_clients = {name: self.create_client(GetParameters, f'/{name}/get_parameters')
                             for name in READ_NAMES}
        self.create_subscription(Float32MultiArray, STATUS_TOPIC,
                                 self._receive_longitudinal, qos_profile_sensor_data)

    def _receive_longitudinal(self, msg):
        try:
            self._longitudinal = decode_status(msg.data)
            self.values[LONGITUDINAL] = self._longitudinal['config']
            self.received[LONGITUDINAL] = time.monotonic()
            self.errors.pop(LONGITUDINAL, None)
        except ValueError as error:
            self.received.pop(LONGITUDINAL, None)
            self.errors[LONGITUDINAL] = str(error)

    def fresh_values(self, node):
        timeout = .7 if node == LONGITUDINAL else 4.0
        if time.monotonic() - self.received.get(node, -10) > timeout:
            return None
        return self.values.get(node)

    def longitudinal_status(self):
        return self._longitudinal if self.fresh_values(LONGITUDINAL) else None

    def tick(self, force=False):
        now = time.monotonic()
        for name, (future, started) in list(self.requests.items()):
            if future.done():
                del self.requests[name]
                try:
                    self._accept(name, future.result())
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
