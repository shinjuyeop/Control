"""Read actual node parameters after the recorder connects, before allowing Run."""

from datetime import datetime, timezone
import time

from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from control_tuning.conditions import condition_document
from control_tuning.longitudinal import STATUS_TOPIC, decode_status
from control_tuning.schema import STEERING, GOVERNOR, READ_NAMES, FIELDS, decode_node


REQUIRED_PARAMETERS = {
    STEERING: set(READ_NAMES[STEERING]) | {'torque_direction_sign', 'max_steering_rate_deg_s',
                                         'steering_enabled', 'hardware_can_enabled', 'actuator_output_enabled'},
    GOVERNOR: {'input_topic', 'output_topic', 'input_timeout_ms'},
}


def wall_time():
    return datetime.now(timezone.utc).isoformat()


class ParameterCapture:
    """Non-blocking service state machine. Calls no parameter setters."""

    def __init__(self, node, condition=None, *, timeout=5.0, require_longitudinal=False):
        self.node, self.started, self.timeout = node, time.monotonic(), timeout
        self.finished = False
        self.document = {
            'schema_version': 2, 'capture_stage': 'recorder_ready_before_run',
            'requested_at': wall_time(),
            'requested_ros_time_ns': node.get_clock().now().nanoseconds,
            'test_condition': condition_document(condition) if condition else {'key': 'unspecified'},
            'status': 'pending', 'nodes': {},
            'note': 'Parameters read from running nodes. Use /parameter_events for subsequent changes.',
        }
        self.require_longitudinal = require_longitudinal
        self.longitudinal = None
        self.longitudinal_subscription = (node.create_subscription(
            Float32MultiArray, STATUS_TOPIC, self._receive_longitudinal, qos_profile_sensor_data)
            if require_longitudinal else None)
        self.document['longitudinal'] = {'status': 'pending' if require_longitudinal else 'not_requested'}
        self.requests = {}
        for name in REQUIRED_PARAMETERS:
            listing = node.create_client(ListParameters, f'/{name}/list_parameters')
            getter = node.create_client(GetParameters, f'/{name}/get_parameters')
            self.requests[name] = dict(listing=listing, getter=getter, stage='waiting', future=None)

    def _receive_longitudinal(self, message):
        try:
            self.longitudinal = (time.monotonic(), decode_status(message.data))
        except ValueError:
            self.longitudinal = None

    def _error(self, name, message):
        self.document['nodes']['/' + name] = {'status': 'failed', 'error': message}
        self.requests[name]['stage'] = 'done'

    def tick(self):
        if self.finished:
            return True
        names = self.node.get_node_names_and_namespaces()
        for name, request in self.requests.items():
            if request['stage'] == 'done':
                continue
            try:
                count = sum(n == name and namespace == '/' for n, namespace in names)
                if count > 1:
                    raise ValueError('동일한 이름의 노드가 여러 개입니다.')
                if request['stage'] == 'waiting':
                    if count == 1 and request['listing'].service_is_ready() and request['getter'].service_is_ready():
                        request.update(stage='list', future=request['listing'].call_async(ListParameters.Request()))
                elif request['future'].done():
                    response = request['future'].result()
                    if request['stage'] == 'list':
                        keys = sorted(response.result.names)
                        if not keys:
                            raise ValueError('실행 노드의 파라미터 목록이 비어 있습니다.')
                        request.update(stage='get', names=keys,
                                       future=request['getter'].call_async(GetParameters.Request(names=keys)))
                    else:
                        keys = request['names']
                        if len(response.values) != len(keys):
                            raise ValueError('파라미터 응답 개수가 일치하지 않습니다.')
                        values = {key: parameter_value_to_python(value)
                                  for key, value in zip(keys, response.values)}
                        missing = sorted(REQUIRED_PARAMETERS[name] - {k for k, v in values.items() if v is not None})
                        entry = {
                            'status': 'partial' if missing else 'complete',
                            'received_at': wall_time(),
                            'received_ros_time_ns': self.node.get_clock().now().nanoseconds,
                            'ros__parameters': values,
                        }
                        if missing:
                            entry.update(missing_parameters=missing, error='필수 제어 파라미터를 읽지 못했습니다.')
                        else:
                            entry['display_values'] = decode_node(name, values)
                            entry['display_units'] = {field.key: field.unit for field in FIELDS
                                                      if field.node == name}
                        self.document['nodes']['/' + name] = entry
                        request['stage'] = 'done'
            except Exception as error:
                self._error(name, str(error))
            if request['stage'] != 'done' and time.monotonic() - self.started > self.timeout:
                self._error(name, '실제 파라미터 조회 시간 초과')
        if all(request['stage'] == 'done' for request in self.requests.values()):
            if self.require_longitudinal:
                if self.longitudinal and time.monotonic() - self.longitudinal[0] < .7:
                    self.document['longitudinal'] = {
                        'status': 'complete', 'topic': STATUS_TOPIC,
                        'received_at': wall_time(), 'readback': self.longitudinal[1]}
                elif time.monotonic() - self.started <= self.timeout:
                    return False
                else:
                    self.document['longitudinal'] = {'status': 'failed',
                        'error': 'STM_A longitudinal readback missing, invalid or stale'}
            self.finished = True
            complete = all(entry['status'] == 'complete' for entry in self.document['nodes'].values())
            complete &= not self.require_longitudinal or self.document['longitudinal']['status'] == 'complete'
            self.document.update(status='complete' if complete else 'incomplete', completed_at=wall_time())
        return self.finished

    def close(self):
        if self.longitudinal_subscription is not None:
            self.node.destroy_subscription(self.longitudinal_subscription)
            self.longitudinal_subscription = None
        for request in self.requests.values():
            future = request['future']
            if future is not None and not future.done():
                for client in (request['listing'], request['getter']):
                    client.remove_pending_request(future)
                future.cancel()
            self.node.destroy_client(request['listing'])
            self.node.destroy_client(request['getter'])
