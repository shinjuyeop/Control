"""Read actual node parameters after the recorder connects, before allowing Run."""

from datetime import datetime, timezone
import time

from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python

from control_tuning.conditions import condition_document
from control_tuning.schema import STEERING, GOVERNOR, READ_NAMES, decode_node


TORQUE_TOPICS = (
    '/steering/actuation_mode', '/steering/requested_angle', '/steering/angle_error',
    '/steering/rate', '/steering/unfiltered_rate', '/steering/torque_p_raw',
    '/steering/torque_d_raw', '/steering/torque_unsaturated_raw',
    '/steering/torque_command_raw', '/steering/torque_current_limit_command_a',
    '/steering/torque_limit_raw', '/steering/torque_saturated',
    '/steering/torque_peak_limited', '/steering/torque_settled',
    '/steering/torque_rate_ready', '/steering/torque_safety',
)
REQUIRED_PARAMETERS = {
    STEERING: set(READ_NAMES[STEERING]) | {'torque_direction_sign', 'max_steering_rate_deg_s',
                                         'steering_enabled', 'hardware_can_enabled', 'actuator_output_enabled'},
    GOVERNOR: {'input_topic', 'output_topic', 'input_timeout_ms'},
}


def wall_time():
    return datetime.now(timezone.utc).isoformat()


class ParameterCapture:
    """Non-blocking service state machine. Calls no parameter setters."""

    def __init__(self, node, condition, *, timeout=5.0):
        self.node, self.started, self.timeout = node, time.monotonic(), timeout
        self.finished = False
        self.document = {
            'schema_version': 1, 'capture_stage': 'recorder_ready_before_run',
            'requested_at': wall_time(),
            'requested_ros_time_ns': node.get_clock().now().nanoseconds,
            'test_condition': condition_document(condition),
            'status': 'pending', 'nodes': {},
            'note': 'Parameters read from running nodes. Use /parameter_events for subsequent changes.',
        }
        self.requests = {}
        for name in REQUIRED_PARAMETERS:
            listing = node.create_client(ListParameters, f'/{name}/list_parameters')
            getter = node.create_client(GetParameters, f'/{name}/get_parameters')
            self.requests[name] = dict(listing=listing, getter=getter, stage='waiting', future=None)

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
                        elif name == STEERING:
                            entry['display_values'] = decode_node(name, values)
                            entry['display_units'] = {
                                'p_gain_a_per_deg': 'A/deg', 'd_gain_a_per_deg_s': 'A/(deg/s)',
                                'peak_current_command_a': 'A', 'continuous_current_command_a': 'A',
                            }
                        self.document['nodes']['/' + name] = entry
                        request['stage'] = 'done'
            except Exception as error:
                self._error(name, str(error))
            if request['stage'] != 'done' and time.monotonic() - self.started > self.timeout:
                self._error(name, '실제 파라미터 조회 시간 초과')
        if all(request['stage'] == 'done' for request in self.requests.values()):
            self.finished = True
            complete = all(entry['status'] == 'complete' for entry in self.document['nodes'].values())
            self.document.update(status='complete' if complete else 'incomplete', completed_at=wall_time())
        return self.finished

    def close(self):
        for request in self.requests.values():
            future = request['future']
            if future is not None and not future.done():
                for client in (request['listing'], request['getter']):
                    client.remove_pending_request(future)
                future.cancel()
            self.node.destroy_client(request['listing'])
            self.node.destroy_client(request['getter'])
