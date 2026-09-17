"""Apply the steering motor setting before publishing scenario commands."""

import time

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters, SetParametersAtomically


class SteeringControl:
    def __init__(self, node):
        self.client = node.create_client(
            SetParameters, '/steering_actuator_node/set_parameters')
        # Gains and enable go in one atomic set: the node validates the whole batch first.
        self.atomic_client = node.create_client(
            SetParametersAtomically, '/steering_actuator_node/set_parameters_atomically')
        self.applied = None
        self.requested = None
        self.future = None
        self.error = '조향 모터 설정 응답 대기'
        self.requested_at = 0.0

    def invalidate(self):
        """Re-apply on the next ready(); gains may have been changed outside the TUI."""
        self.applied = None

    def ready(self, enabled, gains=None):
        desired = enabled if gains is None else (enabled, tuple(sorted(gains.items())))
        client = self.client if gains is None else self.atomic_client
        if not client.service_is_ready():
            self.applied = None
            self.error = 'steering_actuator_node 파라미터 서비스 대기'
            return False
        if self.future is not None:
            if not self.future.done():
                if time.monotonic() - self.requested_at < 1.0:
                    return False
                self.future.client.remove_pending_request(self.future.request_future)
                self.future.request_future.cancel()
                self.future = None
                self.applied = None
                self.error = '조향 모터 설정 응답 시간 초과'
                return False
            try:
                response = self.future.request_future.result()
                result = (response.results[0] if isinstance(response, SetParameters.Response)
                          else response.result)
                if result.successful:
                    self.applied = self.requested
                    self.error = ''
                else:
                    self.error = '조향 모터 설정 실패: ' + result.reason
            except Exception as error:
                self.error = f'조향 모터 설정 실패: {error}'
            self.future = None
        if self.applied == desired:
            return True
        if self.requested != desired or time.monotonic() - self.requested_at >= 1.0:
            self.requested = desired
            self.requested_at = time.monotonic()
            self.error = '조향 모터 설정 응답 대기'
            parameters = [Parameter('steering_enabled', value=enabled).to_parameter_msg()]
            if gains is None:
                future = self.client.call_async(SetParameters.Request(parameters=parameters))
            else:
                parameters += [Parameter(name, value=value).to_parameter_msg()
                               for name, value in sorted(gains.items())]
                future = self.atomic_client.call_async(
                    SetParametersAtomically.Request(parameters=parameters))
            self.future = _Pending(client, future)
        return False


class _Pending:
    """A request future with the client that owns it, so a timeout cancels on the right one."""

    def __init__(self, client, request_future):
        self.client = client
        self.request_future = request_future

    def done(self):
        return self.request_future.done()
