"""Apply the steering motor setting before publishing scenario commands."""

import time

from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters


class SteeringControl:
    def __init__(self, node):
        self.client = node.create_client(
            SetParameters, '/steering_actuator_node/set_parameters')
        self.applied = None
        self.requested = None
        self.future = None
        self.error = '조향 모터 설정 응답 대기'
        self.requested_at = 0.0

    def ready(self, enabled):
        if not self.client.service_is_ready():
            self.applied = None
            self.error = 'steering_actuator_node 파라미터 서비스 대기'
            return False
        if self.future is not None:
            if not self.future.done():
                if time.monotonic() - self.requested_at < 1.0:
                    return False
                self.client.remove_pending_request(self.future)
                self.future.cancel()
                self.future = None
                self.applied = None
                self.error = '조향 모터 설정 응답 시간 초과'
                return False
            try:
                result = self.future.result().results[0]
                if result.successful:
                    self.applied = self.requested
                    self.error = ''
                else:
                    self.error = '조향 모터 설정 실패: ' + result.reason
            except Exception as error:
                self.error = f'조향 모터 설정 실패: {error}'
            self.future = None
        if self.applied == enabled:
            return True
        if self.requested != enabled or time.monotonic() - self.requested_at >= 1.0:
            self.requested = enabled
            self.requested_at = time.monotonic()
            self.error = '조향 모터 설정 응답 대기'
            self.future = self.client.call_async(SetParameters.Request(parameters=[
                Parameter('steering_enabled', value=enabled).to_parameter_msg(),
            ]))
        return False
