"""Small ROS 2 Humble-compatible client for a remote node's parameters."""

from __future__ import annotations

from rcl_interfaces.srv import SetParametersAtomically


class RemoteParameterClient:
    def __init__(self, node, remote_node_name: str) -> None:
        remote = "/" + remote_node_name.strip("/")
        self._node = node
        self._client = node.create_client(
            SetParametersAtomically, remote + "/set_parameters_atomically"
        )

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_sec)

    def set_parameters(self, parameters):
        if not self._client.service_is_ready():
            self._node.get_logger().error("remote parameter service is not ready")
            return None
        request = SetParametersAtomically.Request()
        request.parameters = [parameter.to_parameter_msg() for parameter in parameters]
        return self._client.call_async(request)
