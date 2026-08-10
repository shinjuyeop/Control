"""ROS 2 bridge between STM_TEST USB serial and shared HIL interfaces."""

from __future__ import annotations

from dataclasses import replace
import queue
import threading
import time
from typing import Callable

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .models import HilDesiredState
from .protocol import desired_commands, desired_status_mismatches, update_desired, validate_command
from .transport import SerialCommandClient


class BridgeWorker(threading.Thread):
    """Single owner of serial I/O; emits immutable events to the ROS thread."""

    def __init__(
        self,
        *,
        serial_port: str,
        baud_rate: int,
        status_poll_hz: float,
        command_timeout_sec: float,
        retry_count: int,
        reconnect_interval_sec: float,
        event_callback: Callable[[str, object], None],
        client_factory=SerialCommandClient,
    ) -> None:
        super().__init__(name="stm-test-serial", daemon=True)
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.status_period = 1.0 / max(0.1, status_poll_hz)
        self.command_timeout_sec = command_timeout_sec
        self.retry_count = max(0, retry_count)
        self.reconnect_interval_sec = max(0.1, reconnect_interval_sec)
        self.event_callback = event_callback
        self.client_factory = client_factory
        self.commands: queue.Queue[str] = queue.Queue()
        self._stop_event = threading.Event()
        self._desired_lock = threading.Lock()
        self._desired = HilDesiredState()
        self._connected = False
        self._client = None
        self._protocol_version = None
        self._last_status = None

    def enqueue(self, command: str) -> None:
        self.commands.put(validate_command(command))

    def replace_desired(self, desired: HilDesiredState, *, enqueue: bool = True) -> None:
        with self._desired_lock:
            self._desired = replace(desired)
        if not self._connected:
            # A complete desired-state replacement (currently Safe State) must
            # supersede commands queued while the serial link was down.
            while True:
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break
        if enqueue and self._connected:
            for command in desired_commands(
                desired, protocol_version=self._protocol_version or 1
            ):
                self.enqueue(command)
        self.event_callback("desired", replace(desired))

    def reapply_desired(self) -> None:
        if not self._connected:
            return
        for command in desired_commands(
            self.desired_snapshot(), protocol_version=self._protocol_version or 1
        ):
            self.enqueue(command)

    def desired_snapshot(self) -> HilDesiredState:
        with self._desired_lock:
            return replace(self._desired)

    def stop(self) -> None:
        self._stop_event.set()

    def _set_connected(self, connected: bool, detail: str = "") -> None:
        if connected != self._connected or detail:
            self._connected = connected
            self.event_callback("connection", (connected, detail))

    def _disconnect(self, reason: str) -> None:
        if self._client is not None:
            self._client.close()
        self._client = None
        self._protocol_version = None
        self._last_status = None
        self._set_connected(False, reason)

    def _request(self, command: str):
        last_exception = None
        for _ in range(self.retry_count + 1):
            try:
                # "ERR ..." 응답도 유효한 응답이라 재시도하지 않는다.
                # 재시도는 링크 수준 예외에만 적용한다.
                return self._client.request(command)
            except (OSError, ConnectionError, TimeoutError) as exc:
                last_exception = exc
        raise last_exception

    def _apply_desired_after_connect(self) -> None:
        for command in desired_commands(
            self.desired_snapshot(), protocol_version=self._protocol_version or 1
        ):
            response = self._request(command)
            self.event_callback("result", response.raw)
            if response.kind != "ok":
                raise IOError(f"desired-state replay rejected: {response.raw}")

    def _read_status(self) -> None:
        response = self._request("STATUS")
        if response.kind != "status" or response.status is None:
            raise IOError(f"STATUS request rejected: {response.raw}")
        self._last_status = response.status
        self._protocol_version = response.status.protocol
        self.event_callback("status", response.status)

    def _handle_command(self, command: str) -> None:
        if command.startswith("SET PLANT ") and (self._protocol_version or 1) < 2:
            self.event_callback(
                "result", "ERR plant_not_supported_by_STM_TEST_protocol_1"
            )
            return

        held_motion = None
        if command == "SET PLANT OFF":
            # Capture the plant output immediately before opening the loop so
            # reconnect replay continues from the value the operator saw.
            self._read_status()
            held_motion = self._last_status
        response = self._request(command)
        if response.kind == "status":
            self._last_status = response.status
            self._protocol_version = response.status.protocol
            self.event_callback("status", response.status)
        else:
            self.event_callback("result", response.raw)
            if response.kind == "ok":
                with self._desired_lock:
                    if held_motion is not None:
                        self._desired.speed = held_motion.speed
                        self._desired.rpm = held_motion.rpm
                        self._desired.motion_source = "RPM"
                    update_desired(self._desired, command)
                    snapshot = replace(self._desired)
                self.event_callback("desired", snapshot)

    def run(self) -> None:
        next_reconnect = 0.0
        next_status = time.monotonic()
        while not self._stop_event.is_set():
            now = time.monotonic()
            if self._client is None:
                if now < next_reconnect:
                    self._stop_event.wait(min(0.1, next_reconnect - now))
                    continue
                try:
                    self._client = self.client_factory(
                        self.serial_port,
                        self.baud_rate,
                        self.command_timeout_sec,
                    )
                    self._client.open()
                    # Read capabilities before replay. Protocol-1 firmware does
                    # not understand SET PLANT and must remain connectable.
                    self._read_status()
                    self._apply_desired_after_connect()
                    self._read_status()
                    self._set_connected(True, self.serial_port)
                    next_status = time.monotonic()
                except Exception as exc:
                    self._disconnect(str(exc))
                    next_reconnect = time.monotonic() + self.reconnect_interval_sec
                    continue

            try:
                timeout = max(0.0, min(0.1, next_status - time.monotonic()))
                try:
                    command = self.commands.get(timeout=timeout)
                except queue.Empty:
                    command = None
                if command is not None:
                    self._handle_command(command)
                    if command != "STATUS":
                        # A SET/FAULT reply is "OK", not a STATUS, so the change
                        # was invisible until the next scheduled poll -- up to a
                        # full period of apparent lag on every operator click.
                        # Re-poll now; the deadline below resyncs from here.
                        next_status = time.monotonic()
                if time.monotonic() >= next_status:
                    self._handle_command("STATUS")
                    # Advance from the deadline, not from completion. A STATUS
                    # round trip is ~44 ms of wire time at 115200; charging that
                    # to the next period turned a requested 10 Hz into 6.9 Hz.
                    next_status += self.status_period
                    now = time.monotonic()
                    if next_status <= now:
                        # Behind schedule: resync rather than queue back-to-back
                        # polls, which would starve operator commands.
                        next_status = now + self.status_period
            except Exception as exc:
                self._disconnect(str(exc))
                next_reconnect = time.monotonic() + self.reconnect_interval_sec
        self._disconnect("stopped")


class StmTestBridge(Node):
    def __init__(self) -> None:
        super().__init__("stm_test_bridge")
        self.declare_parameter("serial_port", "/dev/hil_test")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("status_poll_hz", 5.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("retry_count", 2)
        self.declare_parameter("reconnect_interval_sec", 1.0)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, "/hil/status", 10)
        self.desired_pub = self.create_publisher(String, "/hil/desired_state", qos)
        self.connected_pub = self.create_publisher(Bool, "/hil/connected", qos)
        self.result_pub = self.create_publisher(String, "/hil/command_result", 20)
        self.diagnostic_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(String, "/hil/command", self._on_command, 20)
        self.create_service(Trigger, "/hil/reset_safe_state", self._reset_safe_state)
        self.create_service(Trigger, "/hil/refresh_status", self._refresh_status)
        self.create_service(Trigger, "/hil/reapply_desired_state", self._reapply_desired)

        self._events: queue.Queue[tuple[str, object]] = queue.Queue()
        self._connected = False
        self._connection_detail = "not started"
        self._last_status_monotonic = None
        self._last_status = None
        self._worker = BridgeWorker(
            serial_port=str(self.get_parameter("serial_port").value),
            baud_rate=int(self.get_parameter("baud_rate").value),
            status_poll_hz=float(self.get_parameter("status_poll_hz").value),
            command_timeout_sec=float(self.get_parameter("command_timeout_sec").value),
            retry_count=int(self.get_parameter("retry_count").value),
            reconnect_interval_sec=float(self.get_parameter("reconnect_interval_sec").value),
            event_callback=lambda kind, payload: self._events.put((kind, payload)),
        )
        self._worker.start()
        # Draining is a queue pop and a publish; the interval is pure added
        # latency between the serial thread and every subscriber, so keep it
        # well under the status period.
        self.create_timer(0.02, self._drain_events)
        self.create_timer(1.0, self._publish_diagnostics)
        self._publish_desired(self._worker.desired_snapshot())

    def _publish_text(self, publisher, value: str) -> None:
        message = String()
        message.data = value
        publisher.publish(message)

    def _publish_desired(self, desired: HilDesiredState) -> None:
        self._publish_text(self.desired_pub, desired.to_json())

    def _on_command(self, message: String) -> None:
        try:
            self._worker.enqueue(message.data)
        except ValueError as exc:
            self._publish_text(self.result_pub, f"ERR validation: {exc}")

    def _reset_safe_state(self, request, response):
        del request
        self._worker.replace_desired(HilDesiredState(), enqueue=True)
        response.success = True
        response.message = "safe state queued; it will replay after reconnect if disconnected"
        return response

    def _refresh_status(self, request, response):
        del request
        self._worker.enqueue("STATUS")
        response.success = True
        response.message = "STATUS queued"
        return response

    def _reapply_desired(self, request, response):
        del request
        self._worker.reapply_desired()
        response.success = True
        response.message = (
            "desired state replay queued" if self._connected
            else "disconnected; desired state will replay after reconnect"
        )
        return response

    def _drain_events(self) -> None:
        while True:
            try:
                kind, payload = self._events.get_nowait()
            except queue.Empty:
                break
            if kind == "connection":
                self._connected, self._connection_detail = payload
                message = Bool()
                message.data = self._connected
                self.connected_pub.publish(message)
            elif kind == "status":
                self._last_status_monotonic = time.monotonic()
                self._last_status = payload
                self._publish_text(self.status_pub, payload.to_json())
            elif kind == "result":
                self._publish_text(self.result_pub, str(payload))
            elif kind == "desired":
                self._publish_desired(payload)

    def _publish_diagnostics(self) -> None:
        age = None if self._last_status_monotonic is None else time.monotonic() - self._last_status_monotonic
        status = DiagnosticStatus()
        status.name = "STM_TEST serial bridge"
        status.hardware_id = str(self.get_parameter("serial_port").value)
        mismatches = []
        if self._last_status is not None:
            mismatches = desired_status_mismatches(
                self._worker.desired_snapshot(), self._last_status
            )
        if not self._connected:
            status.level = DiagnosticStatus.ERROR
            status.message = "disconnected"
        elif mismatches:
            status.level = DiagnosticStatus.WARN
            status.message = "connected; desired state does not match STATUS"
        else:
            status.level = DiagnosticStatus.OK
            status.message = "connected"
        status.values = [
            KeyValue(key="serial_port", value=status.hardware_id),
            KeyValue(key="detail", value=self._connection_detail),
            KeyValue(key="last_status_age_sec", value="unknown" if age is None else f"{age:.3f}"),
            KeyValue(key="queued_commands", value=str(self._worker.commands.qsize())),
            KeyValue(
                key="desired_state_mismatches",
                value="; ".join(mismatches) if mismatches else "none",
            ),
        ]
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diagnostic_pub.publish(array)

    def destroy_node(self):
        self._worker.stop()
        self._worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StmTestBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
