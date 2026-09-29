"""Vehicle monitoring console."""

import argparse
import asyncio
from contextlib import asynccontextmanager, suppress
from pathlib import Path
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import WebSocket, WebSocketDisconnect
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

from .common.profiles import VEHICLE
from .common.telemetry import ObserverNode
from .common.web import (accept_websocket, add_web_arguments, create_web_app,
                  mount_frontend, positive_number, stream_states)


# Same observation thresholds as the former Qt Vehicle Dashboard. Diagnostics
# arrive at ~1 Hz; RuntimeStats also requires firmware snapshot progress (3 s).
VEHICLE_STALE_SECONDS = {
    "/vehicle/command": 0.3, "/drive/status": 0.3,
    "/brake/status": 0.3, "/steering/status": 0.5,
    "/diagnostics": 3.0,
}


class VehicleObserver(ObserverNode):
    def __init__(self, *, stale_seconds=None):
        super().__init__(
            "vehicle_console", stale_seconds=stale_seconds or VEHICLE_STALE_SECONDS,
            # No remote parameter services or rosout output. rclpy retains its
            # standard parameter-events publisher; there are no command publishers.
            start_parameter_services=False, enable_rosout=False,
        )

    def receive(self, topic, data):
        # Header stamps and raw RuntimeStats duplicate the rendered observations.
        # Keep every status field consumed by the shared instruments, and retain
        # diagnostics values for detailed inspection without a second request API.
        data = {key: value for key, value in data.items() if key != "header"}
        super().receive(topic, data)

    def snapshot(self):
        now = time.monotonic()
        topics = self.topic_snapshot(now)
        for topic in ("/stm_a/runtime_stats", "/stm_b/runtime_stats"):
            if topic in topics:
                topics[topic] = {**topics[topic], "data": {}}
        return dict(type="state", **VEHICLE.metadata(), time=time.time(), topics=topics,
                    runtime=runtime_payload(self.runtime_health.snapshot(now)))


def runtime_payload(boards):
    """Select the shared runtime view fields used by instruments for the WAN.

    Interpretation stays in RuntimeHealth; raw counters and duplicate derived
    fields are available through ROS/Foxglove, not repeated in every Web frame.
    """
    result = {}
    task_fields = ("task_id", "present", "name", "stack", "rate_hz", "wall", "jitter",
                   "deadline_misses", "deadline_misses_delta")
    for name, board in boards.items():
        data = board["data"]
        if data:
            data = {key: data[key] for key in (
                "uptime_ms", "core_clock_hz", "timing_available", "heap",
                "instrumentation_errors", "instrumentation_errors_delta",
                "cpu_load_percent", "estop", "deadline_misses", "deadline_misses_delta",
            )}
            data["tasks"] = [{key: task[key] for key in task_fields if key in task}
                             for task in board["data"]["tasks"]]
            worst = board["data"]["worst_stack"]
            data["worst_stack"] = {key: worst[key] for key in ("name", "stack")} if worst else None
        result[name] = {**board, "data": data}
    return result


def create_app(static_dir, *, allowed_hosts=(), update_hz=5.0, stale_seconds=None):
    @asynccontextmanager
    async def lifespan(app):
        node = VehicleObserver(stale_seconds=stale_seconds)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        app.state.observer = node
        app.state.executor = executor

        async def pump():
            while True:
                executor.spin_once(timeout_sec=0)
                await asyncio.sleep(0.001)

        task = asyncio.create_task(pump())
        try:
            yield
        finally:
            task.cancel()
            try:
                with suppress(asyncio.CancelledError):
                    await task
            finally:
                executor.remove_node(node)
                node.destroy_node()
                executor.shutdown()

    app = create_web_app(lifespan, allowed_hosts=allowed_hosts)

    @app.get("/api/health")
    async def health():
        return dict(service="vehicle_console", **VEHICLE.metadata(), update_hz=update_hz)

    @app.websocket("/ws")
    async def websocket(ws: WebSocket):
        if not await accept_websocket(ws):
            return
        sender = asyncio.create_task(stream_states(
            ws.send_json, app.state.observer.snapshot, update_hz,
        ))
        try:
            # This endpoint is an outbound telemetry stream. Every inbound text
            # or binary application message is rejected, including heartbeat,
            # commands, takeover, touch and attempts to change profile.
            while True:
                message = await ws.receive()
                if message["type"] == "websocket.disconnect":
                    break
                await ws.close(code=1008, reason="Vehicle telemetry WebSocket is read only")
                break
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()
            with suppress(asyncio.CancelledError, WebSocketDisconnect, RuntimeError):
                await sender

    mount_frontend(app, static_dir)
    return app


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    add_web_arguments(parser, default_port=8766)
    for name, default in (("command", 300), ("drive", 300), ("brake", 300), ("steering", 500)):
        parser.add_argument(f"--{name}-stale-ms", type=positive_number, default=default)
    options, ros_args = parser.parse_known_args(args)
    stale = {**VEHICLE_STALE_SECONDS,
             "/vehicle/command": options.command_stale_ms / 1000,
             "/drive/status": options.drive_stale_ms / 1000,
             "/brake/status": options.brake_stale_ms / 1000,
             "/steering/status": options.steering_stale_ms / 1000}
    import uvicorn

    static_dir = options.static_dir or Path(get_package_share_directory("vehicle_console")) / "web"
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        uvicorn.run(create_app(static_dir, allowed_hosts=options.allowed_hosts.split(","),
                               update_hz=options.update_hz, stale_seconds=stale),
                    host=options.bind, port=options.port, ws_max_size=4096,
                    ws_max_queue=4, timeout_graceful_shutdown=5)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
