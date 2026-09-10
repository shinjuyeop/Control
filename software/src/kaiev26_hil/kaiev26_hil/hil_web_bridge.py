"""Local React console. ROS owns the bench; each Web control session has a lease.

Run one process/worker. All ROS callbacks and Web actions share the asyncio
thread; the optional UNO telemetry reader owns its separate read-only serial connection.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
import uuid
from collections import defaultdict, deque
from contextlib import asynccontextmanager, suppress
from dataclasses import asdict, fields
from pathlib import Path
from urllib.parse import urlsplit

import rclpy
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from kaiev26_msgs.msg import (
    ActuatorCommand,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    RuntimeStats,
    SteeringActuatorStatus,
    VehicleState,
)
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.convert import message_to_ordereddict
from starlette.middleware.trustedhost import TrustedHostMiddleware
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger

from .hil_scenario_runner import HilScenarioRunner
from .dashboard_display import DashboardDisplay
from .models import HilDesiredState, HilStatus, decode_json_fields
from .protocol import validate_command
from .remote_parameters import RemoteParameterClient
from .scenario import load_scenario
from .runtime_config import BOARDS, RUNTIME_STALE_SECONDS
from .runtime_health import RuntimeHealth

LEASE_SECONDS = 3.0
STALE_SECONDS = 1.0  # Console freshness, not a firmware timeout or a safety gate.
RATE_WINDOW_SECONDS = 5.0
GENERATOR_FIELDS = (
    "enabled",
    "speed_target_mps",
    "steering_target_rad",
    "brake_engage",
    "web_heartbeat_timeout_sec",
)


def json_message(message):
    # ROS explicitly uses +inf for an unavailable age. JSON sends it as null.
    def convert(value):
        if isinstance(value, dict):
            return {key: convert(item) for key, item in value.items()}
        if isinstance(value, (tuple, list)):
            return [convert(item) for item in value]
        if isinstance(value, float) and not math.isfinite(value):
            return None
        return value

    return convert(message_to_ordereddict(message))


async def ros_response(future, timeout=2.0):
    if future is None:
        raise ValueError("ROS service is unavailable")
    ready = asyncio.get_running_loop().create_future()

    def finished(result):
        if not ready.done():
            try:
                ready.set_result(result.result())
            except Exception as exc:
                ready.set_exception(exc)

    future.add_done_callback(finished)
    return await asyncio.wait_for(ready, timeout)


class WebRos(Node):
    def __init__(self):
        super().__init__("hil_web_bridge")
        self.samples = {}
        self.runtime_health = RuntimeHealth()
        self.received_times = defaultdict(deque)
        self.received_count = defaultdict(int)
        self.connected = False
        self.desired = {}
        self.events = deque(maxlen=150)
        self.sequence = 0
        self.pending = None
        self.generator = {}
        self.generator_received = float("-inf")
        self.command_pub = self.create_publisher(String, "/hil/command", 20)
        self.heartbeat_pub = self.create_publisher(Empty, "/hil/web/heartbeat", 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/hil/connected", self.on_connected, latched)
        self.create_subscription(String, "/hil/desired_state", self.on_desired, latched)
        self.create_subscription(String, "/hil/status", self.on_status, 10)
        self.create_subscription(String, "/hil/command_result", self.on_result, 20)
        from diagnostic_msgs.msg import DiagnosticArray

        for msg_type, topic in (
            (DriveActuatorStatus, "/drive/status"),
            (BrakeActuatorStatus, "/brake/status"),
            (SteeringActuatorStatus, "/steering/status"),
            (VehicleState, "/vehicle/state"),
            (ActuatorCommand, "/planning/command"),
            (ActuatorCommand, "/vehicle/command"),
            (DiagnosticArray, "/diagnostics"),
            *((RuntimeStats, topic) for topic in BOARDS),
        ):
            self.create_subscription(
                msg_type,
                topic,
                lambda message, name=topic: self.receive(name, json_message(message)),
                qos_profile_sensor_data,
            )
        self.hil_services = {
            name: self.create_client(Trigger, "/hil/" + name)
            for name in ("reset_safe_state", "refresh_status", "reapply_desired_state")
        }
        self.generator_client = RemoteParameterClient(self, "hil_command_generator")
        self.generator_query = self.create_client(
            GetParameters, "/hil_command_generator/get_parameters"
        )

    def event(self, kind, text):
        self.sequence += 1
        self.events.appendleft(
            dict(id=self.sequence, time=time.time(), kind=kind, text=text)
        )

    def on_connected(self, message):
        if self.connected != message.data:
            self.event(
                "link",
                "STM_TEST connected" if message.data else "STM_TEST disconnected",
            )
        self.connected = bool(message.data)

    def on_status(self, message):
        try:
            value = decode_json_fields(
                message.data, HilStatus, [f.name for f in fields(HilStatus)]
            )
            if value["protocol"] < 2:
                for name in (
                    "ez_target_rpm",
                    "ez_running",
                    "ez_speed_control",
                    "ez_command_rpm",
                    "ez_command_current",
                    "ez_rx_age_ms",
                    "keya_enabled",
                    "keya_rx_age_ms",
                ):
                    value[name] = None
            self.receive("/hil/status", value)
        except ValueError as exc:
            self.event("error", f"STATUS decode: {exc}")

    def receive(self, topic, data):
        now = time.monotonic()
        self.samples[topic] = (now, data)
        if topic in BOARDS:
            self.runtime_health.receive(topic, data, now)
        arrivals = self.received_times[topic]
        arrivals.append(now)
        while arrivals and arrivals[0] < now - RATE_WINDOW_SECONDS:
            arrivals.popleft()
        self.received_count[topic] += 1

    def topic_snapshot(self, now):
        topics = {}
        for name, (received, data) in self.samples.items():
            arrivals = self.received_times[name]
            while arrivals and arrivals[0] < now - RATE_WINDOW_SECONDS:
                arrivals.popleft()
            span = arrivals[-1] - arrivals[0] if len(arrivals) > 1 else 0.0
            period = span / (len(arrivals) - 1) if span > 0 else None
            stale_after = RUNTIME_STALE_SECONDS if name in BOARDS else STALE_SECONDS
            fresh = now - received < stale_after
            topics[name] = dict(
                data=data,
                age_ms=round((now - received) * 1000),
                fresh=fresh,
                rx_count=self.received_count[name],
                rx_hz=(1.0 / period if period else None) if fresh else 0.0,
                period_ms=period * 1000 if period else None,
            )
        return topics

    def on_desired(self, message):
        try:
            self.desired = decode_json_fields(
                message.data, HilDesiredState, [f.name for f in fields(HilDesiredState)]
            )
        except ValueError as exc:
            self.event("error", f"Desired state decode: {exc}")

    def on_result(self, message):
        self.event(
            "error" if message.data.startswith("ERR") else "response", message.data
        )
        if self.pending:
            command, future = self.pending
            if not future.done() and (
                message.data == "OK " + command or message.data.startswith("ERR")
            ):
                future.set_result(message.data)

    def hil_fresh(self):
        sample = self.samples.get("/hil/status")
        return bool(
            self.connected and sample and time.monotonic() - sample[0] < STALE_SECONDS
        )

    async def command(self, command):
        self.event("command", command)
        future = asyncio.get_running_loop().create_future()
        self.pending = (command, future)
        try:
            self.command_pub.publish(String(data=command))
            result = await asyncio.wait_for(future, 2.5)
            if result.startswith("ERR"):
                raise ValueError(result)
            return result
        finally:
            self.pending = None

    async def service(self, name):
        client = self.hil_services[name]
        if not client.service_is_ready():
            raise ValueError(f"{name} service is unavailable")
        response = await ros_response(client.call_async(Trigger.Request()))
        if not response.success:
            raise ValueError(response.message)
        self.event("service", response.message)
        return response.message

    async def production(self, values):
        response = await ros_response(
            self.generator_client.set_parameters(
                [Parameter(name, value=value) for name, value in values.items()]
            )
        )
        if not response.result.successful:
            raise ValueError(response.result.reason)
        self.generator.update(values)
        self.event("command", "Command source: " + json.dumps(values))
        return "Command generator parameters applied"

    async def read_generator(self):
        try:
            if not self.generator_query.service_is_ready():
                return
            result = await ros_response(
                self.generator_query.call_async(
                    GetParameters.Request(names=list(GENERATOR_FIELDS))
                )
            )
            self.generator = {
                name: Parameter.from_parameter_msg(
                    ParameterMessage(name=name, value=value)
                ).value
                for name, value in zip(GENERATOR_FIELDS, result.values)
            }
            self.generator_received = time.monotonic()
        except (ValueError, TimeoutError):
            pass

    def conflicts(self, own_scenario=False):
        allowed = {"hil_web_bridge"}
        if own_scenario:
            allowed.add("hil_scenario_runner")
        names = [
            info.node_name
            for info in self.get_publishers_info_by_topic("/hil/command")
            if info.node_name not in allowed
        ]
        names += [
            info.node_name
            for info in self.get_publishers_info_by_topic("/planning/command")
            if info.node_name != "hil_command_generator"
        ]
        # Another Web bridge/generator with the same name is still a competing writer.
        for topic, name in (
            ("/hil/command", "hil_web_bridge"),
            ("/planning/command", "hil_command_generator"),
        ):
            if (
                sum(
                    i.node_name == name
                    for i in self.get_publishers_info_by_topic(topic)
                )
                > 1
            ):
                names.append("duplicate " + name)
        return sorted(set(names))


class Console:
    def __init__(self, scenario_dir, results_dir):
        self.scenarios = {p.stem: p for p in Path(scenario_dir).glob("*.yaml")}
        self.definitions = {
            name: load_scenario(path) for name, path in self.scenarios.items()
        }
        self.node = WebRos()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.results_dir = Path(results_dir)
        self.owner = None
        self.control_revision = 0
        self.heartbeat_at = 0.0
        self.armed = False
        self.runner = None
        self.run_id = None
        self.last_run = None
        self.lock = asyncio.Lock()
        self.background = set()
        self.closing = False
        self.releasing = False
        self.generator_poll = None
        self.pump_task = None

    def spawn(self, coroutine):
        task = asyncio.create_task(coroutine)
        self.background.add(task)

        def finished(done):
            self.background.discard(done)
            if not done.cancelled() and done.exception():
                self.node.event("error", str(done.exception()))

        task.add_done_callback(finished)
        return task

    async def pump(self):
        next_poll = next_heartbeat = 0.0
        while True:
            self.executor.spin_once(timeout_sec=0)
            now = time.monotonic()
            if now >= next_poll and (
                self.generator_poll is None or self.generator_poll.done()
            ):
                self.generator_poll = self.spawn(self.node.read_generator())
                next_poll = now + 1.0
            if (
                self.owner
                and not self.releasing
                and now - self.heartbeat_at >= LEASE_SECONDS
            ):
                self.releasing = True
                self.armed = False
                self.spawn(
                    self.release(self.owner, "Operator heartbeat expired", force=True,
                                 expected_revision=self.control_revision)
                )
            if (
                self.owner
                and self.armed
                and not self.releasing
                and now >= next_heartbeat
            ):
                self.node.heartbeat_pub.publish(Empty())
                next_heartbeat = now + 0.2
            if self.runner and self.runner.complete:
                self.last_run = dict(self.runner.result, id=self.run_id)
                self.executor.remove_node(self.runner)
                self.runner.destroy_node()
                self.runner = None
                self.armed = False
                self.releasing = True
                self.spawn(self.finish_run())
            # spin_once dispatches only one callback. Drive/steering alone add
            # 100 status messages/s, and an active scenario subscribes again.
            # A 5 ms sleep capped dispatch below the full bench's callback rate
            # and let queued pre-fault samples consume the expectation window.
            await asyncio.sleep(0.001)

    async def finish_run(self):
        try:
            async with self.lock:
                await self.reset()
        finally:
            self.releasing = False

    async def reset(self):
        # Safe State means queued bridge defaults, never physical stop proof.
        errors = []
        try:
            await self.node.production({"enabled": False})
        except (ValueError, TimeoutError) as exc:
            errors.append("Command source: " + (str(exc) or "timeout"))
        try:
            await self.node.service("reset_safe_state")
        except (ValueError, TimeoutError) as exc:
            errors.append("Safe State: " + (str(exc) or "timeout"))
        for error in errors:
            self.node.event("error", error)
        if errors:
            raise ValueError("; ".join(errors))
        return "Command source disabled; Safe State queued"

    async def stop_runner(self, reason):
        if not self.runner:
            return
        runner, self.runner = self.runner, None
        runner.complete = True
        # Drain issued parameter service requests before disabling the source.
        for request in runner.parameter_requests:
            if not request.done():
                with suppress(ValueError, TimeoutError):
                    await ros_response(request)
        runner.infrastructure_failures.append(reason)
        try:
            runner._finish(outcome="CANCELLED")
            self.last_run = dict(runner.result, id=self.run_id)
        finally:
            self.executor.remove_node(runner)
            runner.destroy_node()
        self.node.event("scenario", reason)

    async def release(self, client, reason="Control released", force=False,
                      expected_revision=None):
        if not force and self.owner != client:
            return
        revision = self.control_revision if expected_revision is None else expected_revision
        async with self.lock:
            if force and expected_revision is None:
                # Shutdown owns the whole console. Watchdog callbacks instead
                # supply the exact revision they observed and remain fenced.
                client, revision = self.owner, self.control_revision
            # A delayed disconnect/watchdog from the previous owner must not reset
            # the new owner's bench, including when the same client later reclaims.
            if self.owner != client or self.control_revision != revision:
                return
            self.releasing = True
            self.armed = False
            self.control_revision += 1
            try:
                await self.stop_runner(reason)
                with suppress(ValueError):
                    await self.reset()
                self.node.event("control", reason)
                self.owner = None
            finally:
                self.control_revision += 1
                self.releasing = False

    async def takeover(self, client, request):
        if request.get("confirmed") is not True or type(request.get("expected_revision")) is not int:
            raise ValueError("현재 조작권 상태를 확인하고 강제 인계를 승인하세요.")
        async with self.lock:
            if self.closing or self.releasing:
                raise ValueError("이전 조작을 정리하고 있습니다.")
            if request["expected_revision"] != self.control_revision:
                raise ValueError("조작권 상태가 변경됐습니다. 현재 상태를 확인하고 다시 시도하세요.")
            if self.owner == client:
                return "이미 이 브라우저가 조작권을 갖고 있습니다."
            if self.owner is None:
                raise ValueError("현재 조작자가 없습니다. 조작권 가져오기를 사용하세요.")
            if self.runner and any(not future.done() for future in self.runner.parameter_requests):
                raise ValueError("시나리오 명령 응답을 기다리고 있습니다. 응답 후 다시 인계하세요.")
            conflicts = self.node.conflicts(self.runner is not None)
            if conflicts:
                raise ValueError("다른 명령 발행자를 먼저 종료하세요: " + ", ".join(conflicts))
            self.releasing = True
            self.armed = False
            self.control_revision += 1
            self.node.event("control", "Control takeover requested; stopping Host command source")
            try:
                # Finish the old action (the shared lock is held), cancel the
                # scenario, then confirm generator disarm before granting control.
                # Do not call Bench Reset: manual inputs/E-stop must be retained.
                try:
                    await self.stop_runner("Operator transferred control")
                finally:
                    await self.node.production({"enabled": False})
                conflicts = self.node.conflicts()
                if conflicts:
                    raise ValueError("다른 명령 발행자가 있습니다: " + ", ".join(conflicts))
                self.owner, self.heartbeat_at = client, time.monotonic()
                self.node.event("control", "Control transferred; generator disabled; operator inputs retained")
                return "조작권을 가져왔습니다. Host 명령은 꺼졌으며 입력은 유지됩니다. 필요하면 다시 ARM하세요."
            finally:
                # Also invalidate confirmations made during a failed transfer.
                # On failure the former owner remains; no new authority is granted.
                self.control_revision += 1
                self.releasing = False

    def require_control(self, client, *, check_writers=True):
        if (
            self.releasing
            or self.owner != client
            or time.monotonic() - self.heartbeat_at >= LEASE_SECONDS
        ):
            raise ValueError("먼저 조작권을 가져오세요.")
        if check_writers:
            conflicts = self.node.conflicts(self.runner is not None)
            if conflicts:
                raise ValueError("다른 명령 발행자가 있습니다: " + ", ".join(conflicts))

    async def action(self, client, request):
        action = request.get("action")
        if action == "claim":
            async with self.lock:
                if self.closing or self.releasing:
                    raise ValueError("이전 조작을 정리하고 있습니다.")
                if self.owner and self.owner != client:
                    raise ValueError("다른 브라우저가 조작 중입니다.")
                conflicts = self.node.conflicts(self.runner is not None)
                if conflicts:
                    raise ValueError(
                        "다른 명령 발행자를 먼저 종료하세요: " + ", ".join(conflicts)
                    )
                if self.owner != client:
                    self.control_revision += 1
                self.owner, self.heartbeat_at = client, time.monotonic()
                self.node.event("control", "Web operator acquired control")
                return "조작권을 가져왔습니다."
        if action == "takeover":
            return await self.takeover(client, request)
        if action == "release":
            await self.release(client)
            return "조작권을 해제했습니다."
        stopping = action in ("safe", "stop_scenario", "refresh") or (
            action == "production"
            and isinstance(request.get("values"), dict)
            and request["values"].get("enabled") is False
        )
        revision = self.control_revision
        self.require_control(client, check_writers=not stopping)
        async with self.lock:
            if revision != self.control_revision:
                raise ValueError("조작권이 변경되어 이전 요청을 취소했습니다.")
            self.require_control(client, check_writers=not stopping)
            if action in ("safe", "stop_scenario"):
                self.armed = False
                await self.stop_runner("Operator stopped scenario")
                result = await self.reset()
                # Only the operator's explicit Bench Reset acknowledges history.
                # Disconnect/watchdog/scenario cleanup must retain diagnostic evidence.
                if action == "safe":
                    cleared = self.node.runtime_health.reset_deadline_history(time.monotonic())
                    note = (
                        "Runtime miss 기준 초기화: " + ", ".join(cleared)
                        if cleared else "Runtime 미수신/지연/invalid: miss 기준 유지"
                    )
                    self.node.event("runtime", note)
                    result += "; " + note
                return result
            if action == "refresh":
                return await self.node.service("refresh_status")
            if self.runner:
                raise ValueError("시나리오 실행 중에는 수동 입력을 변경할 수 없습니다.")
            if not self.node.hil_fresh() and not (
                action == "production"
                and isinstance(request.get("values"), dict)
                and request["values"].get("enabled") is False
            ):
                raise ValueError("STM_TEST의 최신 상태를 수신한 뒤 조작할 수 있습니다.")
            if action == "command":
                raw = request.get("command")
                if not isinstance(raw, str) or len(raw) > 100:
                    raise ValueError("Invalid HIL command")
                command = validate_command(raw)
                if command in ("STATUS", "HELP"):
                    raise ValueError("Use the refresh action")
                status = self.node.samples["/hil/status"][1]
                if command.startswith("SET PLANT") and status["protocol"] < 2:
                    raise ValueError(
                        "Plant control requires STM_TEST protocol 2 or later"
                    )
                if command.startswith(("SET SPEED ", "SET RPM ")) and status["plant"]:
                    raise ValueError(
                        "직접 피드백 모드에서만 속도/RPM을 주입할 수 있습니다."
                    )
                return await self.node.command(command)
            if action == "production":
                values = request.get("values")
                if not isinstance(values, dict):
                    raise ValueError("Production values required")
                enabled, brake = values.get("enabled"), values.get("brake_engage")
                if type(enabled) is not bool or type(brake) is not bool:
                    raise ValueError("enabled/brake_engage must be boolean")
                for name, low, high in (
                    ("speed_target_mps", -50, 50),
                    ("steering_target_rad", -1, 1),
                ):
                    value = values.get(name)
                    if (
                        type(value) not in (float, int)
                        or not math.isfinite(value)
                        or not low <= value <= high
                    ):
                        raise ValueError(f"{name} must be between {low} and {high}")
                if enabled:
                    await self.check_watchdog()
                self.armed = enabled
                try:
                    return await self.node.production(
                        dict(
                            enabled=enabled,
                            brake_engage=brake,
                            speed_target_mps=float(values["speed_target_mps"]),
                            steering_target_rad=float(values["steering_target_rad"]),
                        )
                    )
                except Exception:
                    self.armed = False
                    raise
            if action == "scenario":
                name = request.get("name")
                if not isinstance(name, str) or name not in self.scenarios:
                    raise ValueError("Unknown scenario")
                if request.get("arm") is not True:
                    raise ValueError("Scenario command source must be explicitly armed")
                await self.check_watchdog()
                self.run_id = uuid.uuid4().hex
                destination = self.results_dir / self.run_id
                destination.mkdir(parents=True, exist_ok=False)
                self.runner = HilScenarioRunner(
                    parameter_overrides=[
                        Parameter("scenario_file", value=str(self.scenarios[name])),
                        Parameter("results_dir", value=str(destination)),
                    ],
                    service_wait_sec=0.0,
                )
                self.executor.add_node(self.runner)
                self.armed = True
                self.node.event("scenario", f"Started {name}")
                return "시나리오를 시작했습니다."
            raise ValueError("Unknown action")

    async def check_watchdog(self):
        await self.node.read_generator()
        if time.monotonic() - self.node.generator_received > 2 or not (
            0
            < (self.node.generator.get("web_heartbeat_timeout_sec") or 0)
            <= LEASE_SECONDS
        ):
            raise ValueError("Web 전용 launch의 heartbeat watchdog이 필요합니다.")
        self.node.heartbeat_pub.publish(Empty())
        await asyncio.sleep(0.1)

    def snapshot(self, client):
        now = time.monotonic()
        topics = self.node.topic_snapshot(now)
        scenario = self.last_run
        if self.runner:
            scenario = dict(
                id=self.run_id,
                scenario=self.runner.definition.name,
                result="RUNNING",
                duration_sec=now - self.runner.start_monotonic,
                records=list(self.runner.records),
                step_index=self.runner.step_index,
                step_count=len(self.runner.definition.steps),
                expectations=self.runner.expectation_state,
                infrastructure_failures=list(self.runner.infrastructure_failures),
            )
        return dict(
            type="state",
            time=time.time(),
            topics=topics,
            runtime=self.node.runtime_health.snapshot(now),
            connected=self.node.connected,
            desired=self.node.desired,
            generator=self.node.generator,
            generator_fresh=now - self.node.generator_received < 2,
            control=dict(
                mine=self.owner == client and not self.releasing,
                occupied=self.owner is not None,
                armed=self.armed,
                revision=self.control_revision,
                transitioning=self.releasing,
            ),
            events=list(self.node.events),
            scenario=scenario,
            catalog=[
                dict(id=name, **asdict(definition))
                for name, definition in self.definitions.items()
            ],
        )

    async def close(self):
        self.closing = True
        if self.owner or self.runner:
            await self.release(self.owner, "Web server shutdown", force=True)
        tasks = list(self.background)
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
        if self.pump_task:
            self.pump_task.cancel()
            with suppress(asyncio.CancelledError):
                await self.pump_task
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()


def create_app(
    scenario_dir, results_dir, static_dir, *, dashboard_port="", dashboard_library=None
):
    @asynccontextmanager
    async def lifespan(app):
        console = Console(scenario_dir, results_dir)
        app.state.console = console
        console.pump_task = asyncio.create_task(console.pump())
        display = DashboardDisplay(dashboard_port, dashboard_library)
        app.state.display = display
        display.start()
        try:
            yield
        finally:
            try:
                await asyncio.to_thread(display.stop)
            finally:
                await console.close()

    app = FastAPI(lifespan=lifespan, docs_url=None, redoc_url=None)
    app.add_middleware(
        TrustedHostMiddleware,
        # A shared node can have a different IPv4 address in the recipient's tailnet.
        allowed_hosts=[
            "127.0.0.1", "localhost", "[::1]", "100.112.72.27", "100.112.72.26"
        ],
    )

    @app.get("/api/health")
    async def health():
        return {
            "service": "kaiev26_hil",
            "mode": "local",
            "serial_owner": "stm_test_bridge",
        }

    @app.get("/api/history")
    async def history():
        results = []
        for path in sorted(
            Path(results_dir).glob("*/*.json"),
            key=lambda p: p.stat().st_mtime,
            reverse=True,
        )[:30]:
            try:
                results.append(json.loads(path.read_text()))
            except (OSError, ValueError):
                continue
        return results

    @app.websocket("/ws")
    async def websocket(ws: WebSocket):
        origin = urlsplit(ws.headers.get("origin", ""))
        # A hostile website must not be able to drive a localhost HIL console.
        if origin.scheme not in ("http", "https") or origin.netloc != ws.headers.get(
            "host"
        ):
            await ws.close(code=1008)
            return
        await ws.accept()
        console = app.state.console
        client = uuid.uuid4().hex
        send_lock = asyncio.Lock()
        action_task = None

        async def send(payload):
            async with send_lock:
                await ws.send_json(payload)

        async def states():
            while True:
                snapshot = console.snapshot(client)
                snapshot["display"] = app.state.display.snapshot()
                await send(snapshot)
                await asyncio.sleep(0.2)

        async def perform(request):
            try:
                message = await console.action(client, request)
                await send(
                    dict(type="result", id=request.get("id"), ok=True, message=message)
                )
            except (ValueError, TimeoutError, OSError) as exc:
                message = (
                    str(exc) or "응답 시간이 초과됐습니다. 보드 상태를 확인하세요."
                )
                console.node.event("error", message)
                await send(
                    dict(type="result", id=request.get("id"), ok=False, message=message)
                )

        sender = asyncio.create_task(states())
        try:
            while True:
                raw = await ws.receive_text()
                if len(raw) > 4096:
                    await ws.close(code=1009)
                    break
                try:
                    request = json.loads(raw)
                except ValueError:
                    await ws.close(code=1008)
                    break
                if not isinstance(request, dict):
                    await ws.close(code=1008)
                    break
                if request.get("action") == "heartbeat":
                    if console.owner == client:
                        console.heartbeat_at = time.monotonic()
                elif action_task and not action_task.done():
                    await send(
                        dict(
                            type="result",
                            id=request.get("id"),
                            ok=False,
                            message="이전 명령의 응답을 기다리고 있습니다.",
                        )
                    )
                else:
                    action_task = asyncio.create_task(perform(request))
        except WebSocketDisconnect:
            pass
        finally:
            sender.cancel()
            with suppress(asyncio.CancelledError, WebSocketDisconnect, RuntimeError):
                await sender
            if action_task:
                # A published command cannot be recalled. Finish its bounded ACK wait
                # before the final disarm/reset so it cannot run after cleanup.
                with suppress(WebSocketDisconnect, RuntimeError):
                    await action_task
            await console.release(client, "Browser disconnected")

    static_dir = Path(static_dir)
    if static_dir.is_dir():
        # colcon --symlink-install links trusted built assets out of package share.
        app.mount(
            "/",
            StaticFiles(directory=str(static_dir), html=True, follow_symlink=True),
            name="web",
        )
    else:

        @app.get("/")
        async def missing_build():
            return {
                "message": "Build the console first: cd web && npm ci && npm run build"
            }

    return app


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--static-dir")
    parser.add_argument("--dashboard-port", default="")
    parser.add_argument("--dashboard-library")
    parser.add_argument(
        "--results-dir",
        default=str(Path.home() / ".local/state/kaiev26_hil/web_results"),
    )
    options, ros_args = parser.parse_known_args(args)
    import uvicorn

    share = Path(get_package_share_directory("kaiev26_hil"))
    static_dir = options.static_dir or share / "web"
    # Uvicorn owns shutdown signals; ROS must stay alive for lifespan cleanup.
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        uvicorn.run(
            create_app(
                share / "config/scenarios",
                options.results_dir,
                static_dir,
                dashboard_port=options.dashboard_port,
                dashboard_library=options.dashboard_library
                or share / "dashboard/libdashboard_renderer.so",
            ),
            host="127.0.0.1",
            port=options.port,
            timeout_graceful_shutdown=8,
            ws_max_size=4096,
            ws_max_queue=4,
        )
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
