"""Exercise the Web/ROS boundary without a serial device or physical controller.

The peer publishes known ROS observations and ACKs only; it is not a vehicle
simulator. Watchdog tests run the actual command generator and observe its topic.
"""

import time
import math
from contextlib import asynccontextmanager
from dataclasses import replace
from pathlib import Path

import pytest

pytest.importorskip("fastapi")
pytest.importorskip("httpx")
import rclpy
from fastapi.testclient import TestClient
from kaiev26_hil.hil_command_generator import HilCommandGenerator
from kaiev26_hil.hil_web_bridge import create_app
from kaiev26_hil.models import HilDesiredState
from kaiev26_msgs.msg import (
    ActuatorCommand,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)
from rclpy.node import Node
from rclpy.parameter import Parameter
from starlette.websockets import WebSocketDisconnect
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from test_bridge_worker import make_status


@pytest.fixture
def bench(tmp_path):
    rclpy.init()
    app = create_app(
        Path(__file__).parents[1] / "config/scenarios",
        tmp_path / "results",
        tmp_path / "web",
    )
    original_lifespan = app.router.lifespan_context
    observed = {"commands": [], "planning": [], "safe": 0, "publish": True}
    observed["drive"] = DriveActuatorStatus(
        part1_valid=True,
        speed_mps=1.2,
        part1_age_ms=12.0,
        part2_valid=False,
        part2_age_ms=750.0,
        motor_temperature_c=99.0,
    )

    @asynccontextmanager
    async def lifespan(application):
        async with original_lifespan(application):
            console = application.state.console
            peer = Node("web_test_peer")
            generator = HilCommandGenerator(
                parameter_overrides=[Parameter("web_heartbeat_timeout_sec", value=0.8)]
            )
            status_pub = peer.create_publisher(String, "/hil/status", 10)
            # Match the bridge's transient-local QoS for /hil/connected.
            from rclpy.qos import DurabilityPolicy, QoSProfile

            connected_pub = peer.create_publisher(
                Bool,
                "/hil/connected",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            desired_pub = peer.create_publisher(
                String,
                "/hil/desired_state",
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            )
            result_pub = peer.create_publisher(String, "/hil/command_result", 10)
            drive_pub = peer.create_publisher(DriveActuatorStatus, "/drive/status", 10)
            steering_pub = peer.create_publisher(
                SteeringActuatorStatus, "/steering/status", 10
            )
            status = replace(
                make_status(protocol=3),
                pedal=27.0,
                voltage=72.4,
                rpm=400.0,
                ez_rx=10,
                ez_rx_age_ms=12,
                keya_rx_age_ms=4294967295,
            )

            def publish():
                if not observed["publish"]:
                    return
                status_pub.publish(String(data=status.to_json()))
                connected_pub.publish(Bool(data=True))
                desired_pub.publish(String(data=HilDesiredState(pedal=65.0).to_json()))
                drive_pub.publish(observed["drive"])
                steering_pub.publish(
                    SteeringActuatorStatus(
                        communication_ok=True,
                        feedback_age_ms=float("inf"),
                        fault_bits=1,
                        target_wheel_angle_rad=math.pi / 6,
                    )
                )

            peer.create_timer(0.05, publish)

            def command(message):
                observed["commands"].append(message.data)
                result_pub.publish(String(data="OK " + message.data))

            peer.create_subscription(String, "/hil/command", command, 20)
            peer.create_subscription(
                ActuatorCommand,
                "/planning/command",
                lambda m: observed["planning"].append(
                    (time.monotonic(), m.speed_target_mps)
                ),
                10,
            )

            def safe(_request, response):
                observed["safe"] += 1
                response.success = True
                response.message = "safe state queued"
                return response

            peer.create_service(Trigger, "/hil/reset_safe_state", safe)
            peer.create_service(Trigger, "/hil/refresh_status", safe)
            console.executor.add_node(peer)
            console.executor.add_node(generator)
            observed["generator"] = generator
            try:
                yield
            finally:
                # Close while our owned peer can still answer the release services.
                if console.owner or console.runner:
                    await console.release(console.owner, force=True)
                console.executor.remove_node(peer)
                console.executor.remove_node(generator)
                peer.destroy_node()
                generator.destroy_node()

    app.router.lifespan_context = lifespan
    try:
        with TestClient(app, base_url="http://localhost") as client:
            yield client, observed
    finally:
        rclpy.shutdown()


def receive(ws, predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        message = ws.receive_json()
        if predicate(message):
            return message
    raise AssertionError("Web response deadline exceeded")


def request(ws, action, **values):
    ws.send_json(dict(id=action, action=action, **values))
    return receive(ws, lambda m: m.get("type") == "result" and m.get("id") == action)


def fresh(ws):
    return receive(
        ws,
        lambda m: (
            m.get("type") == "state"
            and m.get("topics", {}).get("/hil/status", {}).get("fresh")
            and m.get("generator_fresh")
        ),
    )


def test_receive_rate_measures_ros_arrivals_and_expires(bench):
    from unittest.mock import patch

    client, _ = bench

    async def measure():
        node = client.app.state.console.node
        with patch(
            "kaiev26_hil.hil_web_bridge.time.monotonic",
            side_effect=[10.0, 10.02, 10.04],
        ):
            for _ in range(3):
                node.receive("/rate_test", {})
        live = node.topic_snapshot(10.04)["/rate_test"]
        stopped = node.topic_snapshot(16.0)["/rate_test"]
        return live, stopped

    live, stopped = client.portal.call(measure)
    assert live["rx_hz"] == pytest.approx(50.0)
    assert live["period_ms"] == pytest.approx(20.0)
    assert live["rx_count"] == 3
    assert live["age_ms"] == 0
    assert stopped["rx_hz"] == 0.0
    assert stopped["period_ms"] is None
    assert not stopped["fresh"]


def test_web_requires_origin_and_single_control_owner(bench):
    client, observed = bench
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect(
            "ws://localhost/ws", headers={"origin": "http://untrusted.example"}
        ):
            pass
    with client.websocket_connect(
        "ws://localhost/ws", headers={"origin": "http://localhost"}
    ) as first:
        state = fresh(first)
        assert state["topics"]["/hil/status"]["data"]["pedal"] == 27
        assert state["desired"]["pedal"] == 65  # An intent is never an observation.
        assert not request(first, "command", command="SET PEDAL 30")["ok"]
        assert request(first, "claim")["ok"]
        with client.websocket_connect(
            "ws://localhost/ws", headers={"origin": "http://localhost"}
        ) as second:
            assert not request(second, "claim")["ok"]
            assert not request(second, "command", command="SET PEDAL 30")["ok"]
        assert request(first, "command", command="SET PEDAL 30")["ok"]
        assert observed["commands"] == ["SET PEDAL 30"]
        assert not request(first, "command", command="SET PEDAL 101")["ok"]
        state = receive(
            first,
            lambda m: m.get("type") == "state" and "/steering/status" in m["topics"],
        )
        assert state["topics"]["/steering/status"]["data"]["feedback_age_ms"] is None
        observed["publish"] = False
        state = receive(
            first,
            lambda m: (
                m.get("type") == "state" and not m["topics"]["/hil/status"]["fresh"]
            ),
        )
        assert not request(first, "command", command="SET PEDAL 5")["ok"]


def test_generator_stops_without_browser_heartbeat_and_requires_rearm(bench):
    client, observed = bench
    with client.websocket_connect(
        "ws://localhost/ws", headers={"origin": "http://localhost"}
    ) as ws:
        fresh(ws)
        assert request(ws, "claim")["ok"]
        values = dict(
            enabled=True,
            speed_target_mps=1.5,
            steering_target_rad=0.0,
            brake_engage=False,
        )
        assert request(ws, "production", values=values)["ok"]
        receive(
            ws, lambda m: m.get("type") == "state" and m["generator"].get("enabled")
        )
        assert observed["planning"]
        receive(ws, lambda m: m.get("type") == "state" and not m["control"]["occupied"])
        count = len(observed["planning"])
        time.sleep(1.0)
        assert len(observed["planning"]) == count
        assert not observed["generator"].get_parameter("enabled").value
        assert observed["safe"] >= 1
        assert request(ws, "claim")["ok"]
        ws.send_json({"action": "heartbeat"})
        time.sleep(0.3)
        assert not observed["generator"].get_parameter("enabled").value


def test_generator_watchdog_survives_missing_web_publisher(bench):
    client, observed = bench
    with client.websocket_connect(
        "ws://localhost/ws", headers={"origin": "http://localhost"}
    ) as ws:
        fresh(ws)
        assert request(ws, "claim")["ok"]
        assert request(
            ws,
            "production",
            values=dict(
                enabled=True,
                speed_target_mps=0.5,
                steering_target_rad=0.0,
                brake_engage=False,
            ),
        )["ok"]

        # Stop only the Web heartbeat source, while ROS/generator keep running.
        async def stop_heartbeat():
            client.app.state.console.armed = False

        client.portal.call(stop_heartbeat)
        time.sleep(1.2)
        assert not observed["generator"].get_parameter("enabled").value
        count = len(observed["planning"])
        time.sleep(0.2)
        assert len(observed["planning"]) == count


def test_scenario_uses_existing_runner_and_finishes_disarmed(bench):
    client, observed = bench
    with client.websocket_connect(
        "ws://localhost/ws", headers={"origin": "http://localhost"}
    ) as ws:
        fresh(ws)
        assert request(ws, "claim")["ok"]
        assert request(ws, "scenario", name="smoke_test", arm=True)["ok"]
        assert not request(ws, "command", command="SET PEDAL 50")["ok"]
        result = None
        deadline = time.monotonic() + 9
        while time.monotonic() < deadline:
            ws.send_json({"action": "heartbeat"})
            state = receive(ws, lambda m: m.get("type") == "state")
            if state.get("scenario", {}).get("result") == "PASS":
                result = state["scenario"]
                break
        assert result is not None
        assert any(
            r["kind"] == "expectation" and r["topic"] == "/drive/status"
            for r in result["records"]
        )
        receive(
            ws, lambda m: m.get("type") == "state" and not m["generator"].get("enabled")
        )
        assert observed["safe"] >= 1
        assert client.get("/api/history").json()[0]["result"] == "PASS"


def test_cancelling_a_scenario_saves_result_and_disarms(bench):
    client, observed = bench
    with client.websocket_connect(
        "ws://localhost/ws", headers={"origin": "http://localhost"}
    ) as ws:
        fresh(ws)
        assert request(ws, "claim")["ok"]
        assert request(ws, "scenario", name="estop_test", arm=True)["ok"]
        assert request(ws, "stop_scenario")["ok"]
        assert not observed["generator"].get_parameter("enabled").value
        assert observed["safe"] >= 1
        assert client.get("/api/history").json()[0]["result"] == "CANCELLED"


def test_react_controls_and_live_observations_in_browser(bench, tmp_path):
    playwright = pytest.importorskip("playwright.sync_api")
    import shutil
    import socket

    import uvicorn

    chrome = shutil.which("google-chrome")
    if chrome is None:
        pytest.skip("Chrome is not installed")
    static_dir = Path(__file__).parents[1] / "web/dist"
    if not (static_dir / "index.html").is_file():
        pytest.skip("Build the React console before the browser test")
    from fastapi.staticfiles import StaticFiles

    client, observed = bench
    # The same app/executor already running in TestClient's event loop serves
    # the browser through Uvicorn. No extra ROS stack or serial emulator.
    client.app.router.routes = [
        route
        for route in client.app.router.routes
        if getattr(route, "path", None) != "/"
    ]
    client.app.mount("/", StaticFiles(directory=static_dir, html=True))
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    address = f"http://127.0.0.1:{sock.getsockname()[1]}"
    server = uvicorn.Server(
        uvicorn.Config(client.app, lifespan="off", log_level="warning")
    )

    async def serve():
        await server.serve(sockets=[sock])

    task = client.portal.start_task_soon(serve)
    try:
        with playwright.sync_playwright() as browser_tools:
            browser = browser_tools.chromium.launch(
                executable_path=chrome, headless=True
            )
            try:
                page = browser.new_page(viewport={"width": 1440, "height": 1050})
                errors = []
                page.on("pageerror", lambda error: errors.append(str(error)))
                page.goto(address)
                page.get_by_text("Web bridge 연결됨", exact=True).wait_for()
                page.get_by_role("button", name="조작권 가져오기").click()
                page.get_by_role("button", name="조작권 해제").wait_for()
                page.get_by_role("navigation").get_by_role(
                    "button", name="벤치", exact=True
                ).click()
                page.get_by_role("group", name="운전 모드").get_by_role(
                    "button", name="AUTO", exact=True
                ).click()
                page.get_by_role("status").get_by_text("OK SET MODE AUTO", exact=True).wait_for()
                assert "SET MODE AUTO" in observed["commands"]
                assert (
                    page.get_by_role("group", name="운전 모드")
                    .get_by_role("button", name="MANUAL", exact=True)
                    .get_attribute("aria-pressed")
                    == "true"
                )
                page.get_by_role("button", name="30%", exact=True).click()
                page.get_by_role("status").get_by_text("OK SET PEDAL 30", exact=True).wait_for()
                assert "SET PEDAL 30" in observed["commands"]
                # ACKs do not overwrite the peer's observed pedal value (27%).
                assert page.get_by_label("가속 페달", exact=True).input_value() == "27"
                page.get_by_role("switch", name="대시보드 E-stop", exact=True).click()
                page.get_by_role("status").get_by_text("OK SET ESTOP DASH ON", exact=True).wait_for()
                assert "SET ESTOP DASH ON" in observed["commands"]
                part2 = page.locator(".frame-card").filter(
                    has=page.get_by_text("EZkontrol Part II", exact=True)
                )
                assert "750" in part2.inner_text() and "INVALID" in part2.inner_text()
                page.get_by_text("조향 상태 플래그", exact=True).click()
                informational_flag = page.locator(".flag-chip").filter(
                    has=page.get_by_text("Enable 피드백 지원", exact=True)
                )
                assert "danger" not in informational_flag.get_attribute("class")
                assert (
                    "FAULT 0x0001"
                    not in page.get_by_label("메시지가 보고한 상태").inner_text()
                )
                page.get_by_text("조향 상태 플래그", exact=True).click()
                speed_metric = page.locator(".metric").filter(
                    has=page.get_by_text("관측 속도", exact=True)
                )
                assert (
                    "4.32" in speed_metric.inner_text()
                    and "km/h" in speed_metric.inner_text()
                )
                angle_fact = page.locator(".fact").filter(
                    has=page.get_by_text("목표 각도", exact=True)
                )
                assert "30.00°" in angle_fact.inner_text()
                page.get_by_label("목표 속도", exact=True).fill("36")
                page.get_by_label("목표 조향각", exact=True).fill("30")
                page.get_by_role("button", name="ARM · 명령 발행", exact=True).click()
                page.get_by_role("dialog").get_by_role(
                    "button", name="ARM · 발행", exact=True
                ).click()
                page.get_by_text(
                    "Command generator parameters applied", exact=True
                ).wait_for()
                assert observed["generator"].get_parameter("enabled").value
                assert observed["generator"].get_parameter(
                    "speed_target_mps"
                ).value == pytest.approx(10.0)
                assert observed["generator"].get_parameter(
                    "steering_target_rad"
                ).value == pytest.approx(math.pi / 6)
                for name in [
                    "벤치",
                    "가상 LCD",
                    "고장 주입",
                    "시나리오",
                    "진단 · 로그",
                ]:
                    page.get_by_role("navigation").get_by_role(
                        "button", name=name, exact=True
                    ).click()
                    page.get_by_role("heading", name=name, exact=True).wait_for()
                    assert not page.evaluate(
                        "document.documentElement.scrollWidth > innerWidth"
                    )
                page.get_by_role("navigation").get_by_role(
                    "button", name="벤치", exact=True
                ).click()
                page.get_by_role("button", name="알림 닫기").click()
                page.screenshot(
                    path=str(tmp_path / "live-operations.png"),
                    full_page=False,
                    animations="disabled",
                )
                page.set_viewport_size({"width": 390, "height": 844})
                page.screenshot(
                    path=str(tmp_path / "live-mobile.png"),
                    full_page=True,
                    animations="disabled",
                )
                assert not page.evaluate(
                    "document.documentElement.scrollWidth > innerWidth"
                )
                page.close()
                end = time.monotonic() + 5
                while (
                    observed["generator"].get_parameter("enabled").value
                    and time.monotonic() < end
                ):
                    time.sleep(0.05)
                assert not observed["generator"].get_parameter("enabled").value
                assert not errors
                print(f"Browser screenshots: {tmp_path}")
            finally:
                browser.close()
    finally:
        server.should_exit = True
        task.result(timeout=12)
        sock.close()
