"""Exercise the Vehicle Web/ROS boundary on an isolated, hardware-free domain."""

from contextlib import asynccontextmanager
import json
from pathlib import Path
import sys
import subprocess
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from fastapi.testclient import TestClient
from kaiev26_msgs.msg import ActuatorCommand, DriveActuatorStatus, RuntimeStats
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.set_message import set_message_fields
from starlette.websockets import WebSocketDisconnect

from runtime_samples import runtime_data
from vehicle_console.common.telemetry import VEHICLE_TOPICS
from vehicle_console.vehicle import create_app


@pytest.fixture
def vehicle(tmp_path):
    rclpy.init()
    app = create_app(tmp_path / "web", allowed_hosts=["100.112.72.27"])
    original = app.router.lifespan_context
    control = {"publish": False, "runtime": True, "diagnostics": True}

    @asynccontextmanager
    async def lifespan(application):
        async with original(application):
            node = Node("vehicle_observation_test_peer")
            publishers = {topic: node.create_publisher(msg_type, topic, qos_profile_sensor_data)
                          for msg_type, topic in VEHICLE_TOPICS}
            started = time.monotonic()

            def publish():
                if not control["publish"]:
                    return
                for msg_type, topic in VEHICLE_TOPICS:
                    if msg_type is RuntimeStats:
                        if not control["runtime"]:
                            continue
                        generation = int((time.monotonic() - started) * 10) + 1
                        msg = RuntimeStats()
                        set_message_fields(msg, runtime_data(1 if "stm_a" in topic else 2,
                                                             generation, generation * 100))
                    elif msg_type is DiagnosticArray:
                        if not control["diagnostics"]:
                            continue
                        msg = DiagnosticArray(status=[DiagnosticStatus(
                            name="drive", hardware_id="STM_A", level=DiagnosticStatus.WARN,
                            message="CAN observation", values=[KeyValue(key="part1", value="valid")],
                        )])
                    elif msg_type is ActuatorCommand:
                        msg = msg_type(speed_target_mps=1.5, steering_target_rad=0.2)
                    elif msg_type is DriveActuatorStatus:
                        msg = msg_type(speed_mps=1.42, mode=1, part1_valid=True, part2_valid=True)
                    else:
                        msg = msg_type()
                    publishers[topic].publish(msg)

            node.create_timer(0.05, publish)
            application.state.executor.add_node(node)
            control["diagnostics_pub"] = publishers["/diagnostics"]
            try:
                yield
            finally:
                application.state.executor.remove_node(node)
                node.destroy_node()

    app.router.lifespan_context = lifespan
    try:
        with TestClient(app, base_url="http://localhost") as client:
            yield client, app.state.observer, control
    finally:
        rclpy.shutdown()


def receive(ws, predicate, timeout=8):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        state = ws.receive_json()
        if predicate(state):
            return state
    raise AssertionError("Vehicle Web state did not reach the expected observation")


def test_vehicle_starts_without_hil_and_owns_only_subscriptions(vehicle):
    client, node, _ = vehicle
    assert {s.topic_name for s in node.subscriptions} >= {topic for _, topic in VEHICLE_TOPICS}
    assert {p.topic_name for p in node.publishers} <= {"/parameter_events"}
    assert not list(node.clients)
    assert not list(node.services)
    subprocess.run([sys.executable, "-c", """
import sys
import rclpy
from vehicle_console.vehicle import VehicleObserver
rclpy.init()
node = VehicleObserver()
try:
    assert not any(n == 'vehicle_console.hil' or n.startswith('vehicle_console.hil.')
                   or n.startswith('kaiev26_hil') for n in sys.modules)
finally:
    node.destroy_node()
    rclpy.shutdown()
"""], check=True, timeout=10, cwd=Path(__file__).parents[1])
    health = client.get("/api/health").json()
    assert health["profile"] == "vehicle"
    assert not {"control_bench", "virtual_touch", "scenarios"} & set(health["capabilities"])
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        state = ws.receive_json()
        assert not state["topics"]
        assert state["runtime"]["STM_A"]["status"] == "waiting"
        assert state["runtime"]["STM_B"]["status"] == "waiting"


def test_live_stale_reconnect_and_runtime_mapping(vehicle):
    client, _, control = vehicle
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        control["publish"] = True
        state = receive(ws, lambda s: len(s["topics"]) == 7)
        assert state["topics"]["/vehicle/command"]["data"]["speed_target_mps"] == 1.5
        assert state["topics"]["/drive/status"]["data"]["speed_mps"] == pytest.approx(1.42)
        for name, topic in (("STM_A", "/stm_a/runtime_stats"), ("STM_B", "/stm_b/runtime_stats")):
            assert state["runtime"][name]["topic"] == topic
            assert state["runtime"][name]["status"] == "ok"
            assert state["runtime"][name]["data"]["cpu_load_percent"] is None
            assert state["topics"][topic]["data"] == {}
        assert state["runtime"]["STM_A"]["data"]["estop"]["max_us"] is None
        assert state["runtime"]["STM_B"]["data"]["estop"]["max_us"] == 6
        assert not {"catalog", "desired", "display", "control", "scenario", "events", "generator"} & state.keys()
        json.dumps(state, allow_nan=False)
        control["publish"] = False
        state = receive(ws, lambda s: not s["topics"]["/drive/status"]["fresh"])
        assert state["topics"]["/drive/status"]["age_ms"] >= 300
        assert state["topics"]["/drive/status"]["rx_hz"] == 0
        state = receive(ws, lambda s: s["runtime"]["STM_B"]["status"] == "stale")
        assert state["runtime"]["STM_A"]["status"] == "stale"
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        assert not ws.receive_json()["topics"]["/drive/status"]["fresh"]
        control["publish"] = True
        assert receive(ws, lambda s: s["topics"]["/drive/status"]["fresh"])["profile"] == "vehicle"


def test_diagnostics_sources_have_independent_freshness(vehicle):
    client, _, control = vehicle
    control["publish"] = True
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        receive(ws, lambda s: "/diagnostics" in s["topics"])
        extra = DiagnosticArray(status=[DiagnosticStatus(name="brake", hardware_id="STM_B", level=DiagnosticStatus.ERROR, message="brake observation")])
        control["diagnostics_pub"].publish(extra)
        state = receive(ws, lambda s: len(s["topics"]["/diagnostics"]["data"]["status"]) == 2)
        rows = {row["name"]: row for row in state["topics"]["/diagnostics"]["data"]["status"]}
        assert rows["drive"]["values"][0]["value"] == "valid"
        assert rows["brake"]["level"] == 2
        state = receive(ws, lambda s: any(row["name"] == "brake" and not row["fresh"] for row in s["topics"]["/diagnostics"]["data"]["status"]))
        assert state["topics"]["/diagnostics"]["fresh"]
        assert next(row for row in state["topics"]["/diagnostics"]["data"]["status"] if row["name"] == "drive")["fresh"]


@pytest.mark.parametrize("action", ["claim", "takeover", "command", "production", "safe", "refresh", "scenario", "virtual_touch", "set_profile", "heartbeat"])
def test_all_vehicle_actions_are_rejected(vehicle, action):
    client, _, _ = vehicle
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        ws.receive_json()
        ws.send_json({"action": action, "profile": "hil", "command": "SET PEDAL 1"})
        with pytest.raises(WebSocketDisconnect) as exc:
            while True:
                ws.receive_json()
        assert exc.value.code == 1008
    assert client.get("/api/health").json()["profile"] == "vehicle"
    assert client.post("/api/" + action, json={}).status_code in (404, 405)
    assert client.get("/api/history").status_code == 404


def test_host_and_origin_validation(vehicle):
    client, _, _ = vehicle
    assert client.get("/api/health", headers={"host": "hostile.example"}).status_code == 400
    for origin in ("https://hostile.example", "", "http://localhost:9999"):
        with pytest.raises(WebSocketDisconnect):
            with client.websocket_connect("ws://localhost/ws", headers={"origin": origin}):
                pass
    with client.websocket_connect("ws://localhost/ws", headers={"host": "100.112.72.27:8765", "origin": "http://100.112.72.27:8765"}) as ws:
        assert ws.receive_json()["profile"] == "vehicle"


def test_vehicle_browser_overview_stale_reconnect_and_responsive(vehicle, tmp_path):
    from browser_support import browser_session
    from playwright.sync_api import expect

    client, node, control = vehicle
    with browser_session(client) as page:
        expect(page.get_by_role("heading", name="Overview", exact=True)).to_be_visible()
        expect(page.locator(".profile-label")).to_have_text("VEHICLE · READ ONLY")
        expect(page.locator(".vehicle-overview")).to_contain_text("NO DATA")
        expect(page.get_by_role("button", name="Bench Reset")).to_have_count(0)
        expect(page.locator("#control-dock")).to_have_count(0)
        expect(page.get_by_role("button", name="조작권 가져오기")).to_have_count(0)
        for label in ("고장 주입", "시나리오", "가상 LCD", "HIL"):
            expect(page.get_by_role("navigation").get_by_role("button", name=label, exact=True)).to_have_count(0)
        control["publish"] = True
        expect(page.locator(".vehicle-overview")).to_contain_text("7 / 7 LIVE")
        expect(page.locator(".overview-drive")).to_contain_text("5.11")
        expect(page.locator(".overview-drive")).to_contain_text("5.40")
        expect(page.locator(".vehicle-overview")).to_contain_text("RUNTIME OK")
        for width in (1920, 1280, 768, 390):
            page.set_viewport_size({"width": width, "height": 900})
            assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
            page.screenshot(path=str(tmp_path / f"vehicle-overview-{width}.png"), full_page=True)
        for name in ("Drive", "Steering", "Safety / Brake", "ECU Runtime", "Diagnostics"):
            page.get_by_role("navigation").get_by_role("button", name=name, exact=True).click()
            expect(page.get_by_role("heading", name=name, exact=True)).to_be_visible()
            assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
        expect(page.locator(".diagnostic-observations")).to_contain_text("CAN observation")
        page.get_by_role("navigation").get_by_role("button", name="ECU Runtime", exact=True).click()
        expect(page.locator(".runtime-latency")).to_contain_text("6 µs")
        expect(page.locator(".runtime-page")).to_contain_text("미지원")
        page.get_by_role("navigation").get_by_role("button", name="Overview", exact=True).click()
        control["publish"] = False
        expect(page.locator(".overview-drive .source")).to_contain_text("STALE")
        expect(page.locator(".overview-drive")).not_to_contain_text("5.11")
        page.context.set_offline(True)
        expect(page.get_by_text("WEB OFFLINE · 현재 상태 확인 불가", exact=True)).to_be_visible(timeout=7000)
        page.context.set_offline(False)
        control["publish"] = True
        expect(page.locator(".vehicle-overview")).to_contain_text("7 / 7 LIVE", timeout=8000)
        assert {p.topic_name for p in node.publishers} <= {"/parameter_events"}
        print(f"Vehicle browser images: {tmp_path}")
