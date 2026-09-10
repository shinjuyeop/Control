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
    RuntimeStats,
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
    observed = {"commands": [], "planning": [], "safe": 0, "publish": True, "ack": True}
    observed["drive"] = DriveActuatorStatus(
        part1_valid=True,
        speed_mps=1.2,
        part1_age_ms=12.0,
        part2_valid=False,
        part2_age_ms=750.0,
        motor_temperature_c=99.0,
        pedal_percent=42,
        pedal_adc_raw=1585,
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
            runtime_pubs = {
                board_id: peer.create_publisher(RuntimeStats, topic, 10)
                for board_id, topic in ((1, "/stm_a/runtime_stats"), (2, "/stm_b/runtime_stats"))
            }

            def publish_runtime():
                if observed.get("runtime"):
                    for message in observed["runtime"]():
                        runtime_pubs[message.board_id].publish(message)

            peer.create_timer(1.0, publish_runtime)
            status = replace(
                make_status(protocol=3),
                pedal=27.0,
                voltage=72.4,
                rpm=400.0,
                ez_rx=10,
                ez_rx_age_ms=12,
                keya_rx_age_ms=4294967295,
            )

            observed["hil"] = status

            def publish():
                if not observed["publish"]:
                    return
                status_pub.publish(String(data=observed["hil"].to_json()))
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
                if observed["ack"]:
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
            observed["result_pub"] = result_pub
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


def runtime_messages():
    from rosidl_runtime_py.set_message import set_message_fields
    from runtime_samples import runtime_data

    uptime_ms = int(time.monotonic()) * 1000
    messages = []
    for board_id in (1, 2):
        message = RuntimeStats()
        set_message_fields(message, runtime_data(board_id, uptime_ms // 1000, uptime_ms))
        messages.append(message)
    return messages


def test_runtime_ros_subscription_and_websocket_freshness(bench):
    client, observed = bench
    observed["runtime"] = runtime_messages
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        state = receive(ws, lambda m: m.get("runtime", {}).get("STM_B", {}).get("status") == "ok")
        assert state["topics"]["/stm_a/runtime_stats"]["data"]["board_id"] == 1
        assert state["topics"]["/stm_b/runtime_stats"]["data"]["board_id"] == 2
        assert state["runtime"]["STM_B"]["data"]["estop"]["max_us"] == 6
        state = receive(ws, lambda m: m.get("runtime", {}).get("STM_A", {}).get("data", {}).get("worst_stack", {}).get("rate_hz") == 50)
        assert state["runtime"]["STM_A"]["data"]["worst_stack"]["stack"]["used_bytes"] == 460
        observed["runtime"] = None
        state = receive(ws, lambda m: m.get("topics", {}).get("/stm_a/runtime_stats", {}).get("age_ms", 0) > 1500)
        assert state["topics"]["/stm_a/runtime_stats"]["fresh"]
        assert state["runtime"]["STM_A"]["status"] == "ok"
        state = receive(ws, lambda m: m.get("runtime", {}).get("STM_A", {}).get("status") == "stale")
        assert not state["topics"]["/stm_a/runtime_stats"]["fresh"]
        assert not observed["commands"] and not observed["planning"]


def test_explicit_bench_reset_acknowledges_runtime_history_for_all_clients(bench):
    client, observed = bench
    misses = [30]

    def runtime():
        messages = runtime_messages()
        messages[1].tasks[7].deadline_misses = misses[0]
        return messages

    observed["runtime"] = runtime
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        receive(ws, lambda s: s.get("runtime", {}).get("STM_B", {}).get("status") == "history")
        assert not request(ws, "safe")["ok"]  # Existing operator ownership applies.
        assert request(ws, "claim")["ok"]
        assert request(ws, "refresh")["ok"]
        state = fresh(ws)
        assert state["runtime"]["STM_B"]["status"] == "history"
        assert request(ws, "safe")["ok"]
        state = fresh(ws)
        assert state["runtime"]["STM_B"]["status"] == "ok"
        assert state["runtime"]["STM_B"]["data"]["deadline_misses_since_reset"] == "0"
        assert state["topics"]["/stm_b/runtime_stats"]["data"]["tasks"][7]["deadline_misses"] == 30
        with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as viewer:
            state = fresh(viewer)
            assert state["runtime"]["STM_B"]["data"]["deadline_misses"] == "30"
            assert state["runtime"]["STM_B"]["data"]["deadline_misses_since_reset"] == "0"
        misses[0] = 31
        state = receive(ws, lambda s: s.get("runtime", {}).get("STM_B", {}).get("data", {}).get("deadline_misses_since_reset") == "1")
        assert state["runtime"]["STM_B"]["status"] == "check"
        assert request(ws, "release")["ok"]
        state = fresh(ws)
        assert state["runtime"]["STM_B"]["data"]["deadline_misses_since_reset"] == "1"
        assert state["runtime"]["STM_B"]["status"] in ("check", "history")


@pytest.mark.parametrize("tailnet_host", ["100.112.72.27", "100.112.72.26"])
def test_web_allows_hil_tailnet_host_only(bench, tailnet_host):
    client, _ = bench
    for host in ("localhost", "127.0.0.1", tailnet_host):
        assert client.get("/api/health", headers={"host": host}).status_code == 200
    for host in ("untrusted.example",):
        assert client.get("/api/health", headers={"host": host}).status_code == 400
    with client.websocket_connect(
        f"ws://{tailnet_host}:8765/ws",
        headers={"origin": f"http://{tailnet_host}:8765"},
    ) as ws:
        assert fresh(ws)["connected"]
    with pytest.raises(WebSocketDisconnect):
        with client.websocket_connect(
            f"ws://{tailnet_host}:8765/ws",
            headers={"origin": "http://untrusted.example"},
        ):
            pass


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
        assert state["profile"] == "hil"
        assert state["topics"]["/hil/status"]["data"]["pedal"] == 27
        assert state["desired"]["pedal"] == 65  # An intent is never an observation.
        assert not request(first, "command", command="SET PEDAL 30")["ok"]
        assert request(first, "claim")["ok"]
        assert not request(first, "set_profile", profile="vehicle")["ok"]
        assert fresh(first)["profile"] == "hil"
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


def test_takeover_disarms_and_revokes_former_browser_without_resetting_inputs(bench):
    client, observed = bench
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as first:
        fresh(first)
        assert request(first, "claim")["ok"]
        assert request(first, "production", values=dict(enabled=True, speed_target_mps=0.5,
            steering_target_rad=0.0, brake_engage=False))["ok"]
        with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as second:
            state = fresh(second)
            revision = state["control"]["revision"]
            assert not request(second, "takeover", expected_revision=revision)["ok"]
            assert request(second, "takeover", confirmed=True, expected_revision=revision)["ok"]
            assert fresh(second)["control"]["mine"]
            assert not observed["generator"].get_parameter("enabled").value
            assert observed["safe"] == 0 and not observed["commands"]
            assert not request(first, "command", command="SET PEDAL 30")["ok"]
            first.send_json({"action": "heartbeat"})
            assert request(first, "release")["ok"]
            assert fresh(second)["control"]["mine"]
            assert observed["safe"] == 0
            assert not request(first, "takeover", confirmed=True, expected_revision=revision)["ok"]


def test_takeover_cancels_scenario_but_does_not_run_bench_reset(bench):
    client, observed = bench
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as first:
        fresh(first)
        assert request(first, "claim")["ok"]
        assert request(first, "scenario", name="smoke_test", arm=True)["ok"]
        with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as second:
            revision = fresh(second)["control"]["revision"]
            assert request(second, "takeover", confirmed=True, expected_revision=revision)["ok"]
            state = fresh(second)
            assert state["scenario"]["result"] == "CANCELLED"
            assert not observed["generator"].get_parameter("enabled").value
            assert observed["safe"] == 0


def test_takeover_serializes_contenders_and_fences_queued_old_actions(bench):
    import asyncio

    client, observed = bench

    async def contend():
        console = client.app.state.console
        await console.action("former", {"action": "claim"})
        revision = console.control_revision
        original = console.node.production
        entered, unblock = asyncio.Event(), asyncio.Event()
        tasks = []

        async def delayed(values):
            entered.set()
            await unblock.wait()
            return await original(values)

        console.node.production = delayed
        await console.lock.acquire()
        lock_held = True
        try:
            transfer = asyncio.create_task(console.action("winner", dict(action="takeover", confirmed=True, expected_revision=revision)))
            tasks.append(transfer)
            await asyncio.sleep(0)
            old_command = asyncio.create_task(console.action("former", dict(action="command", command="SET PEDAL 30")))
            tasks.append(old_command)
            old_release = asyncio.create_task(console.release("former", "Late browser close"))
            tasks.append(old_release)
            loser = asyncio.create_task(console.action("loser", dict(action="takeover", confirmed=True, expected_revision=revision)))
            tasks.append(loser)
            await asyncio.sleep(0)
            console.lock.release()
            lock_held = False
            await asyncio.wait_for(entered.wait(), 2)
            assert console.owner == "former"
            assert console.snapshot("winner")["control"]["transitioning"]
            assert not console.snapshot("winner")["control"]["mine"]
            unblock.set()
            results = await asyncio.gather(*tasks, return_exceptions=True)
            assert isinstance(results[0], str)
            assert isinstance(results[1], ValueError)
            assert results[2] is None
            assert isinstance(results[3], ValueError)
            assert console.owner == "winner"
            await console.release("former", "Late watchdog", force=True, expected_revision=revision)
            assert console.owner == "winner"
        finally:
            if lock_held: console.lock.release()
            unblock.set()
            for task in tasks:
                if not task.done(): task.cancel()
            await asyncio.gather(*tasks, return_exceptions=True)
            console.node.production = original

    client.portal.call(contend)
    assert observed["safe"] == 0 and not observed["commands"]


def test_takeover_refuses_unconfirmed_disarm_and_pending_scenario_requests(bench):
    from types import SimpleNamespace
    from rclpy.task import Future

    client, _ = bench

    async def failures():
        console = client.app.state.console
        await console.action("former", {"action": "claim"})
        revision = console.control_revision
        pending = Future()
        console.runner = SimpleNamespace(parameter_requests=[pending])
        try:
            with pytest.raises(ValueError):
                await console.action("next", dict(action="takeover", confirmed=True, expected_revision=revision))
            assert console.owner == "former"
        finally:
            console.runner = None
            pending.cancel()
        original = console.node.production

        async def fail_disarm(values):
            raise ValueError("Generator service rejected disarm")

        console.node.production = fail_disarm
        try:
            with pytest.raises(ValueError):
                await console.action("next", dict(action="takeover", confirmed=True, expected_revision=revision))
            assert console.owner == "former" and not console.releasing
            assert not console.snapshot("next")["control"]["mine"]
        finally:
            console.node.production = original

    client.portal.call(failures)


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


@pytest.fixture
def browser_page(bench, tmp_path):
    from browser_support import browser_session

    client, observed = bench
    with browser_session(client) as page:
        page.get_by_text("Web bridge 연결됨", exact=True).wait_for()
        yield page, client, observed, tmp_path


def test_runtime_page_readonly_measurements_and_responsive(browser_page):
    from playwright.sync_api import expect

    page, client, observed, tmp_path = browser_page
    observed["runtime"] = runtime_messages
    page.get_by_role("button", name="STM_A RUNTIME").click()
    expect(page.get_by_role("heading", name="ECU 상태", exact=True)).to_be_visible()
    a = page.locator(".runtime-summary").filter(has=page.get_by_role("heading", name="STM_A", exact=True))
    b = page.locator(".runtime-summary").filter(has=page.get_by_role("heading", name="STM_B", exact=True))
    expect(a).to_contain_text("RUNTIME OK")
    expect(a).to_contain_text("12.7 / 64.0 KiB")
    expect(a).to_contain_text("460 / 1024 B")
    expect(a).to_contain_text("564 B margin")
    expect(a).to_contain_text("미지원")
    expect(b.locator(".runtime-latency")).to_contain_text("6 µs")
    expect(b.locator(".runtime-latency")).to_contain_text("3 samples")
    tasks = page.get_by_role("table", name="STM_A task runtime")
    expect(tasks.get_by_role("row").filter(has=page.get_by_role("rowheader", name="BTx", exact=True))).to_contain_text("50.0 Hz")
    expect(tasks.get_by_role("row").filter(has=page.get_by_role("rowheader", name="uROS", exact=True))).to_contain_text("미계측")
    for width in (1920, 1440, 1280, 1024, 390):
        page.set_viewport_size({"width": width, "height": 1000 if width == 1920 else 844})
        expect(page.locator(".profile-label")).to_be_visible()
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
        if width == 1920:
            assert abs(a.bounding_box()["y"] - b.bounding_box()["y"]) < 1
        if width in (1280, 1024, 390):
            assert b.bounding_box()["y"] > a.bounding_box()["y"]
        if width in (1920, 1024, 390):
            page.screenshot(path=str(tmp_path / f"runtime-{width}.png"), full_page=True)
    page.get_by_role("button", name="Control Dock 열기", exact=True).click()
    expect(page.locator("#control-dock")).to_be_visible()
    page.keyboard.press("Escape")
    expect(page.locator("#control-dock")).not_to_be_visible()
    page.get_by_role("button", name="메뉴 열기", exact=True).click()
    page.get_by_role("navigation").get_by_role("button", name="벤치", exact=True).click()
    page.get_by_role("button", name="STM_B RUNTIME").click()
    expect(page.get_by_role("heading", name="ECU 상태", exact=True)).to_be_visible()

    def malformed():
        messages = runtime_messages()
        messages[0].tasks[4].stack_free_bytes = 1025
        messages[1].tasks[7].present = False
        messages[1].estop_valid = False
        messages[1].estop_dispatch_samples = 0
        return messages

    observed["runtime"] = malformed
    expect(a).to_contain_text("INVALID")
    expect(a).to_contain_text("stack free 1025 B > allocated 1024 B")
    expect(page.get_by_role("table", name="STM_B task runtime").get_by_role("rowheader", name="Display", exact=True)).to_have_count(0)
    expect(b.locator(".runtime-latency")).to_contain_text("미계측")
    observed["runtime"] = None
    expect(a).to_contain_text("STALE", timeout=5000)
    expect(a.get_by_role("group", name="FREERTOS HEAP")).to_contain_text("—%")
    assert not observed["commands"] and not observed["planning"] and observed["safe"] == 0
    print(f"Runtime page images: {tmp_path}")


def test_bench_reset_runtime_history_display(browser_page):
    from playwright.sync_api import expect
    import re

    page, _, observed, _ = browser_page
    misses = [30]

    def runtime():
        messages = runtime_messages()
        messages[1].tasks[7].deadline_misses = misses[0]
        return messages

    observed["runtime"] = runtime
    page.get_by_role("button", name="STM_B RUNTIME").click()
    board = page.locator(".runtime-summary").nth(1)
    expect(board).to_contain_text("MISS HISTORY")
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    page.get_by_role("button", name="Bench Reset", exact=True).click()
    expect(board).to_contain_text("RUNTIME OK")
    expect(board).to_contain_text("Deadline misses · 리셋 이후")
    count = board.locator(".fact").filter(has=page.get_by_text("Deadline misses · 리셋 이후", exact=True))
    expect(count.locator("strong")).to_have_text(re.compile(r"^0\D"))
    expect(count).to_contain_text("부팅 누적 30")
    task = page.get_by_role("table", name="STM_B task runtime").get_by_role("row").filter(has=page.get_by_role("rowheader", name="Display", exact=True))
    expect(task).to_contain_text("부팅 누적 30")
    misses[0] = 31
    expect(board).to_contain_text("CHECK")
    expect(count.locator("strong")).to_have_text(re.compile(r"^1\D"))
    expect(count).to_contain_text("부팅 누적 31")


def test_subsystem_instruments_render_observed_faults_and_offline(browser_page):
    from playwright.sync_api import expect

    page, _, observed, tmp_path = browser_page
    nav = page.get_by_role("navigation")
    nav.get_by_role("button", name="구동", exact=True).click()
    expect(page.get_by_role("group", name="주행 속도 비교", exact=True)).to_contain_text("4.32")
    nav.get_by_role("button", name="조향", exact=True).click()
    expect(page.get_by_role("group", name="조향각 목표와 피드백 계기", exact=True)).to_contain_text("30.00°")
    observed["hil"] = replace(observed["hil"], estop_dash=True, ez_fault=True, keya_can=False)
    nav.get_by_role("button", name="브레이크 · 안전", exact=True).click()
    expect(page.get_by_role("group", name="E-stop 입력 관측", exact=True)).to_contain_text("ACTIVE")
    # A test input being active must not fabricate a received STM_B safety report.
    expect(page.get_by_role("group", name="STM_B safety 판단", exact=True)).to_contain_text("UNKNOWN")
    page.screenshot(path=str(tmp_path / "safety-observed-input.png"), full_page=False)
    nav.get_by_role("button", name="고장 주입", exact=True).click()
    expect(page.get_by_role("group", name="현재 고장 주입 상태", exact=True)).to_contain_text("EZ FAULT")
    expect(page.get_by_role("group", name="현재 고장 주입 상태", exact=True)).to_contain_text("KEYA CAN")
    page.screenshot(path=str(tmp_path / "faults-observed.png"), full_page=False)
    observed["publish"] = False
    expect(page.locator(".fault-overview")).to_contain_text("주입 상태 미확인", timeout=4000)
    page.context.set_offline(True)
    expect(page.locator(".connection-notice strong")).to_have_text("Web 연결 대기", timeout=7000)
    expect(page.get_by_role("button", name="조작권 가져오기", exact=True)).to_be_disabled()
    assert not observed["commands"] and not observed["planning"] and observed["safe"] == 0


def test_browser_takeover_requires_confirmation_and_locks_former_operator(browser_page):
    from playwright.sync_api import expect

    first, _, observed, _ = browser_page
    first.get_by_role("button", name="조작권 가져오기", exact=True).click()
    expect(first.get_by_role("button", name="AUTO", exact=True)).to_be_enabled()
    context = first.context.browser.new_context(viewport={"width": 390, "height": 844})
    try:
        second = context.new_page()
        second.goto(first.url)
        second.get_by_role("button", name="조작권 강제 인계", exact=True).click()
        dialog = second.get_by_role("dialog", name="조작권을 강제로 인계합니다", exact=True)
        expect(dialog).to_be_visible()
        dialog.get_by_role("button", name="취소", exact=True).click()
        expect(first.get_by_role("button", name="AUTO", exact=True)).to_be_enabled()
        assert observed["safe"] == 0
        second.get_by_role("button", name="조작권 강제 인계", exact=True).click()
        dialog.get_by_role("button", name="Host 명령 중단 후 인계", exact=True).click()
        expect(second.get_by_role("button", name="조작권 해제", exact=True)).to_be_visible()
        expect(first.get_by_role("button", name="AUTO", exact=True)).to_be_disabled()
        expect(second.get_by_label("주행 명령 상태")).to_contain_text("DISARMED")
        first.close()
        second.wait_for_timeout(250)
        expect(second.get_by_role("button", name="조작권 해제", exact=True)).to_be_visible()
        assert observed["safe"] == 0 and not observed["commands"]
    finally:
        context.close()


def test_operator_remote_across_pages_and_navigation(browser_page):
    from playwright.sync_api import expect
    import re
    import base64
    from kaiev26_hil.dashboard_display import (
        NativeDashboardRenderer,
        decode_display_report,
    )
    from test_dashboard_display import LIBRARY, wire_report

    page, client, observed, tmp_path = browser_page
    # A previous generator target must not prefill a new speed draft.
    observed["generator"].set_parameters([Parameter("speed_target_mps", value=3.0)])
    page.reload()
    page.get_by_text("Web bridge 연결됨", exact=True).wait_for()
    page.wait_for_timeout(500)
    expect(page.get_by_label("목표 속도", exact=True)).to_have_value("0")
    remote = page.get_by_role("region", name="Operator Input", exact=True)
    expect(remote).to_be_visible()
    expect(remote.get_by_role("button", name="AUTO", exact=True)).to_be_disabled()
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_disabled()
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    expect(remote.get_by_role("button", name="AUTO", exact=True)).to_be_enabled()

    page.get_by_role("navigation").get_by_role(
        "button", name="구동", exact=True
    ).click()
    combined = page.get_by_role("img", name=re.compile(r"속도 목표"))
    expect(combined).to_be_visible()
    expect(combined.get_by_text("km/h", exact=True)).to_be_visible()
    expect(combined.get_by_text("RPM", exact=True)).to_be_visible()
    # The Host command form lives in the shared dock, once, on every page.
    expect(page.get_by_label("목표 속도", exact=True)).to_have_count(1)
    expect(
        page.get_by_role("complementary").get_by_label("목표 속도", exact=True)
    ).to_be_attached()
    remote.get_by_role("button", name="AUTO", exact=True).click()
    page.get_by_role("status").get_by_text("OK SET MODE AUTO", exact=True).wait_for()
    assert "SET MODE AUTO" in observed["commands"]
    # The peer still reports MANUAL: ACK must not paint AUTO as observed.
    expect(remote.get_by_role("button", name="MANUAL", exact=True)).to_have_attribute(
        "aria-pressed", "true"
    )

    slider = remote.get_by_label("가속 페달", exact=True)
    box = slider.bounding_box()
    before = len([c for c in observed["commands"] if c.startswith("SET PEDAL ")])
    page.mouse.move(box["x"] + box["width"] * 0.27, box["y"] + box["height"] / 2)
    page.mouse.down()
    page.mouse.move(
        box["x"] + box["width"] * 0.65, box["y"] + box["height"] / 2, steps=8
    )
    page.wait_for_timeout(250)
    assert (
        len([c for c in observed["commands"] if c.startswith("SET PEDAL ")]) == before
    )
    page.mouse.up()
    page.get_by_role("status").get_by_text(re.compile(r"^OK SET PEDAL ")).wait_for()
    assert (
        len([c for c in observed["commands"] if c.startswith("SET PEDAL ")])
        == before + 1
    )
    pedal_observation = page.get_by_role("group", name="페달 관측", exact=True)
    expect(pedal_observation).to_contain_text("42%")
    expect(pedal_observation).to_contain_text("1.28 V")

    # Empty numeric drafts survive telemetry updates until blur, then become zero.
    for label in ["목표 속도", "목표 조향각"]:
        field = page.get_by_label(label, exact=True)
        field.fill("")
        expect(field).to_have_value("")
        page.wait_for_timeout(250)
        expect(field).to_have_value("")
        field.press("Tab")
        expect(field).to_have_value("0")
        field.fill("")
        field.press_sequentially("-2.5")
        expect(field).to_have_value("-2.5")
        field.press("Tab")
        expect(field).to_have_value("-2.5")
    assert not observed["planning"]
    # Command drafts are shared across pages; no navigation publishes them.
    page.get_by_label("목표 속도", exact=True).fill("36")
    page.get_by_label("목표 조향각", exact=True).fill("30")
    page.get_by_role("navigation").get_by_role(
        "button", name="조향", exact=True
    ).click()
    expect(page.get_by_label("목표 속도", exact=True)).to_have_value("36")
    expect(page.get_by_label("목표 조향각", exact=True)).to_have_value("30")
    remote.get_by_role("button", name="REV", exact=True).click()
    page.get_by_role("status").get_by_text("OK SET GEAR REVERSE", exact=True).wait_for()
    assert "SET GEAR REVERSE" in observed["commands"]
    expect(remote.get_by_role("button", name="FWD", exact=True)).to_have_attribute(
        "aria-pressed", "true"
    )
    angle = page.locator(".fact").filter(has=page.get_by_text("목표 각도", exact=True))
    expect(angle).to_contain_text("30.00°")
    assert not observed["planning"]
    page.get_by_role("button", name="ARM · 명령 발행", exact=True).click()
    page.get_by_role("dialog").get_by_role(
        "button", name="ARM · 발행", exact=True
    ).click()
    expect(
        page.get_by_label("주행 명령 상태").get_by_text("ARMED", exact=True)
    ).to_be_visible()
    assert observed["generator"].get_parameter(
        "speed_target_mps"
    ).value == pytest.approx(10)
    assert observed["generator"].get_parameter(
        "steering_target_rad"
    ).value == pytest.approx(math.pi / 6)

    page.get_by_role("navigation").get_by_role(
        "button", name="브레이크 · 안전", exact=True
    ).click()
    remote.get_by_role("switch", name="대시보드 E-stop", exact=True).click()
    page.get_by_role("status").get_by_text(
        "OK SET ESTOP DASH ON", exact=True
    ).wait_for()
    expect(
        remote.get_by_role("switch", name="대시보드 E-stop", exact=True)
    ).to_have_attribute("aria-checked", "false")
    assert "SET ESTOP DASH ON" in observed["commands"]
    page.get_by_role("navigation").get_by_role(
        "button", name="구동", exact=True
    ).click()
    part2 = page.get_by_role("group", name="EZkontrol Part II", exact=True)
    expect(part2).to_contain_text("750")
    expect(part2).to_contain_text("INVALID")

    # Exercise every observation page with the same active Remote and global ARM badge.
    for name in [
        "벤치",
        "조향",
        "브레이크 · 안전",
        "고장 주입",
        "시나리오",
        "가상 LCD",
        "진단 · 로그",
        "ECU 상태",
    ]:
        page.get_by_role("navigation").get_by_role(
            "button", name=name, exact=True
        ).click()
        expect(page.get_by_role("heading", name=name, exact=True)).to_be_visible()
        expect(remote).to_be_visible()
        expect(
            page.get_by_label("주행 명령 상태").get_by_text("ARMED", exact=True)
        ).to_be_visible()
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")

    # Display UI consumes an UNO-shaped report and PNG from the real C++ renderer.
    report = decode_display_report(wire_report())
    png = NativeDashboardRenderer(LIBRARY).render(report)

    async def display_sample():
        display = client.app.state.display
        with display.lock:
            display.port = "/dev/test-display"
            display.serial_open = True
            display.report = report
            display.image = "data:image/png;base64," + base64.b64encode(png).decode()
            display.received_at = time.monotonic()

    client.portal.call(display_sample)
    page.get_by_role("navigation").get_by_role(
        "button", name="가상 LCD", exact=True
    ).click()
    page.get_by_text("UNO LIVE", exact=True).wait_for()
    image = page.get_by_alt_text("UNO가 적용한 상태로 재구성한 가상 LCD")
    expect(image).to_be_visible()
    assert image.evaluate(
        "(img)=>img.naturalWidth===480 && img.naturalHeight===320 && img.complete"
    )
    page.wait_for_timeout(1150)
    expect(
        page.get_by_text("마지막 수신 화면 · 현재 상태 미확인", exact=True)
    ).to_be_visible()

    for width in [1440, 1366, 1280]:
        page.set_viewport_size(
            {"width": width, "height": 900 if width == 1440 else 768}
        )
        page.get_by_role("navigation").get_by_role(
            "button", name="구동", exact=True
        ).click()
        expect(remote).to_be_visible()
        expect(
            remote.get_by_role("switch", name="무선 리모컨 E-stop", exact=True)
        ).to_be_in_viewport()
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
    page.screenshot(path=str(tmp_path / "drive-desktop.png"), full_page=False)
    for width in [1024, 390]:
        page.set_viewport_size({"width": width, "height": 844})
        expect(remote).not_to_be_visible()
        page.get_by_role("button", name="Control Dock 열기", exact=True).click()
        expect(remote).to_be_visible()
        expect(
            remote.get_by_role("switch", name="차체 측면 E-stop", exact=True)
        ).to_be_enabled()
        page.screenshot(path=str(tmp_path / f"remote-{width}.png"), full_page=False)
        if width == 390:
            page.get_by_label("목표 속도", exact=True).fill("54")
            page.get_by_label("목표 조향각", exact=True).fill("-10")
            page.get_by_role("switch", name="브레이크 요청", exact=True).click()
            expect(
                page.get_by_role("dialog", name="주행 명령을 발행합니다", exact=True)
            ).not_to_be_visible()
            page.get_by_role("button", name="목표 변경", exact=True).click()
            confirmation = page.get_by_role(
                "dialog", name="주행 명령을 발행합니다", exact=True
            )
            expect(confirmation).to_be_visible()
            expect(confirmation).to_contain_text("54 km/h")
            page.keyboard.press("Tab")
            assert confirmation.evaluate("d=>d.contains(document.activeElement)")
            page.keyboard.press("Escape")
            expect(confirmation).not_to_be_visible()
            expect(remote).to_be_visible()
            assert observed["generator"].get_parameter(
                "speed_target_mps"
            ).value == pytest.approx(10)
            page.get_by_role("button", name="목표 변경", exact=True).click()
            confirmation.get_by_role("button", name="ARM · 발행", exact=True).click()
            expect(confirmation).not_to_be_visible()
            # The same command conversion and release path work inside the drawer.
            end = time.monotonic() + 3
            while (
                observed["generator"].get_parameter("speed_target_mps").value != 15
                and time.monotonic() < end
            ):
                time.sleep(0.05)
            assert observed["generator"].get_parameter(
                "speed_target_mps"
            ).value == pytest.approx(15)
            assert observed["generator"].get_parameter(
                "steering_target_rad"
            ).value == pytest.approx(-math.pi / 18)
            page.get_by_role("button", name="발행 해제", exact=True).click()
            expect(page.get_by_label("주행 명령 상태")).to_contain_text("DISARMED")
        page.keyboard.press("Escape")
        expect(remote).not_to_be_visible()
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
    page.close()
    end = time.monotonic() + 5
    while (
        observed["generator"].get_parameter("enabled").value and time.monotonic() < end
    ):
        time.sleep(0.05)
    assert not observed["generator"].get_parameter("enabled").value
    print(f"Browser review images: {tmp_path}")


def test_remote_busy_scenario_and_stale_locks(browser_page):
    from playwright.sync_api import expect

    page, client, observed, _ = browser_page
    remote = page.get_by_role("region", name="Operator Input", exact=True)
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    expect(remote.get_by_role("button", name="AUTO", exact=True)).to_be_enabled()
    observed["ack"] = False
    remote.get_by_role("button", name="AUTO", exact=True).click()
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_disabled()
    expect(
        remote.get_by_role("switch", name="대시보드 E-stop", exact=True)
    ).to_be_disabled()
    observed["ack"] = True
    observed["result_pub"].publish(String(data="OK SET MODE AUTO"))
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_enabled()

    page.get_by_role("navigation").get_by_role(
        "button", name="시나리오", exact=True
    ).click()
    page.get_by_role("checkbox").check()
    page.get_by_role("button", name="시나리오 실행", exact=True).click()
    page.get_by_role("dialog").get_by_role(
        "button", name="ARM · 시험 시작", exact=True
    ).click()
    expect(page.get_by_label("시나리오 상태")).to_contain_text("RUNNING")
    expect(remote.get_by_role("button", name="AUTO", exact=True)).to_be_disabled()
    expect(
        remote.get_by_role("switch", name="무선 리모컨 E-stop", exact=True)
    ).to_be_disabled()
    page.get_by_role("navigation").get_by_role(
        "button", name="조향", exact=True
    ).click()
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_disabled()
    page.get_by_role("navigation").get_by_role(
        "button", name="시나리오", exact=True
    ).click()
    page.get_by_role("button", name="중단 · Bench Reset", exact=True).click()
    expect(page.get_by_label("시나리오 상태")).to_contain_text("IDLE")
    expect(page.get_by_label("주행 명령 상태")).to_contain_text("DISARMED")
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_enabled()
    assert client.get("/api/history").json()[0]["result"] == "CANCELLED"
    observed["publish"] = False
    expect(remote.get_by_role("button", name="AUTO", exact=True)).to_be_disabled(
        timeout=3000
    )
    expect(remote.get_by_label("가속 페달", exact=True)).to_be_disabled()
    expect(
        remote.get_by_role("switch", name="대시보드 E-stop", exact=True)
    ).to_be_disabled()
    expect(remote).to_contain_text("STM_TEST 상태 미수신 / 지연")
    page.get_by_role("navigation").get_by_role("button", name="벤치", exact=True).click()
    pedal_observation = page.get_by_role("group", name="페달 관측", exact=True)
    expect(pedal_observation).to_contain_text("—%")
    expect(pedal_observation).to_contain_text("— V")


def test_control_dock_collapse_and_global_estop(browser_page):
    from playwright.sync_api import expect

    page, client, observed, _ = browser_page
    dock = page.locator("#control-dock")
    estop = page.get_by_role("status", name="E-stop 입력 상태", exact=True)
    expect(dock).to_be_visible()
    expect(estop).to_have_text("E-STOP CLEAR")
    expect(page.get_by_label("주행 명령 상태")).to_contain_text("DISARMED")
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    speed = page.get_by_label("목표 속도", exact=True)
    speed.fill("12.5")
    page.get_by_label("목표 조향각", exact=True).fill("-3")
    for width in [1440, 1366, 1280]:
        page.set_viewport_size({"width": width, "height": 768})
        before_width = page.locator("main").bounding_box()["width"]
        page.get_by_role("button", name="Control Dock 접기", exact=True).click()
        expect(dock).not_to_be_visible()
        assert page.locator("main").bounding_box()["width"] > before_width
        page.get_by_role("navigation").get_by_role("button", name="구동", exact=True).click()
        expect(dock).not_to_be_visible()
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
        page.get_by_role("button", name="Control Dock 펼치기", exact=True).click()
        expect(dock).to_be_visible()
        expect(speed).to_have_value("12.5")
        expect(page.get_by_label("목표 조향각", exact=True)).to_have_value("-3")
        expect(dock.get_by_role("button", name="MANUAL", exact=True)).to_have_attribute("aria-pressed", "true")
        expect(page.get_by_label("가속 페달", exact=True)).to_have_value("27")
        assert not page.evaluate("document.documentElement.scrollWidth > innerWidth")
    assert not observed["commands"] and not observed["planning"]
    slider = page.get_by_label("가속 페달", exact=True)
    slider.dispatch_event("pointerdown")
    slider.fill("70")
    page.get_by_role("button", name="Control Dock 접기", exact=True).evaluate("b=>b.click()")
    page.get_by_role("button", name="Control Dock 펼치기", exact=True).click()
    expect(slider).to_have_value("70")
    slider.dispatch_event("pointercancel")
    expect(slider).to_have_value("27")
    assert not observed["commands"]
    page.get_by_role("button", name="Control Dock 접기", exact=True).click()
    for field in ["estop_dash", "estop_side", "estop_rf"]:
        observed["hil"] = replace(observed["hil"], **{field: True})
        expect(estop).to_have_text("E-STOP ACTIVE")
        observed["hil"] = replace(observed["hil"], **{field: False})
        expect(estop).to_have_text("E-STOP CLEAR")
    # Desktop preference is preserved through the tablet drawer breakpoint.
    page.set_viewport_size({"width": 1024, "height": 844})
    page.get_by_role("button", name="Control Dock 열기", exact=True).click()
    expect(dock).to_be_visible()
    close = page.get_by_role("button", name="Control Dock 닫기", exact=True)
    close.focus()
    page.keyboard.press("Shift+Tab")
    assert dock.evaluate("d=>d.contains(document.activeElement)")
    page.keyboard.press("Tab")
    expect(close).to_be_focused()
    page.get_by_role("button", name="패널 닫기", exact=True).click(position={"x": 10, "y": 100})
    expect(dock).not_to_be_visible()
    page.set_viewport_size({"width": 1366, "height": 768})
    expect(dock).not_to_be_visible()
    page.get_by_role("button", name="Control Dock 펼치기", exact=True).click()
    expect(speed).to_have_value("12.5")
    observed["publish"] = False
    expect(estop).to_have_text("E-STOP UNKNOWN", timeout=4000)
    # With no generator service responses, command state must age to UNKNOWN.
    executor = client.app.state.console.executor
    executor.remove_node(observed["generator"])
    try:
        expect(page.get_by_label("주행 명령 상태")).to_contain_text("UNKNOWN", timeout=6000)
    finally:
        executor.add_node(observed["generator"])
    observed["publish"] = True
    expect(page.get_by_label("주행 명령 상태")).to_contain_text("DISARMED")


@pytest.mark.parametrize("lock", ["stale", "busy"])
def test_pedal_edit_recovers_when_disabled_during_drag(browser_page, lock):
    from playwright.sync_api import expect

    page, client, observed, _ = browser_page
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    slider = page.get_by_label("가속 페달", exact=True)
    expect(slider).to_be_enabled()
    box = slider.bounding_box()
    page.mouse.move(box["x"] + box["width"] * .27, box["y"] + box["height"] / 2)
    page.mouse.down()
    page.mouse.move(box["x"] + box["width"] * .7, box["y"] + box["height"] / 2, steps=5)
    assert not [c for c in observed["commands"] if c.startswith("SET PEDAL")]
    if lock == "stale":
        observed["publish"] = False
    else:
        observed["ack"] = False
        page.get_by_role("button", name="AUTO", exact=True).evaluate("b=>b.click()")
    expect(slider).to_be_disabled(timeout=4000)
    page.mouse.up()
    observed["hil"] = replace(observed["hil"], pedal=41.0)
    observed["publish"] = True
    if lock == "busy":
        observed["ack"] = True
        observed["result_pub"].publish(String(data="OK SET MODE AUTO"))
    expect(slider).to_be_enabled()
    expect(slider).to_have_value("41")
    assert not [c for c in observed["commands"] if c.startswith("SET PEDAL")]
    # Cancel also clears the draft without submitting it.
    slider.dispatch_event("pointerdown")
    slider.fill("70")
    slider.dispatch_event("pointercancel")
    expect(slider).to_have_value("41")
    assert not [c for c in observed["commands"] if c.startswith("SET PEDAL")]
    slider.press("ArrowRight")
    page.get_by_text("OK SET PEDAL 42", exact=True).wait_for()
    assert [c for c in observed["commands"] if c.startswith("SET PEDAL")] == ["SET PEDAL 42"]


@pytest.fixture
def uno_display(bench):
    """Existing Web bench with a real serial worker; PTY supplies explicit reports.

    This peer never interprets touch or predicts pages. Tests choose the next
    report to distinguish queued input from firmware-applied observations.
    """
    import asyncio
    import os
    import pty
    import threading
    from kaiev26_hil.dashboard_display import DashboardDisplay
    from test_dashboard_display import LIBRARY, wire_report

    if not LIBRARY.is_file():
        pytest.skip("Build the native dashboard renderer first")
    client, _ = bench
    master, slave = pty.openpty()
    os.set_blocking(master, False)
    display = DashboardDisplay(os.ttyname(slave), LIBRARY)
    original = client.app.state.display
    client.app.state.display = client.app.state.console.display = display
    state = {"page": "MANUAL_TFT", "publish": True, "lines": []}
    stopping = threading.Event()

    async def peer():
        buffer = b""
        while not stopping.is_set():
            if state["publish"]:
                os.write(master, wire_report(vtouch=1, touch_errors=0, page=state["page"]))
            try:
                buffer += os.read(master, 4096)
            except BlockingIOError:
                pass
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if line.startswith(b"VT "):
                    state["lines"].append(line.decode())
            await asyncio.sleep(0.05)

    display.start()
    task = client.portal.start_task_soon(peer)
    try:
        end = time.monotonic() + 4
        while not display.snapshot()["touch_ready"] and time.monotonic() < end:
            time.sleep(0.01)
        assert display.snapshot()["touch_ready"]
        state["lines"].clear()
        yield display, state
    finally:
        display.stop()
        stopping.set()
        task.result(timeout=3)
        os.close(master)
        os.close(slave)
        client.app.state.display = client.app.state.console.display = original
    assert not display.thread.is_alive()


def wait_touch_line(state, line):
    end = time.monotonic() + 3
    while line not in state["lines"] and time.monotonic() < end:
        time.sleep(0.01)
    assert line in state["lines"]


def test_virtual_touch_lease_takeover_and_disconnect_cancel(bench, uno_display):
    client, observed = bench
    display, uno = uno_display
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as first:
        assert fresh(first)
        args = dict(phase="D", x=100, y=160, generation=display.generation, revision=0)
        assert not request(first, "virtual_touch", **args)["ok"]
        assert request(first, "claim")["ok"]
        args["revision"] = fresh(first)["control"]["revision"]
        assert request(first, "virtual_touch", **args)["ok"]
        wait_touch_line(uno, "VT D 100 160")
        assert not observed["commands"] and not client.app.state.console.armed
        with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as second:
            state = fresh(second)
            assert not request(second, "virtual_touch", **args)["ok"]
            assert request(second, "takeover", confirmed=True,
                           expected_revision=state["control"]["revision"])["ok"]
            wait_touch_line(uno, "VT C")
            assert not request(first, "virtual_touch", **dict(args, phase="U", x=300))["ok"]
            assert not request(second, "virtual_touch", **args)["ok"] # Old revision
            args["revision"] = fresh(second)["control"]["revision"]
            assert request(second, "virtual_touch", **args)["ok"]
            wait_touch_line(uno, "VT D 100 160")
            uno["lines"].clear()
        wait_touch_line(uno, "VT C") # Browser disconnect
    assert display.snapshot()["report"]["page"] == "MANUAL_TFT"


def test_virtual_touch_heartbeat_expiry_and_unavailable_uno(bench, uno_display):
    client, _ = bench
    display, uno = uno_display
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as ws:
        fresh(ws)
        assert request(ws, "claim")["ok"]
        args = dict(phase="D", x=100, y=160, generation=display.generation,
                    revision=fresh(ws)["control"]["revision"])
        assert request(ws, "virtual_touch", **args)["ok"]
        wait_touch_line(uno, "VT D 100 160")
        client.app.state.console.heartbeat_at -= 4
        receive(ws, lambda m: m.get("type") == "state" and not m["control"]["occupied"])
        wait_touch_line(uno, "VT C")
        assert not request(ws, "virtual_touch", **dict(args, phase="U", x=300))["ok"]
        assert request(ws, "claim")["ok"]
        args["revision"] = fresh(ws)["control"]["revision"]
        uno["publish"] = False
        time.sleep(1.1)
        assert not request(ws, "virtual_touch", **args)["ok"]
        assert display.snapshot()["report"]["page"] == "MANUAL_TFT"


def test_virtual_lcd_pointer_mapping_cancel_and_report_only_mirror(browser_page, uno_display):
    from playwright.sync_api import expect
    page, client, observed, _ = browser_page
    display, uno = uno_display
    page.get_by_role("navigation").get_by_role("button", name="가상 LCD", exact=True).click()
    expect(page.locator(".virtual-display-container").get_by_text("VIEW ONLY", exact=True)).to_be_visible()
    lcd = page.get_by_role("img", name="UNO가 적용한 상태로 재구성한 가상 LCD")
    box = lcd.bounding_box()
    page.mouse.move(box["x"] + 50, box["y"] + 50)
    page.mouse.down()
    page.mouse.up()
    assert not any(line.startswith("VT D") for line in uno["lines"])
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    expect(page.get_by_text("VTOUCH READY", exact=True)).to_be_visible()
    original_image = lcd.get_attribute("src")
    # Fractional pointer positions avoid rounding ambiguity while proving all
    # supported CSS scales use the same 480 x 320 content coordinate system.
    for width, scale in [(1800, 1), (2400, 2), (1000, 2)]:
        page.set_viewport_size({"width": width, "height": 1000})
        page.get_by_role("button", name="LCD 원본 크기" if scale == 1 else "LCD 두 배 확대").click()
        box = lcd.bounding_box()
        uno["lines"].clear()
        page.mouse.move(box["x"] + box["width"] * 100.25 / 480,
                        box["y"] + box["height"] * 160.25 / 320)
        page.mouse.down()
        page.mouse.move(box["x"] + box["width"] * 300.25 / 480,
                        box["y"] + box["height"] * 160.25 / 320, steps=40)
        page.mouse.up()
        wait_touch_line(uno, "VT D 100 160")
        wait_touch_line(uno, "VT U 300 160")
        assert lcd.get_attribute("src") == original_image
        assert display.snapshot()["report"]["page"] == "MANUAL_TFT"
    uno["page"] = "SWIPE_L" # Only the next UNO report is allowed to change the image.
    expect(lcd).not_to_have_attribute("src", original_image)
    expect(page.locator(".lcd-caption")).to_contain_text("SWIPE_L")
    uno["lines"].clear()
    box = lcd.bounding_box()
    page.mouse.move(box["x"] + box["width"] * 100.25 / 480,
                    box["y"] + box["height"] * 160.25 / 320)
    page.mouse.down()
    wait_touch_line(uno, "VT D 100 160")
    lcd.dispatch_event("pointercancel", {"pointerId": 1})
    page.mouse.up()
    wait_touch_line(uno, "VT C")
    assert not any(line.startswith("VT U") for line in uno["lines"])
    # A burst of PointerEvents is coalesced; pointer capture preserves release
    # outside the LCD, with the final coordinate clamped to the screen edge.
    uno["lines"].clear()
    page.mouse.down()
    wait_touch_line(uno, "VT D 100 160")
    lcd.evaluate("""img => {
      const r = img.getBoundingClientRect();
      for (let i = 0; i < 500; i++) img.dispatchEvent(new PointerEvent('pointermove', {
        bubbles: true, pointerId: 1, isPrimary: true,
        clientX: r.left + r.width * 300.25 / 480,
        clientY: r.top + r.height * 160.25 / 320,
      }));
    }""")
    page.wait_for_timeout(60)
    wait_touch_line(uno, "VT M 300 160")
    assert sum(line.startswith("VT M") for line in uno["lines"]) <= 3
    page.mouse.move(box["x"] + box["width"] + 30,
                    box["y"] + box["height"] * 160.25 / 320)
    page.mouse.up()
    wait_touch_line(uno, "VT U 479 160")
    # A second finger cancels instead of mixing the two pointer sequences.
    uno["lines"].clear()
    page.mouse.move(box["x"] + box["width"] * 100.25 / 480,
                    box["y"] + box["height"] * 160.25 / 320)
    page.mouse.down()
    wait_touch_line(uno, "VT D 100 160")
    lcd.dispatch_event("pointerdown", {"pointerId": 2, "isPrimary": False, "button": 0})
    page.mouse.up()
    wait_touch_line(uno, "VT C")
    assert not any(line.startswith("VT U") for line in uno["lines"])
    assert not observed["commands"]


def test_virtual_lcd_ownership_loss_cancel_and_no_false_success(browser_page, uno_display):
    from playwright.sync_api import expect
    page, client, _, _ = browser_page
    display, uno = uno_display
    page.get_by_role("navigation").get_by_role("button", name="가상 LCD", exact=True).click()
    page.get_by_role("button", name="조작권 가져오기", exact=True).click()
    expect(page.get_by_text("VTOUCH READY", exact=True)).to_be_visible()
    lcd = page.get_by_role("img", name="UNO가 적용한 상태로 재구성한 가상 LCD")
    box = lcd.bounding_box()
    page.mouse.move(box["x"] + 100, box["y"] + 100)
    page.mouse.down()
    page.mouse.move(box["x"] + 250, box["y"] + 100)
    with client.websocket_connect("ws://localhost/ws", headers={"origin": "http://localhost"}) as second:
        state = fresh(second)
        assert request(second, "takeover", confirmed=True,
                       expected_revision=state["control"]["revision"])["ok"]
        expect(page.locator(".virtual-display-container").get_by_text("VIEW ONLY", exact=True)).to_be_visible()
        wait_touch_line(uno, "VT C")
        page.mouse.up()
        assert not any(line.startswith("VT U") for line in uno["lines"])
        uno["publish"] = False
        expect(page.get_by_text("UNO STALE", exact=True)).to_be_visible()
        page.mouse.down()
        page.mouse.up()
        assert display.snapshot()["report"]["page"] == "MANUAL_TFT"
