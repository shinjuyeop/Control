"""UNO serial report boundary and the actual native LCD rendering path."""

import base64
from io import BytesIO
import json
import os
from pathlib import Path
import pty
import select
import time

import pytest
from PIL import Image
from vehicle_console.hil.dashboard_display import DashboardDisplay, decode_display_report

LIBRARY = (
    Path(__file__).resolve().parents[5]
    / "firmware/UNO_R4_DASHBOARD/build/host/libdashboard_renderer.so"
)


def wire_report(**overrides):
    report = dict(
        v=1,
        uptime_ms=10000,
        seq=42,
        can_lost=False,
        can_age_ms=12,
        page="MANUAL_TFT",
        values=[15, 3, 25, 3, 120, 523, 125, 1200, 320, 400, 0],
    )
    report.update(overrides)
    return b"HIL_LCD " + json.dumps(report).encode() + b"\n"


def test_report_decodes_raw_applied_values_and_tolerates_extensions():
    report = decode_display_report(wire_report(extra="future field"))
    assert report["values"]["bus_voltage_decivolt"] == 523
    assert report["values"]["pedal_percent"] == 25
    assert report["can_lost"] is False
    assert decode_display_report(b"DASH v1 alive=42 complete=101\n") is None


@pytest.mark.parametrize(
    "payload",
    [
        wire_report(can_lost="false"),
        wire_report(values=[15, 3, 25]),
        wire_report(values=[15, 3, 101, 3, 120, 523, 125, 1200, 320, 400, 0]),
        wire_report(values=[15, 3, 25, 3, 120, 523, 125, 1200, 320, 400, -1]),
    ],
)
def test_report_rejects_invalid_external_values(payload):
    with pytest.raises(ValueError):
        decode_display_report(payload)


def test_serial_stream_renders_reported_state_and_distinguishes_can_loss():
    if not LIBRARY.is_file():
        pytest.skip("Build the native dashboard renderer first")
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    display = DashboardDisplay(path, LIBRARY)
    try:
        display.start()
        assert select.select([master], [], [], 3)[0]
        assert b"HIL_LCD ON" in os.read(master, 1024)
        payload = wire_report()
        os.write(master, payload[:37])
        os.write(master, payload[37:])
        end = time.monotonic() + 3
        while not display.snapshot()["fresh"] and time.monotonic() < end:
            time.sleep(0.02)
        state = display.snapshot()
        assert state["fresh"]
        assert state["report"]["values"]["pedal_percent"] == 25
        image = Image.open(BytesIO(base64.b64decode(state["image"].split(",", 1)[1])))
        assert image.size == (480, 320)
        assert image.mode == "RGB"
        os.write(master, wire_report(can_lost=True, can_age_ms=450))
        end = time.monotonic() + 2
        while not display.snapshot()["report"]["can_lost"] and time.monotonic() < end:
            time.sleep(0.02)
        state = display.snapshot()
        assert state["fresh"] and state["report"]["can_lost"]
        # Silence is a lost USB report, never an invented CAN LOST observation.
        time.sleep(1.05)
        assert not display.snapshot()["fresh"]
    finally:
        display.stop()
        os.close(master)
        os.close(slave)
    assert not display.thread.is_alive()
    assert not display.snapshot()["serial_open"]


@pytest.fixture
def touch_display(monkeypatch):
    from unittest.mock import Mock
    now = [100.0]
    monkeypatch.setattr("vehicle_console.hil.dashboard_display.time.monotonic", lambda: now[0])
    display = DashboardDisplay("test", LIBRARY)
    display.serial_open = True
    display.received_at = now[0]
    display.report = decode_display_report(wire_report(vtouch=1, touch_errors=0))
    connection = Mock()
    connection.write.side_effect = len
    return display, connection, now


def test_touch_queue_preserves_edges_coalesces_moves_and_caps_rate(touch_display):
    display, connection, now = touch_display
    display.queue_touch("D", 100, 160, generation=0)
    for x in range(101, 250):
        display.queue_touch("M", x, 160, generation=0)
    assert len(display.touch_queue) == 2
    display._write_touch(connection)
    display._write_touch(connection)
    assert [call.args[0] for call in connection.write.call_args_list] == [
        b"VT D 100 160\n", b"VT M 249 160\n"
    ]
    display.queue_touch("M", 280, 160, generation=0)
    now[0] += 0.01
    display._write_touch(connection)
    assert connection.write.call_count == 2
    display.queue_touch("U", 300, 160, generation=0)
    display._write_touch(connection)
    assert connection.write.call_args.args == (b"VT U 300 160\n",)
    assert not display.touch_queue
    assert not display.touch_active


def test_touch_cancel_overflow_timeout_and_stale_reports_discard_gesture(touch_display):
    display, connection, now = touch_display
    display.queue_touch("D", 100, 160, generation=0)
    display._write_touch(connection)
    display.cancel_touch()
    display._write_touch(connection)
    assert connection.write.call_args.args == (b"VT C\n",)
    with pytest.raises(ValueError):
        display.queue_touch("U", 300, 160, generation=0)
    for _ in range(16):
        display.queue_touch("D", 100, 160, generation=0)
        display.queue_touch("U", 300, 160, generation=0)
    with pytest.raises(ValueError):
        display.queue_touch("D", 100, 160, generation=0)
    assert list(display.touch_queue) == [("C", b"VT C\n")]
    display._write_touch(connection)
    display.queue_touch("D", 100, 160, generation=0)
    display._write_touch(connection)
    now[0] += 1.01
    display.received_at = now[0] # Report is fresh, pointer timed out.
    display._write_touch(connection)
    assert connection.write.call_args.args == (b"VT C\n",)
    display.queue_touch("D", 100, 160, generation=0)
    now[0] += 1.01
    display.touch_received_at = now[0] # Pointer fresh, UNO report stale.
    display._write_touch(connection)
    assert connection.write.call_args.args == (b"VT C\n",)
    assert not display.snapshot()["touch_ready"]


@pytest.mark.parametrize("phase,x,y", [
    ("SWIPE_LEFT", 1, 1), ("D", -1, 0), ("D", 480, 0), ("D", 0, 320),
    ("D", 0, -1), ("D", True, 1), ("D", 1.5, 1), ("D", "1", 1),
])
def test_touch_rejects_malformed_web_coordinates(touch_display, phase, x, y):
    display, _, _ = touch_display
    display.queue_touch("D", 100, 160, generation=0)
    with pytest.raises(ValueError):
        display.queue_touch(phase, x, y, generation=0)
    assert not display.touch_active
    assert list(display.touch_queue) == [("C", b"VT C\n")]


def test_touch_requires_current_connection_and_supported_fresh_report(touch_display):
    display, _, _ = touch_display
    for change in ({"serial_open": False}, {"received_at": None},
                   {"report": decode_display_report(wire_report())}):
        before = {name: getattr(display, name) for name in change}
        for name, value in change.items():
            setattr(display, name, value)
        with pytest.raises(ValueError):
            display.queue_touch("D", 0, 0, generation=0)
        for name, value in before.items():
            setattr(display, name, value)
    with pytest.raises(ValueError):
        display.queue_touch("D", 0, 0, generation=-1)


def test_serial_reconnect_discards_touch_and_keeps_reading_reports(tmp_path):
    if not LIBRARY.is_file():
        pytest.skip("Build the native dashboard renderer first")
    masters, slaves = zip(*(pty.openpty() for _ in range(2)))
    link = tmp_path / "uno"
    link.symlink_to(os.ttyname(slaves[0]))
    display = DashboardDisplay(str(link), LIBRARY)

    def wait_for(predicate):
        end = time.monotonic() + 4
        while not predicate() and time.monotonic() < end:
            time.sleep(0.01)
        assert predicate()

    def read_until(master, content):
        data = b""
        end = time.monotonic() + 4
        while content not in data and time.monotonic() < end:
            if select.select([master], [], [], 0.1)[0]:
                data += os.read(master, 4096)
        assert content in data
        return data

    try:
        display.start()
        read_until(masters[0], b"VT C\nHIL_LCD ON\n")
        os.write(masters[0], wire_report(vtouch=1))
        wait_for(lambda: display.snapshot()["touch_ready"])
        generation = display.snapshot()["generation"]
        display.queue_touch("D", 100, 160, generation=generation)
        display.queue_touch("M", 200, 160, generation=generation)
        read_until(masters[0], b"VT D 100 160\n")
        os.write(masters[0], wire_report(vtouch=1, seq=43, page="SWIPE_L"))
        wait_for(lambda: display.snapshot()["report"]["seq"] == 43)
        assert display.snapshot()["report"]["page"] == "SWIPE_L"
        link.unlink()
        link.symlink_to(os.ttyname(slaves[1]))
        os.close(masters[0])
        wait_for(lambda: display.snapshot()["generation"] > generation)
        data = read_until(masters[1], b"VT C\nHIL_LCD ON\n")
        assert b"VT D" not in data and b"VT M" not in data
        assert not display.snapshot()["fresh"]
        with pytest.raises(ValueError):
            display.queue_touch("U", 300, 160, generation=generation)
        os.write(masters[1], wire_report(vtouch=1, seq=44))
        wait_for(lambda: display.snapshot()["touch_ready"])
        with pytest.raises(ValueError):
            display.queue_touch("U", 300, 160, generation=display.generation)
        display.queue_touch("D", 100, 160, generation=display.generation)
        display.queue_touch("U", 300, 160, generation=display.generation)
        read_until(masters[1], b"VT U 300 160\n")
    finally:
        display.stop()
        for fd in (*masters, *slaves):
            try:
                os.close(fd)
            except OSError:
                pass
    assert not display.thread.is_alive()
