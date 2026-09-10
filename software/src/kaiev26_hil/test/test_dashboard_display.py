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
from kaiev26_hil.dashboard_display import DashboardDisplay, decode_display_report

LIBRARY = (
    Path(__file__).resolve().parents[4]
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
        auto=[1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 1,
              9000, 9100, 500, 1200, 1300, 42],
    )
    report.update(overrides)
    return b"HIL_LCD " + json.dumps(report).encode() + b"\n"


def test_report_decodes_raw_applied_values_and_tolerates_extensions():
    report = decode_display_report(wire_report(extra="future field"))
    assert report["values"]["bus_voltage_decivolt"] == 523
    assert report["values"]["pedal_percent"] == 25
    assert report["auto_values"]["target_motor_count"] == 1300
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
