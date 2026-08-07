"""Headless smoke test for the HIL operator window.

The GUI has no ROS or serial dependency of its own: it renders whatever
``node.snapshot()`` returns and pushes commands back through ``node.send``. That
makes it testable with an offscreen Qt platform and a stub node, which is what
this covers -- every panel is built, and a full STATUS is rendered without a
missing-attribute or missing-key failure.

Status dictionaries come from ``parse_status`` on the STATUS lines documented in
``firmware/STM_TEST/README.md``, so a firmware field that the GUI stops reading
correctly fails here rather than on the bench.
"""

import json
import os

# Must precede the PyQt5 import inside kaiev26_hil.hil_gui.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from kaiev26_hil import hil_gui
from kaiev26_hil.models import HilStatus
from kaiev26_hil.protocol import parse_status


pytestmark = pytest.mark.skipif(
    hil_gui.GUI_IMPORT_ERROR is not None,
    reason=f"PyQt5 unavailable: {hil_gui.GUI_IMPORT_ERROR}",
)

PROTOCOL_2_STATUS = (
    "STATUS protocol=2 mode=AUTO gear=FWD pedal=25.0 estop_dash=0 estop_side=0 "
    "estop_rf=0 speed=5.20 rpm=487 voltage=52.3 current=12.5 motor_temp=40.0 "
    "brake_period_us=20000 brake_us=1388 brake=RELEASED pwm_lost=0 steer=1.50 "
    "steer_target=2.00 ez_rx=10 ez_tx=20 keya_rx=20 keya_tx=21 ez_can=1 "
    "keya_can=1 ez_ok=1 keya_ok=1 ez_fault=0 keya_fault=0 plant=1 "
    "ez_target_rpm=1200 ez_running=1 ez_speed_control=1 ez_command_rpm=1200 "
    "ez_command_current=23.5 ez_rx_age_ms=12 keya_enabled=1 keya_rx_age_ms=15"
)

PROTOCOL_1_STATUS = (
    "STATUS mode=MANUAL gear=FWD pedal=0.0 estop_dash=0 estop_side=0 estop_rf=0 "
    "speed=0.00 rpm=0 voltage=0.0 current=0.0 motor_temp=0.0 "
    "brake_period_us=0 brake_us=0 brake=UNKNOWN pwm_lost=1 steer=0.00 "
    "steer_target=0.00 ez_rx=4 ez_tx=0 keya_rx=0 keya_tx=0 ez_can=1 keya_can=1 "
    "ez_ok=1 keya_ok=1 ez_fault=0 keya_fault=0"
)


def status_dict(line):
    """Render a STATUS line the way the bridge publishes it on /hil/status."""
    return json.loads(parse_status(line, received_monotonic=0.0).to_json())


class StubNode:
    """Stands in for HilGuiRosNode: records commands, replays a fixed snapshot."""

    def __init__(self):
        self.connected = False
        self.status = {}
        self.desired = {}
        self.sent = []
        self.production = []
        self.safe_state_calls = 0
        self.refresh_calls = 0

    def snapshot(self):
        return (self.connected, dict(self.status), dict(self.desired), "/dev/hil_test", "")

    def send(self, command):
        self.sent.append(command)

    def safe_state(self):
        self.safe_state_calls += 1

    def refresh(self):
        self.refresh_calls += 1

    def production_command(self, enabled, speed, steering, brake):
        self.production.append((enabled, speed, steering, brake))


@pytest.fixture(scope="module")
def qt_app():
    app = hil_gui.QtWidgets.QApplication.instance()
    if app is None:
        app = hil_gui.QtWidgets.QApplication([])
    yield app


@pytest.fixture
def window(qt_app):
    node = StubNode()
    widget = hil_gui.HilWindow(node)
    # Nothing here runs an event loop; _refresh is driven directly.
    widget.timer.stop()
    yield widget, node
    widget.close()
    widget.deleteLater()


def test_window_builds_every_panel(window):
    widget, _ = window
    titles = [widget.tabs.tabText(i) for i in range(widget.tabs.count())]
    assert titles == ["Signal Flow", "End-to-End Command Source", "Faults & Interfaces"]
    assert set(widget.estop_buttons) == {"DASH", "SIDE", "RF"}
    assert set(widget.feedback_numeric) == {"VOLTAGE", "CURRENT", "MOTOR_TEMP"}
    assert widget.observed, "observation panel registered no fields"


def test_refresh_without_status_leaves_controls_disabled(window):
    widget, _ = window

    widget._refresh()

    assert widget.connection.text() == "DISCONNECTED"
    assert all(not group.isEnabled() for group in widget.control_groups)


def test_refresh_renders_every_observed_field_from_protocol_2_status(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)

    widget._refresh()

    # A key the GUI reads but firmware stopped sending would show up as "--".
    blank = [key for key, label in widget.observed.items() if label.text() == "--"]
    assert blank == []
    assert widget.observed["ez_running"].text() == "RUN"
    assert widget.observed["ez_control_mode"].text() == "SPEED"
    assert widget.observed["ez_command_rpm"].text() == "1200 rpm"
    assert widget.observed["ez_target_rpm"].text() == "1200 rpm"
    assert widget.observed["keya_enabled"].text() == "ENABLED"
    assert widget.actual_rpm.text() == "487 rpm"
    assert widget.actual_speed.text() == "5.20 km/h"
    assert "ACTIVE" in widget.ez_link.text()
    assert "ACTIVE" in widget.keya_link.text()


def test_plant_status_selects_closed_loop_and_locks_direct_override(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)

    widget._refresh()

    assert widget.response_mode.currentIndex() == 0
    assert not widget.motion_override.isEnabled()
    assert not widget.apply_motion.isEnabled()
    # Selecting the mode from status must not echo a command back to the bridge.
    assert node.sent == []

    node.status = status_dict(PROTOCOL_2_STATUS.replace("plant=1", "plant=0"))
    widget._refresh()

    assert widget.response_mode.currentIndex() == 1
    assert widget.motion_override.isEnabled()


def test_protocol_1_status_reports_ages_as_unavailable(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_1_STATUS)

    widget._refresh()

    assert widget.observed["ez_rx_age_ms"].text() == "never / unavailable"
    assert "RX SEEN" in widget.ez_link.text()
    assert "NO RX" in widget.keya_link.text()


def test_response_mode_change_sends_plant_command(window):
    widget, node = window

    widget._response_mode_changed(0)
    widget._response_mode_changed(1)

    assert node.sent == ["SET PLANT ON", "SET PLANT OFF"]


def test_injection_controls_send_the_documented_commands(window):
    widget, node = window
    widget.mode.setCurrentText("AUTO")
    widget.pedal.setValue(30.0)
    widget.steer_override.setValue(12.5)

    for button in widget.findChildren(hil_gui.QtWidgets.QPushButton):
        if button.text() in ("Apply", "Set feedback position"):
            button.click()

    assert "SET MODE AUTO" in node.sent
    assert "SET PEDAL 30" in node.sent
    assert "SET STEER_POS 12.5" in node.sent


def test_motion_source_switch_rescales_the_override_box(window):
    widget, _ = window

    widget.motion_source.setCurrentText("RPM")
    assert widget.motion_override.maximum() == pytest.approx(32000.0)
    assert widget.motion_override.suffix() == " rpm"

    widget.motion_source.setCurrentText("SPEED")
    assert widget.motion_override.maximum() == pytest.approx(200.0)
    assert widget.motion_override.suffix() == " km/h"


def test_status_dataclass_and_gui_agree_on_field_names():
    """The GUI reads status keys by name; keep them tied to the dataclass."""
    status = status_dict(PROTOCOL_2_STATUS)
    assert set(status) == set(HilStatus.__dataclass_fields__)
