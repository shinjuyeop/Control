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
import threading
from types import SimpleNamespace

# Must precede the PyQt5 import inside kaiev26_hil.hil_gui.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

from kaiev26_hil import hil_gui
from kaiev26_hil.models import HilDesiredState
from kaiev26_hil.protocol import parse_status


if hil_gui.GUI_IMPORT_ERROR is None:
    from PyQt5 import QtTest

    QtCore = hil_gui.QtCore
else:
    QtCore = None
    QtTest = None

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
    return hil_gui.decode_status_payload(
        parse_status(line, received_monotonic=0.0).to_json()
    )


def test_json_boundaries_reject_incomplete_bridge_payloads():
    status = json.loads(parse_status(PROTOCOL_2_STATUS).to_json())
    del status["keya_can"]
    with pytest.raises(ValueError, match="missing fields"):
        hil_gui.decode_status_payload(json.dumps(status))

    desired = json.loads(HilDesiredState().to_json())
    del desired["keya_drop_tx"]
    with pytest.raises(ValueError, match="missing fields"):
        hil_gui.decode_desired_payload(json.dumps(desired))


def test_status_json_boundary_rejects_false_encoded_as_text():
    status = json.loads(parse_status(PROTOCOL_2_STATUS).to_json())
    status["keya_can"] = "false"

    with pytest.raises(ValueError, match="keya_can must be boolean"):
        hil_gui.decode_status_payload(json.dumps(status))


def test_status_json_boundary_ignores_unconsumed_extra_fields():
    status = json.loads(parse_status(PROTOCOL_2_STATUS).to_json())
    status["future_diagnostic"] = {"detail": "not consumed by this GUI"}

    assert hil_gui.decode_status_payload(json.dumps(status))["keya_can"] is True


def test_invalid_gear_payload_preserves_prior_status_snapshot():
    prior = status_dict(PROTOCOL_2_STATUS)
    receiver = SimpleNamespace(_lock=threading.Lock(), status=prior)
    invalid = json.loads(parse_status(PROTOCOL_2_STATUS).to_json())
    invalid["gear"] = "BROKEN"

    hil_gui.HilGuiRosNode._status(
        receiver, SimpleNamespace(data=json.dumps(invalid))
    )

    assert receiver.status is prior


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
    if hil_gui.GUI_IMPORT_ERROR is not None:
        pytest.skip(f"PyQt5 unavailable: {hil_gui.GUI_IMPORT_ERROR}")
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
    # Undo Qt's mnemonic escape so this asserts what the operator reads.
    titles = [
        widget.tabs.tabText(i).replace("&&", "&") for i in range(widget.tabs.count())
    ]
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


def test_steer_override_is_locked_out_while_the_keya_emulator_follows(window):
    """SET STEER_POS lands and is then overwritten by the emulator's next tick.

    Firmware gates its follower on ``keya_can_enabled && motor_enabled``; while
    that holds, a manual shaft position survives at most 10 ms. Leaving the
    control live made an ignored command look like an applied one.
    """
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)  # keya_can=1 keya_enabled=1

    widget._refresh()

    assert not widget.steer_override.isEnabled()
    assert not widget.apply_steer.isEnabled()
    assert widget.steer_note.text() == widget.STEER_FOLLOWING_NOTE

    # Disabling the motor hands the shaft back to the operator.
    node.status = status_dict(PROTOCOL_2_STATUS.replace("keya_enabled=1", "keya_enabled=0"))
    widget._refresh()

    assert widget.steer_override.isEnabled()
    assert widget.apply_steer.isEnabled()
    assert widget.steer_note.text() == widget.STEER_IDLE_NOTE

    # Turning CAN2 off stops the follower too, even with the motor enabled.
    node.status = status_dict(PROTOCOL_2_STATUS.replace("keya_can=1", "keya_can=0"))
    widget._refresh()

    assert widget.apply_steer.isEnabled()


def test_steer_override_stays_locked_out_while_disconnected(window):
    widget, node = window
    node.connected = False
    node.status = status_dict(PROTOCOL_2_STATUS.replace("keya_enabled=1", "keya_enabled=0"))

    widget._refresh()

    assert not widget.apply_steer.isEnabled()


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
    widget.steer_override.setValue(12.5)

    # Mode and gear are two-way choices: the button press is the command.
    widget.mode_row["buttons"]["AUTO"].click()
    widget.gear_row["buttons"]["REVERSE"].click()
    for button in widget.findChildren(hil_gui.QtWidgets.QPushButton):
        if button.text() in ("Apply", "Set feedback position"):
            button.click()

    assert "SET MODE AUTO" in node.sent
    assert "SET GEAR REVERSE" in node.sent
    assert "SET STEER_POS 12.5" in node.sent


def test_pedal_drag_commits_once_on_release(window):
    """One serial round trip per interaction, not one per intermediate value."""
    widget, node = window

    widget.pedal_slider.setSliderDown(True)
    for value in (10, 20, 30):
        widget.pedal_slider.setValue(value)
    assert node.sent == [], "mid-drag values must not reach the serial link"
    assert widget.pedal.value() == pytest.approx(30.0), "the spin box still tracks"

    widget.pedal_slider.setSliderDown(False)

    assert node.sent == ["SET PEDAL 30"]


def test_pedal_wheel_or_key_commits_immediately(window):
    """Nothing presses the handle, so there is no release to wait for."""
    widget, node = window

    widget.pedal_slider.setValue(40)

    assert node.sent == ["SET PEDAL 40"]


def test_typing_a_pedal_value_commits_only_when_editing_finishes(window):
    """Typing 51 passes through 5; committing per keystroke would send both."""
    widget, node = window

    widget.pedal.setValue(5.0)
    widget.pedal.setValue(51.0)
    assert node.sent == []
    assert widget.pedal_slider.value() == 51, "the slider still tracks"

    widget.pedal.editingFinished.emit()

    assert node.sent == ["SET PEDAL 51"]


def test_pedal_slider_jumps_to_a_click_instead_of_paging(window):
    widget, _ = window
    slider = widget.pedal_slider
    slider.resize(200, 20)
    slider.setValue(0)

    QtTest.QTest.mouseClick(
        slider,
        QtCore.Qt.LeftButton,
        QtCore.Qt.NoModifier,
        QtCore.QPoint(slider.width() - 10, slider.height() // 2),
    )

    # A paging slider would have advanced by one page step from 0.
    assert slider.value() > slider.pageStep(), (
        f"click landed on {slider.value()}, which is only a page step"
    )


def test_motion_source_switch_rescales_the_override_box(window):
    widget, _ = window

    widget.motion_source.setCurrentText("RPM")
    assert widget.motion_override.maximum() == pytest.approx(32000.0)
    assert widget.motion_override.suffix() == " rpm"

    widget.motion_source.setCurrentText("SPEED")
    assert widget.motion_override.maximum() == pytest.approx(200.0)
    assert widget.motion_override.suffix() == " km/h"


def quick_injection(widget, label):
    """Click a Quick injection preset by its button label."""
    for button in widget.findChildren(hil_gui.QtWidgets.QPushButton):
        if button.text() == label:
            button.click()
            return
    raise AssertionError(f"no quick injection button labelled {label!r}")


def selected(row):
    """The option whose button is currently lit in a choice row."""
    return next(
        (name for name, button in row["buttons"].items() if button.isChecked()),
        None,
    )


def test_quick_injection_moves_the_controls_it_commands(window):
    """A preset that changed only the hardware left the panel showing a lie."""
    widget, node = window
    widget.mode_row["buttons"]["MANUAL"].click()
    widget.gear_row["buttons"]["REVERSE"].click()
    widget.pedal.setValue(51.0)
    node.sent.clear()

    quick_injection(widget, "AUTO + Forward")
    quick_injection(widget, "Pedal 30%")

    assert selected(widget.mode_row) == "AUTO"
    assert selected(widget.gear_row) == "FORWARD"
    assert widget.pedal.value() == pytest.approx(30.0)
    # The slider mirrors the spin box, so the whole row agrees.
    assert widget.pedal_slider.value() == 30
    assert node.sent == ["SET MODE AUTO", "SET GEAR FORWARD", "SET PEDAL 30"]


def test_choice_rows_follow_status_without_echoing_a_command(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)  # mode=AUTO gear=FWD

    widget._refresh()

    assert selected(widget.mode_row) == "AUTO"
    assert selected(widget.gear_row) == "FORWARD"
    assert node.sent == []

    node.status = status_dict(
        PROTOCOL_2_STATUS.replace("mode=AUTO", "mode=MANUAL").replace("gear=FWD", "gear=REV")
    )
    widget._refresh()

    assert selected(widget.mode_row) == "MANUAL"
    assert selected(widget.gear_row) == "REVERSE"
    assert node.sent == []


def test_chosen_mode_survives_refreshes_until_status_echoes_it(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS.replace("mode=AUTO", "mode=MANUAL"))
    widget._refresh()

    widget.mode_row["buttons"]["AUTO"].click()
    assert node.sent == ["SET MODE AUTO"]

    for _ in range(5):  # stale STATUS, as during the serial round trip
        widget._refresh()
    assert selected(widget.mode_row) == "AUTO"

    node.status = status_dict(PROTOCOL_2_STATUS)  # mode=AUTO
    widget._refresh()

    assert selected(widget.mode_row) == "AUTO"
    assert widget.mode_row["pending"] is None
    assert node.sent == ["SET MODE AUTO"]


def test_pressing_the_lit_choice_resends_it(window):
    """Retrying a command that did not take is the obvious operator reflex."""
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)  # mode=AUTO
    widget._refresh()
    assert selected(widget.mode_row) == "AUTO"

    widget.mode_row["buttons"]["AUTO"].click()

    assert node.sent == ["SET MODE AUTO"]
    # An exclusive group must not leave the row with nothing selected.
    assert selected(widget.mode_row) == "AUTO"


def test_unconfirmed_mode_falls_back_to_the_reported_one(window):
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS.replace("mode=AUTO", "mode=MANUAL"))
    widget._refresh()

    widget.mode_row["buttons"]["AUTO"].click()
    wanted, _ = widget.mode_row["pending"]
    widget.mode_row["pending"] = (wanted, 0.0)

    widget._refresh()

    assert selected(widget.mode_row) == "MANUAL"
    assert widget.mode_row["pending"] is None


def test_refresh_never_moves_the_pedal_under_the_operator(window):
    """The pedal commits itself, so STATUS must not fight an adjustment."""
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)  # pedal=25.0
    widget.pedal.setValue(70.0)
    node.sent.clear()

    for _ in range(5):
        widget._refresh()

    assert widget.pedal.value() == pytest.approx(70.0)
    assert node.sent == []


def test_clicked_estop_survives_refreshes_until_status_echoes_it(window):
    """A press must not be undone by the refresh that runs before the echo.

    The bridge round trip is slower than the refresh timer, so syncing the
    button from STATUS unconditionally snapped every press back to OFF for a
    tick and read on the bench as "the button takes a second to respond".
    """
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)
    widget._refresh()

    widget.estop_buttons["DASH"].click()
    assert node.sent == ["SET ESTOP DASH ON"]

    # Several refreshes worth of stale STATUS, as during the serial round trip.
    for _ in range(5):
        widget._refresh()
    assert widget.estop_buttons["DASH"].isChecked()
    assert "…" in widget.estop_buttons["DASH"].text()

    node.status = status_dict(PROTOCOL_2_STATUS.replace("estop_dash=0", "estop_dash=1"))
    widget._refresh()

    assert widget.estop_buttons["DASH"].isChecked()
    assert widget.estop_buttons["DASH"].text() == "DASH: ON"
    # Confirmation must not re-send the command.
    assert node.sent == ["SET ESTOP DASH ON"]


def test_unconfirmed_estop_falls_back_to_reported_state_after_timeout(window):
    """Holding the click forever would hide a command STM_TEST never applied."""
    widget, node = window
    node.connected = True
    node.status = status_dict(PROTOCOL_2_STATUS)
    widget._refresh()

    widget.estop_buttons["DASH"].click()
    button = widget.estop_buttons["DASH"]
    wanted, _ = widget._pending_toggles[button]
    widget._pending_toggles[button] = (wanted, 0.0)

    widget._refresh()

    assert not button.isChecked()
    assert button.text() == "DASH: OFF"
    assert button not in widget._pending_toggles
