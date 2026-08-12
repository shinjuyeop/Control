"""Headless checks for the single-page dashboard.

The window renders whatever ``cache.snapshot()`` returns and publishes nothing,
so an offscreen Qt platform and a stub node cover it. What is worth locking
down here is not the pixel layout but the two rules the layout depends on:

* the Active conditions strip repeats reason fields and names their source --
  it must never invent a diagnosis the message did not state, and
* every flag chip knows which way is nominal, including the ones this hardware
  holds false by contract, which must never be coloured as faults.
"""

import os

# Must precede the PyQt5 import inside visualization.vehicle_dashboard.
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from kaiev26_msgs.msg import (
    ActuatorCommand,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)
import pytest

from visualization import vehicle_dashboard as vd
from visualization.ros_state_cache import (
    BRAKE,
    COMMAND,
    DRIVE,
    NA,
    NO_DATA,
    RosStateCache,
    STEERING,
)


pytestmark = pytest.mark.skipif(
    vd.GUI_IMPORT_ERROR is not None,
    reason=f'PyQt5 unavailable: {vd.GUI_IMPORT_ERROR}',
)


class StubNode:
    """Stands in for VehicleStatusSubscriber: the window only reads these."""

    def __init__(self):
        self.stale_ms = {
            COMMAND: 300.0, DRIVE: 300.0, BRAKE: 300.0, STEERING: 500.0,
        }
        self.ui_update_hz = 4.0


def healthy_command():
    message = ActuatorCommand()
    message.speed_target_mps = 1.5
    message.steering_target_rad = 0.2
    return message


def healthy_drive():
    message = DriveActuatorStatus()
    message.mode = DriveActuatorStatus.MODE_AUTO
    message.gear = DriveActuatorStatus.GEAR_FORWARD
    message.speed_mps = 1.42
    message.command_received = True
    message.tractive_commanded = True
    message.tractive_feedback = True
    message.tractive_feedback_valid = True
    message.part1_valid = True
    message.part2_valid = True
    message.fault_bits_valid = True
    return message


def healthy_brake():
    message = BrakeActuatorStatus()
    message.actual_valid = False  # permanently unwired on this hardware
    return message


def healthy_steering():
    message = SteeringActuatorStatus()
    message.state = SteeringActuatorStatus.STATE_ACTIVE
    message.disable_reason = SteeringActuatorStatus.REASON_NONE
    message.communication_ok = True
    message.feedback_valid = True
    message.mode_received = True
    message.steer_command_received = True
    message.enabled_commanded = True
    message.enabled_feedback_valid = False  # no trustworthy KEYA enable query
    message.fault_bits = 0x0001  # bit0 is the enable state, not a fault
    return message


@pytest.fixture(scope='module')
def qt_app():
    app = vd.QtWidgets.QApplication.instance()
    if app is None:
        app = vd.QtWidgets.QApplication([])
    yield app


@pytest.fixture
def window(qt_app):
    cache = RosStateCache()
    widget = vd.VehicleDashboard(StubNode(), cache)
    # Nothing here runs an event loop; refresh is driven directly.
    widget.timer.stop()
    widget.shutdown_timer.stop()
    yield widget, cache
    widget.close()
    widget.deleteLater()


def publish_healthy(cache):
    cache.update(COMMAND, healthy_command())
    cache.update(DRIVE, healthy_drive())
    cache.update(BRAKE, healthy_brake())
    cache.update(STEERING, healthy_steering())


def chip_style(widget, key):
    return widget.chips[key][0].styleSheet()


def is_alarm(widget, key):
    """A lit chip is filled; a nominal one is only outlined."""
    return 'background:' in chip_style(widget, key)


def test_dashboard_builds_without_plots(window):
    widget, _ = window
    assert not widget.findChildren(vd.QtWidgets.QSplitter), 'one page, no split'
    assert not widget.findChildren(vd.QtWidgets.QScrollArea), 'no scrolling'
    # Every declared flag got a chip, and every chip a polarity.
    for prefix, specs in (
        ('drive', widget.DRIVE_FLAGS), ('steering', widget.STEERING_FLAGS)
    ):
        for field, _caption, polarity in specs:
            assert f'{prefix}.{field}' in widget.chips
            assert polarity in (vd.BAD_TRUE, vd.BAD_FALSE, vd.WARN_TRUE, vd.INFO)


def test_flag_fields_exist_on_the_messages(window):
    """A renamed message field must fail here, not silently read as False."""
    widget, _ = window
    for specs, message in (
        (widget.DRIVE_FLAGS, DriveActuatorStatus()),
        (widget.STEERING_FLAGS, SteeringActuatorStatus()),
    ):
        for field, _caption, _polarity in specs:
            assert hasattr(message, field), field


def test_no_data_names_every_missing_topic(window):
    widget, _ = window

    widget.refresh()

    text = widget.conditions.text()
    for topic in (
        '/vehicle/command', '/drive/status', '/brake/status', '/steering/status'
    ):
        assert topic in text
    assert widget.values['mode'].text() == NO_DATA


def test_healthy_state_asserts_no_conditions(window):
    widget, cache = window
    publish_healthy(cache)

    widget.refresh()

    assert widget.conditions.text() == 'none asserted'
    assert widget.values['mode'].text() == 'AUTO'
    assert widget.values['gear'].text() == 'FORWARD'
    # The headline reads km/h; the contract's m/s stays on the line under it.
    assert widget.values['speed_actual'].text() == '5.11'
    assert widget.values['speed_actual_mps'].text() == '1.42 m/s'
    assert widget.values['steering_state'].text() == 'ACTIVE'


def test_conditions_repeat_reason_fields_and_name_their_source(window):
    """The strip reports contract fields; it must not invent a verdict."""
    widget, cache = window
    publish_healthy(cache)

    brake = healthy_brake()
    brake.stop_reason_bits = BrakeActuatorStatus.STOP_REASON_ESTOP
    cache.update(BRAKE, brake)
    drive = healthy_drive()
    drive.command_timeout = True
    cache.update(DRIVE, drive)
    steering = healthy_steering()
    steering.state = SteeringActuatorStatus.STATE_ESTOP
    steering.disable_reason = SteeringActuatorStatus.REASON_ESTOP
    cache.update(STEERING, steering)

    widget.refresh()

    text = widget.conditions.text()
    assert 'ESTOP (brake.stop_reason_bits)' in text
    assert 'COMMAND_TIMEOUT (drive.command_timeout)' in text
    assert 'ESTOP (steering.state)' in text
    assert 'ESTOP (steering.disable_reason)' in text


def test_steering_enable_bit_is_not_reported_as_a_fault(window):
    """KEYA bit0 carries the enable state; treating it as a fault cries wolf."""
    widget, cache = window
    publish_healthy(cache)

    widget.refresh()
    assert 'steering.fault_bits' not in widget.conditions.text()
    assert widget.values['steer_faults'].text() == '0x0001  (bit0 enable=1)'

    steering = healthy_steering()
    steering.fault_bits = 0x4001
    cache.update(STEERING, steering)
    widget.refresh()

    assert 'FAULT 0x4000 (steering.fault_bits)' in widget.conditions.text()


def test_contractually_false_flags_never_alarm(window):
    """N/A is not a fault: colouring it red trains the operator to ignore red."""
    widget, cache = window
    publish_healthy(cache)

    widget.refresh()

    assert not is_alarm(widget, 'steering.enabled_feedback_valid')
    assert widget.values['brake_actual'].text() == 'NOT WIRED'
    assert widget.conditions.text() == 'none asserted'


def test_flag_polarity_lights_the_abnormal_side_only(window):
    widget, cache = window
    publish_healthy(cache)

    widget.refresh()
    # Nominal: bad-when-false flag is true, bad-when-true flag is false.
    assert not is_alarm(widget, 'drive.command_received')
    assert not is_alarm(widget, 'steering.can_timeout')

    drive = healthy_drive()
    drive.command_received = False
    cache.update(DRIVE, drive)
    steering = healthy_steering()
    steering.can_timeout = True
    cache.update(STEERING, steering)
    widget.refresh()

    assert is_alarm(widget, 'drive.command_received')
    assert is_alarm(widget, 'steering.can_timeout')


def test_invalid_parts_read_as_na_rather_than_zero(window):
    """Part II absent must not look like a 0 °C motor."""
    widget, cache = window
    publish_healthy(cache)
    drive = healthy_drive()
    drive.part2_valid = False
    drive.motor_temperature_c = 0.0
    cache.update(DRIVE, drive)

    widget.refresh()

    assert widget.values['motor_temp'].text() == NA
    assert widget.values['part2_age'].text() == NA
    assert is_alarm(widget, 'drive.part2_valid')


def test_link_badge_text_is_fixed_width(window):
    """The badges repaint several times a second.

    If the state word or the age digits set their own width, every badge to
    the right of a changing one slides sideways. Padding both fields is the
    only thing keeping the header row still.
    """
    widget, _cache = window
    lengths = {
        len(widget._link_text('/drive/status', NO_DATA)),
        len(widget._link_text('/drive/status', 'OK', 7.0)),
        len(widget._link_text('/drive/status', 'OK', 412.0)),
        len(widget._link_text('/drive/status', 'STALE', 1234.0)),
    }

    assert len(lengths) == 1


def test_speed_headline_is_kmh_with_mps_underneath(window):
    """Both units stay on screen: km/h to read, m/s because that is the wire."""
    widget, cache = window
    publish_healthy(cache)

    widget.refresh()

    assert widget.values['speed_target'].text() == '5.40'
    assert widget.values['speed_target_mps'].text() == '1.50 m/s'
    assert widget.values['speed_error'].text() == '+0.29'
    assert widget.values['speed_error_mps'].text() == '+0.08 m/s'
