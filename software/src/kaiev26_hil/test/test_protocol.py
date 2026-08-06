import pytest

from kaiev26_hil.models import HilDesiredState
from kaiev26_hil.protocol import (
    build_fault,
    build_set_estop,
    build_set_pedal,
    desired_status_mismatches,
    desired_commands,
    parse_response,
    parse_status,
    update_desired,
    validate_command,
)


STATUS = (
    "STATUS mode=AUTO gear=FWD pedal=25.0 estop_dash=0 estop_side=1 estop_rf=0 "
    "speed=5.20 rpm=487 voltage=52.3 current=12.5 motor_temp=40.0 "
    "brake_period_us=20000 brake_us=1388 brake=RELEASED pwm_lost=0 "
    "steer=1.50 steer_target=2.00 ez_rx=10 ez_tx=20 keya_rx=30 keya_tx=40 "
    "ez_can=1 keya_can=1 ez_ok=1 keya_ok=0 ez_fault=0 keya_fault=1"
)


def test_builders_match_firmware_protocol():
    assert build_set_pedal(30) == "SET PEDAL 30"
    assert build_set_estop("dash", True) == "SET ESTOP DASH ON"
    assert build_fault("ez", "drop_tx", False) == "FAULT EZ DROP_TX OFF"


def test_validation_rejects_out_of_range_and_unknown_commands():
    with pytest.raises(ValueError):
        build_set_pedal(100.1)
    with pytest.raises(ValueError):
        validate_command("SET TURBO ON")


def test_status_parser_uses_exact_firmware_keys():
    status = parse_status(STATUS, received_monotonic=123.0)
    assert status.mode == "AUTO"
    assert status.estop_side is True
    assert status.brake_us == 1388
    assert status.ez_ok is True
    assert status.keya_ok is False
    assert status.received_monotonic == 123.0


def test_response_parser_distinguishes_ok_error_and_status():
    assert parse_response("OK SET MODE AUTO").kind == "ok"
    assert parse_response("ERR invalid_command_or_range").kind == "error"
    assert parse_response(STATUS).status.keya_fault is True


def test_desired_state_tracks_last_motion_source_for_reconnect():
    desired = HilDesiredState()
    update_desired(desired, "SET SPEED 10.5")
    assert desired.motion_source == "SPEED"
    assert "SET SPEED 10.5" in desired_commands(desired)
    assert not any(command.startswith("SET RPM ") for command in desired_commands(desired))


def test_desired_status_comparison_uses_firmware_gear_and_observable_fields():
    desired = HilDesiredState()
    status = parse_status(STATUS)
    assert desired_status_mismatches(desired, status)
    desired.mode = "AUTO"
    desired.pedal = 25.0
    desired.speed = 5.2
    desired.motion_source = "SPEED"
    desired.voltage = 52.3
    desired.current = 12.5
    desired.motor_temp = 40.0
    desired.steering_position = 2.0
    desired.estop_side = True
    desired.keya_fault = True
    assert desired_status_mismatches(desired, status) == []
