import pytest

from kaiev26_hil.models import HilDesiredState
from kaiev26_hil.protocol import (
    build_fault,
    build_set_estop,
    build_set_pedal,
    build_set_plant,
    desired_status_mismatches,
    desired_commands,
    parse_response,
    parse_status,
    update_desired,
    validate_command,
)


STATUS = (
    "STATUS protocol=3 mode=AUTO gear=FWD pedal=25.0 estop_dash=0 estop_side=1 estop_rf=0 "
    "speed=5.20 rpm=487 voltage=52.3 current=12.5 motor_temp=40.0 "
    "brake_period_us=20000 brake_us=1388 brake=RELEASED pwm_lost=0 "
    "steer=1.50 steer_target=2.00 ez_rx=10 ez_tx=20 keya_rx=30 keya_tx=40 "
    "ez_can=1 keya_can=1 ez_ok=1 keya_ok=0 ez_fault=0 keya_fault=1 "
    "plant=0 ez_target_rpm=0 ez_running=1 ez_speed_control=1 "
    "ez_command_rpm=1200 ez_command_current=23.5 ez_rx_age_ms=12 "
    "keya_enabled=0 keya_rx_age_ms=4294967295 "
    "ez_busoff=0 ez_errpass=0 ez_warn=0 ez_tec=0 ez_rec=0 "
    "keya_busoff=1 keya_errpass=1 keya_warn=1 keya_tec=248 keya_rec=0"
)

_PROTOCOL3_PREFIXES = (
    "ez_busoff=", "ez_errpass=", "ez_warn=", "ez_tec=", "ez_rec=",
    "keya_busoff=", "keya_errpass=", "keya_warn=", "keya_tec=", "keya_rec=",
)

# protocol 2 firmware predates the FDCAN protocol-status fields.
STATUS_V2 = " ".join(
    token for token in STATUS.replace("protocol=3", "protocol=2").split()
    if not token.startswith(_PROTOCOL3_PREFIXES)
)

LEGACY_STATUS = " ".join(
    token for token in STATUS.split()
    if not token.startswith((
        "protocol=", "plant=", "ez_target_rpm=", "ez_running=",
        "ez_speed_control=", "ez_command_rpm=", "ez_command_current=",
        "ez_rx_age_ms=", "keya_enabled=", "keya_rx_age_ms=",
    ) + _PROTOCOL3_PREFIXES)
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
    assert status.protocol == 3
    assert status.ez_command_rpm == 1200
    assert status.ez_rx_age_ms == 12
    assert status.received_monotonic == 123.0


def test_status_parser_reads_fdcan_protocol_status():
    status = parse_status(STATUS)
    # keya_ok stays True-ish per peripheral init while the bus is off, which is
    # why the judgement has to come from these fields instead.
    assert status.ez_busoff is False
    assert status.ez_tec == 0
    assert status.keya_busoff is True
    assert status.keya_errpass is True
    assert status.keya_warn is True
    assert status.keya_tec == 248
    assert status.keya_rec == 0


def test_status_parser_accepts_protocol_2_firmware_without_bus_status():
    status = parse_status(STATUS_V2)
    assert status.protocol == 2
    assert status.ez_command_rpm == 1200
    assert status.keya_busoff is False
    assert status.keya_tec == 0


def test_status_parser_accepts_legacy_firmware_without_plant_fields():
    status = parse_status(LEGACY_STATUS)
    assert status.protocol == 1
    assert status.plant is False
    assert status.ez_command_rpm == 0.0
    assert status.keya_rx_age_ms == 4294967295


def test_protocol_2_status_requires_all_observation_fields():
    with pytest.raises(ValueError, match="missing protocol 2 STATUS keys"):
        parse_status(STATUS_V2.replace(" ez_command_rpm=1200", ""))


def test_protocol_3_status_requires_all_bus_status_fields():
    with pytest.raises(ValueError, match="missing protocol 3 STATUS keys"):
        parse_status(STATUS.replace(" keya_busoff=1", ""))


def test_status_parser_rejects_unsupported_protocol():
    with pytest.raises(ValueError, match="unsupported STATUS protocol"):
        parse_status(STATUS.replace("protocol=3", "protocol=4"))


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
    # The STATUS fixture predates closed loop becoming the default.
    desired.plant_enabled = False
    assert desired_status_mismatches(desired, status) == []


def test_plant_command_round_trips_through_desired_state():
    assert build_set_plant(True) == "SET PLANT ON"
    assert validate_command("set plant off") == "SET PLANT OFF"
    with pytest.raises(ValueError):
        validate_command("SET PLANT MAYBE")

    desired = HilDesiredState()
    # Closed loop is the bench default, and the plant must be started after the
    # injected motion that forms its initial condition.
    assert desired.plant_enabled is True
    assert desired_commands(desired)[-1] == "SET PLANT ON"
    update_desired(desired, "SET PLANT OFF")
    assert desired.plant_enabled is False
    # Opening the loop has to come first, or the running plant overwrites the
    # SET SPEED/RPM that follows it.
    assert desired_commands(desired)[0] == "SET PLANT OFF"
    assert not any(
        command.startswith("SET PLANT ")
        for command in desired_commands(desired, protocol_version=1)
    )


def test_running_plant_excludes_injected_motion_from_mismatches():
    """The plant drives speed/rpm, so desired motion must not be compared."""
    status = parse_status(
        STATUS.replace("plant=0", "plant=1").replace("speed=5.20", "speed=9.90")
    )
    desired = HilDesiredState(
        mode="AUTO", pedal=25.0, speed=5.2, motion_source="SPEED",
        voltage=52.3, current=12.5, motor_temp=40.0, steering_position=2.0,
        estop_side=True, keya_fault=True, plant_enabled=True,
    )
    assert desired_status_mismatches(desired, status) == []

    # The plant flag itself is still compared.
    desired.plant_enabled = False
    assert any(
        entry.startswith("plant:")
        for entry in desired_status_mismatches(desired, status)
    )
