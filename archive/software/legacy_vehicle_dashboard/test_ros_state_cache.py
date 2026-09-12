import math
from types import SimpleNamespace

from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)

from visualization.ros_state_cache import (
    BRAKE,
    brake_reason_text,
    COMMAND,
    DRIVE,
    drive_gear_text,
    drive_mode_text,
    format_if_valid,
    NA,
    NO_DATA,
    OK,
    rad_to_deg,
    RosStateCache,
    STALE,
    STEERING,
    steering_disable_reason_text,
    steering_state_text,
    valid_bool_text,
)


STALE_THRESHOLDS = {
    COMMAND: 300.0,
    DRIVE: 300.0,
    BRAKE: 300.0,
    STEERING: 500.0,
}


def test_snapshot_contains_latest_message_and_no_data_for_others():
    cache = RosStateCache()
    message = SimpleNamespace(value=42)
    cache.update(DRIVE, message, received_at=10.0)

    snapshot = cache.snapshot(STALE_THRESHOLDS, now=10.1)

    assert snapshot[DRIVE].message is message
    assert snapshot[DRIVE].state == OK
    assert math.isclose(snapshot[DRIVE].age_ms, 100.0)
    assert snapshot[COMMAND].state == NO_DATA
    assert math.isinf(snapshot[COMMAND].age_ms)


def test_local_receive_age_drives_stale_classification():
    cache = RosStateCache()
    cache.update(COMMAND, object(), received_at=20.0)

    fresh = cache.snapshot(STALE_THRESHOLDS, now=20.299)
    stale = cache.snapshot(STALE_THRESHOLDS, now=20.301)

    assert fresh[COMMAND].state == OK
    assert stale[COMMAND].state == STALE
    assert stale[COMMAND].age_ms > 300.0


def test_drive_mode_and_gear_strings_use_message_constants():
    assert drive_mode_text(DriveActuatorStatus.MODE_MANUAL) == 'MANUAL'
    assert drive_mode_text(DriveActuatorStatus.MODE_AUTO) == 'AUTO'
    assert drive_mode_text(99) == 'UNKNOWN(99)'
    assert drive_gear_text(DriveActuatorStatus.GEAR_FORWARD) == 'FORWARD'
    assert drive_gear_text(DriveActuatorStatus.GEAR_REVERSE) == 'REVERSE'


def test_steering_state_and_disable_reason_strings():
    assert (
        steering_state_text(SteeringActuatorStatus.STATE_ACTIVE)
        == 'ACTIVE'
    )
    assert (
        steering_state_text(SteeringActuatorStatus.STATE_FAULT)
        == 'FAULT'
    )
    assert (
        steering_disable_reason_text(
            SteeringActuatorStatus.REASON_STEER_COMMAND_TIMEOUT
        )
        == 'STEER_COMMAND_TIMEOUT'
    )
    assert (
        steering_disable_reason_text(
            SteeringActuatorStatus.REASON_MOTOR_FAULT
        )
        == 'MOTOR_FAULT'
    )


def test_brake_reason_bitmask_uses_current_contract():
    bits = (
        BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE
        | BrakeActuatorStatus.STOP_REASON_ESTOP
    )

    assert brake_reason_text(0) == 'NONE'
    assert brake_reason_text(bits) == 'COMMAND_BRAKE | ESTOP'
    assert 'COMMAND_INVALID' in brake_reason_text(
        BrakeActuatorStatus.STOP_REASON_COMMAND_INVALID
    )


def test_rad_to_degree_display_conversion():
    assert math.isclose(rad_to_deg(math.pi / 2.0), 90.0)
    assert math.isclose(rad_to_deg(-math.pi / 6.0), -30.0)


def test_invalid_or_unsupported_values_are_na():
    assert format_if_valid(False, 48.2, '.1f', ' V') == NA
    assert valid_bool_text(False, True) == NA
    assert format_if_valid(True, 48.2, '.1f', ' V') == '48.2 V'
    assert valid_bool_text(True, False) == 'NO'
