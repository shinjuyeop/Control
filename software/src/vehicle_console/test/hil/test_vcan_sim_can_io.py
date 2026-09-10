from __future__ import annotations

import pytest

from vehicle_console.hil.vcan_sim.can_io import (
    CanFrame,
    CTRL_RUN,
    CTRL_SPEED,
    ID_MOTOR_CMD,
    ID_MOTOR_STAT1,
    ID_STEER_CMD,
    MotorCommand,
    SteeringCommand,
    encode_motor_status_frames,
    make_motor_cmd_frame,
    make_motor_handshake_ack,
    make_steer_enable_frame,
    make_steer_pos_frame_from_wheel_deg,
    parse_motor_command,
    parse_steering_command,
)
from vehicle_console.hil.vcan_sim.physics import MotorPlant


def test_motor_command_decoder_matches_interface_bytes():
    frame = CanFrame(
        "vcan0",
        ID_MOTOR_CMD,
        bytes.fromhex("68 8D D2 81 03 00 00 5A"),
    )

    assert parse_motor_command(frame) == MotorCommand(
        handshake_ack=False,
        target_rpm=1234.0,
        phase_limit_a=420.0,
        ctrl_run=True,
        ctrl_speed_mode=True,
        life=0x5A,
    )


def test_motor_command_encoder_matches_interface_bytes():
    frame = make_motor_cmd_frame("vcan0", 1234, current_limit_a=420.0, run=True)

    assert frame.can_id == ID_MOTOR_CMD
    assert frame.data == bytes.fromhex("68 8D D2 81 03 00 00 00")


def test_motor_handshake_matches_firmware_bytes():
    received = CanFrame("vcan0", ID_MOTOR_CMD, b"\xAA" * 8)

    assert parse_motor_command(received) == MotorCommand(handshake_ack=True)
    assert make_motor_handshake_ack("vcan0").data == b"\xAA" * 8


def test_steering_decoder_matches_keya_firmware_bytes():
    enable = CanFrame(
        "vcan1", ID_STEER_CMD, bytes.fromhex("23 0D 20 01 00 00 00 00")
    )
    disable = CanFrame(
        "vcan1", ID_STEER_CMD, bytes.fromhex("23 0C 20 01 00 00 00 00")
    )
    position = CanFrame(
        "vcan1", ID_STEER_CMD, bytes.fromhex("23 02 20 01 00 00 27 10")
    )

    assert parse_steering_command(enable) == SteeringCommand(enable=True)
    assert parse_steering_command(disable) == SteeringCommand(enable=False)
    assert parse_steering_command(position) == SteeringCommand(target_handle_deg=360.0)


def test_steering_encoder_matches_keya_firmware_bytes():
    assert make_steer_enable_frame("vcan1", True).data == bytes.fromhex(
        "23 0D 20 01 00 00 00 00"
    )
    assert make_steer_enable_frame("vcan1", False).data == bytes.fromhex(
        "23 0C 20 01 00 00 00 00"
    )
    assert make_steer_pos_frame_from_wheel_deg("vcan1", 0.0).data == bytes.fromhex(
        "23 02 20 01 00 00 00 00"
    )
    with pytest.raises(ValueError, match="must be finite"):
        make_steer_pos_frame_from_wheel_deg("vcan1", float("nan"))


def test_motor_status_frames_encode_status_bits():
    status1, status2 = encode_motor_status_frames(
        "vcan0",
        bus_voltage_v=52.4,
        bus_current_a=10.0,
        phase_current_a=15.0,
        actual_rpm=321.0,
        ctrl_run=True,
        ctrl_speed_mode=True,
        ctrl_temp_c=30.0,
        motor_temp_c=40.0,
        error_group=0x001234,
    )

    assert status1.can_id == ID_MOTOR_STAT1
    assert status2.data[2] == (CTRL_RUN | CTRL_SPEED)
    assert status2.data[3] == 0x34
    assert status2.data[4] == 0x12


def test_motor_timeout_drops_target_and_run_state():
    plant = MotorPlant()
    plant.apply_command(MotorCommand(handshake_ack=True), 0.0)
    plant.apply_command(
        MotorCommand(
            handshake_ack=False,
            target_rpm=800.0,
            phase_limit_a=150.0,
            ctrl_run=True,
            ctrl_speed_mode=True,
            life=1,
        ),
        0.0,
    )

    plant.step(0.05, 2.5)
    snapshot = plant.snapshot()

    assert snapshot.target_rpm == 0.0
    assert snapshot.ctrl_run is False
