from __future__ import annotations

from kaiev26_hil.bench.can_io import (
    CTRL_RUN,
    CTRL_SPEED,
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
    wheel_to_handle_deg_for_cmd,
)
from kaiev26_hil.bench.physics import MotorPlant


def test_motor_command_round_trip_preserves_target_and_ctrl_bits():
    frame = make_motor_cmd_frame("vcan0", 1234, current_limit_a=420.0, run=True)
    parsed = parse_motor_command(frame)

    assert parsed == MotorCommand(
        handshake_ack=False,
        target_rpm=1234.0,
        phase_limit_a=420.0,
        ctrl_run=True,
        ctrl_speed_mode=True,
        life=0,
    )


def test_motor_handshake_ack_is_detected():
    parsed = parse_motor_command(make_motor_handshake_ack("vcan0"))
    assert parsed == MotorCommand(handshake_ack=True)


def test_steering_frames_round_trip_for_enable_and_position():
    enable = parse_steering_command(make_steer_enable_frame("vcan1", True))
    position_frame = make_steer_pos_frame_from_wheel_deg("vcan1", 10.0)
    position = parse_steering_command(position_frame)

    assert enable == SteeringCommand(enable=True)
    assert position is not None
    assert position.enable is None
    assert abs(position.target_handle_deg - wheel_to_handle_deg_for_cmd(10.0)) < 0.05
    assert position_frame.can_id == ID_STEER_CMD


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
