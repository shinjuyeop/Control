from __future__ import annotations

from kaiev26_hil.vcan_sim.can_io import MotorCommand, SteeringCommand
from kaiev26_hil.vcan_sim.physics import MotorPlant, SteeringPlant


def _enable_motor(plant: MotorPlant) -> None:
    plant.apply_command(MotorCommand(handshake_ack=True), 0.0)
    plant.apply_command(
        MotorCommand(
            handshake_ack=False,
            target_rpm=1800.0,
            phase_limit_a=220.0,
            ctrl_run=True,
            ctrl_speed_mode=True,
            life=1,
        ),
        0.0,
    )


def test_motor_grade_load_reduces_acceleration():
    flat = MotorPlant(road_grade_deg=0.0, rolling_resistance_coeff=0.0, aero_drag_coeff=0.0)
    uphill = MotorPlant(road_grade_deg=12.0, rolling_resistance_coeff=0.0, aero_drag_coeff=0.0)

    _enable_motor(flat)
    _enable_motor(uphill)

    for step_idx in range(40):
        now = (step_idx + 1) * 0.05
        flat.step(0.05, now)
        uphill.step(0.05, now)

    assert flat.snapshot().actual_rpm > uphill.snapshot().actual_rpm + 50.0


def test_steering_backlash_blocks_small_command_but_not_large_command():
    plant = SteeringPlant(backlash_deg=2.0, static_friction_deg=0.0)
    plant.apply_command(SteeringCommand(enable=True))
    plant.apply_command(SteeringCommand(target_handle_deg=1.0))

    for _ in range(40):
        plant.step(0.02)

    assert abs(plant.snapshot().raw_handle_deg) < 0.1

    plant.apply_command(SteeringCommand(target_handle_deg=5.0))
    for _ in range(60):
        plant.step(0.02)

    assert abs(plant.snapshot().raw_handle_deg) > 1.0
