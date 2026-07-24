from kaiev26_control.control_command_gate_node import ControlCommandGateNode
from kaiev26_msgs.msg import ActuatorCommand, TargetSpeed, VehicleState


def make_gate(
    commanded_speed: float,
    vehicle_speed: float,
    target_speed: float = 0.0,
    stop_target_distance: float = 10.0,
) -> ControlCommandGateNode:
    gate = ControlCommandGateNode.__new__(ControlCommandGateNode)
    gate.max_accel_mps2 = 1.0
    gate.max_decel_mps2 = 2.0
    gate.stop_speed_epsilon_mps = 0.05
    gate.planned_stop_brake_distance_m = 2.0
    gate.commanded_speed_mps = commanded_speed
    gate.commanded_steering_rad = 0.0
    gate.latest_target_speed = TargetSpeed()
    gate.latest_target_speed.need_stop = True
    gate.latest_target_speed.target_speed_mps = target_speed
    gate.latest_target_speed.stop_target_distance = stop_target_distance
    gate.latest_vehicle_state = VehicleState()
    gate.latest_vehicle_state.speed_mps = vehicle_speed
    gate.compute_limited_steering = lambda dt: 0.0
    return gate


def test_planned_stop_respects_distance_based_target_speed() -> None:
    gate = make_gate(commanded_speed=5.0, vehicle_speed=5.0, target_speed=4.5)
    command = ActuatorCommand()

    gate.apply_nominal_command(command, dt=0.5)

    assert command.speed_target_mps == 4.5
    assert command.brake_engage is False
    assert command.gear == ActuatorCommand.GEAR_FORWARD


def test_planned_stop_decelerates_before_holding_brake_at_zero_target() -> None:
    gate = make_gate(commanded_speed=2.0, vehicle_speed=2.0)
    command = ActuatorCommand()

    gate.apply_nominal_command(command, dt=0.5)

    assert command.speed_target_mps == 1.0
    assert command.brake_engage is False
    assert command.gear == ActuatorCommand.GEAR_FORWARD


def test_planned_stop_holds_brake_after_vehicle_is_stopped() -> None:
    gate = make_gate(commanded_speed=0.02, vehicle_speed=0.02, stop_target_distance=0.0)
    command = ActuatorCommand()

    gate.apply_nominal_command(command, dt=0.5)

    assert command.speed_target_mps == 0.0
    assert command.brake_engage is True
    assert command.gear == ActuatorCommand.GEAR_FORWARD


def test_planned_stop_brakes_when_stop_target_is_close() -> None:
    gate = make_gate(commanded_speed=1.2, vehicle_speed=1.0, target_speed=0.8, stop_target_distance=1.5)
    command = ActuatorCommand()

    gate.apply_nominal_command(command, dt=0.2)

    assert command.brake_engage is True
    assert command.speed_target_mps == 0.0
    assert command.gear == ActuatorCommand.GEAR_FORWARD
