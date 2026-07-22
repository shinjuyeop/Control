from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
    VehicleState,
)
from vehicle_state_publisher.vehicle_state_publisher_node import fuse_vehicle_state


def make_inputs():
    steering = SteeringActuatorStatus()
    drive = DriveActuatorStatus()
    brake = BrakeActuatorStatus()
    return steering, drive, brake


def test_fuse_maps_each_source_field():
    steering, drive, brake = make_inputs()
    steering.actual_wheel_angle_rad = 0.1
    drive.speed_mps = 2.5
    drive.gear = DriveActuatorStatus.GEAR_REVERSE
    drive.mode = DriveActuatorStatus.MODE_AUTO

    state = fuse_vehicle_state(steering, drive, brake)

    assert state.speed_mps == 2.5
    assert state.steering_rad == 0.1
    assert state.gear == VehicleState.GEAR_REVERSE
    assert state.mode == VehicleState.MODE_AUTO
    assert state.estop_active is False


def test_fuse_reads_estop_from_brake_reason_bits():
    steering, drive, brake = make_inputs()
    brake.stop_reason_bits = (
        BrakeActuatorStatus.STOP_REASON_ESTOP
        | BrakeActuatorStatus.STOP_REASON_SERVICE_BRAKE
    )

    state = fuse_vehicle_state(steering, drive, brake)

    assert state.estop_active is True


def test_fuse_service_brake_alone_is_not_estop():
    steering, drive, brake = make_inputs()
    brake.stop_reason_bits = BrakeActuatorStatus.STOP_REASON_SERVICE_BRAKE

    state = fuse_vehicle_state(steering, drive, brake)

    assert state.estop_active is False
