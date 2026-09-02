import math

from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
    VehicleState,
)
import pytest
from vehicle_state_publisher.vehicle_state_publisher_node import (
    fuse_vehicle_state,
    make_wheel_twist,
)


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
        | BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE
    )

    state = fuse_vehicle_state(steering, drive, brake)

    assert state.estop_active is True


def test_fuse_command_brake_alone_is_not_estop():
    steering, drive, brake = make_inputs()
    brake.stop_reason_bits = BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE

    state = fuse_vehicle_state(steering, drive, brake)

    assert state.estop_active is False


def test_wheel_twist_uses_measured_speed_and_ackermann_angle():
    steering, drive, _ = make_inputs()
    steering.feedback_valid = True
    steering.actual_wheel_angle_rad = 0.2
    drive.part1_valid = True
    drive.speed_mps = 2.0

    msg = make_wheel_twist(
        steering,
        drive,
        stamp=steering.header.stamp,
        frame_id="base_footprint",
        wheelbase_m=1.3,
        speed_scale=1.0,
        steering_scale=1.0,
        steering_offset_rad=0.0,
        yaw_rate_scale=1.0,
        longitudinal_velocity_variance=0.016,
        lateral_velocity_variance=0.25,
        yaw_rate_variance=0.0016,
    )

    assert msg is not None
    assert msg.header.frame_id == "base_footprint"
    assert msg.twist.twist.linear.x == 2.0
    assert msg.twist.twist.angular.z == pytest.approx(2.0 * math.tan(0.2) / 1.3)
    assert msg.twist.covariance[0] == 0.016
    assert msg.twist.covariance[7] == 0.25
    assert msg.twist.covariance[35] == 0.0016


def test_wheel_twist_requires_both_feedback_sources():
    steering, drive, _ = make_inputs()
    steering.feedback_valid = True
    drive.part1_valid = False

    assert make_wheel_twist(
        steering,
        drive,
        stamp=steering.header.stamp,
        frame_id="base_footprint",
        wheelbase_m=1.3,
        speed_scale=1.0,
        steering_scale=1.0,
        steering_offset_rad=0.0,
        yaw_rate_scale=1.0,
        longitudinal_velocity_variance=0.016,
        lateral_velocity_variance=0.25,
        yaw_rate_variance=0.0016,
    ) is None
