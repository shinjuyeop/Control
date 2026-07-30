import math

from kaiev26_msgs.msg import (
    ActuatorCommand,
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class VehicleScenarioPublisher(Node):
    """Publish a deterministic synthetic version of the canonical interface."""

    def __init__(self):
        super().__init__('test_publisher')

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('loop_duration_sec', 70.0)
        self.publish_rate_hz = max(
            1.0,
            float(self.get_parameter('publish_rate_hz').value),
        )
        self.loop_duration_sec = max(
            70.0,
            float(self.get_parameter('loop_duration_sec').value),
        )

        self.command_pub = self.create_publisher(
            ActuatorCommand,
            '/vehicle/command',
            10,
        )
        self.drive_pub = self.create_publisher(
            DriveActuatorStatus,
            '/drive/status',
            10,
        )
        self.brake_pub = self.create_publisher(
            BrakeActuatorStatus,
            '/brake/status',
            10,
        )
        self.steering_pub = self.create_publisher(
            SteeringActuatorStatus,
            '/steering/status',
            10,
        )

        self.start_time = self.get_clock().now()
        self.actual_speed_mps = 0.0
        self.actual_steering_rad = 0.0
        self.previous_steering_rad = 0.0

        self.get_logger().warning(
            'SYNTHETIC TEST ONLY: publishing /vehicle/command directly. '
            'Never run test_publisher with the real vehicle control system.'
        )
        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_state,
        )

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def scenario(self, elapsed_sec):
        phase = elapsed_sec % self.loop_duration_sec
        motion_phase = phase % 18.0
        target_speed_mps = 2.2 + 1.2 * math.sin(
            math.tau * motion_phase / 18.0 - math.pi / 2.0
        )
        target_steering_rad = math.radians(
            11.0 * math.sin(math.tau * phase / 16.0)
        )

        return {
            'phase': phase,
            'target_speed_mps': max(0.0, target_speed_mps),
            'target_steering_rad': target_steering_rad,
            'command_brake': 18.0 <= phase < 23.0,
            'physical_estop': 28.0 <= phase < 33.0,
            'command_timeout': 42.0 <= phase < 47.0,
            'motor_fault': 47.0 <= phase < 54.0,
            'steering_fault': 54.0 <= phase < 62.0,
        }

    def update_response(self, scenario):
        dt = 1.0 / self.publish_rate_hz
        braking = (
            scenario['command_brake']
            or scenario['physical_estop']
            or scenario['command_timeout']
        )
        response_target = (
            0.0 if braking else scenario['target_speed_mps']
        )
        speed_step = 2.0 * dt if braking else 0.65 * dt
        self.actual_speed_mps += self.clamp(
            response_target - self.actual_speed_mps,
            -speed_step,
            speed_step,
        )

        self.previous_steering_rad = self.actual_steering_rad
        steering_target = scenario['target_steering_rad']
        if scenario['physical_estop'] or scenario['steering_fault']:
            steering_target = 0.0
        steering_step = math.radians(24.0) * dt
        self.actual_steering_rad += self.clamp(
            steering_target - self.actual_steering_rad,
            -steering_step,
            steering_step,
        )

    def make_command(self, stamp, scenario):
        msg = ActuatorCommand()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.steering_target_rad = float(scenario['target_steering_rad'])
        msg.speed_target_mps = float(scenario['target_speed_mps'])
        msg.brake_engage = bool(scenario['command_brake'])
        return msg

    def make_drive_status(self, stamp, scenario):
        msg = DriveActuatorStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.mode = DriveActuatorStatus.MODE_AUTO
        msg.gear = DriveActuatorStatus.GEAR_FORWARD
        msg.speed_mps = float(self.actual_speed_mps)
        msg.motor_rpm = float(self.actual_speed_mps * 310.0)
        msg.tractive_commanded = not (
            scenario['command_brake']
            or scenario['physical_estop']
            or scenario['command_timeout']
        )
        msg.tractive_feedback = msg.tractive_commanded
        msg.tractive_feedback_valid = True
        msg.stop_request = not msg.tractive_commanded
        msg.fault_bits = 0x000120 if scenario['motor_fault'] else 0
        msg.fault_bits_valid = True
        msg.command_received = True
        msg.command_timeout = bool(scenario['command_timeout'])
        msg.command_age_ms = 850.0 if msg.command_timeout else 18.0
        msg.bus_voltage_v = 48.2 - self.actual_speed_mps * 0.12
        msg.bus_current_a = self.actual_speed_mps * 3.4
        msg.phase_current_a = self.actual_speed_mps * 5.1
        msg.part1_valid = True
        msg.part1_age_ms = 12.0
        msg.controller_temperature_c = 41.0 + self.actual_speed_mps
        msg.motor_temperature_c = 45.0 + self.actual_speed_mps * 1.5
        msg.controller_control_mode = 1
        msg.part2_valid = True
        msg.part2_age_ms = 17.0
        return msg

    def make_brake_status(self, stamp, scenario):
        msg = BrakeActuatorStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'
        msg.brake_commanded = bool(
            scenario['command_brake']
            or scenario['physical_estop']
            or scenario['command_timeout']
        )
        reason_bits = 0
        if scenario['command_brake']:
            reason_bits |= BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE
        if scenario['physical_estop']:
            reason_bits |= BrakeActuatorStatus.STOP_REASON_ESTOP
        if scenario['command_timeout']:
            reason_bits |= BrakeActuatorStatus.STOP_REASON_COMMAND_INVALID
        msg.stop_reason_bits = reason_bits
        # The current physical servo contract has no actual brake feedback.
        msg.actual_valid = False
        msg.command_age_ms = 850.0 if scenario['command_timeout'] else 18.0
        return msg

    def make_steering_status(self, stamp, scenario):
        msg = SteeringActuatorStatus()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        if scenario['physical_estop']:
            msg.state = SteeringActuatorStatus.STATE_ESTOP
            msg.disable_reason = SteeringActuatorStatus.REASON_ESTOP
        elif scenario['command_brake']:
            msg.state = SteeringActuatorStatus.STATE_BRAKE
            msg.disable_reason = SteeringActuatorStatus.REASON_COMMAND_BRAKE
        elif scenario['command_timeout']:
            msg.state = SteeringActuatorStatus.STATE_DISABLED
            msg.disable_reason = (
                SteeringActuatorStatus.REASON_STEER_COMMAND_TIMEOUT
            )
        elif scenario['steering_fault']:
            msg.state = SteeringActuatorStatus.STATE_FAULT
            msg.disable_reason = SteeringActuatorStatus.REASON_MOTOR_FAULT
        else:
            msg.state = SteeringActuatorStatus.STATE_ACTIVE
            msg.disable_reason = SteeringActuatorStatus.REASON_NONE

        msg.drive_mode = DriveActuatorStatus.MODE_AUTO
        msg.mode_received = True
        msg.mode_timeout = False
        msg.steer_command_timeout = bool(scenario['command_timeout'])
        msg.feedback_timeout = False
        msg.steer_command_received = True
        msg.pending_command_valid = not scenario['command_timeout']
        msg.movement_target_valid = not (
            scenario['physical_estop']
            or scenario['command_timeout']
            or scenario['steering_fault']
        )
        msg.estop_active = bool(scenario['physical_estop'])
        msg.command_brake_active = bool(scenario['command_brake'])
        msg.communication_ok = not scenario['command_timeout']
        msg.can_timeout = False
        msg.feedback_valid = True
        msg.enabled_commanded = msg.state == SteeringActuatorStatus.STATE_ACTIVE
        msg.enabled_feedback = msg.enabled_commanded
        msg.enabled_feedback_valid = True
        msg.enable_mismatch = False
        msg.tracking_control_enabled = msg.enabled_commanded
        tracking_error = (
            scenario['target_steering_rad'] - self.actual_steering_rad
        )
        msg.tracking_error_warning = abs(tracking_error) > math.radians(3.0)
        msg.tracking_error_fault = bool(scenario['steering_fault'])
        msg.target_wheel_angle_rad = float(
            scenario['target_steering_rad']
        )
        msg.commanded_wheel_angle_rad = float(
            scenario['target_steering_rad']
        )
        msg.corrected_wheel_angle_rad = float(
            scenario['target_steering_rad']
        )
        msg.actual_wheel_angle_rad = float(self.actual_steering_rad)
        msg.tracking_error_rad = float(tracking_error)
        msg.tracking_correction_rad = 0.0
        msg.steering_rate_radps = float(
            (self.actual_steering_rad - self.previous_steering_rad)
            * self.publish_rate_hz
        )
        msg.motor_position_count = int(
            self.actual_steering_rad * 100000.0
        )
        msg.motor_angle_deg = math.degrees(self.actual_steering_rad) * 15.0
        msg.motor_speed_rpm = msg.steering_rate_radps * 143.24
        msg.motor_current_a = 1.3 + abs(tracking_error) * 5.0
        msg.bus_voltage_v = 24.1
        msg.driver_temperature_c = 38.0
        msg.fault_bits = 0x0040 if scenario['steering_fault'] else 0
        msg.feedback_age_ms = 14.0
        msg.mode_age_ms = 20.0
        msg.steer_command_age_ms = (
            850.0 if scenario['command_timeout'] else 16.0
        )
        return msg

    def publish_state(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1e-9
        scenario = self.scenario(elapsed)
        self.update_response(scenario)
        stamp = now.to_msg()

        # Stop the command stream during this segment so both the local GUI
        # freshness indicator and the actuator-reported timeout are visible.
        if not scenario['command_timeout']:
            self.command_pub.publish(self.make_command(stamp, scenario))
        self.drive_pub.publish(self.make_drive_status(stamp, scenario))
        self.brake_pub.publish(self.make_brake_status(stamp, scenario))
        self.steering_pub.publish(
            self.make_steering_status(stamp, scenario)
        )


def main(args=None):
    rclpy.init(args=args)
    node = VehicleScenarioPublisher()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
