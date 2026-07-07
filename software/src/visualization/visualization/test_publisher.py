import math
import random

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8


class VehicleScenarioPublisher(Node):
    """Publish virtual vehicle states for Foxglove visualization testing."""

    def __init__(self):
        super().__init__('test_publisher')

        self.pedal_voltage_pub = self.create_publisher(Float32, '/pedal_voltage', 10)
        self.cmd_speed_pub = self.create_publisher(Float32, '/cmd/speed', 10)
        self.ctrl_rpm_pub = self.create_publisher(Float32, '/ctrl/rpm', 10)
        self.cmd_steer_pub = self.create_publisher(Float32, '/cmd/steer', 10)
        self.ctrl_str_pub = self.create_publisher(Float32, '/ctrl/str', 10)

        self.mode_pub = self.create_publisher(UInt8, '/mode', 10)
        self.gear_pub = self.create_publisher(UInt8, '/gear', 10)
        self.estop_pub = self.create_publisher(UInt8, '/estop', 10)
        self.aeb_pub = self.create_publisher(UInt8, '/aeb', 10)

        # These states can be changed at runtime with ros2 param set.
        self.declare_parameter('mode', 1)
        self.declare_parameter('gear', 0)
        self.declare_parameter('estop', 0)
        self.declare_parameter('aeb', 0)
        self.declare_parameter('p_gain', 1.0)
        self.declare_parameter('i_gain', 0.1)
        self.declare_parameter('d_gain', 0.01)
        self.declare_parameter('scenario_enabled', True)
        self.declare_parameter('simulate_aeb_event', True)
        self.declare_parameter('aeb_event_period_sec', 15.0)
        self.declare_parameter('aeb_event_duration_sec', 3.0)
        self.declare_parameter('noise_enabled', True)
        self.declare_parameter('loop_duration_sec', 70.0)

        self.p_gain = self.get_parameter('p_gain').value
        self.i_gain = self.get_parameter('i_gain').value
        self.d_gain = self.get_parameter('d_gain').value
        self.scenario_enabled = self.get_parameter('scenario_enabled').value
        self.simulate_aeb_event = self.get_parameter('simulate_aeb_event').value
        self.aeb_event_period_sec = self.get_parameter(
            'aeb_event_period_sec'
        ).value
        self.aeb_event_duration_sec = self.get_parameter(
            'aeb_event_duration_sec'
        ).value
        self.noise_enabled = self.get_parameter('noise_enabled').value
        self.loop_duration_sec = self.get_parameter('loop_duration_sec').value
        self.param_cb_handle = self.add_on_set_parameters_callback(
            self.update_parameters
        )
        self.log_pid_gains()

        self.start_time = self.get_clock().now()
        self.pedal_voltage_min = 0.8
        self.pedal_voltage_max = 4.0
        self.rpm_min = 0.0
        self.rpm_max = 4000.0
        self.pedal_rpm_deadband = 1.5
        self.tire_diameter_cm = 47.0
        self.transmission_ratio = 4.60
        self.steer_poly_a = 0.0002
        self.steer_poly_b = 0.2159
        self.steer_poly_c = 0.1612
        self.wheel_limit_deg = 30.0
        self.current_speed_kmh = 0.0
        self.current_handle_deg = 0.0
        self.rng = random.Random(42)

        # 10 Hz virtual state publisher.
        self.timer = self.create_timer(0.1, self.publish_state)

    def log_pid_gains(self):
        self.get_logger().info(
            f'PID gains updated: P={self.p_gain:.4f}, '
            f'I={self.i_gain:.4f}, D={self.d_gain:.4f}'
        )

    def update_parameters(self, params):
        updated = False
        for param in params:
            if param.name == 'p_gain':
                self.p_gain = float(param.value)
                updated = True
            elif param.name == 'i_gain':
                self.i_gain = float(param.value)
                updated = True
            elif param.name == 'd_gain':
                self.d_gain = float(param.value)
                updated = True
            elif param.name == 'scenario_enabled':
                self.scenario_enabled = bool(param.value)
            elif param.name == 'simulate_aeb_event':
                self.simulate_aeb_event = bool(param.value)
            elif param.name == 'aeb_event_period_sec':
                self.aeb_event_period_sec = max(1.0, float(param.value))
            elif param.name == 'aeb_event_duration_sec':
                self.aeb_event_duration_sec = max(0.1, float(param.value))
            elif param.name == 'noise_enabled':
                self.noise_enabled = bool(param.value)
            elif param.name == 'loop_duration_sec':
                self.loop_duration_sec = max(10.0, float(param.value))

        if updated:
            self.log_pid_gains()
        return SetParametersResult(successful=True)

    def get_binary_parameter(self, name):
        return 1 if int(self.get_parameter(name).value) else 0

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    def pedal_voltage_to_rpm(self, voltage):
        voltage = self.clamp(
            voltage,
            self.pedal_voltage_min,
            self.pedal_voltage_max,
        )
        rpm = (
            (voltage - self.pedal_voltage_min)
            / (self.pedal_voltage_max - self.pedal_voltage_min)
            * (self.rpm_max - self.rpm_min)
            + self.rpm_min
        )
        rpm = 0.0 if rpm < self.pedal_rpm_deadband else rpm
        return self.clamp(rpm, self.rpm_min, self.rpm_max)

    def speed_kmh_to_rpm(self, speed_kmh):
        wheel_rpm = (
            abs(speed_kmh)
            * 100000.0
            / (math.pi * self.tire_diameter_cm * 60.0)
        )
        rpm = wheel_rpm * self.transmission_ratio
        return self.clamp(rpm, self.rpm_min, self.rpm_max)

    def rpm_to_speed_kmh(self, rpm):
        wheel_rpm = rpm / self.transmission_ratio
        return wheel_rpm * math.pi * self.tire_diameter_cm * 60.0 / 100000.0

    def wheel_to_handle_steer_deg(self, wheel_deg):
        wheel_deg = self.clamp(
            wheel_deg,
            -self.wheel_limit_deg,
            self.wheel_limit_deg,
        )
        if abs(wheel_deg) <= self.steer_poly_c:
            return 0.0

        sign = 1.0 if wheel_deg >= 0.0 else -1.0
        wheel_abs = abs(wheel_deg)
        discriminant = (
            self.steer_poly_b * self.steer_poly_b
            - 4.0 * self.steer_poly_a * (self.steer_poly_c - wheel_abs)
        )
        if discriminant < 0.0:
            return 0.0

        handle_abs = (
            -self.steer_poly_b + math.sqrt(discriminant)
        ) / (2.0 * self.steer_poly_a)
        handle_abs = max(handle_abs, 0.0)
        return -sign * handle_abs

    def sample_noise(self, amplitude):
        if not self.noise_enabled:
            return 0.0
        return self.rng.uniform(-amplitude, amplitude)

    def auto_aeb_active(self, elapsed):
        if not self.simulate_aeb_event:
            return False

        period = max(1.0, self.aeb_event_period_sec)
        duration = min(max(0.1, self.aeb_event_duration_sec), period)
        phase = elapsed % period
        return elapsed >= period and phase < duration

    def scenario_command(self, elapsed):
        phase = elapsed % self.loop_duration_sec

        if phase < 8.0:
            speed_kmh = 1.2 * phase
        elif phase < 22.0:
            speed_kmh = 10.0 + 2.5 * math.sin(0.32 * (phase - 8.0))
        elif phase < 34.0:
            speed_kmh = 13.0 - 0.35 * (phase - 22.0)
        elif phase < 42.0:
            speed_kmh = 8.0
        elif phase < 49.0:
            speed_kmh = max(0.0, 8.0 - 1.2 * (phase - 42.0))
        elif phase < 55.0:
            speed_kmh = 0.0
        else:
            speed_kmh = min(10.0, 1.0 * (phase - 55.0))

        steer_deg = (
            7.0 * math.sin(0.22 * phase)
            + 3.5 * math.sin(0.55 * phase + 0.7)
        )
        aeb_active = self.auto_aeb_active(elapsed)
        return speed_kmh, steer_deg, aeb_active

    def update_vehicle_response(self, command_speed_kmh, command_handle_deg):
        dt = 0.1
        speed_error = command_speed_kmh - self.current_speed_kmh
        accel_limit = 0.8 if speed_error >= 0.0 else 2.5
        self.current_speed_kmh += self.clamp(
            speed_error,
            -accel_limit * dt * 3.6,
            accel_limit * dt * 3.6,
        )

        handle_error = command_handle_deg - self.current_handle_deg
        self.current_handle_deg += self.clamp(handle_error, -18.0 * dt, 18.0 * dt)

    def publish_state(self):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        mode = self.get_binary_parameter('mode')
        gear = self.get_binary_parameter('gear')
        estop = self.get_binary_parameter('estop')
        manual_aeb = self.get_binary_parameter('aeb')

        if self.scenario_enabled:
            cmd_speed, cmd_steer, scenario_aeb = self.scenario_command(elapsed)
        else:
            cmd_speed = 5.0 + 5.0 * math.sin(0.1 * elapsed)
            cmd_steer = 10.0 * math.sin(0.5 * elapsed)
            scenario_aeb = False

        aeb = 1 if manual_aeb or scenario_aeb else 0
        if estop == 1 or aeb == 1:
            cmd_speed = 0.0

        pedal_mid = (self.pedal_voltage_min + self.pedal_voltage_max) * 0.5
        pedal_amp = (self.pedal_voltage_max - self.pedal_voltage_min) * 0.45
        pedal_voltage = pedal_mid + pedal_amp * math.sin(0.18 * elapsed)
        pedal_rpm = self.pedal_voltage_to_rpm(pedal_voltage)
        auto_rpm = self.speed_kmh_to_rpm(cmd_speed)
        target_rpm = pedal_rpm if mode == 0 else auto_rpm
        target_rpm = 0.0 if estop == 1 or aeb == 1 else target_rpm
        target_rpm *= -1.0 if gear == 1 else 1.0

        target_handle = 0.0
        if mode == 1:
            target_handle = self.wheel_to_handle_steer_deg(cmd_steer)

        if estop == 1 or aeb == 1:
            response_speed = 0.0
        elif mode == 1:
            response_speed = cmd_speed
        else:
            response_speed = self.rpm_to_speed_kmh(pedal_rpm)
        response_speed *= -1.0 if gear == 1 else 1.0
        self.update_vehicle_response(response_speed, target_handle)

        ctrl_rpm = self.speed_kmh_to_rpm(self.current_speed_kmh)
        ctrl_rpm *= -1.0 if self.current_speed_kmh < 0.0 else 1.0
        ctrl_rpm += self.sample_noise(18.0)
        ctrl_rpm = self.clamp(ctrl_rpm, -self.rpm_max, self.rpm_max)

        ctrl_str = self.current_handle_deg + self.sample_noise(0.8)

        cmd_speed += self.sample_noise(0.15)
        cmd_steer += self.sample_noise(0.25)
        pedal_voltage += self.sample_noise(0.03)

        self.pedal_voltage_pub.publish(Float32(data=pedal_voltage))
        self.cmd_speed_pub.publish(Float32(data=cmd_speed))
        self.ctrl_rpm_pub.publish(Float32(data=ctrl_rpm))
        self.cmd_steer_pub.publish(Float32(data=cmd_steer))
        self.ctrl_str_pub.publish(Float32(data=ctrl_str))

        # Publish the current parameter values so they can be changed live.
        self.mode_pub.publish(UInt8(data=mode))
        self.gear_pub.publish(UInt8(data=gear))
        self.estop_pub.publish(UInt8(data=estop))
        self.aeb_pub.publish(UInt8(data=aeb))


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
