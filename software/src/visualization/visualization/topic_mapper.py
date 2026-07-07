import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32, UInt8


class TopicMapper(Node):
    """Map vehicle control topics into Foxglove-friendly standard topics."""

    def __init__(self):
        super().__init__('topic_mapper')

        self.declare_parameter('pedal_voltage_min', 0.8)
        self.declare_parameter('pedal_voltage_max', 4.0)
        self.declare_parameter('rpm_min', 0.0)
        self.declare_parameter('rpm_max', 4000.0)
        self.declare_parameter('pedal_rpm_deadband', 1.5)
        self.declare_parameter('tire_diameter_cm', 47.0)
        self.declare_parameter('transmission_ratio', 4.60)
        self.declare_parameter('steer_poly_a', 0.0002)
        self.declare_parameter('steer_poly_b', 0.2159)
        self.declare_parameter('steer_poly_c', 0.1612)
        self.declare_parameter('wheel_limit_deg', 30.0)

        self.pedal_voltage_min = self.get_parameter('pedal_voltage_min').value
        self.pedal_voltage_max = self.get_parameter('pedal_voltage_max').value
        self.rpm_min = self.get_parameter('rpm_min').value
        self.rpm_max = self.get_parameter('rpm_max').value
        self.pedal_rpm_deadband = self.get_parameter('pedal_rpm_deadband').value
        self.tire_diameter_cm = self.get_parameter('tire_diameter_cm').value
        self.transmission_ratio = self.get_parameter('transmission_ratio').value
        self.steer_poly_a = self.get_parameter('steer_poly_a').value
        self.steer_poly_b = self.get_parameter('steer_poly_b').value
        self.steer_poly_c = self.get_parameter('steer_poly_c').value
        self.wheel_limit_deg = self.get_parameter('wheel_limit_deg').value

        self.mode = 1
        self.gear = 0
        self.estop = 0
        self.aeb = 0
        self.last_pedal_rpm = 0.0
        self.last_cmd_speed_kmh = 0.0
        self.last_cmd_steer_deg = 0.0

        self.pedal_rpm_pub = self.create_publisher(Float32, '/pedal_rpm', 10)
        self.target_rpm_pub = self.create_publisher(Float32, '/target/rpm', 10)
        self.current_speed_pub = self.create_publisher(Float32, '/current/speed', 10)
        self.target_steer_pub = self.create_publisher(Float32, '/target/steer', 10)
        self.current_str_pub = self.create_publisher(Float32, '/current/str', 10)
        self.viz_steering_pub = self.create_publisher(
            Float32,
            '/viz/steering_deg',
            10,
        )

        self.create_subscription(Float32, '/pedal_voltage', self.pedal_voltage_cb, 10)
        self.create_subscription(Float32, '/cmd/speed', self.cmd_speed_cb, 10)
        self.create_subscription(Float32, '/ctrl/rpm', self.ctrl_rpm_cb, 10)
        self.create_subscription(Float32, '/cmd/steer', self.cmd_steer_cb, 10)
        self.create_subscription(Float32, '/ctrl/str', self.ctrl_str_cb, 10)
        self.create_subscription(UInt8, '/mode', self.mode_cb, 10)
        self.create_subscription(UInt8, '/gear', self.gear_cb, 10)
        self.create_subscription(UInt8, '/estop', self.estop_cb, 10)
        self.create_subscription(UInt8, '/aeb', self.aeb_cb, 10)

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def as_binary(value):
        return 1 if int(value) else 0

    def gear_sign(self):
        return -1.0 if self.gear == 1 else 1.0

    def safety_active(self):
        return self.estop == 1 or self.aeb == 1

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

    def handle_to_wheel_steer_deg(self, handle_deg):
        if abs(handle_deg) < 1e-6:
            return 0.0

        handle_abs = abs(handle_deg)
        wheel_abs = (
            self.steer_poly_a * handle_abs * handle_abs
            + self.steer_poly_b * handle_abs
            + self.steer_poly_c
        )
        wheel_abs = self.clamp(wheel_abs, 0.0, self.wheel_limit_deg)
        return -math.copysign(wheel_abs, handle_deg)

    def publish_target_rpm(self):
        if self.safety_active():
            target_rpm = 0.0
        elif self.mode == 0:
            target_rpm = self.last_pedal_rpm
        else:
            target_rpm = self.speed_kmh_to_rpm(self.last_cmd_speed_kmh)

        self.target_rpm_pub.publish(Float32(data=target_rpm * self.gear_sign()))

    def publish_target_steer(self):
        target_steer = 0.0
        if self.mode == 1:
            target_steer = self.wheel_to_handle_steer_deg(self.last_cmd_steer_deg)

        self.target_steer_pub.publish(Float32(data=target_steer))

    def pedal_voltage_cb(self, msg):
        self.last_pedal_rpm = self.pedal_voltage_to_rpm(msg.data)
        self.pedal_rpm_pub.publish(Float32(data=self.last_pedal_rpm))
        self.publish_target_rpm()

    def cmd_speed_cb(self, msg):
        self.last_cmd_speed_kmh = msg.data
        self.publish_target_rpm()

    def ctrl_rpm_cb(self, msg):
        self.current_speed_pub.publish(Float32(data=self.rpm_to_speed_kmh(msg.data)))

    def cmd_steer_cb(self, msg):
        self.last_cmd_steer_deg = msg.data
        self.publish_target_steer()

    def ctrl_str_cb(self, msg):
        wheel_deg = round(self.handle_to_wheel_steer_deg(msg.data), 2)
        self.current_str_pub.publish(Float32(data=wheel_deg))
        self.viz_steering_pub.publish(Float32(data=wheel_deg))

    def mode_cb(self, msg):
        self.mode = self.as_binary(msg.data)
        self.publish_target_rpm()
        self.publish_target_steer()

    def gear_cb(self, msg):
        self.gear = self.as_binary(msg.data)
        self.publish_target_rpm()

    def estop_cb(self, msg):
        self.estop = self.as_binary(msg.data)
        self.publish_target_rpm()

    def aeb_cb(self, msg):
        self.aeb = self.as_binary(msg.data)
        self.publish_target_rpm()


def main(args=None):
    rclpy.init(args=args)
    node = TopicMapper()

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
