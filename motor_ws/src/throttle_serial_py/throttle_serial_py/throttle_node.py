import serial, rclpy
from rclpy.node import Node
from throttle_msgs.msg import ThrottleData

HDR      = 0xAA          # 패킷 헤더
RPM_MAX  = 2000.0
STEP_RPM = RPM_MAX / 15  # 33 rpm 단계

class ThrottleSerialNode(Node):
    def __init__(self):
        super().__init__('throttle_serial_node')
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
        self.pub = self.create_publisher(ThrottleData, 'throttle_data', 10)
        self.create_timer(0.01, self.read_once)   # 100 Hz

    def read_once(self):
        if self.ser.read(1) != bytes([HDR]):      # 헤더 대기
            return
        data = self.ser.read(1)
        if len(data) != 1:
            return

        b       = data[0]
        rpm4    = (b >> 4) & 0x0F
        sign    = (b >> 3) & 0x01
        estop   = bool(b & 0x04)
        asms    = bool(b & 0x02)

        rpm_mag = rpm4 * STEP_RPM
        rpm     = -rpm_mag if sign else rpm_mag

        msg = ThrottleData(rpm=int(rpm), estop=estop, asms=asms)
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
