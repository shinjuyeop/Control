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
        hdr = self.ser.read(1)
        if not hdr or hdr[0] != HDR:
            return
        
        packet = self.ser.read(3)       # RPM_H, RPM_L, FLAGS
        if len(packet) != 3:
            return
        
        hi, lo, flags = packet
        
        #16-bit RPM 복원
        rpm16 = (hi << 8) | lo

        reverse = bool((flags >> 2) & 0x01)        # bit 2 = 후진
        estop   = bool((flags >> 1) & 0x01)       # bit 1 = E-STOP
        asms    = bool(flags & 0x01)              # bit 0 = MANUAL 

        rpm = -int(rpm16) if reverse else int(rpm16)

        msg = ThrottleData(
            rpm = rpm,
            estop = estop,
            asms = asms
        )
        self.pub.publlish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
