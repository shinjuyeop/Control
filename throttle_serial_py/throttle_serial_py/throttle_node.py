import rclpy
from rclpy.node import Node
from throttle_msgs.msg import ThrottleData

import serial
from serial import SerialException
from serial.tools import list_ports

HDR      = 0xAA          # 패킷 헤더
RPM_MAX  = 2000.0
STEP_RPM = RPM_MAX / 15  # 33 rpm 단계

class ThrottleSerialNode(Node):
    def __init__(self):
        super().__init__('throttle_serial_node')

        # params (원하면 launch에서 -p로 주입)
        self.declare_parameter('baud', 115200)
        self.declare_parameter('vid', 0)   # 0 = 무시
        self.declare_parameter('pid', 0)   # 0 = 무시

        self.baud = int(self.get_parameter('baud').value)
        self.vid  = int(self.get_parameter('vid').value)
        self.pid  = int(self.get_parameter('pid').value)

        self.ser = None
        self.pub = self.create_publisher(ThrottleData, 'throttle_data', 10)
        self.create_timer(0.01, self.read_once)   # 100 Hz

    # ------- 연결/스캔 -------
    def open_serial_if_needed(self):
        if self.ser and self.ser.is_open:
            return True

        ports = list_ports.comports()

        # (옵션) VID/PID 지정 시 필터링
        if self.vid or self.pid:
            ports = [
                p for p in ports
                if (not self.vid or p.vid == self.vid) and (not self.pid or p.pid == self.pid)
            ]

        # 우선순위: ACM* -> USB*
        acms = [p.device for p in ports if 'ACM' in p.device]
        usbs = [p.device for p in ports if 'USB' in p.device]
        candidates = acms + usbs

        for dev in candidates:
            try:
                s = serial.Serial(dev, self.baud, timeout=0.05)
                s.reset_input_buffer()
                if self.probe_stream(s):
                    self.ser = s
                    self.get_logger().info(f'Connected to {dev} @ {self.baud}')
                    return True
                s.close()
            except SerialException as e:
                self.get_logger().warn(f'Open failed {dev}: {e}')
        return False

    def probe_stream(self, s: serial.Serial, max_bytes: int = 64) -> bool:
        """0xAA 헤더가 들어오는지 짧게 확인"""
        try:
            buf = s.read(max_bytes)
            return HDR in buf
        except SerialException:
            return False

    def read_once(self):
        try:
            if not self.open_serial_if_needed():
                return
            b = self.ser.read(1)
            if not b or b[0] != HDR:
                return
            pkt = self.ser.read(3)      # [RPM_H][RPM_L][FLAGS]

            if len(pkt) != 3:
                return
            
            hi, lo, flags = pkt
            rpm16   = (hi << 8) | lo
            reverse = bool((flags >> 2) & 0x01)
            estop   = bool((flags >> 1) & 0x01)
            manual  = bool(flags & 0x01)

            rpm = -int(rpm16) if reverse else int(rpm16)

            self.get_logger().info(f'RPM = {rpm:4d}, flags={flags:03b} (M = {manual}, E = {estop}, R = {reverse})')

            msg = ThrottleData(rpm = rpm, estop = estop, asms = manual)
            self.pub.publish(msg)
        except (SerialException, OSError) as e:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            self.get_logger().warn(f'Serial error, will retry: {e}')
        hdr = self.ser.read(1)
        if not hdr or hdr[0] != HDR:
            return
        
        packet = self.ser.read(3)       # RPM_H, RPM_L, FLAGS
        if len(packet) != 3:
            return
        
        hi, lo, flags = packet
        
        print(f"flags={flags:03b}, manual={bool(flags & 0x01)}")

        #16-bit RPM 복원
        rpm16 = (hi << 8) | lo

        reverse = bool((flags >> 2) & 0x01)       # bit 2 = 후진
        estop   = bool((flags >> 1) & 0x01)       # bit 1 = E-STOP
        manual  = bool(flags & 0x01)              # bit 0 = MANUAL 

        rpm = -int(rpm16) if reverse else int(rpm16)

        msg = ThrottleData(
            rpm = rpm,
            estop = estop,
            asms = manual
        )
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
