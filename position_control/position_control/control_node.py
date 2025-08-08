import can
import time
import rclpy
from rclpy.node import Node

class CANPositionControl(Node):
    def __init__(self):
        super().__init__('can_position_control')

        # CAN 인터페이스 초기화
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=250000)

        # 모터 Enable
        self.enable_motor()

        # 목표 위치로 이동
        self.set_position(0)  # 예: 1.8바퀴 (648도)

        # 현재 위치 읽기
        self.get_encoder_position()

    def enable_motor(self):
        msg = can.Message(arbitration_id=0x06000001, data=[0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00], is_extended_id=True)
        self.bus.send(msg)
        self.get_logger().info("Motor Enabled")

    def set_position(self, degrees):
        position_value = int((degrees * 10000) / 360)
        position_bytes = position_value.to_bytes(4, byteorder='big', signed=True)

        msg = can.Message(arbitration_id=0x06000001, data=[0x23, 0x02, 0x20, 0x01] + list(position_bytes), is_extended_id=True)
        self.bus.send(msg)
        self.get_logger().info(f"Set Position: {degrees}° ({position_value})")

    def get_encoder_position(self):
        msg = can.Message(arbitration_id=0x06000001, data=[0x40, 0x04, 0x21, 0x01, 0x00, 0x00, 0x00, 0x00], is_extended_id=True)
        self.bus.send(msg)

        # 응답 수신
        response = self.bus.recv(timeout=1.0)
        if response:
            encoder_position = int.from_bytes(response.data[4:], byteorder='big', signed=True)
            self.get_logger().info(f"Encoder Position: {encoder_position}")
        else:
            self.get_logger().warn("No response from motor")

def main(args=None):
    rclpy.init(args=args)
    node = CANPositionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
   