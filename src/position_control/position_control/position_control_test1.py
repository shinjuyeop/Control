import can
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32  # ✅ 현재 조향 각도 퍼블리시용
import time  # ✅ 시간 지연을 위해 추가

class CANSteeringControl(Node):
    def __init__(self):
        super().__init__('can_steering_control')

        # ✅ ROS2 퍼블리셔 (현재 각도를 퍼블리시)
        self.encoder_publisher = self.create_publisher(Float32, 'encoder_angle', 10)

        # ✅ CAN 인터페이스 초기화
        self.bus = can.Bus(interface='socketcan', channel='can0', bitrate=250000)

        # ✅ 모터 Enable
        self.enable_motor()

        # ✅ 조향 목표값 리스트 (사용자가 직접 입력)
        self.target_positions = [
            0, 0,
            -30, -40, -20, -10, 0, 10, 40, 30, 20, 15, #0에서 9
            5, 10, 5, 0, 20, 35, 30, 20, 0, 10, #10에서 19
            10, 0, 10, 10, 0, 20, 30, 20, 30, 0, #20에서 29
            4, 20, 0, 30, 20, 0, 4, 20, 10, -2, #30에서 39
            -20, 10, 4, 0, 4, 6, -4, 0, -4, 0, #40에서 49
            40, 4, 10, 0, 4, 20, 10, 30, 20, 40, #50에서 59
            20, 10, 4, 0, 0, 10, 20, 40, 20, 4, #60에서 69
            4, 10, 14, 16, 40, 30, 20, 30, 20, 10, #70에서 79
            -10, -80, -70, -40, -20, -10, 10, 30, 20, -4, #80에서 89
            -30, -60, -80, -40, -10, 2, 10, 50, 70, 40, #90에서 99
            10, 0, 10, 0, 20, 80, 60, 40, 0, 20, #100에서 109
            14, 10, 4, 0, 10, 0, 20, 10, 6, 20, #110에서 119
            20, 10, -20 #120에서 끝
        ]
        
        # ✅ 목표값 적용 실행
        self.run_position_sequence()

    def enable_motor(self):
        """ 모터 활성화 """
        msg = can.Message(arbitration_id=0x06000001, data=[0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00], is_extended_id=True)
        self.bus.send(msg)
        self.get_logger().info("Motor Enabled")

    def set_position(self, degrees):
        """ 목표 조향 각도를 모터로 전송 """
        position_value = int((degrees * 10000) / 360)  # ✅ 360도 = 10000 단위 변환
        position_bytes = position_value.to_bytes(4, byteorder='big', signed=True)  # ✅ Big Endian 유지

        msg = can.Message(arbitration_id=0x06000001, data=[0x23, 0x02, 0x20, 0x01] + list(position_bytes), is_extended_id=True)
        self.bus.send(msg)
        self.get_logger().info(f"Set Steering Angle: {degrees:.2f}° ({position_value})")

    def get_heartbeat_angle(self):
        """ Heartbeat 메시지를 통해 현재 조향 각도를 가져옴 """
        response = self.bus.recv(timeout=1.0)

        if response is None:
            self.get_logger().warn("No response from motor (CAN timeout)")
            return

        if response.arbitration_id != 0x07000001:
            self.get_logger().warn(f"Ignoring unexpected CAN ID: {hex(response.arbitration_id)}")
            return

        try:
            # ✅ Heartbeat 메시지에서 엔코더 값 추출 (6~7바이트 사용, Big Endian 변환)
            encoder_value = int.from_bytes(response.data[6:8], byteorder='big', signed=True)

            # ✅ 엔코더 값을 각도로 변환 (10000 = 360도 기준)
            current_angle = (encoder_value / 10000) * 360.0

            self.get_logger().info(f"Current Encoder Angle: {current_angle:.2f}°")

            # ✅ ROS2 토픽으로 퍼블리시
            self.encoder_publisher.publish(Float32(data=current_angle))

        except Exception as e:
            self.get_logger().error(f"Failed to parse Heartbeat data: {e}")

    def run_position_sequence(self):
        """ 목표 조향 각도를 리스트 순서대로 실행하는 함수 """
        while rclpy.ok():
            for target_position in self.target_positions:
                self.set_position(target_position)

                # ✅ 목표 위치를 설정한 후 일정 시간 대기
                time.sleep(1.0)  # 필요에 따라 조정 가능

                # ✅ 현재 위치 확인 (Heartbeat 메시지 수신)
                self.get_heartbeat_angle()

def main(args=None):
    rclpy.init(args=args)
    node = CANSteeringControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
