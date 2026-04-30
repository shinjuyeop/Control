#!/usr/bin/env python3
"""vcan CAN 메시지 → ROS2 /cmd_vel 변환 노드"""

import socket
import struct
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# can_io.py에서 가져온 상수들
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

ID_MOTOR_STAT1 = 0x1801D0EF
ID_STEER_FB    = 0x07000001
MOTOR_RPM_OFFSET = 32000

# 튜닝값 — 나중에 실차 맞춰서 조정
MAX_RPM = 3000.0          # 최대 RPM
MAX_LINEAR_MPS = 2.0      # 최대 선속도 (m/s)
WHEEL_RADIUS = 0.05       # 바퀴 반지름 (m) — 실차 맞춰 수정
MAX_HANDLE_DEG = 300.0    # 핸들 최대각
MAX_STEER_RAD = 0.5       # Gazebo 최대 조향각 (rad)


class CanToCmdVel(Node):
    def __init__(self):
        super().__init__('can_to_cmdvel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.current_rpm = 0.0
        self.current_steer = 0.0

        # 10Hz로 /cmd_vel 퍼블리시
        self.timer = self.create_timer(0.1, self.publish_cmdvel)

        # vcan 수신 스레드
        threading.Thread(target=self.can_rx_loop, args=('vcan0',), daemon=True).start()
        threading.Thread(target=self.can_rx_loop, args=('vcan1',), daemon=True).start()

        self.get_logger().info('CAN → cmd_vel 노드 시작!')

    def can_rx_loop(self, iface: str):
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((iface,))
        sock.settimeout(0.1)

        while rclpy.ok():
            try:
                packet = sock.recv(CAN_FRAME_SIZE)
            except TimeoutError:
                continue
            except OSError:
                break

            if len(packet) < CAN_FRAME_SIZE:
                continue

            raw_id, dlc, raw_data = struct.unpack(CAN_FRAME_FMT, packet)
            extended = bool(raw_id & CAN_EFF_FLAG)
            can_id = raw_id & (CAN_EFF_MASK if extended else 0x7FF)
            data = bytes(raw_data[:dlc])

            # 모터 RPM 파싱 (STAT1)
            if can_id == ID_MOTOR_STAT1 and len(data) >= 8:
                rpm_raw = int.from_bytes(data[6:8], 'little')
                self.current_rpm = float(rpm_raw - MOTOR_RPM_OFFSET)
                self.get_logger().debug(f'RPM: {self.current_rpm:.1f}')

            # 조향각 파싱 (STEER_FB)
            elif can_id == ID_STEER_FB and len(data) >= 2:
                handle_deg = int.from_bytes(data[0:2], 'big', signed=True)
                self.current_steer = float(handle_deg)
                self.get_logger().debug(f'Steer: {self.current_steer:.1f} deg')

    def publish_cmdvel(self):
        msg = Twist()

        # RPM → 선속도 (m/s)
        import math
        angular_wheel = (self.current_rpm / 60.0) * 2.0 * math.pi
        msg.linear.x = min((self.current_rpm / MAX_RPM) * MAX_LINEAR_MPS, MAX_LINEAR_MPS)

        # 핸들각 → 조향각 (rad)
        msg.angular.z = (self.current_steer / MAX_HANDLE_DEG) * MAX_STEER_RAD

        self.pub.publish(msg)
        self.get_logger().info(
            f'cmd_vel → linear.x={msg.linear.x:.3f} m/s, angular.z={msg.angular.z:.3f} rad'
        )


def main():
    rclpy.init()
    node = CanToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()