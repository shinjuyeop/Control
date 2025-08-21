#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from throttle_msgs.msg import ThrottleData


class RpmMux(Node):
    def __init__(self):
        super().__init__('rpm_mux')

        # 상태 변수
        self.manual_rpm = 0          # 수동 페달 rpm
        self.auto_rpm   = 0          # 자율 스택 rpm
        self.estop      = False
        self.is_manual  = False      # True = MANUAL, False = AUTO

        # 구독
        self.create_subscription(
            ThrottleData, 'throttle_data', self.cb_throttle, 10)
        self.create_subscription(
            Int32, 'target_rpm_auto', self.cb_auto, 10)

        # 발행
        self.pub = self.create_publisher(Int32, 'target_rpm', 10)

        # 10 ms 주기(100 Hz)
        self.create_timer(0.01, self.tick)

    # ───────── 콜백 ─────────
    def cb_throttle(self, msg: ThrottleData):
        """Arduino → 페달 rpm·모드·ESTOP"""
        self.manual_rpm = msg.rpm
        self.estop      = msg.estop
        self.is_manual  = msg.asms        # bit1==1 → True → MANUAL

    def cb_auto(self, msg: Int32):
        """자율 스택 목표 rpm"""
        self.auto_rpm = msg.data

    # ───────── 주기 실행 ─────────
    def tick(self):
        rpm_out = 0
        if not self.estop:  # ESTOP 시 무조건 0
            rpm_out = self.manual_rpm if self.is_manual else self.auto_rpm
        self.pub.publish(Int32(data=int(rpm_out)))

    # (no per-change logging)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RpmMux())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
