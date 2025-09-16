#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from throttle_msgs.msg import ThrottleData


class RpmMux(Node):
    def __init__(self):
        super().__init__('rpm_mux')

        # 상태 변수
        self.manual_rpm = 0          # 수동 페달 rpm (signed)
        self.auto_rpm   = 0          # 자율 스택 rpm (int, signed 가능)
        self.estop      = False
        self.is_manual  = False      # True = MANUAL, False = AUTO

        # 구독
        self.create_subscription(ThrottleData, 'throttle_data', self.cb_throttle, 10)
        self.create_subscription(Float32, '/cmd/rpm', self.cb_auto, 10)

        # 발행
        self.pub = self.create_publisher(Int32, 'target_rpm', 10)

        # 10 ms 주기(100 Hz)
        self.create_timer(0.01, self.tick)

    # ───────── 콜백 ─────────
    def cb_throttle(self, msg: ThrottleData):
        # Arduino → 페달 rpm·모드·ESTOP
        self.manual_rpm = int(msg.rpm)
        self.estop = bool(msg.estop)
        self.is_manual = bool(msg.asms)
        self.publish_now()

    def cb_auto(self, msg: Float32):
        # 자율 스택 목표 rpm
        self.auto_rpm = int(msg.data)
        if not self.is_manual and not self.estop:
            self.publish_now()

    # ───────── 주기 실행 ─────────
    def tick(self):
        # 안전망: 주기적으로도 발행
        self.publish_now()

    def publish_now(self):
        rpm_out = 0 if self.estop else (self.manual_rpm if self.is_manual else self.auto_rpm)
        self.pub.publish(Int32(data=int(rpm_out)))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RpmMux())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
