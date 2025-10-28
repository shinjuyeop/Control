"""
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
"""

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from std_srvs.srv import Trigger
from throttle_msgs.msg import ThrottleData

class RpmMux(Node):
    def __init__(self):
        super().__init__('rpm_mux')

        # 파라미터
        self.green = bool(self.declare_parameter('start_green', True).value)   # True면 자동 경로 허용
        self.zero_on_block = bool(self.declare_parameter('zero_on_block', False).value)

        # 상태
        self.manual_rpm = 0
        self.auto_rpm = 0
        self.estop = False
        self.is_manual = False

        # 구독 (핸들 보관)
        self.sub_throttle = self.create_subscription(
            ThrottleData, 'throttle_data', self.cb_throttle, 10
        )
        self.sub_auto = self.create_subscription(
            Float32, '/cmd/rpm', self.cb_auto, 10
        )

        # 발행 (핸들 보관)
        self.pub = self.create_publisher(Int32, 'target_rpm', 10)

        # # 서비스 (/green) (핸들 보관)
        # self.srv_green = self.create_service(
        #     Trigger, '/green', self.on_green
        # )

        # 100 Hz 주기 발행 안전망 (핸들 보관)
        self.timer = self.create_timer(0.01, self.tick)

        self.get_logger().info(f'start_green={self.green}, zero_on_block={self.zero_on_block}')

    # 서비스 콜백: green 플래그 활성화
    def on_green(self, req, res):
        self.green = True
        res.success = True
        res.message = 'green=True'
        self.get_logger().info('/green 호출 → 자동 경로 허용')
        return res

    # ───────── 콜백 ─────────
    def cb_throttle(self, msg: ThrottleData):
        self.manual_rpm = int(msg.rpm)
        self.estop = bool(getattr(msg, 'estop', False))
        self.is_manual = bool(getattr(msg, 'asms', False))
        self.publish_now()

    def cb_auto(self, msg: Float32):
        self.auto_rpm = int(msg.data)
        if not self.is_manual and not self.estop and self.green:
            self.publish_now()

    # ───────── 주기 실행 ─────────
    def tick(self):
        self.publish_now()

    def publish_now(self):
        if self.estop:
            rpm_out = 0
        elif self.is_manual:
            rpm_out = self.manual_rpm
        else:
            if not self.green:
                if self.zero_on_block:
                    rpm_out = 0
                else:
                    return  # 자동 차단 시 발행 안 함
            else:
                rpm_out = self.auto_rpm
        self.pub.publish(Int32(data=int(rpm_out)))


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(RpmMux())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

