"""Launch-time RAM calibration request and MCU readback."""
import secrets
import time

from kaiev26_msgs.msg import DriveActuatorStatus
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32MultiArray

from .longitudinal import (CONFIG_TOPIC, STATUS_TOPIC, RESULTS, decode_status,
                           request_data, wire_equal)


class LongitudinalClient:
    def __init__(self, node):
        self.node = node
        self.status = self.drive = None
        self.operation = self.result = None
        self.publisher = node.create_publisher(Float32MultiArray, CONFIG_TOPIC, 1)
        node.create_subscription(Float32MultiArray, STATUS_TOPIC, self.receive, qos_profile_sensor_data)
        node.create_subscription(DriveActuatorStatus, '/drive/status', self.receive_drive, qos_profile_sensor_data)

    def receive(self, msg):
        try:
            self.status = (time.monotonic(), decode_status(msg.data))
        except ValueError:
            self.status = None

    def receive_drive(self, msg):
        self.drive = (time.monotonic(), msg)

    def fresh(self):
        return self.status[1] if self.status and time.monotonic() - self.status[0] < .7 else None

    def reason(self):
        if self.operation:
            return 'STM_A 적용 결과 확인 중'
        if not self.fresh():
            return 'STM_A longitudinal readback 대기 / 오래된 상태'
        if not self.drive or time.monotonic() - self.drive[0] > .5:
            return '차량 상태 수신 대기 / 오래된 상태'
        d = self.drive[1]
        if d.mode != 0 or not d.part1_valid or not d.part2_valid:
            return 'MANUAL 및 유효한 Part I/II가 필요합니다.'
        if not (abs(d.speed_mps) <= .05 and d.pedal_percent == 0 and
                abs(d.commanded_phase_current_a) < .1 and abs(d.phase_current_a) <= 1):
            return '차량 정지·페달 해제·명령/실제 무전류 확인이 필요합니다.'
        endpoints = self.node.get_publishers_info_by_topic(STATUS_TOPIC)
        if len(endpoints) != 1 or endpoints[0].node_name not in ('stm_a_node', 'stm_a_emulator'):
            return 'STM_A longitudinal status 발행자가 정확히 하나여야 합니다.'
        return ''

    def begin(self, config, expected_revision):
        reason = self.reason()
        if reason:
            raise ValueError(reason)
        if self.fresh()['revision'] != expected_revision:
            raise ValueError('검토 이후 STM_A calibration이 변경되었습니다.')
        request_id = secrets.randbelow(16777215) + 1
        if request_id == self.fresh()['request_id']:
            request_id = request_id % 16777215 + 1
        data = request_data(config, request_id, expected_revision, self.fresh()['version'])
        self.operation = dict(config=dict(config), id=request_id, revision=expected_revision,
                              started=time.monotonic())
        self.result = None
        self.publisher.publish(Float32MultiArray(data=data))

    def tick(self):
        op = self.operation
        if not op:
            return
        status = self.fresh()
        if status and status['request_id'] == op['id']:
            success = (status['result'] == 1 and status['revision'] == op['revision'] + 1 and
                       status['source'] == 'HOST_OVERRIDE' and wire_equal(status['config'], op['config']))
            self.result = (success, 'STM_A 적용·readback 완료 (RAM, first-test candidate)' if success
                           else 'STM_A 적용 실패: ' + RESULTS[status['result']])
        elif time.monotonic() - op['started'] > 3:
            self.result = (False, 'STM_A readback 시간 초과: 적용 여부 미확인. 자동 재전송하지 않습니다.')
        if self.result:
            self.operation = None


def main(args=None):
    """Launch-selected preset: one attempt at a fresh safe stop, then exit."""
    import rclpy
    from rclpy.node import Node
    from .longitudinal import CONFIG_KEYS, DEFAULT
    rclpy.init(args=args)
    node = Node('longitudinal_calibration')
    client = LongitudinalClient(node)
    deadline = time.monotonic() + 15
    attempted = False
    try:
        config = {key: node.declare_parameter(key, DEFAULT[key]).value for key in CONFIG_KEYS}
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            client.tick()
            if client.result:
                success, message = client.result
                (node.get_logger().info if success else node.get_logger().error)(message)
                return 0 if success else 1
            if not attempted and not client.reason():
                client.begin(config, client.fresh()['revision'])
                attempted = True
        node.get_logger().error('프리셋 미적용/미확인: ' + (client.reason() or 'readback 대기') +
                                '; 현재 STM_A readback과 정지 상태를 확인하고 선택한 프리셋으로 구동 스택을 재시작하세요.')
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
