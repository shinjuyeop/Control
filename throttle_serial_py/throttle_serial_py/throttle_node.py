#!/usr/bin/env python3
# ROS2 Humble / Python 3.x
# - 포트 자동탐지(기본) + 수동 지정 가능(ROS 파라미터 port)
# - 한 번 연 뒤 충분히 대기(boot_wait)해서 부트로더 종료를 기다린 다음 헤더를 확인
# - 헤더가 바로 없어도 함부로 닫지 않음. 재연결 주기를 길게 해서 반복 리셋 방지
# - 값 변화가 있을 때만 터미널 로그 업데이트

import time
from glob import glob

import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from throttle_msgs.msg import ThrottleData
from std_msgs.msg import Int32

HDR_RX   = 0xAA          # 패킷 헤더
HDR_TX   = 0xAB
RPM_MAX  = 2000.0
STEP_RPM = RPM_MAX / 15  # 33 rpm 단계 (현재 미사용)


class ThrottleSerialNode(Node):
    def __init__(self):
        super().__init__('throttle_serial_node')

        # ── 파라미터
        self.declare_parameter('port', 'auto')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('boot_wait', 2.0)
        self.declare_parameter('probe_window', 3.0)
        self.declare_parameter('read_timeout', 0.05)
        self.declare_parameter('aeb_topic', '/aeb')

        self.pub = self.create_publisher(ThrottleData, 'throttle_data', 10)
        self.ser = None
        self._last_connect_attempt = 0.0
        self._connect_interval = 6.0   # 재연결 최소 간격(s)

        # 변화 감지를 위한 직전 상태 저장
        self._last_rpm = None
        self._last_estop = None
        self._last_manual = None
        self._last_rev = None

        # AEB (긴급제동) 입력 상태
        self._aeb = 0               # 0/1 (정수)
        self._last_sent_aeb = None  # 직전 전송값(연결 재수립 시 동기화 목적)

        # /aeb 구독 (std_msgs/Int32, 0 또는 1 기대)
        aeb_topic = self.get_parameter('aeb_topic').value
        self.create_subscription(Int32, aeb_topic, self._cb_aeb, 10)

        # 100 Hz 주기 읽기
        self.create_timer(0.01, self.read_once)

    # ─────────────────────────────────────────────────────────────
    def _open_serial(self):
        port_param   = self.get_parameter('port').value
        baud         = int(self.get_parameter('baud').value)
        boot_wait    = float(self.get_parameter('boot_wait').value)
        probe_window = float(self.get_parameter('probe_window').value)
        read_timeout = float(self.get_parameter('read_timeout').value)

        # 명시 포트면 바로 오픈
        if port_param and port_param != 'auto':
            try:
                self.get_logger().info(f"Opening serial (explicit): {port_param} @ {baud}")
                ser = serial.Serial(port_param, baudrate=baud, timeout=read_timeout)
                time.sleep(boot_wait)
                ser.reset_input_buffer()
                self.get_logger().info(f"Connected (explicit): {port_param}")
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open explicit port {port_param}: {e}")
                return None

        # 자동 탐색
        candidates = []
        candidates.extend(sorted(glob('/dev/serial/by-id/*')))

        ports = list(serial.tools.list_ports.comports())
        def prio(p):
            desc = (p.description or '').lower()
            score = 0
            if 'arduino' in desc: score += 4
            if 'mega'    in desc: score += 2
            if 'ch340' in desc or 'wch' in desc: score += 3
            if 'cp210'   in desc: score += 2
            if 'ttyacm'  in p.device.lower(): score += 2
            if 'ttyusb'  in p.device.lower(): score += 1
            return -score
        for p in sorted(ports, key=prio):
            candidates.append(p.device)

        # 중복 제거
        seen, unique = set(), []
        for c in candidates:
            if c not in seen:
                unique.append(c); seen.add(c)

        for dev in unique:
            try:
                self.get_logger().info(f"Probing: {dev}")
                ser = serial.Serial(dev, baudrate=baud, timeout=read_timeout)
                time.sleep(boot_wait)
                ser.reset_input_buffer()

                if self._probe_stream(ser, probe_window):
                    self.get_logger().info(f"Connected (auto): {dev}")
                    return ser
                time.sleep(0.2)
                ser.close()
            except serial.SerialException:
                continue

        return None

    def _probe_stream(self, ser, probe_window):
        """열린 포트에서 0xAA 헤더가 들어오는지 probe_window 동안 확인."""
        t0 = time.time()
        while time.time() - t0 < probe_window:
            b = ser.read(1)
            if b:
                if b[0] == HDR_RX:
                    ser.read(3)  # 첫 패킷 정렬
                    return True
            else:
                time.sleep(0.005)
        return False

    # ─────────────────────────────────────────────────────────────
    def _cb_aeb(self, msg: Int32):
        """/aeb (0/1) 수신 시 아두이노로 전달 패킷 전송.
          - STATE: 0=해제, 1=활성
        """
        val = 1 if msg.data else 0
        if val != self._aeb:
            self._aeb = val
            self.get_logger().info(f"/aeb updated → {val}")
            self._send_aeb_if_possible()

    def _send_aeb_if_possible(self):
        """직전 값과 달라졌거나(엣지), 연결 재수립 시 현재 상태 전송."""
        if self.ser is None or not getattr(self.ser, 'is_open', False):
            return
        if self._last_sent_aeb == self._aeb:
            return
        try:
            pkt = bytes([HDR_TX, 0x00, 0x00, 0x01 if self._aeb else 0x00])
            self.ser.write(pkt)
            self.ser.flush()
            self._last_sent_aeb = self._aeb
            self.get_logger().info(f"Sent AEB to Arduino: {self._aeb}")
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Failed to send AEB: {e}")

    # ─────────────────────────────────────────────────────────────
    def read_once(self):
        # 미연결이면 재연결 시도
        if self.ser is None or not self.ser.is_open:
            now = time.time()
            if now - self._last_connect_attempt >= self._connect_interval:
                self._last_connect_attempt = now
                self.ser = self._open_serial()
                # 새 연결 → 직전 전송 상태 초기화 후 동기화 패킷 전송
                self._last_sent_aeb = None
                # 연결 직후 현재 AEB 상태 동기화 시도
                self._send_aeb_if_possible()
            return

        # 연결 상태
        try:
            hdr = self.ser.read(1)
            if not hdr or hdr[0] != HDR_RX:
                return

            packet = self.ser.read(3)
            if len(packet) != 3:
                return

            hi, lo, flags = packet

            rpm16    = (hi << 8) | lo
            reverse  = bool((flags >> 2) & 0x01)  # bit2
            estop_hw = bool((flags >> 1) & 0x01)  # bit1 ESTOP from MEGA
            manual   = bool(flags & 0x01)         # bit0

            rpm = -int(rpm16) if reverse else int(rpm16)
            if -2 < rpm < 2:
                rpm = 0

            effective_estop = estop_hw or (self._aeb == 1)

            # 변화 감지 후 로그
            if (rpm != self._last_rpm or
                effective_estop != self._last_estop or
                manual != self._last_manual or
                reverse != self._last_rev):
                
                self.get_logger().info(
                    f"RPM={rpm}, estop={effective_estop}, manual={manual}, reverse={reverse}"
                )
                self._last_rpm = rpm
                self._last_estop = effective_estop
                self._last_manual = manual
                self._last_rev = reverse

            # 토픽 퍼블리시
            msg = ThrottleData(rpm=rpm, estop=effective_estop, asms=manual)
            self.pub.publish(msg)

        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial error: {e}. Will try to reconnect…")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # 다음 루프에서 재연결
            self._last_sent_aeb = None  # 재연결 시 다시 전송되도록 초기화


def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
