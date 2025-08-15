"""
import serial, rclpy
from rclpy.node import Node
from throttle_msgs.msg import ThrottleData

HDR      = 0xAA          # 패킷 헤더
RPM_MAX  = 2000.0
STEP_RPM = RPM_MAX / 15  # 33 rpm 단계

class ThrottleSerialNode(Node):
    def __init__(self):
        super().__init__('throttle_serial_node')
        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=0.05)
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
"""
#!/usr/bin/env python3
# ROS2 Humble / Python 3.x
# - 포트 자동탐지(기본) + 수동 지정 가능(ROS 파라미터 port)
# - 한 번 연 뒤 충분히 대기(boot_wait)해서 부트로더 종료를 기다린 다음 헤더를 확인
# - 헤더가 바로 없어도 함부로 닫지 않음. 재연결 주기를 길게 해서 반복 리셋 방지

import time
from glob import glob

import serial
import serial.tools.list_ports

import rclpy
from rclpy.node import Node
from throttle_msgs.msg import ThrottleData

HDR      = 0xAA          # 패킷 헤더
RPM_MAX  = 2000.0
STEP_RPM = RPM_MAX / 15  # 33 rpm 단계 (현재 미사용)

class ThrottleSerialNode(Node):
    def __init__(self):
        super().__init__('throttle_serial_node')

        # ── 파라미터
        self.declare_parameter('port', 'auto')        # 'auto' 또는 '/dev/ttyUSB0' 또는 '/dev/serial/by-id/...'
        self.declare_parameter('baud', 115200)
        self.declare_parameter('boot_wait', 2.0)      # 포트 오픈 직후 부트로더 대기(s)
        self.declare_parameter('probe_window', 3.0)   # 헤더 탐지 총 대기(s)
        self.declare_parameter('read_timeout', 0.05)  # 평소 read timeout(s)

        self.pub = self.create_publisher(ThrottleData, 'throttle_data', 10)
        self.ser = None
        self._last_connect_attempt = 0.0
        self._connect_interval = 6.0   # 재연결 최소 간격(s) — 너무 잦은 오픈/클로즈로 인한 리셋 방지

        # 100 Hz 주기 읽기
        self.create_timer(0.01, self.read_once)

    # ─────────────────────────────────────────────────────────────
    def _open_serial(self):
        port_param   = self.get_parameter('port').value
        baud         = int(self.get_parameter('baud').value)
        boot_wait    = float(self.get_parameter('boot_wait').value)
        probe_window = float(self.get_parameter('probe_window').value)
        read_timeout = float(self.get_parameter('read_timeout').value)

        # 명시 포트면 신뢰하고 바로 오픈(프로브 생략 → 불필요한 재리셋 방지)
        if port_param and port_param != 'auto':
            try:
                self.get_logger().info(f"Opening serial (explicit): {port_param} @ {baud}")
                ser = serial.Serial(port_param, baudrate=baud, timeout=read_timeout)
                # 부트 대기
                time.sleep(boot_wait)
                ser.reset_input_buffer()
                self.get_logger().info(f"Connected (explicit): {port_param}")
                return ser
            except serial.SerialException as e:
                self.get_logger().warn(f"Failed to open explicit port {port_param}: {e}")
                return None

        # 자동 탐색: /dev/serial/by-id/* 우선 → ttyACM*/ttyUSB*
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
                time.sleep(boot_wait)      # 오픈 직후 부트로더 종료 대기(리셋 1회만)
                ser.reset_input_buffer()

                if self._probe_stream(ser, probe_window):
                    self.get_logger().info(f"Connected (auto): {dev}")
                    return ser
                # 프로브 실패해도 즉시 닫지 않고 살짝 유예 후 닫기(불필요한 토글 줄이기)
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
                if b[0] == HDR:
                    # 첫 패킷의 나머지 3바이트 소비(정렬)
                    ser.read(3)
                    return True
            else:
                # 타임아웃 → 잠깐 쉼(바쁜 루프 방지)
                time.sleep(0.005)
        return False

    # ─────────────────────────────────────────────────────────────
    def read_once(self):
        # 미연결이면 주기적으로 재연결 시도(너무 잦지 않게)
        if self.ser is None or not self.ser.is_open:
            now = time.time()
            if now - self._last_connect_attempt >= self._connect_interval:
                self._last_connect_attempt = now
                self.ser = self._open_serial()
            return

        # 연결 상태: 일반 읽기
        try:
            hdr = self.ser.read(1)
            if not hdr or hdr[0] != HDR:
                return

            packet = self.ser.read(3)  # RPM_H, RPM_L, FLAGS
            if len(packet) != 3:
                return

            hi, lo, flags = packet
            print(f"flags={flags:03b}, manual={bool(flags & 0x01)}")

            rpm16   = (hi << 8) | lo
            reverse = bool((flags >> 2) & 0x01)  # bit2
            estop   = bool((flags >> 1) & 0x01)  # bit1
            manual  = bool(flags & 0x01)         # bit0

            rpm = -int(rpm16) if reverse else int(rpm16)
            msg = ThrottleData(rpm=rpm, estop=estop, asms=manual)
            self.pub.publish(msg)

        except (serial.SerialException, OSError) as e:
            self.get_logger().warn(f"Serial error: {e}. Will try to reconnect…")
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None  # 다음 루프에서 재연결

def main(args=None):
    rclpy.init(args=args)
    node = ThrottleSerialNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
