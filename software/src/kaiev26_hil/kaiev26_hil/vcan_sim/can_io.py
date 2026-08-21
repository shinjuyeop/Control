"""vcan SIL communication layer.

이 모듈은 세 가지를 맡는다.
1. SocketCAN 송수신
2. raw CAN frame <-> 내부 명령/상태 변환
3. HMI와 runtime이 재사용하는 frame 생성 helper 제공
"""

from __future__ import annotations

import logging
import struct
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, Optional, Tuple

from .steering_calibration import count_to_motor_deg, equivalent_deg_to_count

# Linux raw CAN 소켓에서 쓰는 확장 CAN 플래그/마스크들이다.
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

# 현재 lab에서 쓰는 실제 CAN ID 정의다.
ID_MOTOR_CMD = 0x0C01EFD0
ID_MOTOR_STAT1 = 0x1801D0EF
ID_MOTOR_STAT2 = 0x1802D0EF
ID_STEER_CMD = 0x06000001
ID_STEER_FB = 0x07000001

# 모터 command의 제어 비트다.
CTRL_RUN = 1 << 0
CTRL_SPEED = 1 << 1

# 실차 프로토콜에서 쓰는 raw offset을 그대로 유지한다.
MOTOR_RPM_OFFSET = 32000
MOTOR_CURRENT_OFFSET_A = -3200.0
# Keep the stable logger name so existing log filters continue to match.
LOGGER = logging.getLogger("can_io")


def _warning(message: str, *args) -> None:
    """Emit with the legacy name through the process's current root logger."""
    record = LOGGER.makeRecord(
        LOGGER.name, logging.WARNING, __file__, 0, message, args, None
    )
    logging.getLogger().handle(record)


@dataclass
class CanFrame:
    """버스에 흘리는 단일 CAN frame 표현."""

    iface: str
    can_id: int
    data: bytes
    extended: bool = True
    timestamp: float = 0.0
    source: str = "bus"

    def __post_init__(self) -> None:
        """timestamp 기본값과 classic CAN 길이 제약을 맞춘다."""
        self.timestamp = self.timestamp or time.time()
        if len(self.data) > 8:
            raise ValueError("Classic CAN payload must be <= 8 bytes")

    @property
    def hex_id(self) -> str:
        return f"{self.can_id:08X}" if self.extended else f"{self.can_id:03X}"

    @property
    def hex_data(self) -> str:
        return self.data.hex().upper()

    def pretty(self) -> str:
        ts = time.strftime("%H:%M:%S", time.localtime(self.timestamp))
        return f"[{ts}] {self.iface:<5} {self.hex_id}#{self.hex_data:<16} ({self.source})"


@dataclass(frozen=True)
class BusHealthSnapshot:
    """버스 계층에서 누적한 기본 진단 카운터."""

    sent_frames: int = 0
    received_frames: int = 0
    dispatched_frames: int = 0
    callback_errors: int = 0
    socket_errors: int = 0


class BusHub:
    """송수신 버스 구현이 따라야 하는 최소 인터페이스."""

    def subscribe(self, iface: str, callback: Callable[[CanFrame], None]) -> None:
        raise NotImplementedError

    def send(self, frame: CanFrame) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError


class SocketCANBusHub(BusHub):
    """Linux SocketCAN 위에 직접 붙는 버스 허브."""

    def __init__(self, ifaces: Iterable[str]) -> None:
        import socket

        self._lock = threading.Lock()
        self._running = True
        self._subs: Dict[str, List[Callable[[CanFrame], None]]] = {iface: [] for iface in ifaces}
        self._socks: Dict[str, socket.socket] = {}
        self._threads: List[threading.Thread] = []
        self._sent_frames = 0
        self._received_frames = 0
        self._dispatched_frames = 0
        self._callback_errors = 0
        self._socket_errors = 0
        self._last_callback_error_log_monotonic = 0.0
        self._last_socket_error_log_monotonic = 0.0

        if not hasattr(socket, "AF_CAN"):
            raise RuntimeError("This Python build does not support AF_CAN")

        for iface in ifaces:
            # 인터페이스마다 독립 소켓을 두고 수신 thread를 붙인다.
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            try:
                sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 0)
            except OSError:
                pass
            sock.bind((iface,))
            sock.settimeout(0.1)
            self._socks[iface] = sock
            thread = threading.Thread(target=self._rx_loop, args=(iface, sock), daemon=True)
            thread.start()
            self._threads.append(thread)

    def subscribe(self, iface: str, callback: Callable[[CanFrame], None]) -> None:
        """특정 CAN 인터페이스로 들어온 frame을 구독한다."""
        with self._lock:
            self._subs.setdefault(iface, []).append(callback)

    def send(self, frame: CanFrame) -> None:
        """raw CAN packet으로 내보내고, 로컬 구독자에도 즉시 fan-out 한다."""
        sock = self._socks[frame.iface]
        can_id = frame.can_id & (CAN_EFF_MASK if frame.extended else CAN_SFF_MASK)
        if frame.extended:
            can_id |= CAN_EFF_FLAG
        payload = frame.data.ljust(8, b"\x00")
        packet = struct.pack(CAN_FRAME_FMT, can_id, len(frame.data), payload)
        sock.send(packet)
        self._sent_frames += 1
        # 같은 프로세스 안의 runtime/HMI도 바로 frame을 볼 수 있게 로컬 dispatch 한다.
        self._dispatch(
            CanFrame(
                iface=frame.iface,
                can_id=frame.can_id,
                data=bytes(frame.data),
                extended=frame.extended,
                timestamp=time.time(),
                source=frame.source,
            )
        )

    def _dispatch(self, frame: CanFrame) -> None:
        """구독자 목록을 복사해서 lock 밖에서 안전하게 callback을 돈다."""
        with self._lock:
            callbacks = list(self._subs.get(frame.iface, []))
        self._dispatched_frames = getattr(self, "_dispatched_frames", 0) + 1
        for callback in callbacks:
            try:
                callback(frame)
            except Exception as exc:
                self._callback_errors = getattr(self, "_callback_errors", 0) + 1
                now = time.monotonic()
                last_log = getattr(self, "_last_callback_error_log_monotonic", 0.0)
                if now - last_log >= 2.0:
                    _warning(
                        "SocketCAN subscriber callback raised; callback_errors=%d: %s",
                        self._callback_errors,
                        exc,
                    )
                    self._last_callback_error_log_monotonic = now

    def _rx_loop(self, iface: str, sock) -> None:
        """소켓에서 들어온 raw packet을 읽어 CanFrame으로 복원한다."""
        while self._running:
            try:
                packet = sock.recv(CAN_FRAME_SIZE)
            except TimeoutError:
                continue
            except OSError as exc:
                self._socket_errors += 1
                now = time.monotonic()
                if now - self._last_socket_error_log_monotonic >= 2.0 and self._running:
                    _warning(
                        "SocketCAN recv failed on %s; socket_errors=%d: %s",
                        iface,
                        self._socket_errors,
                        exc,
                    )
                    self._last_socket_error_log_monotonic = now
                if self._running:
                    time.sleep(0.05)
                continue
            if len(packet) < CAN_FRAME_SIZE:
                continue
            raw_id, dlc, raw_data = struct.unpack(CAN_FRAME_FMT, packet)
            extended = bool(raw_id & CAN_EFF_FLAG)
            can_id = raw_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK)
            self._received_frames += 1
            self._dispatch(
                CanFrame(
                    iface=iface,
                    can_id=can_id,
                    data=bytes(raw_data[:dlc]),
                    extended=extended,
                    timestamp=time.time(),
                    source="bus",
                )
            )

    def close(self) -> None:
        """수신 loop를 멈추고 모든 CAN 소켓을 닫는다."""
        self._running = False
        for sock in self._socks.values():
            try:
                sock.close()
            except OSError:
                pass
        for thread in self._threads:
            thread.join(timeout=0.2)

    def health_snapshot(self) -> BusHealthSnapshot:
        """HMI와 runtime이 읽을 버스 상태 스냅샷을 만든다."""
        return BusHealthSnapshot(
            sent_frames=getattr(self, "_sent_frames", 0),
            received_frames=getattr(self, "_received_frames", 0),
            dispatched_frames=getattr(self, "_dispatched_frames", 0),
            callback_errors=getattr(self, "_callback_errors", 0),
            socket_errors=getattr(self, "_socket_errors", 0),
        )


@dataclass(frozen=True)
class MotorCommand:
    """모터 command frame을 해석한 내부 표현."""

    handshake_ack: bool
    target_rpm: float = 0.0
    phase_limit_a: float = 0.0
    ctrl_run: bool = False
    ctrl_speed_mode: bool = False
    life: int = 0


@dataclass(frozen=True)
class SteeringCommand:
    """조향 command frame을 해석한 내부 표현."""

    enable: Optional[bool] = None
    target_handle_deg: Optional[float] = None


def parse_motor_command(frame: CanFrame) -> Optional[MotorCommand]:
    """모터 CAN command를 내부 명령 객체로 바꾼다."""
    if frame.can_id != ID_MOTOR_CMD:
        return None
    data = frame.data
    if len(data) == 8 and data == b"\xAA" * 8:
        return MotorCommand(handshake_ack=True)
    if len(data) < 8:
        return None
    current_raw = int.from_bytes(data[0:2], byteorder="little", signed=False)
    speed_raw = int.from_bytes(data[2:4], byteorder="little", signed=False)
    ctrl = data[4]
    return MotorCommand(
        handshake_ack=False,
        target_rpm=float(speed_raw - MOTOR_RPM_OFFSET),
        phase_limit_a=current_raw * 0.1 + MOTOR_CURRENT_OFFSET_A,
        ctrl_run=bool(ctrl & CTRL_RUN),
        ctrl_speed_mode=bool(ctrl & CTRL_SPEED),
        life=data[7],
    )


def parse_steering_command(frame: CanFrame) -> Optional[SteeringCommand]:
    """조향 CAN command를 enable 또는 목표각 명령으로 해석한다."""
    if frame.can_id != ID_STEER_CMD or len(frame.data) < 8:
        return None
    data = frame.data
    if data[:4] == bytes([0x23, 0x0D, 0x20, 0x01]):
        return SteeringCommand(enable=(data[4] == 0x00))
    if data[:4] == bytes([0x23, 0x02, 0x20, 0x01]):
        scaled = int.from_bytes(data[4:8], byteorder="big", signed=True)
        return SteeringCommand(target_handle_deg=(scaled / 10000.0) * 360.0)
    return None


def encode_motor_handshake_frame(iface: str) -> CanFrame:
    """모터 handshake 대기 상태에서 주기적으로 내보내는 0x55 frame."""
    return CanFrame(iface, ID_MOTOR_STAT1, b"\x55" * 8, True, source="motor-sim")


def encode_motor_status_frames(
    iface: str,
    bus_voltage_v: float,
    bus_current_a: float,
    phase_current_a: float,
    actual_rpm: float,
    ctrl_run: bool,
    ctrl_speed_mode: bool,
    ctrl_temp_c: float,
    motor_temp_c: float,
    error_group: int,
) -> Tuple[CanFrame, CanFrame]:
    """현재 모터 상태를 status1/status2 두 frame으로 인코딩한다."""
    vdc_raw = max(0, min(0xFFFF, round(bus_voltage_v * 10.0)))
    ibus_raw = _encode_motor_current_a(bus_current_a)
    iph_raw = _encode_motor_current_a(phase_current_a)
    rpm_raw = _encode_motor_rpm(actual_rpm)

    status1 = (
        int(vdc_raw).to_bytes(2, "little")
        + int(ibus_raw).to_bytes(2, "little")
        + int(iph_raw).to_bytes(2, "little")
        + int(rpm_raw).to_bytes(2, "little")
    )

    status = 0
    if ctrl_run:
        status |= 0x01
    if ctrl_speed_mode:
        status |= 0x02

    status2 = bytes(
        [
            max(0, min(255, round(ctrl_temp_c + 40.0))),
            max(0, min(255, round(motor_temp_c + 40.0))),
            status,
            error_group & 0xFF,
            (error_group >> 8) & 0xFF,
            (error_group >> 16) & 0xFF,
            0,
            0,
        ]
    )
    return (
        CanFrame(iface, ID_MOTOR_STAT1, status1, True, source="motor-sim"),
        CanFrame(iface, ID_MOTOR_STAT2, status2, True, source="motor-sim"),
    )


def encode_steer_feedback_frame(iface: str, handle_deg: float) -> CanFrame:
    """현재 핸들 각도를 조향 피드백 frame으로 바꾼다."""
    counts = int(round(handle_deg))
    counts = max(-32768, min(32767, counts))
    data = counts.to_bytes(2, byteorder="big", signed=True) + b"\x00" * 6
    return CanFrame(iface, ID_STEER_FB, data, True, source="steer-sim")


def make_motor_cmd_frame(iface: str, target_rpm: int, current_limit_a: float = 500.0, run: bool = True) -> CanFrame:
    """HMI나 테스트에서 쓰는 모터 목표 RPM command frame을 만든다."""
    current_raw = max(0, min(0xFFFF, round((current_limit_a + 3200.0) / 0.1)))
    speed_raw = max(0, min(0xFFFF, round(target_rpm + MOTOR_RPM_OFFSET)))
    ctrl = 0
    if run:
        ctrl |= CTRL_RUN | CTRL_SPEED
    payload = (
        int(current_raw).to_bytes(2, "little")
        + int(speed_raw).to_bytes(2, "little")
        + bytes([ctrl, 0x00, 0x00, 0x00])
    )
    return CanFrame(iface, ID_MOTOR_CMD, payload, True, source="dashboard")


def make_motor_handshake_ack(iface: str) -> CanFrame:
    """모터 handshake 완료용 ACK frame."""
    return CanFrame(iface, ID_MOTOR_CMD, b"\xAA" * 8, True, source="dashboard")


def wheel_to_handle_deg_for_cmd(wheel_deg: float) -> float:
    """중앙 등가 조향각을 KEYA 모터축 명령각으로 바꾼다."""
    return count_to_motor_deg(equivalent_deg_to_count(wheel_deg))


def make_steer_enable_frame(iface: str, enable: bool = True) -> CanFrame:
    """조향 enable/disable SDO frame."""
    data = bytes([0x23, 0x0D, 0x20, 0x01, 0x00 if enable else 0x01, 0, 0, 0])
    return CanFrame(iface, ID_STEER_CMD, data, True, source="dashboard")


def make_steer_pos_frame_from_wheel_deg(iface: str, wheel_deg: float) -> CanFrame:
    """바퀴각 기준 목표를 조향 position write frame으로 만든다."""
    handle_deg = wheel_to_handle_deg_for_cmd(wheel_deg)
    scaled = round((handle_deg / 360.0) * 10000.0)
    data = bytes([0x23, 0x02, 0x20, 0x01]) + int(scaled).to_bytes(4, "big", signed=True)
    return CanFrame(iface, ID_STEER_CMD, data, True, source="dashboard")


def _encode_motor_current_a(value_a: float) -> int:
    """전류 실수값을 프로토콜 raw 정수로 바꾼다."""
    raw = round((value_a - MOTOR_CURRENT_OFFSET_A) / 0.1)
    return max(0, min(0xFFFF, raw))


def _encode_motor_rpm(value_rpm: float) -> int:
    """RPM 실수값을 프로토콜 raw 정수로 바꾼다."""
    raw = round(value_rpm + MOTOR_RPM_OFFSET)
    return max(0, min(0xFFFF, raw))
