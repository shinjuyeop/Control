#!/usr/bin/env python3
"""ROS-free Rich TUI for the XIAO RP2350 LV battery gauge."""

from __future__ import annotations

import glob
import math
import errno
import os
import queue
import secrets
import select
import struct
import sys
import termios
import threading
import time
import tty
import zlib
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Deque, Optional

try:
    from rich.console import Console, Group
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
except ImportError as exc:  # pragma: no cover - exercised by the CLI environment.
    print(f"Rich is required: {exc}", file=sys.stderr)
    raise SystemExit(2)


PROTOCOL_VERSION = 1
SOF = b"\xA5\x5A"
HEADER = struct.Struct("<2sBBHI")
STATUS = struct.Struct("<I4B6H3I7iqI5Q6I")
COMMAND = struct.Struct("<IB3si")
ACK = struct.Struct("<IBBHIi")
MAX_PAYLOAD = max(STATUS.size, COMMAND.size, ACK.size)

MESSAGE_STATUS = 0x01
MESSAGE_COMMAND = 0x10
MESSAGE_ACK = 0x11

COMMAND_SET_FULL = 0x01
COMMAND_RESET_TRIP = 0x02
COMMAND_ZERO_CURRENT = 0x03
COMMAND_SET_CAPACITY = 0x04

RESULT_NAMES = {
    0x00: "성공",
    0x01: "잘못된 명령",
    0x02: "저장 실패",
    0x03: "센서 사용 불가",
    0x04: "장치 사용 중",
    0x05: "요청 ID 충돌",
}

STATUS_SENSOR_IDENTIFIED = 1 << 0
STATUS_MEASUREMENT_VALID = 1 << 1
STATUS_SOC_KNOWN = 1 << 2
STATUS_STORAGE_MOUNTED = 1 << 3
STATUS_ZERO_CALIBRATING = 1 << 4
STATUS_STORAGE_DIRTY = 1 << 5
STATUS_COMMAND_BUSY = 1 << 6

SENSOR_NAMES = {
    0: "INA228 미검출",
    1: "INA228 준비됨",
    2: "INA228 읽기 오류",
    3: "INA228 설정 중",
}

AVG_COUNTS = (1, 4, 16, 64, 128, 256, 512, 1024)
DEVICE_GLOB = "/dev/serial/by-id/usb-Seeed_XIAO_RP2350_*-if00"
PREFERRED_DEVICE = (
    "/dev/serial/by-id/usb-Seeed_XIAO_RP2350_A89369E2B2DEE7A4-if00"
)


def frame(message_type: int, sequence: int, payload: bytes) -> bytes:
    header = HEADER.pack(SOF, PROTOCOL_VERSION, message_type, len(payload), sequence)
    crc = zlib.crc32(header[2:] + payload) & 0xFFFFFFFF
    return header + payload + struct.pack("<I", crc)


class FrameParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, int, bytes]]:
        self.buffer.extend(data)
        frames: list[tuple[int, int, bytes]] = []
        while len(self.buffer) >= HEADER.size:
            start = self.buffer.find(SOF)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == SOF[:1] else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < HEADER.size:
                break
            _, version, message_type, payload_size, sequence = HEADER.unpack(
                self.buffer[: HEADER.size]
            )
            if payload_size > MAX_PAYLOAD:
                del self.buffer[0]
                continue
            total_size = HEADER.size + payload_size + 4
            if len(self.buffer) < total_size:
                break
            received_crc = struct.unpack_from(
                "<I", self.buffer, HEADER.size + payload_size
            )[0]
            expected_crc = zlib.crc32(self.buffer[2 : HEADER.size + payload_size]) & 0xFFFFFFFF
            if version != PROTOCOL_VERSION or received_crc != expected_crc:
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[HEADER.size : HEADER.size + payload_size])
            del self.buffer[:total_size]
            frames.append((message_type, sequence, payload))
        return frames


def configure_serial(fd: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIFLUSH)


@dataclass
class PendingCommand:
    request_id: int
    command: int
    value: int
    encoded: bytes
    last_sent: float = 0.0


@dataclass(frozen=True)
class PlotPoint:
    received_at: float
    voltage_v: Optional[float]
    current_a: Optional[float]
    power_w: Optional[float]


class GaugeClient:
    """Own the USB file descriptor and reconnect without retaining stale data."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._fd: Optional[int] = None
        self._path: Optional[str] = None
        self._note = "USB 장치 검색 중"
        self._parser = FrameParser()
        self._status: Optional[dict[str, int]] = None
        self._status_received = 0.0
        self._acks: queue.Queue[dict[str, int]] = queue.Queue()
        self._status_events: queue.Queue[PlotPoint] = queue.Queue()
        self._pending: Optional[PendingCommand] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="lv-gauge-usb")
        self._thread.start()

    def close(self) -> None:
        self._stop.set()
        self._close_fd("USB 연결 종료")
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _candidate_path(self) -> Optional[str]:
        paths = sorted(glob.glob(DEVICE_GLOB))
        if PREFERRED_DEVICE in paths:
            return PREFERRED_DEVICE
        if len(paths) == 1:
            return paths[0]
        if len(paths) > 1:
            self._note = "같은 모델 USB 장치가 여러 개라 연결하지 않음"
            return None
        self._note = "XIAO RP2350 USB 장치 대기 중"
        return None

    def _open_path(self, path: str) -> None:
        fd: Optional[int] = None
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
            configure_serial(fd)
        except (OSError, termios.error) as exc:
            self._note = f"USB 열기 실패: {exc}"
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
            return
        with self._lock:
            self._fd = fd
            self._path = path
            self._note = "USB 연결됨, XIAO 상태 대기 중"
            self._parser = FrameParser()
            self._status = None
            self._status_received = 0.0

    def _close_fd(self, note: str) -> None:
        with self._lock:
            fd, self._fd = self._fd, None
            self._path = None
            self._note = note
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                fd = self._fd
            if fd is None:
                path = self._candidate_path()
                if path is not None:
                    self._open_path(path)
                self._stop.wait(0.5)
                continue

            try:
                readable, _, _ = select.select([fd], [], [], 0.2)
                if readable:
                    data = os.read(fd, 4096)
                    if not data:
                        self._close_fd("USB 연결 끊김")
                        continue
                    for message_type, sequence, payload in self._parser.feed(data):
                        self._handle_frame(message_type, sequence, payload)
            except OSError as exc:
                if exc.errno not in (errno.EAGAIN, errno.EWOULDBLOCK):
                    self._close_fd(f"USB 읽기 오류: {exc}")

            with self._lock:
                pending = self._pending
            if pending is not None and time.monotonic() - pending.last_sent >= 1.0:
                self._write_pending(pending)

    def _handle_frame(self, message_type: int, sequence: int, payload: bytes) -> None:
        del sequence
        if message_type == MESSAGE_STATUS and len(payload) == STATUS.size:
            fields = STATUS.unpack(payload)
            keys = (
                "uptime_ms",
                "flags",
                "sensor_state",
                "ina_address",
                "reserved",
                "manufacturer_id",
                "device_id",
                "config_register",
                "adc_config_register",
                "shunt_cal_register",
                "diag_alert",
                "vbus_raw24",
                "vshunt_raw24",
                "current_raw24",
                "bus_voltage_uV",
                "shunt_voltage_nV",
                "current_uA",
                "power_mW",
                "current_offset_uA",
                "die_temperature_mC",
                "soc_centi_percent",
                "remaining_uAs",
                "capacity_mAh",
                "charge_trip_uAs",
                "discharge_trip_uAs",
                "charge_trip_mWs",
                "discharge_trip_mWs",
                "active_time_ms",
                "storage_sequence",
                "i2c_error_count",
                "storage_error_count",
                "sample_sequence",
                "last_valid_age_ms",
                "accumulator_fault_count",
            )
            received_at = time.monotonic()
            status = dict(zip(keys, fields))
            with self._lock:
                self._status = status
                self._status_received = received_at
            if status["flags"] & STATUS_MEASUREMENT_VALID:
                point = PlotPoint(
                    received_at,
                    status["bus_voltage_uV"] / 1_000_000,
                    status["current_uA"] / 1_000_000,
                    status["power_mW"] / 1000,
                )
            else:
                point = PlotPoint(received_at, None, None, None)
            self._status_events.put(point)
        elif message_type == MESSAGE_ACK and len(payload) == ACK.size:
            request_id, command, result, _, storage_sequence, detail = ACK.unpack(payload)
            ack = {
                "request_id": request_id,
                "command": command,
                "result": result,
                "storage_sequence": storage_sequence,
                "detail": detail,
            }
            with self._lock:
                if self._pending is not None and self._pending.request_id == request_id:
                    self._pending = None
            self._acks.put(ack)

    def _write_pending(self, pending: PendingCommand) -> None:
        with self._lock:
            fd = self._fd
        if fd is None:
            return
        try:
            os.write(fd, pending.encoded)
            with self._lock:
                if self._pending is not None and self._pending.request_id == pending.request_id:
                    self._pending.last_sent = time.monotonic()
        except OSError as exc:
            self._close_fd(f"USB 쓰기 오류: {exc}")

    def send_command(self, command: int, value: int = 0) -> int:
        with self._lock:
            if self._pending is not None:
                raise RuntimeError("명령 처리 중")
        request_id = secrets.randbits(32) or 1
        payload = COMMAND.pack(request_id, command, b"\x00\x00\x00", value)
        pending = PendingCommand(request_id, command, value, frame(MESSAGE_COMMAND, request_id, payload))
        with self._lock:
            self._pending = pending
        self._write_pending(pending)
        return request_id

    def poll_acks(self) -> list[dict[str, int]]:
        result: list[dict[str, int]] = []
        while True:
            try:
                result.append(self._acks.get_nowait())
            except queue.Empty:
                return result

    def poll_status_events(self) -> list[PlotPoint]:
        result: list[PlotPoint] = []
        while True:
            try:
                result.append(self._status_events.get_nowait())
            except queue.Empty:
                return result

    def view(self) -> tuple[Optional[dict[str, int]], bool, str, Optional[PendingCommand]]:
        with self._lock:
            status = dict(self._status) if self._status is not None else None
            fresh = self._fd is not None and time.monotonic() - self._status_received <= 2.0
            return status if fresh else None, fresh, self._note, self._pending


def fmt_time(milliseconds: int) -> str:
    total = milliseconds // 1000
    hours, remainder = divmod(total, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds:02d}"


def fmt_trip_uas(value: int) -> str:
    return f"{value / 3_600_000_000:.3f} Ah"


def fmt_trip_mws(value: int) -> str:
    return f"{value / 3_600_000:.3f} Wh"


def _plot_panel(
    history: Deque[PlotPoint],
    now: float,
    width: int,
    height: int,
    title: str,
    unit: str,
    attribute: str,
    include_zero: bool,
    border_style: str,
) -> Panel:
    window_start = now - 60.0
    points = [point for point in history if point.received_at >= window_start]
    values = [getattr(point, attribute) for point in points]
    values = [value for value in values if value is not None]
    if not values:
        return Panel(
            Text("최근 60초 수신 대기", style="dim"),
            title=f"{title} [{unit}]", width=width, border_style=border_style,
        )

    label_width = 7
    chart_width = max(8, width - 4 - label_width - 1)
    chart_height = max(4, height)

    # One mean per terminal column keeps a dense time series a single trace.
    # A segment ends at a missing sample; columns on either side stay apart.
    columns = {}
    segment = 0
    for point in points:
        value = getattr(point, attribute)
        if value is None:
            segment += 1
            continue
        x = min(chart_width - 1, max(0, int(
            (point.received_at - window_start) / 60.0 * chart_width)))
        total, count, old_segment = columns.get(x, (0.0, 0, segment))
        if old_segment != segment:
            total, count = 0.0, 0
        columns[x] = (total + value, count + 1, segment)

    means = [total / count for total, count, _ in columns.values()]
    low, high = min(means), max(means)
    if include_zero:
        low, high = min(0.0, low), max(0.0, high)
    padding = max((high - low) * 0.05, 0.01)
    low, high = low - padding, high + padding
    target_step = (high - low) / (chart_height - 2)
    magnitude = 10 ** math.floor(math.log10(target_step))
    step = next(factor * magnitude for factor in (1, 2, 5, 10)
                if factor * magnitude >= target_step)
    low = math.floor(low / step) * step
    high = low + step * (chart_height - 1)

    def map_y(value: float) -> int:
        return round((high - value) / step)

    tick_rows = set(range(0, chart_height, max(1, (chart_height - 1) // 4)))
    tick_rows.add(chart_height - 1)
    zero_row = map_y(0.0) if include_zero and low <= 0 <= high else None
    if zero_row is not None:
        tick_rows.add(zero_row)
    decimals = max(0, -math.floor(math.log10(step)))
    labels = {}
    for y in tick_rows:
        value = high - y * step
        if abs(value) < step * 1e-6:
            value = 0.0
        labels[y] = f"{value:.{decimals}f}" if decimals <= 3 else f"{value:.3g}"

    # Connectivity bits: left=1, right=2, up=4, down=8.
    canvas = [[0] * chart_width for _ in range(chart_height)]
    previous = None
    for x, (total, count, segment) in sorted(columns.items()):
        y = map_y(total / count)
        if previous is not None and previous[2] == segment and x == previous[0] + 1:
            px, py, _ = previous
            canvas[py][px] |= 2
            canvas[py][x] |= 1
            direction = 1 if y > py else -1
            while py != y:
                ny = py + direction
                canvas[py][x] |= 8 if direction > 0 else 4
                canvas[ny][x] |= 4 if direction > 0 else 8
                py = ny
        else:
            canvas[y][x] |= 2
        previous = (x, y, segment)

    # Horizontal endpoints share the horizontal glyph.
    glyphs = {0: " ", 1: "─", 2: "─", 3: "─", 4: "│", 5: "╯",
              6: "╰", 7: "┴", 8: "│", 9: "╮", 10: "╭", 11: "┬",
              12: "│", 13: "┤", 14: "├", 15: "┼"}
    lines = []
    for y, row in enumerate(canvas):
        line = Text(f"{labels.get(y, ''):>{label_width}}", style="dim")
        line.append("┤" if y in labels else "│", style="dim")
        for mask in row:
            if mask:
                line.append(glyphs[mask], style=border_style)
            else:
                line.append("┄" if y == zero_row else " ", style="bright_black")
        lines.append(line)

    tick_fractions = (0, .25, .5, .75, 1) if chart_width >= 30 else (0, .5, 1)
    axis = ["─"] * chart_width
    time_labels = [" "] * chart_width
    for fraction in tick_fractions:
        x = round(fraction * (chart_width - 1))
        axis[x] = "┬"
        label = "now" if x == chart_width - 1 else f"−{round(60 * (1 - fraction))}s"
        start = max(0, min(chart_width - len(label), x - len(label) // 2))
        time_labels[start:start + len(label)] = label
    lines.append(Text(" " * label_width + "└" + "".join(axis), style="dim"))
    lines.append(Text(" " * (label_width + 1) + "".join(time_labels), style="dim"))
    return Panel(Group(*lines), title=f"{title} [{unit}] · 구간 평균",
                 width=width, border_style=border_style)


def plot_layout(
    history: Deque[PlotPoint], now: float, terminal_width: int, terminal_height: int
) -> tuple[object, str]:
    channels = (("전압", "V", "voltage_v", False, "cyan"),
                ("전류", "A", "current_a", True, "magenta"),
                ("전력", "W", "power_w", True, "yellow"))
    available_height = terminal_height - 17
    if terminal_width >= 150 and available_height >= 10:
        panel_width = (terminal_width - 2) // 3
        row = Table.grid(expand=True)
        for _ in channels:
            row.add_column(ratio=1)
        row.add_row(*(_plot_panel(history, now, panel_width, min(12, available_height - 4), *channel)
                      for channel in channels))
        return row, ""
    if terminal_width >= 60 and available_height >= 24:
        chart_height = min(10, available_height // 3 - 4)
        return Group(*(_plot_panel(history, now, terminal_width, chart_height, *channel)
                       for channel in channels)), ""
    if terminal_width >= 105 and available_height >= 8:
        panel_width = (terminal_width - 2) // 3
        row = Table.grid(expand=True)
        for _ in channels:
            row.add_column(ratio=1)
        row.add_row(*(_plot_panel(history, now, panel_width, available_height - 4, *channel)
                      for channel in channels))
        return row, ""
    return Text("플롯을 보려면 터미널 높이나 폭을 늘려주세요.", style="dim"), "플롯 숨김"


def command_name(command: int) -> str:
    return {
        COMMAND_SET_FULL: "완충 설정",
        COMMAND_RESET_TRIP: "트립 초기화",
        COMMAND_ZERO_CURRENT: "영점 보정",
        COMMAND_SET_CAPACITY: "용량 설정",
    }.get(command, f"명령 0x{command:02X}")


def basic_view(
    status: Optional[dict[str, int]],
    fresh: bool,
    note: str,
    pending: Optional[PendingCommand],
    message: str,
    history: Deque[PlotPoint],
    now: float,
    terminal_width: int,
    terminal_height: int,
) -> Group:
    connection = "연결됨 / 최신 상태" if fresh else f"연결 없음 / 수치 표시 안 함 ({note})"
    if status is None:
        measurement = Text("V / A / W: —\n센서 상태를 확인할 수 없습니다.", style="yellow")
        soc = Text("SOC: 미확인\n잔여 Ah: —", style="yellow")
        trips = Text("트립: —", style="yellow")
    else:
        if status["flags"] & STATUS_MEASUREMENT_VALID:
            current = status["current_uA"] / 1_000_000
            direction = "방전" if current > 0 else "충전" if current < 0 else "정지 전류"
            measurement = Text(
                f"{status['bus_voltage_uV'] / 1_000_000:.3f} V\n"
                f"{current:+.3f} A  ({direction})\n"
                f"{status['power_mW'] / 1000:+.3f} W",
                style="green",
            )
        else:
            measurement = Text("V / A / W: —\n센서 측정값 없음", style="yellow")
        if status["flags"] & STATUS_SOC_KNOWN:
            soc = Text(
                f"SOC: {status['soc_centi_percent'] / 100:.2f}%\n"
                f"잔여: {status['remaining_uAs'] / 3_600_000_000:.3f} Ah / "
                f"{status['capacity_mAh'] / 1000:.3f} Ah",
                style="green",
            )
        else:
            soc = Text("SOC: 미설정\n잔여 Ah: 미설정\n완충 설정 필요", style="yellow")
        trips = Text(
            f"충전: {fmt_trip_uas(status['charge_trip_uAs'])} / "
            f"{fmt_trip_mws(status['charge_trip_mWs'])}\n"
            f"방전: {fmt_trip_uas(status['discharge_trip_uAs'])} / "
            f"{fmt_trip_mws(status['discharge_trip_mWs'])}\n"
            f"계측 가동 누적: {fmt_time(status['active_time_ms'])}",
        )

    top = Table.grid(expand=True)
    top.add_column(ratio=1)
    top.add_column(ratio=1)
    top.add_column(ratio=1)
    top.add_row(
        Panel(measurement, title="측정 V / A / W · LPF 100ms"),
        Panel(soc, title="SOC / 잔여량"),
        Panel(trips, title="트립 충전 / 방전"),
    )
    plots, plot_note = plot_layout(history, now, terminal_width, terminal_height)
    state = "대기"
    if status is not None:
        storage = "저장 가능" if status["flags"] & STATUS_STORAGE_MOUNTED else "저장 불가"
        state = f"{SENSOR_NAMES.get(status['sensor_state'], '센서 상태 미상')} · {storage}"
        if status["flags"] & STATUS_ZERO_CALIBRATING:
            state += " · 영점 샘플 수집 중"
    controls = Text(
        "[f] 실제 완충 후 기준점  [t] 트립만 초기화  [z] 명시적 영점 보정  "
        "[c] 용량 설정  [d] 진단  [q] 종료\n"
        f"센서: {state}\n"
        + (f"처리 중: {command_name(pending.command)} (USB 재연결 시 같은 요청 재전송)\n" if pending else "")
        + (f"플롯: {plot_note}\n" if plot_note else "")
        + (message or "화면 수치는 최신 유효 상태만 표시합니다.")
    )
    return Group(
        Panel(Text(f"LV BATTERY GAUGE  ·  {connection}", style="bold cyan")),
        top,
        plots,
        Panel(controls, title="정비 조작"),
    )


def diagnostic_view(
    status: Optional[dict[str, int]], fresh: bool, note: str, message: str
) -> Group:
    if status is None:
        body = Text(f"진단 상태 없음: {note}\n연결이 복구되면 실제 payload를 표시합니다.", style="yellow")
        return Group(Panel(Text("INA228 DIAGNOSTICS", style="bold cyan")), Panel(body))

    config_range = "+/-40.96 mV" if status["config_register"] & (1 << 4) else "+/-163.84 mV"
    adc = status["adc_config_register"]
    mode = (adc >> 12) & 0xF
    vbus_ct = (adc >> 9) & 0x7
    vshunt_ct = (adc >> 6) & 0x7
    temp_ct = (adc >> 3) & 0x7
    avg = AVG_COUNTS[adc & 0x7]
    table = Table(expand=True)
    table.add_column("항목")
    table.add_column("실제 값")
    table.add_row("USB 상태", "최신" if fresh else "연결 없음")
    table.add_row("INA 주소", f"0x{status['ina_address']:02X}")
    table.add_row("제조사 ID", f"0x{status['manufacturer_id']:04X}")
    table.add_row("장치 ID", f"0x{status['device_id']:04X}")
    table.add_row("CONFIG", f"0x{status['config_register']:04X} · {config_range}")
    table.add_row(
        "ADC_CONFIG",
        f"0x{adc:04X} · mode=0x{mode:X}, VBUS={vbus_ct}, VSHUNT={vshunt_ct}, DIETEMP={temp_ct}, avg={avg}",
    )
    table.add_row("SHUNT_CAL", f"0x{status['shunt_cal_register']:04X}")
    table.add_row("진단 플래그", f"0x{status['diag_alert']:04X}")
    if status["flags"] & STATUS_MEASUREMENT_VALID:
        vbus_v = (status["vbus_raw24"] >> 4) * 195.3125e-6
        shunt_counts = status["vshunt_raw24"] >> 4
        if shunt_counts & (1 << 19):
            shunt_counts -= 1 << 20
        shunt_lsb_v = 78.125e-9 if status["config_register"] & (1 << 4) else 312.5e-9
        vdiff_v = shunt_counts * shunt_lsb_v
        table.add_row(
            "VBUS / VIN+ (동일 실측)",
            f"{vbus_v:.6f} V (공통 GND 기준)",
        )
        table.add_row(
            "VDIFF = VIN+ − VIN− (실측)",
            f"{vdiff_v * 1000:+.6f} mV",
        )
        table.add_row(
            "VIN− (VBUS−VDIFF 계산 추정)",
            f"{vbus_v - vdiff_v:.6f} V",
        )
        table.add_row("VBUS raw 24-bit", f"0x{status['vbus_raw24']:06X}")
        table.add_row("VSHUNT raw 24-bit", f"0x{status['vshunt_raw24']:06X}")
        table.add_row("CURRENT raw 24-bit", f"0x{status['current_raw24']:06X}")
    else:
        table.add_row("VBUS / VIN+ (동일 실측)", "— (유효 측정 없음)")
        table.add_row("VDIFF = VIN+ − VIN− (실측)", "— (유효 측정 없음)")
        table.add_row("VIN− (계산 추정)", "— (유효 측정 없음)")
        table.add_row("raw 24-bit", "— (유효 측정 없음)")
    if status["flags"] & STATUS_MEASUREMENT_VALID:
        table.add_row("VDIFF LPF (τ=100 ms)", f"{status['shunt_voltage_nV'] / 1_000_000:+.6f} mV")
        table.add_row("전류 LPF / 영점 보정", f"{status['current_uA'] / 1_000_000:+.6f} A / {status['current_offset_uA'] / 1_000_000:+.6f} A")
        table.add_row("INA die 온도", f"{status['die_temperature_mC'] / 1000:.3f} °C")
    else:
        table.add_row("VDIFF LPF (τ=100 ms)", "— (유효 측정 없음)")
        table.add_row("INA die 온도", "— (유효 측정 대기)")
    table.add_row("I2C 오류 누적", str(status["i2c_error_count"]))
    table.add_row("저장 오류 누적", str(status["storage_error_count"]))
    table.add_row("저장 sequence", str(status["storage_sequence"]))
    table.add_row("누산기 fault 누적", str(status["accumulator_fault_count"]))
    table.add_row("센서 상태", SENSOR_NAMES.get(status["sensor_state"], "알 수 없음"))
    footer = Text("[d] 기본 화면  [f/t/z/c] 정비 조작  [q] 종료\n" + (message or ""))
    return Group(Panel(Text("INA228 DIAGNOSTICS", style="bold cyan")), table, Panel(footer))


@contextmanager
def keyboard_cbreak():
    fd = sys.stdin.fileno()
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, original)
        except termios.error:
            pass


def confirmation_text(command: int, value: int) -> str:
    if command == COMMAND_SET_FULL:
        return "실제로 완충된 배터리인지 확인했습니까? 확인하면 SOC=100%/잔여=용량으로 저장합니다."
    if command == COMMAND_RESET_TRIP:
        return "트립 충전·방전 Ah/Wh와 계측 가동시간만 0으로 저장합니다. SOC/용량/영점은 유지합니다."
    if command == COMMAND_ZERO_CURRENT:
        return (
            "외부 부하와 충전기를 분리하고 션트 전류가 물리적으로 0 A인지 확인했습니까? "
            "화면의 0 A 근처 수치만으로 무전류를 증명할 수 없습니다. "
            "상시전원 소비가 흐르면 그 값을 영점으로 저장합니다. 장치의 CNVRF 완료주기로 "
            "64개 새 샘플을 수집합니다."
        )
    return (
        f"용량 {value / 1000:.3f} Ah를 저장합니다. SOC가 이미 알려져 있으면 잔여 Ah는 보존하고 "
        "새 용량을 분모로 사용합니다."
    )


def run() -> int:
    console = Console()
    if not sys.stdin.isatty():
        console.print("터미널에서 실행하세요.")
        return 2

    client = GaugeClient()
    client.start()
    screen = "basic"
    input_mode: Optional[str] = None
    input_text = ""
    action: Optional[tuple[int, int]] = None
    message = ""
    history: Deque[PlotPoint] = deque(maxlen=720)
    previous_fresh = False
    next_render = time.monotonic()
    try:
        with keyboard_cbreak(), Live(
            console=console, screen=True, refresh_per_second=10, auto_refresh=False
        ) as live:
            while True:
                for ack in client.poll_acks():
                    result = RESULT_NAMES.get(ack["result"], f"결과 0x{ack['result']:02X}")
                    if ack["result"] == 0:
                        message = f"XIAO 저장 완료: {command_name(ack['command'])} · sequence={ack['storage_sequence']}"
                    else:
                        message = f"XIAO 적용 실패: {command_name(ack['command'])} · {result}"

                history.extend(client.poll_status_events())
                status, fresh, note, pending = client.view()
                now = time.monotonic()
                if previous_fresh and not fresh:
                    history.append(PlotPoint(now, None, None, None))
                previous_fresh = fresh

                if now >= next_render:
                    if input_mode == "confirm":
                        message_for_view = confirmation_text(action[0], action[1]) if action else ""
                        message_for_view += "  [y] 확인  [n/Esc] 취소"
                    elif input_mode == "capacity":
                        message_for_view = f"새 용량(Ah): {input_text or ' '}  [Enter] 다음  [Esc] 취소"
                    else:
                        message_for_view = message
                    terminal_size = console.size
                    view = (
                        diagnostic_view(status, fresh, note, message_for_view)
                        if screen == "diagnostic"
                        else basic_view(
                            status,
                            fresh,
                            note,
                            pending,
                            message_for_view,
                            history,
                            now,
                            terminal_size.width,
                            terminal_size.height,
                        )
                    )
                    live.update(view, refresh=True)
                    next_render = now + 0.1

                wait = max(0.0, min(0.05, next_render - time.monotonic()))
                readable, _, _ = select.select([sys.stdin], [], [], wait)
                if not readable:
                    continue
                raw = os.read(sys.stdin.fileno(), 1)
                if not raw:
                    break
                char = raw.decode("utf-8", errors="ignore")

                if input_mode == "confirm":
                    if char.lower() == "y" and action is not None:
                        try:
                            client.send_command(*action)
                            message = "명령을 XIAO에 전송했습니다. 저장 완료 응답을 기다리는 중입니다."
                            input_mode = None
                            action = None
                        except RuntimeError as exc:
                            message = str(exc)
                            input_mode = None
                            action = None
                    elif char.lower() == "n" or char == "\x1b":
                        message = "정비 조작을 취소했습니다."
                        input_mode = None
                        action = None
                    continue

                if input_mode == "capacity":
                    if char == "\x1b":
                        input_mode = None
                        input_text = ""
                        message = "용량 입력을 취소했습니다."
                    elif char in "0123456789.":
                        input_text += char
                    elif char in ("\x7f", "\b"):
                        input_text = input_text[:-1]
                    elif char in ("\r", "\n"):
                        try:
                            capacity_mAh = round(float(input_text) * 1000)
                            if capacity_mAh <= 0 or capacity_mAh > 2_147_483_647:
                                raise ValueError("용량은 0보다 크고 전송 범위 안이어야 합니다.")
                            action = (COMMAND_SET_CAPACITY, capacity_mAh)
                            input_mode = "confirm"
                            input_text = ""
                        except ValueError as exc:
                            message = f"용량 입력 오류: {exc}"
                    continue

                if char.lower() == "q":
                    break
                if char.lower() == "d":
                    screen = "basic" if screen == "diagnostic" else "diagnostic"
                    continue
                if pending is not None:
                    message = "현재 명령의 XIAO 저장 응답을 기다리는 중입니다."
                    continue
                if char.lower() == "f":
                    action = (COMMAND_SET_FULL, 0)
                    input_mode = "confirm"
                elif char.lower() == "t":
                    action = (COMMAND_RESET_TRIP, 0)
                    input_mode = "confirm"
                elif char.lower() == "z":
                    action = (COMMAND_ZERO_CURRENT, 0)
                    input_mode = "confirm"
                elif char.lower() == "c":
                    input_mode = "capacity"
                    input_text = ""
    except KeyboardInterrupt:
        return 0
    finally:
        client.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
