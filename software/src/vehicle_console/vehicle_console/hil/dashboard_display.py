"""UNO-applied display state -> production C++ renderer -> Web PNG.

The serial worker alone owns the native renderer and port. Virtual touch sends
screen coordinates for dashboard navigation; images still come only from reports.
"""

from __future__ import annotations

import base64
import ctypes
from collections import deque
from io import BytesIO
import json
from pathlib import Path
import threading
import time

from PIL import Image
import serial

WIDTH, HEIGHT = 480, 320
VALUE_FIELDS = (
    ("status_flags", 0, 255),
    ("drive_flags", 0, 255),
    ("pedal_percent", 0, 100),
    ("gnss_state", 0, 3),
    ("speed_centi_mps", -32768, 32767),
    ("bus_voltage_decivolt", 0, 65535),
    ("bus_current_deciamp", -32768, 32767),
    ("phase_current_deciamp", -32768, 32767),
    ("controller_temperature_decic", -32768, 32767),
    ("motor_temperature_decic", -32768, 32767),
    ("motor_fault_bits", 0, 0xFFFFFF),
)


def decode_display_report(line: bytes):
    """Validate a single external UNO report; ignore human Serial Monitor lines."""
    if not line.startswith(b"HIL_LCD "):
        return None
    raw = json.loads(line[8:])
    if not isinstance(raw, dict) or raw.get("v") != 1:
        raise ValueError("지원하지 않는 UNO LCD report protocol")
    page = raw.get("page")
    if page not in ("MANUAL_TFT", "AUTO_BASELINE", "SWIPE_WAIT", "SWIPE_L", "SWIPE_R"):
        raise ValueError("지원하지 않는 UNO 화면")
    values = raw.get("values")
    if not isinstance(values, list) or len(values) < len(VALUE_FIELDS):
        raise ValueError("UNO 표시값이 불완전합니다")
    data = {}
    for (name, minimum, maximum), value in zip(VALUE_FIELDS, values):
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(f"UNO {name} 값이 범위를 벗어났습니다")
        data[name] = value
    if type(raw.get("can_lost")) is not bool:
        raise ValueError("UNO can_lost 값이 boolean이 아닙니다")
    for name, minimum in (("uptime_ms", 0), ("seq", 0), ("can_age_ms", -1)):
        if type(raw.get(name)) is not int or not minimum <= raw[name] <= 0xFFFFFFFF:
            raise ValueError(f"UNO {name} 값이 유효하지 않습니다")
    touch_errors = raw.get("touch_errors")
    if touch_errors is not None and (
        type(touch_errors) is not int or not 0 <= touch_errors <= 0xFFFFFFFF
    ):
        raise ValueError("UNO touch_errors 값이 유효하지 않습니다")
    return dict(
        page=page,
        can_lost=raw["can_lost"],
        can_age_ms=raw["can_age_ms"],
        uptime_ms=raw["uptime_ms"],
        seq=raw["seq"],
        values=data,
        vtouch=type(raw.get("vtouch")) is int and raw["vtouch"] == 1,
        touch_errors=touch_errors,
    )


class NativeDashboardRenderer:
    def __init__(self, library: str | Path):
        self.library = ctypes.CDLL(str(library))
        self.render_function = self.library.render_dashboard
        self.render_function.argtypes = [
            ctypes.POINTER(ctypes.c_int32),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self.render_function.restype = None
        self.swipe_render_function = self.library.render_swipe_test
        self.swipe_render_function.argtypes = [
            ctypes.c_int, ctypes.POINTER(ctypes.c_uint8)
        ]
        self.swipe_render_function.restype = None
        self.rgb = (ctypes.c_uint8 * (WIDTH * HEIGHT * 3))()

    def render(self, report):
        values = (ctypes.c_int32 * len(VALUE_FIELDS))(
            *(report["values"][name] for name, _, _ in VALUE_FIELDS)
        )
        if report["page"].startswith("SWIPE_"):
            direction = {"SWIPE_WAIT": 0, "SWIPE_L": 1, "SWIPE_R": 2}[report["page"]]
            self.swipe_render_function(direction, self.rgb)
        else:
            self.render_function(values, int(report["can_lost"]), self.rgb)
        output = BytesIO()
        Image.frombytes("RGB", (WIDTH, HEIGHT), bytes(self.rgb)).save(
            output, format="PNG"
        )
        return output.getvalue()


class DashboardDisplay:
    def __init__(self, port, library, *, serial_factory=serial.Serial):
        self.port = port
        self.library_path = library
        self.serial_factory = serial_factory
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = None
        self.serial_open = False
        self.report = None
        self.received_at = None
        self.image = None
        self.error = ""
        self.generation = 0
        self.touch_queue = deque()
        self.touch_active = False
        self.touch_received_at = 0.0
        self.move_sent_at = 0.0

    def start(self):
        if not self.port:
            return
        self.thread = threading.Thread(target=self._run, name="uno-display-reader")
        self.thread.start()

    def stop(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=3)
            if self.thread.is_alive():
                raise RuntimeError("UNO display serial worker did not stop")

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            age = (
                (now - self.received_at) * 1000
                if self.received_at is not None
                else None
            )
            return dict(
                enabled=bool(self.port),
                port=self.port,
                serial_open=self.serial_open,
                report_age_ms=round(age) if age is not None else None,
                fresh=bool(self.serial_open and age is not None and age < 1000),
                waiting_for_report=bool(self.serial_open and self.received_at is None),
                error=self.error,
                report=self.report,
                image=self.image,
                generation=self.generation,
                touch_ready=self._touch_ready(now),
            )

    def _touch_ready(self, now):
        return bool(
            self.serial_open and self.received_at is not None
            and now - self.received_at < 1.0
            and self.report and self.report["vtouch"]
        )

    def queue_touch(self, phase, x=None, y=None, *, generation):
        """Web boundary: validate once, then enqueue on the existing serial owner."""
        now = time.monotonic()
        with self.lock:
            if type(generation) is not int or generation != self.generation:
                raise ValueError("UNO 연결이 변경되어 이전 터치를 취소했습니다.")
            if phase not in ("D", "M", "U", "C"):
                self._cancel_touch()
                raise ValueError("지원하지 않는 virtual touch phase")
            if phase != "C" and (
                type(x) is not int or type(y) is not int
                or not 0 <= x < WIDTH or not 0 <= y < HEIGHT
            ):
                self._cancel_touch()
                raise ValueError("Virtual touch 좌표는 480 × 320 범위의 정수여야 합니다.")
            if phase == "C":
                self._cancel_touch()
                return
            if not self._touch_ready(now):
                self._cancel_touch()
                raise ValueError("UNO virtual touch 미지원 또는 표시 보고가 끊겼습니다.")
            if phase == "D":
                if self.touch_active:
                    self._cancel_touch()
                    raise ValueError("이전 virtual gesture를 취소했습니다. 다시 누르세요.")
                self.touch_active = True
            elif not self.touch_active:
                raise ValueError("활성 virtual gesture가 없습니다. 다시 누르세요.")
            # Replace only pending MOVE; DOWN/UP remain ordered. UP carries the
            # final coordinate so a MOVE immediately before it is redundant.
            if self.touch_queue and self.touch_queue[-1][0] == "M" and phase in ("M", "U"):
                self.touch_queue.pop()
            if len(self.touch_queue) >= 32:
                self._cancel_touch()
                raise ValueError("UNO 입력 지연으로 gesture를 취소했습니다. 다시 누르세요.")
            self.touch_queue.append((phase, f"VT {phase} {x} {y}\n".encode("ascii")))
            self.touch_received_at = now
            if phase == "U":
                self.touch_active = False

    def _cancel_touch(self):
        self.touch_queue.clear()
        if self.serial_open:
            self.touch_queue.append(("C", b"VT C\n"))
        self.touch_active = False

    def cancel_touch(self):
        with self.lock:
            self._cancel_touch()

    def _write_touch(self, connection):
        # Keep cancellation and writes ordered under the same lock. The port's
        # bounded write timeout limits how long a Web request can wait here.
        now = time.monotonic()
        with self.lock:
            if (self.touch_active or self.touch_queue) and (
                not self._touch_ready(now)
                or now - self.touch_received_at >= 1.0
            ):
                self._cancel_touch()
            if not self.touch_queue:
                return
            phase, wire = self.touch_queue[0]
            if phase == "M" and now - self.move_sent_at < 0.02:
                return
            if connection.write(wire) != len(wire):
                raise serial.SerialException("Incomplete UNO virtual touch write")
            self.touch_queue.popleft()
            if phase == "M":
                self.move_sent_at = now

    def _run(self):
        try:
            renderer = NativeDashboardRenderer(self.library_path)
        except (OSError, AttributeError) as exc:
            with self.lock:
                self.error = f"LCD renderer를 불러올 수 없습니다: {exc}"
            return
        while not self.stopping.is_set():
            connection = None
            try:
                connection = self.serial_factory(
                    self.port,
                    baudrate=115200,
                    timeout=0.01,
                    write_timeout=0.05,
                    exclusive=True,
                )
                connection.reset_input_buffer()
                # Terminate any partial old line, cancel first, then request reports.
                startup = b"\nVT C\nHIL_LCD ON\n"
                if connection.write(startup) != len(startup):
                    raise serial.SerialException("Incomplete UNO display setup write")
                with self.lock:
                    self.generation += 1
                    self.touch_queue.clear()
                    self.touch_active = False
                    self.serial_open = True
                    self.received_at = None
                    self.report = self.image = None
                    self.error = ""
                buffer = bytearray()
                while not self.stopping.is_set():
                    self._write_touch(connection)
                    buffer.extend(
                        connection.read(min(1024, connection.in_waiting or 1))
                    )
                    while b"\n" in buffer:
                        line, _, rest = buffer.partition(b"\n")
                        buffer = bytearray(rest)
                        try:
                            report = decode_display_report(bytes(line))
                        except (ValueError, UnicodeError) as exc:
                            with self.lock:
                                self.error = str(exc)
                            continue
                        if report is None:
                            continue
                        received = time.monotonic()
                        png = renderer.render(report)
                        with self.lock:
                            self.report = report
                            self.image = "data:image/png;base64," + base64.b64encode(
                                png
                            ).decode("ascii")
                            self.received_at = received
                            self.error = ""
                    if len(buffer) > 1024:
                        buffer.clear()
                        with self.lock:
                            self.error = "UNO report가 한 줄의 최대 크기를 초과했습니다"
            except (OSError, serial.SerialException) as exc:
                with self.lock:
                    self.error = str(exc)
            finally:
                with self.lock:
                    self.serial_open = False
                    self.touch_queue.clear()
                    self.touch_active = False
                if connection is not None:
                    try:
                        connection.write(b"\nVT C\nHIL_LCD OFF\n")
                    except (OSError, serial.SerialException):
                        pass
                    connection.close()
            self.stopping.wait(1.0)
