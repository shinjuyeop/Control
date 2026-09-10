"""UNO-applied display state -> production C++ renderer -> Web PNG.

This connection only enables/disables telemetry; it never sends touch, CAN or
vehicle commands. The serial worker alone owns the native renderer and port.
"""

from __future__ import annotations

import base64
import ctypes
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
AUTO_VALUE_FIELDS = (
    ("gps_state", 0, 4),
    ("lidar_state", 0, 2),
    ("camera_left_state", 0, 2),
    ("camera_right_state", 0, 2),
    ("drive_state", 0, 8),
    ("steer_state", 0, 12),
    ("brake_state", 0, 4),
    ("activity", 0, 8),
    ("output_state", 0, 2),
    ("feedback_state", 0, 2),
    ("host_valid", 0, 1),
    ("heading_centideg", 0, 0xFFFF),
    ("target_heading_centideg", 0, 0xFFFF),
    ("target_speed_centi_kmh", -0x8000, 0x7FFF),
    ("actual_motor_count", -0x80000000, 0x7FFFFFFF),
    ("target_motor_count", -0x80000000, 0x7FFFFFFF),
    ("steering_current_deciamp", -0x8000, 0x7FFF),
)
LEGACY_AUTO_VALUES = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0xFFFF, 0xFFFF, -0x8000, -0x80000000, -0x80000000, -0x8000,
]


def decode_display_report(line: bytes):
    """Validate a single external UNO report; ignore human Serial Monitor lines."""
    if not line.startswith(b"HIL_LCD "):
        return None
    raw = json.loads(line[8:])
    if not isinstance(raw, dict) or raw.get("v") != 1:
        raise ValueError("지원하지 않는 UNO LCD report protocol")
    page = raw.get("page")
    if page not in ("MANUAL_TFT", "AUTO_BASELINE", "AUTO_GT3"):
        raise ValueError("지원하지 않는 UNO 화면")
    values = raw.get("values")
    if not isinstance(values, list) or len(values) < len(VALUE_FIELDS):
        raise ValueError("UNO 표시값이 불완전합니다")
    data = {}
    for (name, minimum, maximum), value in zip(VALUE_FIELDS, values):
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(f"UNO {name} 값이 범위를 벗어났습니다")
        data[name] = value
    auto_values = raw.get("auto", LEGACY_AUTO_VALUES)
    if not isinstance(auto_values, list) or len(auto_values) < len(AUTO_VALUE_FIELDS):
        raise ValueError("UNO AUTO 표시값이 불완전합니다")
    auto_data = {}
    for (name, minimum, maximum), value in zip(AUTO_VALUE_FIELDS, auto_values):
        if type(value) is not int or not minimum <= value <= maximum:
            raise ValueError(f"UNO AUTO {name} 값이 범위를 벗어났습니다")
        auto_data[name] = value
    if type(raw.get("can_lost")) is not bool:
        raise ValueError("UNO can_lost 값이 boolean이 아닙니다")
    for name, minimum in (("uptime_ms", 0), ("seq", 0), ("can_age_ms", -1)):
        if type(raw.get(name)) is not int or not minimum <= raw[name] <= 0xFFFFFFFF:
            raise ValueError(f"UNO {name} 값이 유효하지 않습니다")
    return dict(
        page=page,
        can_lost=raw["can_lost"],
        can_age_ms=raw["can_age_ms"],
        uptime_ms=raw["uptime_ms"],
        seq=raw["seq"],
        values=data,
        auto_values=auto_data,
    )


class NativeDashboardRenderer:
    def __init__(self, library: str | Path):
        self.library = ctypes.CDLL(str(library))
        self.render_function = self.library.render_dashboard
        self.render_function.argtypes = [
            ctypes.POINTER(ctypes.c_int32),
            ctypes.POINTER(ctypes.c_int32),
            ctypes.c_int,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self.render_function.restype = None
        self.rgb = (ctypes.c_uint8 * (WIDTH * HEIGHT * 3))()

    def render(self, report):
        values = (ctypes.c_int32 * len(VALUE_FIELDS))(
            *(report["values"][name] for name, _, _ in VALUE_FIELDS)
        )
        auto_values = (ctypes.c_int32 * len(AUTO_VALUE_FIELDS))(
            *(report["auto_values"][name] for name, _, _ in AUTO_VALUE_FIELDS)
        )
        self.render_function(
            values, auto_values, int(report["can_lost"]), self.rgb
        )
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
            )

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
                    timeout=0.1,
                    write_timeout=0.3,
                    exclusive=True,
                )
                connection.reset_input_buffer()
                with self.lock:
                    self.serial_open = True
                    self.received_at = None
                    self.report = self.image = None
                    self.error = ""
                connection.write(b"HIL_LCD ON\n")
                buffer = bytearray()
                while not self.stopping.is_set():
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
                if connection is not None:
                    try:
                        connection.write(b"HIL_LCD OFF\n")
                    except (OSError, serial.SerialException):
                        pass
                    connection.close()
                with self.lock:
                    self.serial_open = False
            self.stopping.wait(1.0)
