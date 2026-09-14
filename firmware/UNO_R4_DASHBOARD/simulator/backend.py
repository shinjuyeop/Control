"""ctypes facade for the actual UNO dashboard firmware and SPI transport."""

from __future__ import annotations

import atexit
import ctypes
import json
import os
from pathlib import Path
import struct
import subprocess
import tempfile
from typing import Callable

from PIL import Image

from .auto_model import AutoTelemetry
from .model import STATUS_ESTOP_ACTIVE, ScenarioSample


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "src"
NATIVE = Path(__file__).resolve().parent / "native"
CACHE = Path(os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")) / "uno_dashboard_preview"
OUTPUT = CACHE / "libuno_dashboard.so"


def _build_library() -> Path:
    CACHE.mkdir(parents=True, exist_ok=True)
    sources = [
        NATIVE / "host_runtime.cpp",
        NATIVE / "bridge.cpp",
        ROOT / "hil_display_report.cpp",
        *sorted(SOURCE.rglob("*.cpp")),
    ]
    sketch = ROOT / "UNO_R4_DASHBOARD.ino"
    dependencies = [
        *sources, sketch, *sorted(SOURCE.rglob("*.h")), *NATIVE.glob("*.h")
    ]
    if OUTPUT.exists() and OUTPUT.stat().st_mtime >= max(
        path.stat().st_mtime for path in dependencies
    ):
        return OUTPUT

    descriptor, temporary_name = tempfile.mkstemp(
        prefix="libuno_dashboard-", suffix=".so", dir=CACHE
    )
    os.close(descriptor)
    temporary = Path(temporary_name)
    command = [
        "g++", "-std=c++17", "-O2", "-fPIC", "-shared", "-pthread",
        "-DDASHBOARD_HOST", f"-I{NATIVE}", f"-I{SOURCE / 'include'}",
        f"-I{ROOT}", *(str(source) for source in sources),
        "-x", "c++", str(sketch), "-o", str(temporary),
    ]
    try:
        subprocess.run(command, check=True)
        os.replace(temporary, OUTPUT)
    finally:
        temporary.unlink(missing_ok=True)
    return OUTPUT


WIDTH = 480
HEIGHT = 320
PROTOCOL_VERSION = 1
FRAME_IDS = tuple(range(0x321, 0x329))
INVALID_HEADING = 0xFFFF
INVALID_I16 = -0x8000
INVALID_I32 = -0x80000000
PROFILE_PATH = Path(__file__).with_name("timing_profile.json")
PROFILE_PAGES = ("startup", "manual", "auto")

GPS_CODES = {"FIXED": 1, "FLOAT": 2, "FIX": 3, "NO FIX": 4, "WAIT": 0}
STREAM_CODES = {"RX": 1, "LOST": 2, "WAIT": 0, "DATA LOST": 0}
DRIVE_CODES = {
    "WAIT": 0, "RUN": 1, "HOLD": 2, "CMD LOST": 3,
    "NO FEEDBACK": 4, "RUN MISMATCH": 5, "FAULT": 6,
    "STATUS LOST": 7, "WAIT CMD": 8, "DATA LOST": 0,
}
STEER_CODES = {
    "WAIT": 0, "ACTIVE": 1, "WAIT CMD": 2, "STATUS LOST": 3,
    "MOTOR FAULT": 4, "CAN LOST": 5, "POS LOST": 6, "CMD LOST": 7,
    "TRACK FAULT": 8, "TRACK WARN": 9, "BRAKE": 10, "E-STOP": 11,
    "DISABLED": 12, "WAIT SPEED": 12, "FAULT": 4, "FB LOST": 6,
    "DATA LOST": 0,
}
BRAKE_CODES = {
    "WAIT": 0, "OFF": 1, "ON": 2, "CMD LOST": 3,
    "STATUS LOST": 4, "DATA LOST": 0,
}
ACTIVITY_CODES = {
    "FOLLOW ROUTE": 1, "REJOIN ROUTE": 2, "TRAFFIC CONTROL": 3,
    "AVOID OBSTACLE": 4, "INTERSECTION": 5, "FINISH": 6,
    "EMERGENCY STOP": 7,
}


def _u16(value: int) -> bytes:
    return struct.pack("<H", value & 0xFFFF)


def _i16(value: int) -> bytes:
    return struct.pack("<h", max(-32768, min(32767, value)))


def _i32(value: int) -> bytes:
    return struct.pack("<i", max(-2147483648, min(2147483647, value)))


def _profile_u32(value: object, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{field} must be a non-negative uint32")
    return value


def _profile_table(
    values: object, length: int, field: str
) -> list[int]:
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f"{field} must contain {length} values")
    return [
        _profile_u32(value, f"{field}[{index}]")
        for index, value in enumerate(values)
    ]


def _load_timing_profile(path: Path) -> tuple[dict, dict[str, ctypes.Array | int]]:
    profile = json.loads(path.read_text(encoding="utf-8"))
    state_values = [
        _profile_u32(profile["state_prepare_us"][page], f"state_prepare_us.{page}")
        for page in PROFILE_PAGES
    ]
    row_values: list[int] = []
    for page in PROFILE_PAGES:
        rows = profile["row_compose_us"][page]
        if not isinstance(rows, list) or len(rows) != HEIGHT:
            raise ValueError(f"row_compose_us.{page} must contain {HEIGHT} rows")
        row_values.extend(
            _profile_u32(value, f"row_compose_us.{page}[{index}]")
            for index, value in enumerate(rows)
        )
    legacy_spi_ns = _profile_u32(
        profile.get("spi_call_us", 0), "spi_call_us"
    ) * 1000
    transport = profile.get("transport")
    if transport is None:
        spi_buffer_ns = [0] + [legacy_spi_ns] * 256
        rgb565_pack_ns = [0] + [legacy_spi_ns] * 128
        solid_pack_ns = [0] + [legacy_spi_ns] * 128
        transaction_begin_ns = legacy_spi_ns
        transaction_end_ns = legacy_spi_ns
        spi_byte_extra_ns = legacy_spi_ns
    else:
        if not isinstance(transport, dict):
            raise ValueError("transport must be an object")
        transaction_begin_ns = _profile_u32(
            transport["transaction_begin_ns"],
            "transport.transaction_begin_ns",
        )
        transaction_end_ns = _profile_u32(
            transport["transaction_end_ns"],
            "transport.transaction_end_ns",
        )
        spi_byte_extra_ns = _profile_u32(
            transport["spi_byte_extra_ns"],
            "transport.spi_byte_extra_ns",
        )
        spi_buffer_ns = _profile_table(
            transport["spi_buffer_extra_ns"], 257,
            "transport.spi_buffer_extra_ns",
        )
        rgb565_pack_ns = _profile_table(
            transport["rgb565_pack_ns"], 129,
            "transport.rgb565_pack_ns",
        )
        solid_pack_ns = _profile_table(
            transport["solid_pack_ns"], 129,
            "transport.solid_pack_ns",
        )

    startup = profile.get("startup_cpu")
    startup_elapsed: list[int] = []
    startup_state: list[int] = []
    startup_rows: list[int] = []
    if startup is not None:
        if not isinstance(startup, dict):
            raise ValueError("startup_cpu must be an object")
        elapsed_values = startup["elapsed_ms"]
        if not isinstance(elapsed_values, list) or not elapsed_values:
            raise ValueError("startup_cpu.elapsed_ms must not be empty")
        if len(elapsed_values) > 141:
            raise ValueError("startup_cpu supports at most 141 samples")
        for index, value in enumerate(elapsed_values):
            elapsed = _profile_u32(value, f"startup_cpu.elapsed_ms[{index}]")
            if elapsed > 1400:
                raise ValueError("startup_cpu elapsed_ms must be <= 1400")
            if startup_elapsed and elapsed <= startup_elapsed[-1]:
                raise ValueError("startup_cpu elapsed_ms must increase")
            startup_elapsed.append(elapsed)
        startup_state = _profile_table(
            startup["state_prepare_us"], len(startup_elapsed),
            "startup_cpu.state_prepare_us",
        )
        rows = startup["row_compose_us"]
        if not isinstance(rows, list) or len(rows) != len(startup_elapsed):
            raise ValueError("startup_cpu row table must match elapsed samples")
        for sample_index, sample_rows in enumerate(rows):
            startup_rows.extend(_profile_table(
                sample_rows, HEIGHT,
                f"startup_cpu.row_compose_us[{sample_index}]",
            ))

    payload: dict[str, ctypes.Array | int] = {
        "state": (ctypes.c_uint32 * len(state_values))(*state_values),
        "rows": (ctypes.c_uint32 * len(row_values))(*row_values),
        "transaction_begin_ns": transaction_begin_ns,
        "transaction_end_ns": transaction_end_ns,
        "spi_byte_extra_ns": spi_byte_extra_ns,
        "spi_buffer": (ctypes.c_uint32 * 257)(*spi_buffer_ns),
        "rgb565_pack": (ctypes.c_uint32 * 129)(*rgb565_pack_ns),
        "solid_pack": (ctypes.c_uint32 * 129)(*solid_pack_ns),
        "startup_elapsed": (ctypes.c_uint16 * len(startup_elapsed))(
            *startup_elapsed
        ),
        "startup_state": (ctypes.c_uint32 * len(startup_state))(
            *startup_state
        ),
        "startup_rows": (ctypes.c_uint32 * len(startup_rows))(*startup_rows),
        "startup_count": len(startup_elapsed),
    }
    return profile, payload


def _drive_state(auto: AutoTelemetry) -> str:
    d = auto.drive
    if d is None:
        return "WAIT"
    if not d.fresh:
        return "STATUS LOST"
    if d.fault_bits_valid and d.fault_bits:
        return "FAULT"
    if d.command_timeout:
        return "CMD LOST"
    if not d.command_received:
        return "WAIT CMD"
    if not (d.part1_valid and d.part2_valid and d.tractive_feedback_valid and d.fault_bits_valid):
        return "NO FEEDBACK"
    if d.tractive_commanded != d.tractive_feedback:
        return "RUN MISMATCH"
    if d.stop_request or not d.tractive_commanded:
        return "HOLD"
    return "RUN"


def _steer_state(auto: AutoTelemetry) -> str:
    if auto.steering_status_fresh is None:
        return "WAIT"
    if not auto.steering_status_fresh:
        return "STATUS LOST"
    if auto.steering_state == "FAULT" or (auto.steering_fault_bits is not None and auto.steering_fault_bits & ~1):
        return "MOTOR FAULT"
    if not auto.steering_communication_ok:
        return "CAN LOST"
    if not auto.steering_feedback_valid:
        return "POS LOST"
    if auto.steering_mode_timeout or auto.steering_command_timeout:
        return "CMD LOST"
    if auto.steering_tracking_fault:
        return "TRACK FAULT"
    if auto.steering_tracking_warning:
        return "TRACK WARN"
    if auto.steering_state == "ACTIVE" and auto.steering_enabled_commanded:
        return "ACTIVE"
    return auto.steering_state


def _brake_state(auto: AutoTelemetry) -> str:
    b = auto.brake
    if b is None:
        return "WAIT"
    if not b.fresh:
        return "STATUS LOST"
    if b.stop_reason_bits & 8:
        return "CMD LOST"
    return "ON" if b.brake_commanded else "OFF"


def _health_states(sample: ScenarioSample, auto: AutoTelemetry) -> tuple[str, ...]:
    gps = {
        "RTK FIXED": "FIXED", "RTK FLOAT": "FLOAT", "FIX": "FIX",
        "NO FIX": "NO FIX", "WAIT": "WAIT",
    }
    states = [gps[auto.gnss_fix]]
    for stream in (auto.lidar_state, auto.camera_left_state, auto.camera_right_state):
        states.append("RX" if stream == "LIVE" else stream)
    states.extend((
        _drive_state(auto),
        _steer_state(auto),
        "ON" if sample.values.status(STATUS_ESTOP_ACTIVE) else "OFF",
        _brake_state(auto),
    ))
    if sample.dashboard_can_lost:
        return ("DATA LOST",) * 8
    if auto.host_state == "LOST":
        return tuple(state if index == 6 else "DATA LOST"
                     for index, state in enumerate(states))
    return tuple(states)


def _can_payloads(sample: ScenarioSample, alive: int) -> tuple[bytes, ...]:
    values = sample.values
    prefix = bytes((PROTOCOL_VERSION, alive & 0xFF))
    native = (
        prefix + bytes((values.status_flags, values.drive_flags,
                        values.pedal_percent, values.gnss_state))
        + _i16(values.speed_centi_mps),
        prefix + _u16(values.bus_voltage_decivolt)
        + _i16(values.bus_current_deciamp) + _i16(values.phase_current_deciamp),
        prefix + _i16(values.controller_temperature_decic)
        + _i16(values.motor_temperature_decic) + b"\0\0",
        prefix + int(values.motor_fault_bits & 0xFFFFFF).to_bytes(3, "little")
        + b"\0\0\0",
    )

    raw = sample.auto or AutoTelemetry()
    states = _health_states(sample, raw)
    live = raw.host_state == "LIVE"
    health = bytes((
        GPS_CODES.get(states[0], 0)
        | (STREAM_CODES.get(states[1], 0) << 4),
        STREAM_CODES.get(states[2], 0)
        | (STREAM_CODES.get(states[3], 0) << 4),
        DRIVE_CODES.get(states[4], 0)
        | (STEER_CODES.get(states[5], 0) << 4),
        BRAKE_CODES.get(states[7], 0)
        | ((ACTIVITY_CODES.get(raw.activity, 8 if live else 0) & 0x0F) << 4),
        (0 if raw.steering_enabled_commanded is None
         else 2 if raw.steering_enabled_commanded else 1)
        | ((0 if raw.steering_feedback_valid is None
            else 2 if raw.steering_feedback_valid and raw.steering_communication_ok
            else 1) << 4),
        0x80 if live else 0,
    ))
    heading = (INVALID_HEADING if raw.vehicle_heading_deg is None
               else round(raw.vehicle_heading_deg * 100.0) % 36000)
    target_heading = (INVALID_HEADING if raw.target_heading_deg is None
                      else round(raw.target_heading_deg * 100.0) % 36000)
    target_speed = (INVALID_I16 if raw.target_speed_kmh is None
                    else round(raw.target_speed_kmh * 100.0))
    steering_live = bool(raw.steering_status_fresh)
    actual = (raw.steering_actual_motor_count
              if steering_live and raw.steering_feedback_valid
              and raw.steering_actual_motor_count is not None else INVALID_I32)
    target = (raw.steering_target_motor_count
              if steering_live and not raw.steering_command_timeout
              and raw.steering_target_motor_count is not None else INVALID_I32)
    current = (round(raw.steering_current_a * 10.0)
               if steering_live and raw.steering_communication_ok
               and raw.steering_current_a is not None else INVALID_I16)
    auto_data = health + struct.pack(
        "<HHhihi", heading, target_heading, target_speed,
        actual, current, target,
    ) + b"\0\0"
    auto_frames = tuple(prefix + auto_data[index:index + 6]
                        for index in range(0, 24, 6))
    return native + auto_frames


class _NativeEvent(ctypes.Structure):
    _fields_ = [
        ("time_ms", ctypes.c_uint32), ("region", ctypes.c_uint16),
        ("boundary", ctypes.c_uint16), ("kind", ctypes.c_uint8),
        ("page", ctypes.c_uint8), ("alive", ctypes.c_uint8),
        ("flags", ctypes.c_uint8), ("direction_auto", ctypes.c_uint8),
    ]


class _StartupFrameEvent(ctypes.Structure):
    _fields_ = [
        ("time_us", ctypes.c_uint32), ("elapsed_ms", ctypes.c_uint16),
        ("done", ctypes.c_uint8),
    ]


class NativeDashboard:
    def __init__(self, timing_profile: Path | None = None) -> None:
        library_path = _build_library()
        self.library = ctypes.CDLL(str(library_path))
        self.library.dashboard_run_until.argtypes = [ctypes.c_uint64]
        self.library.dashboard_run_until.restype = ctypes.c_int
        self.library.dashboard_last_error.restype = ctypes.c_char_p
        self.library.dashboard_now_us.restype = ctypes.c_uint64
        self.library.dashboard_ready.restype = ctypes.c_int
        self.library.dashboard_replay_active.restype = ctypes.c_int
        self.library.dashboard_transition_active.restype = ctypes.c_int
        self.library.dashboard_auto_displayed.restype = ctypes.c_int
        self.library.dashboard_transition_boundary.restype = ctypes.c_uint16
        self.library.dashboard_auto_history_count.restype = ctypes.c_uint32
        self.library.dashboard_startup_start_us.restype = ctypes.c_uint32
        self.library.dashboard_startup_completed_us.restype = ctypes.c_uint32
        self.library.dashboard_copy_framebuffer.argtypes = [
            ctypes.POINTER(ctypes.c_uint16)
        ]
        self.library.dashboard_schedule_can.argtypes = [
            ctypes.c_uint64, ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint8)
        ]
        self.library.dashboard_copy_events.argtypes = [
            ctypes.POINTER(_NativeEvent), ctypes.c_uint32
        ]
        self.library.dashboard_clear_events.argtypes = [ctypes.c_uint16]
        self.library.dashboard_copy_events.restype = ctypes.c_uint32
        self.library.dashboard_set_timing_profile.argtypes = [
            ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
            ctypes.c_uint32, ctypes.c_uint32,
        ]
        self.library.dashboard_set_transport_profile.argtypes = [
            ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32),
        ]
        self.library.dashboard_set_startup_profile.argtypes = [
            ctypes.POINTER(ctypes.c_uint16),
            ctypes.POINTER(ctypes.c_uint32),
            ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint16,
        ]
        self.library.dashboard_copy_startup_frames.argtypes = [
            ctypes.POINTER(_StartupFrameEvent), ctypes.c_uint32
        ]
        self.library.dashboard_copy_startup_frames.restype = ctypes.c_uint32
        profile, timing = _load_timing_profile(
            timing_profile or PROFILE_PATH
        )
        self.profile_calibrated = bool(profile.get("calibrated", False))
        self.library.dashboard_set_timing_profile(
            timing["state"], timing["rows"],
            _profile_u32(profile["spi_call_us"], "spi_call_us"),
            _profile_u32(profile["loop_us"], "loop_us"),
        )
        self.library.dashboard_set_transport_profile(
            timing["transaction_begin_ns"], timing["transaction_end_ns"],
            timing["spi_byte_extra_ns"], timing["spi_buffer"],
            timing["rgb565_pack"], timing["solid_pack"],
        )
        self.library.dashboard_set_startup_profile(
            timing["startup_elapsed"], timing["startup_state"],
            timing["startup_rows"], timing["startup_count"],
        )
        self.library.dashboard_start()
        self._closed = False
        atexit.register(self.close)

    @property
    def now_us(self) -> int:
        return int(self.library.dashboard_now_us())

    @property
    def ready(self) -> bool:
        return bool(self.library.dashboard_ready())

    def run_until(self, time_us: int) -> None:
        if time_us >= self.now_us:
            if self.library.dashboard_run_until(time_us) != 0:
                message = self.library.dashboard_last_error()
                raise RuntimeError(
                    message.decode("utf-8") if message else
                    "native firmware stopped before the requested time"
                )

    def run_until_ready(self, maximum_us: int = 5_000_000) -> int:
        while not self.ready and self.now_us < maximum_us:
            self.run_until(min(maximum_us, self.now_us + 50_000))
        if not self.ready:
            raise RuntimeError("native firmware setup did not finish")
        return self.now_us

    def schedule_sample(self, arrival_us: int, sample: ScenarioSample, alive: int) -> None:
        if sample.dashboard_can_lost:
            return
        payloads = _can_payloads(sample, alive)
        for slot, offset_us in enumerate((0, 10_000, 20_000, 30_000)):
            for frame_index in (slot, slot + 4):
                payload = (ctypes.c_uint8 * 8).from_buffer_copy(payloads[frame_index])
                self.library.dashboard_schedule_can(
                    arrival_us + offset_us, FRAME_IDS[frame_index], payload
                )

    def schedule_frame(self, arrival_us: int, identifier: int, payload: bytes) -> None:
        if len(payload) != 8:
            raise ValueError("CAN payload must contain 8 bytes")
        data = (ctypes.c_uint8 * 8).from_buffer_copy(payload)
        self.library.dashboard_schedule_can(arrival_us, identifier, data)

    def clear_events(self, region_mask: int = 0xFFFF) -> None:
        self.library.dashboard_clear_events(ctypes.c_uint16(region_mask))

    def events(self) -> list[dict[str, int | str]]:
        storage = (_NativeEvent * 256)()
        count = int(self.library.dashboard_copy_events(storage, len(storage)))
        kinds = {1: "region", 2: "mode_start", 3: "mode_done"}
        return [
            {
                "kind": kinds.get(event.kind, "unknown"),
                "time_ms": int(event.time_ms), "page": int(event.page),
                "region": int(event.region), "boundary": int(event.boundary),
                "alive": int(event.alive), "flags": int(event.flags),
                "direction_auto": int(event.direction_auto),
            }
            for event in storage[:count]
        ]

    def startup_frames(self) -> list[dict[str, int | str]]:
        storage = (_StartupFrameEvent * 64)()
        count = int(self.library.dashboard_copy_startup_frames(
            storage, len(storage)
        ))
        return [
            {
                "phase": "done" if event.done else "start",
                "elapsed_ms": int(event.elapsed_ms),
                "time_us": int(event.time_us),
            }
            for event in storage[:count]
        ]

    def image(self) -> Image.Image:
        pixels = (ctypes.c_uint16 * (WIDTH * HEIGHT))()
        self.library.dashboard_copy_framebuffer(pixels)
        return Image.frombytes(
            "RGB", (WIDTH, HEIGHT), bytes(pixels), "raw", "BGR;16"
        )

    def replay_startup(self) -> None:
        self.library.dashboard_replay_startup()

    @property
    def replay_active(self) -> bool:
        return bool(self.library.dashboard_replay_active())

    @property
    def transition_active(self) -> bool:
        return bool(self.library.dashboard_transition_active())

    @property
    def auto_displayed(self) -> bool:
        return bool(self.library.dashboard_auto_displayed())

    @property
    def transition_boundary(self) -> int:
        return int(self.library.dashboard_transition_boundary())

    @property
    def auto_history_count(self) -> int:
        return int(self.library.dashboard_auto_history_count())

    @property
    def startup_times_us(self) -> tuple[int, int]:
        return (
            int(self.library.dashboard_startup_start_us()),
            int(self.library.dashboard_startup_completed_us()),
        )

    def close(self) -> None:
        if not self._closed:
            self.library.dashboard_shutdown()
            self._closed = True


def schedule_samples(
    native: NativeDashboard,
    next_sample_us: int,
    target_us: int,
    period_us: int,
    alive: int,
    sample_at: Callable[[int], ScenarioSample],
) -> tuple[int, int]:
    while next_sample_us <= target_us:
        native.schedule_sample(next_sample_us, sample_at(next_sample_us), alive)
        next_sample_us += period_us
        alive = (alive + 1) & 0xFF
    return next_sample_us, alive
