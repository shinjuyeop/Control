"""Dashboard CAN values and simulator input scenarios."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

from .auto_model import AUTO_CASE_LABELS, AutoTelemetry, auto_telemetry

LCD_WIDTH = 480

LCD_HEIGHT = 320

STATUS_VEHICLE_VALID = 1 << 0

STATUS_DETAIL_VALID = 1 << 1

STATUS_PART1_VALID = 1 << 2

STATUS_PART2_VALID = 1 << 3

STATUS_MODE_AUTO = 1 << 4

STATUS_GEAR_REVERSE = 1 << 5

STATUS_ESTOP_ACTIVE = 1 << 6

STATUS_BRAKE_ACTIVE = 1 << 7

DRIVE_TRACTIVE_FEEDBACK = 1 << 0

DRIVE_COMMAND_RECEIVED = 1 << 1

DRIVE_COMMAND_TIMEOUT = 1 << 2

DRIVE_STOP = 1 << 3

GNSS_UNAVAILABLE = 0

GNSS_NO_FIX = 1

GNSS_FIX = 2

GNSS_RTK_FIXED = 3

GNSS_STATE_NAMES = {
    GNSS_UNAVAILABLE: "UNAVAILABLE",
    GNSS_NO_FIX: "NO FIX",
    GNSS_FIX: "FIX",
    GNSS_RTK_FIXED: "RTK FIXED",
}

GNSS_STATE_VALUES = {name: state for state, name in GNSS_STATE_NAMES.items()}

FAULT_NAMES = (
    "OVERCURRENT",
    "OVERLOAD",
    "OVERVOLTAGE",
    "UNDERVOLTAGE",
    "CTRL OVERHEAT",
    "MOTOR OVERHEAT",
    "MOTOR STALLED",
    "PHASE WIRE",
    "MOTOR SENSOR",
    "AUX SENSOR",
    "ENCODER ALIGN",
    "ANTI RUNAWAY",
    "MAIN THROTTLE",
    "AUX THROTTLE",
    "PRECHARGE",
    "DC CONTACTOR",
    "POWER VALVE",
    "CURRENT SENSOR",
    "AUTO TUNE",
    "RS485",
    "MOTOR CAN",
    "SOFTWARE",
    "FAULT 22",
    "FAULT 23",
)

SCENARIOS = (
    "road", "load", "modes", "links", "faults", "auto_health",
    *(f"auto_{case}" for case in AUTO_CASE_LABELS),
)

@dataclass
class DashboardValues:
    """Canonical values available after a complete four-frame CAN snapshot."""

    status_flags: int = 0
    drive_flags: int = 0
    pedal_percent: int = 0
    gnss_state: int = GNSS_UNAVAILABLE
    speed_centi_mps: int = 0
    bus_voltage_decivolt: int = 0
    bus_current_deciamp: int = 0
    phase_current_deciamp: int = 0
    controller_temperature_decic: int = 0
    motor_temperature_decic: int = 0
    motor_fault_bits: int = 0
    lv_status_flags: int = 0
    lv_voltage_decivolt: int = 0
    lv_soc_percent: int = 255

    def status(self, flag: int) -> bool:
        return bool(self.status_flags & flag)

    def drive(self, flag: int) -> bool:
        return bool(self.drive_flags & flag)

    @property
    def vehicle_valid(self) -> bool:
        return self.status(STATUS_VEHICLE_VALID)

    @property
    def detail_valid(self) -> bool:
        return self.status(STATUS_DETAIL_VALID)

    @property
    def power_valid(self) -> bool:
        return self.detail_valid and self.status(STATUS_PART1_VALID)

    @property
    def thermal_valid(self) -> bool:
        return self.detail_valid and self.status(STATUS_PART2_VALID)

@dataclass
class ScenarioSample:
    values: DashboardValues
    dashboard_can_lost: bool = False
    auto: AutoTelemetry | None = None

def _base_values() -> DashboardValues:
    return DashboardValues(
        status_flags=(
            STATUS_VEHICLE_VALID
            | STATUS_DETAIL_VALID
            | STATUS_PART1_VALID
            | STATUS_PART2_VALID
        ),
        drive_flags=DRIVE_TRACTIVE_FEEDBACK,
        pedal_percent=24,
        gnss_state=GNSS_RTK_FIXED,
        speed_centi_mps=694,
        bus_voltage_decivolt=550,
        bus_current_deciamp=620,
        phase_current_deciamp=1460,
        controller_temperature_decic=410,
        motor_temperature_decic=470,
        motor_fault_bits=0,
    )

def _speed_centi_mps(speed_kmh: float) -> int:
    return round((speed_kmh / 3.6) * 100.0)

def _smoothstep(value: float) -> float:
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)

def _road_scenario(elapsed: float) -> ScenarioSample:
    values = _base_values()
    phase = elapsed % 30.0
    if phase < 6.0:
        load = _smoothstep(phase / 6.0) * 0.72
    elif phase < 12.0:
        load = 0.72 + 0.04 * math.sin((phase - 6.0) * 1.7)
    elif phase < 18.0:
        load = 0.72 - _smoothstep((phase - 12.0) / 6.0) * 0.48
    elif phase < 24.0:
        load = 0.24
    else:
        load = 0.24 * (1.0 - _smoothstep((phase - 24.0) / 6.0))

    pedal = max(0.0, min(100.0, load * 100.0))
    phase_current = load * 430.0
    bus_current = load * 185.0
    speed_kmh = min(82.0, load * 82.0 + (phase / 30.0) * 8.0)
    values.pedal_percent = round(pedal)
    values.phase_current_deciamp = round(phase_current * 10.0)
    values.bus_current_deciamp = round(bus_current * 10.0)
    values.bus_voltage_decivolt = round((55.4 - bus_current * 0.025) * 10.0)
    values.speed_centi_mps = _speed_centi_mps(speed_kmh)
    values.controller_temperature_decic = round((38.0 + load * 20.0) * 10.0)
    values.motor_temperature_decic = round((42.0 + load * 30.0) * 10.0)
    values.drive_flags = DRIVE_TRACTIVE_FEEDBACK if pedal > 2.0 else 0
    return ScenarioSample(values)

def _load_scenario(elapsed: float) -> ScenarioSample:
    values = _base_values()
    phase = elapsed % 12.0
    load = phase / 6.0 if phase < 6.0 else (12.0 - phase) / 6.0
    load = _smoothstep(load)
    phase_current = load * 520.0
    bus_current = load * 380.0
    values.pedal_percent = round(load * 100.0)
    values.phase_current_deciamp = round(phase_current * 10.0)
    values.bus_current_deciamp = round(bus_current * 10.0)
    values.bus_voltage_decivolt = round((55.4 - load * 7.0) * 10.0)
    values.speed_centi_mps = _speed_centi_mps(load * 74.0)
    values.controller_temperature_decic = round((38.0 + load * 31.0) * 10.0)
    values.motor_temperature_decic = round((43.0 + load * 76.0) * 10.0)
    values.drive_flags = DRIVE_TRACTIVE_FEEDBACK if load > 0.02 else 0
    return ScenarioSample(values)

def _modes_scenario(elapsed: float) -> ScenarioSample:
    values = _base_values()
    stage = int(elapsed / 3.0) % 6
    if stage == 0:
        values.pedal_percent = 16
    elif stage == 1:
        values.pedal_percent = 62
        values.phase_current_deciamp = 2980
        values.bus_current_deciamp = 1320
    elif stage == 2:
        values.status_flags |= STATUS_GEAR_REVERSE
        values.pedal_percent = 28
        values.speed_centi_mps = -278
        values.phase_current_deciamp = -1720
        values.bus_current_deciamp = 710
    elif stage == 3:
        values.status_flags |= STATUS_MODE_AUTO
        values.pedal_percent = 0
        values.drive_flags = 0
    elif stage == 4:
        values.status_flags |= STATUS_MODE_AUTO
        values.pedal_percent = 0
        values.drive_flags = DRIVE_COMMAND_RECEIVED | DRIVE_TRACTIVE_FEEDBACK
    else:
        values.status_flags |= STATUS_MODE_AUTO
        values.pedal_percent = 0
        values.drive_flags = DRIVE_COMMAND_TIMEOUT
    return ScenarioSample(values)

def _links_scenario(elapsed: float) -> ScenarioSample:
    values = _base_values()
    stage = int(elapsed / 2.5) % 10
    dashboard_can_lost = False
    if stage == 1:
        values.status_flags &= ~STATUS_PART1_VALID
    elif stage == 2:
        values.status_flags &= ~STATUS_PART2_VALID
    elif stage == 3:
        values.status_flags &= ~(
            STATUS_DETAIL_VALID | STATUS_PART1_VALID | STATUS_PART2_VALID
        )
    elif stage == 4:
        values.status_flags &= ~STATUS_VEHICLE_VALID
    elif stage == 5:
        dashboard_can_lost = True
    elif stage == 6:
        values.drive_flags |= DRIVE_STOP
    elif stage == 7:
        values.status_flags |= STATUS_BRAKE_ACTIVE
    elif stage == 8:
        values.status_flags |= STATUS_ESTOP_ACTIVE | STATUS_BRAKE_ACTIVE
    return ScenarioSample(values, dashboard_can_lost)

def _faults_scenario(elapsed: float) -> ScenarioSample:
    values = _base_values()
    stage = int(elapsed / 1.5) % 27
    if stage < 24:
        values.motor_fault_bits = 1 << stage
    else:
        values.motor_fault_bits = (1 << 0) | (1 << 4) | (1 << 6)
    return ScenarioSample(values)


def _auto_scenario(elapsed: float, case: str) -> ScenarioSample:
    values = _base_values()
    values.status_flags |= STATUS_MODE_AUTO
    values.drive_flags = DRIVE_COMMAND_RECEIVED | DRIVE_TRACTIVE_FEEDBACK
    values.speed_centi_mps = _speed_centi_mps(12.3 + 0.3 * math.sin(elapsed))
    values.pedal_percent = 0
    values.bus_voltage_decivolt = 548
    telemetry = auto_telemetry(case, elapsed)
    if case == "low_speed":
        values.speed_centi_mps = _speed_centi_mps(0.4)
    elif case == "heading_wait":
        values.speed_centi_mps = 0
    elif case == "launch":
        phase = elapsed % 9.0
        values.speed_centi_mps = _speed_centi_mps(phase * 0.3 if phase < 3.0 else 1.1 + (phase - 3.0) * 1.5)
    elif case == "steering_trip":
        values.speed_centi_mps = 0
    values.gnss_state = {
        "WAIT": GNSS_UNAVAILABLE, "NO FIX": GNSS_NO_FIX,
        "RTK FLOAT": GNSS_FIX, "FIX": GNSS_FIX, "RTK FIXED": GNSS_RTK_FIXED,
    }[telemetry.gnss_fix]
    if case == "waiting":
        values.speed_centi_mps = 0
        values.drive_flags = 0
    elif case in ("command_lost", "host_lost"):
        values.drive_flags = DRIVE_COMMAND_TIMEOUT
    elif case == "estop":
        values.status_flags |= STATUS_ESTOP_ACTIVE | STATUS_BRAKE_ACTIVE
        values.drive_flags = 0
        values.speed_centi_mps = 0
    elif case == "brake":
        values.status_flags |= STATUS_BRAKE_ACTIVE
        values.drive_flags = DRIVE_COMMAND_RECEIVED
        values.speed_centi_mps = 0
    if case == "host_lost":
        values.gnss_state = GNSS_UNAVAILABLE
    return ScenarioSample(values, case == "can_lost", telemetry)


def _auto_health_scenario(elapsed: float) -> ScenarioSample:
    cases = tuple(AUTO_CASE_LABELS)
    return _auto_scenario(elapsed % 4.0, cases[int(elapsed / 4.0) % len(cases)])

SCENARIO_BUILDERS: dict[str, Callable[[float], ScenarioSample]] = {
    "road": _road_scenario,
    "load": _load_scenario,
    "modes": _modes_scenario,
    "links": _links_scenario,
    "faults": _faults_scenario,
    "auto_health": _auto_health_scenario,
    **{
        f"auto_{case}": lambda elapsed, case=case: _auto_scenario(elapsed, case)
        for case in AUTO_CASE_LABELS
    },
}
