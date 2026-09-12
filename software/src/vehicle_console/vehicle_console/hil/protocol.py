"""Pure Python implementation of the STM_TEST line protocol."""

from __future__ import annotations

import math
import time
from typing import Callable, Dict

from .models import HilDesiredState, HilStatus, ProtocolResponse


MODES = {"MANUAL", "AUTO"}
GEARS = {"FORWARD", "REVERSE"}
#: Commands spell the gear out; STATUS abbreviates it. Anything comparing a
#: requested gear against a reported one has to go through this.
GEAR_STATUS_NAMES = {"FORWARD": "FWD", "REVERSE": "REV"}
GEAR_COMMAND_NAMES = {status: command for command, status in GEAR_STATUS_NAMES.items()}
#: STM_TEST reports the pedal as a rounded percentage, so an exact float
#: comparison against the value that was sent would never match.
PEDAL_MATCH_TOLERANCE = 0.11
ESTOPS = {"DASH", "SIDE", "RF"}
CONTROLLERS = {"EZ", "KEYA"}
BRAKE_STATES = {"UNKNOWN", "RELEASED", "ENGAGED"}

NUMERIC_LIMITS = {
    "PEDAL": (0.0, 100.0),
    "SPEED": (-200.0, 200.0),
    "RPM": (-32000.0, 32000.0),
    "VOLTAGE": (0.0, 300.0),
    "CURRENT": (-3200.0, 3200.0),
    "MOTOR_TEMP": (-40.0, 210.0),
    "STEER_POS": (-1800.0, 1800.0),
}


def _enum(value: str, allowed: set[str], name: str) -> str:
    result = str(value).strip().upper()
    if result not in allowed:
        raise ValueError(f"{name} must be one of {sorted(allowed)}")
    return result


def _number(value, minimum: float, maximum: float, name: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(result) or not minimum <= result <= maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}]")
    return result


def _format_number(value: float) -> str:
    return format(value, ".9g")


def build_set_mode(mode: str) -> str:
    return f"SET MODE {_enum(mode, MODES, 'mode')}"


def build_set_gear(gear: str) -> str:
    return f"SET GEAR {_enum(gear, GEARS, 'gear')}"


def build_set_estop(location: str, enabled: bool) -> str:
    return f"SET ESTOP {_enum(location, ESTOPS, 'estop')} {'ON' if bool(enabled) else 'OFF'}"


def build_set_numeric(field: str, value) -> str:
    field = str(field).strip().upper()
    if field not in NUMERIC_LIMITS:
        raise ValueError(f"unsupported numeric field: {field}")
    number = _number(value, *NUMERIC_LIMITS[field], field.lower())
    return f"SET {field} {_format_number(number)}"


def build_set_pedal(value) -> str:
    return build_set_numeric("PEDAL", value)


def build_set_can(controller: str, enabled: bool) -> str:
    controller = _enum(controller, CONTROLLERS, "controller")
    return f"SET {controller}_CAN {'ON' if bool(enabled) else 'OFF'}"


def build_set_plant(enabled: bool) -> str:
    return f"SET PLANT {'ON' if bool(enabled) else 'OFF'}"


def build_fault(controller: str, fault: str | None, enabled: bool) -> str:
    controller = _enum(controller, CONTROLLERS, "controller")
    suffix = "" if fault is None else f" {_enum(fault, {'DROP_TX'}, 'fault')}"
    return f"FAULT {controller}{suffix} {'ON' if bool(enabled) else 'OFF'}"


def validate_command(command: str) -> str:
    tokens = str(command).strip().upper().split()
    if tokens in (["STATUS"], ["HELP"]):
        return tokens[0]
    if len(tokens) == 3 and tokens[:2] == ["SET", "MODE"]:
        return build_set_mode(tokens[2])
    if len(tokens) == 3 and tokens[:2] == ["SET", "GEAR"]:
        return build_set_gear(tokens[2])
    if len(tokens) == 4 and tokens[:2] == ["SET", "ESTOP"]:
        return build_set_estop(tokens[2], _enum(tokens[3], {"ON", "OFF"}, "state") == "ON")
    if len(tokens) == 3 and tokens[0] == "SET" and tokens[1] in NUMERIC_LIMITS:
        return build_set_numeric(tokens[1], tokens[2])
    if len(tokens) == 3 and tokens[:2] == ["SET", "PLANT"]:
        return build_set_plant(_enum(tokens[2], {"ON", "OFF"}, "state") == "ON")
    if len(tokens) == 3 and tokens[0] == "SET" and tokens[1] in {"EZ_CAN", "KEYA_CAN"}:
        state = _enum(tokens[2], {"ON", "OFF"}, "state") == "ON"
        return build_set_can(tokens[1].removesuffix("_CAN"), state)
    if len(tokens) == 3 and tokens[0] == "FAULT":
        return build_fault(tokens[1], None, _enum(tokens[2], {"ON", "OFF"}, "state") == "ON")
    if len(tokens) == 4 and tokens[0] == "FAULT":
        return build_fault(tokens[1], tokens[2], _enum(tokens[3], {"ON", "OFF"}, "state") == "ON")
    raise ValueError("unsupported STM_TEST command")


_BASE_STATUS_CONVERTERS: Dict[str, Callable[[str], object]] = {
    "mode": lambda value: _enum(value, MODES, "mode"),
    "gear": lambda value: _enum(value, {"FWD", "REV"}, "gear"),
    "pedal": float,
    "estop_dash": lambda value: bool(int(value)),
    "estop_side": lambda value: bool(int(value)),
    "estop_rf": lambda value: bool(int(value)),
    "speed": float,
    "rpm": float,
    "voltage": float,
    "current": float,
    "motor_temp": float,
    "brake_period_us": int,
    "brake_us": int,
    "brake": lambda value: _enum(value, BRAKE_STATES, "brake"),
    "pwm_lost": lambda value: bool(int(value)),
    "steer": float,
    "steer_target": float,
    "ez_rx": int,
    "ez_tx": int,
    "keya_rx": int,
    "keya_tx": int,
    "ez_can": lambda value: bool(int(value)),
    "keya_can": lambda value: bool(int(value)),
    "ez_ok": lambda value: bool(int(value)),
    "keya_ok": lambda value: bool(int(value)),
    "ez_fault": lambda value: bool(int(value)),
    "keya_fault": lambda value: bool(int(value)),
}

_EXTENDED_STATUS_CONVERTERS: Dict[str, Callable[[str], object]] = {
    "protocol": int,
    "plant": lambda value: bool(int(value)),
    "ez_target_rpm": float,
    "ez_running": lambda value: bool(int(value)),
    "ez_speed_control": lambda value: bool(int(value)),
    "ez_command_rpm": float,
    "ez_command_current": float,
    "ez_rx_age_ms": int,
    "keya_enabled": lambda value: bool(int(value)),
    "keya_rx_age_ms": int,
}

_PROTOCOL3_STATUS_CONVERTERS: Dict[str, Callable[[str], object]] = {
    "ez_busoff": lambda value: bool(int(value)),
    "ez_errpass": lambda value: bool(int(value)),
    "ez_warn": lambda value: bool(int(value)),
    "ez_tec": int,
    "ez_rec": int,
    "keya_busoff": lambda value: bool(int(value)),
    "keya_errpass": lambda value: bool(int(value)),
    "keya_warn": lambda value: bool(int(value)),
    "keya_tec": int,
    "keya_rec": int,
}

_STATUS_CONVERTERS = {
    **_BASE_STATUS_CONVERTERS,
    **_EXTENDED_STATUS_CONVERTERS,
    **_PROTOCOL3_STATUS_CONVERTERS,
}

_LEGACY_DEFAULTS = {
    "protocol": 1,
    "plant": False,
    "ez_target_rpm": 0.0,
    "ez_running": False,
    "ez_speed_control": False,
    "ez_command_rpm": 0.0,
    "ez_command_current": 0.0,
    "ez_rx_age_ms": 4294967295,
    "keya_enabled": False,
    "keya_rx_age_ms": 4294967295,
}


def parse_status(line: str, *, received_monotonic: float | None = None) -> HilStatus:
    tokens = str(line).strip().split()
    if not tokens or tokens[0] != "STATUS":
        raise ValueError("line is not a STATUS response")
    values: Dict[str, object] = {}
    for token in tokens[1:]:
        if "=" not in token:
            raise ValueError(f"malformed STATUS token: {token}")
        key, raw_value = token.split("=", 1)
        converter = _STATUS_CONVERTERS.get(key)
        if converter is None:
            raise ValueError(f"unknown STATUS key: {key}")
        try:
            values[key] = converter(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid STATUS value for {key}: {raw_value}") from exc
    missing = set(_BASE_STATUS_CONVERTERS) - set(values)
    if missing:
        raise ValueError(f"missing STATUS keys: {sorted(missing)}")
    protocol = int(values.get("protocol", 1))
    if protocol not in {1, 2, 3}:
        raise ValueError(f"unsupported STATUS protocol: {protocol}")
    if protocol >= 2:
        missing = set(_EXTENDED_STATUS_CONVERTERS) - set(values)
        if missing:
            raise ValueError(f"missing protocol {protocol} STATUS keys: {sorted(missing)}")
    if protocol >= 3:
        missing = set(_PROTOCOL3_STATUS_CONVERTERS) - set(values)
        if missing:
            raise ValueError(f"missing protocol {protocol} STATUS keys: {sorted(missing)}")
    else:
        # Protocol 1 firmware predates the plant and command-observation fields.
        # Optional extension keys are accepted for the short-lived pre-v2 build.
        for key, default in _LEGACY_DEFAULTS.items():
            values.setdefault(key, default)
    values["received_monotonic"] = time.monotonic() if received_monotonic is None else received_monotonic
    return HilStatus(**values)


def parse_response(line: str) -> ProtocolResponse:
    raw = str(line).strip()
    if raw.startswith("OK "):
        return ProtocolResponse("ok", raw, ok_command=raw[3:])
    if raw.startswith("ERR "):
        return ProtocolResponse("error", raw, error_reason=raw[4:])
    if raw.startswith("STATUS "):
        return ProtocolResponse("status", raw, status=parse_status(raw))
    raise ValueError("unknown STM_TEST response")


def desired_commands(state: HilDesiredState, *, protocol_version: int = 2) -> list[str]:
    commands = []
    # Disable the closed loop before restoring direct motion. Otherwise the
    # running plant can overwrite SET SPEED/RPM before SET PLANT OFF arrives.
    if protocol_version >= 2 and not state.plant_enabled:
        commands.append(build_set_plant(False))
    commands.extend([
        build_set_mode(state.mode),
        build_set_gear(state.gear),
        build_set_estop("DASH", state.estop_dashboard),
        build_set_estop("SIDE", state.estop_side),
        build_set_estop("RF", state.estop_rf),
        build_set_pedal(state.pedal),
        build_set_numeric(state.motion_source, state.speed if state.motion_source == "SPEED" else state.rpm),
        build_set_numeric("VOLTAGE", state.voltage),
        build_set_numeric("CURRENT", state.current),
        build_set_numeric("MOTOR_TEMP", state.motor_temp),
        build_set_numeric("STEER_POS", state.steering_position),
        build_set_can("EZ", state.ez_can_enabled),
        build_set_can("KEYA", state.keya_can_enabled),
        build_fault("EZ", "DROP_TX", state.ez_drop_tx),
        build_fault("KEYA", "DROP_TX", state.keya_drop_tx),
        build_fault("EZ", None, state.ez_fault),
        build_fault("KEYA", None, state.keya_fault),
    ])
    if protocol_version >= 2 and state.plant_enabled:
        # The injected motion above is the initial condition for the plant.
        commands.append(build_set_plant(True))
    return commands


def update_desired(state: HilDesiredState, command: str) -> None:
    tokens = validate_command(command).split()
    if tokens[0] == "SET" and tokens[1] == "MODE": state.mode = tokens[2]
    elif tokens[0] == "SET" and tokens[1] == "GEAR": state.gear = tokens[2]
    elif tokens[:2] == ["SET", "ESTOP"]:
        attr = {"DASH": "estop_dashboard", "SIDE": "estop_side", "RF": "estop_rf"}[tokens[2]]
        setattr(state, attr, tokens[3] == "ON")
    elif tokens[0] == "SET" and tokens[1] in NUMERIC_LIMITS:
        attrs = {"PEDAL": "pedal", "SPEED": "speed", "RPM": "rpm", "VOLTAGE": "voltage",
                 "CURRENT": "current", "MOTOR_TEMP": "motor_temp", "STEER_POS": "steering_position"}
        setattr(state, attrs[tokens[1]], float(tokens[2]))
        if tokens[1] in {"SPEED", "RPM"}: state.motion_source = tokens[1]
    elif tokens[:2] == ["SET", "PLANT"]:
        state.plant_enabled = tokens[2] == "ON"
    elif tokens[0] == "SET" and tokens[1].endswith("_CAN"):
        setattr(state, "ez_can_enabled" if tokens[1] == "EZ_CAN" else "keya_can_enabled", tokens[2] == "ON")
    elif tokens[0] == "FAULT":
        controller = tokens[1].lower()
        if len(tokens) == 4: setattr(state, f"{controller}_drop_tx", tokens[3] == "ON")
        else: setattr(state, f"{controller}_fault", tokens[2] == "ON")


def desired_status_mismatches(state: HilDesiredState, status: HilStatus) -> list[str]:
    """Return observable desired fields that disagree with the latest STATUS."""
    mismatches = []

    def exact(name: str, actual, desired) -> None:
        if actual != desired:
            mismatches.append(f"{name}: actual={actual!r} desired={desired!r}")

    def close(name: str, actual: float, desired: float, tolerance: float) -> None:
        if not math.isclose(float(actual), float(desired), abs_tol=tolerance):
            mismatches.append(f"{name}: actual={actual!r} desired={desired!r}")

    exact("mode", status.mode, state.mode)
    exact("gear", status.gear, GEAR_STATUS_NAMES[state.gear])
    close("pedal", status.pedal, state.pedal, PEDAL_MATCH_TOLERANCE)
    exact("estop_dash", status.estop_dash, state.estop_dashboard)
    exact("estop_side", status.estop_side, state.estop_side)
    exact("estop_rf", status.estop_rf, state.estop_rf)
    if status.protocol >= 2:
        exact("plant", status.plant, state.plant_enabled)
    # With the plant running, motion follows the STM_A command rather than the
    # last injected value, so comparing it against desired would always differ.
    if not (state.plant_enabled and status.protocol >= 2):
        if state.motion_source == "SPEED":
            close("speed", status.speed, state.speed, 0.02)
        else:
            close("rpm", status.rpm, state.rpm, 1.0)
    close("voltage", status.voltage, state.voltage, 0.11)
    close("current", status.current, state.current, 0.11)
    close("motor_temp", status.motor_temp, state.motor_temp, 0.11)
    close("steer_target", status.steer_target, state.steering_position, 0.02)
    exact("ez_can", status.ez_can, state.ez_can_enabled)
    exact("keya_can", status.keya_can, state.keya_can_enabled)
    exact("ez_fault", status.ez_fault, state.ez_fault)
    exact("keya_fault", status.keya_fault, state.keya_fault)
    # DROP_TX flags are intentionally absent from the firmware STATUS contract.
    return mismatches
