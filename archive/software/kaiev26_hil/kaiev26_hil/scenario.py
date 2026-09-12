"""Pure YAML scenario parsing and expectation evaluation."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .models import HilDesiredState
from .protocol import (
    build_fault,
    build_set_can,
    build_set_estop,
    build_set_gear,
    build_set_mode,
    build_set_numeric,
    desired_commands,
)


HIL_FIELDS = {
    "safe_state", "mode", "gear", "pedal", "estop_dashboard", "estop_side", "estop_rf",
    "speed", "rpm", "voltage", "current", "motor_temp", "steering_position",
    "ez_can_enabled", "keya_can_enabled",
}
FAULT_FIELDS = {"ez_drop_tx", "keya_drop_tx", "ez_fault", "keya_fault"}
COMMAND_FIELDS = {"enabled", "speed_target_mps", "steering_target_rad", "brake_engage"}
SUPPORTED_TOPICS = {
    "/drive/status", "/brake/status", "/steering/status", "/vehicle/state", "/hil/status"
}


@dataclass(frozen=True)
class ScenarioStep:
    at: float
    hil: dict[str, Any]
    fault: dict[str, Any]
    command: dict[str, Any]


@dataclass(frozen=True)
class ScenarioExpectation:
    topic: str
    field: str
    after: float
    within_ms: int
    equals: Any = None
    minimum: float | None = None
    maximum: float | None = None
    bitmask: int | None = None


@dataclass(frozen=True)
class ScenarioDefinition:
    name: str
    steps: tuple[ScenarioStep, ...]
    expectations: tuple[ScenarioExpectation, ...]


def _mapping(value, name: str) -> dict:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a mapping")
    return dict(value)


def load_scenario(path: str | Path) -> ScenarioDefinition:
    path = Path(path)
    with path.open("r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict) or not str(document.get("name", "")).strip():
        raise ValueError("scenario requires a non-empty name")
    steps = []
    previous_at = -1.0
    for index, raw in enumerate(document.get("steps", [])):
        if not isinstance(raw, dict):
            raise ValueError(f"steps[{index}] must be a mapping")
        at = float(raw.get("at", -1.0))
        if at < 0.0 or at < previous_at:
            raise ValueError("scenario step times must be non-negative and ordered")
        previous_at = at
        hil = _mapping(raw.get("hil"), f"steps[{index}].hil")
        fault = _mapping(raw.get("fault"), f"steps[{index}].fault")
        command = _mapping(raw.get("command"), f"steps[{index}].command")
        unknown = (set(hil) - HIL_FIELDS) | (set(fault) - FAULT_FIELDS) | (set(command) - COMMAND_FIELDS)
        if unknown:
            raise ValueError(f"unsupported scenario fields: {sorted(unknown)}")
        steps.append(ScenarioStep(at, hil, fault, command))

    expectations = []
    for index, raw in enumerate(document.get("expect", [])):
        if not isinstance(raw, dict):
            raise ValueError(f"expect[{index}] must be a mapping")
        topic = str(raw.get("topic", ""))
        field = str(raw.get("field", ""))
        if topic not in SUPPORTED_TOPICS or not field:
            raise ValueError(f"expect[{index}] has unsupported topic or empty field")
        modes = [key for key in ("equals", "min", "max", "bitmask") if key in raw]
        if not modes:
            raise ValueError(f"expect[{index}] requires equals/min/max/bitmask")
        expectations.append(ScenarioExpectation(
            topic=topic,
            field=field,
            after=float(raw.get("after", 0.0)),
            within_ms=int(raw.get("within_ms", 500)),
            equals=raw.get("equals"),
            minimum=float(raw["min"]) if "min" in raw else None,
            maximum=float(raw["max"]) if "max" in raw else None,
            bitmask=int(raw["bitmask"], 0) if isinstance(raw.get("bitmask"), str) else raw.get("bitmask"),
        ))
    return ScenarioDefinition(str(document["name"]), tuple(steps), tuple(expectations))


def hil_commands(values: dict[str, Any], faults: dict[str, Any]) -> list[str]:
    result = []
    for field, value in values.items():
        if field == "safe_state":
            if value is not True:
                raise ValueError("safe_state must be true when specified")
            result.extend(desired_commands(HilDesiredState()))
        elif field == "mode": result.append(build_set_mode(value))
        elif field == "gear": result.append(build_set_gear(value))
        elif field.startswith("estop_"):
            location = field.removeprefix("estop_").upper()
            if location == "DASHBOARD":
                location = "DASH"
            result.append(build_set_estop(location, bool(value)))
        elif field == "ez_can_enabled": result.append(build_set_can("EZ", bool(value)))
        elif field == "keya_can_enabled": result.append(build_set_can("KEYA", bool(value)))
        else:
            protocol_field = {"motor_temp": "MOTOR_TEMP", "steering_position": "STEER_POS"}.get(field, field.upper())
            result.append(build_set_numeric(protocol_field, value))
    for field, value in faults.items():
        controller = "EZ" if field.startswith("ez_") else "KEYA"
        result.append(build_fault(controller, "DROP_TX" if field.endswith("drop_tx") else None, bool(value)))
    return result


def resolve_field(message: Any, dotted_field: str) -> Any:
    value = message
    for component in dotted_field.split("."):
        if isinstance(value, dict):
            if component not in value:
                raise KeyError(dotted_field)
            value = value[component]
        else:
            value = getattr(value, component)
    return value


def evaluate_expectation(expectation: ScenarioExpectation, message: Any) -> tuple[bool, str]:
    try:
        value = resolve_field(message, expectation.field)
    except (AttributeError, KeyError) as exc:
        return False, f"field unavailable: {expectation.field}"
    if expectation.bitmask is not None and (int(value) & expectation.bitmask) != expectation.bitmask:
        return False, f"{value!r} does not contain bitmask 0x{expectation.bitmask:X}"
    if expectation.equals is not None and value != expectation.equals:
        return False, f"{value!r} != {expectation.equals!r}"
    if expectation.minimum is not None and float(value) < expectation.minimum:
        return False, f"{value!r} < {expectation.minimum!r}"
    if expectation.maximum is not None and float(value) > expectation.maximum:
        return False, f"{value!r} > {expectation.maximum!r}"
    return True, f"observed {expectation.field}={value!r}"
