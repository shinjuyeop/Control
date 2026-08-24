"""Data models shared by the serial bridge, GUI, and scenario runner."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from typing import Iterable, Optional, TypeVar, get_type_hints


_Model = TypeVar("_Model")


def decode_json_fields(
    payload: str, model: type[_Model], required_fields: Iterable[str]
) -> dict[str, object]:
    """Decode and canonicalize only the fields consumed at a JSON boundary."""
    try:
        raw = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise ValueError("payload is not valid JSON") from exc
    if not isinstance(raw, dict):
        raise ValueError("payload root must be an object")

    names = tuple(required_fields)
    missing = sorted(set(names) - set(raw))
    if missing:
        raise ValueError(f"payload is missing fields: {missing}")

    hints = get_type_hints(model)
    canonical: dict[str, object] = {}
    for name in names:
        value = raw[name]
        expected = hints[name]
        if expected is bool:
            if not isinstance(value, bool):
                raise ValueError(f"payload field {name} must be boolean")
        elif expected is int:
            if not isinstance(value, int) or isinstance(value, bool):
                raise ValueError(f"payload field {name} must be an integer")
        elif expected is float:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"payload field {name} must be numeric")
            value = float(value)
            if not math.isfinite(value):
                raise ValueError(f"payload field {name} must be finite")
        elif expected is str and not isinstance(value, str):
            raise ValueError(f"payload field {name} must be a string")
        canonical[name] = value

    return canonical


@dataclass(frozen=True)
class HilStatus:
    mode: str
    gear: str
    pedal: float
    estop_dash: bool
    estop_side: bool
    estop_rf: bool
    speed: float
    rpm: float
    voltage: float
    current: float
    motor_temp: float
    brake_period_us: int
    brake_us: int
    brake: str
    pwm_lost: bool
    steer: float
    steer_target: float
    ez_rx: int
    ez_tx: int
    keya_rx: int
    keya_tx: int
    ez_can: bool
    keya_can: bool
    ez_ok: bool
    keya_ok: bool
    ez_fault: bool
    keya_fault: bool
    plant: bool
    ez_target_rpm: float
    ez_running: bool = False
    ez_speed_control: bool = False
    ez_command_rpm: float = 0.0
    ez_command_current: float = 0.0
    ez_rx_age_ms: int = 4294967295
    keya_enabled: bool = False
    keya_rx_age_ms: int = 4294967295
    # FDCAN protocol status per bus, protocol 3 and later. ez_ok/keya_ok only
    # report peripheral init, so a stalled TX shows up here and not there.
    ez_busoff: bool = False
    ez_errpass: bool = False
    ez_warn: bool = False
    ez_tec: int = 0
    ez_rec: int = 0
    keya_busoff: bool = False
    keya_errpass: bool = False
    keya_warn: bool = False
    keya_tec: int = 0
    keya_rec: int = 0
    protocol: int = 1
    received_monotonic: float = 0.0

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


@dataclass
class HilDesiredState:
    mode: str = "MANUAL"
    gear: str = "FORWARD"
    pedal: float = 0.0
    estop_dashboard: bool = False
    estop_side: bool = False
    estop_rf: bool = False
    speed: float = 0.0
    rpm: float = 0.0
    motion_source: str = "RPM"
    voltage: float = 0.0
    current: float = 0.0
    motor_temp: float = 0.0
    steering_position: float = 0.0
    ez_can_enabled: bool = True
    keya_can_enabled: bool = True
    # Closed-loop traction is the default the bench runs in: the bridge replays
    # this on connect, so STM_TEST leaves its plant-off power-on state without
    # the operator having to select it. Safe State returns here too. The KEYA
    # steering emulator needs no equivalent -- it follows its CAN2 position
    # command whenever the motor is enabled, with no open-loop mode to pick.
    plant_enabled: bool = True
    ez_drop_tx: bool = False
    keya_drop_tx: bool = False
    ez_fault: bool = False
    keya_fault: bool = False

    def to_json(self) -> str:
        return json.dumps(asdict(self), sort_keys=True)


@dataclass(frozen=True)
class ProtocolResponse:
    kind: str
    raw: str
    ok_command: Optional[str] = None
    error_reason: Optional[str] = None
    status: Optional[HilStatus] = None
