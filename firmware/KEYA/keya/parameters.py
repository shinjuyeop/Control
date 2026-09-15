"""Parameter names and the numeric representation used by KEYA V4.3."""

from dataclasses import dataclass
from decimal import Decimal, InvalidOperation


@dataclass(frozen=True)
class ParameterSpec:
    index: int
    name: str
    kind: str
    group: str
    hint: str
    normal_edit: bool = False
    read_only: bool = False


def _spec(index, name, kind, group, hint, normal=False, read_only=False):
    return ParameterSpec(index, name, kind, group, hint, normal, read_only)


PARAMETERS = {
    0: _spec(0, "TAG", "int", "Hardware", "Controller identifier", read_only=True),
    1: _spec(1, "Motor poles", "int", "Hardware", "Manufacturer UI: even number, 2–32"),
    2: _spec(2, "Rated speed", "int", "Hardware", "Manufacturer UI: 80–3500; output-shaft meaning is unconfirmed"),
    3: _spec(3, "Maximum current", "int", "Protection", "Protection current · A"),
    4: _spec(4, "Encoder PPR", "int", "Hardware", "Encoder resolution"),
    5: _spec(5, "Current Kp", "fp3", "Current loop", "Fixed point · resolution 0.001", True),
    6: _spec(6, "Current Ki", "fp3", "Current loop", "Fixed point · resolution 0.001", True),
    7: _spec(7, "Speed Kp", "fp3", "Speed loop", "Fixed point · resolution 0.001", True),
    8: _spec(8, "Speed Ki", "fp3", "Speed loop", "Fixed point · resolution 0.001", True),
    9: _spec(9, "Position Kp", "fp3", "Position loop", "Fixed point · resolution 0.001", True),
    10: _spec(10, "Position Ki", "fp3", "Position loop", "Fixed point · resolution 0.001", True),
    11: _spec(11, "Position Kd", "fp3", "Position loop", "Fixed point · resolution 0.001", True),
    12: _spec(12, "Position Kc", "fp3", "Position loop", "Fixed point · resolution 0.001", True),
    13: _spec(13, "Acceleration time", "int", "Motion", "1–200; raw × 0.1 s", True),
    14: _spec(14, "Slowdown advance", "int", "Motion", "Position Err_H · 20–100; physical unit unconfirmed", True),
    15: _spec(15, "Magnetic position compensation", "fp3", "Hardware", "Encoder calibration"),
    16: _spec(16, "Resolver position compensation", "fp3", "Hardware", "Resolver calibration"),
    17: _spec(17, "Spare / dual-channel mode", "int", "Hardware", "Meaning depends on controller model"),
    18: _spec(18, "System address", "int", "Interface", "CAN node address · decimal"),
    19: _spec(19, "Control interface", "int", "Interface", "2 CAN · 3 RS-232 · 5 CANopen"),
    20: _spec(20, "Control mode", "int", "Interface", "1 speed · 2 torque · 3 absolute position · 4 relative position"),
    21: _spec(21, "CAN bitrate", "int", "Interface", "1 125 kbit/s · 2 250 kbit/s · 3 500 kbit/s"),
    22: _spec(22, "Position feedback", "int", "Hardware", "3 single-turn · 11 multi-turn absolute encoder"),
    23: _spec(23, "Over-voltage", "int", "Protection", "Over-voltage protection · V"),
    24: _spec(24, "Under-voltage", "int", "Protection", "Under-voltage protection · V"),
    25: _spec(25, "Motor temperature", "int", "Protection", "Temperature protection · °C"),
    26: _spec(26, "Direction", "int", "Hardware", "Direction in speed and torque modes"),
    27: _spec(27, "Brake delay", "int", "Protection", "raw × 0.1 s; manual says this motor has no brake function"),
    28: _spec(28, "Overload delay", "int", "Protection", "Overload protection delay · seconds"),
    29: _spec(29, "Hall direction", "int", "Hardware", "Hall sensor direction"),
    30: _spec(30, "Deceleration time", "int", "Motion", "1–200; raw × 0.1 s", True),
    31: _spec(31, "Spare 31", "fp3", "Spare", "Reserved by manufacturer"),
    32: _spec(32, "Spare 32", "fp3", "Spare", "Reserved by manufacturer"),
    41: _spec(41, "Spare 41", "fp3", "Spare", "Reserved by manufacturer"),
    63: _spec(63, "Spare 63", "fp3", "Spare", "Reserved by manufacturer"),
}

ALIASES = {
    "accel": 13, "acceleration": 13, "decel": 30, "deceleration": 30,
    "position-kp": 9, "position-ki": 10, "position-kd": 11, "position-kc": 12,
    "speed-kp": 7, "speed-ki": 8, "current-kp": 5, "current-ki": 6,
    "max-current": 3, "overload-time": 28,
}


def parse_parameter(value):
    if isinstance(value, int):
        index = value
    else:
        text = str(value).strip().lower()
        if text in ALIASES:
            index = ALIASES[text]
        else:
            try:
                index = int(text, 16 if text.startswith("0x") else 10)
            except ValueError as exc:
                raise ValueError(f"invalid parameter: {value}") from exc
    if index not in PARAMETERS:
        raise ValueError(f"parameter {index} is not exposed by KEYA V4.3 Setup")
    return index


def encode_value(parameter, text):
    index = parse_parameter(parameter)
    spec = PARAMETERS[index]
    value_text = str(text).strip()
    try:
        if spec.kind == "int":
            value = int(value_text, 16 if value_text.lower().startswith("0x") else 10)
        else:
            number = Decimal(value_text)
            if not number.is_finite():
                raise ValueError("value must be finite")
            scaled = number * 1000
            if scaled != scaled.to_integral_value():
                raise ValueError("use at most three decimal places")
            value = int(scaled)
    except (InvalidOperation, ValueError) as exc:
        raise ValueError(f"{index:04d}: {exc}") from exc
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{index:04d}: encoded value must fit unsigned 16 bits")
    return value


def decode_value(parameter, raw):
    index = parse_parameter(parameter)
    return raw / 1000.0 if PARAMETERS[index].kind == "fp3" else raw


def format_value(parameter, raw):
    value = decode_value(parameter, raw)
    return f"{value:.3f}" if isinstance(value, float) else str(value)


def prepare_writes(values, advanced=False):
    result = {}
    for parameter, text in values.items():
        index = parse_parameter(parameter)
        spec = PARAMETERS[index]
        if spec.read_only:
            raise ValueError(f"{index:04d} {spec.name} is read-only")
        if not spec.normal_edit and not advanced:
            raise ValueError(f"{index:04d}: unlock advanced parameters to edit")
        if index in result:
            raise ValueError(f"duplicate parameter: {index:04d}")
        result[index] = encode_value(index, text)
    if not result:
        raise ValueError("no changed parameters")
    return result
