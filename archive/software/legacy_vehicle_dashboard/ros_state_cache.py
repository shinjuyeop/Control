"""Thread-safe ROS message cache and GUI-independent display helpers."""

from dataclasses import dataclass
import math
import threading
import time

from kaiev26_msgs.msg import (
    BrakeActuatorStatus,
    DriveActuatorStatus,
    SteeringActuatorStatus,
)


NO_DATA = 'NO DATA'
OK = 'OK'
STALE = 'STALE'
NA = 'N/A'

COMMAND = 'command'
DRIVE = 'drive'
BRAKE = 'brake'
STEERING = 'steering'
TOPIC_KEYS = (COMMAND, DRIVE, BRAKE, STEERING)


@dataclass(frozen=True)
class TopicSnapshot:
    message: object
    state: str
    age_ms: float


class RosStateCache:
    """Store complete message references and local monotonic receive times."""

    def __init__(self):
        self._lock = threading.Lock()
        self._messages = {key: None for key in TOPIC_KEYS}
        self._received_at = {key: None for key in TOPIC_KEYS}

    def update(self, key, message, received_at=None):
        if key not in self._messages:
            raise KeyError(f'unknown cache key: {key}')
        received_at = (
            time.monotonic() if received_at is None else float(received_at)
        )
        with self._lock:
            self._messages[key] = message
            self._received_at[key] = received_at

    def snapshot(self, stale_ms, now=None):
        """Return one consistent cache view with freshness classifications."""
        now = time.monotonic() if now is None else float(now)
        with self._lock:
            messages = dict(self._messages)
            received_at = dict(self._received_at)

        result = {}
        for key in TOPIC_KEYS:
            message = messages[key]
            received = received_at[key]
            if message is None or received is None:
                result[key] = TopicSnapshot(None, NO_DATA, math.inf)
                continue
            age_ms = max(0.0, (now - received) * 1000.0)
            state = STALE if age_ms > float(stale_ms[key]) else OK
            result[key] = TopicSnapshot(message, state, age_ms)
        return result


def _enum_text(value, mapping):
    integer = int(value)
    return mapping.get(integer, f'UNKNOWN({integer})')


def drive_mode_text(value):
    return _enum_text(
        value,
        {
            DriveActuatorStatus.MODE_MANUAL: 'MANUAL',
            DriveActuatorStatus.MODE_AUTO: 'AUTO',
        },
    )


def drive_gear_text(value):
    return _enum_text(
        value,
        {
            DriveActuatorStatus.GEAR_FORWARD: 'FORWARD',
            DriveActuatorStatus.GEAR_REVERSE: 'REVERSE',
        },
    )


def steering_state_text(value):
    return _enum_text(
        value,
        {
            SteeringActuatorStatus.STATE_INIT: 'INIT',
            SteeringActuatorStatus.STATE_DISABLED: 'DISABLED',
            SteeringActuatorStatus.STATE_WAIT_CMD: 'WAIT_CMD',
            SteeringActuatorStatus.STATE_ACTIVE: 'ACTIVE',
            SteeringActuatorStatus.STATE_ESTOP: 'ESTOP',
            SteeringActuatorStatus.STATE_BRAKE: 'BRAKE',
            SteeringActuatorStatus.STATE_CAN_LOST: 'CAN_LOST',
            SteeringActuatorStatus.STATE_FEEDBACK_LOST: 'FEEDBACK_LOST',
            SteeringActuatorStatus.STATE_FAULT: 'FAULT',
        },
    )


def steering_disable_reason_text(value):
    return _enum_text(
        value,
        {
            SteeringActuatorStatus.REASON_NONE: 'NONE',
            SteeringActuatorStatus.REASON_MODE_NOT_RECEIVED:
                'MODE_NOT_RECEIVED',
            SteeringActuatorStatus.REASON_MODE_TIMEOUT: 'MODE_TIMEOUT',
            SteeringActuatorStatus.REASON_MANUAL_MODE: 'MANUAL_MODE',
            SteeringActuatorStatus.REASON_INVALID_MODE: 'INVALID_MODE',
            SteeringActuatorStatus.REASON_ESTOP: 'ESTOP',
            SteeringActuatorStatus.REASON_COMMAND_BRAKE: 'COMMAND_BRAKE',
            SteeringActuatorStatus.REASON_STEER_COMMAND_TIMEOUT:
                'STEER_COMMAND_TIMEOUT',
            SteeringActuatorStatus.REASON_CAN_TIMEOUT: 'CAN_TIMEOUT',
            SteeringActuatorStatus.REASON_FEEDBACK_INVALID:
                'FEEDBACK_INVALID',
            SteeringActuatorStatus.REASON_MOTOR_FAULT: 'MOTOR_FAULT',
        },
    )


def brake_reason_text(bits):
    bits = int(bits)
    if bits == 0:
        return 'NONE'

    reasons = []
    known_bits = 0
    definitions = (
        (
            BrakeActuatorStatus.STOP_REASON_COMMAND_BRAKE,
            'COMMAND_BRAKE',
        ),
        (BrakeActuatorStatus.STOP_REASON_ESTOP, 'ESTOP'),
        (
            BrakeActuatorStatus.STOP_REASON_COMMAND_INVALID,
            'COMMAND_INVALID',
        ),
    )
    for mask, label in definitions:
        if bits & mask:
            reasons.append(label)
            known_bits |= int(mask)
    unknown_bits = bits & ~known_bits
    if unknown_bits:
        reasons.append(f'UNKNOWN(0x{unknown_bits:02X})')
    return ' | '.join(reasons)


def physical_estop_active(brake_message):
    if brake_message is None:
        return None
    return bool(
        int(brake_message.stop_reason_bits)
        & int(BrakeActuatorStatus.STOP_REASON_ESTOP)
    )


def rad_to_deg(value):
    return math.degrees(float(value))


def format_if_valid(valid, value, format_spec='.2f', suffix=''):
    if not bool(valid):
        return NA
    return f'{float(value):{format_spec}}{suffix}'


def bool_text(value):
    return 'YES' if bool(value) else 'NO'


def valid_bool_text(valid, value):
    return bool_text(value) if bool(valid) else NA
