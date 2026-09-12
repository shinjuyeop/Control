"""Fixed ROS summary payload consumed by STM_B's dashboard CAN bridge."""

from __future__ import annotations

from dataclasses import dataclass
import struct


VERSION = 1
PAYLOAD_SIZE = 25

INVALID_HEADING_CENTIDEG = 0xFFFF
INVALID_I16 = -0x8000
INVALID_I32 = -0x80000000

GPS_WAIT = 0
GPS_FIXED = 1
GPS_FLOAT = 2
GPS_FIX = 3
GPS_NO_FIX = 4

STREAM_WAIT = 0
STREAM_RX = 1
STREAM_LOST = 2

DRIVE_WAIT = 0
DRIVE_RUN = 1
DRIVE_HOLD = 2
DRIVE_CMD_LOST = 3
DRIVE_NO_FEEDBACK = 4
DRIVE_RUN_MISMATCH = 5
DRIVE_FAULT = 6
DRIVE_STATUS_LOST = 7
DRIVE_WAIT_CMD = 8

STEER_WAIT = 0
STEER_ACTIVE = 1
STEER_WAIT_CMD = 2
STEER_STATUS_LOST = 3
STEER_MOTOR_FAULT = 4
STEER_CAN_LOST = 5
STEER_POS_LOST = 6
STEER_CMD_LOST = 7
STEER_TRACK_FAULT = 8
STEER_TRACK_WARN = 9
STEER_BRAKE = 10
STEER_ESTOP = 11
STEER_DISABLED = 12

BRAKE_WAIT = 0
BRAKE_OFF = 1
BRAKE_ON = 2
BRAKE_CMD_LOST = 3
BRAKE_STATUS_LOST = 4

ACTIVITY_WAIT = 0
ACTIVITY_FOLLOW_ROUTE = 1
ACTIVITY_REJOIN_ROUTE = 2
ACTIVITY_TRAFFIC_CONTROL = 3
ACTIVITY_AVOID_OBSTACLE = 4
ACTIVITY_INTERSECTION = 5
ACTIVITY_FINISH = 6
ACTIVITY_EMERGENCY = 7
ACTIVITY_AUTO_ACTIVE = 8

OUTPUT_UNKNOWN = 0
OUTPUT_OFF = 1
OUTPUT_ON = 2

FEEDBACK_UNKNOWN = 0
FEEDBACK_LOST = 1
FEEDBACK_LIVE = 2


@dataclass(frozen=True)
class AutoDashboardSummary:
    gps: int = GPS_WAIT
    lidar: int = STREAM_WAIT
    camera_left: int = STREAM_WAIT
    camera_right: int = STREAM_WAIT
    drive: int = DRIVE_WAIT
    steer: int = STEER_WAIT
    brake: int = BRAKE_WAIT
    activity: int = ACTIVITY_WAIT
    output: int = OUTPUT_UNKNOWN
    feedback: int = FEEDBACK_UNKNOWN
    heading_centideg: int = INVALID_HEADING_CENTIDEG
    target_heading_centideg: int = INVALID_HEADING_CENTIDEG
    target_speed_centi_kmh: int = INVALID_I16
    actual_motor_count: int = INVALID_I32
    steering_current_deciamp: int = INVALID_I16
    target_motor_count: int = INVALID_I32


def _pair(low: int, high: int) -> int:
    return (low & 0x0F) | ((high & 0x0F) << 4)


def pack_summary(summary: AutoDashboardSummary) -> list[int]:
    """Return the fixed 25-byte ROS payload; fields are already canonical."""
    payload = bytearray(PAYLOAD_SIZE)
    payload[0] = VERSION
    payload[1] = _pair(summary.gps, summary.lidar)
    payload[2] = _pair(summary.camera_left, summary.camera_right)
    payload[3] = _pair(summary.drive, summary.steer)
    payload[4] = _pair(summary.brake, summary.activity)
    payload[5] = _pair(summary.output, summary.feedback)
    struct.pack_into(
        '<HHhihi',
        payload,
        7,
        summary.heading_centideg,
        summary.target_heading_centideg,
        summary.target_speed_centi_kmh,
        summary.actual_motor_count,
        summary.steering_current_deciamp,
        summary.target_motor_count,
    )
    return list(payload)
