"""Mock ROS telemetry for the AUTO design preview, separate from vehicle CAN."""

from dataclasses import dataclass, replace
import math
from pathlib import Path

import yaml


_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[3]
    / "software"
    / "src"
    / "lateral_ctrl"
    / "config"
    / "steering_motor_calibration.yaml"
)

with _CALIBRATION_PATH.open("r", encoding="utf-8") as stream:
    _CONFIG = yaml.safe_load(stream)

COUNT_SCALE = float(_CONFIG["count_scale"])
COEFFICIENTS = tuple(
    float(_CONFIG[f"equivalent_k{index}_deg"])
    for index in range(1, 6)
)
COMMAND_COUNT_LIMIT = int(_CONFIG["command_count_limit"])


def count_to_equivalent_deg(count: int) -> float:
    value = float(count) / COUNT_SCALE
    k1, k2, k3, k4, k5 = COEFFICIENTS
    return value * (k1 + value * (k2 + value * (k3 + value * (k4 + value * k5))))


def equivalent_deg_to_count(equivalent_deg: float) -> int:
    """Mirror SteeringAngleConverter::equivalentDegToCount for preview data."""
    if not math.isfinite(equivalent_deg):
        raise ValueError("steering angle must be finite")
    minimum = count_to_equivalent_deg(-COMMAND_COUNT_LIMIT)
    maximum = count_to_equivalent_deg(COMMAND_COUNT_LIMIT)
    target = max(minimum, min(maximum, equivalent_deg))
    lower = -COMMAND_COUNT_LIMIT
    upper = COMMAND_COUNT_LIMIT
    while lower < upper:
        middle = lower + (upper - lower) // 2
        if count_to_equivalent_deg(middle) < target:
            lower = middle + 1
        else:
            upper = middle
    upper_candidate = lower
    lower_candidate = max(-COMMAND_COUNT_LIMIT, upper_candidate - 1)
    return (
        lower_candidate
        if abs(count_to_equivalent_deg(lower_candidate) - target)
        <= abs(count_to_equivalent_deg(upper_candidate) - target)
        else upper_candidate
    )


@dataclass(frozen=True)
class DriveTelemetry:
    """Consumed /drive/status fields; freshness belongs to its ROS receiver."""
    fresh: bool = True
    command_received: bool = False
    command_timeout: bool = False
    part1_valid: bool = False
    part2_valid: bool = False
    fault_bits_valid: bool = False
    fault_bits: int = 0
    tractive_commanded: bool = False
    tractive_feedback_valid: bool = False
    tractive_feedback: bool = False
    stop_request: bool = False


@dataclass(frozen=True)
class BrakeTelemetry:
    """STM_B applied output request, not a physical brake measurement."""
    fresh: bool = True
    brake_commanded: bool = False
    stop_reason_bits: int = 0


@dataclass(frozen=True)
class AutoTelemetry:
    host_state: str = "WAIT"
    gnss_fix: str = "WAIT"
    gnss_correction_lost: bool = False
    vehicle_heading_deg: float | None = None
    target_heading_deg: float | None = None
    lidar_state: str = "WAIT"
    camera_left_state: str = "WAIT"
    camera_right_state: str = "WAIT"
    drive: DriveTelemetry | None = None
    brake: BrakeTelemetry | None = None
    target_speed_kmh: float | None = None
    steering_state: str = "WAIT"
    steering_status_fresh: bool | None = None
    steering_command_timeout: bool = False
    steering_mode_timeout: bool = False
    steering_tracking_warning: bool = False
    steering_tracking_fault: bool = False
    steering_current_a: float | None = None
    steering_enabled_commanded: bool | None = None
    steering_communication_ok: bool | None = None
    steering_feedback_valid: bool | None = None
    steering_target_motor_count: int | None = None
    steering_actual_motor_count: int | None = None
    steering_disable_reason: str = ""
    steering_fault_bits: int | None = None
    activity: str = "WAITING FOR ROS DATA"


AUTO_CASE_LABELS = {
    "healthy": "Normal / route following",
    "heading_wait": "Heading unavailable",
    "heading_wrap": "Heading crossing north",
    "low_speed": "Low speed / enable held",
    "launch": "Rolling start / enable request",
    "steering_trip": "Current spike then motor fault",
    "waiting": "Host not received",
    "rtk_float": "GNSS RTK float",
    "gnss_fix": "GNSS ordinary fix",
    "no_fix": "GNSS no fix",
    "rtcm_lost": "Correction data lost",
    "lidar_lost": "LiDAR points stopped",
    "camera_left_lost": "Left camera stopped",
    "camera_right_lost": "Right camera stopped",
    "drive_feedback_lost": "Drive feedback lost / STM publishing",
    "drive_mismatch": "Drive RUN command / feedback mismatch",
    "drive_status_lost": "Drive status stopped",
    "brake_status_lost": "Brake status stopped",
    "brake_command_lost": "Brake command invalid / STM publishing",
    "steering_status_lost": "Steering status stopped",
    "steering_tracking_fault": "Steering tracking fault / feedback live",
    "target_heading_lost": "Target heading unavailable",
    "command_lost": "Vehicle command lost",
    "steering_lost": "Steering feedback lost",
    "steering_fault": "Steering motor fault",
    "host_lost": "Host telemetry lost",
    "can_lost": "Dashboard CAN lost",
    "estop": "Physical E-stop",
    "brake": "Autonomous braking",
}


def auto_telemetry(case: str, elapsed: float) -> AutoTelemetry:
    """Named display examples; these do not define vehicle safety thresholds."""
    drive = DriveTelemetry(
        command_received=True, part1_valid=True, part2_valid=True,
        fault_bits_valid=True, tractive_commanded=True,
        tractive_feedback_valid=True, tractive_feedback=True,
    )
    base = AutoTelemetry(
        host_state="LIVE", gnss_fix="RTK FIXED",
        vehicle_heading_deg=42.0 + 6.0 * math.sin(elapsed * .3),
        target_heading_deg=62.0 + 7.0 * math.sin(elapsed * .3),
        lidar_state="LIVE", camera_left_state="LIVE", camera_right_state="LIVE",
        drive=drive, brake=BrakeTelemetry(), target_speed_kmh=12.0,
        steering_state="ACTIVE", steering_status_fresh=True,
        steering_current_a=3.0, steering_enabled_commanded=True,
        steering_communication_ok=True, steering_feedback_valid=True,
        steering_target_motor_count=equivalent_deg_to_count(12.0),
        steering_actual_motor_count=equivalent_deg_to_count(7.8),
        steering_fault_bits=0,
        activity="FOLLOW ROUTE",
    )
    changes = {
        "healthy": {},
        "heading_wait": {"vehicle_heading_deg": None, "steering_state": "WAIT CMD",
                         "steering_enabled_commanded": False, "steering_current_a": 0.0,
                         "steering_target_motor_count": None, "steering_actual_motor_count": 0,
                         "activity": "START CHECK"},
        "heading_wrap": {"vehicle_heading_deg": 358.0, "target_heading_deg": 2.0},
        "waiting": {},
        "rtk_float": {"gnss_fix": "RTK FLOAT"},
        "gnss_fix": {"gnss_fix": "FIX"},
        "no_fix": {"gnss_fix": "NO FIX"},
        "rtcm_lost": {"gnss_fix": "RTK FLOAT", "gnss_correction_lost": True},
        "lidar_lost": {"lidar_state": "LOST"},
        "camera_left_lost": {"camera_left_state": "LOST"},
        "camera_right_lost": {"camera_right_state": "LOST"},
        "drive_feedback_lost": {"drive": replace(drive, part2_valid=False, tractive_feedback_valid=False, fault_bits_valid=False)},
        "drive_mismatch": {"drive": replace(drive, tractive_feedback=False)},
        "drive_status_lost": {"drive": replace(drive, fresh=False)},
        "brake_status_lost": {"brake": BrakeTelemetry(fresh=False)},
        "brake_command_lost": {"brake": BrakeTelemetry(stop_reason_bits=8)},
        "steering_status_lost": {"steering_status_fresh": False},
        "steering_tracking_fault": {"steering_tracking_fault": True, "steering_actual_motor_count": 0},
        "target_heading_lost": {"target_heading_deg": None},
        "command_lost": {"drive": replace(drive, command_timeout=True), "steering_command_timeout": True,
                         "brake": BrakeTelemetry(stop_reason_bits=8),
                         "target_speed_kmh": None, "steering_state": "WAIT CMD",
                         "steering_enabled_commanded": False, "steering_current_a": 0.0,
                         "steering_disable_reason": "COMMAND TIMEOUT"},
        "steering_lost": {"steering_state": "FB LOST",
                          "steering_current_a": None, "steering_enabled_commanded": False,
                          "steering_feedback_valid": False, "steering_actual_motor_count": None,
                          "steering_disable_reason": "POSITION FEEDBACK LOST"},
        "steering_fault": {"steering_state": "FAULT", "steering_current_a": 0.0,
                           "steering_enabled_commanded": False, "steering_fault_bits": 2,
                           "steering_disable_reason": "MOTOR FAULT"},
        "host_lost": {},
        "can_lost": {},
        "estop": {"steering_state": "E-STOP", "activity": "PHYSICAL STOP",
                  "drive": replace(drive, stop_request=True, tractive_commanded=False, tractive_feedback=False),
                  "brake": BrakeTelemetry(brake_commanded=True, stop_reason_bits=4),
                  "steering_current_a": 0.0, "steering_enabled_commanded": False,
                  "steering_disable_reason": "E-STOP"},
        "brake": {"steering_state": "BRAKE", "activity": "PLANNED STOP",
                  "drive": replace(drive, stop_request=True, tractive_commanded=False, tractive_feedback=False),
                  "brake": BrakeTelemetry(brake_commanded=True, stop_reason_bits=1),
                  "steering_current_a": 0.0, "steering_enabled_commanded": False,
                  "target_speed_kmh": 0.0, "steering_disable_reason": "COMMAND BRAKE"},
    }
    if case == "low_speed" or (case == "launch" and elapsed % 9.0 < 3.0):
        return replace(base, steering_state="WAIT SPEED", steering_current_a=0.0,
                       steering_enabled_commanded=False,
                       steering_target_motor_count=0, steering_actual_motor_count=0,
                       steering_disable_reason="SPEED BELOW 1 KM/H", activity="ROLLING START")
    if case == "launch":
        return replace(base, steering_current_a=float(2 + round(abs(math.sin(elapsed)) * 3)),
                       activity="ROLLING START")
    if case == "steering_trip":
        phase = elapsed % 9.0
        # Synthetic illustration of the reported spike -> zero-current sequence.
        # A fault is an explicit input here, never inferred from an amp threshold.
        if phase < 2.0:
            return replace(base, steering_target_motor_count=equivalent_deg_to_count(22.0),
                           steering_actual_motor_count=0, activity="START SEQUENCE")
        if phase < 3.0:
            return replace(base, steering_current_a=float(min(20, round(8 + 24 * (phase - 2)))),
                           steering_target_motor_count=equivalent_deg_to_count(22.0),
                           steering_actual_motor_count=0,
                           activity="START SEQUENCE")
        return replace(base, **changes["steering_fault"],
                       steering_target_motor_count=equivalent_deg_to_count(22.0),
                       steering_actual_motor_count=0,
                       activity="START SEQUENCE")
    if case == "waiting":
        return AutoTelemetry()
    if case == "host_lost":
        return replace(base, host_state="LOST")
    return replace(base, **changes[case])
