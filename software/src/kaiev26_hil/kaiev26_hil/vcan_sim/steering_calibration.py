"""Shared KEYA steering calibration for the vcan simulator."""

from __future__ import annotations

import math
from pathlib import Path

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


def _calibration_path() -> Path:
    try:
        return (
            Path(get_package_share_directory("lateral_ctrl"))
            / "config"
            / "steering_motor_calibration.yaml"
        )
    except PackageNotFoundError:
        return (
            Path(__file__).resolve().parents[3]
            / "lateral_ctrl"
            / "config"
            / "steering_motor_calibration.yaml"
        )


with _calibration_path().open("r", encoding="utf-8") as stream:
    _CONFIG = yaml.safe_load(stream)

COUNT_SCALE = float(_CONFIG["count_scale"])
EQUIVALENT_K1_DEG = float(_CONFIG["equivalent_k1_deg"])
EQUIVALENT_K2_DEG = float(_CONFIG["equivalent_k2_deg"])
EQUIVALENT_K3_DEG = float(_CONFIG["equivalent_k3_deg"])
EQUIVALENT_K4_DEG = float(_CONFIG["equivalent_k4_deg"])
EQUIVALENT_K5_DEG = float(_CONFIG["equivalent_k5_deg"])
COMMAND_COUNT_LIMIT = int(_CONFIG["command_count_limit"])
COUNTS_PER_REV = float(_CONFIG["counts_per_rev"])

if (
    not all(
        math.isfinite(value)
        for value in (
            COUNT_SCALE,
            EQUIVALENT_K1_DEG,
            EQUIVALENT_K2_DEG,
            EQUIVALENT_K3_DEG,
            EQUIVALENT_K4_DEG,
            EQUIVALENT_K5_DEG,
            COUNTS_PER_REV,
        )
    )
    or COUNT_SCALE <= 0.0
    or COMMAND_COUNT_LIMIT <= 0
    or COUNTS_PER_REV <= 0.0
):
    raise RuntimeError("invalid steering calibration parameters")


def count_to_equivalent_deg(count: float) -> float:
    v = float(count) / COUNT_SCALE
    return v * (
        EQUIVALENT_K1_DEG
        + v * (
            EQUIVALENT_K2_DEG
            + v * (
                EQUIVALENT_K3_DEG
                + v * (EQUIVALENT_K4_DEG + v * EQUIVALENT_K5_DEG)
            )
        )
    )


COMMAND_MIN_ANGLE_DEG = count_to_equivalent_deg(-COMMAND_COUNT_LIMIT)
COMMAND_MAX_ANGLE_DEG = count_to_equivalent_deg(COMMAND_COUNT_LIMIT)

_previous_angle_deg = count_to_equivalent_deg(-COMMAND_COUNT_LIMIT)
for _count in range(-COMMAND_COUNT_LIMIT + 1, COMMAND_COUNT_LIMIT + 1):
    _angle_deg = count_to_equivalent_deg(_count)
    if _angle_deg <= _previous_angle_deg:
        raise RuntimeError(
            "steering calibration must increase over the command range"
        )
    _previous_angle_deg = _angle_deg


def equivalent_deg_to_count(equivalent_deg: float) -> int:
    if not math.isfinite(equivalent_deg):
        raise ValueError("steering angle command must be finite")
    target = max(
        COMMAND_MIN_ANGLE_DEG,
        min(COMMAND_MAX_ANGLE_DEG, equivalent_deg),
    )
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
    if abs(count_to_equivalent_deg(lower_candidate) - target) <= abs(
        count_to_equivalent_deg(upper_candidate) - target
    ):
        return lower_candidate
    return upper_candidate


def count_to_motor_deg(count: float) -> float:
    return float(count) * 360.0 / COUNTS_PER_REV


def motor_deg_to_count(motor_deg: float) -> float:
    return float(motor_deg) * COUNTS_PER_REV / 360.0
