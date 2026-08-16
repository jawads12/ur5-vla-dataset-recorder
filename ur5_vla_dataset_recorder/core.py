"""ROS-independent validation helpers."""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Sequence


UR5_JOINT_ORDER = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)


class ValidationError(ValueError):
    """Raised when a sample cannot safely enter the dataset."""


def reorder_joint_positions(
    names: Sequence[str],
    positions: Sequence[float],
    desired_order: Sequence[str] = UR5_JOINT_ORDER,
) -> list[float]:
    if len(names) != len(positions):
        raise ValidationError("joint name and position arrays have different lengths")
    if len(set(names)) != len(names):
        raise ValidationError("joint state contains duplicate names")

    values = dict(zip(names, positions))
    missing = [name for name in desired_order if name not in values]
    if missing:
        raise ValidationError(f"missing joints: {missing}")

    ordered = [float(values[name]) for name in desired_order]
    require_finite(ordered, "joint positions")
    return ordered


def reorder_command(
    values: Sequence[float],
    command_order: Sequence[str],
    desired_order: Sequence[str] = UR5_JOINT_ORDER,
) -> list[float]:
    if len(values) != len(command_order):
        raise ValidationError("command length does not match command_joint_order")
    return reorder_joint_positions(command_order, values, desired_order)


def require_finite(values: Iterable[float], label: str) -> None:
    if not all(math.isfinite(float(value)) for value in values):
        raise ValidationError(f"{label} contains NaN or infinity")


def stream_ages(now: float, receive_times: Mapping[str, float]) -> dict[str, float]:
    return {name: max(0.0, now - stamp) for name, stamp in receive_times.items()}


def stale_streams(ages: Mapping[str, float], limits: Mapping[str, float]) -> list[str]:
    return [name for name, age in ages.items() if age > limits[name]]

