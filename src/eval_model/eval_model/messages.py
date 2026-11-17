from __future__ import annotations

from typing import Any

import numpy as np


def message_timestamp_ns(msg: Any, fallback_ns: int) -> int:
    try:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return fallback_ns
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return fallback_ns
    value = sec * 1_000_000_000 + nanosec
    return value if value > 0 else fallback_ns


def action_vector(msg: Any) -> np.ndarray:
    try:
        left = np.asarray(msg.left_command, dtype=np.float64)
        right = np.asarray(msg.right_command, dtype=np.float64)
        gripper = np.asarray(msg.gripper_command, dtype=np.float64)
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError("invalid AutonomyArmCommand fields") from exc
    if left.size < 7 or right.size < 7 or gripper.size < 2:
        raise ValueError("AutonomyArmCommand does not contain 7+7 joints and 2 grippers")
    values = np.concatenate([left[:7], gripper[:1], right[:7], gripper[1:2]])
    if values.shape != (16,) or not np.all(np.isfinite(values)):
        raise ValueError("AutonomyArmCommand contains invalid numeric values")
    return values
