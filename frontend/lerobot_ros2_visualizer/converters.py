from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .config import SIGNAL_NAMES


@dataclass
class RateGate:
    hz: float
    last_emit_ns: int | None = None

    def due(self, timestamp_ns: int) -> bool:
        if self.hz <= 0:
            return False
        period_ns = max(1, int(1_000_000_000 / self.hz))
        if (
            self.last_emit_ns is None
            or timestamp_ns < self.last_emit_ns
            or timestamp_ns - self.last_emit_ns >= period_ns
        ):
            self.last_emit_ns = timestamp_ns
            return True
        return False


def message_timestamp_ns(msg: Any, fallback_ns: int) -> int:
    try:
        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return fallback_ns
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        return fallback_ns
    timestamp_ns = sec * 1_000_000_000 + nanosec
    return timestamp_ns if timestamp_ns > 0 else fallback_ns


def _custom_string(value: Any, path: str) -> str:
    try:
        size = int(value.size)
        data = bytes(value.data[:size])
    except (AttributeError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid {path}") from exc
    if not 0 <= size <= len(value.data):
        raise ValueError(f"invalid {path}.size: {size}")
    try:
        return data.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ValueError(f"{path} is not valid UTF-8") from exc


def decode_image1m(msg: Any) -> np.ndarray:
    height = int(msg.height)
    width = int(msg.width)
    step = int(msg.step)
    encoding = _custom_string(msg.encoding, "encoding").lower()
    if height <= 0 or width <= 0:
        raise ValueError(f"invalid image dimensions: {width}x{height}")
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding: {encoding!r}")
    row_bytes = width * 3
    if step < row_bytes:
        raise ValueError(f"image step {step} is smaller than RGB row size {row_bytes}")
    payload_size = height * step
    payload = np.frombuffer(msg.data, dtype=np.uint8)
    if payload.size < payload_size:
        raise ValueError(
            f"image payload has {payload.size} bytes, expected at least {payload_size}"
        )
    rows = payload[:payload_size].reshape(height, step)[:, :row_bytes]
    image = rows.reshape(height, width, 3)
    if encoding == "bgr8":
        image = image[:, :, ::-1]
    return np.ascontiguousarray(image)


def _joint_value(joints: Any, index: int, path: str) -> float:
    try:
        status = joints[index].joint_status
        if len(status) < 1:
            raise ValueError
        return float(status[0])
    except (IndexError, TypeError, AttributeError, ValueError) as exc:
        raise ValueError(f"missing {path}[{index}].joint_status[0]") from exc


def extract_state(msg: Any) -> dict[str, float]:
    values = {
        **{
            f"left_joint_{index}": _joint_value(msg.left_arm, index, "left_arm")
            for index in range(7)
        },
        "left_gripper": _joint_value(msg.gripper, 0, "gripper"),
        **{
            f"right_joint_{index}": _joint_value(msg.right_arm, index, "right_arm")
            for index in range(7)
        },
        "right_gripper": _joint_value(msg.gripper, 1, "gripper"),
    }
    return values


def extract_action(msg: Any) -> dict[str, float]:
    try:
        if len(msg.left_command) < 7 or len(msg.right_command) < 7:
            raise ValueError("arm commands must contain seven values per side")
        if len(msg.gripper_command) < 2:
            raise ValueError("gripper_command must contain two values")
        values = {
            **{f"left_joint_{index}": float(msg.left_command[index]) for index in range(7)},
            "left_gripper": float(msg.gripper_command[0]),
            **{f"right_joint_{index}": float(msg.right_command[index]) for index in range(7)},
            "right_gripper": float(msg.gripper_command[1]),
        }
    except (TypeError, AttributeError, ValueError, IndexError) as exc:
        raise ValueError(f"invalid autonomy arm command: {exc}") from exc
    if tuple(values) != SIGNAL_NAMES:
        raise AssertionError("action signal order no longer matches configured signal names")
    return values
