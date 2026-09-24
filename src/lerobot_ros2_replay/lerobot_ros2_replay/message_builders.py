from __future__ import annotations

from array import array
from typing import Any

import numpy as np
from arm_interfaces.msg import ArmStatus, AutonomyArmCommand
from custom_image_msg.msg import Image1m


IMAGE_CAPACITY = int(Image1m.DATA_MAX_SIZE)


def _set_custom_string(target: Any, text: str) -> None:
    encoded = text.encode("utf-8")
    if len(encoded) > int(target.MAX_SIZE):
        raise ValueError(f"custom string is too long: {text!r}")
    target.data = array("B", encoded + b"\x00" * (256 - len(encoded)))
    target.size = len(encoded)


def build_arm_status(stamp: Any, state: np.ndarray, frame_id: str = "lerobot_replay") -> ArmStatus:
    values = np.asarray(state, dtype=np.float32)
    if values.shape != (16,):
        raise ValueError(f"state must have shape (16,), got {values.shape}")
    msg = ArmStatus()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for index in range(7):
        msg.left_arm[index].joint_status[0] = float(values[index])
        msg.right_arm[index].joint_status[0] = float(values[index + 8])
    msg.gripper[0].joint_status[0] = float(values[7])
    msg.gripper[1].joint_status[0] = float(values[15])
    msg.left_arm_pose.orientation.w = 1.0
    msg.right_arm_pose.orientation.w = 1.0
    return msg


def build_action(stamp: Any, action: np.ndarray, frame_id: str = "lerobot_replay") -> AutonomyArmCommand:
    values = np.asarray(action, dtype=np.float32)
    if values.shape != (16,):
        raise ValueError(f"action must have shape (16,), got {values.shape}")
    msg = AutonomyArmCommand()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for index in range(7):
        msg.left_command[index] = float(values[index])
        msg.right_command[index] = float(values[index + 8])
    msg.gripper_command[0] = float(values[7])
    msg.gripper_command[1] = float(values[15])
    return msg


def build_image(stamp: Any, image_rgb: np.ndarray, frame_id: str) -> Image1m:
    image = np.asarray(image_rgb)
    if image.ndim != 3 or image.shape[2] != 3 or image.dtype != np.uint8:
        raise ValueError(f"image must be HxWx3 uint8 RGB, got {image.shape}/{image.dtype}")
    height, width, _ = image.shape
    payload = image.tobytes(order="C")
    if len(payload) > IMAGE_CAPACITY:
        raise ValueError(
            f"image payload {len(payload)} bytes exceeds Image1m capacity {IMAGE_CAPACITY}"
        )

    msg = Image1m()
    msg.header.stamp = stamp
    _set_custom_string(msg.header.frame_id, frame_id)
    msg.height = height
    msg.width = width
    _set_custom_string(msg.encoding, "rgb8")
    msg.is_bigendian = 0
    msg.step = width * 3
    msg.data = array("B", payload + b"\x00" * (IMAGE_CAPACITY - len(payload)))
    return msg
