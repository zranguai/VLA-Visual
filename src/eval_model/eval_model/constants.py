from __future__ import annotations

import numpy as np


SIGNAL_NAMES = (
    *(f"left_joint_{index}" for index in range(7)),
    "left_gripper",
    *(f"right_joint_{index}" for index in range(7)),
    "right_gripper",
)
DATASET_SIGNAL_NAMES = (
    *(f"left_joint{index}" for index in range(1, 8)),
    "left_gripper",
    *(f"right_joint{index}" for index in range(1, 8)),
    "right_gripper",
)
JOINT_INDICES = np.asarray([*range(7), *range(8, 15)], dtype=np.int64)
LEFT_JOINT_INDICES = np.arange(0, 7, dtype=np.int64)
RIGHT_JOINT_INDICES = np.arange(8, 15, dtype=np.int64)
GRIPPER_INDICES = np.asarray([7, 15], dtype=np.int64)

JOINT_MIN_ONE_ARM = np.asarray(
    [-2.94, -1.62, -2.94, -3.00, -2.94, -0.35, -1.70], dtype=np.float64
)
JOINT_MAX_ONE_ARM = np.asarray(
    [2.94, 1.62, 2.94, 0.01, 2.94, 1.40, 1.70], dtype=np.float64
)
ACTION_MIN = np.concatenate(
    [JOINT_MIN_ONE_ARM, np.asarray([0.0]), JOINT_MIN_ONE_ARM, np.asarray([0.0])]
)
ACTION_MAX = np.concatenate(
    [JOINT_MAX_ONE_ARM, np.asarray([100.0]), JOINT_MAX_ONE_ARM, np.asarray([100.0])]
)
