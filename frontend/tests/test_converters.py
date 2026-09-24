from array import array
from types import SimpleNamespace

import numpy as np
import pytest

from lerobot_ros2_visualizer.config import SIGNAL_NAMES
from lerobot_ros2_visualizer.converters import (
    RateGate,
    decode_image1m,
    extract_action,
    extract_state,
    message_timestamp_ns,
)


def _custom_string(text: str):
    encoded = text.encode("utf-8")
    return SimpleNamespace(size=len(encoded), data=array("B", encoded))


def _joint(value: float):
    return SimpleNamespace(joint_status=[value])


def test_decode_rgb_image_respects_row_padding():
    payload = array("B", [1, 2, 3, 4, 5, 6, 99, 99, 7, 8, 9, 10, 11, 12, 88, 88])
    msg = SimpleNamespace(
        height=2,
        width=2,
        step=8,
        encoding=_custom_string("rgb8"),
        data=payload,
    )
    image = decode_image1m(msg)
    assert image.shape == (2, 2, 3)
    assert image.dtype == np.uint8
    assert image.tolist() == [
        [[1, 2, 3], [4, 5, 6]],
        [[7, 8, 9], [10, 11, 12]],
    ]
    assert image.flags.c_contiguous


def test_decode_bgr_image_converts_to_rgb():
    msg = SimpleNamespace(
        height=1,
        width=1,
        step=3,
        encoding=_custom_string("bgr8"),
        data=array("B", [10, 20, 30]),
    )
    assert decode_image1m(msg).tolist() == [[[30, 20, 10]]]


def test_decode_image_rejects_short_payload():
    msg = SimpleNamespace(
        height=2,
        width=2,
        step=6,
        encoding=_custom_string("rgb8"),
        data=array("B", [0] * 6),
    )
    with pytest.raises(ValueError, match="expected at least 12"):
        decode_image1m(msg)


def test_state_and_action_mapping_use_left_first_layout():
    state_msg = SimpleNamespace(
        left_arm=[_joint(index) for index in range(7)],
        right_arm=[_joint(index + 8) for index in range(7)],
        gripper=[_joint(7), _joint(15)],
    )
    action_msg = SimpleNamespace(
        left_command=list(range(7)),
        right_command=list(range(8, 15)),
        gripper_command=[7, 15],
    )
    state = extract_state(state_msg)
    action = extract_action(action_msg)
    assert tuple(state) == SIGNAL_NAMES
    assert tuple(action) == SIGNAL_NAMES
    assert list(state.values()) == list(range(16))
    assert list(action.values()) == list(range(16))


def test_timestamp_falls_back_for_zero_stamp():
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=0)))
    assert message_timestamp_ns(msg, 123) == 123
    msg.header.stamp.sec = 5
    msg.header.stamp.nanosec = 6
    assert message_timestamp_ns(msg, 123) == 5_000_000_006


def test_rate_gate_limits_camera_frequency_and_recovers_after_clock_reset():
    gate = RateGate(10.0)
    assert gate.due(1_000_000_000)
    assert not gate.due(1_050_000_000)
    assert gate.due(1_100_000_000)
    assert gate.due(500_000_000)
