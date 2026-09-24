import numpy as np
from builtin_interfaces.msg import Time

from lerobot_ros2_replay.message_builders import IMAGE_CAPACITY, build_action, build_arm_status, build_image


def test_arm_and_action_mapping():
    values = np.arange(16, dtype=np.float32)
    stamp = Time(sec=12, nanosec=34)
    status = build_arm_status(stamp, values)
    action = build_action(stamp, values)
    assert [joint.joint_status[0] for joint in status.left_arm] == list(range(7))
    assert [joint.joint_status[0] for joint in status.right_arm] == list(range(8, 15))
    assert status.gripper[0].joint_status[0] == 7.0
    assert status.gripper[1].joint_status[0] == 15.0
    assert list(action.left_command) == list(range(7))
    assert list(action.right_command) == list(range(8, 15))
    assert action.gripper_command[0] == 7.0
    assert action.gripper_command[1] == 15.0


def test_image1m_payload_and_metadata():
    image = np.arange(3 * 4 * 3, dtype=np.uint8).reshape(3, 4, 3)
    msg = build_image(Time(sec=1), image, "top")
    assert msg.height == 3
    assert msg.width == 4
    assert msg.step == 12
    assert msg.encoding.size == 4
    assert bytes(msg.encoding.data[:4]) == b"rgb8"
    assert len(msg.data) == IMAGE_CAPACITY
    assert bytes(msg.data[: image.size]) == image.tobytes()
    assert not any(msg.data[image.size :])
