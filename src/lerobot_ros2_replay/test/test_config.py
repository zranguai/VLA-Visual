from pathlib import Path

import pytest

from lerobot_ros2_replay.config import load_config


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_default_config_is_safe_and_complete():
    config = load_config(PACKAGE_ROOT / "config" / "replay.yaml")
    assert config.ros_domain_id == 100
    assert config.action.name == "/execute_arm_cmd_dummy"
    assert config.playback.loop is False
    assert set(config.cameras) == {"top", "left", "right"}
    assert all(camera.hz == 30.0 for camera in config.cameras.values())


def test_real_robot_domain_is_rejected(tmp_path):
    source = (PACKAGE_ROOT / "config" / "replay.yaml").read_text(encoding="utf-8")
    path = tmp_path / "unsafe.yaml"
    path.write_text(source.replace("ros_domain_id: 100", "ros_domain_id: 38"), encoding="utf-8")
    with pytest.raises(ValueError, match="reserved for the real robot"):
        load_config(path)
