from pathlib import Path

import pytest
import yaml

from lerobot_ros2_visualizer.config import SIGNAL_NAMES, load_config


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "visualizer.yaml"


def _modified_config(tmp_path: Path, update) -> Path:
    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    update(raw)
    path = tmp_path / "visualizer.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    return path


def test_default_config_is_complete_and_safe():
    config = load_config(CONFIG_PATH)
    assert config.ros_domain_id == 100
    assert config.topics.dataset_action == "/execute_arm_cmd_dummy"
    assert config.topics.model_action == "/execute_arm_cmd_inference_dummy"
    assert config.cameras.sample_hz == 10.0
    assert config.cameras.jpeg_quality == 85
    assert config.signals.enabled == SIGNAL_NAMES
    assert set(config.signals.sources) == {"state", "dataset_action", "model_action"}


def test_non_replay_domain_is_rejected(tmp_path):
    path = _modified_config(tmp_path, lambda raw: raw.update(ros_domain_id=38))
    with pytest.raises(ValueError, match="requires ros_domain_id 100"):
        load_config(path)


def test_real_robot_command_topic_is_rejected(tmp_path):
    def update(raw):
        raw["topics"]["model_action"] = "/execute_arm_cmd"

    path = _modified_config(tmp_path, update)
    with pytest.raises(ValueError, match="real robot command topic"):
        load_config(path)


def test_visible_sources_must_be_subscribed(tmp_path):
    def update(raw):
        raw["signals"]["sources"] = ["state"]

    path = _modified_config(tmp_path, update)
    with pytest.raises(ValueError, match="must be a subset"):
        load_config(path)
