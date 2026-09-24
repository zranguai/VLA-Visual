from pathlib import Path

import pytest

from lerobot_ros2_visualizer.config import load_config
from lerobot_ros2_visualizer.node import _configure_domain, _parse_cli


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "visualizer.yaml"


def test_cli_supports_headless_recording():
    args = _parse_cli(["--config", str(CONFIG_PATH), "--save", "/tmp/test.rrd"])
    assert args.config == str(CONFIG_PATH)
    assert args.save == "/tmp/test.rrd"


def test_domain_conflict_is_rejected(monkeypatch):
    config = load_config(CONFIG_PATH)
    monkeypatch.setenv("ROS_DOMAIN_ID", "38")
    with pytest.raises(RuntimeError, match="conflicts"):
        _configure_domain(config)


def test_domain_is_set_when_missing(monkeypatch):
    config = load_config(CONFIG_PATH)
    monkeypatch.delenv("ROS_DOMAIN_ID", raising=False)
    _configure_domain(config)
    assert config.ros_domain_id == 100
