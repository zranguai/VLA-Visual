from pathlib import Path

import pytest
import yaml

from eval_model.config import load_config


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "eval_model.yaml"


def test_default_config_is_safe_and_diagnostic():
    config = load_config(CONFIG_PATH)
    assert config.ros_domain_id == 100
    assert config.topics.reference == "/execute_arm_cmd_dummy"
    assert config.topics.prediction == "/execute_arm_cmd_inference_dummy"
    assert config.report.evaluation_mode == "diagnostic"


def test_real_command_topic_is_rejected(tmp_path):
    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    raw["topics"]["prediction"] = "/execute_arm_cmd"
    path = tmp_path / "unsafe.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="real robot command"):
        load_config(path)


def test_non_isolated_domain_is_rejected(tmp_path):
    raw = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    raw["ros_domain_id"] = 38
    path = tmp_path / "unsafe.yaml"
    path.write_text(yaml.safe_dump(raw), encoding="utf-8")
    with pytest.raises(ValueError, match="domain_id 100"):
        load_config(path)
