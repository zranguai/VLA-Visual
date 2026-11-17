from pathlib import Path

import pytest

from eval_model.config import load_config
from eval_model.node import _configure_domain, _parse_cli


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "eval_model.yaml"


def test_cli_accepts_required_paths():
    args = _parse_cli(
        [
            "--config",
            "/tmp/eval.yaml",
            "--replay-config",
            "/tmp/replay.yaml",
            "--run-label",
            "checkpoint",
        ]
    )
    assert args.run_label == "checkpoint"


def test_domain_conflict_is_rejected(monkeypatch):
    config = load_config(CONFIG_PATH)
    monkeypatch.setenv("ROS_DOMAIN_ID", "38")
    with pytest.raises(RuntimeError, match="conflicts"):
        _configure_domain(config)
