import importlib.util
import os
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest


REPO_ROOT = Path(__file__).resolve().parents[3]
SCRIPT = REPO_ROOT / "inference" / "pi05_inference" / "lite_inference_openpi_temporal_smoothing_hil.py"


@pytest.fixture(scope="module")
def inference_module():
    fake_openpi = ModuleType("openpi_client")
    fake_openpi.image_tools = SimpleNamespace()
    fake_openpi.websocket_client_policy = SimpleNamespace()
    original = sys.modules.get("openpi_client")
    sys.modules["openpi_client"] = fake_openpi
    try:
        spec = importlib.util.spec_from_file_location("replay_inference_under_test", SCRIPT)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
        yield module
    finally:
        if original is None:
            sys.modules.pop("openpi_client", None)
        else:
            sys.modules["openpi_client"] = original


def _parse(monkeypatch, module, *extra):
    monkeypatch.setenv("ROS_DOMAIN_ID", "100")
    monkeypatch.setattr(sys, "argv", [str(SCRIPT), "--host", "127.0.0.1", "--task", "test", *extra])
    return module.parse_args()


def test_replay_mode_uses_safe_default_topic(monkeypatch, inference_module):
    args = _parse(monkeypatch, inference_module, "--replay-mode")
    assert args.cmd_topic == "/execute_arm_cmd_inference_dummy"


def test_replay_mode_refuses_real_command_topic(monkeypatch, inference_module):
    with pytest.raises(SystemExit):
        _parse(
            monkeypatch,
            inference_module,
            "--replay-mode",
            "--cmd-topic",
            "/execute_arm_cmd",
        )


def test_replay_mode_requires_isolated_domain(monkeypatch, inference_module):
    monkeypatch.setenv("ROS_DOMAIN_ID", "38")
    monkeypatch.setattr(
        sys,
        "argv",
        [str(SCRIPT), "--host", "127.0.0.1", "--task", "test", "--replay-mode"],
    )
    with pytest.raises(SystemExit):
        inference_module.parse_args()
