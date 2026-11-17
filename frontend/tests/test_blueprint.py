from pathlib import Path

import rerun.blueprint as rrb

from lerobot_ros2_visualizer import blueprint as blueprint_module
from lerobot_ros2_visualizer.blueprint import (
    SOURCE_STYLES,
    build_blueprint,
    camera_entity_path,
    log_series_styles,
    signal_entity_path,
)
from lerobot_ros2_visualizer.config import load_config


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "visualizer.yaml"


def test_blueprint_builds_for_all_default_views():
    config = load_config(CONFIG_PATH)
    blueprint = build_blueprint(config)
    evaluation_blueprint = build_blueprint(config, evaluation_enabled=True)
    assert isinstance(blueprint, rrb.Blueprint)
    assert isinstance(evaluation_blueprint, rrb.Blueprint)
    assert len(config.cameras.enabled) == 3
    assert len(config.signals.enabled) == 16


def test_entity_paths_are_stable():
    assert camera_entity_path("top") == "/cameras/top"
    assert (
        signal_entity_path("left_joint_0", "model_action")
        == "/signals/left_joint_0/model_action"
    )


def test_series_styles_cover_every_signal_and_source(monkeypatch):
    config = load_config(CONFIG_PATH)
    calls = []
    monkeypatch.setattr(
        blueprint_module.rr,
        "log",
        lambda path, value, static=False: calls.append((path, value, static)),
    )
    log_series_styles(config)
    assert len(calls) == len(config.signals.enabled) * len(config.signals.sources)
    assert all(static for _, _, static in calls)
    assert len({tuple(style[1]) for style in SOURCE_STYLES.values()}) == 3


def test_evaluation_styles_are_logged(monkeypatch):
    config = load_config(CONFIG_PATH)
    calls = []
    monkeypatch.setattr(
        blueprint_module.rr,
        "log",
        lambda path, value, static=False: calls.append((path, value, static)),
    )
    log_series_styles(config, evaluation_enabled=True)
    assert any(path.startswith("/evaluation/overview/") for path, _, _ in calls)
    assert any(path.startswith("/evaluation/signals/") for path, _, _ in calls)
    assert all(static for _, _, static in calls)
