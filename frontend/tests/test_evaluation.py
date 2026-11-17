from dataclasses import replace
from pathlib import Path

import numpy as np

from eval_model.config import load_config
from eval_model.metadata import DatasetMetadata
from lerobot_ros2_visualizer import evaluation as evaluation_module
from lerobot_ros2_visualizer.evaluation import RerunEvaluationBridge


EVAL_CONFIG_PATH = (
    Path(__file__).resolve().parents[2] / "src" / "eval_model" / "config" / "eval_model.yaml"
)


def test_bridge_logs_metrics_and_writes_report(tmp_path, monkeypatch):
    config = load_config(EVAL_CONFIG_PATH)
    config = replace(config, metrics=replace(config.metrics, min_samples=5))
    metadata = DatasetMetadata(
        root=tmp_path / "dataset",
        episode_index=0,
        fps=30,
        total_episodes=1,
        splits={"train": "0:1"},
        action_q01=np.full(16, -1.0),
        action_q99=np.full(16, 100.0),
    )
    logged_paths = []
    monkeypatch.setattr(evaluation_module.rr, "set_time", lambda *args, **kwargs: None)
    monkeypatch.setattr(
        evaluation_module.rr,
        "log",
        lambda path, value, static=False: logged_paths.append((path, static)),
    )
    bridge = RerunEvaluationBridge(
        config,
        metadata,
        "ros_time",
        "test-model",
        tmp_path / "report",
    )
    for index in range(10):
        timestamp = 1_000_000_000 + index * 33_333_333
        action = np.full(16, index / 100.0)
        bridge.add_action("dataset_action", timestamp, action)
        bridge.add_action("model_action", timestamp, action)

    summary_path = bridge.finalize()

    assert summary_path.is_file()
    assert bridge.summary["status"] == "complete"
    assert any(path == "/evaluation/overview/joint_nmae" for path, _ in logged_paths)
    assert any(path == "/evaluation/summary" and static for path, static in logged_paths)
