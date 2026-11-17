from dataclasses import replace
from pathlib import Path

import numpy as np

from eval_model.config import load_config
from eval_model.metadata import DatasetMetadata
from eval_model.metrics import EvaluationSession


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "eval_model.yaml"


def _metadata() -> DatasetMetadata:
    q01 = np.full(16, -1.0, dtype=np.float64)
    q99 = np.full(16, 1.0, dtype=np.float64)
    q01[[7, 15]] = 0.0
    q99[[7, 15]] = 100.0
    return DatasetMetadata(
        root=Path("/tmp/test-dataset"),
        episode_index=0,
        fps=30.0,
        total_episodes=1,
        splits={"train": "0:1"},
        action_q01=q01,
        action_q99=q99,
    )


def _config():
    config = load_config(CONFIG_PATH)
    return replace(config, metrics=replace(config.metrics, min_samples=10))


def _action(step: int) -> np.ndarray:
    value = np.sin(step / 8.0)
    action = np.full(16, value, dtype=np.float64)
    action[7] = 100.0 if step >= 30 else -5.0
    action[15] = 100.0 if step >= 50 else -5.0
    return action


def test_exact_actions_have_zero_error_and_diagnostic_warning():
    session = EvaluationSession(_config(), _metadata())
    for index in range(80):
        timestamp = 1_000_000_000 + index * 33_333_333
        session.add_reference(timestamp, _action(index))
        session.add_prediction(timestamp, _action(index))
    summary = session.summary("exact")

    assert summary["status"] == "complete"
    assert summary["metrics"]["joint"]["all"]["mean"] == 0.0
    assert summary["metrics"]["temporal_alignment"]["estimated_delay_ms"] == 0.0
    assert summary["run"]["valid_for_model_ranking"] is False
    assert any("train split" in warning for warning in summary["warnings"])


def test_known_prediction_delay_is_recovered():
    session = EvaluationSession(_config(), _metadata())
    delay_steps = 6
    for index in range(120):
        timestamp = 1_000_000_000 + index * 33_333_333
        session.add_reference(timestamp, _action(index))
        session.add_prediction(timestamp, _action(max(0, index - delay_steps)))
    summary = session.summary("delayed")
    temporal = summary["metrics"]["temporal_alignment"]

    assert abs(temporal["estimated_delay_ms"] - 200.0) < 1e-6
    assert temporal["compensated_joint_nmae"] < summary["metrics"]["joint"]["all"]["mean"]


def test_gripper_events_are_scored_at_multiple_tolerances():
    session = EvaluationSession(_config(), _metadata())
    for index in range(90):
        timestamp = 1_000_000_000 + index * 33_333_333
        session.add_reference(timestamp, _action(index))
        session.add_prediction(timestamp, _action(max(0, index - 3)))
    summary = session.summary("gripper")
    scores = summary["metrics"]["gripper"]["left_gripper"]["event_scores"]

    assert [score["tolerance_ms"] for score in scores] == [100.0, 250.0, 500.0]
    assert scores[-1]["f1"] == 1.0


def test_insufficient_samples_do_not_produce_metrics():
    session = EvaluationSession(_config(), _metadata())
    session.add_reference(1_000_000_000, _action(0))
    session.add_prediction(1_000_000_000, _action(0))
    summary = session.summary("short")
    assert summary["status"] == "insufficient_data"
    assert summary["metrics"] == {}
