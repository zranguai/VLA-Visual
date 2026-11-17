import csv
import json
from pathlib import Path

import numpy as np

from eval_model.alignment import MatchedSample
from eval_model.metadata import DatasetMetadata
from eval_model.metrics import ActionNormalizer
from eval_model.report import make_run_directory, summary_markdown, write_report


def _normalizer() -> ActionNormalizer:
    metadata = DatasetMetadata(
        root=Path("/tmp/test"),
        episode_index=0,
        fps=30,
        total_episodes=1,
        splits={},
        action_q01=np.zeros(16),
        action_q99=np.ones(16) * 100,
    )
    return ActionNormalizer(metadata)


def test_report_files_are_machine_readable(tmp_path):
    match = MatchedSample(1_000_000_000, np.zeros(16), np.ones(16))
    summary = {
        "status": "complete",
        "run": {"label": "test", "valid_for_model_ranking": False},
        "coverage": {"matched_samples": 1, "prediction_messages": 1},
        "warnings": [],
        "metrics": {"joint": {"all": {"mean": 0.1, "p95": 0.2}}},
    }
    summary_path, samples_path = write_report(
        tmp_path / "report", summary, [match], _normalizer()
    )

    assert json.loads(summary_path.read_text())["status"] == "complete"
    with samples_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.reader(stream))
    assert len(rows) == 2
    assert "abs_normalized_error_left_joint_0" in rows[0]
    assert "Joint NMAE" in summary_markdown(summary)


def test_run_directory_sanitizes_label(tmp_path):
    path = make_run_directory(tmp_path, "pi0.5 / checkpoint 10")
    assert path.parent == tmp_path
    assert "pi0.5-checkpoint-10" in path.name
