from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import rerun as rr
from eval_model.config import EvaluationConfig
from eval_model.constants import SIGNAL_NAMES
from eval_model.metadata import DatasetMetadata
from eval_model.metrics import EvaluationSession, LiveMetrics
from eval_model.report import make_run_directory, summary_markdown, write_report

from .blueprint import EVALUATION_OVERVIEW_PATHS, evaluation_error_path


class RerunEvaluationBridge:
    def __init__(
        self,
        config: EvaluationConfig,
        metadata: DatasetMetadata,
        timeline: str,
        run_label: str,
        output_directory: Path | None = None,
    ) -> None:
        self.config = config
        self.timeline = timeline
        self.run_label = run_label
        self.output_directory = output_directory or make_run_directory(
            config.report.output_root, run_label
        )
        self.session = EvaluationSession(config, metadata)
        self.last_reference_monotonic: float | None = None
        self.finalized = False
        self.summary: dict | None = None

    def add_action(
        self, source: str, timestamp_ns: int, values: np.ndarray
    ) -> list[LiveMetrics]:
        if self.finalized:
            return []
        if source == "dataset_action":
            self.last_reference_monotonic = time.monotonic()
            metrics = self.session.add_reference(timestamp_ns, values)
        elif source == "model_action":
            metrics = self.session.add_prediction(timestamp_ns, values)
        else:
            return []
        for metric in metrics:
            self._log_live(metric)
        return metrics

    def _log_live(self, metrics: LiveMetrics) -> None:
        rr.set_time(self.timeline, timestamp=np.datetime64(metrics.timestamp_ns, "ns"))
        values = {
            "joint_nmae": metrics.joint_nmae,
            "rolling_joint_nmae": metrics.rolling_joint_nmae,
            "left_joint_nmae": metrics.left_joint_nmae,
            "right_joint_nmae": metrics.right_joint_nmae,
            "left_gripper_match": metrics.left_gripper_match,
            "right_gripper_match": metrics.right_gripper_match,
        }
        for key, value in values.items():
            rr.log(EVALUATION_OVERVIEW_PATHS[key], rr.Scalars(value))
        for index, signal in enumerate(SIGNAL_NAMES):
            rr.log(
                evaluation_error_path(signal),
                rr.Scalars(float(metrics.normalized_error[index])),
            )

    def idle_timeout_reached(self) -> bool:
        return bool(
            not self.finalized
            and self.last_reference_monotonic is not None
            and time.monotonic() - self.last_reference_monotonic
            >= self.config.report.idle_timeout_s
        )

    def finalize(self) -> Path:
        if self.finalized:
            return self.output_directory / "summary.json"
        for metrics in self.session.finalize():
            self._log_live(metrics)
        self.summary = self.session.summary(self.run_label)
        summary_path, _ = write_report(
            self.output_directory,
            self.summary,
            self.session.aligner.matches,
            self.session.normalizer,
        )
        rr.log(
            "/evaluation/summary",
            rr.TextDocument(summary_markdown(self.summary), media_type="text/markdown"),
            static=True,
        )
        self.finalized = True
        return summary_path
