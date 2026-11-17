from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class TopicConfig:
    reference: str
    prediction: str


@dataclass(frozen=True)
class AlignmentConfig:
    max_interpolation_gap_ms: float
    lag_search_ms: float
    resample_hz: float


@dataclass(frozen=True)
class MetricConfig:
    gripper_threshold: float
    event_tolerances_ms: tuple[float, ...]
    motion_quantiles: tuple[float, float]
    min_samples: int
    rolling_window_s: float


@dataclass(frozen=True)
class ReportConfig:
    output_root: Path
    idle_timeout_s: float
    status_interval_s: float
    evaluation_mode: str


@dataclass(frozen=True)
class EvaluationConfig:
    ros_domain_id: int
    topics: TopicConfig
    alignment: AlignmentConfig
    metrics: MetricConfig
    report: ReportConfig


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def _required(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ValueError(f"missing {path}.{key}")
    return mapping[key]


def _topic(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.startswith("/"):
        raise ValueError(f"{path} must be an absolute ROS topic")
    if value == "/execute_arm_cmd":
        raise ValueError(f"{path} must not use the real robot command topic")
    return value


def _positive(value: Any, path: str) -> float:
    result = float(value)
    if result <= 0:
        raise ValueError(f"{path} must be greater than zero")
    return result


def load_config(path: str | Path) -> EvaluationConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"evaluation config does not exist: {config_path}")
    root = _mapping(yaml.safe_load(config_path.read_text(encoding="utf-8")), "config")

    domain = int(_required(root, "ros_domain_id", "config"))
    if domain != 100:
        raise ValueError("offline action evaluation requires ros_domain_id 100")

    topics_raw = _mapping(_required(root, "topics", "config"), "topics")
    topics = TopicConfig(
        reference=_topic(_required(topics_raw, "reference", "topics"), "topics.reference"),
        prediction=_topic(_required(topics_raw, "prediction", "topics"), "topics.prediction"),
    )
    if topics.reference == topics.prediction:
        raise ValueError("reference and prediction topics must be different")

    alignment_raw = _mapping(_required(root, "alignment", "config"), "alignment")
    alignment = AlignmentConfig(
        max_interpolation_gap_ms=_positive(
            _required(alignment_raw, "max_interpolation_gap_ms", "alignment"),
            "alignment.max_interpolation_gap_ms",
        ),
        lag_search_ms=_positive(
            _required(alignment_raw, "lag_search_ms", "alignment"),
            "alignment.lag_search_ms",
        ),
        resample_hz=_positive(
            _required(alignment_raw, "resample_hz", "alignment"),
            "alignment.resample_hz",
        ),
    )

    metrics_raw = _mapping(_required(root, "metrics", "config"), "metrics")
    tolerances_raw = _required(metrics_raw, "event_tolerances_ms", "metrics")
    if not isinstance(tolerances_raw, list) or not tolerances_raw:
        raise ValueError("metrics.event_tolerances_ms must be a non-empty list")
    event_tolerances = tuple(float(value) for value in tolerances_raw)
    if any(value <= 0 for value in event_tolerances) or tuple(sorted(event_tolerances)) != event_tolerances:
        raise ValueError("metrics.event_tolerances_ms must be positive and sorted")
    motion_raw = _required(metrics_raw, "motion_quantiles", "metrics")
    if not isinstance(motion_raw, list) or len(motion_raw) != 2:
        raise ValueError("metrics.motion_quantiles must contain two values")
    motion_quantiles = (float(motion_raw[0]), float(motion_raw[1]))
    if not 0 < motion_quantiles[0] < motion_quantiles[1] < 1:
        raise ValueError("metrics.motion_quantiles must satisfy 0 < low < high < 1")
    min_samples = int(_required(metrics_raw, "min_samples", "metrics"))
    if min_samples < 2:
        raise ValueError("metrics.min_samples must be at least 2")
    gripper_threshold = float(_required(metrics_raw, "gripper_threshold", "metrics"))
    if not 0 <= gripper_threshold <= 100:
        raise ValueError("metrics.gripper_threshold must be between 0 and 100")
    metrics = MetricConfig(
        gripper_threshold=gripper_threshold,
        event_tolerances_ms=event_tolerances,
        motion_quantiles=motion_quantiles,
        min_samples=min_samples,
        rolling_window_s=_positive(
            _required(metrics_raw, "rolling_window_s", "metrics"),
            "metrics.rolling_window_s",
        ),
    )

    report_raw = _mapping(_required(root, "report", "config"), "report")
    output_root = Path(str(_required(report_raw, "output_root", "report"))).expanduser()
    if not output_root.is_absolute():
        output_root = (Path.cwd() / output_root).resolve()
    evaluation_mode = str(_required(report_raw, "evaluation_mode", "report"))
    if evaluation_mode not in ("diagnostic", "held_out"):
        raise ValueError("report.evaluation_mode must be 'diagnostic' or 'held_out'")
    report = ReportConfig(
        output_root=output_root,
        idle_timeout_s=_positive(
            _required(report_raw, "idle_timeout_s", "report"), "report.idle_timeout_s"
        ),
        status_interval_s=_positive(
            _required(report_raw, "status_interval_s", "report"),
            "report.status_interval_s",
        ),
        evaluation_mode=evaluation_mode,
    )
    return EvaluationConfig(
        ros_domain_id=domain,
        topics=topics,
        alignment=alignment,
        metrics=metrics,
        report=report,
    )
