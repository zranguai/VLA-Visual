from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Iterable

import numpy as np

from .alignment import ActionAligner, ActionSample, MatchedSample
from .config import EvaluationConfig
from .constants import (
    ACTION_MAX,
    ACTION_MIN,
    GRIPPER_INDICES,
    JOINT_INDICES,
    LEFT_JOINT_INDICES,
    RIGHT_JOINT_INDICES,
    SIGNAL_NAMES,
)
from .metadata import DatasetMetadata


@dataclass(frozen=True)
class LiveMetrics:
    timestamp_ns: int
    normalized_error: np.ndarray
    joint_nmae: float
    left_joint_nmae: float
    right_joint_nmae: float
    rolling_joint_nmae: float
    left_gripper_match: float
    right_gripper_match: float


class ActionNormalizer:
    def __init__(self, metadata: DatasetMetadata) -> None:
        self.q01 = metadata.action_q01.astype(np.float64, copy=True)
        self.q99 = metadata.action_q99.astype(np.float64, copy=True)
        self.scale = np.maximum(self.q99 - self.q01, 1e-6)
        self.scale[GRIPPER_INDICES] = 100.0

    @staticmethod
    def canonicalize(values: np.ndarray) -> np.ndarray:
        result = np.asarray(values, dtype=np.float64).copy()
        result[..., GRIPPER_INDICES] = np.clip(
            result[..., GRIPPER_INDICES], 0.0, 100.0
        )
        return result

    def normalized_error(self, prediction: np.ndarray, reference: np.ndarray) -> np.ndarray:
        prediction_value = self.canonicalize(prediction)
        reference_value = self.canonicalize(reference)
        return (prediction_value - reference_value) / self.scale


class EvaluationSession:
    def __init__(self, config: EvaluationConfig, metadata: DatasetMetadata) -> None:
        self.config = config
        self.metadata = metadata
        self.aligner = ActionAligner(config.alignment.max_interpolation_gap_ms)
        self.normalizer = ActionNormalizer(metadata)
        self._rolling: deque[tuple[int, float]] = deque()
        self._finalized = False

    def add_reference(self, timestamp_ns: int, values: np.ndarray) -> list[LiveMetrics]:
        if self._finalized:
            return []
        return [self._live_metrics(match) for match in self.aligner.add_reference(timestamp_ns, values)]

    def add_prediction(self, timestamp_ns: int, values: np.ndarray) -> list[LiveMetrics]:
        if self._finalized:
            return []
        return [self._live_metrics(match) for match in self.aligner.add_prediction(timestamp_ns, values)]

    def _live_metrics(self, match: MatchedSample) -> LiveMetrics:
        normalized_error = np.abs(
            self.normalizer.normalized_error(match.prediction, match.reference)
        )
        joint_nmae = float(np.mean(normalized_error[JOINT_INDICES]))
        left_joint_nmae = float(np.mean(normalized_error[LEFT_JOINT_INDICES]))
        right_joint_nmae = float(np.mean(normalized_error[RIGHT_JOINT_INDICES]))
        self._rolling.append((match.timestamp_ns, joint_nmae))
        cutoff = match.timestamp_ns - int(self.config.metrics.rolling_window_s * 1e9)
        while self._rolling and self._rolling[0][0] < cutoff:
            self._rolling.popleft()
        rolling = float(np.mean([value for _, value in self._rolling]))
        reference = self.normalizer.canonicalize(match.reference)
        prediction = self.normalizer.canonicalize(match.prediction)
        threshold = self.config.metrics.gripper_threshold
        return LiveMetrics(
            timestamp_ns=match.timestamp_ns,
            normalized_error=normalized_error,
            joint_nmae=joint_nmae,
            left_joint_nmae=left_joint_nmae,
            right_joint_nmae=right_joint_nmae,
            rolling_joint_nmae=rolling,
            left_gripper_match=float(
                (reference[7] >= threshold) == (prediction[7] >= threshold)
            ),
            right_gripper_match=float(
                (reference[15] >= threshold) == (prediction[15] >= threshold)
            ),
        )

    def finalize(self) -> list[LiveMetrics]:
        if self._finalized:
            return []
        produced = [self._live_metrics(match) for match in self.aligner.finalize()]
        self._finalized = True
        return produced

    def summary(self, run_label: str) -> dict[str, Any]:
        self.finalize()
        return build_summary(self.config, self.metadata, self.aligner, self.normalizer, run_label)


def _float(value: float | np.floating[Any]) -> float:
    return float(value)


def _distribution(values: np.ndarray) -> dict[str, float] | None:
    array = np.asarray(values, dtype=np.float64)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return None
    return {
        "mean": _float(np.mean(array)),
        "rmse": _float(np.sqrt(np.mean(np.square(array)))),
        "p50": _float(np.quantile(array, 0.50)),
        "p95": _float(np.quantile(array, 0.95)),
        "max": _float(np.max(array)),
    }


def _observed_hz(samples: Iterable[ActionSample]) -> float | None:
    timestamps = np.asarray([sample.timestamp_ns for sample in samples], dtype=np.int64)
    if timestamps.size < 2:
        return None
    intervals = np.diff(timestamps).astype(np.float64) / 1e9
    intervals = intervals[intervals > 0]
    if intervals.size == 0:
        return None
    return _float(1.0 / np.median(intervals))


def _balanced_accuracy(reference: np.ndarray, prediction: np.ndarray) -> float | None:
    recalls = []
    for state in (False, True):
        mask = reference == state
        if np.any(mask):
            recalls.append(np.mean(prediction[mask] == state))
    return None if not recalls else _float(np.mean(recalls))


def _binary_f1(reference: np.ndarray, prediction: np.ndarray) -> float | None:
    true_positive = int(np.sum(reference & prediction))
    false_positive = int(np.sum(~reference & prediction))
    false_negative = int(np.sum(reference & ~prediction))
    denominator = 2 * true_positive + false_positive + false_negative
    return None if denominator == 0 else 2.0 * true_positive / denominator


def _events(timestamps: np.ndarray, states: np.ndarray) -> list[tuple[int, bool]]:
    indices = np.flatnonzero(states[1:] != states[:-1]) + 1
    return [(int(timestamps[index]), bool(states[index])) for index in indices]


def _event_score(
    reference_events: list[tuple[int, bool]],
    prediction_events: list[tuple[int, bool]],
    tolerance_ms: float,
) -> tuple[dict[str, float | int | None], list[float]]:
    tolerance_ns = int(tolerance_ms * 1e6)
    unused = set(range(len(prediction_events)))
    signed_errors_ms: list[float] = []
    for reference_time, direction in reference_events:
        candidates = [
            index
            for index in unused
            if prediction_events[index][1] == direction
            and abs(prediction_events[index][0] - reference_time) <= tolerance_ns
        ]
        if not candidates:
            continue
        best = min(
            candidates,
            key=lambda index: abs(prediction_events[index][0] - reference_time),
        )
        unused.remove(best)
        signed_errors_ms.append((prediction_events[best][0] - reference_time) / 1e6)
    matched = len(signed_errors_ms)
    precision = matched / len(prediction_events) if prediction_events else None
    recall = matched / len(reference_events) if reference_events else None
    f1 = (
        None
        if precision is None or recall is None or precision + recall == 0
        else 2 * precision * recall / (precision + recall)
    )
    return (
        {
            "tolerance_ms": tolerance_ms,
            "reference_events": len(reference_events),
            "prediction_events": len(prediction_events),
            "matched_events": matched,
            "precision": precision,
            "recall": recall,
            "f1": f1,
        },
        signed_errors_ms,
    )


def _gripper_metrics(
    timestamps: np.ndarray,
    reference: np.ndarray,
    prediction: np.ndarray,
    threshold: float,
    tolerances_ms: tuple[float, ...],
    normalizer: ActionNormalizer,
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for index, name in ((7, "left_gripper"), (15, "right_gripper")):
        reference_state = reference[:, index] >= threshold
        prediction_state = prediction[:, index] >= threshold
        reference_events = _events(timestamps, reference_state)
        prediction_events = _events(timestamps, prediction_state)
        tolerance_scores = []
        final_errors: list[float] = []
        for tolerance in tolerances_ms:
            score, errors = _event_score(reference_events, prediction_events, tolerance)
            tolerance_scores.append(score)
            final_errors = errors
        absolute_timing = np.abs(np.asarray(final_errors, dtype=np.float64))
        result[name] = {
            "continuous_nmae": _float(
                np.mean(
                    np.abs(prediction[:, index] - reference[:, index])
                    / normalizer.scale[index]
                )
            ),
            "balanced_accuracy": _balanced_accuracy(reference_state, prediction_state),
            "f1_open": _binary_f1(reference_state, prediction_state),
            "event_scores": tolerance_scores,
            "matched_event_timing_ms": _distribution(absolute_timing),
        }
    return result


def _dynamic_metrics(
    timestamps: np.ndarray,
    reference: np.ndarray,
    prediction: np.ndarray,
    normalizer: ActionNormalizer,
) -> dict[str, Any]:
    if len(timestamps) < 2:
        return {}
    dt = np.diff(timestamps).astype(np.float64) / 1e9
    valid = dt > 1e-6
    if not np.any(valid):
        return {}
    reference_delta = np.diff(reference[:, JOINT_INDICES], axis=0)[valid]
    prediction_delta = np.diff(prediction[:, JOINT_INDICES], axis=0)[valid]
    scale = normalizer.scale[JOINT_INDICES]
    velocity_error = np.abs(
        prediction_delta / dt[valid, None] - reference_delta / dt[valid, None]
    ) / scale
    reference_tv = _float(np.sum(np.abs(reference_delta / scale)))
    prediction_tv = _float(np.sum(np.abs(prediction_delta / scale)))
    prediction_step = np.abs(prediction_delta / scale)
    return {
        "velocity_nmae_per_s": _distribution(velocity_error.reshape(-1)),
        "total_variation_ratio": None if reference_tv <= 1e-12 else prediction_tv / reference_tv,
        "reference_total_variation": reference_tv,
        "prediction_total_variation": prediction_tv,
        "prediction_max_normalized_step": _float(np.max(prediction_step)),
    }


def _motion_strata(
    reference: np.ndarray,
    normalized_abs_error: np.ndarray,
    normalizer: ActionNormalizer,
    quantiles: tuple[float, float],
) -> dict[str, Any]:
    intensity = np.zeros(len(reference), dtype=np.float64)
    if len(reference) > 1:
        intensity[1:] = np.mean(
            np.abs(np.diff(reference[:, JOINT_INDICES], axis=0))
            / normalizer.scale[JOINT_INDICES],
            axis=1,
        )
    low, high = np.quantile(intensity, quantiles)
    masks = {
        "low_0_50": intensity <= low,
        "medium_50_90": (intensity > low) & (intensity <= high),
        "high_90_100": intensity > high,
    }
    return {
        "thresholds": {"q50": _float(low), "q90": _float(high)},
        "groups": {
            name: {
                "sample_count": int(np.sum(mask)),
                "joint_nmae": (
                    None
                    if not np.any(mask)
                    else _float(np.mean(normalized_abs_error[mask][:, JOINT_INDICES]))
                ),
                "joint_p95": (
                    None
                    if not np.any(mask)
                    else _float(
                        np.quantile(normalized_abs_error[mask][:, JOINT_INDICES], 0.95)
                    )
                ),
            }
            for name, mask in masks.items()
        },
    }


def _interpolate_stream(
    samples: list[ActionSample], grid_ns: np.ndarray
) -> np.ndarray:
    timestamps = np.asarray([sample.timestamp_ns for sample in samples], dtype=np.int64)
    values = np.stack([sample.values for sample in samples]).astype(np.float64)
    unique_timestamps, reverse_indices = np.unique(timestamps[::-1], return_index=True)
    keep = len(timestamps) - 1 - reverse_indices
    order = np.argsort(unique_timestamps)
    timestamps = unique_timestamps[order]
    values = values[keep[order]]
    origin = int(grid_ns[0])
    sample_seconds = (timestamps - origin).astype(np.float64) / 1e9
    grid_seconds = (grid_ns - origin).astype(np.float64) / 1e9
    output = np.empty((len(grid_ns), 16), dtype=np.float64)
    for index in JOINT_INDICES:
        output[:, index] = np.interp(grid_seconds, sample_seconds, values[:, index])
    positions = np.searchsorted(timestamps, grid_ns, side="right") - 1
    positions = np.clip(positions, 0, len(timestamps) - 1)
    output[:, GRIPPER_INDICES] = values[positions][:, GRIPPER_INDICES]
    return output


def _lag_metrics(
    references: list[ActionSample],
    predictions: list[ActionSample],
    config: EvaluationConfig,
    normalizer: ActionNormalizer,
) -> dict[str, Any] | None:
    if len(references) < 2 or len(predictions) < 2:
        return None
    start_ns = max(references[0].timestamp_ns, predictions[0].timestamp_ns)
    end_ns = min(references[-1].timestamp_ns, predictions[-1].timestamp_ns)
    period_ns = max(1, int(1e9 / config.alignment.resample_hz))
    if end_ns - start_ns < period_ns * config.metrics.min_samples:
        return None
    grid_ns = np.arange(start_ns, end_ns + 1, period_ns, dtype=np.int64)
    reference = normalizer.canonicalize(_interpolate_stream(references, grid_ns))
    prediction = normalizer.canonicalize(_interpolate_stream(predictions, grid_ns))
    max_steps = int(round(config.alignment.lag_search_ms / 1000 * config.alignment.resample_hz))
    candidates = []
    for steps in range(-max_steps, max_steps + 1):
        if steps > 0:
            predicted_part, reference_part = prediction[steps:], reference[:-steps]
        elif steps < 0:
            predicted_part, reference_part = prediction[:steps], reference[-steps:]
        else:
            predicted_part, reference_part = prediction, reference
        if len(predicted_part) < config.metrics.min_samples:
            continue
        error = np.abs(
            normalizer.normalized_error(predicted_part, reference_part)[:, JOINT_INDICES]
        )
        candidates.append((float(np.mean(error)), steps, error))
    if not candidates:
        return None
    best_error, best_steps, error_values = min(
        candidates, key=lambda item: (item[0], abs(item[1]))
    )
    return {
        "estimated_delay_ms": best_steps * 1000.0 / config.alignment.resample_hz,
        "compensated_joint_nmae": best_error,
        "compensated_joint_error": _distribution(error_values.reshape(-1)),
        "search_range_ms": config.alignment.lag_search_ms,
        "resample_hz": config.alignment.resample_hz,
    }


def _episode_in_train_split(metadata: DatasetMetadata) -> bool:
    value = metadata.splits.get("train")
    if not value or ":" not in value:
        return False
    try:
        start_text, stop_text = value.split(":", maxsplit=1)
        start = int(start_text or 0)
        stop = int(stop_text)
    except ValueError:
        return False
    return start <= metadata.episode_index < stop


def _baselines(reference: np.ndarray, normalizer: ActionNormalizer) -> dict[str, Any]:
    if len(reference) < 2:
        return {}
    persistence_error = np.abs(
        normalizer.normalized_error(reference[:-1], reference[1:])[:, JOINT_INDICES]
    )
    median_prediction = np.broadcast_to(np.median(reference, axis=0), reference.shape)
    median_error = np.abs(
        normalizer.normalized_error(median_prediction, reference)[:, JOINT_INDICES]
    )
    return {
        "previous_reference_action": _distribution(persistence_error.reshape(-1)),
        "per_dimension_median": _distribution(median_error.reshape(-1)),
    }


def build_summary(
    config: EvaluationConfig,
    metadata: DatasetMetadata,
    aligner: ActionAligner,
    normalizer: ActionNormalizer,
    run_label: str,
) -> dict[str, Any]:
    matches = sorted(aligner.matches, key=lambda item: item.timestamp_ns)
    matched_count = len(matches)
    status = "complete" if matched_count >= config.metrics.min_samples else "insufficient_data"
    warnings: list[str] = []
    train_episode = _episode_in_train_split(metadata)
    valid_for_ranking = config.report.evaluation_mode == "held_out" and not train_episode
    if config.report.evaluation_mode == "diagnostic":
        warnings.append("diagnostic mode: results must not be used for model ranking")
    if train_episode:
        warnings.append("selected episode belongs to the dataset train split")
    if metadata.total_episodes <= 1:
        warnings.append("dataset contains only one episode; no generalization estimate is possible")
    if aligner.prediction_count == 0:
        warnings.append("no model action messages were received")
    if status != "complete":
        warnings.append(
            f"matched samples {matched_count} are below required minimum {config.metrics.min_samples}"
        )

    summary: dict[str, Any] = {
        "schema_version": 1,
        "status": status,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "run": {
            "label": run_label,
            "evaluation_mode": config.report.evaluation_mode,
            "valid_for_model_ranking": valid_for_ranking,
        },
        "dataset": {
            "root": str(metadata.root),
            "episode_index": metadata.episode_index,
            "fps": metadata.fps,
            "total_episodes": metadata.total_episodes,
            "splits": metadata.splits,
        },
        "topics": {
            "reference": config.topics.reference,
            "prediction": config.topics.prediction,
        },
        "coverage": {
            "reference_messages": aligner.reference_count,
            "prediction_messages": aligner.prediction_count,
            "matched_samples": matched_count,
            "unmatched_predictions": aligner.unmatched_prediction_count,
            "prediction_match_rate": (
                None
                if aligner.prediction_count == 0
                else matched_count / aligner.prediction_count
            ),
            "reference_hz": _observed_hz(aligner.references),
            "prediction_hz": _observed_hz(aligner.predictions),
        },
        "warnings": warnings,
        "metrics": {},
    }
    if status != "complete":
        return summary

    timestamps = np.asarray([match.timestamp_ns for match in matches], dtype=np.int64)
    reference_raw = np.stack([match.reference for match in matches])
    prediction_raw = np.stack([match.prediction for match in matches])
    reference = normalizer.canonicalize(reference_raw)
    prediction = normalizer.canonicalize(prediction_raw)
    signed_error = normalizer.normalized_error(prediction, reference)
    absolute_error = np.abs(signed_error)
    per_signal = {
        name: _distribution(absolute_error[:, index])
        for index, name in enumerate(SIGNAL_NAMES)
    }
    out_of_range = (prediction_raw < ACTION_MIN) | (prediction_raw > ACTION_MAX)
    summary["metrics"] = {
        "joint": {
            "all": _distribution(absolute_error[:, JOINT_INDICES].reshape(-1)),
            "left": _distribution(absolute_error[:, LEFT_JOINT_INDICES].reshape(-1)),
            "right": _distribution(absolute_error[:, RIGHT_JOINT_INDICES].reshape(-1)),
        },
        "per_signal": per_signal,
        "gripper": _gripper_metrics(
            timestamps,
            reference,
            prediction,
            config.metrics.gripper_threshold,
            config.metrics.event_tolerances_ms,
            normalizer,
        ),
        "temporal_alignment": _lag_metrics(
            aligner.references, aligner.predictions, config, normalizer
        ),
        "dynamics": _dynamic_metrics(timestamps, reference, prediction, normalizer),
        "motion_strata": _motion_strata(
            reference, absolute_error, normalizer, config.metrics.motion_quantiles
        ),
        "safety": {
            "out_of_range_rate": _float(np.mean(out_of_range)),
            "out_of_range_by_signal": {
                name: _float(np.mean(out_of_range[:, index]))
                for index, name in enumerate(SIGNAL_NAMES)
            },
        },
        "baselines": _baselines(reference, normalizer),
    }
    return summary
