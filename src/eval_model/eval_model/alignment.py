from __future__ import annotations

from bisect import bisect_left
from dataclasses import dataclass

import numpy as np

from .constants import GRIPPER_INDICES, JOINT_INDICES


@dataclass(frozen=True)
class ActionSample:
    timestamp_ns: int
    values: np.ndarray


@dataclass(frozen=True)
class MatchedSample:
    timestamp_ns: int
    reference: np.ndarray
    prediction: np.ndarray


class ActionAligner:
    def __init__(self, max_gap_ms: float) -> None:
        self.max_gap_ns = int(max_gap_ms * 1_000_000)
        self.references: list[ActionSample] = []
        self.predictions: list[ActionSample] = []
        self.matches: list[MatchedSample] = []
        self._reference_times: list[int] = []
        self._pending: list[ActionSample] = []
        self.reference_count = 0
        self.prediction_count = 0
        self.unmatched_prediction_count = 0

    @staticmethod
    def _validated(timestamp_ns: int, values: np.ndarray) -> ActionSample:
        vector = np.asarray(values, dtype=np.float64)
        if timestamp_ns <= 0 or vector.shape != (16,) or not np.all(np.isfinite(vector)):
            raise ValueError("action sample must contain a positive timestamp and finite 16D vector")
        return ActionSample(int(timestamp_ns), vector.copy())

    def add_reference(self, timestamp_ns: int, values: np.ndarray) -> list[MatchedSample]:
        sample = self._validated(timestamp_ns, values)
        self.reference_count += 1
        position = bisect_left(self._reference_times, sample.timestamp_ns)
        if position < len(self._reference_times) and self._reference_times[position] == sample.timestamp_ns:
            self.references[position] = sample
        else:
            self._reference_times.insert(position, sample.timestamp_ns)
            self.references.insert(position, sample)
        return self._drain_pending()

    def add_prediction(self, timestamp_ns: int, values: np.ndarray) -> list[MatchedSample]:
        sample = self._validated(timestamp_ns, values)
        self.prediction_count += 1
        position = bisect_left([item.timestamp_ns for item in self.predictions], sample.timestamp_ns)
        self.predictions.insert(position, sample)
        pending_position = bisect_left([item.timestamp_ns for item in self._pending], sample.timestamp_ns)
        self._pending.insert(pending_position, sample)
        return self._drain_pending()

    def _interpolate(self, prediction: ActionSample) -> MatchedSample | None:
        if not self.references:
            return None
        right = bisect_left(self._reference_times, prediction.timestamp_ns)
        if right < len(self.references) and self._reference_times[right] == prediction.timestamp_ns:
            reference = self.references[right].values.copy()
            return MatchedSample(prediction.timestamp_ns, reference, prediction.values.copy())
        if right == 0 or right >= len(self.references):
            return None
        left_sample = self.references[right - 1]
        right_sample = self.references[right]
        left_gap = prediction.timestamp_ns - left_sample.timestamp_ns
        right_gap = right_sample.timestamp_ns - prediction.timestamp_ns
        if left_gap > self.max_gap_ns and right_gap > self.max_gap_ns:
            return None
        if left_gap > self.max_gap_ns or right_gap > self.max_gap_ns:
            nearest = left_sample if left_gap <= right_gap else right_sample
            return MatchedSample(
                prediction.timestamp_ns,
                nearest.values.copy(),
                prediction.values.copy(),
            )
        span = right_sample.timestamp_ns - left_sample.timestamp_ns
        weight = 0.0 if span <= 0 else left_gap / span
        reference = left_sample.values.copy()
        reference[JOINT_INDICES] = (
            left_sample.values[JOINT_INDICES] * (1.0 - weight)
            + right_sample.values[JOINT_INDICES] * weight
        )
        reference[GRIPPER_INDICES] = left_sample.values[GRIPPER_INDICES]
        return MatchedSample(prediction.timestamp_ns, reference, prediction.values.copy())

    def _is_expired(self, prediction: ActionSample) -> bool:
        return bool(
            self.references
            and self.references[-1].timestamp_ns > prediction.timestamp_ns + self.max_gap_ns
        )

    def _drain_pending(self) -> list[MatchedSample]:
        produced: list[MatchedSample] = []
        remaining: list[ActionSample] = []
        for prediction in self._pending:
            match = self._interpolate(prediction)
            if match is not None:
                self.matches.append(match)
                produced.append(match)
            elif self._is_expired(prediction):
                self.unmatched_prediction_count += 1
            else:
                remaining.append(prediction)
        self._pending = remaining
        self.matches.sort(key=lambda item: item.timestamp_ns)
        return produced

    def finalize(self) -> list[MatchedSample]:
        produced = self._drain_pending()
        for prediction in self._pending:
            if not self.references:
                self.unmatched_prediction_count += 1
                continue
            position = bisect_left(self._reference_times, prediction.timestamp_ns)
            candidates = []
            if position > 0:
                candidates.append(self.references[position - 1])
            if position < len(self.references):
                candidates.append(self.references[position])
            nearest = min(
                candidates,
                key=lambda sample: abs(sample.timestamp_ns - prediction.timestamp_ns),
            )
            if abs(nearest.timestamp_ns - prediction.timestamp_ns) <= self.max_gap_ns:
                match = MatchedSample(
                    prediction.timestamp_ns,
                    nearest.values.copy(),
                    prediction.values.copy(),
                )
                self.matches.append(match)
                produced.append(match)
            else:
                self.unmatched_prediction_count += 1
        self._pending = []
        self.matches.sort(key=lambda item: item.timestamp_ns)
        return produced
