from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Iterable

import numpy as np


RIGHT_NAMES = [*(f"right_joint{i}" for i in range(1, 8)), "right_gripper"]
LEFT_NAMES = [*(f"left_joint{i}" for i in range(1, 8)), "left_gripper"]
ROS_ORDER_NAMES = [*LEFT_NAMES, *RIGHT_NAMES]


def _feature_names(feature: dict[str, Any], key: str) -> list[str]:
    names = feature.get("names")
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise ValueError(f"feature {key!r} has no usable named dimensions")
    return names


class LeRobotEpisodeReader:
    """Read one LeRobot v3 episode and expose robot-order vectors and RGB frames."""

    def __init__(
        self,
        root: str | Path,
        episode_index: int,
        video_backend: str = "pyav",
        start_frame: int = 0,
        end_frame: int | None = None,
    ) -> None:
        self.root = Path(root).expanduser().resolve()
        info_path = self.root / "meta" / "info.json"
        if not info_path.is_file():
            raise FileNotFoundError(f"missing LeRobot metadata: {info_path}")
        self.info = json.loads(info_path.read_text(encoding="utf-8"))
        if self.info.get("codebase_version") != "v3.0":
            raise ValueError(
                f"expected LeRobot codebase_version v3.0, got {self.info.get('codebase_version')!r}"
            )

        features = self.info.get("features")
        if not isinstance(features, dict):
            raise ValueError("meta/info.json has no features mapping")
        for key in ("observation.state", "action"):
            if key not in features:
                raise ValueError(f"dataset is missing required feature {key!r}")
        self._state_indices = self._build_reorder_indices(features["observation.state"], "observation.state")
        self._action_indices = self._build_reorder_indices(features["action"], "action")

        try:
            from lerobot.datasets.lerobot_dataset import LeRobotDataset
        except ImportError as exc:
            raise RuntimeError("LeRobot is not installed in the active Python environment") from exc

        repo_id = f"local/{self.root.name}"
        self.dataset = LeRobotDataset(
            repo_id=repo_id,
            root=self.root,
            episodes=[int(episode_index)],
            video_backend=video_backend,
        )
        episode_length = len(self.dataset)
        if episode_length <= 0:
            raise ValueError(f"episode {episode_index} is empty or unavailable")
        if start_frame >= episode_length:
            raise ValueError(f"start_frame {start_frame} is outside episode length {episode_length}")
        stop = episode_length if end_frame is None else end_frame
        if stop > episode_length:
            raise ValueError(f"end_frame {stop} is outside episode length {episode_length}")

        self.start_frame = int(start_frame)
        self.end_frame = int(stop)
        self.length = self.end_frame - self.start_frame
        self.fps = float(self.dataset.fps)
        timestamps = np.asarray(self.dataset.hf_dataset["timestamp"], dtype=np.float64)
        self.timestamps = timestamps[self.start_frame : self.end_frame]
        if self.timestamps.size != self.length or np.any(np.diff(self.timestamps) < 0):
            raise ValueError("episode timestamps are missing or not monotonic")
        self.timestamps = self.timestamps - self.timestamps[0]
        self.duration_s = float(self.timestamps[-1] + 1.0 / self.fps)
        self._image_cache_index: int | None = None
        self._image_cache: dict[str, np.ndarray] = {}

    @staticmethod
    def _build_reorder_indices(feature: dict[str, Any], key: str) -> np.ndarray:
        names = _feature_names(feature, key)
        if len(names) != 16:
            raise ValueError(f"feature {key!r} must have 16 dimensions, got {len(names)}")
        missing = [name for name in ROS_ORDER_NAMES if name not in names]
        if missing:
            raise ValueError(f"feature {key!r} is missing dimensions: {missing}")
        return np.asarray([names.index(name) for name in ROS_ORDER_NAMES], dtype=np.int64)

    def _absolute_index(self, frame_index: int) -> int:
        if not 0 <= frame_index < self.length:
            raise IndexError(f"frame index {frame_index} is outside 0..{self.length - 1}")
        return self.start_frame + frame_index

    def source_index_at(self, source_time_s: float) -> int:
        index = int(np.searchsorted(self.timestamps, source_time_s, side="right") - 1)
        return int(np.clip(index, 0, self.length - 1))

    def low_dimensional(self, frame_index: int) -> tuple[np.ndarray, np.ndarray]:
        row = self.dataset.hf_dataset[self._absolute_index(frame_index)]
        state = np.asarray(row["observation.state"], dtype=np.float32)
        action = np.asarray(row["action"], dtype=np.float32)
        if state.shape != (16,) or action.shape != (16,):
            raise ValueError(f"frame {frame_index} has invalid state/action shapes {state.shape}/{action.shape}")
        return state[self._state_indices], action[self._action_indices]

    def images(self, frame_index: int, feature_keys: Iterable[str]) -> dict[str, np.ndarray]:
        absolute_index = self._absolute_index(frame_index)
        keys = tuple(feature_keys)
        missing_features = [key for key in keys if key not in self.info["features"]]
        if missing_features:
            raise ValueError(f"dataset is missing camera features: {missing_features}")
        if self._image_cache_index != absolute_index:
            item = self.dataset[absolute_index]
            self._image_cache = {key: self._to_rgb_uint8(item[key], key) for key in keys}
            self._image_cache_index = absolute_index
        else:
            uncached = [key for key in keys if key not in self._image_cache]
            if uncached:
                item = self.dataset[absolute_index]
                for key in uncached:
                    self._image_cache[key] = self._to_rgb_uint8(item[key], key)
        return {key: self._image_cache[key] for key in keys}

    @staticmethod
    def _to_rgb_uint8(value: Any, key: str) -> np.ndarray:
        if hasattr(value, "detach"):
            value = value.detach().cpu().numpy()
        image = np.asarray(value)
        if image.ndim != 3:
            raise ValueError(f"camera {key!r} must be a 3D tensor, got shape {image.shape}")
        if image.shape[0] == 3:
            image = np.transpose(image, (1, 2, 0))
        if image.shape[-1] != 3:
            raise ValueError(f"camera {key!r} must contain three channels, got shape {image.shape}")
        if np.issubdtype(image.dtype, np.floating):
            image = np.rint(np.clip(image, 0.0, 1.0) * 255.0)
        image = np.clip(image, 0, 255).astype(np.uint8, copy=False)
        return np.ascontiguousarray(image)
