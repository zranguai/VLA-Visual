from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from .constants import DATASET_SIGNAL_NAMES


@dataclass(frozen=True)
class DatasetMetadata:
    root: Path
    episode_index: int
    fps: float
    total_episodes: int
    splits: dict[str, str]
    action_q01: np.ndarray
    action_q99: np.ndarray


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def _feature_names(info: dict[str, Any]) -> list[str]:
    features = _mapping(info.get("features"), "meta.info.features")
    action = _mapping(features.get("action"), "meta.info.features.action")
    names = action.get("names")
    if isinstance(names, list) and len(names) == 1 and isinstance(names[0], list):
        names = names[0]
    if not isinstance(names, list) or not all(isinstance(name, str) for name in names):
        raise ValueError("action metadata has no usable named dimensions")
    return names


def load_dataset_metadata(replay_config_path: str | Path) -> DatasetMetadata:
    replay_path = Path(replay_config_path).expanduser().resolve()
    if not replay_path.is_file():
        raise FileNotFoundError(f"replay config does not exist: {replay_path}")
    replay = _mapping(yaml.safe_load(replay_path.read_text(encoding="utf-8")), "replay")
    dataset = _mapping(replay.get("dataset"), "replay.dataset")
    root = Path(str(dataset.get("root", ""))).expanduser().resolve()
    episode_index = int(dataset.get("episode_index", 0))

    info_path = root / "meta" / "info.json"
    stats_path = root / "meta" / "stats.json"
    if not info_path.is_file() or not stats_path.is_file():
        raise FileNotFoundError(f"dataset metadata is incomplete under: {root / 'meta'}")
    info = _mapping(json.loads(info_path.read_text(encoding="utf-8")), "meta.info")
    stats = _mapping(json.loads(stats_path.read_text(encoding="utf-8")), "meta.stats")
    if info.get("codebase_version") != "v3.0":
        raise ValueError(f"expected LeRobot v3.0 metadata, got {info.get('codebase_version')!r}")

    names = _feature_names(info)
    missing = [name for name in DATASET_SIGNAL_NAMES if name not in names]
    if missing:
        raise ValueError(f"action metadata is missing dimensions: {missing}")
    indices = np.asarray([names.index(name) for name in DATASET_SIGNAL_NAMES], dtype=np.int64)
    action_stats = _mapping(stats.get("action"), "meta.stats.action")
    q01 = np.asarray(action_stats.get("q01"), dtype=np.float64)
    q99 = np.asarray(action_stats.get("q99"), dtype=np.float64)
    if q01.shape != (16,) or q99.shape != (16,) or np.any(q99 <= q01):
        raise ValueError("action q01/q99 must be finite increasing 16D vectors")
    q01 = q01[indices]
    q99 = q99[indices]
    if not np.all(np.isfinite(q01)) or not np.all(np.isfinite(q99)):
        raise ValueError("action q01/q99 contain non-finite values")

    splits_raw = info.get("splits", {})
    splits = (
        {str(key): str(value) for key, value in splits_raw.items()}
        if isinstance(splits_raw, dict)
        else {}
    )
    return DatasetMetadata(
        root=root,
        episode_index=episode_index,
        fps=float(info.get("fps", 0.0)),
        total_episodes=int(info.get("total_episodes", 0)),
        splits=splits,
        action_q01=q01,
        action_q99=q99,
    )
