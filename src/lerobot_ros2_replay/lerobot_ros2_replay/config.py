from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class TopicConfig:
    name: str
    hz: float


@dataclass(frozen=True)
class CameraConfig(TopicConfig):
    feature_key: str
    frame_id: str


@dataclass(frozen=True)
class DatasetConfig:
    root: Path
    episode_index: int
    video_backend: str


@dataclass(frozen=True)
class PlaybackConfig:
    speed: float
    loop: bool
    start_frame: int
    end_frame: int | None


@dataclass(frozen=True)
class ReplayConfig:
    ros_domain_id: int
    dataset: DatasetConfig
    playback: PlaybackConfig
    arm_status: TopicConfig
    action: TopicConfig
    cameras: dict[str, CameraConfig]

    @property
    def enabled_rates(self) -> list[float]:
        rates = [self.arm_status.hz, self.action.hz]
        rates.extend(camera.hz for camera in self.cameras.values())
        return [rate for rate in rates if rate > 0]


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def _required(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ValueError(f"missing config key: {path}.{key}")
    return mapping[key]


def _topic(mapping: dict[str, Any], path: str) -> TopicConfig:
    topic = TopicConfig(
        name=str(_required(mapping, "name", path)),
        hz=float(_required(mapping, "hz", path)),
    )
    _validate_topic(topic, path)
    return topic


def _validate_topic(topic: TopicConfig, path: str) -> None:
    if not topic.name.startswith("/"):
        raise ValueError(f"{path}.name must be an absolute ROS topic")
    if topic.hz < 0:
        raise ValueError(f"{path}.hz must be >= 0")


def load_config(path: str | Path) -> ReplayConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"config file does not exist: {config_path}")
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    root = _mapping(raw, "config")

    dataset_raw = _mapping(_required(root, "dataset", "config"), "dataset")
    playback_raw = _mapping(_required(root, "playback", "config"), "playback")
    topics_raw = _mapping(_required(root, "topics", "config"), "topics")
    cameras_raw = _mapping(_required(topics_raw, "cameras", "topics"), "topics.cameras")

    dataset = DatasetConfig(
        root=Path(str(_required(dataset_raw, "root", "dataset"))).expanduser().resolve(),
        episode_index=int(dataset_raw.get("episode_index", 0)),
        video_backend=str(dataset_raw.get("video_backend", "pyav")),
    )
    end_frame_raw = playback_raw.get("end_frame")
    playback = PlaybackConfig(
        speed=float(playback_raw.get("speed", 1.0)),
        loop=bool(playback_raw.get("loop", False)),
        start_frame=int(playback_raw.get("start_frame", 0)),
        end_frame=None if end_frame_raw is None else int(end_frame_raw),
    )

    cameras: dict[str, CameraConfig] = {}
    for role in ("top", "left", "right"):
        camera_raw = _mapping(_required(cameras_raw, role, "topics.cameras"), f"topics.cameras.{role}")
        topic = _topic(camera_raw, f"topics.cameras.{role}")
        camera = CameraConfig(
            name=topic.name,
            hz=topic.hz,
            feature_key=str(_required(camera_raw, "feature_key", f"topics.cameras.{role}")),
            frame_id=str(camera_raw.get("frame_id", role)),
        )
        cameras[role] = camera

    config = ReplayConfig(
        ros_domain_id=int(root.get("ros_domain_id", 100)),
        dataset=dataset,
        playback=playback,
        arm_status=_topic(
            _mapping(_required(topics_raw, "arm_status", "topics"), "topics.arm_status"),
            "topics.arm_status",
        ),
        action=_topic(
            _mapping(_required(topics_raw, "action", "topics"), "topics.action"),
            "topics.action",
        ),
        cameras=cameras,
    )
    _validate_config(config)
    return config


def _validate_config(config: ReplayConfig) -> None:
    if not 0 <= config.ros_domain_id <= 232:
        raise ValueError("ros_domain_id must be in the Fast DDS range 0..232")
    if config.ros_domain_id == 38:
        raise ValueError("ROS domain 38 is reserved for the real robot")
    if config.dataset.episode_index < 0:
        raise ValueError("dataset.episode_index must be >= 0")
    if config.dataset.video_backend != "pyav":
        raise ValueError("dataset.video_backend must be 'pyav' for the sample AV1 videos")
    if config.playback.speed <= 0:
        raise ValueError("playback.speed must be > 0")
    if config.playback.start_frame < 0:
        raise ValueError("playback.start_frame must be >= 0")
    if config.playback.end_frame is not None and config.playback.end_frame <= config.playback.start_frame:
        raise ValueError("playback.end_frame must be greater than playback.start_frame")
    if not config.enabled_rates:
        raise ValueError("at least one topic must have hz > 0")

    names = [config.arm_status.name, config.action.name]
    names.extend(camera.name for camera in config.cameras.values())
    if len(names) != len(set(names)):
        raise ValueError("all replay topic names must be unique")
