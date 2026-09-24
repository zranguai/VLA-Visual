from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


CAMERA_ROLES = ("top", "left", "right")
SOURCE_NAMES = ("state", "dataset_action", "model_action")
SIGNAL_NAMES = (
    *(f"left_joint_{index}" for index in range(7)),
    "left_gripper",
    *(f"right_joint_{index}" for index in range(7)),
    "right_gripper",
)


@dataclass(frozen=True)
class TopicConfig:
    arm_status: str
    dataset_action: str
    model_action: str
    cameras: dict[str, str]


@dataclass(frozen=True)
class CameraDisplayConfig:
    enabled: tuple[str, ...]
    default_view: str
    sample_hz: float
    jpeg_quality: int


@dataclass(frozen=True)
class SignalDisplayConfig:
    enabled: tuple[str, ...]
    sources: tuple[str, ...]
    default_visible_sources: tuple[str, ...]
    default_signal: str


@dataclass(frozen=True)
class ViewerConfig:
    application_id: str
    memory_limit: str
    timeline: str
    history_window_s: float


@dataclass(frozen=True)
class VisualizationConfig:
    ros_domain_id: int
    topics: TopicConfig
    cameras: CameraDisplayConfig
    signals: SignalDisplayConfig
    viewer: ViewerConfig


def _mapping(value: Any, path: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    return value


def _required(mapping: dict[str, Any], key: str, path: str) -> Any:
    if key not in mapping:
        raise ValueError(f"missing {path}.{key}")
    return mapping[key]


def _string(value: Any, path: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty string")
    return value.strip()


def _names(value: Any, allowed: tuple[str, ...], path: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not value:
        raise ValueError(f"{path} must be a non-empty list")
    names = tuple(_string(item, path) for item in value)
    if len(set(names)) != len(names):
        raise ValueError(f"{path} must not contain duplicates")
    unknown = sorted(set(names) - set(allowed))
    if unknown:
        raise ValueError(f"{path} contains unsupported values: {', '.join(unknown)}")
    return names


def _topic(value: Any, path: str) -> str:
    topic = _string(value, path)
    if not topic.startswith("/"):
        raise ValueError(f"{path} must be an absolute ROS topic")
    if topic == "/execute_arm_cmd":
        raise ValueError(f"{path} must not use the real robot command topic /execute_arm_cmd")
    return topic


def load_config(path: str | Path) -> VisualizationConfig:
    config_path = Path(path).expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"visualizer config does not exist: {config_path}")
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    root = _mapping(raw, "config")

    ros_domain_id = int(_required(root, "ros_domain_id", "config"))
    if ros_domain_id != 100:
        raise ValueError("offline visualization requires ros_domain_id 100")

    topics_raw = _mapping(_required(root, "topics", "config"), "topics")
    camera_topics_raw = _mapping(_required(topics_raw, "cameras", "topics"), "topics.cameras")
    camera_topics = {
        role: _topic(_required(camera_topics_raw, role, "topics.cameras"), f"topics.cameras.{role}")
        for role in CAMERA_ROLES
    }
    topics = TopicConfig(
        arm_status=_topic(_required(topics_raw, "arm_status", "topics"), "topics.arm_status"),
        dataset_action=_topic(
            _required(topics_raw, "dataset_action", "topics"), "topics.dataset_action"
        ),
        model_action=_topic(
            _required(topics_raw, "model_action", "topics"), "topics.model_action"
        ),
        cameras=camera_topics,
    )
    all_topics = [topics.arm_status, topics.dataset_action, topics.model_action, *camera_topics.values()]
    if len(set(all_topics)) != len(all_topics):
        raise ValueError("configured ROS topics must be unique")

    cameras_raw = _mapping(_required(root, "cameras", "config"), "cameras")
    enabled_cameras = _names(
        _required(cameras_raw, "enabled", "cameras"), CAMERA_ROLES, "cameras.enabled"
    )
    default_view = _string(
        _required(cameras_raw, "default_view", "cameras"), "cameras.default_view"
    )
    if default_view != "all" and default_view not in enabled_cameras:
        raise ValueError("cameras.default_view must be 'all' or an enabled camera")
    sample_hz = float(_required(cameras_raw, "sample_hz", "cameras"))
    if sample_hz <= 0:
        raise ValueError("cameras.sample_hz must be greater than zero")
    jpeg_quality = int(_required(cameras_raw, "jpeg_quality", "cameras"))
    if not 1 <= jpeg_quality <= 100:
        raise ValueError("cameras.jpeg_quality must be between 1 and 100")
    cameras = CameraDisplayConfig(
        enabled=enabled_cameras,
        default_view=default_view,
        sample_hz=sample_hz,
        jpeg_quality=jpeg_quality,
    )

    signals_raw = _mapping(_required(root, "signals", "config"), "signals")
    enabled_signals = _names(
        _required(signals_raw, "enabled", "signals"), SIGNAL_NAMES, "signals.enabled"
    )
    sources = _names(
        _required(signals_raw, "sources", "signals"), SOURCE_NAMES, "signals.sources"
    )
    default_visible_sources = _names(
        _required(signals_raw, "default_visible_sources", "signals"),
        SOURCE_NAMES,
        "signals.default_visible_sources",
    )
    if not set(default_visible_sources).issubset(sources):
        raise ValueError("signals.default_visible_sources must be a subset of signals.sources")
    default_signal = _string(
        _required(signals_raw, "default_signal", "signals"), "signals.default_signal"
    )
    if default_signal not in enabled_signals:
        raise ValueError("signals.default_signal must be enabled")
    signals = SignalDisplayConfig(
        enabled=enabled_signals,
        sources=sources,
        default_visible_sources=default_visible_sources,
        default_signal=default_signal,
    )

    viewer_raw = _mapping(_required(root, "viewer", "config"), "viewer")
    history_window_s = float(
        _required(viewer_raw, "history_window_s", "viewer")
    )
    if history_window_s <= 0:
        raise ValueError("viewer.history_window_s must be greater than zero")
    viewer = ViewerConfig(
        application_id=_string(
            _required(viewer_raw, "application_id", "viewer"), "viewer.application_id"
        ),
        memory_limit=_string(
            _required(viewer_raw, "memory_limit", "viewer"), "viewer.memory_limit"
        ),
        timeline=_string(_required(viewer_raw, "timeline", "viewer"), "viewer.timeline"),
        history_window_s=history_window_s,
    )

    return VisualizationConfig(
        ros_domain_id=ros_domain_id,
        topics=topics,
        cameras=cameras,
        signals=signals,
        viewer=viewer,
    )
