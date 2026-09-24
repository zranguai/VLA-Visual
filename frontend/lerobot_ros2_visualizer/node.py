from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np
import rclpy
import rerun as rr
from arm_interfaces.msg import ArmStatus, AutonomyArmCommand
from custom_image_msg.msg import Image1m
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from .blueprint import build_blueprint, camera_entity_path, log_series_styles, signal_entity_path
from .config import VisualizationConfig, load_config
from .converters import RateGate, decode_image1m, extract_action, extract_state, message_timestamp_ns


class RerunVisualizerNode(Node):
    def __init__(self, config: VisualizationConfig) -> None:
        super().__init__(
            "lerobot_ros2_visualizer",
            enable_rosout=False,
            start_parameter_services=False,
        )
        # Humble creates this publisher unconditionally; the bridge must remain subscription-only.
        parameter_event_publisher = self._parameter_event_publisher
        self._parameter_event_publisher = None
        self.destroy_publisher(parameter_event_publisher)
        self.config = config
        self._warning_times: dict[str, float] = {}
        self._topic_subscriptions: list[Any] = []
        self._camera_gates = {
            role: RateGate(config.cameras.sample_hz) for role in config.cameras.enabled
        }

        reliable_state = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_action = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        if "state" in config.signals.sources:
            self._topic_subscriptions.append(
                self.create_subscription(
                    ArmStatus, config.topics.arm_status, self._state_callback, reliable_state
                )
            )
        action_sources = (
            ("dataset_action", config.topics.dataset_action),
            ("model_action", config.topics.model_action),
        )
        for source, topic in action_sources:
            if source in config.signals.sources:
                self._topic_subscriptions.append(
                    self.create_subscription(
                        AutonomyArmCommand,
                        topic,
                        self._action_callback(source),
                        reliable_action,
                    )
                )
        for role in config.cameras.enabled:
            self._topic_subscriptions.append(
                self.create_subscription(
                    Image1m,
                    config.topics.cameras[role],
                    self._image_callback(role),
                    camera_qos,
                )
            )

        sources = ", ".join(config.signals.sources)
        cameras = ", ".join(config.cameras.enabled)
        self.get_logger().info(
            f"Rerun visualizer ready | ROS_DOMAIN_ID={config.ros_domain_id} | "
            f"cameras={cameras} @ {config.cameras.sample_hz:g}Hz | sources={sources}"
        )

    def _timestamp_ns(self, msg: Any) -> int:
        return message_timestamp_ns(msg, self.get_clock().now().nanoseconds)

    def _set_time(self, timestamp_ns: int) -> None:
        rr.set_time(
            self.config.viewer.timeline,
            timestamp=np.datetime64(timestamp_ns, "ns"),
        )

    def _warn_limited(self, key: str, message: str) -> None:
        now = time.monotonic()
        if now - self._warning_times.get(key, float("-inf")) >= 5.0:
            self._warning_times[key] = now
            self.get_logger().warning(message)

    def _log_signals(self, msg: Any, source: str, converter: Callable[[Any], dict[str, float]]) -> None:
        try:
            values = converter(msg)
        except ValueError as exc:
            self._warn_limited(source, f"Dropped invalid {source} message: {exc}")
            return
        self._set_time(self._timestamp_ns(msg))
        for signal in self.config.signals.enabled:
            rr.log(signal_entity_path(signal, source), rr.Scalars(values[signal]))

    def _state_callback(self, msg: ArmStatus) -> None:
        self._log_signals(msg, "state", extract_state)

    def _action_callback(self, source: str) -> Callable[[AutonomyArmCommand], None]:
        def callback(msg: AutonomyArmCommand) -> None:
            self._log_signals(msg, source, extract_action)

        return callback

    def _image_callback(self, role: str) -> Callable[[Image1m], None]:
        def callback(msg: Image1m) -> None:
            timestamp_ns = self._timestamp_ns(msg)
            if not self._camera_gates[role].due(timestamp_ns):
                return
            try:
                image = decode_image1m(msg)
            except ValueError as exc:
                self._warn_limited(f"camera:{role}", f"Dropped invalid {role} image: {exc}")
                return
            self._set_time(timestamp_ns)
            rr.log(
                camera_entity_path(role),
                rr.Image(image).compress(jpeg_quality=self.config.cameras.jpeg_quality),
            )

        return callback


def _parse_cli(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize LeRobot ROS 2 replay topics in Rerun")
    parser.add_argument("--config", required=True, help="Path to visualizer YAML configuration")
    parser.add_argument(
        "--save",
        metavar="PATH",
        help="Write a headless Rerun .rrd recording instead of spawning the Viewer",
    )
    return parser.parse_args(remove_ros_args(args=[sys.argv[0], *argv])[1:])


def _configure_domain(config: VisualizationConfig) -> None:
    current = os.environ.get("ROS_DOMAIN_ID")
    if current is not None:
        try:
            current_domain = int(current)
        except ValueError as exc:
            raise RuntimeError(f"invalid ROS_DOMAIN_ID={current!r}") from exc
        if current_domain != config.ros_domain_id:
            raise RuntimeError(
                f"ROS_DOMAIN_ID={current_domain} conflicts with visualization domain "
                f"{config.ros_domain_id}"
            )
    os.environ["ROS_DOMAIN_ID"] = str(config.ros_domain_id)


def _initialize_rerun(config: VisualizationConfig, save_path: str | None) -> None:
    blueprint = build_blueprint(config)
    rr.init(
        config.viewer.application_id,
        spawn=False,
        default_blueprint=blueprint,
    )
    if save_path:
        output_path = Path(save_path).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        rr.save(output_path, default_blueprint=blueprint)
    else:
        rr.spawn(
            memory_limit=config.viewer.memory_limit,
            hide_welcome_screen=True,
            default_blueprint=blueprint,
        )
    log_series_styles(config)


def main(args: list[str] | None = None) -> None:
    cli = _parse_cli(sys.argv[1:] if args is None else args)
    config = load_config(cli.config)
    _configure_domain(config)
    _initialize_rerun(config, cli.save)

    rclpy.init(args=[])
    node: RerunVisualizerNode | None = None
    try:
        node = RerunVisualizerNode(config)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        rr.disconnect()


if __name__ == "__main__":
    main()
