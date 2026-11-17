from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Any

import rclpy
from arm_interfaces.msg import AutonomyArmCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from .config import EvaluationConfig, load_config
from .messages import action_vector, message_timestamp_ns
from .metadata import DatasetMetadata, load_dataset_metadata
from .metrics import EvaluationSession
from .report import make_run_directory, summary_markdown, write_report


class ModelEvaluationNode(Node):
    def __init__(
        self,
        config: EvaluationConfig,
        metadata: DatasetMetadata,
        output_directory: Path,
        run_label: str,
    ) -> None:
        super().__init__(
            "eval_model",
            enable_rosout=False,
            start_parameter_services=False,
        )
        parameter_event_publisher = self._parameter_event_publisher
        self._parameter_event_publisher = None
        self.destroy_publisher(parameter_event_publisher)
        self.config = config
        self.output_directory = output_directory
        self.run_label = run_label
        self.session = EvaluationSession(config, metadata)
        self.done = False
        self._written = False
        self._last_reference_monotonic: float | None = None
        self._last_status_monotonic = time.monotonic()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._topic_subscriptions: list[Any] = [
            self.create_subscription(
                AutonomyArmCommand,
                config.topics.reference,
                self._reference_callback,
                qos,
            ),
            self.create_subscription(
                AutonomyArmCommand,
                config.topics.prediction,
                self._prediction_callback,
                qos,
            ),
        ]
        self._status_timer = self.create_timer(0.5, self._check_status)
        self.get_logger().info(
            f"Action evaluator ready | ROS_DOMAIN_ID={config.ros_domain_id} | "
            f"reference={config.topics.reference} | prediction={config.topics.prediction}"
        )

    def _timestamp(self, msg: Any) -> int:
        return message_timestamp_ns(msg, self.get_clock().now().nanoseconds)

    def _reference_callback(self, msg: AutonomyArmCommand) -> None:
        try:
            self.session.add_reference(self._timestamp(msg), action_vector(msg))
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid reference action: {exc}")
            return
        self._last_reference_monotonic = time.monotonic()

    def _prediction_callback(self, msg: AutonomyArmCommand) -> None:
        try:
            self.session.add_prediction(self._timestamp(msg), action_vector(msg))
        except ValueError as exc:
            self.get_logger().warning(f"Dropped invalid prediction action: {exc}")

    def _check_status(self) -> None:
        now = time.monotonic()
        if now - self._last_status_monotonic >= self.config.report.status_interval_s:
            aligner = self.session.aligner
            self.get_logger().info(
                f"evaluation progress | reference={aligner.reference_count} | "
                f"prediction={aligner.prediction_count} | matched={len(aligner.matches)}"
            )
            self._last_status_monotonic = now
        if (
            self._last_reference_monotonic is not None
            and now - self._last_reference_monotonic >= self.config.report.idle_timeout_s
        ):
            self.finalize()
            self.done = True

    def finalize(self) -> Path:
        if self._written:
            return self.output_directory / "summary.json"
        summary = self.session.summary(self.run_label)
        summary_path, _ = write_report(
            self.output_directory,
            summary,
            self.session.aligner.matches,
            self.session.normalizer,
        )
        self._written = True
        self.get_logger().info(f"Evaluation report written: {summary_path}")
        for line in summary_markdown(summary).splitlines():
            if line and not line.startswith("#"):
                self.get_logger().info(line)
        return summary_path


def _parse_cli(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate model actions against replay actions")
    parser.add_argument("--config", required=True, help="Evaluation YAML configuration")
    parser.add_argument("--replay-config", required=True, help="Replay YAML configuration")
    parser.add_argument("--output-dir", help="Exact output directory for this run")
    parser.add_argument("--run-label", default="model", help="Model/checkpoint label")
    return parser.parse_args(remove_ros_args(args=[sys.argv[0], *argv])[1:])


def _configure_domain(config: EvaluationConfig) -> None:
    current = os.environ.get("ROS_DOMAIN_ID")
    if current is not None:
        try:
            domain = int(current)
        except ValueError as exc:
            raise RuntimeError(f"invalid ROS_DOMAIN_ID={current!r}") from exc
        if domain != config.ros_domain_id:
            raise RuntimeError(
                f"ROS_DOMAIN_ID={domain} conflicts with evaluation domain {config.ros_domain_id}"
            )
    os.environ["ROS_DOMAIN_ID"] = str(config.ros_domain_id)


def main(args: list[str] | None = None) -> None:
    cli = _parse_cli(sys.argv[1:] if args is None else args)
    config = load_config(cli.config)
    metadata = load_dataset_metadata(cli.replay_config)
    _configure_domain(config)
    output_directory = (
        Path(cli.output_dir).expanduser().resolve()
        if cli.output_dir
        else make_run_directory(config.report.output_root, cli.run_label)
    )

    rclpy.init(args=[])
    node: ModelEvaluationNode | None = None
    try:
        node = ModelEvaluationNode(config, metadata, output_directory, cli.run_label)
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.finalize()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
