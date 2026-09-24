from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from arm_interfaces.msg import ArmStatus, AutonomyArmCommand
from custom_image_msg.msg import Image1m
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args

from .config import CameraConfig, ReplayConfig, load_config
from .dataset_reader import LeRobotEpisodeReader
from .message_builders import build_action, build_arm_status, build_image


@dataclass
class RateGate:
    hz: float
    next_due_s: float = 0.0

    def due(self, elapsed_s: float) -> bool:
        if self.hz <= 0 or elapsed_s + 1e-9 < self.next_due_s:
            return False
        period = 1.0 / self.hz
        skipped = int((elapsed_s - self.next_due_s) // period) + 1
        self.next_due_s += skipped * period
        return True


class LeRobotReplayNode(Node):
    def __init__(self, config: ReplayConfig) -> None:
        super().__init__("lerobot_ros2_replay")
        self.config = config
        self.reader = LeRobotEpisodeReader(
            root=config.dataset.root,
            episode_index=config.dataset.episode_index,
            video_backend=config.dataset.video_backend,
            start_frame=config.playback.start_frame,
            end_frame=config.playback.end_frame,
        )
        reliable_state = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_action = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        camera_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.arm_publisher = self.create_publisher(ArmStatus, config.arm_status.name, reliable_state)
        self.action_publisher = self.create_publisher(
            AutonomyArmCommand, config.action.name, reliable_action
        )
        self.camera_publishers = {
            role: self.create_publisher(Image1m, camera.name, camera_qos)
            for role, camera in config.cameras.items()
        }
        self.gates = {
            "arm_status": RateGate(config.arm_status.hz),
            "action": RateGate(config.action.hz),
            **{f"camera:{role}": RateGate(camera.hz) for role, camera in config.cameras.items()},
        }
        self.finished = False
        self._start_monotonic = time.monotonic()
        max_hz = max(config.enabled_rates)
        timer_period = min(0.01, 0.5 / max_hz)
        self.timer = self.create_timer(timer_period, self._tick)
        self.get_logger().info(
            f"Loaded episode {config.dataset.episode_index}: {self.reader.length} frames, "
            f"{self.reader.fps:g} fps, {self.reader.duration_s:.3f}s | "
            f"ROS_DOMAIN_ID={config.ros_domain_id}"
        )

    def _tick(self) -> None:
        elapsed_s = time.monotonic() - self._start_monotonic
        source_elapsed_s = elapsed_s * self.config.playback.speed
        if source_elapsed_s >= self.reader.duration_s:
            if self.config.playback.loop:
                source_elapsed_s %= self.reader.duration_s
            else:
                self.finished = True
                self.timer.cancel()
                self.get_logger().info("Episode replay completed.")
                return

        due_arm = self.gates["arm_status"].due(elapsed_s)
        due_action = self.gates["action"].due(elapsed_s)
        due_cameras = {
            role: self.gates[f"camera:{role}"].due(elapsed_s)
            for role in self.config.cameras
        }
        if not (due_arm or due_action or any(due_cameras.values())):
            return

        frame_index = self.reader.source_index_at(source_elapsed_s)
        stamp = self.get_clock().now().to_msg()
        if due_arm or due_action:
            state, action = self.reader.low_dimensional(frame_index)
            if due_arm:
                self.arm_publisher.publish(build_arm_status(stamp, state))
            if due_action:
                self.action_publisher.publish(build_action(stamp, action))

        camera_roles = [role for role, due in due_cameras.items() if due]
        if camera_roles:
            feature_keys = [self.config.cameras[role].feature_key for role in camera_roles]
            images = self.reader.images(frame_index, feature_keys)
            for role in camera_roles:
                camera: CameraConfig = self.config.cameras[role]
                self.camera_publishers[role].publish(
                    build_image(stamp, images[camera.feature_key], camera.frame_id)
                )


def _parse_cli(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a LeRobot v3 episode as ROS 2 topics")
    parser.add_argument("--config", required=True, help="Path to replay YAML configuration")
    return parser.parse_args(remove_ros_args(args=[sys.argv[0], *argv])[1:])


def _configure_domain(config: ReplayConfig) -> None:
    current = os.environ.get("ROS_DOMAIN_ID")
    if current is not None and int(current) != config.ros_domain_id:
        raise RuntimeError(
            f"ROS_DOMAIN_ID={current} conflicts with replay config domain {config.ros_domain_id}"
        )
    os.environ["ROS_DOMAIN_ID"] = str(config.ros_domain_id)


def main(args: list[str] | None = None) -> None:
    argv = sys.argv[1:] if args is None else args
    cli = _parse_cli(argv)
    config = load_config(Path(cli.config))
    _configure_domain(config)
    # The replay CLI arguments are application-specific, not ROS arguments.
    rclpy.init(args=[])
    node: LeRobotReplayNode | None = None
    try:
        node = LeRobotReplayNode(config)
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
