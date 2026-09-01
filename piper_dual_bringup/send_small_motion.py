#!/usr/bin/env python3
"""Publish one conservative out-and-back JointTrajectory reference."""

from __future__ import annotations

import argparse
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from safety import choose_demo_target, expected_joint_names


class SmallMotionPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("piper_small_motion_publisher")
        self.joint_names = expected_joint_names(side)
        self.current: tuple[float, ...] | None = None
        self.joint1_min: float | None = None
        self.joint1_max: float | None = None
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            JointTrajectory, f"/execution/{side}_arm/joint_reference", qos
        )
        self.subscription = self.create_subscription(JointState, "/joint_states", self._state, state_qos)

    def _state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.joint_names):
            self.current = tuple(float(positions[name]) for name in self.joint_names)
            joint1 = self.current[0]
            self.joint1_min = joint1 if self.joint1_min is None else min(self.joint1_min, joint1)
            self.joint1_max = joint1 if self.joint1_max is None else max(self.joint1_max, joint1)

    def send(self, positions: tuple[float, ...]) -> None:
        message = JointTrajectory()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = list(self.joint_names)
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=0, nanosec=500_000_000)
        message.points = [point]
        self.publisher.publish(message)


def wait_until(predicate, node: Node, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise TimeoutError(f"timed out waiting for {description}")


def publish_for(node: SmallMotionPublisher, positions: tuple[float, ...], duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.send(positions)
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--delta",
        type=float,
        default=0.03,
        help="joint-1 out-and-back displacement in radians (default: 0.03)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = SmallMotionPublisher(args.side)
    try:
        wait_until(lambda: node.publisher.get_subscription_count() > 0, node, 10.0, "board subscriber")
        wait_until(lambda: node.current is not None, node, 10.0, "Piper joint feedback")
        initial = node.current
        assert initial is not None
        target = choose_demo_target(initial, delta_rad=args.delta)
        node.get_logger().info(f"initial={initial}; target={target}")
        publish_for(node, target, 1.5)
        publish_for(node, initial, 1.5)
        publish_for(node, initial, 0.5)
        node.get_logger().info(
            "small out-and-back motion completed; "
            f"observed_joint1_range=[{node.joint1_min}, {node.joint1_max}] rad"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
