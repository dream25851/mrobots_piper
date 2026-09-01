#!/usr/bin/env python3
"""Publish one bounded gripper open/close motion and verify remote feedback."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from safety import GRIPPER_MAX_FINGER_POSITION_M, clamp_gripper_finger_position


class GripperMotionPublisher(Node):
    def __init__(self, side: str) -> None:
        super().__init__("piper_gripper_motion_publisher")
        self.joint_name = f"{side}_gripper_joint1"
        self.position: float | None = None
        self.minimum: float | None = None
        self.maximum: float | None = None
        command_qos = QoSProfile(
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
            Float64MultiArray,
            f"/execution/{side}_gripper/joint_reference",
            command_qos,
        )
        self.subscription = self.create_subscription(
            JointState, "/joint_states", self._on_state, state_qos
        )

    def _on_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if self.joint_name not in positions:
            return
        self.position = float(positions[self.joint_name])
        self.minimum = self.position if self.minimum is None else min(self.minimum, self.position)
        self.maximum = self.position if self.maximum is None else max(self.maximum, self.position)

    def publish(self, position: float) -> None:
        self.publisher.publish(Float64MultiArray(data=[position]))


def wait_until(predicate, node: Node, timeout_s: float, description: str) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return
    raise TimeoutError(f"timed out waiting for {description}")


def publish_for(node: GripperMotionPublisher, position: float, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish(position)
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--delta", type=float, default=0.01)
    args = parser.parse_args()
    if not 0.0 < args.delta <= GRIPPER_MAX_FINGER_POSITION_M:
        parser.error(f"--delta must be in (0, {GRIPPER_MAX_FINGER_POSITION_M}]")

    rclpy.init()
    node = GripperMotionPublisher(args.side)
    try:
        wait_until(lambda: node.publisher.get_subscription_count() > 0, node, 10.0, "bridge subscriber")
        wait_until(lambda: node.position is not None, node, 10.0, "gripper feedback")
        initial = node.position
        assert initial is not None
        if initial + args.delta <= GRIPPER_MAX_FINGER_POSITION_M:
            target = initial + args.delta
        else:
            target = initial - args.delta
        target = clamp_gripper_finger_position(target)
        node.get_logger().info(f"initial={initial:.6f} m; target={target:.6f} m")
        publish_for(node, target, 2.0)
        publish_for(node, initial, 2.0)
        wait_until(
            lambda: node.position is not None and abs(node.position - initial) < 0.002,
            node,
            3.0,
            "gripper return",
        )
        observed = (node.maximum or 0.0) - (node.minimum or 0.0)
        if observed < min(args.delta * 0.5, 0.003):
            raise RuntimeError(f"gripper feedback moved only {observed:.6f} m")
        node.get_logger().info(
            f"gripper motion completed; observed_range=[{node.minimum}, {node.maximum}] m"
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
