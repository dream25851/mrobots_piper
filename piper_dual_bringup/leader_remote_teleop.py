#!/usr/bin/env python3
"""Zero-offset workstation Leader relay for the RK3588S Piper bridges.

This is the Fast DDS data-plane process used before the board has a native
controller_manager.  ``host_start_leader_remote.sh`` owns the Cyclone DDS
Leader service plane and starts this relay.  Grippers are intentionally out of
scope.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from safety import CommandRejected, expected_joint_names, offset_slew_target


SIDES = ("left", "right")
SOURCE_TIMEOUT_S = 0.15
FEEDBACK_TIMEOUT_S = 0.15


@dataclass
class SideState:
    side: str
    feedback: tuple[float, ...] | None = None
    feedback_time: float | None = None
    board_ready: bool = False
    leader_anchor: tuple[float, ...] | None = None
    follower_anchor: tuple[float, ...] | None = None
    source_time: float | None = None
    relayed_messages: int = 0
    minimum: list[float] = field(default_factory=lambda: [float("inf")] * 6)
    maximum: list[float] = field(default_factory=lambda: [float("-inf")] * 6)


class LeaderRemoteTeleop(Node):
    def __init__(self, selected_sides: tuple[str, ...]) -> None:
        super().__init__("piper_remote_leader_teleop")
        self.selected_sides = selected_sides
        self.states = {side: SideState(side) for side in selected_sides}
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
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._command_publishers = {}
        self._kept_subscriptions = []
        self._kept_subscriptions.append(
            self.create_subscription(JointState, "/joint_states", self._on_feedback, state_qos)
        )
        for side in selected_sides:
            self._command_publishers[side] = self.create_publisher(
                JointTrajectory, f"/execution/{side}_arm/joint_reference", command_qos
            )
            self._kept_subscriptions.append(
                self.create_subscription(
                    JointTrajectory,
                    f"/action_sources/piper_leader_{side}/arm/joint_reference",
                    lambda message, selected=side: self._on_leader(selected, message),
                    command_qos,
                )
            )
            can_interface = "piper1" if side == "left" else "piper0"
            self._kept_subscriptions.append(
                self.create_subscription(
                    String,
                    f"/{can_interface}/status",
                    lambda message, selected=side: self._on_status(selected, message),
                    status_qos,
                )
            )

    def _on_feedback(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        now = time.monotonic()
        for side, state in self.states.items():
            names = expected_joint_names(side)
            if all(name in positions for name in names):
                state.feedback = tuple(float(positions[name]) for name in names)
                state.feedback_time = now
                for index, value in enumerate(state.feedback):
                    state.minimum[index] = min(state.minimum[index], value)
                    state.maximum[index] = max(state.maximum[index], value)

    def _on_status(self, side: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            self.states[side].board_ready = False
            return
        self.states[side].board_ready = bool(payload.get("ready")) and not payload.get("fault")

    def _on_leader(self, side: str, message: JointTrajectory) -> None:
        state = self.states[side]
        now = time.monotonic()
        if (
            state.feedback is None
            or state.feedback_time is None
            or now - state.feedback_time > FEEDBACK_TIMEOUT_S
            or not state.board_ready
            or len(message.points) != 1
        ):
            return
        names = expected_joint_names(side)
        values = dict(zip(message.joint_names, message.points[0].positions))
        if not all(name in values for name in names):
            return
        leader = tuple(float(values[name]) for name in names)
        if state.leader_anchor is None:
            state.leader_anchor = leader
            state.follower_anchor = state.feedback
            self.get_logger().info(
                f"{side}: zero-offset anchor captured; follower={state.follower_anchor}; "
                f"leader={state.leader_anchor}"
            )
        assert state.follower_anchor is not None
        try:
            target = offset_slew_target(
                state.follower_anchor,
                state.leader_anchor,
                leader,
                state.feedback,
            )
        except CommandRejected as error:
            self.get_logger().warning(f"{side}: reference rejected: {error}")
            return

        output = JointTrajectory()
        output.header.stamp = self.get_clock().now().to_msg()
        output.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = list(target)
        output.points = [point]
        self._command_publishers[side].publish(output)
        state.source_time = now
        state.relayed_messages += 1

    def wait_until_ready(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            feedback_ready = all(state.feedback is not None for state in self.states.values())
            boards_ready = all(state.board_ready for state in self.states.values())
            subscribers_ready = all(
                publisher.get_subscription_count() > 0
                for publisher in self._command_publishers.values()
            )
            if feedback_ready and boards_ready and subscribers_ready:
                return
        details = {
            side: {
                "feedback": state.feedback is not None,
                "board_ready": state.board_ready,
                "board_subscribers": self._command_publishers[side].get_subscription_count(),
            }
            for side, state in self.states.items()
        }
        raise TimeoutError(f"remote teleop prerequisites missing: {details}")

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sides", choices=("left", "right", "both"), default="both")
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="seconds before automatic release; zero runs until Ctrl+C",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    selected = SIDES if args.sides == "both" else (args.sides,)
    rclpy.init()
    node = LeaderRemoteTeleop(selected)
    stopping = False

    def request_stop(*_args) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node.wait_until_ready(10.0)
        node.get_logger().info(
            f"ready: sides={selected}; waiting for Leader preempt source messages"
        )
        start = time.monotonic()
        while not stopping and (args.duration <= 0.0 or time.monotonic() - start < args.duration):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for side, state in node.states.items():
            node.get_logger().info(
                f"{side}: relayed={state.relayed_messages}; "
                f"follower_min={state.minimum}; follower_max={state.maximum}"
            )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
