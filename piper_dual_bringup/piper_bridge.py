#!/usr/bin/env python3
"""Conservative ROS 2 to Piper SDK bridge for one logical Piper interface."""

from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from safety import CommandRejected, expected_joint_names, millidegrees_to_radians
from safety import radians_to_millidegrees, validate_target

from piper_sdk import C_PiperInterface_V2, LogLevel


ALLOWED_CAN_INTERFACES = ("piper0", "piper1")
FEEDBACK_TIMEOUT_S = 0.20
REFERENCE_TIMEOUT_S = 0.30
CONTROL_PERIOD_S = 0.005


class PiperBridge(Node):
    def __init__(
        self, *, side: str, can_interface: str, actuate: bool, speed_percent: int
    ) -> None:
        super().__init__(f"{can_interface}_bridge")
        self._side = side
        self._can_interface = can_interface
        self._actuate = actuate
        self._speed_percent = speed_percent
        self._joint_names = expected_joint_names(side)
        self._current: tuple[float, ...] | None = None
        self._target: tuple[float, ...] | None = None
        self._last_feedback_monotonic: float | None = None
        self._last_feedback_sdk_stamp: float | None = None
        self._last_status_sdk_stamp: float | None = None
        self._last_reference_monotonic: float | None = None
        self._last_enable_attempt = 0.0
        self._last_status_publish = 0.0
        self._last_log: dict[str, float] = {}
        self._ready = False
        self._fault = ""

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
        topic = f"/execution/{side}_arm/joint_reference"
        self._reference_sub = self.create_subscription(
            JointTrajectory, topic, self._on_reference, command_qos
        )
        self._joint_state_pub = self.create_publisher(JointState, "/joint_states", state_qos)
        self._status_pub = self.create_publisher(
            String, f"/{can_interface}/status", command_qos
        )

        self.get_logger().info(
            f"opening SocketCAN interface {can_interface}; topic={topic}; actuate={actuate}"
        )
        self._piper = C_PiperInterface_V2(
            can_name=can_interface,
            judge_flag=False,
            can_auto_init=True,
            start_sdk_joint_limit=True,
            start_sdk_gripper_limit=True,
            logger_level=LogLevel.WARNING,
        )
        # Read-only mode must remain truly passive. PiperInit() sends parameter
        # query frames, so only run it after actuation was explicitly enabled.
        self._piper.ConnectPort(piper_init=actuate)
        self._timer = self.create_timer(CONTROL_PERIOD_S, self._tick)

    def _log_throttled(self, level: str, key: str, message: str, period_s: float = 2.0) -> None:
        now = time.monotonic()
        if now - self._last_log.get(key, 0.0) < period_s:
            return
        self._last_log[key] = now
        # This board's rclpy/rcutils rejects changing severity at one Python
        # callsite. Keep each severity on a distinct source line.
        if level == "error":
            self.get_logger().error(message)
        elif level == "warning":
            self.get_logger().warning(message)
        elif level == "info":
            self.get_logger().info(message)
        else:
            raise ValueError(f"unsupported log level: {level}")

    def _read_feedback(self, now: float) -> None:
        feedback = self._piper.GetArmJointMsgs()
        sdk_stamp = float(feedback.time_stamp)
        if sdk_stamp <= 0.0 or sdk_stamp == self._last_feedback_sdk_stamp:
            return
        self._last_feedback_sdk_stamp = sdk_stamp
        raw = feedback.joint_state
        current = millidegrees_to_radians(
            (raw.joint_1, raw.joint_2, raw.joint_3, raw.joint_4, raw.joint_5, raw.joint_6)
        )
        self._current = current
        self._last_feedback_monotonic = now

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._joint_names)
        message.position = list(current)
        self._joint_state_pub.publish(message)

    def _feedback_fresh(self, now: float) -> bool:
        return (
            self._current is not None
            and self._last_feedback_monotonic is not None
            and now - self._last_feedback_monotonic <= FEEDBACK_TIMEOUT_S
        )

    def _check_fault(self) -> None:
        wrapped = self._piper.GetArmStatus()
        sdk_stamp = float(wrapped.time_stamp)
        if sdk_stamp <= 0.0 or sdk_stamp == self._last_status_sdk_stamp:
            return
        self._last_status_sdk_stamp = sdk_stamp
        status = wrapped.arm_status
        if status.err_code != 0 or int(status.arm_status) not in (0x00,):
            self._fault = f"arm_status={int(status.arm_status):#04x}, err_code={status.err_code:#06x}"
            self._ready = False

    def _ensure_enabled(self, now: float) -> None:
        if not self._actuate or self._ready or not self._feedback_fresh(now) or self._fault:
            return
        if now - self._last_enable_attempt < 0.10:
            return
        self._last_enable_attempt = now
        enabled_before_request = self._piper.EnablePiper()
        if enabled_before_request or all(self._piper.GetArmEnableStatus()):
            self._target = self._current
            self._ready = True
            self.get_logger().info("Piper enabled; command buffer synchronized to feedback")

    def _on_reference(self, message: JointTrajectory) -> None:
        now = time.monotonic()
        if not self._actuate or not self._ready:
            self._log_throttled("warning", "not_ready", "reference rejected: bridge is not actuation-ready")
            return
        if not self._feedback_fresh(now):
            self._ready = False
            self._log_throttled("error", "feedback_stale", "reference rejected: Piper feedback is stale")
            return
        if tuple(message.joint_names) != self._joint_names:
            self._log_throttled(
                "warning",
                "joint_names",
                f"reference rejected: expected joint_names={list(self._joint_names)}",
            )
            return
        if len(message.points) != 1:
            self._log_throttled("warning", "point_count", "reference rejected: exactly one point is required")
            return
        try:
            self._target = validate_target(message.points[0].positions, self._current)
        except CommandRejected as exc:
            self._log_throttled("warning", "unsafe_reference", f"reference rejected: {exc}")
            return
        self._last_reference_monotonic = now

    def _write_command(self, now: float) -> None:
        if not self._actuate or not self._ready or not self._feedback_fresh(now):
            return
        if self._target is None:
            self._target = self._current
        if (
            self._last_reference_monotonic is not None
            and now - self._last_reference_monotonic > REFERENCE_TIMEOUT_S
        ):
            self._target = self._current
            self._last_reference_monotonic = None
            self._log_throttled("warning", "watchdog", "command watchdog expired; holding feedback pose")

        raw = radians_to_millidegrees(self._target)
        self._piper.MotionCtrl_2(0x01, 0x01, self._speed_percent, 0x00)
        self._piper.JointCtrl(*raw)

    def _publish_status(self, now: float) -> None:
        if now - self._last_status_publish < 0.20:
            return
        self._last_status_publish = now
        feedback_age = None
        if self._last_feedback_monotonic is not None:
            feedback_age = now - self._last_feedback_monotonic
        message = String()
        message.data = json.dumps(
            {
                "can_interface": self._can_interface,
                "side": self._side,
                "actuate": self._actuate,
                "ready": self._ready,
                "fault": self._fault,
                "feedback_age_s": feedback_age,
                "positions_rad": self._current,
            },
            ensure_ascii=False,
        )
        self._status_pub.publish(message)

    def _tick(self) -> None:
        now = time.monotonic()
        try:
            self._read_feedback(now)
            self._check_fault()
            self._ensure_enabled(now)
            self._write_command(now)
            self._publish_status(now)
        except Exception as exc:  # keep the process alive long enough to report and stop writes
            self._fault = f"runtime exception: {exc}"
            self._ready = False
            self._log_throttled("error", "runtime", self._fault, period_s=0.5)

    def close(self) -> None:
        self._ready = False
        try:
            if self._actuate and self._current is not None:
                self._piper.MotionCtrl_2(0x01, 0x01, self._speed_percent, 0x00)
                self._piper.JointCtrl(*radians_to_millidegrees(self._current))
        finally:
            # Deliberately do not disable motors here: disabling can let the arm fall.
            self._piper.DisconnectPort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--can-interface", choices=ALLOWED_CAN_INTERFACES, default="piper1"
    )
    parser.add_argument("--actuate", action="store_true", help="enable the arm and accept references")
    parser.add_argument("--speed-percent", type=int, default=10)
    args = parser.parse_args()
    if not 1 <= args.speed_percent <= 20:
        parser.error("--speed-percent must be in [1, 20] for this bring-up tool")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = PiperBridge(
        side=args.side,
        can_interface=args.can_interface,
        actuate=args.actuate,
        speed_percent=args.speed_percent,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
