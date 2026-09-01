#!/usr/bin/env python3
"""Minimal controller-manager contract for the transitional Piper bridge.

This is not ros2_control.  It lets the workstation Execution Manager perform
its normal lease/switch transaction while the RK3588S still runs the Python
CAN bridges.  A controller can only be activated when the corresponding board
bridge reports fresh, actuation-ready, fault-free status.
"""

from __future__ import annotations

import json
import os
import time

import rclpy
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import ListControllers, SwitchController
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


STATUS_TIMEOUT_S = 1.0
CONTROLLER_TYPES = {
    "joint_state_broadcaster": "joint_state_broadcaster/JointStateBroadcaster",
    "left_arm_jspc": "compat/JointSpacePositionController",
    "left_arm_jtc": "compat/JointTrajectoryController",
    "left_arm_tskpc": "compat/TaskSpaceKinematicPositionController",
    "left_gripper_fwd": "compat/ForwardCommandController",
    "right_arm_jspc": "compat/JointSpacePositionController",
    "right_arm_jtc": "compat/JointTrajectoryController",
    "right_arm_tskpc": "compat/TaskSpaceKinematicPositionController",
    "right_gripper_fwd": "compat/ForwardCommandController",
}


class PiperControllerManagerCompat(Node):
    def __init__(self) -> None:
        super().__init__("piper_controller_manager_compat")
        manager = os.environ.get(
            "PIPER_CONTROLLER_MANAGER", "/rk3588_piper/controller_manager"
        ).rstrip("/")
        if not manager:
            raise ValueError("PIPER_CONTROLLER_MANAGER must not be empty")
        self._manager = manager
        self._states = {name: "inactive" for name in CONTROLLER_TYPES}
        self._states["joint_state_broadcaster"] = "active"
        self._board_status: dict[str, tuple[bool, bool, float, str]] = {}

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._status_subscriptions = [
            self.create_subscription(
                String,
                "/piper1/status",
                lambda message: self._on_status("left", message),
                status_qos,
            ),
            self.create_subscription(
                String,
                "/piper0/status",
                lambda message: self._on_status("right", message),
                status_qos,
            ),
        ]
        self._list_service = self.create_service(
            ListControllers,
            f"{self._manager}/list_controllers",
            self._list_controllers,
        )
        self._switch_service = self.create_service(
            SwitchController,
            f"{self._manager}/switch_controller",
            self._switch_controllers,
        )
        self.get_logger().warning(
            "transitional controller-manager compatibility is active; "
            f"manager={self._manager}; this is not a ros2_control controller_manager"
        )

    def _on_status(self, side: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
            ready = (
                bool(payload.get("actuate"))
                and bool(payload.get("ready"))
                and not payload.get("fault")
            )
            fault = str(payload.get("fault") or "")
            gripper_ready = bool(payload.get("gripper_ready")) and not payload.get(
                "gripper_fault"
            )
        except (TypeError, ValueError, json.JSONDecodeError):
            ready = False
            gripper_ready = False
            fault = "malformed bridge status"
        self._board_status[side] = (ready, gripper_ready, time.monotonic(), fault)

    def _side_ready(self, side: str, *, gripper: bool = False) -> tuple[bool, str]:
        status = self._board_status.get(side)
        if status is None:
            return False, f"{side} bridge status has not been received"
        arm_ready, gripper_ready, received_at, fault = status
        age = time.monotonic() - received_at
        if age > STATUS_TIMEOUT_S:
            return False, f"{side} bridge status is stale ({age:.3f}s)"
        ready = gripper_ready if gripper else arm_ready
        if not ready:
            component = "gripper" if gripper else "arm"
            return False, f"{side} {component} is not ready" + (f": {fault}" if fault else "")
        return True, ""

    def _list_controllers(self, _request, response):
        response.controller = [
            ControllerState(name=name, state=self._states[name], type=controller_type)
            for name, controller_type in CONTROLLER_TYPES.items()
        ]
        return response

    def _switch_controllers(self, request, response):
        requested = set(request.activate_controllers) | set(request.deactivate_controllers)
        unknown = sorted(requested - self._states.keys())
        if unknown and request.strictness == SwitchController.Request.STRICT:
            response.ok = False
            response.message = "unknown controllers: " + ", ".join(unknown)
            return response

        for side in ("left", "right"):
            side_activations = [
                name
                for name in request.activate_controllers
                if name.startswith(f"{side}_")
            ]
            if side_activations:
                requirements = []
                if any("gripper" not in name for name in side_activations):
                    requirements.append(False)
                if any("gripper" in name for name in side_activations):
                    requirements.append(True)
                for needs_gripper in requirements:
                    ready, reason = self._side_ready(side, gripper=needs_gripper)
                    if not ready:
                        response.ok = False
                        response.message = reason
                        return response

        for name in request.deactivate_controllers:
            if name in self._states and name != "joint_state_broadcaster":
                self._states[name] = "inactive"
        for name in request.activate_controllers:
            if name in self._states:
                self._states[name] = "active"

        response.ok = True
        response.message = "transitional Piper bridge controller state updated"
        return response


def main() -> None:
    rclpy.init()
    node = PiperControllerManagerCompat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
