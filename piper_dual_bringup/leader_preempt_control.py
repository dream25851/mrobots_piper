#!/usr/bin/env python3
"""Cyclone DDS service-plane controller for both workstation Piper Leaders."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


SIDES = ("left", "right")


class LeaderPreemptControl(Node):
    def __init__(self) -> None:
        super().__init__("piper_leader_preempt_control")
        self._preempt_clients = {
            side: self.create_client(SetBool, f"/piper_leader_{side}/preempt")
            for side in SIDES
        }

    def set_side(self, side: str, enabled: bool, timeout_s: float = 8.0) -> str:
        client = self._preempt_clients[side]
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(f"{side} Leader preempt service unavailable")
        future = client.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if not future.done() or future.result() is None:
            raise TimeoutError(f"{side} Leader preempt service timed out")
        response = future.result()
        if not response.success:
            raise RuntimeError(f"{side} Leader rejected preempt: {response.message}")
        return response.message


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--state", choices=("true", "false"), required=True)
    parser.add_argument(
        "--attempts",
        type=int,
        default=2,
        help="bounded attempts per side when enabling; disabling always tries once",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    enabled = args.state == "true"
    if args.attempts < 1 or args.attempts > 3:
        raise ValueError("--attempts must be in [1, 3]")
    rclpy.init()
    node = LeaderPreemptControl()
    failures: list[str] = []
    try:
        for side in SIDES:
            attempts = args.attempts if enabled else 1
            for attempt in range(1, attempts + 1):
                try:
                    message = node.set_side(side, enabled)
                    node.get_logger().info(f"{side}: {message}")
                    break
                except Exception as error:
                    if attempt == attempts:
                        failures.append(str(error))
                    else:
                        node.get_logger().warning(
                            f"{side}: preempt attempt {attempt} failed: {error}; retrying"
                        )
                        time.sleep(0.5)
            if enabled and failures:
                # Do not engage the other Leader after one side failed.
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if failures:
        for failure in failures:
            print(f"ERROR: {failure}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
