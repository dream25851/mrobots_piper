#!/usr/bin/env python3
"""Read-only SocketCAN health check for a logical Piper interface."""

from __future__ import annotations

import argparse
from collections import Counter
import socket
import struct
import time


ALLOWED_CAN_INTERFACES = ("piper0", "piper1")
CAN_FRAME = struct.Struct("=IB3x8s")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument(
        "--can-interface", choices=ALLOWED_CAN_INTERFACES, default="piper1"
    )
    args = parser.parse_args()

    counts: Counter[int] = Counter()
    samples: dict[int, bytes] = {}
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.settimeout(0.25)
    sock.bind((args.can_interface,))
    deadline = time.monotonic() + max(0.1, args.seconds)
    try:
        while time.monotonic() < deadline:
            try:
                raw = sock.recv(CAN_FRAME.size)
            except TimeoutError:
                continue
            can_id, data_len, payload = CAN_FRAME.unpack(raw)
            can_id &= socket.CAN_EFF_MASK
            counts[can_id] += 1
            samples.setdefault(can_id, payload[:data_len])
    finally:
        sock.close()

    print(f"interface={args.can_interface} frames={sum(counts.values())}")
    for can_id in sorted(counts):
        print(
            f"id=0x{can_id:03X} count={counts[can_id]} "
            f"sample={samples[can_id].hex(' ')}"
        )
    return 0 if counts else 2


if __name__ == "__main__":
    raise SystemExit(main())
