"""Pure safety helpers shared by the board bridge and host demo publisher."""

from __future__ import annotations

import math
from collections.abc import Sequence


# Conservative limits from the current official Piper SDK.  Joint 6 is kept at
# +/-120 degrees even though some older URDFs advertise a wider range.
JOINT_LIMITS_RAD = (
    (-2.6179, 2.6179),
    (0.0, 3.14),
    (-2.967, 0.0),
    (-1.745, 1.745),
    (-1.22, 1.22),
    (-2.09439, 2.09439),
)

DEFAULT_DEMO_DELTA_RAD = 0.03
DEFAULT_REMOTE_SLEW_STEP_RAD = 0.03
MILLIDEGREES_PER_RADIAN = 1000.0 * 180.0 / math.pi
# Piper SDK 0.6.2 limits the full gripper opening to 0.07 m.  The ROS joint
# represents one finger, which travels half of the full opening width.
GRIPPER_MAX_FINGER_POSITION_M = 0.035
GRIPPER_RAW_UNITS_PER_METER = 1_000_000.0


class CommandRejected(ValueError):
    """Raised when a command fails a deterministic safety check."""


def expected_joint_names(side: str) -> tuple[str, ...]:
    if side not in ("left", "right"):
        raise ValueError("side must be 'left' or 'right'")
    return tuple(f"{side}_joint{i}" for i in range(1, 7))


def validate_target(
    target: Sequence[float],
    current: Sequence[float],
) -> tuple[float, ...]:
    """Validate a six-joint absolute target and current feedback.

    Absolute targets are not rejected based on their distance from feedback.
    Hard joint limits remain enforced here; feedback freshness, drive faults and
    the command watchdog are enforced by the bridge.
    """

    if len(target) != 6 or len(current) != 6:
        raise CommandRejected("target and current feedback must each contain 6 joints")
    checked = tuple(float(value) for value in target)
    feedback = tuple(float(value) for value in current)
    if not all(math.isfinite(value) for value in checked + feedback):
        raise CommandRejected("joint values must be finite")

    for index, (value, (lower, upper)) in enumerate(zip(checked, JOINT_LIMITS_RAD), 1):
        if value < lower or value > upper:
            raise CommandRejected(
                f"joint{index} target {value:.6f} rad is outside [{lower:.6f}, {upper:.6f}]"
            )

    return checked


def choose_demo_target(
    current: Sequence[float],
    *,
    joint_index: int = 0,
    delta_rad: float = DEFAULT_DEMO_DELTA_RAD,
    limit_margin_rad: float = 0.10,
) -> tuple[float, ...]:
    """Choose a small move in the direction with more limit headroom."""

    if len(current) != 6 or not 0 <= joint_index < 6:
        raise CommandRejected("valid six-joint feedback and joint_index are required")
    if not math.isfinite(delta_rad) or delta_rad <= 0.0:
        raise CommandRejected("demo delta must be finite and positive")

    feedback = tuple(float(value) for value in current)
    if not all(math.isfinite(value) for value in feedback):
        raise CommandRejected("joint feedback must be finite")

    lower, upper = JOINT_LIMITS_RAD[joint_index]
    positive_headroom = upper - limit_margin_rad - feedback[joint_index]
    negative_headroom = feedback[joint_index] - (lower + limit_margin_rad)
    if max(positive_headroom, negative_headroom) < delta_rad:
        raise CommandRejected(f"joint{joint_index + 1} has insufficient limit headroom")

    direction = 1.0 if positive_headroom >= negative_headroom else -1.0
    target = list(feedback)
    target[joint_index] += direction * delta_rad
    return validate_target(target, feedback)


def offset_slew_target(
    follower_anchor: Sequence[float],
    leader_anchor: Sequence[float],
    leader_current: Sequence[float],
    follower_current: Sequence[float],
    *,
    max_step_rad: float = DEFAULT_REMOTE_SLEW_STEP_RAD,
    limit_margin_rad: float = 0.01,
) -> tuple[float, ...]:
    """Map Leader displacement onto a follower anchor and slew from feedback.

    Absolute Leader and follower poses do not have to match at takeover.  The
    first Leader sample is zero displacement, so takeover cannot jump the
    follower.  Every later output is bounded relative to current remote
    feedback and clamped inside the conservative Piper joint limits.
    """

    vectors = (follower_anchor, leader_anchor, leader_current, follower_current)
    if any(len(values) != 6 for values in vectors):
        raise CommandRejected("all leader/follower vectors must contain 6 joints")
    if not math.isfinite(max_step_rad) or max_step_rad <= 0.0:
        raise CommandRejected("max_step_rad must be finite and positive")
    if not math.isfinite(limit_margin_rad) or limit_margin_rad < 0.0:
        raise CommandRejected("limit_margin_rad must be finite and non-negative")

    converted = tuple(tuple(float(value) for value in values) for values in vectors)
    if not all(math.isfinite(value) for values in converted for value in values):
        raise CommandRejected("leader/follower joint values must be finite")
    follower_zero, leader_zero, leader_now, follower_now = converted

    target: list[float] = []
    for index, (follower_start, leader_start, leader_value, feedback) in enumerate(
        zip(follower_zero, leader_zero, leader_now, follower_now)
    ):
        lower, upper = JOINT_LIMITS_RAD[index]
        lower += limit_margin_rad
        upper -= limit_margin_rad
        if lower > upper:
            raise CommandRejected(f"joint{index + 1} limit margin is too large")
        desired = follower_start + (leader_value - leader_start)
        desired = min(upper, max(lower, desired))
        delta = min(max_step_rad, max(-max_step_rad, desired - feedback))
        target.append(feedback + delta)

    return validate_target(target, follower_now)


def radians_to_millidegrees(values: Sequence[float]) -> tuple[int, ...]:
    if len(values) != 6 or not all(math.isfinite(float(value)) for value in values):
        raise CommandRejected("six finite joint values are required")
    return tuple(round(float(value) * MILLIDEGREES_PER_RADIAN) for value in values)


def millidegrees_to_radians(values: Sequence[int]) -> tuple[float, ...]:
    if len(values) != 6:
        raise CommandRejected("six joint values are required")
    return tuple(float(value) / MILLIDEGREES_PER_RADIAN for value in values)


def clamp_gripper_finger_position(value: float) -> float:
    """Clamp one-finger position in metres to the board SDK's gripper range."""

    converted = float(value)
    if not math.isfinite(converted):
        raise CommandRejected("gripper position must be finite")
    return min(GRIPPER_MAX_FINGER_POSITION_M, max(0.0, converted))


def gripper_finger_position_to_raw_width(value: float) -> int:
    """Convert one-finger metres to SDK full-width units of 0.001 mm."""

    finger_position = clamp_gripper_finger_position(value)
    return round(finger_position * 2.0 * GRIPPER_RAW_UNITS_PER_METER)


def gripper_raw_width_to_finger_position(value: int) -> float:
    """Convert SDK full-width units of 0.001 mm to one-finger metres."""

    converted = float(value) / (2.0 * GRIPPER_RAW_UNITS_PER_METER)
    return clamp_gripper_finger_position(converted)
