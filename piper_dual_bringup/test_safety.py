import math
import unittest

from safety import CommandRejected, choose_demo_target, expected_joint_names
from safety import millidegrees_to_radians, offset_slew_target
from safety import radians_to_millidegrees, validate_target


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.current = (0.0, 1.0, -1.0, 0.0, 0.0, 0.0)

    def test_expected_joint_names(self):
        self.assertEqual(expected_joint_names("left")[0], "left_joint1")
        self.assertEqual(expected_joint_names("right")[-1], "right_joint6")

    def test_small_target_is_accepted(self):
        target = list(self.current)
        target[0] += 0.03
        self.assertEqual(validate_target(target, self.current), tuple(target))

    def test_large_in_limit_step_is_accepted(self):
        target = list(self.current)
        target[0] += 0.50
        self.assertEqual(validate_target(target, self.current), tuple(target))

    def test_limit_and_nonfinite_are_rejected(self):
        for invalid in (3.0, math.nan, math.inf):
            target = list(self.current)
            target[0] = invalid
            with self.assertRaises(CommandRejected):
                validate_target(target, self.current)

    def test_demo_target_is_small_and_safe(self):
        target = choose_demo_target(self.current)
        self.assertAlmostEqual(abs(target[0] - self.current[0]), 0.03)
        validate_target(target, self.current)

    def test_round_trip_units(self):
        raw = radians_to_millidegrees(self.current)
        restored = millidegrees_to_radians(raw)
        for expected, actual in zip(self.current, restored):
            self.assertAlmostEqual(expected, actual, places=5)

    def test_offset_takeover_has_no_initial_jump(self):
        leader = (0.4, 0.2, -0.3, 0.8, -0.4, 0.5)
        self.assertEqual(
            offset_slew_target(self.current, leader, leader, self.current),
            self.current,
        )

    def test_offset_motion_is_slewed_from_remote_feedback(self):
        leader_anchor = (0.0,) * 6
        leader_current = (0.5, 0.0, 0.0, 0.0, 0.0, 0.0)
        target = offset_slew_target(
            self.current, leader_anchor, leader_current, self.current
        )
        self.assertAlmostEqual(target[0], self.current[0] + 0.03)
        validate_target(target, self.current)

    def test_offset_target_clamps_joint_limits(self):
        follower = (2.60, 1.0, -1.0, 0.0, 0.0, 0.0)
        target = offset_slew_target(
            follower, (0.0,) * 6, (1.0, 0.0, 0.0, 0.0, 0.0, 0.0), follower
        )
        self.assertLessEqual(target[0], 2.6179 - 0.01)

    def test_offset_slew_can_be_configured_above_old_guard(self):
        target = offset_slew_target(
            self.current,
            (0.0,) * 6,
            (0.5, 0.0, 0.0, 0.0, 0.0, 0.0),
            self.current,
            max_step_rad=0.20,
        )
        self.assertAlmostEqual(target[0], self.current[0] + 0.20)


if __name__ == "__main__":
    unittest.main()
