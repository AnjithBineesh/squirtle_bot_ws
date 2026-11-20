import math
import unittest

from squirtle_bot.tests.odometry_line_test import (
    turn_speed_for_error,
    turn_target_reached,
    waypoint_turn_command,
    waypoint_turn_target,
)


class TurnAngleTest(unittest.TestCase):
    def test_speed_reduces_as_target_gets_close(self):
        self.assertEqual(turn_speed_for_error(30, 30), 30)
        self.assertEqual(turn_speed_for_error(20, 30), 20)
        self.assertEqual(turn_speed_for_error(5, 30), 20)

    def test_completed_turn_requests_no_motion(self):
        self.assertEqual(turn_speed_for_error(0, 30), 0)

    def test_turn_brakes_before_target_with_heading_compensation(self):
        self.assertFalse(turn_target_reached(84.2, 90.0, "L"))
        self.assertTrue(turn_target_reached(84.3, 90.0, "L"))
        self.assertFalse(turn_target_reached(83.3, 90.0, "R"))
        self.assertTrue(turn_target_reached(83.4, 90.0, "R"))

    def test_waypoint_turn_command_uses_shortest_rotation(self):
        self.assertEqual(waypoint_turn_command(0.0, 0.0, 1.0, 0.0, 0.0), ("L", 0.0))
        self.assertEqual(waypoint_turn_command(0.0, 0.0, 1.0, 0.0, 90.0), ("R", 90.0))
        self.assertEqual(waypoint_turn_command(0.0, 0.0, 0.0, 1.0, 90.0), ("L", 0.0))
        self.assertEqual(waypoint_turn_command(0.0, 0.0, 0.0, 1.0, 270.0), ("R", 180.0))

    def test_waypoint_turn_target_compensates_for_brake_margin(self):
        self.assertAlmostEqual(waypoint_turn_target(4.0), 10.7, places=1)
        self.assertAlmostEqual(waypoint_turn_target(45.0), 51.7, places=1)
        self.assertEqual(waypoint_turn_target(90.0), 90.0)

    def test_small_turns_finish_without_stuck_residual(self):
        self.assertFalse(turn_target_reached(3.0, 4.0, "L"))
        self.assertTrue(turn_target_reached(4.0, 4.0, "L"))
        self.assertFalse(turn_target_reached(3.0, 4.0, "R"))
        self.assertTrue(turn_target_reached(4.0, 4.0, "R"))


if __name__ == "__main__":
    unittest.main()