import math
import unittest

from squirtle_bot.robot.odometry import EncoderOdometry


class OdometryCalibrationTest(unittest.TestCase):
    def test_separate_wheel_resolutions_produce_equal_revolution_distance(self):
        odometry = EncoderOdometry(
            wheel_base=0.0,
            wheel_diameter=0.06,
            left_ticks_per_rev=149,
            right_ticks_per_rev=152,
            distance_scale=1.0,
        )

        x, y, theta = odometry.update(left_ticks=149, right_ticks=152)

        self.assertAlmostEqual(x, math.pi * 0.06)
        self.assertEqual(y, 0.0)
        self.assertEqual(theta, 0.0)

    def test_ground_truth_scale_corrects_recorded_one_meter_trial(self):
        odometry = EncoderOdometry(wheel_base=0.0)

        x, y, theta = odometry.update(left_ticks=649, right_ticks=666)

        self.assertAlmostEqual(x, 1.0, places=2)
        self.assertEqual(y, 0.0)
        self.assertEqual(theta, 0.0)

    def test_effective_wheelbase_matches_recorded_full_turn(self):
        odometry = EncoderOdometry()

        _, _, theta = odometry.update(left_ticks=765, right_ticks=-417)

        self.assertAlmostEqual(math.degrees(theta), -360.0, delta=5.0)


if __name__ == "__main__":
    unittest.main()
