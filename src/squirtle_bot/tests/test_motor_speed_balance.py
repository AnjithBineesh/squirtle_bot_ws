import unittest

from squirtle_bot.tests.odometry_line_test import (
    OdometryLineTestController,
    balanced_motor_speeds,
)


class FakeEncoder:
    def __init__(self, ticks):
        self.ticks = ticks

    def get_ticks(self):
        return self.ticks


class FakeDriver:
    def __init__(self):
        self.commands = []

    def set_motors(self, left_speed, right_speed):
        self.commands.append((left_speed, right_speed))


class MotorSpeedBalanceTest(unittest.TestCase):
    def test_faster_left_wheel_is_trimmed(self):
        left, right = balanced_motor_speeds(30, left_rate=0.5, right_rate=0.4)

        self.assertEqual(left, 27)
        self.assertEqual(right, 33)

    def test_faster_right_wheel_is_trimmed(self):
        left, right = balanced_motor_speeds(30, left_rate=0.4, right_rate=0.5)

        self.assertEqual(left, 33)
        self.assertEqual(right, 27)

    def test_correction_is_capped(self):
        left, right = balanced_motor_speeds(30, left_rate=10.0, right_rate=0.0)

        self.assertEqual(left, 22)
        self.assertEqual(right, 38)

    def test_wheel_behind_in_rotation_gets_more_power(self):
        left, right = balanced_motor_speeds(
            30,
            left_rate=0.4,
            right_rate=0.4,
            left_progress=0.1,
            right_progress=0.0,
        )

        self.assertEqual(left, 25)
        self.assertEqual(right, 35)

    def test_physical_left_wheel_trim_keeps_fractional_pwm(self):
        left, right = balanced_motor_speeds(
            30,
            left_rate=0.5,
            right_rate=0.5,
            left_speed_trim_percent=2.0,
        )

        self.assertAlmostEqual(left, 30.6)
        self.assertEqual(right, 30)

    def test_live_sync_uses_per_wheel_encoder_resolutions(self):
        controller = object.__new__(OdometryLineTestController)
        controller.speed = 30
        controller._straight_direction = 1
        controller._last_sync_time = 1.0
        controller._last_sync_ticks = (0, 0)
        controller._straight_start_ticks = (0, 0)
        controller.left_encoder = FakeEncoder(10)
        controller.right_encoder = FakeEncoder(8)
        controller.driver = FakeDriver()

        controller._sync_wheel_speeds(1.3)

        self.assertEqual(len(controller.driver.commands), 1)
        left_speed, right_speed = controller.driver.commands[0]
        self.assertLess(left_speed, right_speed)


if __name__ == "__main__":
    unittest.main()