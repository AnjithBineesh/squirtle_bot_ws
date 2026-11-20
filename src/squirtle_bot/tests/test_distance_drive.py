import unittest
from unittest.mock import patch

from squirtle_bot.tests.odometry_line_test import (
    OdometryLineTestController,
    distance_target_reached,
)


class FakeEncoder:
    def reset(self):
        pass

    def get_ticks(self):
        return 0


class FakeOdometry:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def update(self, left_ticks, right_ticks):
        return 0.0, 0.0, 0.0

    def wheel_distances_from_ticks(self, left_ticks, right_ticks):
        return 0.0, 0.0


class FakeDriver:
    def __init__(self):
        self.stop_calls = 0

    def set_motors(self, left_speed, right_speed):
        pass

    def stop(self):
        self.stop_calls += 1


class DistanceDriveTest(unittest.TestCase):
    def test_brake_margin_triggers_before_target(self):
        self.assertFalse(distance_target_reached(0.979, 0.979, 0.979, 1.0))
        self.assertTrue(distance_target_reached(0.98, 0.98, 0.98, 1.0))

    def test_leading_wheel_hard_limit_stops_run(self):
        self.assertTrue(distance_target_reached(0.975, 0.97, 0.98, 1.0))

    def test_center_distance_stops_even_if_wheels_differ(self):
        self.assertTrue(distance_target_reached(0.98, 0.97, 0.99, 1.0))

    def test_small_target_does_not_trigger_at_zero(self):
        self.assertFalse(distance_target_reached(0.0, 0.0, 0.0, 0.1))

    def test_distance_timeout_stops_without_timer_name_error(self):
        controller = object.__new__(OdometryLineTestController)
        controller.speed = 30
        controller.left_encoder = FakeEncoder()
        controller.right_encoder = FakeEncoder()
        controller.odometry = FakeOdometry()
        controller.driver = FakeDriver()
        controller._sync_wheel_speeds = lambda now: None

        with patch(
            "squirtle_bot.tests.odometry_line_test.time.monotonic",
            side_effect=[1.0, 17.0],
        ):
            controller.drive_distance(1.0)

        self.assertEqual(controller.driver.stop_calls, 1)

    def test_move_to_waypoint_already_at_target(self):
        controller = object.__new__(OdometryLineTestController)
        controller.update_pose = lambda: (1.0, 1.0, 0.0)
        self.assertTrue(controller.move_to_waypoint(1.0, 1.0, tolerance_m=0.05))

    def test_move_to_waypoint_steps_toward_target(self):
        controller = object.__new__(OdometryLineTestController)
        poses = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        controller.update_pose = lambda: poses.pop(0) if poses else (1.0, 0.0, 0.0)
        controller.turn_angle = lambda angle, direction, reset_state=False: True
        drives = []
        controller.drive_distance = lambda dist, reset_state=False: drives.append(dist) or True

        result = controller.move_to_waypoint(1.0, 0.0, tolerance_m=0.05)
        self.assertTrue(result)
        self.assertEqual(drives, [1.0])

    def test_controller_cleanup_calls_all_subsystems(self):
        controller = object.__new__(OdometryLineTestController)
        cleaned = []
        controller.cleanup_imu = lambda: cleaned.append("imu")
        controller.cleanup_encoders = lambda: cleaned.append("encoders")
        controller.driver = type("D", (), {"cleanup": lambda self: cleaned.append("driver")})()

        controller.cleanup()
        self.assertEqual(cleaned, ["imu", "encoders", "driver"])


if __name__ == "__main__":
    unittest.main()