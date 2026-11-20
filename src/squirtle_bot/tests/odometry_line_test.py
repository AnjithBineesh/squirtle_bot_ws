"""Manual straight-line motor and odometry test."""

from __future__ import annotations

import logging
import math
import select
import sys
import time

from squirtle_bot.config import (
    INITIAL_TEST_SPEED,
    LEFT_ENCODER_A,
    LEFT_ENCODER_B,
    LEFT_ENCODER_INVERTED,
    LEFT_ENCODER_TICKS_PER_REV,
    LEFT_MOTOR_SPEED_TRIM_PERCENT,
    RIGHT_ENCODER_A,
    RIGHT_ENCODER_B,
    RIGHT_ENCODER_INVERTED,
    RIGHT_ENCODER_TICKS_PER_REV,
)
from squirtle_bot.hardware.encoder import QuadratureEncoder
from squirtle_bot.robot.odometry import EncoderOdometry
from squirtle_bot.sensors.imu import HeadingEstimator, MPU6050
from squirtle_bot.tests.motor_test import MotorTestController

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)
SYNC_INTERVAL_S = 0.2
SYNC_PROPORTIONAL_GAIN = 30.0
POSITION_PROPORTIONAL_GAIN = 50.0
MAX_SYNC_CORRECTION = 8
DISTANCE_STOP_MARGIN_M = 0.02
MAX_DISTANCE_DRIVE_TIME_S = 15.0
MAX_TURN_TIME_S = 15.0
LEFT_TURN_BRAKE_MARGIN_DEG = 4.2
RIGHT_TURN_BRAKE_MARGIN_DEG = 5.1
TURN_HEADING_COMPENSATION_DEG = 1.5


def distance_target_reached(
    center_progress_m: float,
    left_progress_m: float,
    right_progress_m: float,
    target_m: float,
) -> bool:
    target_threshold = max(0.0, target_m - DISTANCE_STOP_MARGIN_M)
    return center_progress_m >= target_threshold or max(left_progress_m, right_progress_m) >= target_threshold


def turn_speed_for_error(remaining_degrees: float, base_speed: int = INITIAL_TEST_SPEED) -> int:
    if remaining_degrees <= 0:
        return 0
    if remaining_degrees <= 10:
        return max(20, round(base_speed * 0.67))
    if remaining_degrees <= 25:
        return max(20, round(base_speed * 0.67))
    return base_speed


def turn_target_reached(progress_degrees: float, target_degrees: float, direction: str) -> bool:
    base_margin = LEFT_TURN_BRAKE_MARGIN_DEG if direction == "L" else RIGHT_TURN_BRAKE_MARGIN_DEG
    if target_degrees <= 10.0:
        effective_target = target_degrees - 0.25
    elif target_degrees <= 20.0:
        effective_target = target_degrees - 0.5
    else:
        effective_target = target_degrees - base_margin - TURN_HEADING_COMPENSATION_DEG
    return progress_degrees >= max(0.0, effective_target)


def waypoint_turn_command(
    start_x: float,
    start_y: float,
    target_x: float,
    target_y: float,
    current_heading_deg: float,
) -> tuple[str, float]:
    """Return the shortest turn direction and magnitude needed to face a waypoint."""
    dx = target_x - start_x
    dy = target_y - start_y
    if math.isclose(dx, 0.0, abs_tol=1e-9) and math.isclose(dy, 0.0, abs_tol=1e-9):
        return "L", 0.0

    target_heading = math.degrees(math.atan2(dy, dx))
    delta = ((target_heading - current_heading_deg + 180.0) % 360.0) - 180.0
    if abs(delta) < 1e-6:
        return "L", 0.0
    direction = "L" if delta > 0 else "R"
    return direction, abs(delta)


def waypoint_turn_target(turn_degrees: float) -> float:
    """Compensate for the robot's early braking so small heading corrections actually complete."""
    if turn_degrees <= 0.0:
        return 0.0
    return min(90.0, turn_degrees + LEFT_TURN_BRAKE_MARGIN_DEG + TURN_HEADING_COMPENSATION_DEG + 1.0)


def balanced_motor_speeds(
    base_speed: float,
    left_rate: float,
    right_rate: float,
    left_progress: float = 0.0,
    right_progress: float = 0.0,
    left_speed_trim_percent: float = 0.0,
) -> tuple[float, float]:
    """Trim PWM to balance wheel speed and catch up cumulative rotation error."""
    rate_error = left_rate - right_rate
    progress_error = left_progress - right_progress
    correction = (
        SYNC_PROPORTIONAL_GAIN * rate_error
        + POSITION_PROPORTIONAL_GAIN * progress_error
    )
    correction = max(-MAX_SYNC_CORRECTION, min(MAX_SYNC_CORRECTION, correction))
    left_speed = max(0.0, base_speed - correction + base_speed * left_speed_trim_percent / 100.0)
    right_speed = max(0.0, base_speed + correction)
    return left_speed, right_speed


class OdometryLineTestController(MotorTestController):
    """Reuse motor-test commands and report pose from encoder counts."""

    def __init__(self, enable_imu: bool = False) -> None:
        super().__init__(speed=INITIAL_TEST_SPEED)
        self.left_encoder = QuadratureEncoder(
            LEFT_ENCODER_A,
            LEFT_ENCODER_B,
            name="LEFT",
            direction_inverted=LEFT_ENCODER_INVERTED,
        )
        self.right_encoder = QuadratureEncoder(
            RIGHT_ENCODER_A,
            RIGHT_ENCODER_B,
            name="RIGHT",
            direction_inverted=RIGHT_ENCODER_INVERTED,
        )
        self.odometry = EncoderOdometry()
        self.imu = None
        self.heading_estimator = None
        self._heading_update_time = time.monotonic()
        if enable_imu:
            self._initialize_imu()
        self._straight_direction = 0
        self._last_sync_time = time.monotonic()
        self._last_sync_ticks = (0, 0)
        self._straight_start_ticks = (0, 0)

    def _initialize_imu(self) -> None:
        try:
            imu = MPU6050()
            if imu.bus is None:
                logger.warning("I2C Python library unavailable; using encoder-only heading.")
                return
            logger.info("Keep the robot still while calibrating MPU6050 gyro bias.")
            imu.calibrate_gyro_bias(samples=200, delay_seconds=0.01)
            self.imu = imu
            self.heading_estimator = HeadingEstimator(math.degrees(self.odometry.theta))
            self._heading_update_time = time.monotonic()
            logger.info("MPU6050 gyro calibrated; encoder/gyro heading fusion enabled.")
        except (OSError, RuntimeError) as exc:
            logger.warning("Could not initialize MPU6050 (%s); using encoder-only heading.", exc)

    def cleanup_imu(self) -> None:
        if self.imu is not None:
            self.imu.close()
            self.imu = None
            self.heading_estimator = None

    def _set_motion(self, command: str) -> None:
        super()._set_motion(command)
        if command == "F":
            self._straight_direction = 1
        elif command == "B":
            self._straight_direction = -1
        else:
            self._straight_direction = 0
        self._last_sync_time = time.monotonic()
        self._last_sync_ticks = (self.left_encoder.get_ticks(), self.right_encoder.get_ticks())
        self._straight_start_ticks = self._last_sync_ticks
        if command == "S":
            self.report_pose()

    def _sync_wheel_speeds(self, now: float) -> None:
        if not self._straight_direction or now - self._last_sync_time < SYNC_INTERVAL_S:
            return

        left_ticks = self.left_encoder.get_ticks()
        right_ticks = self.right_encoder.get_ticks()
        elapsed = now - self._last_sync_time
        left_rate = self._straight_direction * (left_ticks - self._last_sync_ticks[0])
        right_rate = self._straight_direction * (right_ticks - self._last_sync_ticks[1])
        left_rate /= LEFT_ENCODER_TICKS_PER_REV * elapsed
        right_rate /= RIGHT_ENCODER_TICKS_PER_REV * elapsed

        left_progress = self._straight_direction * (left_ticks - self._straight_start_ticks[0])
        right_progress = self._straight_direction * (right_ticks - self._straight_start_ticks[1])
        left_progress /= LEFT_ENCODER_TICKS_PER_REV
        right_progress /= RIGHT_ENCODER_TICKS_PER_REV

        left_speed, right_speed = balanced_motor_speeds(
            self.speed,
            left_rate,
            right_rate,
            left_progress,
            right_progress,
            LEFT_MOTOR_SPEED_TRIM_PERCENT,
        )
        direction = self._straight_direction
        self.driver.set_motors(direction * left_speed, direction * right_speed)
        self._last_sync_time = now
        self._last_sync_ticks = (left_ticks, right_ticks)

    def run(self) -> None:
        logger.info("Odometry line test started at motor speed %s.", self.speed)
        logger.info("Commands: F/B move, L/R turn, S stop, Q quit. Wheel-speed balancing is active for F/B.")
        logger.info("Enter each command and press Enter. Final readings appear after S or Q.")
        print("Command> ", end="", flush=True)

        try:
            while True:
                now = time.monotonic()
                self._sync_wheel_speeds(now)

                readable, _, _ = select.select([sys.stdin], [], [], 0.02)
                if not readable:
                    continue

                raw = sys.stdin.readline()
                if not raw:
                    break
                command = raw.strip().upper()
                if not command:
                    print("Command> ", end="", flush=True)
                    continue
                if command == "Q":
                    self._straight_direction = 0
                    self.driver.stop()
                    self.report_pose()
                    break
                self._set_motion(command)
                print("Command> ", end="", flush=True)
        finally:
            self._straight_direction = 0
            self.driver.stop()
            self.cleanup_encoders()
            self.driver.cleanup()

    def drive_distance(self, target_m: float, reset_state: bool = True) -> bool:
        if not 0.1 <= target_m <= 1.0:
            raise ValueError("Distance target must be between 0.10 m and 1.00 m for this test.")

        if reset_state:
            self.left_encoder.reset()
            self.right_encoder.reset()
            self.odometry.reset()
            if getattr(self, "heading_estimator", None) is not None:
                self.heading_estimator.reset(math.degrees(self.odometry.theta))
                self._heading_update_time = time.monotonic()
        start_left_ticks = self.left_encoder.get_ticks()
        start_right_ticks = self.right_encoder.get_ticks()
        start_x = self.odometry.x
        start_y = self.odometry.y
        _, _, start_heading = self.update_pose()
        self._straight_direction = 1
        self._last_sync_time = time.monotonic()
        drive_started_at = self._last_sync_time
        self._last_sync_ticks = (start_left_ticks, start_right_ticks)
        self._straight_start_ticks = (start_left_ticks, start_right_ticks)
        self.driver.set_motors(self.speed, self.speed)
        logger.info("Driving forward %.2f m at speed %s; press S and Enter to abort.", target_m, self.speed)
        completed = False

        try:
            while True:
                now = time.monotonic()
                self._sync_wheel_speeds(now)
                x, _, _ = self.update_pose()
                dx = x - start_x
                dy = self.odometry.y - start_y
                center_progress = dx * math.cos(start_heading) + dy * math.sin(start_heading)
                left_progress_m, right_progress_m = self.odometry.wheel_distances_from_ticks(
                    self.left_encoder.get_ticks() - start_left_ticks,
                    self.right_encoder.get_ticks() - start_right_ticks,
                )
                if distance_target_reached(center_progress, left_progress_m, right_progress_m, target_m):
                    logger.info("Distance target reached; braking.")
                    completed = True
                    break
                if now - drive_started_at >= MAX_DISTANCE_DRIVE_TIME_S:
                    logger.warning("Distance test timed out; braking for safety.")
                    break

                readable, _, _ = select.select([sys.stdin], [], [], 0.02)
                if readable:
                    command = sys.stdin.readline().strip().upper()
                    if command in {"S", "Q"}:
                        logger.info("Distance drive aborted by user.")
                        break
        finally:
            self._straight_direction = 0
            self.driver.stop()

        self.report_pose()
        return completed

    def turn_angle(self, target_degrees: float, direction: str, reset_state: bool = True) -> bool:
        if not 1.0 <= target_degrees <= 90.0:
            raise ValueError("Turn target must be between 1 and 90 degrees for this test.")
        if direction not in {"L", "R"}:
            raise ValueError("Turn direction must be L or R.")

        if reset_state:
            self.left_encoder.reset()
            self.right_encoder.reset()
            self.odometry.reset()
            if getattr(self, "heading_estimator", None) is not None:
                self.heading_estimator.reset(math.degrees(self.odometry.theta))
                self._heading_update_time = time.monotonic()
        turn_sign = 1 if direction == "L" else -1
        _, _, start_heading = self.update_pose()
        started_at = time.monotonic()
        current_speed = 0
        logger.info("Turning %s %.1f degrees; press S and Enter to abort.", direction, target_degrees)
        completed = False

        try:
            while True:
                x, y, theta = self.update_pose()
                heading_delta = math.degrees(theta - start_heading)
                progress_degrees = turn_sign * MPU6050.normalize_heading_deg(heading_delta)
                remaining_degrees = target_degrees - progress_degrees
                if turn_target_reached(progress_degrees, target_degrees, direction):
                    logger.info("Turn target reached; braking.")
                    completed = True
                    break

                if time.monotonic() - started_at >= MAX_TURN_TIME_S:
                    logger.warning("Turn test timed out; braking for safety.")
                    break

                desired_speed = turn_speed_for_error(remaining_degrees, self.speed)
                if desired_speed != current_speed:
                    self.driver.set_motors(
                        -turn_sign * desired_speed,
                        turn_sign * desired_speed,
                    )
                    current_speed = desired_speed

                readable, _, _ = select.select([sys.stdin], [], [], 0.02)
                if readable and sys.stdin.readline().strip().upper() in {"S", "Q"}:
                    logger.info("Turn aborted by user.")
                    break
        finally:
            self.driver.stop()

        self.report_pose()
        return completed

    def update_pose(self) -> tuple[float, float, float]:
        x, y, encoder_theta = self.odometry.update(
            self.left_encoder.get_ticks(),
            self.right_encoder.get_ticks(),
        )
        imu = getattr(self, "imu", None)
        heading_estimator = getattr(self, "heading_estimator", None)
        if imu is None or heading_estimator is None:
            return x, y, encoder_theta

        now = time.monotonic()
        dt = max(now - self._heading_update_time, 0.001)
        self._heading_update_time = now
        try:
            gyro_z = imu.gyro_deg_per_sec()[2]
            fused_heading = heading_estimator.fuse_with_encoder(
                math.degrees(encoder_theta),
                gyro_z,
                dt=dt,
            )
        except (OSError, RuntimeError) as exc:
            logger.warning("IMU read failed (%s); continuing with encoder-only heading.", exc)
            self.cleanup_imu()
            return x, y, encoder_theta
        return x, y, math.radians(fused_heading)

    def report_pose(self) -> None:
        x, y, theta = self.update_pose()
        left_ticks = self.left_encoder.get_ticks()
        right_ticks = self.right_encoder.get_ticks()
        left_distance, right_distance = self.odometry.wheel_distances_from_ticks(
            left_ticks,
            right_ticks,
        )
        distance = math.hypot(x, y)
        print(
            f"Ticks L/R: {left_ticks}/{right_ticks} | "
            f"Pose x={x:.3f} m, y={y:.3f} m, distance={distance:.3f} m, "
            f"heading={math.degrees(theta):.1f} deg"
        )
        print(
            f"Wheel travel L/R: {left_distance:.3f}/{right_distance:.3f} m | "
            f"difference={right_distance - left_distance:+.3f} m"
        )

    def move_to_waypoint(
        self,
        target_x: float,
        target_y: float,
        tolerance_m: float = 0.05,
        max_step_count: int = 15,
    ) -> bool:
        """Drive toward a waypoint using the existing straight/turn primitives."""
        for _ in range(max_step_count):
            x, y, theta = self.update_pose()
            dx = target_x - x
            dy = target_y - y
            distance_to_target = math.hypot(dx, dy)
            if distance_to_target <= tolerance_m:
                return True

            desired_heading = math.degrees(math.atan2(dy, dx))
            heading_error = ((desired_heading - math.degrees(theta) + 180.0) % 360.0) - 180.0
            if abs(heading_error) > 2.0:
                direction, turn_degrees = waypoint_turn_command(x, y, target_x, target_y, math.degrees(theta))
                if turn_degrees > 0.0:
                    turn_amount = waypoint_turn_target(min(turn_degrees, 90.0))
                    if not self.turn_angle(turn_amount, direction, reset_state=False):
                        return False
                continue

            step_distance = min(distance_to_target, 1.0)
            if step_distance < 0.1:
                if distance_to_target <= tolerance_m:
                    return True
                step_distance = 0.1
            if not self.drive_distance(step_distance, reset_state=False):
                return False

        x, y, _ = self.update_pose()
        return math.hypot(target_x - x, target_y - y) <= tolerance_m

    def cleanup_encoders(self) -> None:
        self.left_encoder.cleanup()
        self.right_encoder.cleanup()

    def cleanup(self) -> None:
        """Clean up IMU, encoders, and motor driver safely."""
        self.cleanup_imu()
        self.cleanup_encoders()
        if hasattr(self, "driver") and self.driver is not None:
            self.driver.cleanup()


def main() -> None:
    controller = None
    try:
        controller = OdometryLineTestController()
        controller.run()
    except RuntimeError as exc:
        logger.error("Could not start the odometry line test: %s", exc)
        raise SystemExit(1)
    finally:
        if controller is not None:
            controller.cleanup()


if __name__ == "__main__":
    main()