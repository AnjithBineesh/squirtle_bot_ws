"""Encoder odometry for Squirtle Bot."""

from __future__ import annotations

import math

from squirtle_bot.config import (
    LEFT_ENCODER_TICKS_PER_REV,
    ODOMETRY_DISTANCE_SCALE,
    ODOMETRY_WHEEL_BASE,
    RIGHT_ENCODER_TICKS_PER_REV,
    WHEEL_DIAMETER,
)


class EncoderOdometry:
    """Estimate robot pose from encoder data using differential-drive kinematics."""

    def __init__(
        self,
        wheel_base: float = ODOMETRY_WHEEL_BASE,
        wheel_diameter: float = WHEEL_DIAMETER,
        ticks_per_rev: float | None = None,
        x: float = 0.0,
        y: float = 0.0,
        theta: float = 0.0,
        left_ticks_per_rev: float = LEFT_ENCODER_TICKS_PER_REV,
        right_ticks_per_rev: float = RIGHT_ENCODER_TICKS_PER_REV,
        distance_scale: float = ODOMETRY_DISTANCE_SCALE,
    ) -> None:
        self.wheel_base = wheel_base
        self.wheel_diameter = wheel_diameter
        self.distance_scale = distance_scale
        if ticks_per_rev is not None:
            left_ticks_per_rev = ticks_per_rev
            right_ticks_per_rev = ticks_per_rev
        self.left_ticks_per_rev = left_ticks_per_rev
        self.right_ticks_per_rev = right_ticks_per_rev
        self.ticks_per_rev = ticks_per_rev if ticks_per_rev is not None else left_ticks_per_rev
        self.x = x
        self.y = y
        self.theta = theta

        self.last_left_ticks = 0
        self.last_right_ticks = 0

    def _ticks_to_distance(self, ticks: int, ticks_per_rev: float | None = None) -> float:
        resolution = self.ticks_per_rev if ticks_per_rev is None else ticks_per_rev
        if resolution <= 0 or self.wheel_diameter <= 0:
            return 0.0
        revolutions = ticks / resolution
        return revolutions * math.pi * self.wheel_diameter * self.distance_scale

    def wheel_distances_from_ticks(self, left_ticks: int, right_ticks: int) -> tuple[float, float]:
        """Convert cumulative left/right counts to their calibrated travel distances."""
        return (
            self._ticks_to_distance(left_ticks, self.left_ticks_per_rev),
            self._ticks_to_distance(right_ticks, self.right_ticks_per_rev),
        )

    def update(self, left_ticks: int, right_ticks: int) -> tuple[float, float, float]:
        """Integrate encoder deltas into a pose estimate."""
        left_delta = left_ticks - self.last_left_ticks
        right_delta = right_ticks - self.last_right_ticks

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        left_distance = self._ticks_to_distance(left_delta, self.left_ticks_per_rev)
        right_distance = self._ticks_to_distance(right_delta, self.right_ticks_per_rev)

        if self.wheel_base <= 0:
            self.x += (left_distance + right_distance) / 2.0 * math.cos(self.theta)
            self.y += (left_distance + right_distance) / 2.0 * math.sin(self.theta)
            return self.x, self.y, self.theta

        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_base

        if abs(delta_theta) < 1e-9:
            self.x += delta_distance * math.cos(self.theta)
            self.y += delta_distance * math.sin(self.theta)
            return self.x, self.y, self.theta

        radius = delta_distance / delta_theta
        self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
        self.y += radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
        self.theta += delta_theta
        return self.x, self.y, self.theta

    def reset(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> None:
        self.x = x
        self.y = y
        self.theta = theta
        self.last_left_ticks = 0
        self.last_right_ticks = 0


if __name__ == "__main__":  # pragma: no cover
    odom = EncoderOdometry(wheel_base=0.38, wheel_diameter=0.065, ticks_per_rev=1000)
    print(odom.update(100, 100))
