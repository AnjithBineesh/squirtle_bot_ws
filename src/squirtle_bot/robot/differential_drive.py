"""Differential-drive kinematics for Squirtle Bot."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass
class DifferentialDrive:
    """Convert robot-level motion commands into left/right wheel speeds."""

    wheel_base: float = 0.0

    def set_wheel_base(self, wheel_base: float) -> None:
        self.wheel_base = wheel_base

    def command_to_wheel_speeds(self, linear_velocity: float, angular_velocity: float) -> tuple[float, float]:
        """Use standard differential-drive kinematics.

        left = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right = linear_velocity + (angular_velocity * self.wheel_base / 2.0)
        """
        if self.wheel_base <= 0:
            return linear_velocity, linear_velocity

        left = linear_velocity - (angular_velocity * self.wheel_base / 2.0)
        right = linear_velocity + (angular_velocity * self.wheel_base / 2.0)
        return left, right

    def wheel_speeds_to_linear_angular(self, left_speed: float, right_speed: float) -> tuple[float, float]:
        """Inverse kinematics to compute robot linear and angular velocity."""
        if self.wheel_base <= 0:
            return (left_speed + right_speed) / 2.0, 0.0

        linear_velocity = (left_speed + right_speed) / 2.0
        angular_velocity = (right_speed - left_speed) / self.wheel_base
        return linear_velocity, angular_velocity


if __name__ == "__main__":  # pragma: no cover
    drive = DifferentialDrive(wheel_base=0.38)
    print(drive.command_to_wheel_speeds(0.3, 0.2))
