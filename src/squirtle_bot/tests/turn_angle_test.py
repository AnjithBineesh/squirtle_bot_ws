"""Safely test an encoder-controlled in-place turn."""

from __future__ import annotations

import logging

from squirtle_bot.tests.odometry_line_test import OdometryLineTestController

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def main() -> None:
    controller = None
    try:
        controller = OdometryLineTestController(enable_imu=True)
        direction = input("Turn direction (L/R): ").strip().upper()
        target = float(input("Turn angle in degrees (1 to 90): ").strip())
        controller.turn_angle(target, direction)
    except (RuntimeError, ValueError) as exc:
        logger.error("Turn test could not run: %s", exc)
        raise SystemExit(1)
    finally:
        if controller is not None:
            controller.cleanup()


if __name__ == "__main__":
    main()