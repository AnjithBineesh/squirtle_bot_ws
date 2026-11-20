"""Run an encoder-controlled square path and report its closure error."""

from __future__ import annotations

import logging

from squirtle_bot.tests.odometry_line_test import OdometryLineTestController

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def main() -> None:
    controller = None
    try:
        controller = OdometryLineTestController(enable_imu=True)
        side_length = float(input("Square side length in meters (0.10 to 1.00): ").strip())
        if not 0.1 <= side_length <= 1.0:
            raise ValueError("Side length must be between 0.10 m and 1.00 m.")
        direction = input("Turn direction at each corner (L/R): ").strip().upper()
        if direction not in {"L", "R"}:
            raise ValueError("Turn direction must be L or R.")

        input("Place the robot at the marked start, clear the square path, then press Enter...")
        logger.info("Running a %.2f m square with four %s-degree turns.", side_length, 90)
        for segment in range(4):
            logger.info("Side %s of 4.", segment + 1)
            if not controller.drive_distance(side_length, reset_state=(segment == 0)):
                logger.warning("Square path stopped before completing the side.")
                break
            if not controller.turn_angle(90.0, direction, reset_state=False):
                logger.warning("Square path stopped before completing the turn.")
                break
        else:
            logger.info("Square path complete; compare the final pose to the start mark.")
    except (RuntimeError, ValueError) as exc:
        logger.error("Square path test could not run: %s", exc)
        raise SystemExit(1)
    finally:
        if controller is not None:
            controller.cleanup()


if __name__ == "__main__":
    main()