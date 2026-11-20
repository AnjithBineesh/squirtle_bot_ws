"""Drive to a waypoint using the proven straight-line and turn primitives."""

from __future__ import annotations

import logging

from squirtle_bot.tests.odometry_line_test import OdometryLineTestController

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def main() -> None:
    controller = None
    try:
        controller = OdometryLineTestController(enable_imu=True)
        target_x = float(input("Target X in meters: ").strip())
        target_y = float(input("Target Y in meters: ").strip())
        if not controller.move_to_waypoint(target_x, target_y):
            logger.warning("Waypoint navigation did not complete successfully.")
        controller.report_pose()
        logger.info("Final pose reported above. Compare it with the target waypoint.")
    except (RuntimeError, ValueError) as exc:
        logger.error("Waypoint navigation failed: %s", exc)
        raise SystemExit(1)
    finally:
        if controller is not None:
            controller.cleanup()


if __name__ == "__main__":
    main()
