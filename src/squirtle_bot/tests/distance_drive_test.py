"""Run a short encoder-based forward distance test."""

from __future__ import annotations

import logging

from squirtle_bot.tests.odometry_line_test import OdometryLineTestController

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def main() -> None:
    controller = None
    try:
        controller = OdometryLineTestController()
        raw_target = input("Forward distance in meters (0.10 to 1.00): ").strip()
        target_m = float(raw_target)
        controller.drive_distance(target_m)
    except (RuntimeError, ValueError) as exc:
        logger.error("Distance test could not run: %s", exc)
        raise SystemExit(1)
    finally:
        if controller is not None:
            controller.cleanup()


if __name__ == "__main__":
    main()