"""Interactive safety-first motor test for Squirtle Bot.

Controls:
  F - forward
  B - backward
  L - left turn
  R - right turn
  S - stop
  Q - quit
"""

from __future__ import annotations

import logging
import signal
import sys
from typing import Optional

from squirtle_bot.config import INITIAL_TEST_SPEED, MOTOR_TEST_SPEED
from squirtle_bot.hardware.motor_driver import L298NMotorDriver

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


class MotorTestController:
    """Handle the interactive key-based motor validation workflow."""

    def __init__(self, driver: Optional[L298NMotorDriver] = None, speed: int = MOTOR_TEST_SPEED) -> None:
        self.driver = driver or L298NMotorDriver()
        self.speed = speed
        self._stopped = False
        self._install_signal_handlers()

    def _install_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_sigint)

    def _handle_sigint(self, signum, frame) -> None:  # pragma: no cover - signal based path
        logger.warning("SIGINT received. Stopping motors safely.")
        self.stop_and_exit()

    def _set_motion(self, command: str) -> None:
        if command == "F":
            self.driver.set_motors(self.speed, self.speed)
        elif command == "B":
            self.driver.set_motors(-self.speed, -self.speed)
        elif command == "L":
            self.driver.set_motors(-self.speed, self.speed)
        elif command == "R":
            self.driver.set_motors(self.speed, -self.speed)
        elif command == "S":
            self.driver.stop()
        else:
            logger.warning("Unknown command: %s", command)

    def stop_and_exit(self) -> None:
        """Stop motors and close GPIO cleanly."""
        if self._stopped:
            return
        self._stopped = True
        try:
            self.driver.stop()
        finally:
            self.driver.cleanup()
            raise SystemExit(0)

    def run(self) -> None:
        logger.info("Squirtle Bot motor test started.")
        logger.info("Commands: F, B, L, R, S, Q")
        logger.info("Initial speed: %s", self.speed)
        logger.info("Keep the wheels off the ground during the first direction checks.")

        while True:
            try:
                raw = input("Command> ").strip().upper()
            except KeyboardInterrupt:
                logger.warning("Keyboard interrupt received.")
                self.stop_and_exit()
            except EOFError:
                logger.warning("Input closed. Stopping motors.")
                self.stop_and_exit()

            if not raw:
                continue

            if raw == "Q":
                logger.info("Quit requested. Stopping motors safely.")
                self.driver.stop()
                break

            self._set_motion(raw)

        self.driver.stop()
        self.driver.cleanup()


def run_motor_test() -> None:
    """Entry point for running the low-speed motor validation test."""
    try:
        motor_driver = L298NMotorDriver()
        controller = MotorTestController(driver=motor_driver, speed=INITIAL_TEST_SPEED)
        controller.run()
    except RuntimeError as exc:
        logger.error(str(exc))
        logger.error(
            "GPIO initialization failed. On Raspberry Pi 5, install the compatible backend with: "
            "sudo apt install -y python3-rpi-lgpio"
        )
        raise SystemExit(1)
    except KeyboardInterrupt:
        logger.warning("Motor test interrupted.")
        if "motor_driver" in locals():
            motor_driver.stop()
            motor_driver.cleanup()
        raise SystemExit(0)


if __name__ == "__main__":
    run_motor_test()
