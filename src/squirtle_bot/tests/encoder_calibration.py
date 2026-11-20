"""Manual encoder calibration helper for Squirtle Bot.

This script measures the number of encoder ticks produced for a known number of
wheel revolutions. Use it to determine the real ENCODER_TICKS_PER_REV value for
odometry.
"""

from __future__ import annotations

import logging

from squirtle_bot.config import (
    LEFT_ENCODER_A,
    LEFT_ENCODER_B,
    LEFT_ENCODER_INVERTED,
    RIGHT_ENCODER_A,
    RIGHT_ENCODER_B,
    RIGHT_ENCODER_INVERTED,
)
from squirtle_bot.hardware.encoder import QuadratureEncoder

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def calibrate_encoder(
    channel_a: int,
    channel_b: int,
    name: str,
    direction_inverted: bool = False,
) -> None:
    """Prompt the user to rotate the wheel and estimate ticks per revolution."""
    encoder = QuadratureEncoder(
        channel_a,
        channel_b,
        name=name,
        direction_inverted=direction_inverted,
    )

    try:
        while True:
            raw = input(f"{name} encoder calibration: enter number of wheel turns to measure (integer, 0 to quit): ").strip()
            if not raw:
                continue

            try:
                turns = int(raw)
            except ValueError:
                logger.warning("Please enter an integer number of turns.")
                continue

            if turns <= 0:
                logger.info("Calibration cancelled.")
                break

            encoder.reset()
            logger.info("Rotate the %s wheel exactly %s full turn(s). Press Enter when done.", name, turns)
            input("Press Enter to capture the measured ticks... ")

            ticks = abs(encoder.get_ticks())
            ticks_per_rev = ticks / turns
            logger.info("Measured %s ticks over %s turn(s).", ticks, turns)
            logger.info("Estimated %s ticks per revolution: %.2f", name, ticks_per_rev)
            logger.info("Current tick count: %s", encoder.get_ticks())
    finally:
        encoder.cleanup()


def main() -> None:
    logger.info("Encoder calibration started.")
    logger.info("Calibrate each wheel separately; record both results for odometry.")
    calibrate_encoder(LEFT_ENCODER_A, LEFT_ENCODER_B, "LEFT", LEFT_ENCODER_INVERTED)
    calibrate_encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, "RIGHT", RIGHT_ENCODER_INVERTED)


if __name__ == "__main__":
    main()
