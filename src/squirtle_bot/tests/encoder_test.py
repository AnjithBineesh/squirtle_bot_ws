"""Standalone encoder testing script for Squirtle Bot.

This script prints the tick count and direction for each encoder. It is intended
for manual wheel rotation testing while the wheels are free to spin.
"""

from __future__ import annotations

import logging
import time

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


def main() -> None:
    left = QuadratureEncoder(
        LEFT_ENCODER_A,
        LEFT_ENCODER_B,
        name="LEFT",
        direction_inverted=LEFT_ENCODER_INVERTED,
    )
    right = QuadratureEncoder(
        RIGHT_ENCODER_A,
        RIGHT_ENCODER_B,
        name="RIGHT",
        direction_inverted=RIGHT_ENCODER_INVERTED,
    )

    try:
        logger.info("Encoder test started. Rotate each wheel manually.")
        logger.info("Press Ctrl+C to quit.")
        while True:
            time.sleep(0.2)
            left_dir = left.get_direction().value
            right_dir = right.get_direction().value
            print(f"Left ticks: {left.get_ticks()} | Right ticks: {right.get_ticks()}")
            print(f"Left direction: {left_dir} | Right direction: {right_dir}")
            print("-" * 40)
    except KeyboardInterrupt:
        logger.info("Encoder test interrupted.")
    finally:
        left.cleanup()
        right.cleanup()


if __name__ == "__main__":
    main()
