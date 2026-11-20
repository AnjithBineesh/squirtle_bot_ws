"""Manual calibration helper for the right wheel encoder."""

from __future__ import annotations

import logging

from squirtle_bot.config import RIGHT_ENCODER_A, RIGHT_ENCODER_B, RIGHT_ENCODER_INVERTED
from squirtle_bot.tests.encoder_calibration import calibrate_encoder

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")


def main() -> None:
    logging.info("Right encoder calibration started.")
    calibrate_encoder(
        RIGHT_ENCODER_A,
        RIGHT_ENCODER_B,
        "RIGHT",
        RIGHT_ENCODER_INVERTED,
    )


if __name__ == "__main__":
    main()
