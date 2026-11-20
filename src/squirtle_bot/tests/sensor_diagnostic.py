"""Unified live diagnostic script for Squirtle Bot Encoders and IMU.

Tests the MPU6050 and both quadrature encoders simultaneously,
providing real-time feedback on wheel ticks, travel distances,
instantaneous gyro angular rates, and integrated yaw heading.
"""

from __future__ import annotations

import logging
import math
import signal
import sys
import time

from squirtle_bot.config import (
    LEFT_ENCODER_A,
    LEFT_ENCODER_B,
    LEFT_ENCODER_INVERTED,
    LEFT_ENCODER_TICKS_PER_REV,
    ODOMETRY_DISTANCE_SCALE,
    RIGHT_ENCODER_A,
    RIGHT_ENCODER_B,
    RIGHT_ENCODER_INVERTED,
    RIGHT_ENCODER_TICKS_PER_REV,
    WHEEL_DIAMETER,
)
from squirtle_bot.hardware.encoder import QuadratureEncoder
from squirtle_bot.sensors.imu import MPU6050

logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
logger = logging.getLogger(__name__)


def ticks_to_meters(ticks: int, ticks_per_rev: float) -> float:
    if ticks_per_rev <= 0 or WHEEL_DIAMETER <= 0:
        return 0.0
    return (ticks / ticks_per_rev) * math.pi * WHEEL_DIAMETER * ODOMETRY_DISTANCE_SCALE


def run_diagnostic(duration_seconds: float = 30.0) -> None:
    left_enc = None
    right_enc = None
    imu = None

    running = True

    def _sig_handler(signum, frame):
        nonlocal running
        running = False
        print("\nStopping diagnostic...")

    signal.signal(signal.SIGINT, _sig_handler)

    try:
        logger.info("Initializing MPU6050 IMU on I2C bus...")
        imu = MPU6050()
        logger.info("Keep robot stationary! Calibrating gyro zero-rate bias (100 samples)...")
        imu.calibrate_gyro_bias(samples=100, delay_seconds=0.01)
        bx, by, bz = imu.gyro_bias
        logger.info("Gyro bias calibrated: X=%.1f, Y=%.1f, Z=%.1f LSB", bx, by, bz)

        logger.info("Initializing Left Encoder (Pins %d, %d)...", LEFT_ENCODER_A, LEFT_ENCODER_B)
        left_enc = QuadratureEncoder(
            LEFT_ENCODER_A,
            LEFT_ENCODER_B,
            name="LEFT",
            direction_inverted=LEFT_ENCODER_INVERTED,
        )

        logger.info("Initializing Right Encoder (Pins %d, %d)...", RIGHT_ENCODER_A, RIGHT_ENCODER_B)
        right_enc = QuadratureEncoder(
            RIGHT_ENCODER_A,
            RIGHT_ENCODER_B,
            name="RIGHT",
            direction_inverted=RIGHT_ENCODER_INVERTED,
        )

        left_enc.reset()
        right_enc.reset()
        imu.reset_yaw(0.0)

        print("\n" + "=" * 78)
        print(" SQUIRTLE BOT — LIVE SENSOR DIAGNOSTIC")
        print(" Rotate wheels or turn the robot body to observe real-time feedback.")
        print(" Press Ctrl+C or wait for timeout to complete.")
        print("=" * 78 + "\n")

        start_time = time.monotonic()
        while running:
            elapsed = time.monotonic() - start_time
            if duration_seconds > 0 and elapsed >= duration_seconds:
                logger.info("Diagnostic completed after %.1f seconds.", elapsed)
                break

            # Read IMU
            yaw = imu.update_yaw()
            gx, gy, gz = imu.gyro_deg_per_sec()

            # Read Encoders
            l_ticks = left_enc.get_ticks()
            r_ticks = right_enc.get_ticks()
            l_dir = left_enc.get_direction().value
            r_dir = right_enc.get_direction().value
            l_dist = ticks_to_meters(l_ticks, LEFT_ENCODER_TICKS_PER_REV)
            r_dist = ticks_to_meters(r_ticks, RIGHT_ENCODER_TICKS_PER_REV)

            output = (
                f"\r[T={elapsed:4.1f}s] "
                f"L: {l_ticks:+5d}t ({l_dist:+5.2f}m) [{l_dir:7s}] | "
                f"R: {r_ticks:+5d}t ({r_dist:+5.2f}m) [{r_dir:7s}] | "
                f"Yaw: {yaw:+6.1f}° | Gz: {gz:+5.1f}°/s"
            )
            sys.stdout.write(output)
            sys.stdout.flush()
            time.sleep(0.1)

        print("\n\n" + "=" * 78)
        print(" FINAL SENSOR SUMMARY:")
        print(f" Left Wheel:  {left_enc.get_ticks()} ticks -> {ticks_to_meters(left_enc.get_ticks(), LEFT_ENCODER_TICKS_PER_REV):.3f} m")
        print(f" Right Wheel: {right_enc.get_ticks()} ticks -> {ticks_to_meters(right_enc.get_ticks(), RIGHT_ENCODER_TICKS_PER_REV):.3f} m")
        print(f" IMU Heading: {imu.yaw_deg:.2f}° (Integrated Yaw)")
        print("=" * 78)

    finally:
        if left_enc is not None:
            left_enc.cleanup()
        if right_enc is not None:
            right_enc.cleanup()
        if imu is not None:
            imu.close()
        logger.info("All GPIO and I2C hardware cleaned up safely.")


if __name__ == "__main__":
    duration = 30.0
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            duration = 30.0
    run_diagnostic(duration_seconds=duration)
