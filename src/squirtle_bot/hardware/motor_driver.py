"""L298N motor driver interface for Squirtle Bot.

This module owns GPIO control for the H-bridge and exposes a small, testable API
for left/right motor commands using a normalized range of -100 to +100.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - used when running outside Raspberry Pi hardware.
    GPIO = None

from squirtle_bot.config import (
    ENA,
    ENB,
    IN1,
    IN2,
    IN3,
    IN4,
    LEFT_MOTOR_INVERTED,
    MAX_SAFE_PWM,
    MOTOR_BRAKE_TIME_S,
    MOTOR_PWM_FREQUENCY_HZ,
    MOTOR_SPEED_LIMIT,
    RIGHT_MOTOR_INVERTED,
)

logger = logging.getLogger(__name__)


class L298NMotorDriver:
    """Control the L298N dual-motor driver with GPIO and PWM."""

    def __init__(
        self,
        enable_a: int = ENA,
        in1: int = IN1,
        in2: int = IN2,
        enable_b: int = ENB,
        in3: int = IN3,
        in4: int = IN4,
        pwm_frequency_hz: int = MOTOR_PWM_FREQUENCY_HZ,
        left_motor_inverted: bool = LEFT_MOTOR_INVERTED,
        right_motor_inverted: bool = RIGHT_MOTOR_INVERTED,
        gpio_module: Optional[object] = None,
    ) -> None:
        self.gpio = gpio_module or GPIO
        if self.gpio is None:
            raise RuntimeError(
                "RPi.GPIO is not available. This motor driver requires a Raspberry Pi GPIO environment."
            )

        self.enable_a = enable_a
        self.in1 = in1
        self.in2 = in2
        self.enable_b = enable_b
        self.in3 = in3
        self.in4 = in4
        self.pwm_frequency_hz = pwm_frequency_hz
        self.left_motor_inverted = left_motor_inverted
        self.right_motor_inverted = right_motor_inverted
        self.left_pwm = None
        self.right_pwm = None
        self._cleaned = False

        self._setup_gpio()

    def _setup_gpio(self) -> None:
        """Configure the L298N control pins for output mode."""
        if self.gpio.getmode() is None:
            self.gpio.setmode(self.gpio.BCM)

        self.gpio.setwarnings(False)

        for pin in (self.enable_a, self.in1, self.in2, self.enable_b, self.in3, self.in4):
            self.gpio.setup(pin, self.gpio.OUT, initial=self.gpio.LOW)

        self.left_pwm = self.gpio.PWM(self.enable_a, self.pwm_frequency_hz)
        self.right_pwm = self.gpio.PWM(self.enable_b, self.pwm_frequency_hz)

        self.left_pwm.start(0)
        self.right_pwm.start(0)

    @staticmethod
    def _clamp_speed(speed: float) -> float:
        """Clamp to the allowed normalized range of -100 to +100."""
        if speed > MAX_SAFE_PWM:
            return float(MAX_SAFE_PWM)
        if speed < -MAX_SAFE_PWM:
            return float(-MAX_SAFE_PWM)
        return float(speed)

    def _prepare_motor_command(self, in_a: int, in_b: int, speed: float, inverted: bool = False) -> float:
        """Set motor direction pins and return the PWM duty cycle to apply."""
        requested_speed = self._clamp_speed(speed)
        effective_speed = -requested_speed if inverted else requested_speed

        if requested_speed == 0:
            self.gpio.output(in_a, self.gpio.LOW)
            self.gpio.output(in_b, self.gpio.LOW)
            return 0

        if effective_speed > 0:
            self.gpio.output(in_a, self.gpio.HIGH)
            self.gpio.output(in_b, self.gpio.LOW)
        else:
            self.gpio.output(in_a, self.gpio.LOW)
            self.gpio.output(in_b, self.gpio.HIGH)

        return abs(effective_speed)

    def _apply_motor_command(self, pwm, in_a: int, in_b: int, speed: float, inverted: bool = False) -> None:
        """Apply a normalized command to a motor channel."""
        duty_cycle = self._prepare_motor_command(in_a, in_b, speed, inverted)
        pwm.ChangeDutyCycle(duty_cycle)

    def set_left_motor(self, speed: float) -> None:
        """Set the left motor speed using a normalized range of -100 to +100."""
        self._apply_motor_command(self.left_pwm, self.in1, self.in2, speed, inverted=self.left_motor_inverted)

    def set_right_motor(self, speed: float) -> None:
        """Set the right motor speed using a normalized range of -100 to +100."""
        self._apply_motor_command(self.right_pwm, self.in3, self.in4, speed, inverted=self.right_motor_inverted)

    def set_motors(self, left_speed: float, right_speed: float) -> None:
        """Prepare both directions before applying PWM to either motor."""
        left_duty = self._prepare_motor_command(
            self.in1,
            self.in2,
            left_speed,
            inverted=self.left_motor_inverted,
        )
        right_duty = self._prepare_motor_command(
            self.in3,
            self.in4,
            right_speed,
            inverted=self.right_motor_inverted,
        )
        self.left_pwm.ChangeDutyCycle(left_duty)
        self.right_pwm.ChangeDutyCycle(right_duty)

    def stop(self) -> None:
        """Briefly brake both motors, then disable PWM."""
        if self._cleaned:
            return
        self._prepare_motor_command(self.in1, self.in2, 0)
        self._prepare_motor_command(self.in3, self.in4, 0)
        self.left_pwm.ChangeDutyCycle(100)
        self.right_pwm.ChangeDutyCycle(100)
        time.sleep(MOTOR_BRAKE_TIME_S)
        self.left_pwm.ChangeDutyCycle(0)
        self.right_pwm.ChangeDutyCycle(0)

    def stop_left(self) -> None:
        """Stop the left motor only."""
        self._apply_motor_command(self.left_pwm, self.in1, self.in2, 0)

    def stop_right(self) -> None:
        """Stop the right motor only."""
        self._apply_motor_command(self.right_pwm, self.in3, self.in4, 0)

    def cleanup(self) -> None:
        """Stop PWM and clean up Raspberry Pi GPIO state."""
        if self._cleaned:
            return
        self._cleaned = True
        logger.info("Cleaning up motor GPIO pins.")
        pwm_channels = []
        if hasattr(self, "left_pwm") and self.left_pwm is not None:
            pwm_channels.append(self.left_pwm)
            del self.left_pwm
        if hasattr(self, "right_pwm") and self.right_pwm is not None:
            pwm_channels.append(self.right_pwm)
            del self.right_pwm
        for pwm in pwm_channels:
            try:
                pwm.stop()
            except Exception:  # pragma: no cover - defensive cleanup
                pass
        pwm = None
        del pwm
        del pwm_channels
        self.gpio.cleanup()

    def __enter__(self) -> "L298NMotorDriver":
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.cleanup()


if __name__ == "__main__":  # pragma: no cover
    logging.basicConfig(level=logging.INFO)
    driver = L298NMotorDriver()
    try:
        driver.set_left_motor(20)
        driver.set_right_motor(20)
        logger.info("Motor driver test command executed.")
    finally:
        driver.cleanup()
