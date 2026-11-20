"""Quadrature encoder abstraction for Squirtle Bot.

This module supports the left and right encoder instances using GPIO edge
interrupts. It exposes a simple API for tick counting, direction detection,
velocity estimation, and reset.
"""

from __future__ import annotations

import logging
import time
from enum import Enum
from typing import Optional

from squirtle_bot.config import (
    LEFT_ENCODER_A,
    LEFT_ENCODER_B,
    LEFT_ENCODER_INVERTED,
    RIGHT_ENCODER_A,
    RIGHT_ENCODER_B,
    RIGHT_ENCODER_INVERTED,
)

try:
    import RPi.GPIO as GPIO
except ImportError:  # pragma: no cover - safety check for non-Pi hosts.
    GPIO = None

logger = logging.getLogger(__name__)


class EncoderDirection(str, Enum):
    FORWARD = "FORWARD"
    REVERSE = "REVERSE"
    STOPPED = "STOPPED"


class QuadratureEncoder:
    """Read a quadrature encoder attached to two GPIO inputs."""

    def __init__(
        self,
        channel_a: int,
        channel_b: int,
        gpio_module: Optional[object] = None,
        name: str = "encoder",
        direction_inverted: bool = False,
    ) -> None:
        self.gpio = gpio_module or GPIO
        if self.gpio is None:
            raise RuntimeError("RPi.GPIO is not available. This encoder requires a Raspberry Pi GPIO environment.")

        self.channel_a = channel_a
        self.channel_b = channel_b
        self.name = name
        self.direction_inverted = direction_inverted
        self._cleaned = False
        self._ticks = 0
        self._last_state = None
        self._direction = EncoderDirection.STOPPED
        self._last_tick_time = time.monotonic()
        self._last_velocity = 0.0

        self._setup_gpio()

    def _setup_gpio(self) -> None:
        """Configure the encoder pins and edge callbacks."""
        self.gpio.setmode(self.gpio.BCM)
        self.gpio.setup(self.channel_a, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)
        self.gpio.setup(self.channel_b, self.gpio.IN, pull_up_down=self.gpio.PUD_UP)

        # Each A/B edge is one valid quadrature step; the state table rejects invalid jumps.
        self.gpio.add_event_detect(self.channel_a, self.gpio.BOTH, callback=self._on_edge)
        self.gpio.add_event_detect(self.channel_b, self.gpio.BOTH, callback=self._on_edge)

        initial_a = self.gpio.input(self.channel_a)
        initial_b = self.gpio.input(self.channel_b)
        self._last_state = (initial_a, initial_b)

    def _on_edge(self, channel: int) -> None:
        """Handle encoder edge transitions and update direction and tick count."""
        a_state = bool(self.gpio.input(self.channel_a))
        b_state = bool(self.gpio.input(self.channel_b))
        current_state = (a_state, b_state)

        prev_state = self._last_state
        if prev_state is None:
            self._last_state = current_state
            return

        if current_state == prev_state:
            return

        # Only accept actual quadrature steps. Count once per A-channel pulse, while B
        # determines direction; counting every transition would report x4 ticks per cycle.
        valid_forward = {
            (False, False): (False, True),
            (False, True): (True, True),
            (True, True): (True, False),
            (True, False): (False, False),
        }
        valid_reverse = {
            (False, False): (True, False),
            (True, False): (True, True),
            (True, True): (False, True),
            (False, True): (False, False),
        }

        if valid_forward.get(prev_state) == current_state:
            if not prev_state[0] and current_state[0]:
                self._tick_forward()
        elif valid_reverse.get(prev_state) == current_state:
            if not prev_state[0] and current_state[0]:
                self._tick_reverse()

        self._last_state = current_state

    def _tick_forward(self) -> None:
        step = -1 if self.direction_inverted else 1
        self._ticks += step
        self._direction = EncoderDirection.REVERSE if self.direction_inverted else EncoderDirection.FORWARD
        self._update_velocity()

    def _tick_reverse(self) -> None:
        step = 1 if self.direction_inverted else -1
        self._ticks += step
        self._direction = EncoderDirection.FORWARD if self.direction_inverted else EncoderDirection.REVERSE
        self._update_velocity()

    def _update_velocity(self) -> None:
        now = time.monotonic()
        dt = max(now - self._last_tick_time, 1e-6)
        self._last_tick_time = now
        # Approximate velocity in ticks per second.
        self._last_velocity = 1.0 / dt

    def get_ticks(self) -> int:
        return self._ticks

    def get_direction(self) -> EncoderDirection:
        return self._direction

    def reset(self) -> None:
        self._ticks = 0
        self._direction = EncoderDirection.STOPPED
        self._last_velocity = 0.0

    def get_velocity(self) -> float:
        if time.monotonic() - self._last_tick_time > 0.2:
            return 0.0
        return self._last_velocity

    def cleanup(self) -> None:
        if self._cleaned:
            return
        self._cleaned = True
        try:
            self.gpio.remove_event_detect(self.channel_a)
            self.gpio.remove_event_detect(self.channel_b)
        except Exception:  # pragma: no cover - best effort cleanup
            pass

    def __del__(self) -> None:
        try:
            self.cleanup()
        except Exception:  # pragma: no cover - ignore cleanup errors during interpreter shutdown
            pass


if __name__ == "__main__":  # pragma: no cover
    logging.basicConfig(level=logging.INFO)
    left = QuadratureEncoder(LEFT_ENCODER_A, LEFT_ENCODER_B, name="LEFT", direction_inverted=LEFT_ENCODER_INVERTED)
    right = QuadratureEncoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B, name="RIGHT", direction_inverted=RIGHT_ENCODER_INVERTED)

    try:
        for _ in range(10):
            time.sleep(0.5)
            print(f"Left ticks: {left.get_ticks()}  Right ticks: {right.get_ticks()}")
    finally:
        left.cleanup()
        right.cleanup()
