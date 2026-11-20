"""Hardware abstraction layer for Squirtle Bot."""

from .encoder import QuadratureEncoder
from .motor_driver import L298NMotorDriver

__all__ = ["L298NMotorDriver", "QuadratureEncoder"]
