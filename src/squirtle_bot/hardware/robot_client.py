"""Network Robot Client for Squirtle Bot.

Used by remote workstations/laptops to send motor commands and query robot status
over UDP. Implements the same driver interface (set_motors, stop, cleanup)
as L298NMotorDriver so it drops right into VisionNavigator.
"""

from __future__ import annotations

import json
import logging
import socket
import time
from typing import Any, Dict, Optional, Tuple

logger = logging.getLogger(__name__)

DEFAULT_PORT = 5005


class RobotNetworkClient:
    """UDP network client for remote robot teleoperation and vision-guided control."""

    def __init__(
        self,
        robot_ip: str,
        port: int = DEFAULT_PORT,
        timeout: float = 0.5,
    ) -> None:
        self.robot_ip = robot_ip
        self.port = port
        self.timeout = timeout
        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._last_latency_ms: float = -1.0
        self._open_socket()

    def _open_socket(self) -> None:
        if self._sock is None:
            self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._sock.settimeout(self.timeout)

    def ping(self) -> Tuple[bool, float]:
        """Ping the robot and measure round-trip latency in milliseconds.

        Returns:
            (success: bool, latency_ms: float)
        """
        t0 = time.time()
        payload = json.dumps({"cmd": "ping", "t": t0}).encode("utf-8")
        try:
            self._sock.sendto(payload, (self.robot_ip, self.port))
            data, _ = self._sock.recvfrom(2048)
            resp = json.loads(data.decode("utf-8"))
            if resp.get("status") == "pong":
                latency_ms = (time.time() - t0) * 1000.0
                self._last_latency_ms = latency_ms
                self._connected = True
                return True, latency_ms
        except Exception:
            self._connected = False
            self._last_latency_ms = -1.0
            return False, -1.0
        return False, -1.0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def last_latency_ms(self) -> float:
        return self._last_latency_ms

    def set_motors(self, left_speed: float, right_speed: float) -> bool:
        """Send normalized speeds (-100 to +100) to the robot."""
        payload = json.dumps({
            "cmd": "speed",
            "left": round(float(left_speed), 2),
            "right": round(float(right_speed), 2),
        }).encode("utf-8")
        try:
            self._sock.sendto(payload, (self.robot_ip, self.port))
            return True
        except Exception as exc:
            logger.debug("Failed sending motor speeds to %s: %s", self.robot_ip, exc)
            self._connected = False
            return False

    def stop(self) -> bool:
        """Send emergency stop to brake robot motors immediately."""
        payload = json.dumps({"cmd": "stop"}).encode("utf-8")
        try:
            self._sock.sendto(payload, (self.robot_ip, self.port))
            return True
        except Exception:
            return False

    def test_pulse(self, speed: float = 30.0, duration: float = 0.3) -> bool:
        """Trigger a brief safe motor pulse on the robot to verify wheel drive."""
        payload = json.dumps({
            "cmd": "test_pulse",
            "speed": float(speed),
            "duration": float(duration),
        }).encode("utf-8")
        try:
            self._sock.sendto(payload, (self.robot_ip, self.port))
            return True
        except Exception:
            return False

    def cleanup(self) -> None:
        """Stop motors and close client socket."""
        self.stop()
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
        self._connected = False
