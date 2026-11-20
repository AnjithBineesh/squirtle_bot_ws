"""Automated tests for RobotServer and RobotNetworkClient communication."""

import time
import unittest
from unittest.mock import MagicMock

from squirtle_bot.hardware.robot_client import RobotNetworkClient
from squirtle_bot.hardware.robot_server import RobotServer


class TestRobotNetwork(unittest.TestCase):
    """Verify UDP communication, command handling, and watchdog behavior."""

    def setUp(self):
        self.mock_driver = MagicMock()
        # Bind to localhost on an ephemeral port
        self.server = RobotServer(
            host="127.0.0.1",
            port=0,
            watchdog_timeout_s=0.2,
            driver=self.mock_driver,
        )
        self.server.start(blocking=False)
        # Retrieve the assigned ephemeral port
        self.port = self.server._sock.getsockname()[1]
        self.client = RobotNetworkClient(robot_ip="127.0.0.1", port=self.port, timeout=0.5)

    def tearDown(self):
        self.client.cleanup()
        self.server.stop()

    def test_ping_and_latency(self):
        ok, latency = self.client.ping()
        self.assertTrue(ok)
        self.assertGreater(latency, 0.0)
        self.assertTrue(self.client.is_connected)

    def test_set_motors_and_stop(self):
        ok = self.client.set_motors(35.0, -35.0)
        self.assertTrue(ok)
        time.sleep(0.05)
        self.mock_driver.set_motors.assert_called_with(35.0, -35.0)

        ok = self.client.stop()
        self.assertTrue(ok)
        time.sleep(0.05)
        self.mock_driver.stop.assert_called()

    def test_watchdog_timeout(self):
        # Send speed command
        self.client.set_motors(40.0, 40.0)
        time.sleep(0.05)
        self.mock_driver.set_motors.assert_called_with(40.0, 40.0)

        # Wait longer than watchdog timeout (0.2s)
        time.sleep(0.3)
        # Server should have called stop due to watchdog
        self.mock_driver.stop.assert_called()


if __name__ == "__main__":
    unittest.main()
