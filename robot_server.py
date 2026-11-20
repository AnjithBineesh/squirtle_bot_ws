#!/usr/bin/env python3
"""Squirtle Bot — UDP Motor Controller & Telemetry Server entrypoint."""

import os
import sys

# Ensure src/ is on PYTHONPATH
workspace_dir = os.path.dirname(os.path.abspath(__file__))
src_dir = os.path.join(workspace_dir, "src")
if src_dir not in sys.path:
    sys.path.insert(0, src_dir)

from squirtle_bot.hardware.robot_server import DEFAULT_PORT, RobotServer, run_server

if __name__ == "__main__":
    run_server()

