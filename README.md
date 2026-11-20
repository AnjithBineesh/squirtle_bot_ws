# Squirtle Bot 🐢🤖

Squirtle Bot is an autonomous differential-drive robot built on **Raspberry Pi 5**. The system integrates low-level GPIO motor control, optical quadrature wheel encoders, 6-axis MPU6050 IMU heading fusion, odometry dead-reckoning, closed-loop waypoint navigation, and overhead ArUco visual tracking.

---

## Hardware Architecture & Pinout

### BCM GPIO Pin Mapping

| Component | Pin Function | BCM GPIO | Physical Header Pin |
|---|---|---|---|
| **L298N Left Motor** | ENA (PWM) | GPIO 12 | Pin 32 |
| | IN1 (Direction) | GPIO 17 | Pin 11 |
| | IN2 (Direction) | GPIO 18 | Pin 12 |
| **L298N Right Motor** | ENB (PWM) | GPIO 13 | Pin 33 |
| | IN3 (Direction) | GPIO 27 | Pin 13 |
| | IN4 (Direction) | GPIO 22 | Pin 15 |
| **Left Wheel Encoder** | Channel A | GPIO 20 | Pin 38 |
| | Channel B | GPIO 21 | Pin 40 |
| **Right Wheel Encoder** | Channel A | GPIO 26 | Pin 37 |
| | Channel B | GPIO 19 | Pin 35 |
| **MPU6050 IMU (I2C-1)** | SDA | GPIO 2 | Pin 3 |
| | SCL | GPIO 3 | Pin 5 |

### Physical Parameters (`config.py`)

- **Wheel Diameter**: 6.0 cm (`0.06 m`)
- **Wheel Base**: 14.0 cm physical, calibrated effective turning base: `0.285 m`
- **Left Encoder Resolution**: ~149.0 ticks / revolution
- **Right Encoder Resolution**: ~151.4 ticks / revolution
- **Odometry Distance Scale**: `1.21` (ground-truth surface traction compensation)
- **Left Motor Speed Trim**: `+2.0%` (compensates for physical gearbox friction delta)

---

## Software Subsystems

### 1. Unified Launcher & CLI (`src/main.py` / `squirtle_bot.main`)
Provides an interactive menu for operators and full command-line dispatch for automated workflows.

```bash
cd /home/squirtle_bot/squirtle_bot_ws
python3 src/main.py --help
```

Available subcommands:
- `python3 src/main.py menu` (or run without arguments in a terminal)
- `python3 src/main.py motor` — Interactive low-speed motor verification
- `python3 src/main.py diagnostic` — Live IMU & Encoder terminal dashboard
- `python3 src/main.py encoders` — Real-time wheel tick counting
- `python3 src/main.py calibrate` — Wheel tick calibration tool
- `python3 src/main.py odometry` — Interactive line test with live wheel-speed sync
- `python3 src/main.py distance` — Precise straight-line distance drive
- `python3 src/main.py turn` — Calibrated in-place turn angle test
- `python3 src/main.py square` — 4-corner square path closed loop
- `python3 src/main.py waypoint` — Target coordinate navigation
- `python3 src/main.py server` — Start UDP Robot Server (listens for remote navigation commands)
- `python3 src/main.py test` — Run all automated unit tests

---

### 2. Live Sensor Diagnostic (`src/squirtle_bot/tests/sensor_diagnostic.py`)
Displays a live terminal dashboard of left/right wheel ticks, cumulative meters traveled, wheel rotation direction, instantaneous gyro angular rates ($G_z$), and integrated yaw heading.

```bash
cd /home/squirtle_bot/squirtle_bot_ws
PYTHONPATH=src python3 -m squirtle_bot.tests.sensor_diagnostic 30
```

---

### 3. Closed-Loop Odometry & Navigation
- **Wheel-Speed Synchronizer**: Runs during straight drives to dynamically balance left and right PWM based on real-time encoder pulse rates.
- **Turn Angle Brake Compensator**: Automatically accounts for differential momentum so commanded angles (e.g. 90°) stop accurately on the target heading.
- **Waypoint Navigation**: Calculates shortest-rotation angles and segmented distances to guide the robot across arbitrary 2D target waypoints.

```bash
# Calibrated distance test
PYTHONPATH=src python3 -m squirtle_bot.tests.distance_drive_test

# Turn angle test
PYTHONPATH=src python3 -m squirtle_bot.tests.turn_angle_test

# 4-corner square path benchmark
PYTHONPATH=src python3 -m squirtle_bot.tests.square_path_test

# Autonomous waypoint navigation
PYTHONPATH=src python3 -m squirtle_bot.tests.waypoint_navigation_test
```

---

### 4. Network Robot Server (`robot_server.py` / `src/squirtle_bot/hardware/robot_server.py`)
Runs on the Raspberry Pi listening on UDP port `5005` for motion and telemetry commands from a remote workstation/laptop running overhead vision navigation:
- **Low-Latency UDP Commands**: Supports continuous differential speeds (`{"cmd": "speed", "left": L, "right": R}`), heartbeat ping (`{"cmd": "ping", "t": T}`), emergency stop (`{"cmd": "stop"}`), and test pulses (`{"cmd": "test_pulse"}`).
- **Failsafe Watchdog**: Automatically stops both motors if no packet is received for >0.5s.
- **Deadband Compensation**: Overcomes DC motor friction with configurable minimum PWM (`MIN_PWM = 25%`).

```bash
cd /home/squirtle_bot/squirtle_bot_ws
python3 src/main.py server
# or directly:
python3 robot_server.py
```

---

## Automated Unit Testing

The repository includes a comprehensive 40-test automated suite covering:
- Motor driver GPIO synchronization and PWM lifecycle cleanup
- Quadrature encoder valid state transitions and noise rejection
- MPU6050 16-bit register unpacking, zero-rate bias calibration, and heading fusion
- Odometry wheel-distance scaling and differential kinematics
- Closed-loop speed balancing and turn braking compensations
- Waypoint steering, distance chunking, and unified subsystem cleanup
- Vision-guided motion planning and heading error normalization

Run the entire test suite:
```bash
cd /home/squirtle_bot/squirtle_bot_ws
python3 src/main.py test
```
Or via standard unittest discovery:
```bash
PYTHONPATH=src python3 -m unittest discover -s src -p "test_*.py"
```

---

## Installation & Setup

On Raspberry Pi OS (Debian 12 / Ubuntu 24.04 on Raspberry Pi 5):

```bash
# System GPIO & I2C packages
sudo apt update
sudo apt install -y python3-rpi-lgpio python3-smbus

# Optional: Vision & GUI dependencies
sudo apt install -y python3-opencv
pip install --break-system-packages -r requirements.txt
```

---

## Safety Guidelines

1. **Common Ground**: Ensure the Raspberry Pi ground and the L298N motor power supply ground are connected together.
2. **Dedicated Motor Power**: Never power DC motors directly from the Raspberry Pi 5V or 3.3V pins.
3. **Elevated Wheel Check**: Always verify wheel direction and safety stops with the wheels lifted off the ground before free-running on a surface.
4. **Clean Exit**: All scripts include graceful signal handlers (`SIGINT`) that safely stop motors and release GPIO pins upon exit.
