#!/usr/bin/env python3
"""Squirtle Bot — Universal UDP Server with Multi-Protocol WASD & Differential Speed Control.

Runs on Raspberry Pi (192.168.137.26).
Listens on UDP port 5005.

Supports:
- Dual-Protocol: JSON commands and raw serial-style ASCII characters (W/A/S/D/X)
- Raspberry Pi 5 (direct lgpio / gpiozero) and Raspberry Pi 4 (gpiozero)
- USB Serial Bridge auto-detection (Arduino / ESP32 on /dev/ttyUSB* or /dev/ttyACM*)
- Automatic 0.5s watchdog failsafe (brakes motors if connection drops)
- Deadband friction compensation (MIN_PWM = 25%)
- Dual motor direction inversion and swap flags
- Non-blocking UDP command processing with crash-proof error handling
"""

from __future__ import annotations

import argparse
import glob
import json
import logging
import os
import signal
import socket
import sys
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SquirtleServer")

# ==============================================================================
# PIN & CALIBRATION CONFIGURATION (L298N Motor Driver)
# ==============================================================================
try:
    from squirtle_bot.config import (
        ENA as CFG_ENA,
        ENB as CFG_ENB,
        IN1 as CFG_IN1,
        IN2 as CFG_IN2,
        IN3 as CFG_IN3,
        IN4 as CFG_IN4,
        LEFT_MOTOR_INVERTED as CFG_INV_L,
        RIGHT_MOTOR_INVERTED as CFG_INV_R,
        LEFT_MOTOR_SPEED_TRIM_PERCENT as CFG_TRIM_L,
    )
except ImportError:
    CFG_ENA, CFG_IN1, CFG_IN2 = 12, 17, 18
    CFG_ENB, CFG_IN3, CFG_IN4 = 13, 27, 22
    CFG_INV_L, CFG_INV_R = True, True
    CFG_TRIM_L = 2.0

LEFT_ENA  = CFG_ENA   # PWM speed pin (GPIO 12)
LEFT_IN1  = CFG_IN1   # Direction pin 1 (GPIO 17)
LEFT_IN2  = CFG_IN2   # Direction pin 2 (GPIO 18)

RIGHT_ENB = CFG_ENB   # PWM speed pin (GPIO 13)
RIGHT_IN3 = CFG_IN3   # Direction pin 3 (GPIO 27)
RIGHT_IN4 = CFG_IN4   # Direction pin 4 (GPIO 22)

# Polarity & Swap flags
INVERT_LEFT  = CFG_INV_L if CFG_INV_L is not None else True
INVERT_RIGHT = CFG_INV_R if CFG_INV_R is not None else True
SWAP_MOTORS  = False

MIN_PWM = 25.0              # Minimum duty cycle (0-100) to overcome gear friction
DEFAULT_WASD_SPEED = 25.0   # Default speed for manual W/A/S/D steps

UDP_IP   = "0.0.0.0"
UDP_PORT = 5005
DEFAULT_PORT = UDP_PORT
WATCHDOG_TIMEOUT_SEC = 0.50
DEFAULT_WATCHDOG_TIMEOUT_S = WATCHDOG_TIMEOUT_SEC


# ==============================================================================
# USB SERIAL BRIDGE (Arduino / ESP32 Auto-Detection)
# ==============================================================================
class SerialBridge:
    """Auto-detects USB Arduino/ESP32 motor controllers and forwards serial characters."""

    def __init__(self, baud: int = 9600):
        self.ser = None
        try:
            import serial
            ports = glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
            if ports:
                self.ser = serial.Serial(ports[0], baud, timeout=0.1)
                logger.info("Serial bridge connected to %s at %d baud", ports[0], baud)
        except Exception as exc:
            logger.debug("No serial bridge found (%s). Using direct GPIO.", exc)

    def write_char(self, ch: str) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(ch.encode("ascii"))
            except Exception as e:
                logger.debug("Serial write error: %s", e)

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# ==============================================================================
# HARDWARE GPIO ENGINE (gpiozero -> direct lgpio for RPi 5 -> Mock)
# ==============================================================================
class L298NDriver:
    """L298N motor driver engine with Raspberry Pi 5 lgpio & gpiozero support."""

    def __init__(
        self,
        ena: Optional[int] = LEFT_ENA,
        in1: int = LEFT_IN1,
        in2: int = LEFT_IN2,
        enb: Optional[int] = RIGHT_ENB,
        in3: int = RIGHT_IN3,
        in4: int = RIGHT_IN4,
        min_pwm: float = MIN_PWM,
        invert_left: bool = INVERT_LEFT,
        invert_right: bool = INVERT_RIGHT,
        swap_motors: bool = SWAP_MOTORS,
        left_trim: float = CFG_TRIM_L,
    ):
        self.ena = ena
        self.in1 = in1
        self.in2 = in2
        self.enb = enb
        self.in3 = in3
        self.in4 = in4
        self.min_pwm = min_pwm
        self.invert_left = invert_left
        self.invert_right = invert_right
        self.swap_motors = swap_motors
        self.left_trim = left_trim

        self.backend_name = "mock"
        self._backend: Any = None
        self._init_backend()

    def _init_backend(self) -> None:
        # 1. Try gpiozero (standard modern Raspberry Pi library)
        try:
            from gpiozero import DigitalOutputDevice, PWMOutputDevice
            left_in1 = DigitalOutputDevice(self.in1)
            left_in2 = DigitalOutputDevice(self.in2)
            left_pwm = PWMOutputDevice(self.ena) if self.ena is not None else None

            right_in3 = DigitalOutputDevice(self.in3)
            right_in4 = DigitalOutputDevice(self.in4)
            right_pwm = PWMOutputDevice(self.enb) if self.enb is not None else None

            self._backend = {
                "type": "gpiozero",
                "left_in1": left_in1,
                "left_in2": left_in2,
                "left_pwm": left_pwm,
                "right_in3": right_in3,
                "right_in4": right_in4,
                "right_pwm": right_pwm,
            }
            self.backend_name = "gpiozero"
            logger.info("Hardware L298N driver initialized successfully via gpiozero.")
            return
        except Exception as exc:
            logger.debug("gpiozero unavailable (%s). Trying direct lgpio...", exc)

        # 2. Try direct lgpio (official Raspberry Pi 5 kernel library for RP1 chip)
        try:
            import lgpio
            chip_handle = None
            # On Raspberry Pi 5, RP1 GPIOs are on chip 4; try 4 then 0..3
            for c in (4, 0, 1, 2, 3):
                try:
                    chip_handle = lgpio.gpiochip_open(c)
                    break
                except Exception:
                    continue

            if chip_handle is not None:
                pins = [self.in1, self.in2, self.in3, self.in4]
                if self.ena is not None:
                    pins.append(self.ena)
                if self.enb is not None:
                    pins.append(self.enb)

                for pin in pins:
                    lgpio.gpio_claim_output(chip_handle, pin, 0)

                self._backend = {
                    "type": "lgpio",
                    "lib": lgpio,
                    "handle": chip_handle,
                    "claimed": pins,
                }
                self.backend_name = "lgpio"
                logger.info("Hardware L298N driver initialized successfully via direct lgpio (RPi 5).")
                return
        except Exception as exc:
            logger.debug("direct lgpio unavailable (%s). Running in mock/serial mode.", exc)

        # 3. Fallback to simulation/mock mode
        logger.warning("No hardware GPIO backend available. Running in SIMULATION mode.")
        self.backend_name = "mock"

    def _calc_pwm(self, speed: float) -> Tuple[float, int]:
        """Calculate direction (1=fwd, -1=rev, 0=stop) and PWM duty (0.0 to 100.0)."""
        if abs(speed) < 1.0:
            return 0.0, 0
        direction = 1 if speed > 0 else -1
        mag = min(100.0, max(0.0, abs(speed)))
        pwm_val = self.min_pwm + (100.0 - self.min_pwm) * (mag / 100.0)
        pwm_val = min(100.0, max(0.0, pwm_val))
        return pwm_val, direction

    def _stop_pwm_pin(self, lg: Any, h: Any, pin: Optional[int]) -> None:
        if pin is None:
            return
        try:
            lg.tx_pwm(h, pin, 0, 0)
        except Exception:
            pass
        try:
            lg.gpio_write(h, pin, 0)
        except Exception:
            pass

    def set_motors(self, left_speed: float, right_speed: float) -> None:
        """Drive left and right motors (-100.0 to +100.0)."""
        if self.swap_motors:
            left_speed, right_speed = right_speed, left_speed

        if self.invert_left:
            left_speed = -left_speed
        if self.invert_right:
            right_speed = -right_speed

        left_pwm, left_dir = self._calc_pwm(left_speed)
        right_pwm, right_dir = self._calc_pwm(right_speed)

        if self.left_trim != 0.0 and left_pwm > 0:
            left_pwm = min(100.0, left_pwm * (1.0 + self.left_trim / 100.0))

        if self.backend_name == "gpiozero":
            gz = self._backend
            if left_dir == 0:
                gz["left_in1"].off()
                gz["left_in2"].off()
                if gz["left_pwm"]:
                    gz["left_pwm"].value = 0.0
            else:
                if left_dir > 0:
                    gz["left_in1"].on()
                    gz["left_in2"].off()
                else:
                    gz["left_in1"].off()
                    gz["left_in2"].on()
                if gz["left_pwm"]:
                    gz["left_pwm"].value = left_pwm / 100.0

            if right_dir == 0:
                gz["right_in3"].off()
                gz["right_in4"].off()
                if gz["right_pwm"]:
                    gz["right_pwm"].value = 0.0
            else:
                if right_dir > 0:
                    gz["right_in3"].on()
                    gz["right_in4"].off()
                else:
                    gz["right_in3"].off()
                    gz["right_in4"].on()
                if gz["right_pwm"]:
                    gz["right_pwm"].value = right_pwm / 100.0

        elif self.backend_name == "lgpio":
            lg = self._backend["lib"]
            h = self._backend["handle"]
            if left_dir == 0:
                lg.gpio_write(h, self.in1, 0)
                lg.gpio_write(h, self.in2, 0)
                self._stop_pwm_pin(lg, h, self.ena)
            else:
                lg.gpio_write(h, self.in1, 1 if left_dir > 0 else 0)
                lg.gpio_write(h, self.in2, 0 if left_dir > 0 else 1)
                if self.ena is not None:
                    lg.tx_pwm(h, self.ena, 1000, float(left_pwm))

            if right_dir == 0:
                lg.gpio_write(h, self.in3, 0)
                lg.gpio_write(h, self.in4, 0)
                self._stop_pwm_pin(lg, h, self.enb)
            else:
                lg.gpio_write(h, self.in3, 1 if right_dir > 0 else 0)
                lg.gpio_write(h, self.in4, 0 if right_dir > 0 else 1)
                if self.enb is not None:
                    lg.tx_pwm(h, self.enb, 1000, float(right_pwm))

    def stop(self) -> None:
        """Immediately stop and brake both motors."""
        if self.backend_name == "gpiozero":
            gz = self._backend
            gz["left_in1"].off()
            gz["left_in2"].off()
            if gz["left_pwm"]:
                gz["left_pwm"].value = 0.0
            gz["right_in3"].off()
            gz["right_in4"].off()
            if gz["right_pwm"]:
                gz["right_pwm"].value = 0.0

        elif self.backend_name == "lgpio":
            lg = self._backend["lib"]
            h = self._backend["handle"]
            lg.gpio_write(h, self.in1, 0)
            lg.gpio_write(h, self.in2, 0)
            lg.gpio_write(h, self.in3, 0)
            lg.gpio_write(h, self.in4, 0)
            self._stop_pwm_pin(lg, h, self.ena)
            self._stop_pwm_pin(lg, h, self.enb)

    def cleanup(self) -> None:
        """Release all GPIO resources."""
        self.stop()
        if self.backend_name == "gpiozero":
            try:
                gz = self._backend
                for dev in (gz["left_in1"], gz["left_in2"], gz["left_pwm"],
                            gz["right_in3"], gz["right_in4"], gz["right_pwm"]):
                    if dev is not None:
                        dev.close()
            except Exception:
                pass
        elif self.backend_name == "lgpio":
            try:
                lg = self._backend["lib"]
                h = self._backend["handle"]
                for p in self._backend.get("claimed", []):
                    try:
                        lg.gpio_free(h, p)
                    except Exception:
                        pass
                lg.gpiochip_close(h)
            except Exception:
                pass


GPIOEngine = L298NDriver


# ==============================================================================
# UNIVERSAL ROBOT SERVER & WATCHDOG
# ==============================================================================
class RobotServer:
    """Multi-Protocol UDP Server handling JSON, WASD, and serial characters."""

    def __init__(
        self,
        host: str = UDP_IP,
        port: int = DEFAULT_PORT,
        watchdog_timeout_s: float = DEFAULT_WATCHDOG_TIMEOUT_S,
        driver: Optional[Any] = None,
    ) -> None:
        self.host = host
        self.port = port
        self.watchdog_timeout_s = watchdog_timeout_s
        self.driver = driver if driver is not None else L298NDriver()
        self.gpio = self.driver
        self._owns_driver = driver is None

        self.serial_bridge = SerialBridge()
        self.current_motors: List[float] = [0.0, 0.0]
        self._last_cmd_time = 0.0
        self._motors_active = False
        self._running = False
        self._lock = threading.Lock()

        self.sock: Optional[socket.socket] = None
        self._sock: Optional[socket.socket] = None
        self._watchdog_thread: Optional[threading.Thread] = None
        self._server_thread: Optional[threading.Thread] = None

    def start(self, blocking: bool = True) -> None:
        """Start UDP listener and watchdog threads."""
        self._running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.1)
        self._sock = self.sock

        self._last_cmd_time = time.time()
        self._watchdog_thread = threading.Thread(target=self._watchdog, daemon=True)
        self._watchdog_thread.start()

        if blocking:
            self.run()
        else:
            self._server_thread = threading.Thread(target=self.run, daemon=True)
            self._server_thread.start()

    def stop(self) -> None:
        """Stop server, brake motors, and close resources."""
        self._running = False
        self.stop_all()

        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
            self._sock = None

        self.serial_bridge.close()

        if self._owns_driver and self.driver:
            try:
                self.driver.cleanup()
            except Exception:
                pass
            self.driver = None

    def _watchdog(self) -> None:
        """Emergency failsafe: stops motors if no command received for >0.5s."""
        while self._running:
            time.sleep(0.05)
            with self._lock:
                timed_out = (time.time() - self._last_cmd_time) > self.watchdog_timeout_s
                should_brake = self._motors_active and timed_out

            if should_brake:
                logger.warning(
                    "RobotServer Watchdog fired! No command received for >%.2fs; braking robot.",
                    self.watchdog_timeout_s,
                )
                self.stop_all()

    def set_speeds(self, left: float, right: float) -> None:
        """Apply motor speeds with lock and watchdog refresh."""
        with self._lock:
            self._last_cmd_time = time.time()
            self.current_motors = [float(left), float(right)]
            self._motors_active = (abs(left) > 0.1 or abs(right) > 0.1)
            if self.driver:
                try:
                    self.driver.set_motors(left, right)
                except Exception as exc:
                    logger.error("Error setting motor speeds: %s", exc)

    def stop_all(self) -> None:
        """Immediately halt both motors and forward stop character to serial bridge."""
        with self._lock:
            self.current_motors = [0.0, 0.0]
            self._motors_active = False
            if self.driver:
                try:
                    self.driver.stop()
                except Exception as exc:
                    logger.error("Error executing stop: %s", exc)
        try:
            self.serial_bridge.write_char("x")
        except Exception:
            pass

    def handle_wasd(self, key: str, speed: float = DEFAULT_WASD_SPEED) -> None:
        """Handle manual driving key: W=fwd, S=back, A=left, D=right, X/space=stop."""
        k = key.lower()
        self.serial_bridge.write_char(k)
        if k == "w":
            self.set_speeds(speed, speed)
        elif k == "s":
            self.set_speeds(-speed, -speed)
        elif k == "a":
            self.set_speeds(-speed, speed)
        elif k == "d":
            self.set_speeds(speed, -speed)
        elif k in ("x", " "):
            self.stop_all()

    def _run_pulse_test(self, speed: float, duration: float) -> None:
        """Run non-blocking motor pulse test."""
        try:
            logger.info("Motor test pulse: speed=%.1f duration=%.2fs", speed, duration)
            self.set_speeds(speed, speed)
            time.sleep(duration)
        finally:
            self.stop_all()

    def _process_packet(self, data: bytes, addr: Tuple[str, int]) -> Optional[bytes]:
        """Crash-proof multi-protocol packet processing."""
        self._last_cmd_time = time.time()

        # 1. Handle raw single-byte serial characters (w, a, s, d, x, space)
        if len(data) == 1:
            try:
                ch = chr(data[0]).lower()
                self.handle_wasd(ch)
                return json.dumps({"status": "ok", "ack": ch}).encode("utf-8")
            except Exception as e:
                logger.debug("Raw byte parse error: %s", e)
                return None

        # 2. Try JSON decoding
        try:
            msg: Dict[str, Any] = json.loads(data.decode("utf-8"))
        except Exception:
            # Fallback to plain text string
            try:
                text = data.decode("utf-8", errors="ignore").strip().lower()
                if text in ("w", "s", "a", "d", "x", " "):
                    self.handle_wasd(text)
                    return json.dumps({"status": "ok", "ack": text}).encode("utf-8")
            except Exception:
                pass
            return json.dumps({"status": "error", "message": "Invalid JSON/Text"}).encode("utf-8")

        cmd = str(msg.get("cmd", "")).lower()

        # Heartbeat Ping
        if cmd == "ping":
            return json.dumps({
                "status": "pong",
                "t": msg.get("t", 0.0),
                "server_time": time.time(),
                "motors": self.current_motors,
            }).encode("utf-8")

        # Autonomous Differential Speed Control
        elif cmd == "speed":
            left = float(msg.get("left", 0.0))
            right = float(msg.get("right", 0.0))
            self.set_speeds(left, right)
            return json.dumps({
                "status": "ok",
                "ack": "speed",
                "left": left,
                "right": right,
            }).encode("utf-8")

        # Manual WASD Direction Commands
        elif cmd == "forward":
            spd = float(msg.get("speed", msg.get("left", DEFAULT_WASD_SPEED)))
            self.handle_wasd("w", spd)
            return json.dumps({"status": "ok", "ack": "forward"}).encode("utf-8")

        elif cmd == "backward":
            spd = float(msg.get("speed", msg.get("left", DEFAULT_WASD_SPEED)))
            self.handle_wasd("s", spd)
            return json.dumps({"status": "ok", "ack": "backward"}).encode("utf-8")

        elif cmd == "left":
            spd = float(msg.get("speed", msg.get("right", DEFAULT_WASD_SPEED)))
            self.handle_wasd("a", spd)
            return json.dumps({"status": "ok", "ack": "left"}).encode("utf-8")

        elif cmd == "right":
            spd = float(msg.get("speed", msg.get("left", DEFAULT_WASD_SPEED)))
            self.handle_wasd("d", spd)
            return json.dumps({"status": "ok", "ack": "right"}).encode("utf-8")

        # Emergency Stop
        elif cmd in ("stop", "x"):
            self.stop_all()
            return json.dumps({"status": "ok", "ack": "stop"}).encode("utf-8")

        # Motor Test Pulse
        elif cmd == "test_pulse":
            spd = float(msg.get("speed", 30.0))
            dur = min(float(msg.get("duration", 0.3)), 1.5)
            threading.Thread(target=self._run_pulse_test, args=(spd, dur), daemon=True).start()
            return json.dumps({"status": "ok", "ack": "test_pulse", "duration": dur}).encode("utf-8")

        # Status Inquiry
        elif cmd == "status":
            return json.dumps({
                "status": "ok",
                "running": self._running,
                "motors": self.current_motors,
                "watchdog_timeout_s": self.watchdog_timeout_s,
            }).encode("utf-8")

        return json.dumps({"status": "unknown_cmd", "cmd": cmd}).encode("utf-8")

    def run(self) -> None:
        """Main listening loop."""
        print(f"[SQUIRTLEBOT] UDP Server listening on {self.host}:{self.port}...")
        logger.info("[SQUIRTLEBOT] UDP Server listening on %s:%d...", self.host, self.port)

        while self._running:
            try:
                data, addr = self.sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                break
            except Exception as exc:
                if self._running:
                    logger.debug("Socket recv error: %s", exc)
                continue

            try:
                resp = self._process_packet(data, addr)
                if resp and self.sock:
                    try:
                        self.sock.sendto(resp, addr)
                    except Exception:
                        pass
            except Exception as exc:
                logger.error("Unexpected packet handler error: %s", exc)


def run_server(
    port: int = DEFAULT_PORT,
    invert_left: Optional[bool] = None,
    invert_right: Optional[bool] = None,
    swap_motors: Optional[bool] = None,
    min_pwm: float = MIN_PWM,
) -> None:
    """CLI entry point for starting the robot server."""
    parser = argparse.ArgumentParser(description="Squirtle Bot Universal UDP Server")
    parser.add_argument("--port", "-p", type=int, default=port, help=f"UDP Port (default: {DEFAULT_PORT})")
    parser.add_argument("port_pos", nargs="?", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--invert-left", dest="inv_left", action="store_true", default=None, help="Invert left motor direction")
    parser.add_argument("--no-invert-left", dest="inv_left", action="store_false", help="Do not invert left motor direction")
    parser.add_argument("--invert-right", dest="inv_right", action="store_true", default=None, help="Invert right motor direction")
    parser.add_argument("--no-invert-right", dest="inv_right", action="store_false", help="Do not invert right motor direction")
    parser.add_argument("--swap-motors", dest="swap", action="store_true", default=None, help="Swap left and right motor channels")
    parser.add_argument("--min-pwm", type=float, default=min_pwm, help="Minimum deadband PWM duty cycle (default: 25.0)")

    clean_args = [a for a in sys.argv[1:] if a != "server"]
    parsed_args, _ = parser.parse_known_args(clean_args)

    actual_port = parsed_args.port
    if parsed_args.port_pos is not None:
        try:
            actual_port = int(parsed_args.port_pos)
        except ValueError:
            pass

    actual_inv_left = parsed_args.inv_left if parsed_args.inv_left is not None else (invert_left if invert_left is not None else INVERT_LEFT)
    actual_inv_right = parsed_args.inv_right if parsed_args.inv_right is not None else (invert_right if invert_right is not None else INVERT_RIGHT)
    actual_swap = parsed_args.swap if parsed_args.swap is not None else (swap_motors if swap_motors is not None else SWAP_MOTORS)
    actual_min_pwm = parsed_args.min_pwm

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = "127.0.0.1"
    finally:
        s.close()

    print("=" * 62)
    print(" SQUIRTLE BOT — UNIVERSAL NETWORK ROBOT SERVER")
    print("=" * 62)
    print(f" Listening Address: 0.0.0.0:{actual_port} (UDP)")
    print(f" Detected Robot IP: {local_ip}")
    print(f" Connect your laptop interface to IP: {local_ip}  Port: {actual_port}")
    print(" Protocols: JSON speeds/pings + Raw ASCII WASD / Serial")
    print(" Safety Watchdog: Active (0.50s timeout)")
    print(f" Deadband Compensation: Active (MIN_PWM = {actual_min_pwm:.1f}%)")
    print(f" Motor Direction Inversion: Left={actual_inv_left}, Right={actual_inv_right}")
    print(f" Motor Channel Swap: {actual_swap}")
    print(" Press Ctrl+C to stop the server.")
    print("=" * 62)

    custom_driver = L298NDriver(
        invert_left=actual_inv_left,
        invert_right=actual_inv_right,
        swap_motors=actual_swap,
        min_pwm=actual_min_pwm,
    )
    server = RobotServer(host="0.0.0.0", port=actual_port, driver=custom_driver)

    def _signal_handler(sig, frame):
        print("\nStopping robot server and braking motors...")
        server.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    try:
        server.start(blocking=True)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()


if __name__ == "__main__":
    run_server()
