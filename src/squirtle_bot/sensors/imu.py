"""MPU6050 IMU wrapper using the Raspberry Pi I2C bus."""

from __future__ import annotations

import math
import time
from typing import Callable

from squirtle_bot.config import I2C_BUS, MPU6050_ADDRESS

try:
    import smbus2 as smbus_backend
except ImportError:  # pragma: no cover
    try:
        import smbus as smbus_backend
    except ImportError:  # pragma: no cover
        smbus_backend = None


class MPU6050:
    """Read gyro and accelerometer data from an MPU6050 over I2C."""

    GYRO_SENSITIVITY_LSB_DPS = 131.0

    def __init__(self, bus_number: int = I2C_BUS, address: int = MPU6050_ADDRESS) -> None:
        self.bus = None
        self.address = address
        self.gyro_bias = (0.0, 0.0, 0.0)
        self.yaw_deg = 0.0
        self.last_time = time.monotonic()

        if smbus_backend is None:
            return

        try:
            self.bus = smbus_backend.SMBus(bus_number)
            self._wake_up()
        except OSError as exc:  # pragma: no cover - hardware access is environment-specific.
            raise RuntimeError(
                "Could not open the MPU6050 over I2C. Check that the Raspberry Pi I2C bus is enabled and the device is connected."
            ) from exc

    @staticmethod
    def normalize_heading_deg(angle_deg: float) -> float:
        """Wrap a heading to the range [-180, 180)."""
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def _wake_up(self) -> None:
        if self.bus is None:
            raise RuntimeError("MPU6050 bus is unavailable.")
        self.bus.write_byte_data(self.address, 0x6B, 0x00)

    def _read_word_2c(self, reg: int) -> int:
        if self.bus is None:
            raise RuntimeError("MPU6050 bus is unavailable.")
        high = self.bus.read_byte_data(self.address, reg)
        low = self.bus.read_byte_data(self.address, reg + 1)
        value = (high << 8) | low
        if value >= 0x8000:
            value -= 0x10000
        return value

    def read_gyro_raw(self) -> tuple[float, float, float]:
        x = self._read_word_2c(0x43)
        y = self._read_word_2c(0x45)
        z = self._read_word_2c(0x47)
        return x, y, z

    def read_accel_raw(self) -> tuple[float, float, float]:
        x = self._read_word_2c(0x3B)
        y = self._read_word_2c(0x3D)
        z = self._read_word_2c(0x3F)
        return x, y, z

    def read_gyro_calibrated(self) -> tuple[float, float, float]:
        x, y, z = self.read_gyro_raw()
        bias_x, bias_y, bias_z = self.gyro_bias
        return (x - bias_x) / self.GYRO_SENSITIVITY_LSB_DPS, (
            y - bias_y
        ) / self.GYRO_SENSITIVITY_LSB_DPS, (z - bias_z) / self.GYRO_SENSITIVITY_LSB_DPS

    def gyro_deg_per_sec(self) -> tuple[float, float, float]:
        return self.read_gyro_calibrated()

    def calibrate_gyro_bias(
        self,
        sample_reader: Callable[[], tuple[float, float, float]] | None = None,
        samples: int = 200,
        delay_seconds: float = 0.01,
    ) -> tuple[float, float, float]:
        """Average a stationary sample set to remove the gyro zero-rate bias."""
        if samples <= 0:
            raise ValueError("samples must be greater than zero")
        if sample_reader is None:
            sample_reader = self.read_gyro_raw

        bias_x = 0.0
        bias_y = 0.0
        bias_z = 0.0
        for _ in range(samples):
            x, y, z = sample_reader()
            bias_x += x
            bias_y += y
            bias_z += z
            if delay_seconds > 0.0:
                time.sleep(delay_seconds)

        self.gyro_bias = (
            bias_x / samples,
            bias_y / samples,
            bias_z / samples,
        )
        return self.gyro_bias

    def update_yaw(self) -> float:
        now = time.monotonic()
        dt = max((now - self.last_time), 0.001)
        self.last_time = now
        _, _, gyro_z = self.gyro_deg_per_sec()
        self.yaw_deg = self.normalize_heading_deg(self.yaw_deg + gyro_z * dt)
        return self.yaw_deg

    def reset_yaw(self, heading_deg: float = 0.0) -> float:
        self.yaw_deg = self.normalize_heading_deg(heading_deg)
        self.last_time = time.monotonic()
        return self.yaw_deg

    def close(self) -> None:
        if self.bus is not None:
            self.bus.close()

    @staticmethod
    def fuse_heading(encoder_heading_deg: float, imu_heading_deg: float, imu_weight: float = 0.85) -> float:
        """Blend gyro-integrated heading with encoder heading to reduce drift."""
        if not 0.0 <= imu_weight <= 1.0:
            raise ValueError("imu_weight must be between 0.0 and 1.0")
        delta = MPU6050.normalize_heading_deg(imu_heading_deg - encoder_heading_deg)
        fused = encoder_heading_deg + delta * imu_weight
        return MPU6050.normalize_heading_deg(fused)


class HeadingEstimator:
    """Simple yaw estimator that integrates gyro rate and can be anchored to encoder heading."""

    ENCODER_CORRECTION_TIME_CONSTANT_S = 5.0

    def __init__(self, initial_heading_deg: float = 0.0) -> None:
        self.heading_deg = MPU6050.normalize_heading_deg(initial_heading_deg)
        self.last_time = time.monotonic()

    def update_gyro(self, gyro_z_deg_per_sec: float, dt: float | None = None) -> float:
        now = time.monotonic()
        if dt is None:
            dt = max(now - self.last_time, 0.001)
        self.last_time = now
        self.heading_deg = MPU6050.normalize_heading_deg(self.heading_deg + gyro_z_deg_per_sec * dt)
        return self.heading_deg

    def reset(self, heading_deg: float = 0.0) -> float:
        self.heading_deg = MPU6050.normalize_heading_deg(heading_deg)
        self.last_time = time.monotonic()
        return self.heading_deg

    def fuse_with_encoder(self, encoder_heading_deg: float, gyro_z_deg_per_sec: float, dt: float | None = None) -> float:
        self.update_gyro(gyro_z_deg_per_sec, dt=dt)
        elapsed = max(dt if dt is not None else 0.001, 0.001)
        gyro_weight = math.exp(-elapsed / self.ENCODER_CORRECTION_TIME_CONSTANT_S)
        encoder_error = MPU6050.normalize_heading_deg(encoder_heading_deg - self.heading_deg)
        self.heading_deg = MPU6050.normalize_heading_deg(
            self.heading_deg + encoder_error * (1.0 - gyro_weight)
        )
        return self.heading_deg


if __name__ == "__main__":  # pragma: no cover
    if smbus_backend is None:
        raise SystemExit(
            "No I2C Python module found. Install python3-smbus or install smbus2 in the environment used to run this module."
        )

    try:
        imu = MPU6050()
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    try:
        print("Keep the robot completely still; calibrating gyro bias for 2 seconds.")
        imu.calibrate_gyro_bias(samples=200, delay_seconds=0.01)
        print(f"Gyro bias raw LSB: {imu.gyro_bias}")
        while True:
            pitch, roll, yaw = imu.gyro_deg_per_sec()
            print(f"gyro: {pitch:.2f}, {roll:.2f}, {yaw:.2f} deg/s")
            time.sleep(0.2)
    except KeyboardInterrupt:
        imu.close()
