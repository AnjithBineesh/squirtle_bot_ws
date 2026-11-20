import unittest

from squirtle_bot.sensors.imu import HeadingEstimator, MPU6050


class FakeBus:
    def __init__(self, registers):
        self.registers = registers

    def read_byte_data(self, address, register):
        return self.registers[register]


class IMUCalibrationTest(unittest.TestCase):
    def test_read_word_uses_mpu_high_byte_first_order(self):
        imu = MPU6050.__new__(MPU6050)
        imu.address = 0x68
        imu.bus = FakeBus({0x43: 0x01, 0x44: 0x02, 0x45: 0xFF, 0x46: 0xFE})

        self.assertEqual(imu._read_word_2c(0x43), 258)
        self.assertEqual(imu._read_word_2c(0x45), -2)

    def test_calibrate_gyro_bias_uses_stationary_average(self):
        imu = MPU6050.__new__(MPU6050)
        imu.gyro_bias = (0.0, 0.0, 0.0)
        samples = iter([
            (10, 20, 30),
            (30, 40, 50),
            (20, 60, 70),
        ])

        imu.calibrate_gyro_bias(sample_reader=lambda: next(samples), samples=3, delay_seconds=0.0)

        self.assertAlmostEqual(imu.gyro_bias[0], 20.0)
        self.assertAlmostEqual(imu.gyro_bias[1], 40.0)
        self.assertAlmostEqual(imu.gyro_bias[2], 50.0)

    def test_normalize_angle_keeps_heading_in_range(self):
        self.assertAlmostEqual(MPU6050.normalize_heading_deg(450.0), 90.0)
        self.assertAlmostEqual(MPU6050.normalize_heading_deg(-190.0), 170.0)

    def test_heading_fusion_uses_shortest_wraparound_error(self):
        estimator = HeadingEstimator(initial_heading_deg=359.0)

        fused = estimator.fuse_with_encoder(359.0, gyro_z_deg_per_sec=2.0, dt=1.0)

        self.assertAlmostEqual(fused, 0.64, places=1)

    def test_encoder_correction_scales_with_elapsed_time(self):
        estimator = HeadingEstimator(initial_heading_deg=10.0)

        fused = estimator.fuse_with_encoder(20.0, gyro_z_deg_per_sec=0.0, dt=1.0)

        self.assertAlmostEqual(fused, 11.81, places=1)


if __name__ == "__main__":
    unittest.main()
