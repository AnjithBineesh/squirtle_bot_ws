"""Configuration values for the Raspberry Pi GPIO and motor system.

These are the current BCM GPIO assignments required by the design.
"""

# L298N motor-driver GPIO mapping
ENA = 12
IN1 = 17
IN2 = 18

IN3 = 27
IN4 = 22
ENB = 13

# Motor direction configuration for physical orientation corrections.
# Set to True if the motor rotation direction is opposite to the software expectation.
# The motors were observed to run opposite to the intended software direction, so
# invert both sides for the initial corrected test behaviour.
LEFT_MOTOR_INVERTED = True
RIGHT_MOTOR_INVERTED = True

# PWM settings
MOTOR_PWM_FREQUENCY_HZ = 1000
MOTOR_SPEED_LIMIT = 100
INITIAL_TEST_SPEED = 30
MOTOR_BRAKE_TIME_S = 0.1
LEFT_MOTOR_SPEED_TRIM_PERCENT = 2.0

# Right encoder GPIO mapping
RIGHT_ENCODER_A = 26
RIGHT_ENCODER_B = 19
RIGHT_ENCODER_INVERTED = True

# Left encoder GPIO mapping
LEFT_ENCODER_A = 20
LEFT_ENCODER_B = 21
LEFT_ENCODER_INVERTED = False

# I2C GPIO mapping
I2C_SDA = 2
I2C_SCL = 3
I2C_BUS = 1
MPU6050_ADDRESS = 0x68
MPU6050_SDA_GPIO = 2
MPU6050_SCL_GPIO = 3

# Robot geometry and odometry placeholders.
# These values are measured on the physical robot before odometry is used for navigation.
WHEEL_DIAMETER = 0.06  # 6 cm in meters
WHEEL_BASE = 0.14      # 14 cm in meters
ODOMETRY_WHEEL_BASE = 0.285
LEFT_ENCODER_TICKS_PER_REV = 149
RIGHT_ENCODER_TICKS_PER_REV = 151.44
ENCODER_TICKS_PER_REV = LEFT_ENCODER_TICKS_PER_REV
ODOMETRY_DISTANCE_SCALE = 1.21

# Safety / testing defaults
MOTOR_TEST_SPEED = 30
MAX_SAFE_PWM = 100
