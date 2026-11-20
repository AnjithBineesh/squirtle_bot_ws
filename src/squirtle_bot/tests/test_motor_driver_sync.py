import unittest
from unittest.mock import patch

from squirtle_bot.hardware.motor_driver import L298NMotorDriver


class FakePWM:
    def __init__(self, pin, events):
        self.pin = pin
        self.events = events

    def start(self, duty_cycle):
        self.events.append(("start", self.pin, duty_cycle))

    def ChangeDutyCycle(self, duty_cycle):
        self.events.append(("duty", self.pin, duty_cycle))

    def stop(self):
        self.events.append(("stop", self.pin))

    def __del__(self):
        self.events.append(("delete", self.pin))


class FakeGPIO:
    BCM = 11
    OUT = 1
    LOW = 0
    HIGH = 1

    def __init__(self):
        self.events = []
        self.mode = None

    def getmode(self):
        return self.mode

    def setmode(self, mode):
        self.mode = mode

    def setwarnings(self, enabled):
        pass

    def setup(self, pin, mode, initial=None):
        pass

    def PWM(self, pin, frequency):
        return FakePWM(pin, self.events)

    def output(self, pin, value):
        self.events.append(("direction", pin, value))

    def cleanup(self):
        self.events.append(("cleanup",))


class MotorDriverSyncTest(unittest.TestCase):
    def test_both_directions_are_set_before_either_pwm_starts(self):
        gpio = FakeGPIO()
        driver = L298NMotorDriver(gpio_module=gpio)

        gpio.events.clear()
        driver.set_motors(30, 30)

        direction_indices = [i for i, event in enumerate(gpio.events) if event[0] == "direction"]
        pwm_indices = [i for i, event in enumerate(gpio.events) if event[0] == "duty"]
        self.assertEqual(len(direction_indices), 4)
        self.assertEqual(len(pwm_indices), 2)
        self.assertLess(max(direction_indices), min(pwm_indices))
        self.assertEqual(gpio.events[pwm_indices[0]][2], gpio.events[pwm_indices[1]][2])

    def test_pwm_stops_before_gpio_cleanup_and_cleanup_is_idempotent(self):
        gpio = FakeGPIO()
        driver = L298NMotorDriver(gpio_module=gpio)

        gpio.events.clear()
        driver.cleanup()
        driver.stop()
        driver.cleanup()

        pwm_stop_indices = [i for i, event in enumerate(gpio.events) if event[0] == "stop"]
        cleanup_indices = [i for i, event in enumerate(gpio.events) if event[0] == "cleanup"]
        self.assertEqual(len(pwm_stop_indices), 2)
        self.assertEqual(len(cleanup_indices), 1)
        self.assertLess(max(pwm_stop_indices), cleanup_indices[0])

        delete_indices = [i for i, event in enumerate(gpio.events) if event[0] == "delete"]
        self.assertEqual(len(delete_indices), 2)
        self.assertLess(max(delete_indices), cleanup_indices[0])

    def test_stop_brakes_both_motors_then_disables_pwm(self):
        gpio = FakeGPIO()
        driver = L298NMotorDriver(gpio_module=gpio)

        gpio.events.clear()
        with patch("squirtle_bot.hardware.motor_driver.time.sleep") as sleep:
            driver.stop()

        duty_events = [event for event in gpio.events if event[0] == "duty"]
        self.assertEqual([event[2] for event in duty_events], [100, 100, 0, 0])
        sleep.assert_called_once()

    def test_motor_driver_preserves_fractional_pwm_duty(self):
        gpio = FakeGPIO()
        driver = L298NMotorDriver(gpio_module=gpio)

        gpio.events.clear()
        driver.set_motors(30.6, 30.0)

        duty_events = [event for event in gpio.events if event[0] == "duty"]
        self.assertEqual([event[2] for event in duty_events], [30.6, 30.0])


if __name__ == "__main__":
    unittest.main()