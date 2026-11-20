import unittest

from squirtle_bot.hardware.encoder import QuadratureEncoder


class FakeGPIO:
    BCM = 1
    IN = 0
    PUD_UP = 1
    BOTH = 2

    def __init__(self):
        self._pins = {20: 0, 21: 0, 26: 0, 19: 0}
        self._callbacks = {}
        self.removed_channels = []

    def setmode(self, mode):
        self.mode = mode

    def setup(self, channel, direction, pull_up_down=None):
        pass

    def add_event_detect(self, channel, edge, callback=None):
        self._callbacks[channel] = callback

    def remove_event_detect(self, channel):
        self.removed_channels.append(channel)
        self._callbacks.pop(channel, None)

    def input(self, channel):
        return self._pins[channel]

    def set_state(self, a, b):
        previous = (self._pins[20], self._pins[21])
        self._pins[20] = a
        self._pins[21] = b
        changed_channel = 20 if previous[0] != a else 21 if previous[1] != b else None
        if changed_channel is not None:
            callback = self._callbacks.get(changed_channel)
            if callback is not None:
                callback(changed_channel)


class EncoderNoiseTest(unittest.TestCase):
    def test_full_quadrature_cycle_counts_one_tick(self):
        gpio = FakeGPIO()
        encoder = QuadratureEncoder(20, 21, gpio_module=gpio, name="LEFT")

        for state in ((0, 1), (1, 1), (1, 0), (0, 0)):
            gpio.set_state(*state)
        self.assertEqual(encoder.get_ticks(), 1)

    def test_reverse_cycle_counts_one_negative_tick(self):
        gpio = FakeGPIO()
        encoder = QuadratureEncoder(20, 21, gpio_module=gpio, name="LEFT")

        for state in ((1, 0), (1, 1), (0, 1), (0, 0)):
            gpio.set_state(*state)
        self.assertEqual(encoder.get_ticks(), -1)

    def test_invalid_jumped_state_is_ignored(self):
        gpio = FakeGPIO()
        encoder = QuadratureEncoder(20, 21, gpio_module=gpio, name="LEFT")

        gpio.set_state(0, 1)
        self.assertEqual(encoder.get_ticks(), 0)

        gpio.set_state(1, 1)
        self.assertEqual(encoder.get_ticks(), 1)

        gpio.set_state(0, 0)
        self.assertEqual(encoder.get_ticks(), 1)

    def test_cleanup_is_idempotent(self):
        gpio = FakeGPIO()
        encoder = QuadratureEncoder(20, 21, gpio_module=gpio, name="LEFT")

        encoder.cleanup()
        encoder.cleanup()

        self.assertCountEqual(gpio.removed_channels, [20, 21])


if __name__ == "__main__":
    unittest.main()
