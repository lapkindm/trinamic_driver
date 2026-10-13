import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from trinamic_driver import protocol as P  # noqa: E402
from trinamic_driver.sixpack import SixPack  # noqa: E402


class Recorder:
    def __init__(self):
        self.sent = b""

    def write(self, data):
        self.sent += data

    def read(self, size=1):
        return b""

    def reset_input_buffer(self):
        pass


def frame(fn):
    rec = Recorder()
    fn(SixPack(rec, address=0))
    return rec.sent


class ManualExamples(unittest.TestCase):
    """Frames from the manual's worked examples (bytes as printed there)."""

    def test_motor_parameters_mechanical_reference(self):
        # 5.5.3.2: Poslimit = 10000, NullLeft, MechRef
        self.assertEqual(frame(lambda s: s.set_motor_parameters(0, 10000, 0x0410)),
                         bytes([0x00, 0x15, 0x00, 0x10, 0x27, 0x00, 0x00, 0x10, 0x04]))

    def test_motor_parameters_bit_layout(self):
        # 6.2.2.1: poslimit 400, P5 = 200 (TestNull|StopNull|FilterSwitch), P6 = 90
        sent = frame(lambda s: s.set_motor_parameters(0, 400, 200 | 90 << 8))
        self.assertEqual(sent[2:], bytes([0x00, 0x90, 0x01, 0x00, 0x00, 0xC8, 0x5A]))

    def test_start_velocity(self):
        # 5.6.2.1, motor 3 (index 2); the bytes encode vmin=10, vstart=10, div=2
        self.assertEqual(frame(lambda s: s.set_start_velocity(2, 10, 10, 2)),
                         bytes([0x00, 0x13, 0x02, 0x0A, 0x00, 0x0A, 0x00, 0x02, 0x00]))

    def test_amax_vmax(self):
        # 5.6.2.2: AMax = 200, VMax = 400 for motor 4 (index 3)
        self.assertEqual(frame(lambda s: s.set_amax_vmax(3, 200, 400)),
                         bytes([0x00, 0x14, 0x03, 0xC8, 0x00, 0x90, 0x01, 0x00, 0x00]))

    def test_reference_search(self):
        # 5.5.3.2
        self.assertEqual(frame(lambda s: s.reference_search(0)), bytes([0x00, 0x22] + [0] * 7))

    def test_set_target_lsb_first(self):
        # 6.2.2.4: motor 1 to 1000 ($E8 $03)
        self.assertEqual(frame(lambda s: s.set_target(0, 1000))[2:7], bytes([0x00, 0xE8, 0x03, 0x00, 0x00]))

    def test_frequencies(self):
        # defaults in 6.2.1.8: 26 kHz, 254 Hz, 102 Hz
        self.assertAlmostEqual(P.step_frequency(511, 2, 5), 25991, delta=1)
        self.assertAlmostEqual(P.step_frequency(5, 2, 5), 254, delta=1)
        self.assertAlmostEqual(P.step_frequency(4, 3, 5), 102, delta=1)

    def test_peak_current_default(self):
        self.assertEqual(P.peak_current_value(50), 128)  # manual default "Peak current=128"

    def test_debounce_default(self):
        self.assertEqual(P.debounce_mask(22), 0x0FFF)  # "De-bounce mask=$0FFF ... 22 ms"
        self.assertEqual(P.debounce_mask(0), 0x0001)
        self.assertEqual(P.debounce_mask(30), 0xFFFF)


class Conversions(unittest.TestCase):
    def test_roundtrip(self):
        for div in range(4):
            v = P.velocity_value(P.step_frequency(300, div, 5), div, 5)
            self.assertAlmostEqual(v, 300)
            a = P.amax_value(P.acceleration_of(200, div, 5), div, 5)
            self.assertAlmostEqual(a, 200)

    def test_choose_div(self):
        self.assertEqual(P.choose_div(5400, 5), 3)    # m1: v = 212 at div 3
        self.assertEqual(P.choose_div(12800, 5), 3)   # a1: v = 503
        self.assertEqual(P.choose_div(13000, 5), 2)
        with self.assertRaises(P.ProtocolError):
            P.choose_div(200e3, 5)

    def test_current_index(self):
        self.assertEqual([P.current_index(x) for x in (100, 75, 50, 0)], [0, 1, 2, 8])
        self.assertEqual(P.current_index(40), 3)  # nearest: 38 %

    def test_home_method(self):
        self.assertEqual(P.split_home_method(0x60D0), (0xD0, 0x60))
        self.assertEqual(P.describe_home_method(0x60D0),
                         "NullLeft|StopNull|FilterSwitch|StopNoRef|NullPositive")
        with self.assertRaises(P.ProtocolError):
            P.split_home_method(0x0001)

    def test_ranges(self):
        with self.assertRaises(P.ProtocolError):
            frame(lambda s: s.start_ramp(6, 0))
        with self.assertRaises(P.ProtocolError):
            frame(lambda s: s.set_amax_vmax(0, 1, 512))
        with self.assertRaises(P.ProtocolError):
            frame(lambda s: s.raw(0xCC))  # hardware reset is refused

    def test_decode(self):
        r = P.Reply(0x20, bytes([2]) + (-1234).to_bytes(4, "little", signed=True) + bytes([5, 1]))
        d = P.decode_position(r)
        self.assertEqual((d.motor, d.position, d.action, d.stop), (2, -1234, 5, True))
        info = P.decode_unit_info(P.Reply(0x43, bytes([166, 1, 41]) + (5816).to_bytes(2, "little") + b"\0\0"))
        self.assertEqual((info.firmware_str, info.reset, info.temperature, info.serial), ("1.66", True, 41, 5816))


if __name__ == "__main__":
    unittest.main()
