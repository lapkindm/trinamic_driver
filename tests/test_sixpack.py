import os
import sys
import time
import unittest

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fake_sixpack import FakeSixPack, FakeTransport  # noqa: E402
from trinamic_driver import Action, SixPack, SixPackError, SixPackTimeout  # noqa: E402


def make(reply_length_sim=8, reply_length=None, **kw):
    sim = FakeSixPack(reply_length=reply_length_sim, **kw)
    return sim, SixPack(FakeTransport(sim), address=0, timeout=0.3, reply_length=reply_length)


def wait_inactive(pack, motor, timeout=3.0):
    deadline = time.monotonic() + timeout
    while pack.position(motor).action != Action.INACTIVE:
        if time.monotonic() > deadline:
            raise AssertionError("motor did not stop")
        time.sleep(0.01)


class ReplyFormat(unittest.TestCase):
    def test_detects_eight_byte_replies(self):
        sim, pack = make(8)
        self.assertEqual(pack.unit_info().firmware_str, "1.66")
        self.assertEqual(pack.reply_length, 8)

    def test_detects_nine_byte_replies(self):
        sim, pack = make(9)
        self.assertEqual(pack.unit_info().serial, 5816)
        self.assertEqual(pack.reply_length, 9)

    def test_forced_format(self):
        sim, pack = make(9, reply_length=9)
        self.assertEqual(pack.position(1).motor, 1)

    def test_wrong_forced_format_fails(self):
        sim, pack = make(8, reply_length=9)
        with self.assertRaises(SixPackTimeout):
            pack.unit_info()

    def test_timeout(self):
        sim, pack = make(8)
        pack.transport.drop_replies = True
        with self.assertRaises(SixPackTimeout):
            pack.position(0)

    def test_wrong_unit_address(self):
        sim = FakeSixPack(address=3)
        pack = SixPack(FakeTransport(sim), address=0, timeout=0.2)
        with self.assertRaises(SixPackTimeout):
            pack.unit_info()

    def test_garbage_reply(self):
        sim, pack = make(8)
        pack.transport.write = lambda data: setattr(sim, "out", b"\x99" * 9)
        with self.assertRaises(SixPackError):
            pack.unit_info()


class Retries(unittest.TestCase):
    """spec's "Trinamic motor became unresponsive" was cured by restarting
    spec: one lost or garbled reply must not stop a measurement."""

    def test_lost_reply_is_asked_again(self):
        sim, pack = make(8)
        sim.drop_next = 1
        self.assertEqual(pack.position(1).motor, 1)
        self.assertEqual(pack.retried, 1)

    def test_garbled_reply_is_asked_again(self):
        sim, pack = make(8)
        write = pack.transport.write
        calls = []

        def garble_first(data):
            calls.append(data)
            if len(calls) == 1:
                sim.out = b"\x99" * 9
                return len(data)
            return write(data)

        pack.transport.write = garble_first
        self.assertEqual(pack.position(2).motor, 2)
        self.assertEqual(len(calls), 2)

    def test_still_fails_when_the_unit_stays_silent(self):
        sim, pack = make(8)
        pack.transport.drop_replies = True
        with self.assertRaises(SixPackTimeout):
            pack.position(0)
        self.assertEqual(pack.retried, 2)

    def test_unit_info_is_not_asked_again(self):  # reading $43 clears the reset flag
        sim, pack = make(8)
        sim.drop_next = 1
        with self.assertRaises(SixPackTimeout):
            pack.unit_info()
        self.assertEqual(pack.retried, 0)

    def test_commands_are_not_repeated(self):
        sim, pack = make(8)
        pack.set_clock_divider(5)
        self.assertEqual(len(sim.frames), 1)


class Unit(unittest.TestCase):
    def setUp(self):
        self.sim, self.pack = make(8)

    def test_reset_flag_read_once(self):
        self.sim.power_cycle()
        self.assertTrue(self.pack.unit_info().reset)
        self.assertFalse(self.pack.unit_info().reset)

    def test_ramp(self):
        self.pack.set_clock_divider(5)
        self.pack.set_start_velocity(2, 0, 8, 3)
        self.pack.set_amax_vmax(2, 200, 400)
        self.pack.start_ramp(2, 1500)
        self.assertEqual(self.pack.position(2).action, Action.RAMP)
        wait_inactive(self.pack, 2)
        self.assertEqual(self.pack.position(2).position, 1500)
        self.assertEqual(self.pack.activities(), (0,) * 6)

    def test_stop(self):
        self.pack.start_ramp(0, 10**7)
        time.sleep(0.05)
        self.pack.stop(0b1)
        wait_inactive(self.pack, 0)
        self.assertLess(self.pack.position(0).position, 10**7)

    def test_rotation_then_ramp_only_stops(self):
        self.pack.rotate(1, -100)
        time.sleep(0.05)
        self.assertEqual(self.pack.velocity(1).velocity, -100)
        self.pack.start_ramp(1, 0)  # ends the rotation, no ramp
        self.assertEqual(self.pack.position(1).action, Action.INACTIVE)

    def test_stop_flag_cleared_by_read(self):
        self.pack.transport.write(bytes([0xFE, 0x02, 4]) + bytes(6))
        self.assertTrue(self.pack.position(4).stop)
        self.assertFalse(self.pack.position(4).stop)

    def test_reference_search(self):
        self.pack.set_position(3, 500)
        self.pack.set_null_offset(3, 20, 240)
        self.pack.set_reference_parameters(3, 100, 0x0FFF, True)
        self.pack.reference_search(3)
        self.assertGreaterEqual(self.pack.position(3).action, 20)
        wait_inactive(self.pack, 3)
        self.assertEqual(self.pack.position(3).position, 20)

    def test_inputs(self):
        r = self.pack.inputs(7)
        self.assertEqual((r.channel, r.analog), (7, 512))


if __name__ == "__main__":
    unittest.main()
