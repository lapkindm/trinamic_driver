"""Bliss controller's $43 check against the simulator (needs Bliss importable,
no Beacon: the controller is built without its config)."""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fake_sixpack import FakeSixPack, FakeTransport  # noqa: E402
from trinamic_driver import SixPack, SixPackTimeout  # noqa: E402

try:
    from trinamic_driver import bliss_trinamic as bt
except ImportError:  # no Bliss here
    bt = None


@unittest.skipIf(bt is None, "Bliss not installed")
class UnitInfo(unittest.TestCase):
    def setUp(self):
        self.sim = FakeSixPack(reply_length=9)
        ctrl = bt.Trinamic.__new__(bt.Trinamic)
        ctrl._pack = SixPack(FakeTransport(self.sim), address=0, timeout=0.05, reply_length=9)
        ctrl._data_by_name = {}
        ctrl._unit_info_retries, ctrl._unit_info_pause = 5, 0.01
        ctrl._last_check = 0.0
        self.ctrl = ctrl
        self.ctrl._pack.unit_info()  # clear the power-on reset flag
        patches = [mock.patch.object(bt.Trinamic, "name", "trinamic_test", create=True),
                   mock.patch.object(bt, "log_warning"),
                   mock.patch.object(bt.Trinamic, "_handle_reset")]
        for p in patches:
            p.start()
            self.addCleanup(p.stop)
        self.warn, self.reset = bt.log_warning, bt.Trinamic._handle_reset

    def test_answers_after_four_silent_queries(self):
        self.sim.drop_unit_info = 4
        info = self.ctrl._unit_info()
        self.assertFalse(info.reset)
        self.assertEqual(self.warn.call_count, 5)  # four "asking again" + "answered again"
        self.reset.assert_not_called()

    def test_gives_up_after_five_retries(self):
        self.sim.drop_unit_info = 6
        with self.assertRaises(SixPackTimeout):
            self.ctrl._unit_info()
        self.assertEqual(self.warn.call_count, 5)

    def test_five_retries_are_enough(self):
        self.sim.drop_unit_info = 5
        self.ctrl._unit_info()
        self.assertEqual(self.sim.drop_unit_info, 0)

    def test_reset_with_lost_reply_still_detected_from_positions(self):
        data = mock.Mock(channel=0, last_steps=1234)
        self.ctrl._data_by_name = {"a2": data}
        self.sim.power_cycle()
        self.sim.drop_unit_info = 1  # the reply with the reset flag is lost
        info = self.ctrl._unit_info()
        self.assertTrue(info.reset)
        self.reset.assert_called_once()


if __name__ == "__main__":
    unittest.main()
