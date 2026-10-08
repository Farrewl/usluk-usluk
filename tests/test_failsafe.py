"""Test failsafe navigator (tanpa hardware): stale-link & low-batt.

GroundSimNavigator.__new__ dipakai agar tanpa YOLO/kamera/MAVLink —
hanya state + config + _check_failsafe/_request_rtl yang diuji. Master
MAVLink diganti stub (command_long_send tercatat, tak ada I/O).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import time
import unittest
from types import SimpleNamespace

from app import settings as cfg
from app.simulator import GroundSimNavigator


class _MavStub:
    def __init__(self):
        self.calls = []
        self.target_system = 1
        self.target_component = 1

    class _Mav:
        def __init__(self, outer):
            self._outer = outer

        def command_long_send(self, *args):
            self._outer.calls.append(args)

    @property
    def mav(self):
        return self._Mav(self)


def _nav_bare():
    """Navigator tanpa __init__ (tanpa hardware), state failsafe default."""
    nav = GroundSimNavigator.__new__(GroundSimNavigator)
    nav.config = cfg.Config()
    nav.failsafe_active = False
    nav.failsafe_reason = ""
    nav._lowbatt_since = None
    nav._rtl_sent_mono = 0.0
    nav._last_telem_mono = time.monotonic()
    nav.current_voltage = 16.0
    nav.current_current = 5.0
    nav.current_battery_pct = 90.0
    nav.master = _MavStub()
    return nav


class TestFailsafeStaleLink(unittest.TestCase):

    def test_telemetri_segar_aman(self):
        nav = _nav_bare()
        nav._last_telem_mono = time.monotonic()
        self.assertEqual(nav._check_failsafe(), "")
        self.assertFalse(nav.failsafe_active)

    def test_telemetri_basi_picU(self):
        nav = _nav_bare()
        nav.config.FAILSAFE_TELEM_TIMEOUT_S = 2.0
        nav._last_telem_mono = time.monotonic() - 5.0
        reason = nav._check_failsafe()
        self.assertTrue(reason, "telemetri basi harus memicu failsafe")
        self.assertTrue(nav.failsafe_active)
        self.assertIn("telemetri", nav.failsafe_reason)
        # RTL best-effort terkirim 1x (throttle 5 detik).
        self.assertEqual(len(nav.master.calls), 1)
        # Panggilan berikutnya throttled (tak ada kirim ulang).
        nav._check_failsafe()
        self.assertEqual(len(nav.master.calls), 1)

    def test_pulih_otomatis(self):
        nav = _nav_bare()
        nav._last_telem_mono = time.monotonic() - 5.0
        nav._check_failsafe()
        self.assertTrue(nav.failsafe_active)
        nav._last_telem_mono = time.monotonic()
        self.assertEqual(nav._check_failsafe(), "")
        self.assertFalse(nav.failsafe_active)

    def test_disable_mematikan(self):
        nav = _nav_bare()
        nav.config.FAILSAFE_ENABLED = False
        nav._last_telem_mono = time.monotonic() - 99.0
        self.assertEqual(nav._check_failsafe(), "")
        self.assertFalse(nav.failsafe_active)


class TestFailsafeLowBatt(unittest.TestCase):

    def test_spike_sesaat_tak_memicu(self):
        nav = _nav_bare()
        nav.config.FAILSAFE_LOW_BATT_PCT = 20.0
        nav.config.FAILSAFE_LOW_BATT_HOLD_S = 3.0
        nav.current_battery_pct = 5.0  # drop sesaat
        self.assertEqual(nav._check_failsafe(), "")
        self.assertFalse(nav.failsafe_active)

    def test_rendah_ditahan_memicu(self):
        nav = _nav_bare()
        nav.config.FAILSAFE_LOW_BATT_PCT = 20.0
        nav.config.FAILSAFE_LOW_BATT_HOLD_S = 3.0
        nav.current_battery_pct = 5.0
        nav._check_failsafe()  # mulai hitung hold
        nav._lowbatt_since = time.monotonic() - 4.0  # tahan lewat
        reason = nav._check_failsafe()
        self.assertTrue(reason)
        self.assertIn("baterai", reason)

    def test_tegangan_rendah_memicu(self):
        nav = _nav_bare()
        nav.config.FAILSAFE_LOW_VOLT_V = 13.2
        nav.config.FAILSAFE_LOW_BATT_HOLD_S = 3.0
        nav.current_battery_pct = None  # firmware tak kirim persen
        nav.current_voltage = 12.0
        nav._check_failsafe()
        nav._lowbatt_since = time.monotonic() - 4.0
        self.assertTrue(nav._check_failsafe())

    def test_tak_ada_data_batt_aman(self):
        nav = _nav_bare()
        nav.current_battery_pct = None
        nav.current_voltage = None
        self.assertEqual(nav._check_failsafe(), "")


class TestRtlCommand(unittest.TestCase):

    def test_mode_rover_rtl_11(self):
        nav = _nav_bare()
        nav._request_rtl()
        self.assertEqual(len(nav.master.calls), 1)
        args = nav.master.calls[0]
        # args: (tgt_sys, tgt_comp, MAV_CMD_DO_SET_MODE, conf, flag, mode,...)
        self.assertEqual(args[5], 11, "mode RTL ArduPilot Rover = 11")


if __name__ == "__main__":
    unittest.main()
