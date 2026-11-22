"""tests/test_safety.py — failsafe wajib HOLD saat competition_mode."""
import time
import unittest

from app.safety import SafetyManager


class _Cfg:
    FAILSAFE_ENABLED = True
    FAILSAFE_TELEM_TIMEOUT_S = 0.01
    FAILSAFE_LOW_BATT_PCT = 0.0
    FAILSAFE_LOW_VOLT_V = 0.0
    FAILSAFE_LOW_BATT_HOLD_S = 0.01
    COMPETITION_MODE = True


class _Link:
    def __init__(self):
        self.hold = 0
        self.rtl = 0

    def request_hold(self):
        self.hold += 1

    def request_rtl(self):
        self.rtl += 1


class TestSafety(unittest.TestCase):
    def _reason_after_stale(self, comp_mode):
        cfg = _Cfg()
        cfg.COMPETITION_MODE = comp_mode
        link = _Link()
        safety = SafetyManager(cfg, link=link)
        safety.mark_telem()
        time.sleep(0.02)
        reason = safety.check(100.0, 16.0)
        return reason, link

    def test_competition_mode_failsafe_requests_hold(self):
        reason, link = self._reason_after_stale(comp_mode=True)
        self.assertTrue(reason)
        self.assertEqual(link.hold, 1)
        self.assertEqual(link.rtl, 0)

    def test_non_competition_mode_failsafe_requests_rtl(self):
        reason, link = self._reason_after_stale(comp_mode=False)
        self.assertTrue(reason)
        self.assertEqual(link.rtl, 1)
        self.assertEqual(link.hold, 0)


if __name__ == "__main__":
    unittest.main()
