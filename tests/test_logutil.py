"""Test app/logutil.py — logger + throttle (tanpa I/O berat).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import time
import unittest

from app.logutil import get_logger, throttled


class TestLogUtil(unittest.TestCase):

    def test_get_logger_singleton_nama(self):
        a = get_logger()
        b = get_logger()
        self.assertIs(a, b)
        self.assertEqual(a.name, "asv")

    def test_throttled_interval(self):
        key = "uji-%f" % time.time()
        self.assertTrue(throttled(key, 60.0))
        self.assertFalse(throttled(key, 60.0))
        self.assertTrue(throttled(key + "-lain", 60.0))

    def test_throttled_interval_nol_selalu_true(self):
        key = "nol-%f" % time.time()
        self.assertTrue(throttled(key, 0))
        self.assertTrue(throttled(key, 0))
        self.assertTrue(throttled(key, -1))

    def test_throttled_kedaluwarsa(self):
        key = "exp-%f" % time.time()
        self.assertTrue(throttled(key, 0.05))
        time.sleep(0.07)
        self.assertTrue(throttled(key, 0.05))


if __name__ == "__main__":
    unittest.main()
