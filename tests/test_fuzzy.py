"""tests/test_fuzzy.py — tes unit fuzzy controller C (libaterkia.so)."""
import unittest
from app import aterkia_core as c


class TestFuzzy(unittest.TestCase):
    def test_fuzzy_gate_p_gain_far_nonzero(self):
        """Jarak > 1.0 m (misal 2.0 m) dan error != 0 tidak boleh mengembalikan 0.0."""
        gain = c.fuzzy_gate_gain(2.0, 100.0)
        self.assertGreater(gain, 0.0, "Gain pada jarak 2m + error 100 tidak boleh 0")
        self.assertAlmostEqual(gain, 2.5, places=1)

    def test_fuzzy_gate_p_gain_ranges(self):
        """Uji bahu kiri (jarak 0m) dan kanan (2m)."""
        # Dekat
        g0 = c.fuzzy_gate_gain(0.1, 10.0)
        self.assertGreater(g0, 0.0)
        # Jauh
        g2 = c.fuzzy_gate_gain(2.0, 10.0)
        self.assertGreater(g2, 0.0)


if __name__ == "__main__":
    unittest.main()
