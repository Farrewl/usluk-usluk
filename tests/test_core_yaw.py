"""Test kunci P2 — yaw_correction_c (C) SAMA dengan rumus Python navigator.

Rumus acuan (dulu diduplikasi 4x di _calculate_yaw_correction_*):
  error_m = (target_x - center_x) * dist / focal
  raw     = atan2(error_m, dist), guard dist < 0.1 / focal <= 0 -> 0.0

Jalankan:  python3 -m unittest discover -s tests -v
"""

import math
import random
import unittest

from app import aterkia_core as core


def _py_yaw(target_x, center_x, dist, focal):
    if dist < 0.1 or focal <= 0:
        return 0.0
    error_m = ((target_x - center_x) * dist) / focal
    return math.atan2(error_m, dist)


class TestYawCorrectionVsC(unittest.TestCase):

    def test_nilai_polos(self):
        # error 40 px @ 4 m, f=400 -> error_m=0.4 -> atan2(0.4, 4).
        c = core.yaw_correction_c(360.0, 320.0, 4.0, 400.0)
        self.assertAlmostEqual(c, math.atan2(0.4, 4.0), places=12)

    def test_nol_saat_tengah(self):
        self.assertEqual(core.yaw_correction_c(320.0, 320.0, 4.0, 400.0),
                         0.0)

    def test_guard_jarak_dan_focal(self):
        self.assertEqual(core.yaw_correction_c(360.0, 320.0, 0.05, 400.0),
                         0.0)
        self.assertEqual(core.yaw_correction_c(360.0, 320.0, 4.0, 0.0), 0.0)
        self.assertEqual(core.yaw_correction_c(360.0, 320.0, 4.0, -10.0),
                         0.0)

    def test_acak_sama_dengan_python(self):
        rng = random.Random(41)
        for _ in range(500):
            tx = rng.uniform(0, 640)
            cx = rng.uniform(0, 640)
            dist = rng.uniform(0.0, 12.0)
            focal = rng.choice([0.0, 200.0, 400.0, 800.0])
            c = core.yaw_correction_c(tx, cx, dist, focal)
            py = _py_yaw(tx, cx, dist, focal)
            self.assertAlmostEqual(c, py, places=12,
                                   msg=f"tx={tx} cx={cx} d={dist} f={focal}")


if __name__ == "__main__":
    unittest.main()
