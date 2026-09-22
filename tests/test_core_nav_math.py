"""Test kunci — buktikan bahwa C (nav_math.c) menghasilkan output yang
SAMA dengan Python (app/geo.py), bukan sekadar "angka mirip".

Cara kerja:
  1. Compile core/src/nav_math.c -> libnav_math.so (gcc, sementara).
  2. Panggil fungsi C lewat ctypes.
  3. Bandingkan terhadap app/geo.py pada grid deterministik + 500 kasus acak.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import ctypes
import math
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from app import geo

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_C = os.path.join(REPO, "core", "src", "nav_math.c")
INC_C = os.path.join(REPO, "core", "include")


def _build_nav_math_lib():
    """Compile nav_math.c jadi shared library sementara + set prototipe ctypes."""
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        raise unittest.SkipTest("gcc tidak tersedia di mesin ini")

    tmp = tempfile.mkdtemp(prefix="libnav_math_")
    so = os.path.join(tmp, "libnav_math.so")
    subprocess.run([cc, "-O2", "-fPIC", "-shared", "-I", INC_C,
                    SRC_C, "-o", so, "-lm"],
                   check=True, capture_output=True)

    lib = ctypes.CDLL(so)
    _c = ctypes.c_double
    _pc = ctypes.POINTER(ctypes.c_double)

    lib.nav_distance_bearing.argtypes = [_c, _c, _c, _c, _pc]
    lib.nav_distance_bearing.restype = _c
    lib.nav_destination.argtypes = [_c, _c, _c, _c, _pc, _pc]
    lib.nav_destination.restype = None
    lib.nav_normalize_angle.argtypes = [_c]
    lib.nav_normalize_angle.restype = _c
    lib.nav_cross_track_distance.argtypes = [_c] * 6
    lib.nav_cross_track_distance.restype = _c
    return lib


def c_distance_bearing(lib, lat1, lon1, lat2, lon2):
    b = ctypes.c_double()
    d = lib.nav_distance_bearing(lat1, lon1, lat2, lon2, ctypes.byref(b))
    return d, b.value


def c_destination(lib, lat, lon, d, brg):
    la, lo = ctypes.c_double(), ctypes.c_double()
    lib.nav_destination(lat, lon, d, brg, ctypes.byref(la), ctypes.byref(lo))
    return la.value, lo.value


class TestCTallyVsPython(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_nav_math_lib()

    def close(self, a, b, rel=1e-9, abs_=1e-9, label=""):
        ok = math.isclose(a, b, rel_tol=rel, abs_tol=abs_)
        self.assertTrue(
            ok,
            f"{label} C={a:.17g} vs Python={b:.17g} (diff={abs(a-b):.3g})")

    # ---------- kasus deterministik ----------
    def test_known_values(self):
        lib = self.lib

        d, brg = c_distance_bearing(lib, 0, 0, 1, 0)
        exp_d, exp_brg = geo.distance_bearing(0, 0, 1, 0)
        self.close(d, exp_d, rel=1e-12, label="1 derajat lintang")
        self.close(brg, exp_brg, rel=1e-12, label="bearing utara")

        d, brg = c_distance_bearing(lib, 0, 0, 0, 1)
        exp_d, exp_brg = geo.distance_bearing(0, 0, 0, 1)
        self.close(brg, exp_brg, rel=1e-12, label="bearing timur")

        d, brg = c_distance_bearing(lib, -7.9153897, 112.589158,
                                    -7.9153897, 112.589158)
        exp_d, exp_brg = geo.distance_bearing(-7.9153897, 112.589158,
                                              -7.9153897, 112.589158)
        self.close(d, exp_d, abs_=1e-9, label="jarak titik sama")

    def test_invalid_nan_matches_python_inf(self):
        lib = self.lib
        d, brg = c_distance_bearing(lib, float("nan"), 0.0, 1.0, 0.0)
        self.assertTrue(math.isinf(d) and d > 0, "C must return +inf on NaN")
        self.assertEqual(brg, 0.0)

    # ---------- grid deterministik: C vs Python 1:1 ----------
    def test_grid_of_coordinates(self):
        lib = self.lib
        lats = [-80.0, -45.5, -7.9153897, 0.0, 12.34, 45.5, 80.0]
        lons = [-170.0, -60.0, -7.0, 0.0, 112.589158, 170.0]
        for lat1 in lats:
            for lon1 in lons:
                for lat2 in lats:
                    for lon2 in lons:
                        p_d, p_b = geo.distance_bearing(lat1, lon1, lat2, lon2)
                        c_d, c_b = c_distance_bearing(lib, lat1, lon1, lat2, lon2)
                        self.close(c_d, p_d, rel=1e-9, abs_=1e-9,
                                   label=f"dist ({lat1},{lon1})->({lat2},{lon2})")
                        if not math.isinf(p_d):
                            self.close(c_b, p_b, rel=1e-9, abs_=1e-9,
                                       label=f"brg ({lat1},{lon1})->({lat2},{lon2})")

    def test_normalize_matches(self):
        lib = self.lib
        for ang in [0.0, 0.5, math.pi, -math.pi, 4.0, -4.0, 10.0, -10.0,
                    3.5 * math.pi, -3.5 * math.pi]:
            self.close(lib.nav_normalize_angle(ang), geo.normalize_angle(ang),
                       rel=1e-12, abs_=1e-12, label=f"normalize({ang})")

    def test_destination_matches(self):
        lib = self.lib
        for lat, lon, d, brg in [(-7.9153897, 112.589158, 1000.0, 0.0),
                                 (-7.9153897, 112.589158, 500.0, math.pi / 2),
                                 (0.0, 0.0, 20000.0, 2.1),
                                 (-33.9, 151.2, 1.0, -0.5)]:
            p_lat, p_lon = geo.destination(lat, lon, d, brg)
            c_lat, c_lon = c_destination(lib, lat, lon, d, brg)
            self.close(c_lat, p_lat, rel=1e-9, abs_=1e-9, label="dest.lat")
            self.close(c_lon, p_lon, rel=1e-9, abs_=1e-9, label="dest.lon")

    def test_cross_track_matches(self):
        lib = self.lib
        cases = [(0.0, 0.005, 0.0, 0.0, 0.0, 0.01),
                 (0.001, 0.005, 0.0, 0.0, 0.0, 0.01),
                 (-0.001, 0.005, 0.0, 0.0, 0.0, 0.01),
                 (-7.9153897, 112.589158, -7.9155, 112.5889,
                  -7.9153897, 112.589158)]
        for case in cases:
            p = geo.cross_track_distance(*case)
            c = lib.nav_cross_track_distance(*case)
            self.close(c, p, rel=1e-9, abs_=1e-9, label=f"xtd {case}")

    # ---------- kasus acak (benih tetap agar reproducible) ----------
    def test_random_cases(self):
        lib = self.lib
        rng = random.Random(20260922)
        for _ in range(500):
            lat1, lon1 = rng.uniform(-85, 85), rng.uniform(-179, 179)
            lat2, lon2 = rng.uniform(-85, 85), rng.uniform(-179, 179)
            p_d, p_b = geo.distance_bearing(lat1, lon1, lat2, lon2)
            c_d, c_b = c_distance_bearing(lib, lat1, lon1, lat2, lon2)
            self.close(c_d, p_d, rel=1e-9, abs_=1e-6, label=f"random dist #{_}")
            if not math.isinf(p_d):
                self.close(c_b, p_b, rel=1e-9, abs_=1e-9, label=f"random brg #{_}")


if __name__ == "__main__":
    unittest.main()