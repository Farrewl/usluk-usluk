"""Test kunci — buktikan bahwa C (fuzzy.c) menghasilkan output yang SAMA
dengan Python (app/fuzzy.py), bukan sekadar "angka mirip".

Cara kerja (sama dengan test_core_nav_math.py):
  1. Compile core/src/fuzzy.c -> libfuzzy.so (gcc, sementara).
  2. Panggil fungsi C lewat ctypes.
  3. Bandingkan terhadap app/fuzzy.py pada grid deterministik + 500 kasus acak
     di dalam semesta, plus titik di luar semesta.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import ctypes
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from app.fuzzy import docking_p_gain, gate_p_gain

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_C = os.path.join(REPO, "core", "src", "fuzzy.c")
INC_C = os.path.join(REPO, "core", "include")

TOL = 1e-12


def _build_fuzzy_lib():
    """Compile fuzzy.c jadi shared library sementara + set prototipe ctypes."""
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        raise unittest.SkipTest("gcc tidak tersedia di mesin ini")

    tmp = tempfile.mkdtemp(prefix="libfuzzy_")
    so = os.path.join(tmp, "libfuzzy.so")
    subprocess.run([cc, "-O2", "-fPIC", "-shared", "-I", INC_C,
                    SRC_C, "-o", so, "-lm"],
                   check=True, capture_output=True)

    lib = ctypes.CDLL(so)
    _c = ctypes.c_double
    lib.fuzzy_gate_p_gain.argtypes = [_c, _c]
    lib.fuzzy_gate_p_gain.restype = _c
    lib.fuzzy_docking_p_gain.argtypes = [_c, _c]
    lib.fuzzy_docking_p_gain.restype = _c
    return lib


class TestCFuzzyVsPython(unittest.TestCase):
    """Cross-check C vs Python pada grid + acak (identik, bukan mirip)."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_fuzzy_lib()

    def _assert_sama(self, c_val, py_val, jarak, error):
        self.assertAlmostEqual(
            c_val, py_val, places=12,
            msg=f"jarak={jarak} error={error}: C={c_val!r} != py={py_val!r}")

    def test_grid_gate(self):
        # grid deterministik di semesta gate: jarak 0..1.6, error 0..320
        for i in range(33):
            jarak = i * 0.05
            for j in range(65):
                error = j * 5.0
                self._assert_sama(self.lib.fuzzy_gate_p_gain(jarak, error),
                                  gate_p_gain(jarak, error), jarak, error)

    def test_grid_docking(self):
        for i in range(41):
            jarak = i * 0.1
            for j in range(33):
                error = j * 10.0
                self._assert_sama(self.lib.fuzzy_docking_p_gain(jarak, error),
                                  docking_p_gain(jarak, error), jarak, error)

    def test_acak_gate(self):
        rng = random.Random(20260923)
        for _ in range(500):
            jarak = rng.uniform(0.0, 1.2)
            error = rng.uniform(0.0, 320.0)
            self._assert_sama(self.lib.fuzzy_gate_p_gain(jarak, error),
                              gate_p_gain(jarak, error), jarak, error)

    def test_acak_docking(self):
        rng = random.Random(20260923)
        for _ in range(500):
            jarak = rng.uniform(0.0, 1.5)
            error = rng.uniform(0.0, 320.0)
            self._assert_sama(self.lib.fuzzy_docking_p_gain(jarak, error),
                              docking_p_gain(jarak, error), jarak, error)

    def test_luar_semesta_sama(self):
        # Di luar semesta keduanya harus 0.0 (tidak ada aturan menyala).
        for jarak, error in [(5.0, 500.0), (2.0, 320.0), (-1.0, 10.0),
                             (0.5, -5.0), (10.0, 321.0)]:
            self._assert_sama(self.lib.fuzzy_gate_p_gain(jarak, error),
                              gate_p_gain(jarak, error), jarak, error)
            self._assert_sama(self.lib.fuzzy_docking_p_gain(jarak, error),
                              docking_p_gain(jarak, error), jarak, error)


if __name__ == "__main__":
    unittest.main()