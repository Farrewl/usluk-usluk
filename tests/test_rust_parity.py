"""tests/test_rust_parity.py — Rust core harus identik dengan C core."""
import ctypes
import os
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
C_PATH = os.path.join(ROOT, "core", "libaterkia.so")
RUST_PATH = os.path.join(ROOT, "core", "libaterkia_rust.so")


class Pid(ctypes.Structure):
    _fields_ = [(n, ctypes.c_double) for n in
                ("kp", "ki", "kd", "deadband", "output_limit",
                 "integral_limit", "integral", "last_error")]


def _load(path):
    lib = ctypes.CDLL(path)
    lib.nav_distance_bearing.restype = ctypes.c_double
    lib.nav_distance_bearing.argtypes = [ctypes.c_double] * 4 + [ctypes.POINTER(ctypes.c_double)]
    lib.fuzzy_gate_p_gain.restype = ctypes.c_double
    lib.fuzzy_gate_p_gain.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.fuzzy_docking_p_gain.restype = ctypes.c_double
    lib.fuzzy_docking_p_gain.argtypes = [ctypes.c_double, ctypes.c_double]
    lib.cf_pid_update.restype = ctypes.c_double
    lib.cf_pid_update.argtypes = [ctypes.POINTER(Pid), ctypes.c_double, ctypes.c_double]
    lib.gv_yaw_correction.restype = ctypes.c_double
    lib.gv_yaw_correction.argtypes = [ctypes.c_double] * 4
    lib.mixer_diff_drive.argtypes = [ctypes.c_double, ctypes.c_double,
                                     ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
    return lib


@unittest.skipUnless(os.path.exists(C_PATH) and os.path.exists(RUST_PATH),
                     "butuh core/libaterkia.so + libaterkia_rust.so")
class TestParity(unittest.TestCase):
    def setUp(self):
        self.c = _load(C_PATH)
        self.r = _load(RUST_PATH)

    def test_fuzzy_gate(self):
        for d in (0.0, 0.15, 0.35, 0.55, 0.75, 0.9, 1.2):
            for e in (0.0, 25.0, 50.0, 100.0, 200.0, 400.0):
                self.assertAlmostEqual(self.c.fuzzy_gate_p_gain(d, e),
                                       self.r.fuzzy_gate_p_gain(d, e), places=9)

    def test_fuzzy_docking(self):
        for d in (0.0, 0.6, 1.5, 3.0, 5.0, 8.0, 12.0):
            for e in (0.0, 30.0, 70.0, 150.0, 400.0):
                self.assertAlmostEqual(self.c.fuzzy_docking_p_gain(d, e),
                                       self.r.fuzzy_docking_p_gain(d, e), places=9)

    def test_pid(self):
        def run(lib):
            p = Pid(kp=1.5, ki=0.2, kd=0.05, deadband=0.01,
                    output_limit=1.0, integral_limit=2.0,
                    integral=0.0, last_error=0.0)
            return [lib.cf_pid_update(ctypes.byref(p), err, 0.05)
                    for err in (0.5, 0.4, 0.3, 0.0, -0.2, -0.6)]
        self.assertEqual(run(self.c), run(self.r))

    def test_nav_distance(self):
        b_c, b_r = ctypes.c_double(), ctypes.c_double()
        dc = self.c.nav_distance_bearing(-6.2, 106.8, -6.21, 106.81, ctypes.byref(b_c))
        dr = self.r.nav_distance_bearing(-6.2, 106.8, -6.21, 106.81, ctypes.byref(b_r))
        self.assertAlmostEqual(dc, dr, places=6)
        self.assertAlmostEqual(b_c.value, b_r.value, places=9)

    def test_yaw_correction(self):
        for tx in (100.0, 320.0, 500.0):
            self.assertAlmostEqual(
                self.c.gv_yaw_correction(tx, 320.0, 3.0, 500.0),
                self.r.gv_yaw_correction(tx, 320.0, 3.0, 500.0), places=9)

    def test_mixer(self):
        for surge, yaw in ((0.5, 0.2), (-1.2, 0.4), (1.0, -1.0)):
            lc, rc = ctypes.c_int(), ctypes.c_int()
            lr, rr = ctypes.c_int(), ctypes.c_int()
            self.c.mixer_diff_drive(surge, yaw, ctypes.byref(lc), ctypes.byref(rc))
            self.r.mixer_diff_drive(surge, yaw, ctypes.byref(lr), ctypes.byref(rr))
            self.assertEqual((lc.value, rc.value), (lr.value, rr.value))


if __name__ == "__main__":
    unittest.main()
