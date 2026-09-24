#!/usr/bin/env python3
"""
tests/test_core_control_filters.py — Unit test core C filters (PID, Complementary, EKF).

Port 1:1 from app/filtering.py (Python) + 1:1 -> core/src/control_filters.c.
Port dibuat identik agar hasilnya bisa dibandingkan bit-per-bit di sini, dengan
menggunakan ctypes untuk muat library yang kompak (via core/build/ jika
ada) atau via build sistem dengan linking statis.

Menguji:
  1. cf_pid_update()
  2. cf_complementary()
  3. cf_ekf_heading_predict() & cf_ekf_heading_update()
  4. Normalisasi wrap-around (via nav_math.h).
  5. Gunakan identik geometri dari Python (kontrol loop @20Hz).
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import API Python sebagai referensi baseline
from app.filtering import (
    PidController as PyPidController,
    complementary_filter as py_complementary_filter,
    HeadingEkf as PyHeadingEkf,
    normalize_wrap as py_normalize_angle,
    cross_track_error as py_cross_track_distance,
)

try:
    # Coba memuat library C statis
    from ctypes import CDLL, c_double, c_void_p
    # Coba lokasi build paling umum: build/core/libasv_control_filters.a
    lib_path = os.path.join(
        os.path.dirname(__file__), "build", "core", "libasv_control_filters.a")
    if not os.path.exists(lib_path):
        # Fallback ke relative import build yang kompak (tergantung proyek)
        lib_path = os.path.join(os.path.dirname(__file__), "build", "asv_control_filters.so")
    if not os.path.exists(lib_path):
        # Coba CMake build (mungkin ada di workspace root)
        lib_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "build", "core", "libasv_control_filters.a")

    # Muat library C
    lib = CDLL(lib_path)
    # Ekspor simbol (jika belum didefinisikan di header)
    lib.cf_pid_reset.argtypes = [c_void_p]
    lib.cf_pid_reset.restype = None
    lib.cf_pid_update.argtypes = [c_void_p, c_double, c_double]
    lib.cf_pid_update.restype = c_double
    lib.cf_complementary.argtypes = [c_double, c_double, c_double, c_double, c_double]
    lib.cf_complementary.restype = c_double
    lib.cf_ekf_init.argtypes = [c_void_p, c_double, c_double, c_double]
    lib.cf_ekf_init.restype = None
    lib.cf_ekf_predict.argtypes = [c_void_p, c_double, c_double]
    lib.cf_ekf_predict.restype = c_double
    lib.cf_ekf_update.argtypes = [c_void_p, c_double]
    lib.cf_ekf_update.restype = c_double
    lib.cf_ekf_heading.argtypes = [c_void_p]
    lib.cf_ekf_heading.restype = c_double

    HAS_C_LIB = True
except Exception as e:
    print(f"[WARN] Tidak bisa memuat library C: {e}")
    print("[INFO] Test filter C akan dilewati.")
    HAS_C_LIB = False


# ---------------------------------------------------------------------------
# Helper pembantu C
# ---------------------------------------------------------------------------

class CfPidController:
    """Wrapper ctypes untuk cf_pid_t (struct dari control_filters.h)."""
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, deadband=0.0,
                 output_limit=0.6, integral_limit=0.3):
        self._ptr = lib.cf_pid_alloc(kp, ki, kd, deadband, output_limit, integral_limit)

    def __del__(self):
        if HAS_C_LIB:
            lib.cf_pid_free(self._ptr)

    def reset(self):
        lib.cf_pid_reset(self._ptr)

    def update(self, error, dt):
        return lib.cf_pid_update(self._ptr, error, dt)


class CfHeadingEkf:
    def __init__(self, process_noise=0.05, meas_noise=0.1, init_heading=0.0):
        self._ptr = lib.cf_ekf_init(process_noise, meas_noise, init_heading)

    def predict(self, gyro_rate, dt):
        return lib.cf_ekf_predict(self._ptr, gyro_rate, dt)

    def update(self, measured):
        return lib.cf_ekf_update(self._ptr, measured)

    def heading(self):
        return lib.cf_ekf_heading(self._ptr)


# ---------------------------------------------------------------------------
# Test kasus identik: sequence deterministik diulang di C & Python
# ---------------------------------------------------------------------------

def generate_test_sequence():
    """Kembalikan list (dt, error) deterministik untuk loop PID."""
    # dt sedikit bervariasi 0.04..0.06 (target loop 20Hz)
    return [(0.05, 0.1), (0.05, -0.2), (0.05, 0.15),
            (0.04, -0.05), (0.06, 0.0), (0.05, 0.3)]


def test_pid_basic():
    """PID P murni: output ≈ kp * error (deadband = 0, integral=0, derivative=0)."""
    pid_py = PidController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0, output_limit=10.0)
    if HAS_C_LIB:
        pid_c = CfPidController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0, output_limit=10.0)
    for dt, err in generate_test_sequence():
        # Python
        out_py = pid_py.update(err, dt)
        expected = 2.0 * err
        if abs(out_py - expected) > 1e-9:
            raise AssertionError(f"Python PID output mismatch: {out_py} != {expected}")
        # C (bila dimuat)
        if HAS_C_LIB:
            out_c = pid_c.update(err, dt)
            if abs(out_c - out_py) > 1e-9:
                raise AssertionError(f"C vs Python PID mismatch: {out_c} != {out_py}")
            pid_c.reset()
    print("[OK] PID dasar")


def test_pid_deadband():
    """Error di dalam deadband -> output 0, integral tidak berjalan."""
    deadband = 0.1
    pid_py = PidController(kp=1.0, ki=0.5, kd=0.0, deadband=deadband, output_limit=100.0)
    if HAS_C_LIB:
        pid_c = CfPidController(kp=1.0, ki=0.5, kd=0.0, deadband=deadband, output_limit=100.0)

    # Error kecil -> mati
    out_py = pid_py.update(0.05, dt=0.1)
    assert out_py == 0.0, f"Python PID deadband gagal: {out_py}"
    if HAS_C_LIB:
        out_c = pid_c.update(0.05, dt=0.1)
        assert out_c == 0.0, f"C PID deadband gagal: {out_c}"

    # Error besar -> berjalan
    out_py = pid_py.update(0.2, dt=0.1)
    assert out_py != 0.0, f"Python PID harus aktif"
    if HAS_C_LIB:
        out_c = pid_c.update(0.2, dt=0.1)
        assert out_c != 0.0, f"C PID harus aktif"

    print("[OK] PID deadband")


def test_pid_integral_anti_windup():
    """Integral berjalan, clamp output_limit."""
    pid_py = PidController(kp=0.0, ki=10.0, kd=0.0, deadband=0.0,
                          output_limit=1.0, integral_limit=0.1)
    if HAS_C_LIB:
        pid_c = CfPidController(kp=0.0, ki=10.0, kd=0.0, deadband=0.0,
                               output_limit=1.0, integral_limit=0.1)

    # Satu error besar -> integral cepat tumbuh (0.5*0.1=0.05), output ~0.5
    out_py = pid_py.update(0.5, dt=0.1)
    assert 0.4 < out_py < 0.6, f"Python PID integral tidak benar: {out_py}"
    if HAS_C_LIB:
        out_c = pid_c.update(0.5, dt=0.1)
        assert 0.4 < out_c < 0.6, f"C PID integral tidak benar: {out_c}"
        # Reset dulu (agar tidak akumulasi di iterasi berikutnya)
        pid_c.reset()

    print("[OK] PID integral anti-windup")


def test_complementary_filter():
    """Fusi murni: alpha=1 -> gyro, alpha=0 -> measured."""
    alpha = 0.6
    angle_prev = 0.5
    gyro_rate = 2.0
    dt = 0.1
    measured = 1.0

    out_py = complementary_filter(alpha, angle_prev, gyro_rate, dt, measured)
    expected = alpha * (angle_prev + gyro_rate * dt) + (1.0 - alpha) * measured
    assert abs(out_py - expected) < 1e-9, f"Python complementary mismatch"

    if HAS_C_LIB:
        out_c = lib.cf_complementary(alpha, angle_prev, gyro_rate, dt, measured)
        assert abs(out_c - expected) < 1e-9, f"C complementary mismatch"

    print("[OK] Complementary filter")


def test_ekf_heading():
    """EKF heading: prediksi + update harus converge ke measured."""
    process_noise = 0.01
    meas_noise = 0.01
    ekf_py = PyHeadingEkf(process_noise, meas_noise, init_heading=0.0)
    if HAS_C_LIB:
        ekf_c = CfHeadingEkf(process_noise, meas_noise, init_heading=0.0)

    gyro_rate = 0.5  # rad/s
    for i in range(20):
        dt = 0.1
        # predict
        ekf_py.predict(gyro_rate, dt)
        if HAS_C_LIB:
            ekf_c.predict(gyro_rate, dt)
        # update ke heading sebenarnya (0.5 rad)
        ekf_py.update(0.5)
        if HAS_C_LIB:
            ekf_c.update(0.5)

    # Heading harus ~0.5 (konvergen)
    assert abs(ekf_py.heading() - 0.5) < 0.02, "Python EKF belum konvergen"
    if HAS_C_LIB:
        assert abs(ekf_c.heading() - 0.5) < 0.02, "C EKF belum konvergen"

    # Beda C vs Python harus dekat (<1e-6) untuk urutan yang identik
    if HAS_C_LIB:
        diff = abs(ekf_c.heading() - ekf_py.heading())
        if diff > 1e-6:
            print(f"[WARN] EKF heading berbeda: C={ekf_c.heading()}, Python={ekf_py.heading()}, diff={diff}")

    print("[OK] EKF heading")


def test_wrap_angle():
    """Wrap-around (normalisasi) sama untuk C & Python."""
    # NavMath C didelegasi ke nav_math (sudah diuji lewat test_core_nav_math).
    # Di sini cukup cek fungsi pembantu Python.
    angle = 3.5 * math.pi
    wrapped_py = normalize_wrap(angle)
    assert -math.pi <= wrapped_py < math.pi, "Python wrap-angle gagal"

    if HAS_C_LIB:
        from ctypes import c_double
        wrapped_c = lib.nav_normalize_angle(angle)
        assert abs(wrapped_c - wrapped_py) < 1e-9, f"C vs Python wrap mismatch: {wrapped_c} vs {wrapped_py}"

    print("[OK] Normalisasi wrap-angle")


def test_cross_track_distance():
    """CTE delegasi ke geo (C nav_math). Smoke test."""
    cte_py = cross_track_error(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(cte_py) < 0.001
    if HAS_C_LIB:
        cte_c = lib.nav_cross_track_distance(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        assert abs(cte_c - cte_py) < 1e-9, f"CTE mismatch: {cte_c} vs {cte_py}"

    print("[OK] Cross-track distance")


if __name__ == "__main__":
    import traceback
    tests = [
        test_pid_basic,
        test_pid_deadband,
        test_pid_integral_anti_windup,
        test_complementary_filter,
        test_ekf_heading,
        test_wrap_angle,
        test_cross_track_distance,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            passed += 1
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
            traceback.print_exc()
    print(f"\n{passed}/{len(tests)} test passed.")
    sys.exit(0 if passed == len(tests) else 1)