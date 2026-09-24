#!/usr/bin/env python3
"""
app/filtering.py — Kumpulan filter & kontroler murni Python untuk galat navigasi.

Port 1:1 ke core/src/control_filters.c (ditunjukkan identik lewat
`tests/test_core_control_filters.py`).

Menguji:
  1. PID dengan deadband, anti-windup, clamp output.
  2. Filter komplementer fusi heading.
  3. EKF heading 1-D (predict + update + wrap-around inovasi).
  4. Normalisasi wrap-around (delegasi ke app/geo.normalize_angle).
  5. Cross-Track Error (delegasi ke app/geo.cross_track_distance).

Semua test deterministik (tanpa random).
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.filtering import (
    PidController, complementary_filter, HeadingEkf,
    normalize_wrap, cross_track_error,
)

# ---------------------------------------------------------------------------
# PID + deadband
# ---------------------------------------------------------------------------

def test_pid_basic_proportional():
    pid = PidController(kp=2.0, ki=0.0, kd=0.0, deadband=0.0, output_limit=10.0)
    out = pid.update(1.0, dt=0.1)
    assert abs(out - 2.0) < 1e-9


def test_pid_deadband_zero_inside():
    pid = PidController(kp=1.0, ki=0.5, kd=0.0, deadband=0.05, output_limit=10.0)
    pid.update(0.02, dt=0.1)
    assert pid.integral == 0.0
    assert pid.update(0.02, dt=0.1) == 0.0


def test_pid_deadband_active_outside():
    pid = PidController(kp=1.0, ki=0.0, kd=0.0, deadband=0.05, output_limit=10.0)
    out = pid.update(0.1, dt=0.1)
    assert abs(out - 0.1) < 1e-9


def test_pid_integral_accumulates():
    pid = PidController(kp=0.0, ki=10.0, kd=0.0, deadband=0.0, output_limit=100.0)
    pid.update(0.5, dt=0.1)
    pid.update(0.5, dt=0.1)
    out = pid.update(0.0, dt=0.1)
    assert abs(out - 1.0) < 1e-9


def test_pid_anti_windup_integral_limit():
    pid = PidController(kp=0.0, ki=100.0, kd=0.0, deadband=0.0,
                        output_limit=100.0, integral_limit=0.01)
    pid.update(1.0, dt=0.1)
    pid.update(1.0, dt=0.1)
    out = pid.update(0.0, dt=0.1)
    assert abs(out - 1.0) < 1e-9


def test_pid_output_clamp():
    pid = PidController(kp=100.0, ki=0.0, kd=0.0, deadband=0.0, output_limit=0.5)
    out = pid.update(1.0, dt=0.1)
    assert abs(out - 0.5) < 1e-9
    out2 = pid.update(-1.0, dt=0.1)
    assert abs(out2 + 0.5) < 1e-9


def test_pid_derivative_term():
    pid = PidController(kp=0.0, ki=0.0, kd=5.0, deadband=0.0, output_limit=10.0)
    out = pid.update(1.0, dt=0.1)
    assert abs(out - 10.0) < 1e-6


def test_pid_reset_clears_state():
    pid = PidController(kp=1.0, ki=1.0, kd=1.0, deadband=0.0)
    pid.update(1.0, dt=0.1)
    pid.update(1.0, dt=0.1)
    pid.reset()
    assert pid.integral == 0.0
    assert pid.last_error == 0.0


# ---------------------------------------------------------------------------
# Complementary filter
# ---------------------------------------------------------------------------

def test_complementary_pure_gyro_alpha_1():
    out = complementary_filter(1.0, 0.5, 2.0, 0.1, 100.0)
    assert abs(out - 0.7) < 1e-9


def test_complementary_pure_meas_alpha_0():
    out = complementary_filter(0.0, 0.5, 2.0, 0.1, 1.2)
    assert abs(out - 1.2) < 1e-9


def test_complementary_blend():
    out = complementary_filter(0.5, 0.0, 1.0, 1.0, 0.0)
    assert abs(out - 0.5) < 1e-9


# ---------------------------------------------------------------------------
# EKF Heading 1-D
# ---------------------------------------------------------------------------

def test_ekf_init_state():
    ekf = HeadingEkf(process_noise=0.1, meas_noise=0.2, init_heading=1.5)
    assert abs(ekf.heading - 1.5) < 1e-9
    assert abs(ekf.state[1]) < 1e-9


def test_ekf_predict_increments_heading():
    ekf = HeadingEkf(process_noise=0.01, meas_noise=0.1, init_heading=0.0)
    h = ekf.predict(gyro_rate=1.0, dt=0.1)
    assert abs(h - 0.1) < 1e-9
    assert abs(ekf.heading - 0.1) < 1e-9


def test_ekf_predict_increments_uncertainty():
    ekf = HeadingEkf(process_noise=1.0, meas_noise=0.1, init_heading=0.0)
    p00_before = ekf.p00
    ekf.predict(gyro_rate=0.0, dt=1.0)
    assert ekf.p00 > p00_before


def test_ekf_update_measured():
    ekf = HeadingEkf(process_noise=0.0, meas_noise=0.1, init_heading=0.0)
    ekf.predict(0.0, 0.1)
    h = ekf.update(0.5)
    assert 0.0 < h < 0.5


def test_ekf_wrap_around_innovation():
    ekf = HeadingEkf(process_noise=0.0, meas_noise=0.1, init_heading=math.pi - 0.1)
    ekf.predict(0.0, 0.1)
    h = ekf.update(-math.pi + 0.1)
    assert abs(h) < 10.0


def test_ekf_multiple_updates_converge():
    ekf = HeadingEkf(process_noise=0.01, meas_noise=0.01, init_heading=0.0)
    for _ in range(20):
        ekf.predict(0.0, 0.1)
        ekf.update(0.5)
    assert abs(ekf.heading - 0.5) < 0.02


# ---------------------------------------------------------------------------
# Wrap-around & CTE (delegasi ke geo)
# ---------------------------------------------------------------------------

def test_normalize_wrap_identity():
    assert abs(normalize_wrap(0.0)) < 1e-9
    assert abs(normalize_wrap(math.pi) - math.pi) < 1e-9
    assert abs(normalize_wrap(-math.pi) + math.pi) < 1e-9


def test_normalize_wrap_overflow():
    assert abs(normalize_wrap(math.pi) - math.pi) < 1e-9
    assert abs(normalize_wrap(-math.pi) + math.pi) < 1e-9


def test_cross_track_error_delegate():
    cte = cross_track_error(0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    assert abs(cte) < 0.001


if __name__ == "__main__":
    tests = [
        test_pid_basic_proportional,
        test_pid_deadband_zero_inside,
        test_pid_deadband_active_outside,
        test_pid_integral_accumulates,
        test_pid_anti_windup_integral_limit,
        test_pid_output_clamp,
        test_pid_derivative_term,
        test_pid_reset_clears_state,
        test_complementary_pure_gyro_alpha_1,
        test_complementary_pure_meas_alpha_0,
        test_complementary_blend,
        test_ekf_init_state,
        test_ekf_predict_increments_heading,
        test_ekf_predict_increments_uncertainty,
        test_ekf_update_measured,
        test_ekf_wrap_around_innovation,
        test_ekf_multiple_updates_converge,
        test_normalize_wrap_identity,
        test_normalize_wrap_overflow,
        test_cross_track_error_delegate,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"[OK] {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
    print(f"\n{passed}/{len(tests)} test passed.")
    sys.exit(0 if passed == len(tests) else 1)