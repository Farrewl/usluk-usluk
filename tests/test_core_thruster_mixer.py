#!/usr/bin/env python3
"""
tests/test_core_thruster_mixer.py — Unit test C thruster_mixer (ctypes cross-check).

Urutan operasi identik app/aterkia_core.mix_diff_drive <-> core/src/thruster_mixer.c.
Toleransi: hasil C == Python bit-per-bit.
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core


def test_mix_diff_drive_center():
    """surge=0, yaw=0 -> kiri=kanan=1500"""
    l, r = core.mix_diff_drive(0.0, 0.0)
    assert l == 1500 and r == 1500


def test_mix_diff_drive_forward():
    """surge=+1.0, yaw=0 -> kiri=kanan=1900"""
    l, r = core.mix_diff_drive(1.0, 0.0)
    assert l == 1900 and r == 1900


def test_mix_diff_drive_reverse():
    """surge=-1.0, yaw=0 -> kiri=kanan=1100"""
    l, r = core.mix_diff_drive(-1.0, 0.0)
    assert l == 1100 and r == 1100


def test_mix_diff_drive_yaw_right():
    """surge=0, yaw=+1.0 -> kiri=1100, kanan=1900"""
    l, r = core.mix_diff_drive(0.0, 1.0)
    assert l == 1100 and r == 1900


def test_mix_diff_drive_yaw_left():
    """surge=0, yaw=-1.0 -> kiri=1900, kanan=1100"""
    l, r = core.mix_diff_drive(0.0, -1.0)
    assert l == 1900 and r == 1100


def test_mix_diff_drive_combined():
    """surge=0.5, yaw=0.2 -> cek nilai pasti"""
    l, r = core.mix_diff_drive(0.5, 0.2)
    # span = 400, center=1500
    # left = 1500 + 0.5*400 - 0.2*400 = 1500 + 200 - 80 = 1620
    # right = 1500 + 200 + 80 = 1780
    assert l == 1620 and r == 1780


def test_mix_diff_drive_clamp_input():
    """input > 1.0 di-clamp ke 1.0"""
    l, r = core.mix_diff_drive(2.0, -2.0)
    # surge clamped to +1.0, yaw clamped to -1.0
    # left = 1500 + 1.0*400 - (-1.0)*400 = 2300 → clamped to 1900
    # right = 1500 + 1.0*400 + (-1.0)*400 = 1500
    assert l == 1900 and r == 1500


if __name__ == "__main__":
    import traceback
    tests = [
        test_mix_diff_drive_center,
        test_mix_diff_drive_forward,
        test_mix_diff_drive_reverse,
        test_mix_diff_drive_yaw_right,
        test_mix_diff_drive_yaw_left,
        test_mix_diff_drive_combined,
        test_mix_diff_drive_clamp_input,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"[OK] {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
            traceback.print_exc()
    print(f"\n{passed}/{len(tests)} test passed.")
    sys.exit(0 if passed == len(tests) else 1)