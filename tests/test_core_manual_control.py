#!/usr/bin/env python3
"""
tests/test_core_manual_control.py — Unit test C manual_control (ctypes).

Kontrak (lihat core/include/manual_control.h):
  - Tanpa enabled/deadman/rc_ok -> output 0 + memori nol (lepas = diam).
  - Deadband: |x| < deadband -> 0. Expo 0 = linear pasca-rescale.
  - Batas gas: output <= MANUAL_MAX_*. Rate-limit: |step| <= rate*dt.
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core

DB = 0.05
EXPO = 0.3
RATE = 2.0
DT = 0.05
MAX_S = 0.6
MAX_Y = 0.7


def test_manual_inactive_zeroes():
    """Syarat tak lengkap -> 0 & memori di-nol-kan."""
    m = core.ManualState()
    o = m.update(0.8, 0.5, True, False, True, DB, EXPO, RATE, DT, MAX_S, MAX_Y)
    assert o == {"surge": 0.0, "yaw": 0.0, "active": False}
    o = m.update(0.8, 0.5, True, True, False, DB, EXPO, RATE, DT, MAX_S, MAX_Y)
    assert o["active"] is False


def test_manual_active_bounded():
    """Stick penuh -> output aktif tapi <= batas gas."""
    m = core.ManualState()
    o = m.update(1.0, -1.0, True, True, True, DB, EXPO, RATE, DT, MAX_S, MAX_Y)
    assert o["active"] is True
    assert abs(o["surge"]) <= MAX_S + 1e-9
    assert abs(o["yaw"]) <= MAX_Y + 1e-9


def test_manual_deadband():
    """Stick di dalam deadband -> 0 walau aktif."""
    m = core.ManualState()
    o = m.update(0.02, -0.02, True, True, True, DB, EXPO, RATE, DT,
                 MAX_S, MAX_Y)
    assert o["surge"] == 0.0 and o["yaw"] == 0.0
    assert o["active"] is True  # syarat lengkap, hanya kurva = 0


def test_manual_rate_limit():
    """Langkah pertama <= rate*dt (tidak melonjak)."""
    m = core.ManualState()
    o = m.update(1.0, 1.0, True, True, True, DB, EXPO, RATE, DT, MAX_S, MAX_Y)
    assert abs(o["surge"]) <= RATE * DT + 1e-9
    assert abs(o["yaw"]) <= RATE * DT + 1e-9


if __name__ == "__main__":
    import traceback
    tests = [test_manual_inactive_zeroes, test_manual_active_bounded,
             test_manual_deadband, test_manual_rate_limit]
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
