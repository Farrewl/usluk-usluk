#!/usr/bin/env python3
"""
tests/test_core_rc_decode.py — Unit test C rc_decode (ctypes cross-check).

Kontrak (lihat core/include/rc_decode.h):
  - PWM valid 900..2100 µs; normalisasi (pwm-1500)/500 di-clamp [-1..1].
  - Stick invalid -> rc_ok=0 (failsafe). Nomor channel salah -> rc_ok=0.
  - Switch mode/deadman > 1500 = aktif (invalid = non-aktif, bukan gagal).
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core


def test_rc_center_neutral():
    """Semua 1500 -> surge/yaw 0, switch mati, rc_ok=1."""
    o = core.rc_decode([1500] * 16)
    assert o["surge"] == 0.0 and o["yaw"] == 0.0
    assert o["mode_manual"] is False and o["deadman"] is False
    assert o["rc_ok"] is True


def test_rc_full_deflection():
    """Throttle 2000 -> surge +1; yaw 1000 -> -1; switch aktif."""
    ch = [1500] * 16
    ch[2] = 2000  # CH3 throttle
    ch[3] = 1000  # CH4 yaw
    ch[4] = 1800  # CH5 mode
    ch[6] = 1900  # CH7 deadman
    o = core.rc_decode(ch)
    assert abs(o["surge"] - 1.0) < 1e-9
    assert abs(o["yaw"] - (-1.0)) < 1e-9
    assert o["mode_manual"] is True and o["deadman"] is True
    assert o["rc_ok"] is True


def test_rc_invalid_stick_failsafe():
    """Stick di luar 900..2100 -> rc_ok=0, output 0 (kapal diam)."""
    ch = [1500] * 16
    ch[2] = 500  # kabel putus / frame rusak
    o = core.rc_decode(ch)
    assert o["rc_ok"] is False
    assert o["surge"] == 0.0 and o["yaw"] == 0.0


def test_rc_bad_channel_config():
    """Nomor channel di luar 1..n -> rc_ok=0 (konfigurasi salah)."""
    o = core.rc_decode([1500] * 8, ch_throttle=9)
    assert o["rc_ok"] is False


if __name__ == "__main__":
    import traceback
    tests = [test_rc_center_neutral, test_rc_full_deflection,
             test_rc_invalid_stick_failsafe, test_rc_bad_channel_config]
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
