#!/usr/bin/env python3
"""
tests/test_core_arbitrator.py — Unit test C arbitrator (ctypes cross-check).

Prioritas: KILL > MANUAL > AUTO. Output di-clamp [-1..1].
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core


def test_arb_kill_wins():
    """Kill aktif -> output 0, source KILLED, walau manual/auto ada isi."""
    o = core.arbitrate(True, True, 0.8, 0.5, 0.6, -0.3)
    assert o["surge"] == 0.0 and o["yaw"] == 0.0
    assert o["source"] == core.SRC_KILLED
    assert o["killed"] is True


def test_arb_manual_beats_auto():
    """Manual aktif (tanpa kill) -> pakai manual."""
    o = core.arbitrate(False, True, 0.8, 0.5, 0.6, -0.3)
    assert abs(o["surge"] - 0.8) < 1e-9
    assert abs(o["yaw"] - 0.5) < 1e-9
    assert o["source"] == core.SRC_MANUAL
    assert o["killed"] is False


def test_arb_auto_fallback():
    """Tanpa kill & manual -> pakai auto."""
    o = core.arbitrate(False, False, 0.8, 0.5, 0.6, -0.3)
    assert abs(o["surge"] - 0.6) < 1e-9
    assert abs(o["yaw"] - (-0.3)) < 1e-9
    assert o["source"] == core.SRC_AUTO


def test_arb_clamp():
    """Input di luar [-1..1] di-clamp."""
    o = core.arbitrate(False, True, 5.0, -5.0, 0.0, 0.0)
    assert o["surge"] == 1.0 and o["yaw"] == -1.0


if __name__ == "__main__":
    import traceback
    tests = [test_arb_kill_wins, test_arb_manual_beats_auto,
             test_arb_auto_fallback, test_arb_clamp]
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
