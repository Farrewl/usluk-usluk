#!/usr/bin/env python3
"""
tests/test_core_mode_manager.py — Unit test C mode_manager (ctypes).

Kontrak (lihat core/include/mode_manager.h):
  KILL > MANUAL > HOLD > AUTO. RC putus di tengah MANUAL -> HOLD selama
  MANUAL_LOST_HOLD_S detik, lalu AUTO. Keluar KILL -> AUTO (bukan MANUAL).
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core

HOLD_S = 1.0
DT = 0.05


def test_mode_kill_wins_and_releases_to_auto():
    """Kill menang; lepas kill -> AUTO."""
    m = core.ModeState()
    assert m.update(True, True, True, DT, HOLD_S)["mode"] == core.OP_KILL
    assert m.update(False, True, True, DT, HOLD_S)["mode"] == core.OP_MANUAL
    m.update(True, False, False, DT, HOLD_S)
    assert m.update(False, False, False, DT, HOLD_S)["mode"] == core.OP_AUTO


def test_mode_manual_needs_rc():
    """Minta manual tanpa RC sehat -> tetap AUTO."""
    m = core.ModeState()
    assert m.update(False, True, False, DT, HOLD_S)["mode"] == core.OP_AUTO
    assert m.update(False, True, True, DT, HOLD_S)["mode"] == core.OP_MANUAL


def test_mode_hold_then_auto():
    """RC putus saat MANUAL -> HOLD (tahan 1 s) -> AUTO."""
    m = core.ModeState()
    m.update(False, True, True, DT, HOLD_S)
    assert m.update(False, True, False, DT, HOLD_S)["mode"] == core.OP_HOLD
    got = core.OP_HOLD
    for _ in range(int(HOLD_S / DT) + 2):
        got = m.update(False, True, False, DT, HOLD_S)["mode"]
    assert got == core.OP_AUTO


def test_mode_hold_recovers():
    """RC pulih saat HOLD + masih minta manual -> kembali MANUAL."""
    m = core.ModeState()
    m.update(False, True, True, DT, HOLD_S)
    m.update(False, True, False, DT, HOLD_S)
    assert m.update(False, True, True, DT, HOLD_S)["mode"] == core.OP_MANUAL


if __name__ == "__main__":
    import traceback
    tests = [test_mode_kill_wins_and_releases_to_auto,
             test_mode_manual_needs_rc, test_mode_hold_then_auto,
             test_mode_hold_recovers]
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
