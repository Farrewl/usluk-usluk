#!/usr/bin/env python3
"""
tests/test_core_ecu_link.py — Unit test C ecu_link (ctypes cross-check).

Frame 8 byte STM32: [AA 55 flags curr temp_esc temp_amb volt checksum].
"""

import os
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core


def _frame(flags, curr10, t_esc, t_amb, volt10):
    cks = (flags + curr10 + t_esc + t_amb + volt10) & 0xFF
    return bytes([0xAA, 0x55, flags, curr10, t_esc, t_amb, volt10, cks])


def test_ecu_parse_valid():
    """Frame normal: valid, nilai fisik benar."""
    raw = _frame(0x00, 100, 45, 30, 148)
    s = core.parse_ecu_frame(raw)
    assert s["valid"] is True
    assert s["kill"] is False
    assert abs(s["current_a"] - 10.0) < 1e-9
    assert abs(s["temp_esc_c"] - 45.0) < 1e-9
    assert abs(s["temp_amb_c"] - 30.0) < 1e-9
    assert abs(s["voltage_v"] - 14.8) < 1e-9


def test_ecu_parse_flags():
    """Flag kill+estop+overtemp+overcurrent terbaca."""
    raw = _frame(0x0F, 50, 80, 35, 120)
    s = core.parse_ecu_frame(raw)
    assert s["valid"] is True
    assert s["kill"] is True
    assert s["estop"] is True
    assert s["overtemp"] is True
    assert s["overcurrent"] is True


def test_ecu_parse_bad_header():
    """Header salah -> valid=False."""
    raw = bytes([0x00, 0x00, 0, 100, 45, 30, 148, 0])
    s = core.parse_ecu_frame(raw)
    assert s["valid"] is False


def test_ecu_parse_bad_checksum():
    """Checksum salah -> valid=False."""
    raw = bytes([0xAA, 0x55, 0, 100, 45, 30, 148, 0xFF])
    s = core.parse_ecu_frame(raw)
    assert s["valid"] is False


def test_ecu_build_cmd():
    """Perintah 0/1/2 lolos, nilai asing -> 0 (normal)."""
    assert core.build_ecu_cmd(0) == 0
    assert core.build_ecu_cmd(1) == 1
    assert core.build_ecu_cmd(2) == 2
    assert core.build_ecu_cmd(99) == 0


if __name__ == "__main__":
    import traceback
    tests = [test_ecu_parse_valid, test_ecu_parse_flags,
             test_ecu_parse_bad_header, test_ecu_parse_bad_checksum,
             test_ecu_build_cmd]
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
