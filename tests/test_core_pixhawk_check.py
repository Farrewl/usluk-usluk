#!/usr/bin/env python3
"""
tests/test_core_pixhawk_check.py — Unit test app/pixhawk_check.py.

Mengunci bug nyata: `master.param_request_read_send` TIDAK ADA (harus lewat
`master.mav.param_request_read_send`) dan `param_id` harus bytes — dulu
exception-nya ditelan sehingga SEMUA parameter tampil "TIDAK DIJAWAB".

Juga mengunci pemilihan spec berdasar firmware (PX4 vs ArduPilot).
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import pixhawk_check as pc  # noqa: E402


class _Msg:
    def __init__(self, name, value):
        self.param_id = name
        self.param_value = value


class _FakeMavObj:
    """Meniru master.mav: HARUS bytes, kalau str -> TypeError (seperti asli)."""

    def __init__(self, state):
        self.state = state

    def param_request_read_send(self, ts, tc, param_id, param_index,
                                force_mavlink1=False):
        if not isinstance(param_id, bytes):
            raise TypeError("must be str or None, not bytes")
        name = param_id.decode()
        self.state["sent"].append(name)
        if name in self.state["answers"]:
            self.state["queue"].append(_Msg(name, self.state["answers"][name]))


class _FakeMaster:
    def __init__(self, autopilot=12, answers=None):
        self.target_system = 1
        self.target_component = 1
        self.autopilot = autopilot
        self.state = {"sent": [], "answers": answers or {}, "queue": []}
        self.mav = _FakeMavObj(self.state)

    def recv_match(self, type=None, blocking=False, timeout=0):
        if self.state["queue"]:
            return self.state["queue"].pop(0)
        return None


class TestFirmwareSpec(unittest.TestCase):
    def test_mapping(self):
        self.assertEqual(pc.firmware_name(12), "PX4")
        self.assertEqual(pc.firmware_name(3), "ArduPilot")
        self.assertEqual(pc.firmware_name(None), "?")
        self.assertTrue(pc.spec_path_for(12).endswith("pixhawk_required_px4.txt"))
        self.assertTrue(pc.spec_path_for(3).endswith("pixhawk_required.txt"))
        # unknown -> fallback ArduPilot (tak crash)
        self.assertTrue(pc.spec_path_for(99).endswith("pixhawk_required.txt"))

    def test_px4_spec_exists_and_parses(self):
        spec = pc.load_spec(pc.spec_path_for(12))
        names = [s["name"] for s in spec]
        for n in ("NAV_DLL_ACT", "NAV_RCL_ACT", "BAT_LOW_THR"):
            self.assertIn(n, names)
        self.assertTrue(all(s["min"] is not None for s in spec))


class TestRequestAndCheck(unittest.TestCase):
    def test_request_uses_bytes_via_master_mav(self):
        m = _FakeMaster()
        ok = pc._request_param(m, "NAV_DLL_ACT")
        self.assertTrue(ok)
        # FakeMavObj melempar TypeError bila bukan bytes -> membuktikan bytes.
        self.assertEqual(m.state["sent"], ["NAV_DLL_ACT"])

    def test_check_params_reads_answers(self):
        answers = {"NAV_DLL_ACT": 2.0, "NAV_RCL_ACT": 1.0, "BAT_LOW_THR": 0.12}
        m = _FakeMaster(answers=answers)
        bounds = {"NAV_DLL_ACT": (1.0, 3.0), "NAV_RCL_ACT": (1.0, 3.0),
                  "BAT_LOW_THR": (0.05, 0.30)}
        spec = [dict(name=n, min=bounds[n][0], max=bounds[n][1], critical=True)
                for n in answers]
        results = pc.check_params(m, spec, poll_s=0.001, timeout_s=0.5)
        self.assertEqual([r["status"] for r in results], ["ok"] * 3)
        self.assertEqual(pc.summarize(results), "OK")

    def test_detect_autopilot_from_attribute(self):
        self.assertEqual(pc.detect_autopilot(_FakeMaster(autopilot=12)), 12)

    def test_decode_int_param_from_float_bits(self):
        # PX4 mengemas int ke bit float; param_type INT32=6 -> decode balik.
        from struct import pack, unpack

        def mk(i, t):
            class M:
                param_value = unpack("<f", pack("<i", i))[0]
                param_type = t
            return M()

        self.assertEqual(pc._decode_param(mk(2, 6)), 2.0)
        self.assertEqual(pc._decode_param(mk(0, 6)), 0.0)

        class F:
            param_value = 0.15
            param_type = 9
        self.assertAlmostEqual(pc._decode_param(F()), 0.15, places=6)

    def test_summarize_flags_unread_and_violation(self):
        results = [
            dict(name="A", min=1.0, max=2.0, critical=True,
                 value=None, status="unread"),
            dict(name="B", min=1.0, max=2.0, critical=False,
                 value=9.0, status="violation"),
        ]
        s = pc.summarize(results)
        self.assertIn("A TIDAK DIJAWAB (KRITIS)", s)
        self.assertIn("B=9.0 luar [1.0..2.0] (advisory)", s)


if __name__ == "__main__":
    unittest.main()
