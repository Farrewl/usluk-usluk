"""Test PixhawkCheck: spec file + logika baca/lapor parameter failsafe.

Tanpa hardware: master diganti stub yang membalas PARAM_VALUE terhadap
PARAM_REQUEST_READ. Yang diuji (lih. app/pixhawk_check.py):
- load_spec membaca spec nyata config/pixhawk_required.txt
- semua nilai dalam range -> seluruh status 'ok'
- nilai di luar range  -> status 'violation' + flag critical terjaga
- nama tak dijawab     -> status 'unread'
- summarize() -> 'OK' / daftar pelanggaran

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import unittest
from types import SimpleNamespace

from app.pixhawk_check import load_spec, check_params, summarize

SPEC_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "config", "pixhawk_required.txt")


class _ParamStub:
    """Master tiruan: menjawab PARAM_VALUE hanya untuk nama di `values`."""

    def __init__(self, values):
        self.values = dict(values)
        self.requested = []
        self.replied = set()
        self.target_system = 1
        self.target_component = 1

    def param_request_read_send(self, *args):
        self.requested.append(args)

    def recv_match(self, type=None, blocking=False):
        if type != "PARAM_VALUE":
            return None
        for name in self.values:
            if name not in self.replied:
                self.replied.add(name)
                return SimpleNamespace(param_id=name,
                                       param_value=self.values[name])
        return None


def _good_values(spec):
    """Nilai sah: tengah rentang tiap param."""
    out = {}
    for p in spec:
        if p["min"] is None or p["max"] is None:
            out[p["name"]] = 1.0
        else:
            out[p["name"]] = (p["min"] + p["max"]) / 2.0
    return out


class TestSpecFile(unittest.TestCase):

    def test_spec_terbaca_dan_lengkap(self):
        spec = load_spec(SPEC_PATH)
        names = {p["name"] for p in spec}
        self.assertGreaterEqual(len(spec), 6, "minimal 6 param wajib")
        for wajib in ("FS_GCS_ENABLE", "FS_THR_ENABLE",
                      "FS_CRASH_CHECK", "ARMING_CHECK"):
            self.assertIn(wajib, names, f"{wajib} wajib ada di spec")
        fs = next(p for p in spec if p["name"] == "FS_GCS_ENABLE")
        self.assertTrue(fs["critical"], "FS_GCS_ENABLE harus kritis")
        self.assertEqual((fs["min"], fs["max"]), (1.0, 1.0))


class TestCheckParams(unittest.TestCase):

    def setUp(self):
        self.spec = load_spec(SPEC_PATH)

    def test_semua_ok(self):
        master = _ParamStub(_good_values(self.spec))
        results = check_params(master, self.spec)
        self.assertTrue(all(r["status"] == "ok" for r in results))
        self.assertEqual(len(master.requested), len(self.spec),
                         "setiap param harus di-request")

    def test_violation_tertangkap(self):
        vals = _good_values(self.spec)
        vals["FS_GCS_ENABLE"] = 0.0          # mati -> bahaya
        vals["FS_CRASH_CHECK"] = 999.9       # di luar range
        results = check_params(_ParamStub(vals), self.spec)
        by_name = {r["name"]: r for r in results}
        self.assertEqual(by_name["FS_GCS_ENABLE"]["status"], "violation")
        self.assertEqual(by_name["FS_CRASH_CHECK"]["status"], "violation")
        self.assertTrue(by_name["FS_GCS_ENABLE"]["critical"],
                        "flag critical harus dipertahankan")

    def test_unread_saat_tidak_dijawab(self):
        vals = _good_values(self.spec)
        vals.pop("ARMING_CHECK")             # Pixhawk tidak punya/balas
        results = check_params(_ParamStub(vals), self.spec)
        arm = next(r for r in results if r["name"] == "ARMING_CHECK")
        self.assertEqual(arm["status"], "unread")
        self.assertIsNone(arm["value"])

    def test_summarize_ok_dan_pelanggaran(self):
        self.assertEqual(summarize(
            check_params(_ParamStub(_good_values(self.spec)), self.spec)), "OK")
        vals = _good_values(self.spec)
        vals["FS_GCS_ENABLE"] = 0.0
        s = summarize(check_params(_ParamStub(vals), self.spec))
        self.assertIn("FS_GCS_ENABLE", s)
        self.assertIn("KRITIS", s)


if __name__ == "__main__":
    unittest.main()