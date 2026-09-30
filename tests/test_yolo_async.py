"""Test app/yolo_async.py — worker inferensi tak-memblokir (tanpa model asli).

Model diganti stub (objek callable) yang meniru API Ultralytics
`model(frame, verbose, imgsz, half, device, conf)` -> list `results` dengan
`.boxes` berisi box {cls, xyxy, conf}. Verifikasi kontrak docblock:
submit/latest/drop/busy, frame-terbaru-menang, hasil basi, parse format.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import math
import time
import unittest

from app.yolo_async import YoloAsyncWorker


class _Box:
    def __init__(self, cls, x1, y1, x2, y2, conf=0.9):
        self.cls = [cls]
        self.xyxy = [[x1, y1, x2, y2]]
        self.conf = [conf]


class _Results:
    def __init__(self, boxes):
        self.boxes = boxes


class _StubModel:
    """Model palsu: catat panggilan, kembalikan box tetap, delay opsional."""

    def __init__(self, boxes=(), delay_s=0.0):
        self.boxes = list(boxes)
        self.delay_s = delay_s
        self.calls = 0

    def __call__(self, frame, verbose=False, imgsz=320, half=False,
                 device="cpu", conf=0.25):
        self.calls += 1
        if self.delay_s:
            time.sleep(self.delay_s)
        return [_Results(self.boxes)]


def _wait(pred, timeout_s=3.0):
    t0 = time.monotonic()
    while not pred():
        if time.monotonic() - t0 > timeout_s:
            return False
        time.sleep(0.01)
    return True


class TestSubmitLatest(unittest.TestCase):

    def setUp(self):
        self.w = YoloAsyncWorker()
        self.w.start()

    def tearDown(self):
        self.w.stop_and_wait()

    def test_hasil_masuk_format_navigator(self):
        m = _StubModel([_Box(1, 10, 20, 30, 40, 0.8)])
        self.assertTrue(self.w.submit("gate", m, object(), 0.25, 320))
        ok = _wait(lambda: self.w.latest("gate")[0])
        self.assertTrue(ok, "hasil tak kunjung tiba")
        det, age = self.w.latest("gate")
        self.assertIn(1, det)
        ent = det[1][0]
        self.assertEqual((ent["cx"], ent["cy"]), (20, 30))
        self.assertEqual(ent["box"], (10, 20, 30, 40))
        self.assertEqual(ent["area"], 20 * 20)
        self.assertAlmostEqual(ent["conf"], 0.8)
        self.assertGreaterEqual(age, 0.0)

    def test_belum_ada_hasil_kosong(self):
        det, age = self.w.latest("tak-ada")
        self.assertEqual(det, {})
        self.assertEqual(age, float("inf"))

    def test_model_none_ditolak(self):
        self.assertFalse(self.w.submit("gate", None, object(), 0.25, 320))
        det, _ = self.w.latest("gate")
        self.assertEqual(det, {})

    def test_hasil_basi_none(self):
        m = _StubModel([_Box(0, 0, 0, 10, 10)])
        self.w.submit("gate", m, object(), 0.25, 320)
        self.assertTrue(_wait(lambda: self.w.latest("gate")[0]))
        time.sleep(0.05)
        det, age = self.w.latest("gate", max_age_s=0.01)
        self.assertIsNone(det, "hasil basi harus None")
        self.assertGreater(age, 0.01)

    def test_drop_menghapus_cache(self):
        m = _StubModel([_Box(0, 0, 0, 10, 10)])
        self.w.submit("gate", m, object(), 0.25, 320)
        self.assertTrue(_wait(lambda: self.w.latest("gate")[0]))
        self.w.drop("gate")
        det, _ = self.w.latest("gate")
        self.assertEqual(det, {})

    def test_frame_terbaru_menang(self):
        # Worker lambat (0.3 s/job): submit 2 job cepat -> antrean buang lama.
        slow = _StubModel([_Box(0, 0, 0, 10, 10)], delay_s=0.3)
        self.w.submit("gate", slow, "LAMA", 0.25, 320)
        self.assertTrue(_wait(lambda: self.w.busy, timeout_s=2.0))
        # Job ke-2 masuk antrean (kapasitas 1).
        self.w.submit("gate", slow, "TENGAH", 0.25, 320)
        # Job ke-3 membuang TENGAH (LAMA sedang diproses, TENGAH menunggu).
        ok3 = self.w.submit("gate", slow, "BARU", 0.25, 320)
        self.assertTrue(ok3)
        # Tunggu 2 job selesai (LAMA + 1 antrean). Model dipanggil >= 2x dan
        # worker kembali idle — tanpa menggantung.
        self.assertTrue(_wait(lambda: slow.calls >= 2 and not self.w.busy,
                              timeout_s=5.0))
        det, _ = self.w.latest("gate")
        self.assertTrue(det, "hasil akhir harus ada")

    def test_error_model_tak_meracuni_cache(self):
        class _Rusak:
            def __call__(self, *a, **k):
                raise RuntimeError("model rusak")
        m_ok = _StubModel([_Box(0, 5, 5, 15, 15)])
        self.w.submit("gate", m_ok, object(), 0.25, 320)
        self.assertTrue(_wait(lambda: self.w.latest("gate")[0]))
        sebelum, _ = self.w.latest("gate")
        self.w.submit("gate", _Rusak(), object(), 0.25, 320)
        self.assertTrue(_wait(lambda: not self.w.busy, timeout_s=3.0))
        time.sleep(0.05)
        sesudah, _ = self.w.latest("gate")
        self.assertEqual(sesudah, sebelum)

    def test_stop_idempoten(self):
        self.w.stop_and_wait()
        self.w.stop_and_wait()  # tak boleh macet / error


class TestParseStatic(unittest.TestCase):

    def test_parse_tanpa_filter_warna(self):
        # Dua class berbeda tetap dua-duanya keluar (filter di caller).
        det = YoloAsyncWorker._parse_results(
            [_Results([_Box(0, 0, 0, 4, 4), _Box(1, 10, 10, 20, 20)])])
        self.assertEqual(set(det), {0, 1})

    def test_parse_results_kosong(self):
        self.assertEqual(YoloAsyncWorker._parse_results([]), {})
        self.assertEqual(YoloAsyncWorker._parse_results([_Results([])]), {})


if __name__ == "__main__":
    unittest.main()
