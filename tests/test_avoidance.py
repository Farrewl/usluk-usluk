"""Test hindar-rintangan reaktif (P0-3): C vs mirror + integrasi.

Tiga lapis:
1. TestAvoidCore      — avoid_update (C) SAMA dengan mirror Python (formula
                        di core/src/avoidance.c); sweep geometri penuh.
2. TestAvoidWorker    — _YoloWorker.latest_boxes_any: box terbaru semua
                        model + penolakan hasil basi.
3. TestAvoidScan      — GroundSimNavigator._avoidance_scan: objek besar
                        di kanan frame -> bias negatif (belok kiri), aktif;
                        tanpa objek -> decay memori lalu mati.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app import aterkia_core as core
from app import settings as cfg
from app.simulator import GroundSimNavigator, _YoloWorker

# Mirror formula — SAMA dengan core/src/avoidance.c (jangan diubah salah satu
# tanpa mengubah yang lain).
W_MIN, CX_LIMIT, MEMORY_S, ACTIVE = 0.25, 0.80, 0.8, 0.05


def _ref_update(state, cx, w, dt):
    bias, mem = state.get("yaw_bias", 0.0), state.get("memory_s", 0.0)
    if w >= W_MIN and -CX_LIMIT <= cx <= CX_LIMIT:
        strength = min(1.0, max(0.0, (w - W_MIN) / (1.0 - W_MIN)))
        bias = max(-1.0, min(1.0, -strength * cx))
        mem = MEMORY_S
    elif mem > 0:
        mem -= max(0.0, dt)
        if mem <= 0:
            mem, bias = 0.0, 0.0
    return {"yaw_bias": bias, "memory_s": mem}


class TestAvoidCore(unittest.TestCase):

    def assertClose(self, got, want, tol=1e-9):
        self.assertAlmostEqual(got, want, places=9)

    def test_sweep_sama_dengan_mirror(self):
        for cx in [x / 10.0 for x in range(-10, 11)]:
            for w in [x / 10.0 for x in range(0, 11)]:
                for dt in (0.1, 0.5):
                    state = {"yaw_bias": 0.0, "memory_s": 0.0}
                    got = core.avoid_update(dict(state), cx, w, dt)
                    want = _ref_update(state, cx, w, dt)
                    self.assertClose(got["yaw_bias"], want["yaw_bias"], 1e-6)
                    self.assertClose(got["memory_s"], want["memory_s"], 1e-6)

    def test_objek_kanan_belok_kiri(self):
        out = core.avoid_update({"yaw_bias": 0.0, "memory_s": 0.0},
                                0.5, 0.5, 0.1)
        self.assertLess(out["yaw_bias"], 0.0)
        self.assertTrue(out["active"])

    def test_objek_kiri_belok_kanan(self):
        out = core.avoid_update({"yaw_bias": 0.0, "memory_s": 0.0},
                                -0.8, 0.5, 0.1)
        self.assertGreater(out["yaw_bias"], 0.0)

    def test_bias_maks_di_batas_cx_limit(self):
        # cx = ±0.8 = batas CVOID_CX_LIMIT, w = 1.0 (selebar frame):
        # strength penuh 1.0 -> bias = ∓0.8 (maks geometri valid).
        out = core.avoid_update({"yaw_bias": 0.0, "memory_s": 0.0},
                                -0.8, 1.0, 0.1)
        self.assertAlmostEqual(out["yaw_bias"], 0.8, places=9)
        out = core.avoid_update({"yaw_bias": 0.0, "memory_s": 0.0},
                                0.8, 1.0, 0.1)
        self.assertAlmostEqual(out["yaw_bias"], -0.8, places=9)
        # di luar CX_LIMIT -> bukan rintangan (bias 0)
        out = core.avoid_update({"yaw_bias": 0.0, "memory_s": 0.0},
                                -1.0, 1.0, 0.1)
        self.assertEqual(out["yaw_bias"], 0.0)

    def test_objek_kecil_atau_tepi_diabaikan(self):
        st = {"yaw_bias": 0.0, "memory_s": 0.0}
        for cx, w in ((0.0, 0.1), (0.9, 0.6), (0.5, 0.2)):
            out = core.avoid_update(dict(st), cx, w, 0.1)
            self.assertEqual(out["yaw_bias"], 0.0)
            self.assertFalse(out["active"])

    def test_memori_tahan_lalu_decay(self):
        st = {"yaw_bias": 0.0, "memory_s": 0.0}
        core.avoid_update(st, 0.5, 0.5, 0.1)     # rintangan
        self.assertEqual(st["memory_s"], MEMORY_S)
        out = core.avoid_update(st, 0.0, 0.0, 0.5)  # hilang, masih ditahan
        self.assertGreater(out["memory_s"], 0.0)
        self.assertTrue(out["active"])
        out = core.avoid_update(st, 0.0, 0.0, 0.5)  # habis -> mati
        self.assertEqual(out["yaw_bias"], 0.0)
        self.assertEqual(out["memory_s"], 0.0)
        self.assertFalse(out["active"])


class TestAvoidWorker(unittest.TestCase):

    def _bare(self, latest):
        w = _YoloWorker.__new__(_YoloWorker)
        w._latest = latest
        w._lock = threading.Lock()
        return w

    def test_box_terbaru_dari_semua_model(self):
        now = time.monotonic()
        w = self._bare({
            1: (True, None, [(0, 0.8, [10, 10, 320, 240])], now - 0.4),
            2: (True, None, [(1, 0.9, [5, 5, 200, 200])], now - 0.1),
            3: (False, None, [], now - 0.2),   # tanpa box diabaikan
        })
        boxes = w.latest_boxes_any(max_age_s=0.5)
        self.assertEqual(boxes, [(1, 0.9, [5, 5, 200, 200])],
                         "harus pakai hasil TERBARU")

    def test_hasil_basi_ditolak(self):
        now = time.monotonic()
        w = self._bare({
            1: (True, None, [(0, 0.8, [10, 10, 320, 240])], now - 60.0),
        })
        self.assertEqual(w.latest_boxes_any(max_age_s=0.5), [])


class TestAvoidScan(unittest.TestCase):

    def _nav_bare(self, boxes):
        nav = GroundSimNavigator.__new__(GroundSimNavigator)
        nav.config = cfg.Config()
        nav.config.FRAME_WIDTH = 640
        nav._yolo_worker = type("W", (), {"latest_boxes_any":
                                          lambda self, max_age_s=0.5: boxes})()
        nav.avoid_state = {"yaw_bias": 0.0, "memory_s": 0.0}
        nav._avoid_last_mono = time.monotonic() - 0.1
        return nav

    def test_rintangan_kanan_bias_negatif(self):
        nav = self._nav_bare([(0, 0.9, [300, 100, 600, 400])])  # besar, kanan
        out = nav._avoidance_scan()
        self.assertTrue(out["active"])
        self.assertLess(out["yaw_bias"], 0.0, "objek kanan -> belok kiri")

    def test_tanpa_rintangan_bias_nol(self):
        nav = self._nav_bare([])
        out = nav._avoidance_scan()
        self.assertFalse(out["active"])
        self.assertEqual(out["yaw_bias"], 0.0)

    def test_objek_kecil_tidak_menggerakkan_avoid(self):
        nav = self._nav_bare([(0, 0.9, [310, 200, 350, 260])])  # lebar 40px
        out = nav._avoidance_scan()
        self.assertFalse(out["active"])
        self.assertEqual(out["yaw_bias"], 0.0)


if __name__ == "__main__":
    unittest.main()