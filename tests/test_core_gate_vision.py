"""Test kunci — buktikan C (gate_vision.c) memberi hasil SAMA dengan
Python referensi (app/gate_sequencer.py + criteria geometri validate_buoy).

Cara kerja (pola test_core_nav_math / test_core_state_machine):
  1. Compile core/src/gate_vision.c -> libgate_vision.so (gcc, sementara).
  2. Bungkus fungsi & struct C dengan ctypes (urutan field identik — header
     gate_vision.h).
  3. Bandingkan tiap primitif:
       - gv_collect_pairs   vs collect_gate_pairs   (himpunan pasangan)
       - gv_estimate_distance vs estimate_gate_distance
       - gv_buoy_ok/_fail  vs referensi geometri Python
       - gv_seq_* (latch/lewat/lost) vs GateSequencer.update pada trace
         multi-frame (termasuk reset & amnesia memori).
  4. Jalankan trace yang sama pada GateSequencer Python & struct C.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import ctypes
import math
import os
import random
import shutil
import subprocess
import tempfile
import unittest

from app.gate_sequencer import (
    GateSequencer,
    collect_gate_pairs,
    estimate_gate_distance,
)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_C = os.path.join(REPO, "core", "src", "gate_vision.c")
INC_C = os.path.join(REPO, "core", "include")

GV_MAX_DET = 64
GV_MAX_PAIRS = 64
GV_MEM_PER_CLS = 32

_D = ctypes.c_double
_I = ctypes.c_int


class CBall(ctypes.Structure):
    _fields_ = [("cx", _D), ("cy", _D), ("area", _D)]


class CPair(ctypes.Structure):
    _fields_ = [("red_idx", _I), ("green_idx", _I)]


class CMem(ctypes.Structure):
    _fields_ = [("cx", _D), ("cy", _D), ("area", _D), ("unseen", _I)]


class CSeq(ctypes.Structure):
    _fields_ = [
        ("pass_distance_m", _D),
        ("lost_tolerance_frames", _I),
        ("gate_width_m", _D),
        ("focal_length_px", _D),
        ("midpoint_match_px", _D),
        ("track_match_px", _D),
        ("track_boost", _D),
        ("has_active", _I),
        ("active_mid_x", _D),
        ("active_mid_y", _D),
        ("active_dist", _D),
        ("active_red", _I),
        ("active_green", _I),
        ("lost_frames", _I),
        ("is_passed", _I),
        ("mem", CMem * 2 * GV_MEM_PER_CLS),
        ("mem_n", _I * 2),
    ]


def _build_lib():
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        raise unittest.SkipTest("gcc tidak tersedia di mesin ini")
    tmp = tempfile.mkdtemp(prefix="libgate_vision_")
    so = os.path.join(tmp, "libgate_vision.so")
    subprocess.run([cc, "-O2", "-fPIC", "-shared", "-I", INC_C,
                    SRC_C, "-o", so, "-lm"],
                   check=True, capture_output=True)
    lib = ctypes.CDLL(so)

    lib.gv_collect_pairs.argtypes = [
        ctypes.POINTER(CBall), _I, ctypes.POINTER(CBall), _I,
        _D, _D, ctypes.POINTER(CPair)]
    lib.gv_collect_pairs.restype = _I
    lib.gv_estimate_distance.argtypes = [_D, _D, _D]
    lib.gv_estimate_distance.restype = _D
    lib.gv_buoy_ok.argtypes = [_D, _D, _D, _D]
    lib.gv_buoy_ok.restype = _I
    lib.gv_buoy_fail.argtypes = [_D, _D, _D, _D]
    lib.gv_buoy_fail.restype = _I
    lib.gv_seq_init.argtypes = [ctypes.POINTER(CSeq), _D, _I,
                                _D, _D, _D, _D, _D]
    lib.gv_seq_init.restype = None
    lib.gv_seq_reset.argtypes = [ctypes.POINTER(CSeq)]
    lib.gv_seq_reset.restype = None
    lib.gv_seq_update.argtypes = [
        ctypes.POINTER(CSeq), ctypes.POINTER(CBall), _I,
        ctypes.POINTER(CBall), _I, ctypes.POINTER(CPair), _I, _D,
        ctypes.POINTER(_D), ctypes.POINTER(_D),
        ctypes.POINTER(_D), ctypes.POINTER(_I)]
    lib.gv_seq_update.restype = _I
    return lib


def _balls_to_c(balls):
    n = min(len(balls), GV_MAX_DET)
    arr = (CBall * max(n, 1))()
    for i in range(n):
        arr[i].cx = float(balls[i].get("cx", 0.0))
        arr[i].cy = float(balls[i].get("cy", 0.0))
        arr[i].area = float(balls[i].get("area", 0.0))
    return arr, n


def _ball(cx, cy, area):
    return {"cx": float(cx), "cy": float(cy), "area": float(area)}


class TestCollectPairsVsPython(unittest.TestCase):
    """gv_collect_pairs == collect_gate_pairs (himpunan pasangan)."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_lib()

    def _c_pairs(self, red, green, valign, ratio):
        r_arr, n_r = _balls_to_c(red)
        g_arr, n_g = _balls_to_c(green)
        out = (CPair * GV_MAX_PAIRS)()
        n = self.lib.gv_collect_pairs(r_arr, n_r, g_arr, n_g,
                                      float(valign), float(ratio), out)
        return [(out[i].red_idx, out[i].green_idx) for i in range(n)]

    def _py_pairs(self, red, green, valign, ratio):
        pairs = collect_gate_pairs(red, green, valign, ratio)
        return [(red.index(r), green.index(g)) for r, g in pairs]

    def test_pasangan_polos_satu(self):
        red = [_ball(100, 200, 500)]
        green = [_ball(150, 205, 520)]
        c = self._c_pairs(red, green, 75.0, 0.5)
        py = self._py_pairs(red, green, 75.0, 0.5)
        self.assertEqual(c, [(0, 0)])
        self.assertEqual(c, py)

    def test_sejajar_dan_luas_tidak_sama_tolak(self):
        # dy besar tolak; luas tak mirip tolak.
        red = [_ball(100, 200, 500), _ball(300, 400, 100)]
        green = [_ball(150, 205, 520), _ball(500, 100, 500)]
        c = self._c_pairs(red, green, 75.0, 0.5)
        py = self._py_pairs(red, green, 75.0, 0.5)
        self.assertEqual(len(c), 1)
        self.assertEqual(c, py)

    def test_luas_nol_tolak(self):
        red = [_ball(100, 200, 0)]
        green = [_ball(150, 200, 500)]
        self.assertEqual(self._c_pairs(red, green, 75.0, 0.5), [])
        self.assertEqual(self._py_pairs(red, green, 75.0, 0.5), [])

    def test_sejajar_dan_acak_banyak(self):
        rng = random.Random(1234)
        for _ in range(200):
            red = [_ball(rng.uniform(0, 640), rng.uniform(0, 480),
                         rng.uniform(0, 2000)) for _ in range(rng.randint(0, 5))]
            green = [_ball(rng.uniform(0, 640), rng.uniform(0, 480),
                           rng.uniform(0, 2000)) for _ in range(rng.randint(0, 5))]
            valign = rng.choice([10.0, 50.0, 90.0, 200.0])
            ratio = rng.choice([0.0, 0.3, 0.5, 0.9, 1.0])
            c = sorted(self._c_pairs(red, green, valign, ratio))
            py = sorted(self._py_pairs(red, green, valign, ratio))
            self.assertEqual(c, py,
                             f"red={red} green={green} valign={valign} "
                             f"ratio={ratio}")


class TestDistanceVsPython(unittest.TestCase):
    """gv_estimate_distance == estimate_gate_distance."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_lib()

    def test_pinhole_dan_inf(self):
        for pw, gw, fl in [(100.0, 1.0, 400.0), (5.0, 1.0, 400.0),
                           (0.0, 1.0, 400.0), (50.0, 2.5, 800.0),
                           (50.0, 1.0, 0.0), (50.0, 0.0, 400.0)]:
            c = self.lib.gv_estimate_distance(pw, gw, fl)
            pair = ({'cx': 0.0, 'cy': 0.0}, {'cx': pw, 'cy': 0.0})
            py = estimate_gate_distance(pair, gw, fl)
            self.assertEqual(math.isinf(c), math.isinf(py), f"{pw},{gw},{fl}")
            if not math.isinf(c):
                self.assertAlmostEqual(c, py, places=9, msg=f"{pw},{gw},{fl}")


class TestBuoyGeometryVsPython(unittest.TestCase):
    """gv_buoy_ok/_fail == referensi geometri Python (area/aspek)."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_lib()

    @staticmethod
    def _py_fail(w, h, min_area, max_dev):
        """Referensi Python (spec validate_buoy sebelum delegasi ke C)."""
        if w <= 2 or h <= 2:
            return 1
        if w * h < min_area:
            return 2
        if abs(w / h - 1.0) > max_dev:
            return 3
        return 0

    def test_grid_dan_acak(self):
        rng = random.Random(99)
        for w in [0, 1, 2, 3, 4, 10, 16, 30, 100]:
            for h in [0, 1, 2, 3, 4, 10, 16, 30, 100]:
                for min_area in [0, 16, 80, 500]:
                    for max_dev in [0.0, 0.3, 0.5, 0.9]:
                        c_fail = self.lib.gv_buoy_fail(
                            float(w), float(h), float(min_area), float(max_dev))
                        c_ok = self.lib.gv_buoy_ok(
                            float(w), float(h), float(min_area), float(max_dev))
                        py_fail = self._py_fail(w, h, min_area, max_dev)
                        self.assertEqual(
                            c_fail, py_fail,
                            f"fail({w},{h},min={min_area},dev={max_dev})")
                        self.assertEqual(
                            c_ok, 0 if py_fail else 1,
                            f"ok({w},{h},min={min_area},dev={max_dev})")
        # acak float
        for _ in range(300):
            w = rng.uniform(-5, 300)
            h = rng.uniform(-5, 300)
            min_area = rng.uniform(0, 500)
            max_dev = rng.uniform(0, 1.2)
            c_fail = self.lib.gv_buoy_fail(w, h, min_area, max_dev)
            py_fail = self._py_fail(w, h, min_area, max_dev)
            self.assertEqual(c_fail, py_fail, f"{w},{h},{min_area},{max_dev}")


class TestSequencerVsPython(unittest.TestCase):
    """gv_seq_* == GateSequencer.update pada trace multi-frame."""

    @classmethod
    def setUpClass(cls):
        cls.lib = _build_lib()

    def _drive_c(self, frames, init=None, reset_at=None, **kw):
        init = init or {}
        st = CSeq()
        self.lib.gv_seq_init(
            ctypes.byref(st),
            float(kw.get("pass_distance_m", 1.2)),
            int(kw.get("lost_tolerance_frames", 5)),
            float(kw.get("gate_width_m", 1.0)),
            float(kw.get("focal_length_px", 400.0)),
            float(kw.get("midpoint_match_px", 6.0)),
            float(kw.get("track_match_px", 30.0)),
            float(kw.get("track_boost", 1.5)))
        out = []
        for i, (red, green, center_y, valign, ratio) in enumerate(frames):
            if reset_at is not None and i in reset_at:
                self.lib.gv_seq_reset(ctypes.byref(st))
            r_arr, n_r = _balls_to_c(red)
            g_arr, n_g = _balls_to_c(green)
            prs = self._collect_pairs_c(red, green, valign, ratio)
            parr = (CPair * max(len(prs), 1))()
            for k, (ri, gi) in enumerate(prs):
                parr[k].red_idx = ri
                parr[k].green_idx = gi
            mx, my, dist = _D(0.0), _D(0.0), _D(float("inf"))
            ps = _I(0)
            has = self.lib.gv_seq_update(
                ctypes.byref(st), r_arr, n_r, g_arr, n_g,
                parr, len(prs), float(center_y),
                ctypes.byref(mx), ctypes.byref(my),
                ctypes.byref(dist), ctypes.byref(ps))
            if not has:
                out.append((None, None, float("inf"), False))
            else:
                out.append((mx.value, my.value, dist.value, bool(ps.value)))
        return out

    def _collect_pairs_c(self, red, green, valign, ratio):
        r_arr, n_r = _balls_to_c(red)
        g_arr, n_g = _balls_to_c(green)
        out = (CPair * GV_MAX_PAIRS)()
        n = self.lib.gv_collect_pairs(r_arr, n_r, g_arr, n_g,
                                      float(valign), float(ratio), out)
        return [(out[i].red_idx, out[i].green_idx) for i in range(n)]

    def _drive_py(self, frames, reset_at=None, **kw):
        seq = GateSequencer(**kw)
        out = []
        for i, (red, green, center_y, valign, ratio) in enumerate(frames):
            if reset_at is not None and i in reset_at:
                seq.reset()
            pairs = collect_gate_pairs(red, green, valign, ratio)
            out.append(seq.update(pairs, center_y))
        return out

    def _assert_sama(self, frames, tag, reset_at=None, **kw):
        py = self._drive_py(frames, reset_at=reset_at, **kw)
        c = self._drive_c(frames, reset_at=reset_at, **kw)
        self.assertEqual(len(py), len(c), f"{tag}: panjang beda")
        for i, (a, b) in enumerate(zip(py, c)):
            # (mid_x, mid_y, dist, is_passed) per frame harus identik.
            if a[0] is None or a[1] is None:
                self.assertIsNone(b[0], f"{tag} frame {i}: py={a} C={b}")
                self.assertIsNone(b[1], f"{tag} frame {i}: py={a} C={b}")
                continue
            self.assertAlmostEqual(a[0], b[0], places=6,
                                   msg=f"{tag} frame {i} mid_x")
            self.assertAlmostEqual(a[1], b[1], places=6,
                                   msg=f"{tag} frame {i} mid_y")
            if math.isinf(a[2]):
                self.assertTrue(math.isinf(b[2]), f"{tag} frame {i} dist")
            else:
                self.assertAlmostEqual(a[2], b[2], places=6,
                                       msg=f"{tag} frame {i} dist")
            self.assertEqual(a[3], b[3], f"{tag} frame {i} is_passed")

    def test_skenario_latch_saat_hilang(self):
        # Satu gate terlihat, lalu hilang beberapa frame (latch), lalu muncul lagi.
        g = (_ball(500, 300, 500), _ball(600, 305, 520))
        frames = [
            ([g[0]], [g[1]], 360.0, 75.0, 0.5),  # target terbentuk
            ([g[0]], [g[1]], 360.0, 75.0, 0.5),  # sama, latch
            ([], [], 360.0, 75.0, 0.5),          # hilang 1 (tahan)
            ([], [], 360.0, 75.0, 0.5),          # hilang 2 (tahan)
            ([g[0]], [g[1]], 360.0, 75.0, 0.5),  # muncul lagi
        ]
        self._assert_sama(frames, "latch-hilang")

    def test_skenario_lewat_lompat_ke_gate_depan(self):
        # Dua gate: "near" (lebar piksel kecil -> jarak jauh) dan "far"
        # (lebar piksel besar -> jarak dekat, di bawah pass_distance).
        near = (_ball(500, 380, 900), _ball(700, 385, 920))  # lebar 200 -> jauh
        far = (_ball(100, 200, 500), _ball(150, 205, 520))    # lebar 50 -> dekat
        frames = [
            ([far[0]], [far[1]], 360.0, 75.0, 0.5),   # target = gate dekat
            ([far[0]], [far[1]], 360.0, 75.0, 0.5),   # latch
            ([near[0], far[0]], [near[1], far[1]], 360.0, 75.0, 0.5),  # muncul
        ]
        self._assert_sama(frames, "lewat-lompat")

    def test_skenario_berlalu_penuh(self):
        # mid_y > center => langsung "lewat", tetap chosen passed.
        b = (_ball(500, 500, 500), _ball(600, 505, 520))
        frames = [([b[0]], [b[1]], 360.0, 75.0, 0.5)]
        self._assert_sama(frames, "berlalu-penuh")

    def test_skenario_hilang_terus_kosong(self):
        b = (_ball(500, 300, 500), _ball(600, 305, 520))
        frames = [
            ([b[0]], [b[1]], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),  # > toleransi -> kosong
        ]
        self._assert_sama(frames, "hilang-terus", lost_tolerance_frames=5)

    def test_skenario_reset(self):
        b = (_ball(500, 300, 500), _ball(600, 305, 520))
        frames = [
            ([b[0]], [b[1]], 360.0, 75.0, 0.5),
            ([], [], 360.0, 75.0, 0.5),
            ([b[0]], [b[1]], 360.0, 75.0, 0.5),  # reset di sini
            ([], [], 360.0, 75.0, 0.5),          # latch hilang karena reset
        ]
        self._assert_sama(frames, "reset", reset_at={2})

    def test_skenario_acak_multi_frame(self):
        rng = random.Random(7)
        frames = []
        for _ in range(60):
            n_r = rng.randint(0, 3)
            n_g = rng.randint(0, 3)
            red = [_ball(rng.uniform(50, 600), rng.uniform(100, 420),
                         rng.uniform(100, 1500)) for _ in range(n_r)]
            green = [_ball(rng.uniform(50, 600), rng.uniform(100, 420),
                           rng.uniform(100, 1500)) for _ in range(n_g)]
            frames.append((red, green, 360.0, 75.0, 0.5))
        self._assert_sama(frames, "acak-multiframe")


if __name__ == "__main__":
    unittest.main()
