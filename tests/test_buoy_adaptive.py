"""Test P4-A (adaptif gelap) + P4-D (ambang per-class) — deteksi buoy.

Buoy hijau tenggelam duluan di remang: saturasi & value jatuh di bawah
ambang tetap (S 0.35 / V 40), hue bergeser. Test ini membuktikan dengan
frame SINTETIS (tanpa kamera/model):
  1. Hijau pudar DITOLAK jalur lama, LOLOS jalur adaptif-gelap.
  2. Siang terang: jalur adaptif IDENTIK dengan lama (tak melonggarkan).
  3. Kulit/wajah tetap DITOLAK walau adaptif aktif (hue di luar pita).
  4. Merah pekat tetap lolos di kedua jalur (tak regresi).
  5. Ambang per-class hijau < merah (default config).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import unittest

import cv2
import numpy as np

from app import settings as cfg
from app.detection_validation import (
    adaptive_thresholds, adaptive_min_value, frame_brightness,
    validate_buoy,
)

W, H = 640, 480


def _circle_frame(color_bgr, radius=100):
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.circle(frame, (W // 2, H // 2), radius, color_bgr, -1)
    return frame


def _box(radius=100):
    return (W // 2 - radius, H // 2 - radius,
            W // 2 + radius, H // 2 + radius)


# BGR sintetis: hijau pudar-remang (H~48, S~0.19, V~62) vs hijau pekat.
# Diukur via OpenCV: S 0.19 < ambang normal 0.35 (DITOLAK jalur lama),
# tapi > ambang adaptif-gelap ~0.20 (DITERIMA jalur adaptif) — persis
# kasus lapangan buoy hijau di remang.
GREEN_DIM = (47, 63, 52)     # hijau gelap-pudar khas remang
GREEN_BRIGHT = (0, 255, 0)   # hijau pekat siang
RED_BRIGHT = (0, 0, 255)     # merah pekat (kontrol tak-regresi)
SKIN_BGR = (138, 172, 224)   # kulit: hue oranye, bukan merah/hijau


class TestAdaptiveThresholds(unittest.TestCase):
    """Rumus interpolasi P4-A: terang utuh, gelap longgar, monoton."""

    def test_terang_identik_normal(self):
        sat, mult, gelap = adaptive_thresholds(200.0)
        self.assertFalse(gelap)
        self.assertAlmostEqual(sat, 0.35)
        self.assertAlmostEqual(mult, 1.0)

    def test_batas_threshold_masih_terang(self):
        _, _, gelap = adaptive_thresholds(80.0, brightness_threshold=80)
        self.assertFalse(gelap)

    def test_gelappenuh_longgar_maks(self):
        sat, mult, gelap = adaptive_thresholds(
            0.0, brightness_threshold=80, base_saturation=0.35,
            dark_saturation=0.20, color_fraction_mult=0.5)
        self.assertTrue(gelap)
        self.assertAlmostEqual(sat, 0.20)
        self.assertAlmostEqual(mult, 0.5)

    def test_monoton_transisi(self):
        prev_sat, prev_mult = 1.0, 2.0
        for v in (70, 50, 30, 10):
            sat, mult, gelap = adaptive_thresholds(float(v))
            self.assertTrue(gelap)
            self.assertLessEqual(sat, prev_sat)
            self.assertLessEqual(mult, prev_mult)
            prev_sat, prev_mult = sat, mult

    def test_min_value_monoton(self):
        self.assertEqual(adaptive_min_value(200.0), 40)
        tengah = adaptive_min_value(40.0)
        self.assertLess(tengah, 40)
        self.assertGreater(tengah, 20)
        self.assertAlmostEqual(adaptive_min_value(0.0), 20)


class TestFrameBrightness(unittest.TestCase):

    def test_hitam_nol_putih_penuh(self):
        hitam = np.zeros((H, W, 3), dtype=np.uint8)
        putih = np.full((H, W, 3), 255, dtype=np.uint8)
        self.assertAlmostEqual(frame_brightness(hitam), 0.0, delta=1.0)
        self.assertAlmostEqual(frame_brightness(putih), 255.0, delta=1.0)

    def test_frame_rusak_aman(self):
        self.assertEqual(frame_brightness(None), 255.0)

    def test_kisaran_valid(self):
        fr = _circle_frame(GREEN_DIM)
        b = frame_brightness(fr)
        self.assertGreaterEqual(b, 0.0)
        self.assertLessEqual(b, 255.0)


class TestHijauRemang(unittest.TestCase):
    """Inti P4-A: hijau pudar lolos HANYA via jalur adaptif-gelap."""

    def test_jalur_lama_menolak_hijau_pudar(self):
        fr = _circle_frame(GREEN_DIM)
        self.assertFalse(validate_buoy(
            fr, cls=0, xyxy=_box(), min_area=80,
            min_color_fraction=0.05, min_saturation=0.35,
            max_aspect_deviation=0.5))

    def test_jalur_adaptif_menerima_hijau_pudar(self):
        fr = _circle_frame(GREEN_DIM)
        bright = frame_brightness(fr)
        self.assertLess(bright, 80, "sintetis harus terbaca gelap")
        self.assertTrue(validate_buoy(
            fr, cls=0, xyxy=_box(), min_area=80,
            min_color_fraction=0.05, min_saturation=0.35,
            max_aspect_deviation=0.5, adaptive_enabled=True,
            brightness=bright))

    def test_siang_adaptif_tak_melonggarkan(self):
        # Hijau pekat siang lolos di kedua jalur (konsisten).
        fr = _circle_frame(GREEN_BRIGHT)
        kw = dict(cls=0, xyxy=_box(), min_area=80, min_color_fraction=0.05,
                  min_saturation=0.35, max_aspect_deviation=0.5)
        self.assertTrue(validate_buoy(fr, **kw))
        self.assertTrue(validate_buoy(
            fr, **kw, adaptive_enabled=True,
            brightness=frame_brightness(fr)))

    def test_kulit_tetap_ditolak_walau_adaptif(self):
        fr = _circle_frame(SKIN_BGR)
        self.assertFalse(validate_buoy(
            fr, cls=0, xyxy=_box(), min_area=80,
            min_color_fraction=0.05, min_saturation=0.35,
            max_aspect_deviation=0.5, adaptive_enabled=True,
            brightness=10.0))
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box(), min_area=80,
            min_color_fraction=0.05, min_saturation=0.35,
            max_aspect_deviation=0.5, adaptive_enabled=True,
            brightness=10.0))

    def test_merah_pekat_tak_regresi(self):
        fr = _circle_frame(RED_BRIGHT)
        kw = dict(cls=1, xyxy=_box(), min_area=80, min_color_fraction=0.05,
                  min_saturation=0.35, max_aspect_deviation=0.5)
        self.assertTrue(validate_buoy(fr, **kw))
        self.assertTrue(validate_buoy(fr, **kw, adaptive_enabled=True,
                                      brightness=30.0))


class TestPerClassDefaults(unittest.TestCase):
    """P4-D: default hijau lebih longgar dari merah + key terdaftar."""

    def test_hijau_lebih_longgar(self):
        c = cfg.Config()
        self.assertLess(c.BUOY_CONF_THRESHOLD_GREEN,
                        c.BUOY_CONF_THRESHOLD_RED)
        self.assertLess(c.BUOY_CONF_SMALL_THRESHOLD_GREEN,
                        c.BUOY_CONF_SMALL_THRESHOLD_RED)
        self.assertLessEqual(c.BUOY_MIN_COLOR_FRACTION_GREEN,
                             c.BUOY_MIN_COLOR_FRACTION_RED)
        self.assertLessEqual(c.BUOY_MIN_SATURATION_GREEN,
                             c.BUOY_MIN_SATURATION_RED)

    def test_key_baru_di_allowlist(self):
        for key in ("BUOY_CONF_THRESHOLD_GREEN",
                    "BUOY_CONF_SMALL_THRESHOLD_GREEN",
                    "BUOY_CONF_THRESHOLD_RED",
                    "BUOY_CONF_SMALL_THRESHOLD_RED",
                    "BUOY_MIN_COLOR_FRACTION_GREEN",
                    "BUOY_MIN_COLOR_FRACTION_RED",
                    "BUOY_MIN_SATURATION_GREEN",
                    "BUOY_MIN_SATURATION_RED",
                    "BUOY_ADAPTIVE_ENABLED",
                    "BUOY_BRIGHTNESS_THRESHOLD",
                    "BUOY_ADAPTIVE_MIN_SATURATION",
                    "BUOY_ADAPTIVE_MIN_VALUE",
                    "BUOY_ADAPTIVE_COLOR_FRACTION_MULT"):
            self.assertIn(key, cfg.TUNING_PARAM_KEYS)


if __name__ == "__main__":
    unittest.main()
