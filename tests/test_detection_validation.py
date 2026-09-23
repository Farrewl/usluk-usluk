"""Test app/detection_validation.py — filter pasca-YOLO untuk buoy.

Sasaran utama: deteksi yang salah class (mis. wajah operator yang dianggap
red_ball oleh YOLO) harus DITOLAK karena warnanya bukan merah/hijau pekat,
bentuknya bukan bola, atau ukurannya tak memenuhi area minimum.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import unittest

import cv2
import numpy as np

from app.detection_validation import (
    validate_buoy, color_fraction, draw_validated_boxes,
)

W, H = 640, 480


def _solid_frame(color_bgr):
    """Frame polos berwarna `color_bgr` (BGR)."""
    return np.full((H, W, 3), color_bgr, dtype=np.uint8)


def _circle_frame(color_bgr, cx=320, cy=240, radius=100):
    """Frame hitam dengan lingkaran berwarna — meniru bola/buoy."""
    frame = np.zeros((H, W, 3), dtype=np.uint8)
    cv2.circle(frame, (cx, cy), radius, color_bgr, -1)
    return frame


def _box_for_circle(cx=320, cy=240, radius=100):
    """Kotak persis mengelilingi lingkaran radius `radius`."""
    return (cx - radius, cy - radius, cx + radius, cy + radius)


# Warnah BGR khas yang dipakai di kasus uji.
RED = (0, 0, 255)
GREEN = (0, 255, 0)
GRAY = (128, 128, 128)
BLACK = (0, 0, 0)
# Kulit manusia (BGR): hue ~17-20 -> di luar pita merah ATAU hijau.
SKIN_BGR = (138, 172, 224)


class TestLolosBuoyAsli(unittest.TestCase):
    """Buoy asli (bola merah/hijau) harus LOLOS validasi."""

    def test_bola_merah_lolos(self):
        fr = _circle_frame(RED)
        self.assertTrue(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(),
            min_area=80))

    def test_bola_hijau_lolos(self):
        fr = _circle_frame(GREEN)
        self.assertTrue(validate_buoy(
            fr, cls=0, xyxy=_box_for_circle(),
            min_area=80))

    def test_bola_merah_persegi_padat_lolos(self):
        # Buoy menyentuh seluruh kotak (bukan lingkaran kena bingkai):
        # fraksi warna tinggi, bentuk bujur sangkar -> lolos.
        fr = _solid_frame(RED)
        self.assertTrue(validate_buoy(
            fr, cls=1, xyxy=(200, 200, 400, 400), min_area=80))


class TestTolakFalsePositive(unittest.TestCase):
    """Yang bukan buoy (wajah, abu-abu, bentuk lonjong, kecil) DITOLAK."""

    def test_warna_kulit_ditolak_sebagai_merah(self):
        # Wajah: warna kulit tidak masuk pita merah.
        fr = _solid_frame(SKIN_BGR)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=(200, 200, 400, 400), min_area=80))

    def test_warna_kulit_ditolak_sebagai_hijau(self):
        fr = _solid_frame(SKIN_BGR)
        self.assertFalse(validate_buoy(
            fr, cls=0, xyxy=(200, 200, 400, 400), min_area=80))

    def test_bola_hijau_bukan_merah(self):
        # Deteksi salah class: objek hijau dinyatakan merah -> TOlak.
        fr = _circle_frame(GREEN)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(), min_area=80))

    def test_bola_merah_bukan_hijau(self):
        fr = _circle_frame(RED)
        self.assertFalse(validate_buoy(
            fr, cls=0, xyxy=_box_for_circle(), min_area=80))

    def test_abu_abu_ditolak(self):
        # Saturasi 0 -> bukan warna pekat buoy.
        fr = _circle_frame(GRAY)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(), min_area=80))

    def test_hitam_ditolak(self):
        fr = _circle_frame(BLACK)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(), min_area=80))

    def test_bentuk_lonjong_ditolak(self):
        # Persegi panjang merah tinggi: |w/h - 1| > batas aspek.
        fr = np.zeros((H, W, 3), dtype=np.uint8)
        fr[50:350, 270:370] = RED  # 100x300 px
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=(270, 50, 370, 350), min_area=80))

    def test_area_kecil_ditolak(self):
        # Kotak 6x6 = 36 px < MIN_BUOY_AREA_PX (80) -> ditolak.
        fr = _circle_frame(RED, radius=3)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(radius=3), min_area=80))

    def test_class_di_luar_buoy_ditolak(self):
        fr = _solid_frame(RED)
        self.assertFalse(validate_buoy(
            fr, cls=7, xyxy=(200, 200, 400, 400), min_area=80))


class TestParameterBisaDikustom(unittest.TestCase):
    """Ambang warna/area/rasio aspek bisa diubah lewat parameter."""

    def test_ambang_fraksi_warna_lebih_ketat(self):
        # Bola merah mengisi ~78% kotak; min_color_fraction=0.9 menolak.
        fr = _circle_frame(RED)
        self.assertFalse(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(), min_area=80,
            min_color_fraction=0.9))

    def test_ambang_fraksi_warna_longgar(self):
        fr = _circle_frame(RED)
        self.assertTrue(validate_buoy(
            fr, cls=1, xyxy=_box_for_circle(), min_area=80,
            min_color_fraction=0.5))

    def test_ambang_saturasi_longgar_menerima_objek_pudar(self):
        # Merah pudar (hue 0, saturasi ~0.11): lolos bila ambang saturasi
        # diturunkan (nilai HSV terverifikasi: (0, 28, 230)).
        faded = np.full((H, W, 3), (205, 205, 230), dtype=np.uint8)
        self.assertTrue(validate_buoy(
            faded, cls=1, xyxy=(200, 200, 400, 400), min_area=80,
            min_saturation=0.1))

    def test_rasio_aspek_longgar_menerima_oval(self):
        # Oval 150x250 px: |1/1.67 - 1| = 0.4 -> lolos bila batas 0.45.
        fr = np.zeros((H, W, 3), dtype=np.uint8)
        cv2.ellipse(fr, (320, 240), (150, 250), 0, 0, 360, RED, -1)
        self.assertTrue(validate_buoy(
            fr, cls=1, xyxy=(170, -10, 470, 490), min_area=80,
            max_aspect_deviation=0.45))


class TestColorFraction(unittest.TestCase):
    def test_layer_penuh_berwarna_fraksi_1(self):
        fr = _solid_frame(RED)
        frac = color_fraction(fr, cls=1, xyxy=(0, 0, W, H),
                              min_saturation=0.55)
        self.assertAlmostEqual(frac, 1.0, places=3)

    def test_hitam_tidak_dihitung(self):
        fr = _solid_frame(BLACK)
        frac = color_fraction(fr, cls=1, xyxy=(0, 0, W, H),
                              min_saturation=0.55)
        self.assertEqual(frac, 0.0)

    def test_crop_kosong_aman(self):
        fr = _circle_frame(RED)
        self.assertEqual(color_fraction(fr, cls=1, xyxy=(0, 0, 0, 0),
                                        min_saturation=0.55), 0.0)


class TestDrawValidatedBoxes(unittest.TestCase):
    def test_menggambar_box_tanpa_error(self):
        fr = _circle_frame(RED)
        kept = [(1, 0.95, list(_box_for_circle()))]
        out = draw_validated_boxes(fr, kept, {0: "hijau", 1: "merah"})
        self.assertEqual(out.shape, fr.shape)
        self.assertEqual(out.dtype, fr.dtype)

    def test_annotasi_tidak_mengubah_frame_asli(self):
        fr = _circle_frame(RED)
        kept = [(0, 0.9, list(_box_for_circle(radius=60)))]
        original = fr.copy()
        draw_validated_boxes(fr, kept, {0: "hijau", 1: "merah"})
        self.assertTrue(np.array_equal(fr, original),
                        "frame masukan tidak boleh termutasi")


if __name__ == "__main__":
    unittest.main()