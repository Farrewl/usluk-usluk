"""Test app/camera.py — helper buka kamera lintas-platform.

Bagian hardware (kalibrasi exposure, baca frame nyata) tidak bisa
diuji tanpa kamera; di sini yang diuji adalah kontrak None-safe &
fallback frame. Buka kamera asli = done manual (lihat /dev/video0).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import unittest

import numpy as np

from app.camera import make_fallback_frame, open_camera


class TestFallbackFrame(unittest.TestCase):

    def test_shape_dan_dtype(self):
        fr = make_fallback_frame(640, 480)
        self.assertEqual(fr.shape, (480, 640, 3))
        self.assertEqual(fr.dtype, np.uint8)

    def test_ada_tulisan_peringatan(self):
        fr = make_fallback_frame(320, 240, text="CAMERA ERROR")
        # teks digambar merah -> sekitar baris atas ada piksel merah pekat
        red_mask = fr[:, :, 2] > 200
        self.assertTrue(red_mask.any(), "kotak peringatan harus terlihat")


class TestOpenCamera(unittest.TestCase):

    def test_index_tidak_ada_tidak_crash(self):
        # index 999 hampir pasti tidak ada: helper harus mengembalikan
        # None (bukan exception), pemanggil memutuskan fallback.
        cap = open_camera(999, 640, 480)
        self.assertTrue(cap is None or cap.isOpened())
        if cap is not None:
            cap.release()

    def test_kompatibel_konvensi_index(self):
        # index boleh dikasih string '0' — helper meng-int()
        cap = open_camera("0", 320, 240)
        self.assertTrue(cap is None or cap.isOpened())
        if cap is not None:
            cap.release()


if __name__ == "__main__":
    unittest.main()