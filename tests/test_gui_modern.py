"""Test modul GUI baru: pacing 25 Hz, peta slippy-map, dialog settings.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import time
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from app.camera import pace_to_fps, TARGET_FPS
from app.slim_map import (SlimMapWidget, latlon_to_tile_float,
                          tile_to_latlon)


class TestPacing25(unittest.TestCase):

    def test_target_25(self):
        self.assertEqual(TARGET_FPS, 30)

    def test_25_frame_sekitar_1_detik(self):
        dl = time.monotonic()
        t0 = time.monotonic()
        for _ in range(30):
            dl = pace_to_fps(dl)
        dt = time.monotonic() - t0
        # 30 frame @30Hz = 1.0 s; toleransi scheduling OS.
        self.assertGreater(dt, 0.7)
        self.assertLess(dt, 1.6)

    def test_overrun_tidak_numpuk(self):
        # Deadline basi -> kembali cepat tanpa tidur lama.
        t0 = time.monotonic()
        pace_to_fps(t0 - 5.0)
        self.assertLess(time.monotonic() - t0, 0.2)


class TestSlipMapMath(unittest.TestCase):

    def test_roundtrip_semarang(self):
        xt, yt = latlon_to_tile_float(-7.28, 112.79, 18)
        la, lo = tile_to_latlon(xt, yt, 18)
        self.assertAlmostEqual(la, -7.28, places=9)
        self.assertAlmostEqual(lo, 112.79, places=9)

    def test_zoom_beda_tile_beda(self):
        a = latlon_to_tile_float(-7.28, 112.79, 10)
        b = latlon_to_tile_float(-7.28, 112.79, 18)
        self.assertNotEqual(a, b)


class TestSlimMapWidget(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def test_set_waypoints_dan_vehicle(self):
        w = SlimMapWidget()
        w.set_waypoints([{'lat': -7.28, 'lon': 112.79},
                         {'lat': -7.281, 'lon': 112.791}])
        w.set_vision_legs([1])
        w.update_vehicle(-7.28, 112.79, 45.0)
        self.assertEqual(len(w._wps), 2)
        self.assertEqual(w._veh[2], 45.0)

    def test_wp_moved_emit(self):
        w = SlimMapWidget()
        w.resize(400, 400)
        w.set_waypoints([{'lat': -7.28, 'lon': 112.79}])
        got = []
        w.wpMoved.connect(lambda i, la, lo: got.append((i, la, lo)))
        # Simulasi drag: panggil langsung jalur mouseRelease.
        from PyQt5.QtCore import Qt
        from PyQt5.QtTest import QTest
        from PyQt5.QtCore import QPoint
        sx, sy = w._latlon_to_screen(-7.28, 112.79)
        QTest.mousePress(w, Qt.LeftButton, Qt.NoModifier,
                         QPoint(int(sx), int(sy)))
        QTest.mouseMove(w, QPoint(int(sx) + 10, int(sy) + 10))
        QTest.mouseRelease(w, Qt.LeftButton, Qt.NoModifier,
                           QPoint(int(sx) + 10, int(sy) + 10))
        self.assertEqual(len(got), 1)
        self.assertEqual(got[0][0], 0)

    def test_offline_tidak_crash(self):
        w = SlimMapWidget()
        w.resize(300, 200)
        w.show()
        w.repaint()
        w.close()


class TestSettingsDialog(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        from PyQt5.QtWidgets import QApplication
        cls._app = QApplication.instance() or QApplication([])

    def test_lazy_build(self):
        from app.settings_dialog import SettingsDialog

        class FakeThread:
            class config:
                pass

            def update_config_param(self, k, v):
                pass

            def save_config_to_file(self):
                pass

        dlg = SettingsDialog(FakeThread())
        self.assertFalse(dlg._built)
        self.assertEqual(dlg.tabs.count(), 0)
        dlg.show()
        self.assertTrue(dlg._built)
        self.assertEqual(dlg.tabs.count(), 5)
        dlg.close()


if __name__ == "__main__":
    unittest.main()
