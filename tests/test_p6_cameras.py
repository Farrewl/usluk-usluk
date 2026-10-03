"""Test P6: kamera plug-and-play + Pixhawk stabil + manajer ganda.

Tanpa hardware: VideoCapture & serial dimock; yang diuji logika murni:
  - list_cameras(): parsing by-id -> UID stabil (mock listdir/readlink).
  - cache UID: beda UID -> cache dibuang; cache basi -> negosiasi ulang.
  - CameraSlot reconnect: gagal N frame -> RETRY, open lagi throttled.
  - CameraManager: NAV + BAWAH hidup bareng (mock open_camera).
  - detect_serial_ports(): list semua port (bukan 1).
  - histeresis telemetri: butuh N bagus beruntun sebelum REAL.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import sys
import unittest
from types import SimpleNamespace
from unittest import mock

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from app import camera as cam_mod
from app import mavlink_telemetry as mav_mod


class _FakeCap:
    """VideoCapture palsu: read() sesuai skrip gagal/berhasil.

    Bila skrip habis, ulangi pola terakhir (bukan sukses mendadak).
    """

    def __init__(self, reads):
        self._reads = list(reads)
        self._last = self._reads[-1] if self._reads else None
        self.released = False

    def isOpened(self):
        return True

    def read(self):
        if self._reads:
            self._last = self._reads.pop(0)
            return self._last
        if self._last is not None:
            return self._last
        import numpy as np
        return True, np.zeros((480, 640, 3), dtype=np.uint8)

    def release(self):
        self.released = True

    def get(self, prop):
        return {3: 640, 4: 480}.get(int(prop), 0)


def _frame_ok():
    import numpy as np
    return True, np.zeros((480, 640, 3), dtype=np.uint8)


class TestListCameras(unittest.TestCase):

    def test_by_id_jadi_uid_stabil(self):
        with mock.patch.object(cam_mod, "_index_may_exist",
                               side_effect=lambda i: i in (0, 2)), \
             mock.patch("os.path.isdir", return_value=True), \
             mock.patch("os.listdir",
                        return_value=["usb-Logitech_C920_ABC-video-index0",
                                      "usb-NoName_2-video-index0"]), \
             mock.patch("os.readlink",
                        side_effect=["../../video0", "../../video2"]):
            cams = cam_mod.list_cameras(max_index=3)
        self.assertEqual([c["index"] for c in cams], [0, 2])
        self.assertEqual(cams[0]["uid"],
                         "usb-Logitech_C920_ABC-video-index0")
        self.assertIn("Logitech", cams[0]["label"])

    def test_tanpa_by_id_tetap_jalan(self):
        with mock.patch.object(cam_mod, "_index_may_exist",
                               side_effect=lambda i: i == 0), \
             mock.patch("os.path.isdir", return_value=False):
            cams = cam_mod.list_cameras(max_index=2)
        self.assertEqual(len(cams), 1)
        self.assertIsNone(cams[0]["uid"])


class TestCacheUid(unittest.TestCase):

    def test_uid_berubah_buang_cache(self):
        cache = {"index": 1, "fourcc": 123, "width": 1280, "height": 720,
                 "fps": 30.0, "uid": "usb-LAMA-video-index0"}
        with mock.patch.object(cam_mod, "_load_cam_cache",
                               return_value=cache), \
             mock.patch.object(cam_mod, "_v4l_uid_for_index",
                               return_value="usb-BARU-video-index0"), \
             mock.patch.object(cam_mod, "invalidate_cam_cache") as inv, \
             mock.patch.object(cam_mod, "_probe_index_quality",
                               return_value=(1280, 720, 30.0)), \
             mock.patch.object(cam_mod, "negotiate_camera",
                               return_value=(42, 1280, 720, 30.0)), \
             mock.patch.object(cam_mod, "_save_cam_cache"):
            best = cam_mod.negotiate_best_camera(1, force_renegotiate=False)
        inv.assert_called_once()
        self.assertEqual(best[0], 1)  # negosiasi ulang jalan

    def test_cache_basi_negosiasi_ulang(self):
        cache = {"index": 0, "fourcc": 123, "width": 640, "height": 480,
                 "fps": 30.0, "uid": "usb-SAMA-video-index0"}
        dead = _FakeCap([])
        dead.isOpened = lambda: False
        with mock.patch.object(cam_mod, "_load_cam_cache",
                               return_value=cache), \
             mock.patch.object(cam_mod, "_v4l_uid_for_index",
                               return_value="usb-SAMA-video-index0"), \
             mock.patch.object(cam_mod, "_make_capture",
                               return_value=dead), \
             mock.patch.object(cam_mod, "invalidate_cam_cache") as inv, \
             mock.patch.object(cam_mod, "_probe_index_quality",
                               return_value=(640, 480, 30.0)), \
             mock.patch.object(cam_mod, "negotiate_camera",
                               return_value=(42, 640, 480, 30.0)), \
             mock.patch.object(cam_mod, "_save_cam_cache"):
            best = cam_mod.negotiate_best_camera(0, force_renegotiate=False)
        inv.assert_called_once()
        self.assertIsNotNone(best)


class TestCameraSlotReconnect(unittest.TestCase):

    def _slot(self, reads_seq):
        from app.camera_manager import CameraSlot
        caps = [_FakeCap(r) for r in reads_seq]

        def fake_open(idx):
            return caps.pop(0) if caps else _FakeCap([_frame_ok()])

        with mock.patch.object(CameraSlot, "_do_open",
                               side_effect=fake_open):
            slot = CameraSlot("NAV", 0)
        return slot

    def test_gagal_beruntun_jadi_retry_lalu_pulih(self):
        from app.camera_manager import CAM_FAIL_THRESHOLD
        import app.camera_manager as mgr_mod
        import numpy as np

        def _gagal():
            return False, None

        reads_init = [[_gagal()]]  # selalu gagal sampai threshold
        slot = self._slot(reads_init)
        # gagal terus sebanyak threshold -> RETRY + cap ditutup
        for _ in range(CAM_FAIL_THRESHOLD):
            ok, _, st = slot.read()
            self.assertFalse(ok)
            self.assertEqual(st, "RETRY")
        self.assertIsNone(slot.cap)
        # coba lagi sebelum interval -> tetap RETRY tanpa open
        ok, _, st = slot.read()
        self.assertFalse(ok)
        # majukan waktu -> reopen throttled -> REOPENED/OK
        slot.last_try_mono -= (mgr_mod.CAM_RECONNECT_INTERVAL_S + 1.0)
        with mock.patch.object(type(slot), "_do_open",
                               return_value=_FakeCap([_frame_ok()])):
            ok, frame, st = slot.read()
        self.assertIn(st, ("REOPENED", "OK"))
        self.assertIsNotNone(frame)

    def test_select_beda_index_buang_cache(self):
        from app.camera_manager import CameraSlot
        with mock.patch.object(CameraSlot, "_do_open",
                               return_value=_FakeCap([_frame_ok()])), \
             mock.patch("app.camera_manager.invalidate_cam_cache") as inv:
            slot = CameraSlot("NAV", 0)
            slot.select(2)
        inv.assert_called_once()


class TestCameraManagerDuaKamera(unittest.TestCase):

    def test_nav_dan_bawah_hidup_bareng(self):
        from app.camera_manager import CameraManager
        fake_cfg = SimpleNamespace(CAMERA_INDEX=0,
                                   WAYPOINT_PHOTO_CAMERA_INDEX=2,
                                   CAMERA_TARGET_FPS=30, CAMERA_FLIP_MODE=1)
        with mock.patch("app.camera_manager.open_camera",
                        return_value=_FakeCap([_frame_ok()])), \
             mock.patch("app.camera_manager.list_cameras",
                        return_value=[{"index": 0, "uid": "a",
                                       "label": "0: A"},
                                      {"index": 2, "uid": "b",
                                       "label": "2: B"}]):
            mgr = CameraManager(fake_cfg)
            ok1, f1, _ = mgr.read_primary()
            ok2, f2, _ = mgr.read_secondary()
            cams = mgr.rescan()
            self.assertTrue(ok1)
            self.assertIsNotNone(f1)
            # secondary boleh OK/None tergantung mock — yang penting tak crash
            self.assertTrue(ok2 or f2 is None)
            self.assertEqual(len(cams), 2)
            mgr.close()

    def test_read_primary_tanpa_kamera_teks_jujur(self):
        from app.camera_manager import CameraManager
        fake_cfg = SimpleNamespace(CAMERA_INDEX=9,
                                   WAYPOINT_PHOTO_CAMERA_INDEX=9,
                                   CAMERA_TARGET_FPS=30, CAMERA_FLIP_MODE=1)
        with mock.patch("app.camera_manager.open_camera",
                        return_value=None):
            mgr = CameraManager(fake_cfg)
            ok, fb, st = mgr.read_primary()
            self.assertFalse(ok)
            self.assertIn(st, ("NO_CAM", "RETRY"))
            self.assertIsNotNone(fb)  # fallback frame jujur
            mgr.close()


class TestDetectSerialPorts(unittest.TestCase):

    def test_kembalikan_list_bukan_satu(self):
        with mock.patch("glob.glob",
                        side_effect=[["/dev/ttyACM0"], ["/dev/ttyUSB0"],
                                     []]):
            with mock.patch("os.path.exists", return_value=True):
                ports = mav_mod.detect_serial_ports("/dev/ttyACM0")
        self.assertIsInstance(ports, list)
        self.assertIn("/dev/ttyACM0", ports)
        self.assertIn("/dev/ttyUSB0", ports)
        # preferred duluan
        self.assertEqual(ports[0], "/dev/ttyACM0")

    def test_kosong_bila_tak_ada_hardware(self):
        with mock.patch("glob.glob", return_value=[]):
            ports = mav_mod.detect_serial_ports(None)
        self.assertEqual(ports, [])


class TestHisteresisTelemetri(unittest.TestCase):

    def _nav(self, need=3):
        from app.simulator import GroundSimNavigator
        nav = GroundSimNavigator.__new__(GroundSimNavigator)
        nav.config = SimpleNamespace(TELEM_HYSTERESIS_FRAMES=need)
        nav._telem_good_streak = 0
        nav._telem_bad_streak = 0
        nav._telem_use_real = False
        return nav

    def test_butuh_N_bagus_sebelum_real(self):
        nav = self._nav(need=3)
        self.assertFalse(nav._telem_decide_source(True))
        self.assertFalse(nav._telem_decide_source(True))
        self.assertTrue(nav._telem_decide_source(True))

    def test_gagal_beruntun_kembali_mock(self):
        nav = self._nav(need=2)
        nav._telem_decide_source(True)
        nav._telem_decide_source(True)
        self.assertTrue(nav._telem_use_real)
        self.assertTrue(nav._telem_decide_source(False))  # 1 gagal: tahan
        self.assertFalse(nav._telem_decide_source(False))  # 2 gagal: mock

    def test_goyang_tak_lompat(self):
        nav = self._nav(need=3)
        for _ in range(10):  # selang-seling tak pernah cukup streak
            nav._telem_decide_source(True)
            nav._telem_decide_source(False)
        self.assertFalse(nav._telem_use_real)


if __name__ == "__main__":
    unittest.main()
