"""app/camera_manager.py — Pegang 2 kamera USB hidup bareng + reconnect.

Latar: 1 kapal bawa 2 kamera (navigasi depan + bawah air). Kode lama hanya
pegang 1 `self.cap`; kamera kedua dibuka eksklusif sesaat (navigasi
di-release dulu) sehingga dua view tak bisa jalan bareng. Modul ini
memegang dua VideoCapture persisten:

  primary   — navigasi (resolusi negosiasi, ~30 fps, untuk YOLO + GUI)
  secondary — bawah air (640x480, untuk view kecil + foto WP8)

Fitur plug-and-play:
  - `read_*()` menghitung gagal beruntun; >= CAM_FAIL_THRESHOLD maka
    status LONGKAP dan coba reopen tiap CAM_RECONNECT_INTERVAL_S
    (throttle, non-blocking) — colok belakangan / cabut-colok USB
    pulih sendiri tanpa restart.
  - `rescan()` = list_cameras() tanpa membuka frame (cepat, untuk dropdown).
  - `select_primary/seconday(index)` ganti kamera saat runtime (dipakai
    tombol Scan GUI); cache basi otomatis dibuang (lihat camera.py).

Thread-safety: loop baca jalan di thread navigator, GUI memanggil
select_* dari thread Qt — semua mutasi cap dikunci Lock.
"""

import threading
import time

try:
    import cv2
    _CV2_OK = True
except ImportError:  # pragma: no cover - mesin tanpa cv2
    cv2 = None
    _CV2_OK = False

from .camera import (
    CAM_FAIL_THRESHOLD,
    CAM_RECONNECT_INTERVAL_S,
    flip_frame_if_needed,
    invalidate_cam_cache,
    list_cameras,
    make_fallback_frame,
    open_camera,
)


class CameraSlot:
    """Satu slot kamera (primary / secondary) + state reconnect."""

    def __init__(self, name, index, width=None, height=None,
                 target_fps=None, auto_highest=False, flip_mode=1):
        self.name = str(name)
        self.want_index = int(index)
        self.width = width
        self.height = height
        self.target_fps = target_fps
        self.auto_highest = bool(auto_highest)
        self.flip_mode = int(flip_mode)
        self.cap = None
        self.fail_count = 0
        self.last_try_mono = 0.0
        self.last_ok_mono = 0.0
        self.label = ""
        self.open()

    # ------------------- buka / tutup -------------------

    def _do_open(self, index):
        """Buka index; return cap atau None (cache basi dibuang)."""
        try:
            if self.auto_highest:
                cap = open_camera(int(index), target_fps=self.target_fps,
                                  auto_highest=True)
            elif self.width and self.height:
                cap = open_camera(int(index), self.width, self.height)
            else:
                cap = open_camera(int(index))
        except Exception:
            cap = None
        if cap is None or not cap.isOpened():
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            return None
        return cap

    def open(self):
        """Buka kamera sesuai want_index; catat label untuk GUI."""
        self.close()
        self.cap = self._do_open(self.want_index)
        self.fail_count = 0
        self.last_try_mono = time.monotonic()
        if self.cap is not None:
            try:
                w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                self.label = f"{self.name}: idx {self.want_index} {w}x{h}"
            except Exception:
                self.label = f"{self.name}: idx {self.want_index}"
            self.last_ok_mono = time.monotonic()
            return True
        self.label = f"{self.name}: idx {self.want_index} (TIDAK ADA)"
        return False

    def close(self):
        cap, self.cap = self.cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    def select(self, index):
        """Ganti kamera saat runtime (buang cache bila pindah index)."""
        try:
            index = int(index)
        except (TypeError, ValueError):
            return False
        if index == self.want_index and self.cap is not None:
            return True
        if index != self.want_index:
            invalidate_cam_cache()
        self.want_index = index
        return self.open()

    # ------------------- baca -------------------

    def read(self):
        """Baca 1 frame; (ret, frame, status_str).

        status: 'OK' | 'NO_CAM' (belum pernah terbuka) |
                'RETRY' (putus, coba lagi throttled) |
                'REOPENED' (baru pulih — pemanggil boleh log sekali).
        """
        if self.cap is None:
            now = time.monotonic()
            if now - self.last_try_mono >= CAM_RECONNECT_INTERVAL_S:
                self.last_try_mono = now
                if self.open():
                    return True, self._grab(), "REOPENED"
            return False, None, ("NO_CAM" if self.fail_count == 0
                                 else "RETRY")
        try:
            ret, frame = self.cap.read()
        except Exception:
            ret, frame = False, None
        if ret and frame is not None:
            self.fail_count = 0
            self.last_ok_mono = time.monotonic()
            try:
                frame = flip_frame_if_needed(frame, self.flip_mode)
            except Exception:
                pass
            return True, frame, "OK"
        self.fail_count += 1
        if self.fail_count >= CAM_FAIL_THRESHOLD:
            # Anggap putus (cabut USB / reset bus saat Pixhawk dicolok):
            # tutup + coba lagi throttled agar loop tak spam open.
            self.close()
            self.last_try_mono = time.monotonic()
            return False, None, "RETRY"
        return False, None, "RETRY"

    def _grab(self):
        try:
            ret, frame = self.cap.read()
            if ret and frame is not None:
                return flip_frame_if_needed(frame, self.flip_mode)
        except Exception:
            pass
        return None

    def status(self):
        """Dict ringkas untuk GUI/chip (tanpa frame)."""
        opened = self.cap is not None
        return {
            "name": self.name,
            "index": self.want_index,
            "opened": opened,
            "label": self.label,
            "fail_count": self.fail_count,
        }


class CameraManager:
    """Pegang primary + secondary; API dipakai GroundSimNavigator."""

    def __init__(self, config):
        self.config = config
        self._lock = threading.Lock()
        prim_idx = getattr(config, "CAMERA_INDEX", 0)
        sec_idx = getattr(config, "WAYPOINT_PHOTO_CAMERA_INDEX", 1)
        fps = getattr(config, "CAMERA_TARGET_FPS", 30)
        flip = getattr(config, "CAMERA_FLIP_MODE", 1)
        self.primary = CameraSlot("NAV", prim_idx, target_fps=fps,
                                  auto_highest=True, flip_mode=flip)
        # Secondary ringan: 640x480 tanpa negosiasi penuh (hemat 1-2 dtk).
        self.secondary = CameraSlot("BAWAH", sec_idx, width=640,
                                    height=480, flip_mode=flip)
        self._reported_retry = {"NAV": False, "BAWAH": False}

    # ------------------- API loop -------------------

    def read_primary(self, fb_width=1280, fb_height=720):
        """Frame navigasi + teks status jujur untuk fallback."""
        with self._lock:
            ok, frame, st = self.primary.read()
        if ok and frame is not None:
            self._reported_retry["NAV"] = False
            return True, frame, "OK"
        if st == "REOPENED":
            self._reported_retry["NAV"] = False
            if frame is not None:
                return True, frame, "REOPENED"
        if st in ("RETRY", "NO_CAM"):
            if not self._reported_retry["NAV"]:
                self._reported_retry["NAV"] = True
                print(f"[CAM] NAV idx={self.primary.want_index} "
                      "terputus — mencoba lagi tiap "
                      f"{CAM_RECONNECT_INTERVAL_S:.0f} dtk...")
            text = ("KAMERA TERPUTUS — mencoba lagi..."
                    if st == "RETRY" else "KAMERA BELUM ADA — colok USB / Scan")
            fb = (make_fallback_frame(fb_width, fb_height, text=text)
                  if _CV2_OK else None)
            return False, fb, st
        fb = (make_fallback_frame(fb_width, fb_height,
                                  text="KAMERA ERROR") if _CV2_OK else None)
        return False, fb, st

    def read_secondary(self, max_width=320):
        """Frame kecil bawah air (downscale) atau None bila tak ada."""
        with self._lock:
            ok, frame, st = self.secondary.read()
        if not ok or frame is None:
            return False, None, st
        try:
            h, w = frame.shape[:2]
            if w > max_width:
                scale = max_width / float(w)
                nh, nw = int(h * scale), int(w * scale)
                frame = cv2.resize(frame, (nw, nh),
                                   interpolation=cv2.INTER_AREA)
        except Exception:
            pass
        return True, frame, "OK"

    # ------------------- API GUI -------------------

    def rescan(self):
        """Daftar kamera tanpa buka frame (untuk dropdown Scan)."""
        try:
            return list_cameras()
        except Exception:
            return []

    def select_primary(self, index):
        with self._lock:
            ok = self.primary.select(index)
            try:
                self.config.CAMERA_INDEX = int(index)
            except Exception:
                pass
        print(f"[CAM] NAV -> idx {index} "
              f"({'OK' if ok else 'GAGAL'})")
        return ok

    def select_secondary(self, index):
        with self._lock:
            ok = self.secondary.select(index)
            try:
                self.config.WAYPOINT_PHOTO_CAMERA_INDEX = int(index)
            except Exception:
                pass
        print(f"[CAM] BAWAH -> idx {index} "
              f"({'OK' if ok else 'GAGAL'})")
        return ok

    def status(self):
        with self._lock:
            return {
                "primary": self.primary.status(),
                "secondary": self.secondary.status(),
            }

    def close(self):
        with self._lock:
            self.primary.close()
            self.secondary.close()

    # Kompat: kode lama pegang `.cap` langsung (navigator.py jalur lama).
    @property
    def cap(self):
        return self.primary.cap
