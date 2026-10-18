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
except ImportError:
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
    """Pegang kamera primary + secondary.

    Primary (NAV) dibaca blocking dari loop navigator sesuai kebutuhan.

    Secondary (BAWAH) dibaca TERUS-MENERUS oleh thread daemon terpisah
    (`_sec_thread`) lalu disimpan sebagai cache kecil. `read_secondary()`
    HANYA mengembalikan cache (non-blocking) - loop navigasi tak PERNAH
    menunggu `cap.read()` kamera bawah. Ini kuncinya: baca kamera bawah
    yang blocking (apalagi bandwidth USB terbagi 2 UVC) dulu menahan loop
    NAV sehingga fps jatuh ke ~2. Lock dipisah (`_lock_prim` vs
    `_lock_sec`) supaya baca bawah tak pernah menahan baca primary.
    """

    def __init__(self, config):
        self.config = config
        self._lock_prim = threading.Lock()
        self._lock_sec = threading.Lock()
        prim_idx = getattr(config, "CAMERA_INDEX", 0)
        sec_idx = getattr(config, "WAYPOINT_PHOTO_CAMERA_INDEX", 1)
        fps = getattr(config, "CAMERA_TARGET_FPS", 30)
        flip = getattr(config, "CAMERA_FLIP_MODE", 1)
        self.primary = CameraSlot("NAV", prim_idx, target_fps=fps,
                                  auto_highest=True, flip_mode=flip)

        # Kamera bawah dibuka cap fps rendah + cache kecil untuk tekan
        # bandwidth USB saat berbagi controller dengan NAV.
        sec_fps = int(getattr(config, "SECONDARY_FPS", 15) or 15)
        sec_w = int(getattr(config, "SECONDARY_CACHE_WIDTH", 640) or 640)
        self._sec_fps = max(1, sec_fps)
        self._cache_width = max(160, sec_w)
        self.secondary = CameraSlot("BAWAH", sec_idx, width=640,
                                    height=480, target_fps=self._sec_fps,
                                    flip_mode=flip)
        self._reported_retry = {"NAV": False, "BAWAH": False}

        # Cache frame bawah + thread pembaca (di balik lock kecil).
        self._cache_lock = threading.Lock()
        self._sec_latest = None
        self._sec_latest_ts = 0.0
        self._sec_running = threading.Event()
        self._sec_running.set()
        self._sec_thread = threading.Thread(
            target=self._sec_loop, daemon=True, name="sec-reader")
        self._sec_thread.start()

    # ---- thread reader kamera bawah --------------------------------
    def _sec_loop(self):
        """Baca kamera bawah terus-menerus -> cache. Tak menyentuh NAV."""
        interval = 1.0 / float(self._sec_fps)
        while self._sec_running.is_set():
            t0 = time.monotonic()
            try:
                with self._lock_sec:
                    ok, frame, _st = self.secondary.read()
                if ok and frame is not None:
                    small = self._downscale(frame, self._cache_width)
                    with self._cache_lock:
                        self._sec_latest = small
                        self._sec_latest_ts = time.monotonic()
            except Exception:
                pass
            dt = time.monotonic() - t0
            if dt < interval:
                time.sleep(interval - dt)

    @staticmethod
    def _downscale(frame, max_width):
        """Resize bila lebih lebar dari max_width (INTER_AREA)."""
        try:
            h, w = frame.shape[:2]
            if w > max_width:
                scale = max_width / float(w)
                nh, nw = int(h * scale), int(w * scale)
                return cv2.resize(frame, (nw, nh),
                                  interpolation=cv2.INTER_AREA)
        except Exception:
            pass
        return frame

    def read_primary(self, fb_width=1280, fb_height=720):
        """Frame navigasi + teks status jujur untuk fallback."""
        with self._lock_prim:
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
        """Ambil cache kamera bawah (NON-BLOCKING) atau None bila belum ada.

        Tidak memanggil cap.read() - frame datang dari thread `_sec_loop`.
        `max_width` hanya membatasi lebar hasil; cache disimpan pada
        lebar `SECONDARY_CACHE_WIDTH` (640).
        """
        with self._cache_lock:
            frame = self._sec_latest
        if frame is None:
            return False, None, "NO_CAM"
        if max_width and frame.shape[1] > max_width:
            frame = self._downscale(frame, max_width)
        return True, frame, "OK"

    def secondary_fresh(self, max_age_s=1.0):
        """True bila cache kamera bawah masih segar (< max_age_s)."""
        with self._cache_lock:
            ts = self._sec_latest_ts
        return ts > 0 and (time.monotonic() - ts) <= max_age_s



    def rescan(self):
        """Daftar kamera tanpa buka frame (untuk dropdown Scan)."""
        try:
            return list_cameras()
        except Exception:
            return []

    def select_primary(self, index):
        with self._lock_prim:
            ok = self.primary.select(index)
            try:
                self.config.CAMERA_INDEX = int(index)
            except Exception:
                pass
        print(f"[CAM] NAV -> idx {index} "
              f"({'OK' if ok else 'GAGAL'})")
        return ok

    def select_secondary(self, index):
        with self._lock_sec:
            ok = self.secondary.select(index)
            try:
                self.config.WAYPOINT_PHOTO_CAMERA_INDEX = int(index)
            except Exception:
                pass
        print(f"[CAM] BAWAH -> idx {index} "
              f"({'OK' if ok else 'GAGAL'})")
        return ok

    def status(self):
        with self._lock_prim:
            prim = self.primary.status()
        with self._lock_sec:
            sec = self.secondary.status()
        return {"primary": prim, "secondary": sec}

    def close(self):
        try:
            self._sec_running.clear()
            th = self._sec_thread
            if th is not None and th.is_alive():
                th.join(timeout=1.0)
        except Exception:
            pass
        with self._lock_prim:
            self.primary.close()
        with self._lock_sec:
            self.secondary.close()


    @property
    def cap(self):
        return self.primary.cap
