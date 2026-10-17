"""app/monitor_export.py — ekspor monitoring via FILE BRIDGE (tanpa web).

Kapal menulis kondisi lengkapnya ke `data/monitor/` supaya **dashboard web
terpisah** (repo lain, jalan di mini-PC yang sama) tinggal membacanya:

    data/monitor/telemetry.json   snapshot penuh, ditulis atomik @ MONITOR_HZ
    data/monitor/frame.jpg        frame kamera NAV (annotasi YOLO) @ FRAME_HZ
    data/monitor/frame_bawah.jpg  kamera bawah (bila ada)
    data/monitor/photos.json      indeks foto misi + geo-tag
    data/monitor/events.jsonl     log kejadian (state/failsafe/gate) append

Tanpa HTTP, tanpa port, tanpa dependency web. Navigasi memanggil
`publish(...)` dari loop-nya; penulisan JPEG dilakukan di thread terpisah
dengan buffer 1-slot (frame lama dibuang) agar loop tak terblokir.

Semua bisa dimatikan dengan `MONITOR_ENABLED=False` (env `ASV_MONITOR=0`).
"""

import json
import os
import threading
import time

import cv2

from . import settings as cfg
from .telemetry_snapshot import build_snapshot

_instance = None
_instance_lock = threading.Lock()


class MonitorExporter:
    def __init__(self, config):
        self.config = config
        self.enabled = bool(getattr(config, "MONITOR_ENABLED", True))
        self.dir = str(getattr(config, "MONITOR_DIR", "data/monitor"))
        self.hz = max(0.1, float(getattr(config, "MONITOR_HZ", 5.0)))
        self.frame_enabled = bool(getattr(config, "MONITOR_FRAME", True))
        self.frame_hz = max(0.0, float(getattr(config, "MONITOR_FRAME_HZ", 10.0)))
        self.frame_width = int(getattr(config, "MONITOR_FRAME_WIDTH", 640))
        self.jpeg_quality = int(getattr(config, "MONITOR_JPEG_QUALITY", 70))
        self.events_enabled = bool(getattr(config, "MONITOR_EVENTS", True))

        self._state_lock = threading.Lock()
        self._mission = {}
        self._vision = {}
        self._camera = {}
        self._last_photo = None
        self._photos = []

        self._snap_t = 0.0
        self._frame_lock = threading.Lock()
        self._pending = None
        self._running = bool(self.enabled)
        self._frame_thread = None
        self._start = time.time()

        if self.enabled:
            try:
                os.makedirs(self.dir, exist_ok=True)
            except Exception:
                self.enabled = False
        if self.enabled:
            self._load_photos()
            if self.frame_enabled:
                self._frame_thread = threading.Thread(
                    target=self._frame_loop, daemon=True, name="monitor-frames")
                self._frame_thread.start()

    # ---- API publik -------------------------------------------------
    def publish(self, pkt, frame=None, frame_secondary=None, mission=None,
                vision=None, camera=None):
        if not self.enabled:
            return
        if mission is not None or vision is not None or camera is not None:
            with self._state_lock:
                if mission is not None:
                    self._mission = mission
                if vision is not None:
                    self._vision = vision
                if camera is not None:
                    self._camera = camera
        if frame is not None and self.frame_enabled:
            with self._frame_lock:
                self._pending = (frame, frame_secondary)
        now = time.time()
        if (now - self._snap_t) >= (1.0 / self.hz):
            self._snap_t = now
            self._write_snapshot(pkt)

    def register_photo(self, info):
        if not self.enabled or not info:
            return
        with self._state_lock:
            self._last_photo = info
            path = info.get("path") or info.get("file")
            self._photos = [p for p in self._photos
                            if (p.get("path") or p.get("file")) != path]
            self._photos.append(info)
            photos = list(self._photos)
        self._atomic_write_json(os.path.join(self.dir, "photos.json"), photos)
        self.event("photo", photo_kind=(info.get("kind")), path=path)

    def event(self, name, **kw):
        if not self.enabled or not self.events_enabled:
            return
        rec = {"ts": time.time(), "kind": name}
        rec.update(kw)
        try:
            with open(os.path.join(self.dir, "events.jsonl"), "a",
                      encoding="utf-8") as f:
                f.write(json.dumps(rec, default=str) + "\n")
        except Exception:
            pass

    def stop(self):
        self._running = False
        th = self._frame_thread
        if th is not None and th.is_alive():
            try:
                th.join(timeout=1.0)
            except Exception:
                pass
        self._frame_thread = None

    # ---- internal ----------------------------------------------------
    def _load_photos(self):
        try:
            path = os.path.join(self.dir, "photos.json")
            if os.path.exists(path):
                with open(path, "r", encoding="utf-8") as f:
                    data = json.load(f)
                if isinstance(data, list):
                    self._photos = data
        except Exception:
            self._photos = []

    def _write_snapshot(self, pkt):
        try:
            with self._state_lock:
                mission = dict(self._mission)
                vision = self._vision
                camera = self._camera
                last_photo = self._last_photo
            snap = build_snapshot(
                pkt, self.config, mission=mission, vision=vision,
                camera=camera, last_photo=last_photo,
                extra={"uptime_s": time.time() - self._start})
            self._atomic_write_json(
                os.path.join(self.dir, "telemetry.json"), snap)
        except Exception:
            pass

    def _frame_loop(self):
        dt = (1.0 / self.frame_hz) if self.frame_hz > 0 else 0.1
        while self._running:
            got = None
            with self._frame_lock:
                if self._pending is not None:
                    got = self._pending
                    self._pending = None
            if got is None:
                time.sleep(min(0.05, dt))
                continue
            frame, sec = got
            try:
                self._write_jpeg(os.path.join(self.dir, "frame.jpg"), frame)
                if sec is not None:
                    self._write_jpeg(
                        os.path.join(self.dir, "frame_bawah.jpg"), sec)
            except Exception:
                pass
            time.sleep(dt)

    def _write_jpeg(self, path, img):
        if img is None:
            return
        out = img
        if self.frame_width and img.shape[1] > self.frame_width:
            scale = self.frame_width / float(img.shape[1])
            out = cv2.resize(img, (self.frame_width,
                                   max(1, int(img.shape[0] * scale))),
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", out,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
        if not ok:
            return
        self._atomic_write_bytes(path, buf.tobytes())

    def _atomic_write_json(self, path, obj):
        try:
            data = json.dumps(obj, default=str).encode("utf-8")
        except Exception:
            return
        self._atomic_write_bytes(path, data)

    @staticmethod
    def _atomic_write_bytes(path, data):
        tmp = path + ".tmp"
        try:
            with open(tmp, "wb") as f:
                f.write(data)
            os.replace(tmp, path)
        except Exception:
            try:
                if os.path.exists(tmp):
                    os.remove(tmp)
            except Exception:
                pass


def get_exporter(config=None):
    """Singleton per proses (hanya satu backend aktif sekali jalan)."""
    global _instance
    with _instance_lock:
        if _instance is None:
            _instance = MonitorExporter(config if config is not None else cfg)
        return _instance


def publish(pkt, frame=None, frame_secondary=None, mission=None, vision=None,
            camera=None):
    get_exporter().publish(pkt, frame, frame_secondary, mission, vision, camera)


def register_photo(info):
    get_exporter().register_photo(info)


def event(kind, **kw):
    get_exporter().event(kind, **kw)


def shutdown():
    global _instance
    with _instance_lock:
        inst = _instance
        _instance = None
    if inst is not None:
        try:
            inst.stop()
        except Exception:
            pass
