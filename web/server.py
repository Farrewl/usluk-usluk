"""web/server.py — server dashboard (stdlib http.server, SSE + MJPEG).

Nol dependency baru: pakai `http.server` + `cv2` (sudah ada untuk kamera).
- GET /                -> static/index.html
- GET /static/<file>   -> aset statis
- GET /config          -> identitas tim, lintasan, geometri grid/waypoint
- GET /events          -> Server-Sent Events, snapshot telemetri @ SSE_HZ
- GET /video           -> MJPEG dari frame NAV terakhir @ VIDEO_HZ
- GET /frame.jpg       -> satu frame JPEG terbaru (dipakai klien polling)
- GET /captures        -> daftar foto misi + geo-tag (sidecar JSON)
- GET /captures/<dir>/<name> -> file gambar capture

Data dibaca dari app/state_bus.py (diisi navigator / mock feeder).
"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from app import state_bus
from app import settings as cfg

STATIC_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
_CAPTURE_DIRS = {
    "captures": cfg.CAPTURES_DIR,
    "waypoint_captures": cfg.WAYPOINT_PHOTO_DIR,
}
_MIME = {
    ".html": "text/html; charset=utf-8",
    ".js": "application/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".json": "application/json; charset=utf-8",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".jpeg": "image/jpeg",
    ".svg": "image/svg+xml",
    ".ico": "image/x-icon",
}


def list_captures(limit=40):
    """Kumpulkan sidecar JSON foto misi (terbaru dulu)."""
    items = []
    for folder in _CAPTURE_DIRS.values():
        if not os.path.isdir(folder):
            continue
        for name in os.listdir(folder):
            if not name.endswith(".json"):
                continue
            try:
                with open(os.path.join(folder, name), "r", encoding="utf-8") as f:
                    info = json.load(f)
                info["url"] = f"/captures/{os.path.basename(folder)}/{info.get('file', '')}"
                items.append(info)
            except Exception:
                continue
    items.sort(key=lambda x: x.get("ts", 0.0), reverse=True)
    return items[:limit]


class _Handler(BaseHTTPRequestHandler):
    server_version = "AterkiaDashboard/1.0"
    protocol_version = "HTTP/1.1"

    def log_message(self, *args):
        pass


    def _send_json(self, obj, code=200):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def _send_file(self, path):
        if not os.path.isfile(path):
            self._send_json({"error": "not found"}, 404)
            return
        ext = os.path.splitext(path)[1].lower()
        try:
            with open(path, "rb") as f:
                body = f.read()
        except Exception:
            self._send_json({"error": "read error"}, 500)
            return
        self.send_response(200)
        self.send_header("Content-Type", _MIME.get(ext, "application/octet-stream"))
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass


    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path in ("/", "/index.html"):
            self._send_file(os.path.join(STATIC_DIR, "index.html"))
        elif path.startswith("/static/"):
            name = os.path.basename(path)
            self._send_file(os.path.join(STATIC_DIR, name))
        elif path == "/config":
            track = None
            if "?" in self.path:
                from urllib.parse import parse_qs
                track = (parse_qs(self.path.split("?", 1)[1]).get("track") or [None])[0]
            self._send_json(self.server.config_payload(track))
        elif path == "/events":
            self._stream_sse()
        elif path == "/video":
            self._stream_mjpeg()
        elif path in ("/frame.jpg", "/frame.jpeg", "/snapshot.jpg"):
            self._send_frame_jpeg()
        elif path == "/captures":
            self._send_json({"captures": list_captures()})
        elif path.startswith("/captures/"):
            parts = path.split("/")
            if len(parts) >= 4 and parts[2] in _CAPTURE_DIRS:
                self._send_file(os.path.join(_CAPTURE_DIRS[parts[2]],
                                             os.path.basename(parts[3])))
            else:
                self._send_json({"error": "bad path"}, 400)
        else:
            self._send_json({"error": "not found"}, 404)


    def _stream_sse(self):
        hz = max(1.0, float(getattr(self.server, "sse_hz", 5.0)))
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            while True:
                snap = state_bus.snapshot()
                self.wfile.write(b"data: " + json.dumps(snap).encode() + b"\n\n")
                self.wfile.flush()
                time.sleep(1.0 / hz)
        except Exception:
            return

    def _send_frame_jpeg(self):
        """Kirim SATU frame terbaru sebagai JPEG (untuk polling klien web).

        Lebih tahan dibanding MJPEG pada WiFi goyah: klien meminta ulang
        per frame, jadi koneksi yang putus otomatis pulih tanpa layar beku.
        """
        img, _seq = state_bus.get_frame()
        if img is None:
            self._send_json({"error": "no frame"}, 503)
            return
        width = int(getattr(self.server, "video_width", 640))
        try:
            if width and img.shape[1] > width:
                scale = width / float(img.shape[1])
                img = cv2.resize(img, (width, int(img.shape[0] * scale)),
                                 interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        except Exception:
            ok = False
        if not ok:
            self._send_json({"error": "encode"}, 500)
            return
        body = buf.tobytes()
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            self.wfile.write(body)
        except Exception:
            pass

    def _stream_mjpeg(self):
        hz = max(1.0, float(getattr(self.server, "video_hz", 10.0)))
        width = int(getattr(self.server, "video_width", 640))
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        last_seq = -1
        try:
            while True:
                img, seq = state_bus.get_frame()
                if img is None or seq == last_seq:
                    time.sleep(1.0 / max(hz, 1.0))
                    continue
                last_seq = seq
                try:
                    if width and img.shape[1] > width:
                        scale = width / float(img.shape[1])
                        img = cv2.resize(
                            img, (width, int(img.shape[0] * scale)),
                            interpolation=cv2.INTER_AREA)
                    ok, buf = cv2.imencode(
                        ".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                except Exception:
                    ok = False
                if not ok:
                    time.sleep(0.1)
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n"
                                 b"Content-Length: "
                                 + str(len(buf)).encode() + b"\r\n\r\n")
                self.wfile.write(buf.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / max(hz, 1.0))
        except Exception:
            return


class _Server(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, cfgobj):
        super().__init__(addr, _Handler)
        self._cfg = cfgobj

    @property
    def sse_hz(self):
        return getattr(self._cfg, "SSE_HZ", 5.0)

    @property
    def video_hz(self):
        return getattr(self._cfg, "VIDEO_HZ", 10.0)

    @property
    def video_width(self):
        return getattr(self._cfg, "VIDEO_WIDTH", 640)

    def config_payload(self, track=None):
        if track:
            trk = cfg.load_track(track)
            return {
                "team": getattr(self._cfg, "TEAM_NAME", ""),
                "pt": getattr(self._cfg, "TEAM_PT", ""),
                "track": str(track).upper(),
                "track_start": {"lat": trk.get("start_lat"),
                                "lon": trk.get("start_lon"),
                                "color": trk.get("start_color", "red")},
                "show_speed_kmh": getattr(self._cfg, "SHOW_SPEED_KMH", True),
                "grid": trk.get("grid", {}),
                "waypoints": trk.get("waypoints", []),
                "vision_legs": trk.get("vision_legs", []),
            }
        trk = getattr(self._cfg, "TRACK_DATA", {}) or {}
        return {
            "team": getattr(self._cfg, "TEAM_NAME", ""),
            "pt": getattr(self._cfg, "TEAM_PT", ""),
            "track": getattr(self._cfg, "TRACK", "A"),
            "track_start": getattr(self._cfg, "TRACK_START", None),
            "show_speed_kmh": getattr(self._cfg, "SHOW_SPEED_KMH", True),
            "grid": getattr(self._cfg, "TRACK_GRID", {}) or {
                "origin": "START",
                "cell_m": getattr(self._cfg, "GRID_CELL_M", 5),
                "cols": getattr(self._cfg, "GRID_COLS", 5),
                "rows": getattr(self._cfg, "GRID_ROWS", 5),
            },
            "waypoints": trk.get("waypoints", []),
            "vision_legs": getattr(self._cfg, "VISION_ENABLED_LEGS", []),
        }


class DashboardServer:
    """Wrapper thread daemon untuk server dashboard."""

    def __init__(self, cfgobj):
        self.cfg = cfgobj
        self.httpd = None
        self.thread = None

    def start(self):
        host = getattr(self.cfg, "WEB_HOST", "0.0.0.0")
        port = int(getattr(self.cfg, "WEB_PORT", 8080))
        self.httpd = _Server((host, port), self.cfg)
        self.thread = threading.Thread(target=self.httpd.serve_forever,
                                       name="DashboardServer", daemon=True)
        self.thread.start()
        return host, port

    def stop(self):
        if self.httpd is not None:
            try:
                self.httpd.shutdown()
                self.httpd.server_close()
            except Exception:
                pass
