"""app/slim_map.py — Peta ringan murni PyQt5 pengganti folium+QWebEngine.

Kenapa diganti: QWebEngine = Chromium penuh (start lambat, RAM besar,
berat di Raspberry Pi). Widget ini menggambar manual pakai QPainter:
tile OSM + garis rute + marker WP + panah kapal. Tanpa dependensi baru,
jalan di semua laptop (Windows/Linux) selama PyQt5 ada.

Fitur (sama seperti peta lama):
  - set_waypoints(list[{'lat','lon'}]) + set_vision_legs(list[int])
  - update_vehicle(lat, lon, yaw_deg) — panggil max ~1 Hz dari GUI
  - sinyal wpMoved(index, lat, lon) saat marker digeser mouse
  - wheel = zoom, drag kosong = pan, tombol follow opsional

Tile OSM di-cache di data/tiles/{z}/{x}/{y}.png (LRU sederhana, max
~2000 file). Offline -> fallback grid gelap + label, fungsi tetap jalan.

Kontrak gambar (slippy-map OSM):
  n = 2**zoom; xtile = (lon+180)/360*n
  ytile = (1 - ln(tan(lat)+1/cos(lat))/pi)/2*n
"""

import math
import os

from PyQt5.QtCore import Qt, pyqtSignal, QPointF, QUrl
from PyQt5.QtGui import QPainter, QColor, QPen, QPixmap, QFont
from PyQt5.QtWidgets import QWidget
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest

APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(APP_DIR)
TILE_DIR = os.path.join(ROOT_DIR, "data", "tiles")
TILE_URL = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
MAX_CACHE_FILES = 2000
DEFAULT_ZOOM = 18
MIN_ZOOM = 3
MAX_ZOOM = 19
TILE_PX = 256


def latlon_to_tile_float(lat, lon, zoom):
    """Koordinat slippy-map float (bisa pecahan) untuk lat/lon."""
    n = float(1 << int(zoom))
    xt = (float(lon) + 180.0) / 360.0 * n
    lat_r = math.radians(max(-85.05, min(85.05, float(lat))))
    yt = (1.0 - math.log(math.tan(lat_r) + 1.0 / math.cos(lat_r))
          / math.pi) / 2.0 * n
    return xt, yt


def tile_to_latlon(xt, yt, zoom):
    """Balikan latlon_to_tile_float."""
    n = float(1 << int(zoom))
    lon = xt / n * 360.0 - 180.0
    lat_r = math.atan(math.sinh(math.pi * (1.0 - 2.0 * yt / n)))
    return math.degrees(lat_r), lon


class SlimMapWidget(QWidget):
    """Widget peta ringan: tile + rute + WP draggable + panah kapal."""

    wpMoved = pyqtSignal(int, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(220)
        self._zoom = DEFAULT_ZOOM
        # Pusat peta (lat/lon); None = belum ada data -> default Semarang.
        self._center = (-6.9834, 110.4098)
        self._wps = []
        self._vision_legs = set()
        self._veh = None  # (lat, lon, yaw_deg)
        self._follow = True
        self._tiles = {}  # (z,x,y) -> QPixmap | None(sedang diunduh)
        self._net = QNetworkAccessManager(self)
        self._net.finished.connect(self._on_tile_done)
        self._pending = {}  # QNetworkReply -> (z,x,y,path)
        self._drag_wp = -1
        self._panning = False
        self._last_mouse = None
        os.makedirs(TILE_DIR, exist_ok=True)

    # ------------------- API publik (dipakai main.py) -------------------

    def set_waypoints(self, wps):
        """Ganti daftar waypoint [{'lat','lon'}]; recenters bila follow."""
        self._wps = [{'lat': float(w['lat']), 'lon': float(w['lon'])}
                     for w in (wps or [])]
        if self._follow and self._wps:
            self._center = (self._wps[0]['lat'], self._wps[0]['lon'])
        self.update()

    def set_vision_legs(self, legs):
        """Index leg (1-based) yang vision ON — digambar hijau solid."""
        try:
            self._vision_legs = set(int(i) for i in (legs or []))
        except Exception:
            self._vision_legs = set()
        self.update()

    def update_vehicle(self, lat, lon, yaw_deg):
        """Posisi kapal; ikut menggeser pusat bila follow aktif."""
        try:
            self._veh = (float(lat), float(lon), float(yaw_deg))
        except (TypeError, ValueError):
            return
        if self._follow:
            self._center = (float(lat), float(lon))
        self.update()

    def set_follow(self, on):
        self._follow = bool(on)

    # ------------------- tile OSM + cache -------------------

    def _tile_path(self, z, x, y):
        return os.path.join(TILE_DIR, str(z), str(x), f"{y}.png")

    def _get_tile(self, z, x, y):
        key = (z, x, y)
        if key in self._tiles:
            return self._tiles[key]
        path = self._tile_path(z, x, y)
        if os.path.exists(path):
            pm = QPixmap(path)
            if not pm.isNull():
                self._tiles[key] = pm
                return pm
        # Antre unduh sekali saja.
        self._tiles[key] = None
        try:
            req = QNetworkRequest(QUrl(TILE_URL.format(z=z, x=x, y=y)))
            req.setRawHeader(b"User-Agent", b"AterkiaASV/1.0")
            reply = self._net.get(req)
            self._pending[reply] = (z, x, y, path)
        except Exception:
            pass
        return None

    def _on_tile_done(self, reply):
        info = self._pending.pop(reply, None)
        try:
            if info is not None and not reply.error():
                z, x, y, path = info
                data = reply.readAll()
                pm = QPixmap()
                if pm.loadFromData(data):
                    try:
                        os.makedirs(os.path.dirname(path), exist_ok=True)
                        with open(path, "wb") as f:
                            f.write(bytes(data))
                        self._prune_cache()
                    except Exception:
                        pass
                    self._tiles[(z, x, y)] = pm
                    self.update()
        finally:
            try:
                reply.deleteLater()
            except Exception:
                pass

    def _prune_cache(self):
        """Batasi jumlah file tile agar hemat storage."""
        try:
            files = []
            for z in os.listdir(TILE_DIR):
                zd = os.path.join(TILE_DIR, z)
                if not os.path.isdir(zd):
                    continue
                for x in os.listdir(zd):
                    xd = os.path.join(zd, x)
                    if not os.path.isdir(xd):
                        continue
                    for f in os.listdir(xd):
                        p = os.path.join(xd, f)
                        try:
                            files.append((os.path.getmtime(p), p))
                        except OSError:
                            pass
            if len(files) > MAX_CACHE_FILES:
                files.sort()
                for _, p in files[:len(files) - MAX_CACHE_FILES]:
                    try:
                        os.remove(p)
                    except OSError:
                        pass
        except Exception:
            pass

    # ------------------- interaksi mouse -------------------

    def _screen_to_latlon(self, px, py):
        cx, cy = latlon_to_tile_float(*self._center, self._zoom)
        w, h = max(1, self.width()), max(1, self.height())
        xt = cx + (px - w / 2.0) / TILE_PX
        yt = cy + (py - h / 2.0) / TILE_PX
        return tile_to_latlon(xt, yt, self._zoom)

    def _latlon_to_screen(self, lat, lon):
        cx, cy = latlon_to_tile_float(*self._center, self._zoom)
        xt, yt = latlon_to_tile_float(lat, lon, self._zoom)
        w, h = max(1, self.width()), max(1, self.height())
        return ((xt - cx) * TILE_PX + w / 2.0,
                (yt - cy) * TILE_PX + h / 2.0)

    def _wp_at(self, px, py, radius=12):
        for i, wp in enumerate(self._wps):
            sx, sy = self._latlon_to_screen(wp['lat'], wp['lon'])
            if abs(sx - px) <= radius and abs(sy - py) <= radius:
                return i
        return -1

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            idx = self._wp_at(event.x(), event.y())
            if idx >= 0:
                self._drag_wp = idx
                self._follow = False
            else:
                self._panning = True
            self._last_mouse = (event.x(), event.y())
            event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_wp >= 0:
            lat, lon = self._screen_to_latlon(event.x(), event.y())
            self._wps[self._drag_wp] = {'lat': lat, 'lon': lon}
            self.update()
            event.accept()
        elif self._panning and self._last_mouse is not None:
            dx = event.x() - self._last_mouse[0]
            dy = event.y() - self._last_mouse[1]
            self._last_mouse = (event.x(), event.y())
            cx, cy = latlon_to_tile_float(*self._center, self._zoom)
            cx -= dx / TILE_PX
            cy -= dy / TILE_PX
            self._center = tile_to_latlon(cx, cy, self._zoom)
            self._follow = False
            self.update()
            event.accept()

    def mouseReleaseEvent(self, event):
        if self._drag_wp >= 0 and event.button() == Qt.LeftButton:
            idx = self._drag_wp
            self._drag_wp = -1
            wp = self._wps[idx]
            self.wpMoved.emit(idx, float(wp['lat']), float(wp['lon']))
            event.accept()
        self._panning = False
        self._last_mouse = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0 and self._zoom < MAX_ZOOM:
            self._zoom += 1
        elif delta < 0 and self._zoom > MIN_ZOOM:
            self._zoom -= 1
        else:
            return
        self.update()
        event.accept()

    # ------------------- gambar -------------------

    def paintEvent(self, _event):
        painter = QPainter(self)
        w, h = max(1, self.width()), max(1, self.height())
        painter.fillRect(0, 0, w, h, QColor("#0e1113"))

        cx, cy = latlon_to_tile_float(*self._center, self._zoom)
        x0 = int(cx - w / 2.0 / TILE_PX) - 1
        x1 = int(cx + w / 2.0 / TILE_PX) + 1
        y0 = int(cy - h / 2.0 / TILE_PX) - 1
        y1 = int(cy + h / 2.0 / TILE_PX) + 1
        n = 1 << self._zoom
        drawn_any = False
        for tx in range(x0, x1 + 1):
            for ty in range(y0, y1 + 1):
                if tx < 0 or ty < 0 or tx >= n or ty >= n:
                    continue
                pm = self._get_tile(self._zoom, tx, ty)
                sx = int((tx - cx) * TILE_PX + w / 2.0)
                sy = int((ty - cy) * TILE_PX + h / 2.0)
                if pm is not None:
                    painter.drawPixmap(sx, sy, TILE_PX, TILE_PX, pm)
                    drawn_any = True
        if not drawn_any:
            # Offline / tile belum ada: grid gelap + label.
            painter.setPen(QPen(QColor(60, 70, 80), 1))
            for gx in range(0, w, 40):
                painter.drawLine(gx, 0, gx, h)
            for gy in range(0, h, 40):
                painter.drawLine(0, gy, w, gy)
            painter.setPen(QColor("#8b949e"))
            painter.setFont(QFont("sans-serif", 10))
            painter.drawText(12, 24, "Peta offline — tile OSM dimuat saat online.")

        # --- garis rute ---
        for i in range(1, len(self._wps)):
            x1s, y1s = self._latlon_to_screen(
                self._wps[i - 1]['lat'], self._wps[i - 1]['lon'])
            x2s, y2s = self._latlon_to_screen(
                self._wps[i]['lat'], self._wps[i]['lon'])
            vision = i in self._vision_legs
            color = QColor(46, 204, 113) if vision else QColor(140, 150, 160)
            pen = QPen(color, 4 if vision else 2)
            if not vision:
                pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(int(x1s), int(y1s), int(x2s), int(y2s))

        # --- marker waypoint ---
        painter.setFont(QFont("sans-serif", 9, QFont.Bold))
        for i, wp in enumerate(self._wps):
            sx, sy = self._latlon_to_screen(wp['lat'], wp['lon'])
            r = 8
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(0, 212, 170))
            painter.drawEllipse(int(sx - r), int(sy - r), r * 2, r * 2)
            painter.setPen(QColor("#ffffff"))
            painter.drawText(int(sx + r + 3), int(sy + 4), f"WP{i + 1}")

        # --- panah kapal ---
        if self._veh is not None:
            sx, sy = self._latlon_to_screen(self._veh[0], self._veh[1])
            yaw = self._veh[2]
            painter.save()
            painter.translate(int(sx), int(sy))
            painter.rotate(yaw)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(255, 80, 80))
            painter.drawPolygon(
                QPointF(0, -11), QPointF(8, 9), QPointF(0, 4), QPointF(-8, 9))
            painter.restore()

        # --- label zoom ---
        painter.setPen(QColor("#8b949e"))
        painter.setFont(QFont("monospace", 9))
        painter.drawText(10, h - 10, f"z{self._zoom}")
        painter.end()
