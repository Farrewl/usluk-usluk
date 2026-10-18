import sys
import os
import time

import cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QSplitter, QFrame,
                             QPushButton, QTabWidget, QComboBox)
from PyQt5.QtCore import QTimer, Qt, QLibraryInfo, QRect
from PyQt5.QtGui import QImage, QPixmap, QPainter, QColor, QFont


os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = QLibraryInfo.location(
    QLibraryInfo.PluginsPath)
os.environ.pop("QT_QPA_FONTDIR", None)
os.environ.setdefault("QT_LOGGING_RULES", "qt.network.ssl.warning=false")

from app.navigator import NavigatorThread

from app.slim_map import SlimMapWidget
from app.settings_dialog import SettingsDialog
from app import settings as cfg
from app import gui_theme as theme
from app.logutil import get_logger, throttled

log = get_logger()

ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
CFG_PATH = os.path.join(ROOT_DIR, "config")

SITE = cfg.SITE
WAYPOINTS = cfg.track_waypoints(SITE.get("TRACK"))


class HudOverlay(QWidget):
    """2 pil HUD di atas video: state (kiri) + yaw/koordinat (kanan)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAttribute(Qt.WA_TransparentForMouseEvents)
        self._state = "NO_TELEM"
        self._yaw = 0.0
        self._lat = 0.0
        self._lon = 0.0
        self._dist = 0.0

    def set_data(self, state, yaw, lat, lon, dist):
        self._state = state


        self._yaw = float(yaw) if yaw is not None else 0.0
        self._lat = lat
        self._lon = lon
        self._dist = float(dist) if dist is not None else 0.0
        self.update()

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 150))
        painter.drawRoundedRect(8, 8, 220, 30, 8, 8)
        accent = (QColor(0, 212, 170) if self._state.startswith(
            ("GATE", "DOCK", "PHOTO", "RETREAT", "VISION", "MANUAL"))
            else QColor(255, 255, 255))
        font = QFont("monospace")
        font.setBold(True)
        font.setPointSize(11)
        painter.setFont(font)
        painter.setPen(accent)
        painter.drawText(QRect(16, 12, 204, 22), Qt.AlignLeft | Qt.AlignVCenter, f"STATE: {self._state}")
        painter.setPen(Qt.NoPen)
        painter.setBrush(QColor(0, 0, 0, 150))
        painter.drawRoundedRect(w - 272, 8, 264, 46, 8, 8)
        info = (f"H {self._yaw:5.1f}°   d {self._dist:5.1f} m\n"
                f"{'--' if self._lat is None else f'{self._lat:.6f}'}, "
                f"{'--' if self._lon is None else f'{self._lon:.6f}'}")
        painter.setFont(QFont("monospace", 10))
        painter.setPen(QColor(255, 255, 255))
        painter.drawText(QRect(w - 264, 12, 248, 38), Qt.AlignLeft | Qt.AlignVCenter, info)
        painter.end()


class VideoPanel(QWidget):
    """Panel video + HUD overlay yang ikut meresize."""

    def __init__(self, parent=None, title="NAV"):
        super().__init__(parent)
        self.title = str(title)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.video_label = QLabel("")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet(
            "background-color: #000; color: #FFF; border-radius: 14px;")
        layout.addWidget(self.video_label)
        self.hud = HudOverlay(self)
        self.hud.setGeometry(0, 0, self.width(), self.height())
        self.hud.raise_()

    def set_frame(self, frame):
        """Tampilkan frame BGR OpenCV (None = hitam polos)."""
        try:
            if frame is None:
                self.video_label.clear()
                return
            h, w, ch = frame.shape
            if h <= 0 or w <= 0:
                return
            lw, lh = self.video_label.width(), self.video_label.height()
            disp = frame
            if lw > 1 and lh > 1 and (w > lw or h > lh):
                scale = min(lw / w, lh / h)
                if scale < 1.0:
                    disp = cv2.resize(
                        frame, (int(w * scale), int(h * scale)),
                        interpolation=cv2.INTER_AREA)
                h, w, ch = disp.shape
            try:
                fmt = QImage.Format_BGR888
            except AttributeError:
                fmt = None
            if fmt is not None:
                if not disp.flags["C_CONTIGUOUS"]:
                    disp = np.ascontiguousarray(disp)
                q_img = QImage(disp.data, w, h, disp.strides[0], fmt)
            else:
                q_img = QImage(disp.data, w, h, ch * w,
                               QImage.Format_RGB888).rgbSwapped()
            self.video_label.setPixmap(QPixmap.fromImage(q_img))
        except Exception:
            pass

    def resizeEvent(self, event):
        self.hud.setGeometry(0, 0, self.width(), self.height())
        self.hud.raise_()
        super().resizeEvent(event)


def _card(title):
    """Bingkai kartu glass + judul kecil; return (frame, layout)."""
    frame = QFrame()
    frame.setProperty("class", "card")
    frame.setStyleSheet(
        "QFrame { background: rgba(255,255,255,0.06);"
        " border: 1px solid rgba(255,255,255,0.12); border-radius: 14px; }")
    layout = QVBoxLayout(frame)
    head = QLabel(title.upper())
    head.setProperty("class", "card-title")
    head.setStyleSheet(
        "font-size: 11px; font-weight: bold; letter-spacing: 2px;"
        " color: #8b949e; background: transparent;")
    layout.addWidget(head)
    return frame, layout


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Aterkia ASV — Dashboard")
        app = QApplication.instance()
        scr = app.primaryScreen().availableGeometry() if app else None
        if scr is not None and scr.width() > 0:
            win_w = min(1500, int(scr.width() * 0.96))
            win_h = min(880, int(scr.height() * 0.94))
            x = max(4, (scr.width() - win_w) // 2)
            y = max(4, (scr.height() - win_h) // 2)
            self.setGeometry(x, y, win_w, win_h)
        else:
            self.setGeometry(100, 100, 1500, 880)
        self.setMinimumSize(1150, 660)
        self.current_lat = 0.0
        self.current_lon = 0.0

        self._chip_cache = {}
        self._last_text_t = 0.0
        self._last_map_t = 0.0
        self._fps_count = 0
        self._fps_t0 = time.monotonic()
        self._fps_val = 0.0
        self._settings_dlg = None

        self.nav_thread = NavigatorThread(WAYPOINTS, self)
        sidebar = self._build_sidebar()
        center = self._build_center()
        right = self._build_right_tabs()

        main_split = QSplitter(Qt.Horizontal)
        main_split.addWidget(sidebar)
        main_split.addWidget(center)
        main_split.addWidget(right)
        main_split.setSizes([260, 830, 410])
        center.setMinimumWidth(480)
        self.setCentralWidget(main_split)



        self.nav_thread.newData.connect(self._on_new_data)
        self._setup_status_bar()
        self._refresh_map_waypoints()
        self.nav_thread.start()



    def _build_sidebar(self):
        side = QWidget()
        side.setFixedWidth(260)
        layout = QVBoxLayout(side)

        f, l = _card("Mode")
        self.side_mode = QLabel("AUTO")
        self.side_mode.setStyleSheet(
            "font-family: monospace; font-weight: bold; font-size: 26px;"
            f" color: {theme.COLOR_OK}; background: transparent;")
        l.addWidget(self.side_mode)
        self.side_rc = QLabel("RC: --")
        self.side_rc.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.side_rc)
        layout.addWidget(f)

        f, l = _card("State")
        self.side_state = QLabel("--")
        self.side_state.setStyleSheet(
            "font-family: monospace; font-weight: bold; font-size: 16px;"
            " background: transparent;")
        l.addWidget(self.side_state)
        self.side_yaw = QLabel("H -- | d --")
        self.side_yaw.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.side_yaw)
        layout.addWidget(f)

        f, l = _card("Waypoint")
        self.side_wp = QLabel("WP -- / --")
        self.side_wp.setStyleSheet(
            "font-family: monospace; font-size: 15px; background: transparent;")
        l.addWidget(self.side_wp)
        self.side_dist = QLabel("-- m")
        self.side_dist.setStyleSheet(
            "font-family: monospace; font-size: 22px; font-weight: bold;"
            " background: transparent;")
        l.addWidget(self.side_dist)
        layout.addWidget(f)

        f, l = _card("Baterai")
        self.side_batt = QLabel("--")
        self.side_batt.setStyleSheet(
            "font-family: monospace; font-size: 22px; font-weight: bold;"
            " background: transparent;")
        l.addWidget(self.side_batt)
        self.side_volt = QLabel("-- V | -- A")
        self.side_volt.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.side_volt)
        layout.addWidget(f)

        f, l = _card("Param Pixhawk")
        self.side_param = QLabel("menunggu MAVLink")
        self.side_param.setWordWrap(True)
        self.side_param.setStyleSheet(
            "font-family: monospace; font-size: 12px; color: #8b949e;"
            " background: transparent;")
        l.addWidget(self.side_param)
        layout.addWidget(f)

        self.start_btn = QPushButton("▶ MULAI AUTO")
        self.start_btn.setObjectName("startBtn")
        self.start_btn.setCheckable(True)
        if not getattr(self.nav_thread.config, "STARTUP_HOLD", True):
            self.start_btn.setChecked(True)
            self.start_btn.setText("⏸ HOLD")
        self.start_btn.clicked.connect(self.on_start_toggle)
        layout.addWidget(self.start_btn)

        self.kill_btn = QPushButton("⏹ KILL")
        self.kill_btn.setObjectName("killBtn")
        self.kill_btn.setCheckable(True)
        self.kill_btn.clicked.connect(self.on_manual_kill_toggle)
        layout.addWidget(self.kill_btn)

        self.settings_btn = QPushButton("⚙ Settings")
        self.settings_btn.clicked.connect(self.open_settings)
        layout.addWidget(self.settings_btn)
        layout.addStretch(1)
        return side

    def _build_center(self):
        center = QWidget()
        layout = QVBoxLayout(center)
        layout.setContentsMargins(0, 0, 0, 0)

        split = QSplitter(Qt.Vertical)
        self.slim_map = SlimMapWidget()

        self.video_panel = VideoPanel(title="NAV")
        self.video_label = self.video_panel.video_label
        self.hud_overlay = self.video_panel.hud
        self.video_panel_bawah = VideoPanel(title="BAWAH")
        self.video_panel_bawah.hud.hide()
        self.video_label_bawah = self.video_panel_bawah.video_label
        vid_split = QSplitter(Qt.Horizontal)
        vid_split.addWidget(self.video_panel)
        vid_split.addWidget(self.video_panel_bawah)
        vid_split.setSizes([700, 260])
        self.video_panel_bawah.hide()
        self._bawah_shown = False
        self._last_sec_obj = None
        split.addWidget(vid_split)
        split.addWidget(self.slim_map)
        split.setSizes([560, 300])
        layout.addWidget(split, 1)
        return center

    def _build_right_tabs(self):
        tabs = QTabWidget()
        tabs.setFixedWidth(410)


        misi = QWidget()
        ml = QVBoxLayout(misi)
        f, l = _card("Posisi")
        self.misi_pos = QLabel("--")
        self.misi_pos.setStyleSheet(
            "font-family: monospace; font-size: 13px; background: transparent;")
        l.addWidget(self.misi_pos)
        ml.addWidget(f)
        f, l = _card("Attitude")
        self.misi_att = QLabel("Y --  P --  R --")
        self.misi_att.setStyleSheet(
            "font-family: monospace; font-size: 15px; background: transparent;")
        l.addWidget(self.misi_att)
        self.misi_speed = QLabel("-- m/s")
        self.misi_speed.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.misi_speed)
        ml.addWidget(f)
        f, l = _card("Link")
        self.misi_link = QLabel("Autopilot: -- | GPS: --")
        self.misi_link.setStyleSheet(
            "font-family: monospace; font-size: 13px; background: transparent;")
        l.addWidget(self.misi_link)
        ml.addWidget(f)

        f, l = _card("Lintasan")
        trow = QHBoxLayout()
        self.track_combo = QComboBox()
        self.track_combo.addItems(["A", "B"])
        self.track_combo.setCurrentText(str(SITE.get("TRACK", "A")).upper())
        self.track_apply_btn = QPushButton("Pakai Lintasan")
        self.track_apply_btn.setProperty("class", "toolBtn")
        self.track_apply_btn.clicked.connect(self.on_track_apply)
        trow.addWidget(self.track_combo, 1)
        trow.addWidget(self.track_apply_btn)
        l.addLayout(trow)
        ml.addWidget(f)

        f, l = _card("Kamera")
        row = QHBoxLayout()
        self.cam_combo = QComboBox()
        self.cam_scan_btn = QPushButton("Scan")
        self.cam_scan_btn.setProperty("class", "toolBtn")
        self.cam_scan_btn.clicked.connect(self.on_camera_scan)
        row.addWidget(self.cam_combo, 1)
        row.addWidget(self.cam_scan_btn)
        l.addLayout(row)
        row2 = QHBoxLayout()
        self.cam_nav_btn = QPushButton("Pakai NAV")
        self.cam_nav_btn.setProperty("class", "toolBtn")
        self.cam_nav_btn.clicked.connect(self.on_camera_select_nav)
        self.cam_bawah_btn = QPushButton("Pakai BAWAH")
        self.cam_bawah_btn.setProperty("class", "toolBtn")
        self.cam_bawah_btn.clicked.connect(self.on_camera_select_bawah)
        row2.addWidget(self.cam_nav_btn)
        row2.addWidget(self.cam_bawah_btn)
        l.addLayout(row2)
        self.cam_status_label = QLabel("Kamera: --")
        self.cam_status_label.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.cam_status_label)
        ml.addWidget(f)

        f, l = _card("Pixhawk")
        prow = QHBoxLayout()
        self.mav_combo = QComboBox()
        self.mav_scan_btn = QPushButton("Scan")
        self.mav_scan_btn.setProperty("class", "toolBtn")
        self.mav_scan_btn.clicked.connect(self.on_mav_scan)
        prow.addWidget(self.mav_combo, 1)
        prow.addWidget(self.mav_scan_btn)
        l.addLayout(prow)
        self.mav_apply_btn = QPushButton("Pakai Port Ini")
        self.mav_apply_btn.setProperty("class", "toolBtn")
        self.mav_apply_btn.clicked.connect(self.on_mav_select)
        l.addWidget(self.mav_apply_btn)
        self.mav_status_label = QLabel("Pixhawk: --")
        self.mav_status_label.setStyleSheet(
            "font-family: monospace; color: #8b949e; background: transparent;")
        l.addWidget(self.mav_status_label)
        ml.addWidget(f)
        ml.addStretch(1)
        tabs.addTab(misi, "Misi")

        return tabs

    def _setup_status_bar(self):
        """4 chip saja: link, mode, baterai, FPS."""
        bar = self.statusBar()
        self.chip_link = QLabel("● --")
        self.chip_mode = QLabel("AUTO")
        self.chip_batt = QLabel("--")
        self.chip_fps = QLabel("FPS --/30")
        for chip in (self.chip_link, self.chip_mode,
                     self.chip_batt, self.chip_fps):
            chip.setStyleSheet(
                "font-family: monospace; font-weight: bold;"
                "padding: 2px 10px; color: #8b949e; background: transparent;")
            bar.addPermanentWidget(chip)



    def _set_chip(self, widget, key, text, color):
        """setText + warna hanya saat berubah (hindari recalc style Qt)."""
        old = self._chip_cache.get(key)
        if old == (text, color):
            return
        self._chip_cache[key] = (text, color)
        if widget.text() != text:
            widget.setText(text)
        widget.setStyleSheet(
            "font-family: monospace; font-weight: bold; padding: 2px 10px;"
            f" color: {color}; background: transparent;")



    def _refresh_map_waypoints(self):
        try:
            wps = self.nav_thread.navigator.waypoints
            self.slim_map.set_waypoints(wps)
            legs = getattr(self.nav_thread.config,
                            "VISION_ENABLED_LEGS", [])
            self.slim_map.set_vision_legs(legs)
        except Exception:
            pass



    def on_track_apply(self):
        """Ganti lintasan A/B: muat waypoint + leg misi + persist site.json."""
        name = str(self.track_combo.currentText()).strip().upper()
        try:
            cfg.save_site({"TRACK": name})
            cfg.SITE["TRACK"] = name
        except Exception:
            pass
        try:
            trk = cfg.load_track(name)
            c = self.nav_thread.config
            c.TRACK = name
            c.TRACK_DATA = trk
            c.TRACK_START = {"lat": trk.get("start_lat"),
                             "lon": trk.get("start_lon"),
                             "color": trk.get("start_color", "red")}
            c.TRACK_GRID = trk.get("grid", {})
            if "vision_legs" in trk:
                c.VISION_ENABLED_LEGS = trk["vision_legs"]
            if "photo_box_legs" in trk:
                c.PHOTO_BOX_LEGS = trk["photo_box_legs"]
            if "blue_box_photo_legs" in trk:
                c.BLUE_BOX_PHOTO_LEGS = trk["blue_box_photo_legs"]
            wps = cfg.track_waypoints(name)
            self.nav_thread.update_waypoints(wps)
            self._refresh_map_waypoints()
            log.info("Lintasan -> %s (%d waypoint)", name, len(wps))
        except Exception as e:
            log.error("Ganti lintasan gagal: %s", e)



    def open_settings(self):
        if self._settings_dlg is None:
            self._settings_dlg = SettingsDialog(self.nav_thread, self)
        self._settings_dlg.show()
        self._settings_dlg.raise_()
        self._settings_dlg.activateWindow()

    def on_start_toggle(self):
        """▶ MULAI AUTO / ⏸ HOLD dari operator.

        Saat startup kapal HOLD (aman); tombol ini melepasnya ke AUTO/
        OFFBOARD. Saat ditekan lagi kapal kembali HOLD. QGroundControl
        ikut ter-mirror karena Pixhawk broadcast HEARTBEAT custom_mode.
        """
        on = self.start_btn.isChecked()
        self.start_btn.setText("⏸ HOLD" if on else "▶ MULAI AUTO")
        try:
            n = self.nav_thread.navigator
            if on:
                if hasattr(n, "start_auto"):
                    n.start_auto()
            else:
                if hasattr(n, "stop_auto"):
                    n.stop_auto()
        except Exception as e:
            print(f"[START] Gagal toggle auto: {e}")

    def on_manual_kill_toggle(self):
        """KILL / Estop dari GUI: masuk jalur C (mode_manager KILL) +
        kirim DISARM Pixhawk. BUKAN mode manual — ini penghenti darurat."""
        on = self.kill_btn.isChecked()
        self.kill_btn.setText("⏹ KILL AKTIF" if on else "⏹ KILL")
        try:
            n = self.nav_thread.navigator
            if hasattr(n, "kill_request"):
                if on:
                    print("[KILL] Tombol KILL ditekan — gerak dihentikan, "
                          "kirim DISARM ke Pixhawk (best-effort).")
                n.kill_request = bool(on)
        except Exception:
            pass



    def _cam_index_from_combo(self):
        try:
            text = self.cam_combo.currentText()
            return int(str(text).split(":")[0].strip().split(" ")[0])
        except Exception:
            return None

    def on_camera_scan(self):
        """Isi dropdown dengan kamera terdeteksi (tanpa buka frame)."""
        try:
            cams = self.nav_thread.navigator.rescan_cameras()
        except Exception:
            cams = []
        try:
            self.cam_combo.blockSignals(True)
            self.cam_combo.clear()
            if not cams:
                self.cam_combo.addItem("-- tidak ada kamera --")
            for c in cams:
                self.cam_combo.addItem(
                    f"{c['index']}: {c.get('label', '')}")
            self.cam_combo.blockSignals(False)
            self.cam_status_label.setText(f"Kamera: {len(cams)} ditemukan")
        except Exception:
            pass

    def on_camera_select_nav(self):
        idx = self._cam_index_from_combo()
        if idx is None:
            return
        try:
            ok = self.nav_thread.navigator.select_camera("NAV", idx)
            self.cam_status_label.setText(
                f"NAV -> idx {idx} ({'OK' if ok else 'GAGAL'})")
        except Exception:
            pass

    def on_camera_select_bawah(self):
        idx = self._cam_index_from_combo()
        if idx is None:
            return
        try:
            ok = self.nav_thread.navigator.select_camera("BAWAH", idx)
            self.cam_status_label.setText(
                f"BAWAH -> idx {idx} ({'OK' if ok else 'GAGAL'})")
        except Exception:
            pass



    def on_mav_scan(self):
        try:
            from app.mavlink_telemetry import detect_serial_ports
            ports = detect_serial_ports(
                getattr(self.nav_thread.config, "SERIAL_PORT", None))
        except Exception:
            ports = []
        try:
            self.mav_combo.blockSignals(True)
            self.mav_combo.clear()
            if not ports:
                self.mav_combo.addItem("-- tidak ada port --")
            for p in ports:
                self.mav_combo.addItem(p)
            self.mav_combo.blockSignals(False)
        except Exception:
            pass

    def on_mav_select(self):
        try:
            port = self.mav_combo.currentText().strip()
        except Exception:
            return
        if not port or port.startswith("--"):
            return
        try:
            self.nav_thread.update_config_param("SERIAL_PORT", port)
            self.mav_status_label.setText(f"Pixhawk: {port} (reconnect...)")
        except Exception:
            pass



    def _on_new_data(self, data):
        """Slot aman untuk sinyal `newData` dari thread navigator.

        Exception apa pun di `update_ui` ditangkap + dicatat (throttled) agar
        GUI tidak mati: PyQt5 memanggil qFatal (SIGABRT) bila exception lolos
        dari slot. Berguna saat mesin tanpa hardware (lat/lon None, dsb.).
        """
        try:
            self.update_ui(data)
        except Exception as e:
            if throttled("ui_update_err", 5.0):
                log.error("update_ui gagal: %s: %s", type(e).__name__, e)

    def update_ui(self, data):
        self.current_lat = data['lat']
        self.current_lon = data['lon']


        try:
            self.video_panel.set_frame(data.get('frame'))
        except Exception:
            pass

        try:
            sec = data.get('frame_secondary')
            if sec is not None:



                if sec is not self._last_sec_obj:
                    self._last_sec_obj = sec
                    if not self._bawah_shown:
                        self.video_panel_bawah.show()
                        self._bawah_shown = True
                    self.video_panel_bawah.set_frame(sec)
            elif self._bawah_shown:
                self.video_panel_bawah.hide()
                self._bawah_shown = False
                self._last_sec_obj = None
        except Exception:
            pass

        try:
            cam_st = str(data.get('cam_status', 'OK'))
            src = str(data.get('telem_source', '-'))
            if "->" not in self.cam_status_label.text():
                self.cam_status_label.setText(f"Kamera: {cam_st} | {src}")
            mav_ok = bool(data.get('mavlink_ok', False))
            port = (data.get('mav_port')
                    or getattr(self.nav_thread.config, "SERIAL_PORT", "?"))
            reason = str(data.get('mav_reason', '') or '')
            status = "LINK" if mav_ok else f"NO-LINK{reason}"
            self.mav_status_label.setText(
                f"Pixhawk: {port} ({status} | {src})")
        except Exception:
            pass


        self._fps_count += 1
        now_m = time.monotonic()
        if now_m - self._fps_t0 >= 1.0:
            self._fps_val = self._fps_count / (now_m - self._fps_t0)
            self._fps_count = 0
            self._fps_t0 = now_m
            fps_c = (theme.COLOR_OK if self._fps_val >= 27
                     else theme.COLOR_WARN if self._fps_val >= 15
                     else theme.COLOR_BAD)
            self._set_chip(self.chip_fps, "fps",
                           f"FPS {self._fps_val:.0f}/30", fps_c)

        now = time.time()
        if now - self._last_text_t < 0.2:

            if now - self._last_map_t >= 1.0:
                self._last_map_t = now
                try:
                    self.slim_map.update_vehicle(
                        data['lat'], data['lon'], data['yaw_deg'])
                except Exception:
                    pass
            return
        self._last_text_t = now

        yaw = float(data.get('yaw_deg', 0.0) or 0.0)
        pitch = float(data.get('pitch_deg', 0.0) or 0.0)
        roll = float(data.get('roll_deg', 0.0) or 0.0)
        state = str(data.get('state', '--'))
        dist = float(data.get('dist_to_wp_m', 0.0) or 0.0)
        op_mode = str(data.get('op_mode', 'AUTO'))
        man_on = bool(data.get('manual_active', False))
        m_surge = float(data.get('manual_surge', 0.0) or 0.0)
        m_yaw = float(data.get('manual_yaw', 0.0) or 0.0)
        rc_ok = bool(data.get('rc_ok', False))


        try:
            self.hud_overlay.set_data(state, yaw, data['lat'],
                                      data['lon'], dist)
        except Exception:
            pass


        if now - self._last_map_t >= 1.0:
            self._last_map_t = now
            try:
                self.slim_map.update_vehicle(data['lat'], data['lon'], yaw)
            except Exception:
                pass


        fs_on = bool(data.get('failsafe_active', False))
        fs_reason = str(data.get('failsafe_reason', '') or '')
        up_pending = int(data.get('upload_pending', 0) or 0)
        pc = str(data.get('pixhawk_check', '') or '')
        if pc and self.side_param.text() != pc:
            self.side_param.setText(pc)
            pc_ok = pc == "OK" or pc.endswith("OK")
            pc_c = (theme.COLOR_OK if pc_ok
                    else theme.COLOR_BAD if "KRITIS" in pc
                    else theme.COLOR_WARN)
            self.side_param.setStyleSheet(
                "font-family: monospace; font-size: 12px;"
                f" color: {pc_c}; background: transparent;")
        mode_c = (theme.COLOR_BAD if (op_mode == "KILL" or fs_on)
                  else theme.COLOR_WARN if op_mode in ("MANUAL", "HOLD")
                  else theme.COLOR_OK)
        mode_txt = (f"FAILSAFE:{fs_reason[:18]}" if fs_on else op_mode)
        if self.side_mode.text() != mode_txt:
            self.side_mode.setText(mode_txt)
        if self._chip_cache.get("side_mode_c") != mode_c:
            self._chip_cache["side_mode_c"] = mode_c
            self.side_mode.setStyleSheet(
                "font-family: monospace; font-weight: bold; font-size: 26px;"
                f" color: {mode_c}; background: transparent;")
        self.side_rc.setText(f"RC: {'OK' if rc_ok else 'PUTUS'}"
                             f" | {'STICK' if man_on else 'AUTO'}")
        if self.side_state.text() != state:
            self.side_state.setText(state)
        self.side_yaw.setText(f"H {yaw:5.1f}° | d {dist:5.1f} m")

        n_wp = len(getattr(self.nav_thread.navigator, "waypoints", []))
        tgt = int(data.get('target_wp_idx', 0) or 0) + 1
        self.side_wp.setText(f"WP {min(tgt, max(n_wp, 1))} / {n_wp}")
        self.side_dist.setText(f"{dist:.1f} m")


        volt = data.get('voltage_v')
        curr = data.get('current_a')
        pct = data.get('battery_pct')
        if pct is None and volt is not None:
            try:
                pct = max(0.0, min(100.0, (float(volt) - 12.8) / 4.0 * 100.0))
            except (TypeError, ValueError):
                pct = None
        if pct is None:
            self.side_batt.setText("--")
            self.side_volt.setText("-- V | -- A")
            self._set_chip(self.chip_batt, "batt", "BAT --",
                           theme.COLOR_IDLE)
        else:
            volt_s = f"{volt:.1f} V" if volt is not None else "--"
            curr_s = f"{curr:.1f} A" if curr is not None else "--"
            self.side_batt.setText(f"{pct:.0f} %")
            bc = (theme.COLOR_OK if pct >= 50
                  else theme.COLOR_WARN if pct >= 20 else theme.COLOR_BAD)
            self.side_batt.setStyleSheet(
                "font-family: monospace; font-size: 22px; font-weight: bold;"
                f" color: {bc}; background: transparent;")
            self.side_volt.setText(f"{volt_s} | {curr_s}")
            self._set_chip(self.chip_batt, "batt",
                           f"BAT {pct:.0f}% ({volt_s})", bc)


        lat, lon = data.get('lat'), data.get('lon')
        if lat is None or lon is None:
            self.misi_pos.setText("-- belum ada GPS --")
        else:
            self.misi_pos.setText(f"{lat:.6f}, {lon:.6f}")
        self.misi_att.setText(f"Y {yaw:6.1f}°  P {pitch:5.1f}°"
                              f"  R {roll:5.1f}°")
        self.misi_speed.setText(
            f"{float(data.get('groundspeed', 0.0) or 0.0):4.1f} m/s")
        mav_ok = bool(data.get('mavlink_ok', False))
        gps_fix = bool(data.get('gps_fix', False))
        gcs_on = bool(data.get('gcs_forward', False))
        reason = str(data.get('mav_reason', '') or '')
        self.misi_link.setText(
            f"Autopilot: {'NYALA' if mav_ok else 'MATI' + reason} | "
            f"GPS: {'FIX' if gps_fix else 'MENCARI'} | "
            f"QGC: {'FWD' if gcs_on else '--'}")
        self._set_chip(self.chip_link, "link",
                       f"● {'LINK' if mav_ok else 'NO-LINK'}"
                       f"{'+QGC' if gcs_on else ''}",
                       theme.COLOR_OK if mav_ok else theme.COLOR_BAD)
        self._set_chip(self.chip_mode, "mode",
                       "%s%s%s" % (mode_txt, '*' if man_on else '',
                                   (" ^%d" % up_pending) if up_pending else ''),
                       mode_c)

    def closeEvent(self, event):
        print("Closing application...")
        try:
            self.nav_thread.stop()
            self.nav_thread.wait()
        except Exception:
            pass
        if hasattr(self.nav_thread, 'save_config_to_file'):
            print("Menyimpan parameter tuning terakhir...")
            try:
                self.nav_thread.save_config_to_file()
            except Exception:
                pass
        event.accept()


if __name__ == "__main__":
    if not WAYPOINTS:
        print("WARNING: lintasan kosong — cek config/trackA.json / trackB.json.")
        print("Application will start with an empty map.")
        print("Use '+ WP' to add waypoints.")

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    theme.apply_theme(app)

    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
