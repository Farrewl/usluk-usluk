"""app/settings_dialog.py — Dialog Settings bertab (lazy-load).

Tuning (~50 parameter) TIDAK lagi dibuat saat start `main.py` — widget
baru dibangun saat dialog pertama dibuka. Nilai dibaca dari
`nav_thread.config`, disimpan live via `update_config_param`, dan
persist ke JSON saat Accept (via `save_config_to_file`).

Tab: Navigasi / Vision / Misi / YOLO+Sistem / Manual.
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QDialogButtonBox,
                             QFormLayout, QLabel, QLineEdit, QTabWidget,
                             QWidget, QScrollArea, QDoubleSpinBox, QSpinBox)


def _parse_int_list(text):
    """'1, 3, 5' -> [1, 3, 5]; token bukan-digit dibuang."""
    return [int(x.strip()) for x in text.split(',') if x.strip().isdigit()]


class SettingsDialog(QDialog):
    """Dialog tuning; butuh `nav_thread` (update_config_param + config)."""

    def __init__(self, nav_thread, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Settings — Tuning ASV")
        self.resize(560, 640)
        self.nav_thread = nav_thread
        self._built = False

        layout = QVBoxLayout(self)
        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self._on_accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _on_accept(self):
        try:
            self.nav_thread.save_config_to_file()
        except Exception:
            pass
        self.accept()

    # ------------------- lazy build -------------------

    def showEvent(self, event):  # noqa: N802 - API Qt
        if not self._built:
            self._build_all()
            self._built = True
        super().showEvent(event)

    def _form_page(self, title):
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        page = QWidget()
        form = QFormLayout(page)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
        scroll.setWidget(page)
        self.tabs.addTab(scroll, title)
        return form

    def _add_header(self, form, text):
        label = QLabel(text)
        label.setStyleSheet(
            "font-size: 13px; font-weight: bold; margin-top: 10px;")
        form.addRow(label)

    def _add_row(self, form, label_text, config_key, min_val, max_val,
                 step, is_int=False):
        widget = QSpinBox() if is_int else QDoubleSpinBox()
        widget.setRange(min_val, max_val)
        widget.setSingleStep(step)
        try:
            current = getattr(self.nav_thread.config, config_key, 0)
            widget.setValue(current)
        except Exception:
            pass
        widget.valueChanged.connect(
            lambda v, k=config_key: self.nav_thread.update_config_param(k, v))
        form.addRow(label_text, widget)
        return widget

    def _add_list(self, form, label_text, config_key):
        widget = QLineEdit()
        widget.setPlaceholderText("Contoh: 1, 3, 5")
        try:
            cur = getattr(self.nav_thread.config, config_key, [])
            if isinstance(cur, list):
                widget.setText(", ".join(map(str, cur)))
        except Exception:
            pass

        def save_list():
            try:
                new_list = _parse_int_list(widget.text())
                self.nav_thread.update_config_param(config_key, new_list)
                widget.setText(", ".join(map(str, new_list)))
            except Exception:
                pass

        widget.editingFinished.connect(save_list)
        form.addRow(label_text, widget)
        return widget

    def _build_all(self):
        nav = self.nav_thread
        _ = nav  # baca config via helper di bawah

        f = self._form_page("Navigasi")
        self._add_header(f, "Navigasi Umum & GPS")
        self._add_row(f, "ACCEPTANCE_RADIUS_M:", "ACCEPTANCE_RADIUS_M",
                      0.5, 10.0, 0.1)
        self._add_row(f, "THRUST_VALUE (Transit):", "THRUST_VALUE",
                      0.0, 1.0, 0.05)
        self._add_row(f, "TRANSITION_DURATION_S:", "TRANSITION_DURATION_S",
                      0.0, 5.0, 0.1)
        self._add_row(f, "GEOFENCE_WIDTH_METERS:", "GEOFENCE_WIDTH_METERS",
                      0.5, 10.0, 0.1)
        self._add_header(f, "Trigger Misi (Waypoint Index)")
        self._add_list(f, "VISION_ENABLED_LEGS (Gate):", "VISION_ENABLED_LEGS")
        self._add_list(f, "PHOTO_BOX_LEGS (Hijau):", "PHOTO_BOX_LEGS")
        self._add_list(f, "BLUE_BOX_PHOTO_LEGS (Biru):", "BLUE_BOX_PHOTO_LEGS")
        self._add_list(f, "STOP_AND_PHOTO_AT_WP:", "STOP_AND_PHOTO_AT_WP")

        f = self._form_page("Vision")
        self._add_header(f, "Parameter Vision (Global)")
        self._add_row(f, "FOCAL_LENGTH_PX:", "FOCAL_LENGTH_PX",
                      100, 2000, 10, is_int=True)
        self._add_row(f, "VISION_P_GAIN:", "VISION_P_GAIN", 0.0, 10.0, 0.1)
        self._add_row(f, "VISION_SMOOTHING_ALPHA:", "VISION_SMOOTHING_ALPHA",
                      0.0, 1.0, 0.05)
        self._add_row(f, "ROI_TOP_CUTOFF_PERCENT:", "ROI_TOP_CUTOFF_PERCENT",
                      0.0, 1.0, 0.05)
        self._add_header(f, "Misi Gate (Buoy)")
        self._add_row(f, "GATE_WIDTH_METERS:", "GATE_WIDTH_METERS",
                      0.5, 5.0, 0.1)
        self._add_row(f, "MIN_BUOY_AREA_PX:", "MIN_BUOY_AREA_PX",
                      0, 5000, 10, is_int=True)
        self._add_row(f, "GATE_AREA_SIMILARITY_RATIO:",
                      "GATE_AREA_SIMILARITY_RATIO", 0.0, 1.0, 0.05)
        self._add_row(f, "GATE_VERTICAL_ALIGN_PX:", "GATE_VERTICAL_ALIGN_PX",
                      0, 300, 5, is_int=True)
        self._add_row(f, "GATE_PASS_DISTANCE_M:", "GATE_PASS_DISTANCE_M",
                      0.2, 5.0, 0.1)
        self._add_row(f, "GATE_LOST_TOLERANCE_FRAMES:",
                      "GATE_LOST_TOLERANCE_FRAMES", 0, 30, 1, is_int=True)
        self._add_row(f, "BUOY_CONF_THRESHOLD:", "BUOY_CONF_THRESHOLD",
                      0.0, 1.0, 0.05)
        self._add_row(f, "BUOY_CONF_SMALL_THRESHOLD:",
                      "BUOY_CONF_SMALL_THRESHOLD", 0.0, 1.0, 0.05)
        self._add_row(f, "BUOY_SMALL_AREA_PX:", "BUOY_SMALL_AREA_PX",
                      0, 1000, 10, is_int=True)
        self._add_header(f, "Ambang per-Class (P4-D: hijau lebih longgar)")
        self._add_row(f, "CONF_GREEN:", "BUOY_CONF_THRESHOLD_GREEN",
                      0.0, 1.0, 0.05)
        self._add_row(f, "CONF_SMALL_GREEN:",
                      "BUOY_CONF_SMALL_THRESHOLD_GREEN", 0.0, 1.0, 0.05)
        self._add_row(f, "CONF_RED:", "BUOY_CONF_THRESHOLD_RED",
                      0.0, 1.0, 0.05)
        self._add_row(f, "CONF_SMALL_RED:",
                      "BUOY_CONF_SMALL_THRESHOLD_RED", 0.0, 1.0, 0.05)
        self._add_row(f, "COLOR_FRAC_GREEN:",
                      "BUOY_MIN_COLOR_FRACTION_GREEN", 0.0, 1.0, 0.01)
        self._add_row(f, "COLOR_FRAC_RED:",
                      "BUOY_MIN_COLOR_FRACTION_RED", 0.0, 1.0, 0.01)
        self._add_row(f, "SAT_GREEN:", "BUOY_MIN_SATURATION_GREEN",
                      0.0, 1.0, 0.05)
        self._add_row(f, "SAT_RED:", "BUOY_MIN_SATURATION_RED",
                      0.0, 1.0, 0.05)
        self._add_header(f, "Adaptif Gelap (P4-A: longgar otomatis)")
        self._add_row(f, "ADAPTIVE_ENABLED:", "BUOY_ADAPTIVE_ENABLED",
                      0, 1, 1, is_int=True)
        self._add_row(f, "BRIGHTNESS_THRESHOLD:",
                      "BUOY_BRIGHTNESS_THRESHOLD", 20, 200, 5)
        self._add_row(f, "ADAPTIVE_MIN_SATURATION:",
                      "BUOY_ADAPTIVE_MIN_SATURATION", 0.0, 1.0, 0.05)
        self._add_row(f, "ADAPTIVE_MIN_VALUE:",
                      "BUOY_ADAPTIVE_MIN_VALUE", 5, 60, 1)
        self._add_row(f, "ADAPTIVE_FRAC_MULT:",
                      "BUOY_ADAPTIVE_COLOR_FRACTION_MULT", 0.1, 1.0, 0.1)

        f = self._form_page("Misi")
        self._add_header(f, "Misi Foto Box Hijau")
        self._add_row(f, "SEARCH_THRUST:", "SEARCH_THRUST", 0.0, 1.0, 0.05)
        self._add_row(f, "ALIGN_THRUST:", "ALIGN_THRUST", 0.0, 1.0, 0.05)
        self._add_row(f, "RETREAT_THRUST:", "RETREAT_THRUST", -1.0, 0.0, 0.05)
        self._add_row(f, "RETREAT_DURATION_S:", "RETREAT_DURATION_S",
                      0.0, 10.0, 0.1)
        self._add_row(f, "BOX_WIDTH_METERS:", "BOX_WIDTH_METERS",
                      0.1, 2.0, 0.05)
        self._add_row(f, "APPROACH_DISTANCE_M:", "BOX_APPROACH_DISTANCE_M",
                      0.5, 5.0, 0.1)
        self._add_row(f, "YAW_SEARCH_BOX:", "YAW_SEARCH_BOX",
                      -180, 180, 5, is_int=True)
        self._add_row(f, "LATERAL_THRUST:", "BOX_SEARCH_LATERAL_THRUST",
                      -1.0, 1.0, 0.05)
        self._add_header(f, "Misi Foto Box Biru")
        self._add_row(f, "SEARCH_THRUST:", "BLUE_BOX_SEARCH_THRUST",
                      0.0, 1.0, 0.05)
        self._add_row(f, "ALIGN_THRUST:", "BLUE_BOX_ALIGN_THRUST",
                      0.0, 1.0, 0.05)
        self._add_row(f, "BOX_WIDTH_METERS:", "BLUE_BOX_WIDTH_METERS",
                      0.1, 2.0, 0.05)
        self._add_row(f, "APPROACH_DISTANCE_M:", "BLUE_BOX_APPROACH_DISTANCE_M",
                      0.5, 5.0, 0.1)
        self._add_row(f, "LATERAL_OFFSET_M:", "BLUE_BOX_LATERAL_OFFSET_M",
                      -5.0, 5.0, 0.1)
        self._add_row(f, "YAW_SEARCH:", "BLUE_BOX_YAW_SEARCH",
                      -180, 180, 5, is_int=True)
        self._add_header(f, "Misi Docking (Box Merah)")
        self._add_row(f, "ALIGN_THRUST:", "DOCK_ALIGN_THRUST", 0.0, 1.0, 0.05)
        self._add_row(f, "HOLD_DURATION_S:", "DOCK_HOLD_DURATION_S",
                      0.0, 20.0, 0.5)
        self._add_row(f, "BOX_WIDTH_METERS:", "RED_BOX_WIDTH_METERS",
                      0.1, 2.0, 0.05)
        self._add_row(f, "DOCK_DISTANCE_M:", "RED_BOX_DOCK_DISTANCE_M",
                      0.1, 5.0, 0.1)
        self._add_row(f, "YAW_SEARCH:", "YAW_SEARCH_DOCK",
                      -180, 180, 5, is_int=True)

        f = self._form_page("YOLO+Sistem")
        self._add_header(f, "YOLO & System")
        self._add_row(f, "YOLO_FRAME_SKIP:", "YOLO_FRAME_SKIP",
                      0, 10, 1, is_int=True)
        self._add_row(f, "YOLO_INFERENCE_SIZE:", "YOLO_INFERENCE_SIZE",
                      320, 1280, 32, is_int=True)
        self._add_row(f, "YOLO_RESULT_MAX_AGE_S:", "YOLO_RESULT_MAX_AGE_S",
                      0.1, 2.0, 0.1)
        self._add_header(f, "Failsafe Otomatis (stale-link / low-batt)")
        self._add_row(f, "FAILSAFE_ENABLED:", "FAILSAFE_ENABLED",
                      0, 1, 1, is_int=True)
        self._add_row(f, "FAILSAFE_TELEM_TIMEOUT_S:",
                      "FAILSAFE_TELEM_TIMEOUT_S", 0.5, 10.0, 0.5)
        self._add_row(f, "FAILSAFE_LOW_BATT_PCT:", "FAILSAFE_LOW_BATT_PCT",
                      5.0, 50.0, 1.0)
        self._add_row(f, "FAILSAFE_LOW_VOLT_V:", "FAILSAFE_LOW_VOLT_V",
                      11.0, 16.0, 0.1)
        self._add_row(f, "FAILSAFE_LOW_BATT_HOLD_S:",
                      "FAILSAFE_LOW_BATT_HOLD_S", 0.0, 10.0, 0.5)
        self._add_row(f, "SESSION_VIDEO_FPS:", "SESSION_VIDEO_FPS",
                      1.0, 30.0, 1.0)
        self._add_row(f, "CAMERA_FLIP_MODE:", "CAMERA_FLIP_MODE",
                      0, 3, 1, is_int=True)
        self._add_header(f, "Pixhawk Non-blocking + Histeresis (P6-B)")
        self._add_row(f, "MAV_CONNECT_TIMEOUT_S:", "MAV_CONNECT_TIMEOUT_S",
                      0.5, 10.0, 0.5)
        self._add_row(f, "MAV_RETRY_INTERVAL_S:", "MAV_RETRY_INTERVAL_S",
                      1.0, 30.0, 1.0)
        self._add_row(f, "TELEM_HYSTERESIS_FRAMES:",
                      "TELEM_HYSTERESIS_FRAMES", 1, 30, 1, is_int=True)
        self._add_header(f, "Filter & Kontroler (PID/EKF)")
        self._add_row(f, "PID_KP:", "PID_KP", 0.0, 10.0, 0.1)
        self._add_row(f, "PID_KI:", "PID_KI", 0.0, 5.0, 0.05)
        self._add_row(f, "PID_KD:", "PID_KD", 0.0, 5.0, 0.05)
        self._add_row(f, "PID_DEADBAND:", "PID_DEADBAND", 0.0, 0.5, 0.01)
        self._add_row(f, "PID_OUTPUT_LIMIT:", "PID_OUTPUT_LIMIT",
                      0.0, 2.0, 0.05)
        self._add_row(f, "COMPLEMENTARY_ALPHA:", "COMPLEMENTARY_ALPHA",
                      0.0, 1.0, 0.05)

        f = self._form_page("Manual")
        self._add_header(f, "Kendali Manual (RC / Gamepad — hitung di C)")
        self._add_row(f, "MANUAL_ENABLED:", "MANUAL_ENABLED",
                      0, 1, 1, is_int=True)
        self._add_row(f, "MANUAL_MAX_SURGE:", "MANUAL_MAX_SURGE",
                      0.0, 1.0, 0.05)
        self._add_row(f, "MANUAL_MAX_YAW:", "MANUAL_MAX_YAW", 0.0, 1.0, 0.05)
        self._add_row(f, "MANUAL_DEADBAND:", "MANUAL_DEADBAND",
                      0.0, 0.5, 0.01)
        self._add_row(f, "MANUAL_EXPO:", "MANUAL_EXPO", 0.0, 1.0, 0.05)
        self._add_row(f, "MANUAL_RATE_LIMIT:", "MANUAL_RATE_LIMIT",
                      0.0, 10.0, 0.1)
        self._add_row(f, "RC_TIMEOUT_MS:", "RC_TIMEOUT_MS",
                      100, 2000, 50, is_int=True)
        self._add_row(f, "RC_CH_THROTTLE:", "RC_CH_THROTTLE",
                      1, 16, 1, is_int=True)
        self._add_row(f, "RC_CH_YAW:", "RC_CH_YAW", 1, 16, 1, is_int=True)
        self._add_row(f, "RC_CH_MODE:", "RC_CH_MODE", 1, 16, 1, is_int=True)
        self._add_row(f, "RC_CH_DEADMAN:", "RC_CH_DEADMAN",
                      1, 16, 1, is_int=True)
        self._add_row(f, "MANUAL_LOST_HOLD_S:", "MANUAL_LOST_HOLD_S",
                      0.0, 5.0, 0.1)
