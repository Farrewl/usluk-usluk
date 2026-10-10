"""app/navigator.py — Navigator PRODUKSI (kapal asli, jalur kendali Pixhawk).

Berbeda dari `app/simulator.py` (darat/uji): di sini posisi & heading datang
dari Pixhawk NYATA (MAVLink), bukan mock; perintah gerak dikirim lewat
`SET_ATTITUDE_TARGET` (Pixhawk tetap pemegang ESC/mixer), dan keputusan
transisi state misi didelegasikan ke mesin-state C
(`core/src/state_machine.c` via `app/aterkia_core.py`).

Alur loop (paced `CAMERA_TARGET_FPS`, default 30 Hz):
  1. connect Pixhawk non-blokir (throttled) + baca telemetri;
  2. baca frame NAV + BAWAH (CameraManager, reconnect otomatis);
  3. submit inferensi YOLO async (worker shared, non-blokir);
  4. hitung flag vision/timer -> `sm_transition` (C) -> terapkan efek;
  5. arbitrasi KILL > MANUAL (RC/gamepad + QGC) > AUTO;
  6. failsafe (stale-link / low-batt) menimpa gerak;
  7. kirim setpoint, gambar HUD, emit paket ke GUI.

Foto misi disimpan LOKAL saja (`data/captures`, `data/waypoint_captures`) —
tanpa upload (sesuai keputusan kompetisi).

Output thruster: `SET_ATTITUDE_TARGET` via MAVLink -> Pixhawk.
"""

import math
import os
import threading
import time

import cv2
import numpy as np

from PyQt5.QtCore import QThread, pyqtSignal

from . import settings as cfg
from . import aterkia_core as core
from .logutil import get_logger, throttled
from .camera import (
    make_fallback_frame,
    pace_to_fps,
)
from .camera_manager import CameraManager
from .detection_validation import (
    validate_buoy,
    frame_brightness,
)
from .manual_link import ManualLink
from .qgc_offboard import QgcOffboard
from .output_link import OutputLink
from .safety import SafetyManager
from .yolo_worker import YoloWorker

try:
    from . import mavlink_telemetry as mavlink_mod
except Exception:  # pragma: no cover
    mavlink_mod = None

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except Exception:  # pragma: no cover - ultralytics tidak terpasang
    YOLO = None
    YOLO_AVAILABLE = False

log = get_logger()

# Fuzzy Sugeno sudah dipindah ke C (core/src/fuzzy.c via aterkia_core):
# tidak butuh objek controller, cukup fungsi murni — aktif bila C ada.
FUZZY_ENABLED = core.C_AVAILABLE

# Peta nama -> kode sub-step mundur (untuk sm_transition). Nama dibuat oleh C.
_SM_STEP_BY_NAME = {
    core.sm_retreat_step_name(core.SM_STEP_IDLE): core.SM_STEP_IDLE,
    core.sm_retreat_step_name(core.SM_STEP_START_BRAKE): core.SM_STEP_START_BRAKE,
    core.sm_retreat_step_name(core.SM_STEP_GOTO_NEUTRAL):
        core.SM_STEP_GOTO_NEUTRAL,
    core.sm_retreat_step_name(core.SM_STEP_START_REVERSE):
        core.SM_STEP_START_REVERSE,
}

# Sudut defleksi maksimum saat hindar-rintangan (rad) — dipakai untuk
# menerjemahkan bias yaw ternormalisasi C ke heading absolut.
_AVOID_MAX_DEFLECT_RAD = math.radians(25.0)

STATE_ORDER = list(core.SM_STATES)


class VisionNavigator:
    """Navigator produksi: telemetri Pixhawk nyata + mesin-state C."""

    _MID_GATE = "gate"
    _MID_BOX = "box"
    _MID_BLUE = "blue"
    _MID_RED = "red"

    def __init__(self, config):
        self.config = config
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_state = "WAITING_GPS"
        self.running = False

        # --- Telemetri NYATA (diisi tiap poll) ---
        self.mav = None
        self._mav_retry_mono = 0.0
        self._mav_port_logged = None
        self.current_lat = None
        self.current_lon = None
        self.current_yaw_rad = None
        self.current_roll_deg = 0.0
        self.current_pitch_deg = 0.0
        self.current_groundspeed = 0.0
        self.dist_to_wp = 0.0

        # --- Kamera ganda (navigasi + bawah air) ---
        self.cam = CameraManager(config)
        self.cap = self.cam.cap  # alias kompat GUI/kode lama
        self._sync_frame_dims()

        # --- Model YOLO ---
        self.gate_model = None
        self.box_model = None
        self.blue_box_model = None
        self.red_dock_model = None
        self._models_loading = False
        self._models_loaded = False
        if YOLO_AVAILABLE:
            try:
                log.info("Memuat model GATE: %s", self.config.MODEL_PATH)
                self.gate_model = YOLO(self.config.MODEL_PATH,
                                       task='detect').to(self.config.YOLO_DEVICE)
            except Exception as e:
                raise FileNotFoundError(
                    f"FATAL: gagal memuat model GATE: {e}")

        # --- Worker YOLO async (dipakai bersama simulator) ---
        self.yolo_worker = YoloWorker(self.config)
        self.yolo_worker.bind_gate_model(self.gate_model)
        self._det_cache = {}
        self._frame_seq = 0
        self._last_frame = None
        self._sec_seq = 0            # throttle baca kamera bawah (preview)
        self._last_sec_frame = None

        # --- Filter & kontroler galat (hitung di C) ---
        self.pid_gate = core.PidState(
            kp=config.PID_KP, ki=config.PID_KI, kd=config.PID_KD,
            deadband=config.PID_DEADBAND,
            output_limit=config.PID_OUTPUT_LIMIT,
            integral_limit=config.PID_INTEGRAL_LIMIT,
        )
        self.ekf_heading = core.EkfState(
            process_noise=config.EKF_PROCESS_NOISE,
            meas_noise=config.EKF_MEAS_NOISE,
        )
        self.comp_angle_prev = 0.0
        self._last_yaw_meas = None
        self._last_loop_time = time.time()
        self._loop_dt = 0.033

        # --- Sequencer gate (titik tengah merah+hijau, hitung di C) ---
        self.gate_seq = core.GateSequencerC(
            pass_distance_m=config.GATE_PASS_DISTANCE_M,
            lost_tolerance_frames=config.GATE_LOST_TOLERANCE_FRAMES,
            gate_width_m=config.GATE_WIDTH_METERS,
            focal_length_px=config.FOCAL_LENGTH_PX,
        )
        self.gate_active_mid = None
        self._gate_last_leg = -1

        # --- State misi transien ---
        self.task_timer = 0.0
        self.retreat_step = "IDLE"
        self.leg_start_lat = None
        self.leg_start_lon = None
        self.last_vision_correction_rad = 0.0
        self.last_used_p_gain = 0.0
        self.green_box_confirm_timer = None
        self.green_box_lost_timer = None
        self.blue_box_confirm_timer = None
        self.blue_box_lost_timer = None
        self._wp_photo_taken = False
        self._blue_photo_taken = False

        # --- Manual: RC/gamepad kapal + QGC (Xbox laptop) ---
        self.manual_link = ManualLink(config)
        self.qgc_link = QgcOffboard(config)
        self.manual_gui_request = False
        self.kill_request = False
        self.manual_last = {"surge": 0.0, "yaw": 0.0, "active": False,
                            "mode": 0, "mode_name": "AUTO", "rc_ok": False,
                            "source": "AUTO"}
        self.qgc_last = dict(self.manual_last)

        # --- Output ke Pixhawk + failsafe ---
        self.output = OutputLink(config)
        self.safety = SafetyManager(config, link=self.output)

        # --- Hindar-rintangan reaktif (C) ---
        self.avoid_state = {"yaw_bias": 0.0, "memory_s": 0.0}
        self._avoid_last_mono = time.monotonic()

        # Cek parameter failsafe Pixhawk sekali (thread daemon, baca saja).
        self.pixhawk_check = "menunggu MAVLink"
        threading.Thread(target=self._pixhawk_param_check_worker,
                         daemon=True).start()

        # Rekaman sesi opsional.
        self.video_writer = None

    # ------------------------------------------------------------------
    # Dimensi frame turunan
    # ------------------------------------------------------------------
    def _sync_frame_dims(self):
        self.processing_width = int(self.config.FRAME_WIDTH)
        self.processing_height = int(self.config.FRAME_HEIGHT)
        self.image_center_x = self.processing_width / 2.0
        self.image_center_y = self.processing_height / 2.0
        self.roi_top_y_cutoff = int(
            self.processing_height * self.config.ROI_TOP_CUTOFF_PERCENT)

    # ------------------------------------------------------------------
    # Muat model non-gate (lazy)
    # ------------------------------------------------------------------
    def _load_models_async(self):
        if self._models_loading or self._models_loaded or not YOLO_AVAILABLE:
            return
        self._models_loading = True

        def _loader():
            for attr, path, label in (
                    ("box_model", self.config.BOX_MODEL_PATH, "BOX HIJAU"),
                    ("red_dock_model", self.config.RED_DOCK_MODEL_PATH,
                     "BOX MERAH"),
                    ("blue_box_model", self.config.BLUE_BOX_MODEL_PATH,
                     "BOX BIRU")):
                try:
                    log.info("Memuat model %s (lazy)...", label)
                    setattr(self, attr,
                            YOLO(path, task='detect').to(self.config.YOLO_DEVICE))
                except Exception as e:
                    log.warning("Gagal memuat model %s: %s", label, e)
            self._models_loaded = True
            self._models_loading = False
            log.info("Lazy-load model selesai.")

        threading.Thread(target=_loader, daemon=True, name="lazy-models").start()

    # ------------------------------------------------------------------
    # MAVLink: connect non-blokir + telemetri
    # ------------------------------------------------------------------
    def _mav_ensure_connected(self):
        if self.mav is not None and self.mav.connected:
            return True
        if mavlink_mod is None or not mavlink_mod.MAVLINK_AVAILABLE:
            return False
        now = time.monotonic()
        interval = float(getattr(self.config, "MAV_RETRY_INTERVAL_S", 5.0))
        if now - self._mav_retry_mono < interval:
            return False
        self._mav_retry_mono = now
        try:
            self.mav = mavlink_mod.MavlinkTelemetry(
                port=self.config.SERIAL_PORT, baud=self.config.BAUD_RATE)
        except Exception:
            return False
        try:
            ok = self.mav.connect(
                timeout_s=float(getattr(self.config,
                                        "MAV_CONNECT_TIMEOUT_S", 2.0)))
        except Exception:
            ok = False
        if ok:
            try:
                self.manual_link.attach_mav(self.mav.master)
                self.qgc_link.attach_mav(self.mav.master)
                self.output.attach_mav(self.mav.master)
            except Exception:
                pass
            if os.environ.get("GCS_FORWARD", "0") == "1":
                try:
                    self.mav.enable_gcs_forward(
                        True,
                        ip=os.environ.get("GCS_IP", "127.0.0.1"),
                        port=int(os.environ.get("GCS_PORT", "14550")))
                except Exception:
                    pass
            if self._mav_port_logged != self.mav.port:
                self._mav_port_logged = self.mav.port
                log.info("Telemetri AKTIF via %s.", self.mav.port)
        return ok

    def _poll_telemetry(self):
        self._mav_ensure_connected()
        mav = self.mav
        if mav is None or not mav.connected:
            return
        try:
            mav.poll()
        except Exception:
            return
        if mav.has_attitude:
            self.safety.mark_telem()
            self.current_yaw_rad = math.radians(mav.yaw_deg)
            self.current_roll_deg = mav.roll_deg
            self.current_pitch_deg = mav.pitch_deg
        if mav.lat is not None:
            self.current_lat = mav.lat
            self.current_lon = mav.lon
            self.safety.mark_telem()
        self.current_groundspeed = mav.groundspeed

    # ------------------------------------------------------------------
    # Kamera
    # ------------------------------------------------------------------
    def _read_frames(self):
        if self.cam is not None:
            ok, frame, cam_st = self.cam.read_primary(
                self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT)
            self.cap = self.cam.cap
            if not ok or frame is None:
                frame = make_fallback_frame(
                    self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT,
                    text=("KAMERA TERPUTUS — mencoba lagi..."
                          if cam_st == "RETRY"
                          else "KAMERA BELUM ADA — colok USB / Scan"))
        else:
            frame, cam_st = make_fallback_frame(
                self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT,
                text="KAMERA BELUM ADA"), "NONE"
        sec = None
        if self.cam is not None:
            try:
                # Kamera bawah hanya untuk PREVIEW di GUI (resolusi kecil).
                # Membacanya tiap frame ikut memakan bandwidth USB + CPU;
                # cukup 1x per SECONDARY_PREVIEW_INTERVAL frame. Frame terakhir
                # disimpan agar panel GUI tidak berkedip/hilang.
                self._sec_seq += 1
                interval = max(1, int(getattr(
                    self.config, "SECONDARY_PREVIEW_INTERVAL", 3)))
                if self._last_sec_frame is None or \
                        (self._sec_seq % interval) == 0:
                    sec_ok, sec, _ = self.cam.read_secondary()
                    self._last_sec_frame = sec if (sec_ok and sec is not None) \
                        else None
                sec = self._last_sec_frame
            except Exception:
                sec = self._last_sec_frame
        return frame, sec, cam_st

    # ------------------------------------------------------------------
    # Deteksi (async worker)
    # ------------------------------------------------------------------
    def _request_detections(self, frame):
        self._frame_seq += 1
        skip = int(getattr(self.config, "YOLO_FRAME_SKIP", 2))
        if skip >= 0 and self._frame_seq % (skip + 1) != 0:
            return
        st = self.current_state
        want = None
        if st == "WAYPOINT_NAV":
            want = (self._MID_GATE, self.gate_model,
                    self.config.BUOY_CONF_SMALL_THRESHOLD)
        elif st in ("APPROACH_BOX_SEARCH", "APPROACH_BOX_ALIGN"):
            want = (self._MID_BOX, self.box_model, 0.25)
        elif st in ("APPROACH_BLUE_BOX_SEARCH", "APPROACH_BLUE_BOX_ALIGN"):
            want = (self._MID_BLUE, self.blue_box_model, 0.25)
        elif st in ("APPROACH_RED_BOX_SEARCH", "APPROACH_RED_BOX_ALIGN"):
            want = (self._MID_RED, self.red_dock_model, 0.25)
        if want is None:
            return
        mid, model, conf = want
        if model is None:
            return
        # `.copy()` sengaja dipertahankan: worker menaikkan frame ini di
        # thread lain, sementara loop utama menggambar HUD (kotak/teks) di
        # buffer yang sama. Tanpa copy, inferensi bisa "melihat" overlay HUD
        # (risiko deteksi palsu). Biaya copy (~2,7 MB @ ~10 Hz) < 1 ms —
        # bukan bottleneck (inferensi YOLO imgsz=256 ≈ 18 ms @ i3). Pindah
        # copy ke sisi tampilan justru MENAMBAH copy (tiap frame 30 Hz).
        self.yolo_worker.submit(mid, model, frame.copy(), conf)

    def _take_latest(self, model_id):
        max_age = float(getattr(self.config, "YOLO_RESULT_MAX_AGE_S", 0.5))
        raw, _age = self.yolo_worker.latest(model_id, max_age_s=max_age)
        if raw is None:
            return dict(self._det_cache.get(model_id, {}))
        if raw:
            self._det_cache[model_id] = raw
            return raw
        return dict(self._det_cache.get(model_id, {}))

    def _parse_detections(self, raw, validate_gate=False):
        """Hasil mentah -> format navigator {cls: [{cx,cy,box,area,conf}]}.

        `validate_gate=True` (model gate): ambang conf adaptif + validate_buoy
        per-class (hijau lebih longgar) + pelonggaran saat gelap. Satu
        panggilan validate_buoy per box (debug dipakai sekaligus keputusan).
        """
        detections = {}
        is_debug = bool(getattr(self.config, "DETECTION_DEBUG", False))
        bright = None
        if validate_gate and getattr(self.config, "BUOY_ADAPTIVE_ENABLED", False):
            try:
                bright = frame_brightness(self._last_frame)
            except Exception:
                bright = None
        for cls, items in (raw or {}).items():
            for b in items:
                try:
                    x1, y1, x2, y2 = b["box"]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                except (KeyError, TypeError, ValueError):
                    continue
                area = (x2 - x1) * (y2 - y1)
                conf_val = float(b.get("conf", 1.0))
                if validate_gate and cls in (self.config.RED_BALL_CLASS_ID,
                                             self.config.GREEN_BALL_CLASS_ID):
                    is_green = (cls == self.config.GREEN_BALL_CLASS_ID)
                    if is_green:
                        conf_big = getattr(self.config,
                                           "BUOY_CONF_THRESHOLD_GREEN",
                                           self.config.BUOY_CONF_THRESHOLD)
                        conf_small = getattr(self.config,
                                             "BUOY_CONF_SMALL_THRESHOLD_GREEN",
                                             self.config.BUOY_CONF_SMALL_THRESHOLD)
                        frac_cfg = getattr(self.config,
                                           "BUOY_MIN_COLOR_FRACTION_GREEN",
                                           self.config.BUOY_MIN_COLOR_FRACTION)
                        sat_cfg = getattr(self.config,
                                          "BUOY_MIN_SATURATION_GREEN",
                                          self.config.BUOY_MIN_SATURATION)
                    else:
                        conf_big = getattr(self.config,
                                           "BUOY_CONF_THRESHOLD_RED",
                                           self.config.BUOY_CONF_THRESHOLD)
                        conf_small = getattr(self.config,
                                             "BUOY_CONF_SMALL_THRESHOLD_RED",
                                             self.config.BUOY_CONF_SMALL_THRESHOLD)
                        frac_cfg = getattr(self.config,
                                           "BUOY_MIN_COLOR_FRACTION_RED",
                                           self.config.BUOY_MIN_COLOR_FRACTION)
                        sat_cfg = getattr(self.config,
                                          "BUOY_MIN_SATURATION_RED",
                                          self.config.BUOY_MIN_SATURATION)
                    conf_thresh = (conf_small
                                   if area < self.config.BUOY_SMALL_AREA_PX
                                   else conf_big)
                    if conf_val < conf_thresh:
                        continue
                    ok, reasons = validate_buoy(
                        self._last_frame, cls, (x1, y1, x2, y2),
                        min_area=self.config.MIN_BUOY_AREA_PX,
                        min_color_fraction=frac_cfg,
                        min_saturation=sat_cfg,
                        max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION,
                        debug=True, brightness=bright,
                        adaptive_enabled=getattr(
                            self.config, "BUOY_ADAPTIVE_ENABLED", False),
                        brightness_threshold=getattr(
                            self.config, "BUOY_BRIGHTNESS_THRESHOLD", 80),
                        dark_saturation=getattr(
                            self.config, "BUOY_ADAPTIVE_MIN_SATURATION", 0.20),
                        color_fraction_mult=getattr(
                            self.config, "BUOY_ADAPTIVE_COLOR_FRACTION_MULT",
                            0.5))
                    if not ok:
                        if is_debug and throttled("detect_dbg", 1.0):
                            log.info("[DETECT] buoy cls=%s area=%s ditolak: %s",
                                     cls, area, "; ".join(reasons))
                        continue
                detections.setdefault(cls, []).append(
                    {"cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2,
                     "box": (x1, y1, x2, y2), "area": area, "conf": conf_val})
        return detections

    def _detect(self, frame, model):
        self._last_frame = frame
        if model is None:
            return {}
        if model is self.gate_model:
            return self._parse_detections(self._take_latest(self._MID_GATE),
                                          validate_gate=True)
        if model is self.box_model:
            return self._parse_detections(self._take_latest(self._MID_BOX))
        if model is self.blue_box_model:
            return self._parse_detections(self._take_latest(self._MID_BLUE))
        if model is self.red_dock_model:
            return self._parse_detections(self._take_latest(self._MID_RED))
        return {}

    # ------------------------------------------------------------------
    # Vision helpers
    # ------------------------------------------------------------------
    def _find_best_box(self, detections, target_class_id, use_roi=False):
        boxes = detections.get(target_class_id, [])
        if use_roi and self.roi_top_y_cutoff > 0:
            boxes = [b for b in boxes if b['cy'] >= self.roi_top_y_cutoff]
        if not boxes:
            return None
        return max(boxes, key=lambda b: b['area'])

    def _get_distance_to_box(self, box, bwm, flp):
        try:
            if not isinstance(box, dict) or 'box' not in box:
                return float('inf')
            pixel_width = box['box'][2] - box['box'][0]
            if pixel_width < 1 or self.config.FOCAL_LENGTH_PX == 0:
                return float('inf')
            return (bwm * flp) / pixel_width
        except (ZeroDivisionError, TypeError, KeyError):
            return float('inf')

    def _track_gate(self, detections):
        if self._gate_last_leg != self.current_waypoint_index:
            self.gate_seq.reset()
            self._gate_last_leg = self.current_waypoint_index
        red_balls = detections.get(self.config.RED_BALL_CLASS_ID, [])
        green_balls = detections.get(self.config.GREEN_BALL_CLASS_ID, [])
        min_area = self.config.MIN_BUOY_AREA_PX
        red_balls = [b for b in red_balls if b['area'] >= min_area]
        green_balls = [b for b in green_balls if b['area'] >= min_area]
        if not red_balls or not green_balls:
            pairs = []
        else:
            pairs = core.collect_gate_pairs_c(
                red_balls, green_balls,
                self.config.GATE_VERTICAL_ALIGN_PX,
                self.config.GATE_AREA_SIMILARITY_RATIO)
        mid_x, mid_y, dist, is_passed = self.gate_seq.update(
            red_balls, green_balls, pairs, self.image_center_y)
        if mid_x is None:
            self.gate_active_mid = None
            return None, float('inf')
        self.gate_active_mid = (mid_x, mid_y)
        pair = self.gate_seq.active_pair
        if pair is None or is_passed:
            return None, float('inf')
        return pair, dist

    def _apply_yaw_pid(self, raw_correction_rad, dynamic_gain, dt=None):
        self.pid_gate.kp = dynamic_gain
        return self.pid_gate.update(
            raw_correction_rad, self._loop_dt if dt is None else dt)

    def _calculate_yaw_correction_gate(self, best_gate, distance_m):
        if not isinstance(best_gate, (list, tuple)) or len(best_gate) != 2:
            return 0.0
        r_ball, g_ball = best_gate
        if (not isinstance(r_ball, dict) or not isinstance(g_ball, dict)
                or 'cx' not in r_ball or 'cx' not in g_ball):
            return 0.0
        midpoint_x = (r_ball['cx'] + g_ball['cx']) / 2.0
        raw_correction_rad = core.yaw_correction_c(
            midpoint_x, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        dynamic_p_gain = self.config.VISION_P_GAIN
        if FUZZY_ENABLED:
            dynamic_p_gain = core.fuzzy_gate_gain(
                min(max(distance_m, 0.0), 2.0),
                abs(midpoint_x - self.image_center_x))
        self.last_used_p_gain = dynamic_p_gain
        return self._apply_yaw_pid(raw_correction_rad, dynamic_p_gain)

    def _calculate_yaw_correction_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box:
            return 0.0
        raw_correction_rad = core.yaw_correction_c(
            best_box['cx'], self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        self.last_used_p_gain = self.config.VISION_P_GAIN
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN)

    def _calculate_yaw_correction_blue_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box:
            return 0.0
        box_center_x_px = best_box['cx']
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0:
                return 0.0
            offset_in_pixels = (self.config.BLUE_BOX_LATERAL_OFFSET_M
                                * self.config.FOCAL_LENGTH_PX) / distance_m
            target_x_in_frame = box_center_x_px + offset_in_pixels
        except ZeroDivisionError:
            return 0.0
        raw_correction_rad = core.yaw_correction_c(
            target_x_in_frame, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        self.last_used_p_gain = self.config.VISION_P_GAIN
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN)

    def _calculate_yaw_correction_red_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box:
            return 0.0
        midpoint_x = best_box['cx']
        raw_correction_rad = core.yaw_correction_c(
            midpoint_x, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        dynamic_p_gain = self.config.VISION_P_GAIN
        if FUZZY_ENABLED:
            dynamic_p_gain = core.fuzzy_docking_gain(
                min(max(distance_m, 0.0), 10.0),
                abs(midpoint_x - self.image_center_x))
        self.last_used_p_gain = dynamic_p_gain
        return self._apply_yaw_pid(raw_correction_rad, dynamic_p_gain)

    def _get_gate_nav_yaw(self, bearing_ke_wp, best_gate, gate_distance):
        start_lat, start_lon = self.leg_start_lat, self.leg_start_lon
        if self.current_waypoint_index > 0:
            if self.current_waypoint_index < len(self.waypoints):
                prev_wp = self.waypoints[self.current_waypoint_index - 1]
                start_lat, start_lon = prev_wp['lat'], prev_wp['lon']
            else:
                start_lat, start_lon = self.current_lat, self.current_lon
        if self.current_waypoint_index >= len(self.waypoints):
            return self.current_yaw_rad or 0.0, "END_OF_MISSION"
        if start_lat is None or start_lon is None:
            start_lat, start_lon = self.current_lat, self.current_lon
        target_wp = self.waypoints[self.current_waypoint_index]
        jarak_ke_wp, _ = core.nav_distance_bearing(
            self.current_lat, self.current_lon,
            target_wp['lat'], target_wp['lon'])

        is_pre_turning = False
        next_wp_index = self.current_waypoint_index + 1
        if next_wp_index < len(self.waypoints) and jarak_ke_wp < 1.0:
            next_wp = self.waypoints[next_wp_index]
            _, bearing_next = core.nav_distance_bearing(
                self.current_lat, self.current_lon,
                next_wp['lat'], next_wp['lon'])
            bearing_ke_wp = bearing_next
            is_pre_turning = True

        cross_track = core.nav_cross_track_distance(
            self.current_lat, self.current_lon, start_lat, start_lon,
            target_wp['lat'], target_wp['lon'])

        target_yaw = bearing_ke_wp
        status = "WAYPOINT (Default)"
        is_vision_leg = self.current_waypoint_index in self.config.VISION_ENABLED_LEGS
        is_on_track = abs(cross_track) < self.config.GEOFENCE_WIDTH_METERS

        if is_vision_leg and not is_pre_turning:
            if is_on_track:
                if best_gate:
                    dt = self._loop_dt
                    gyro_rate = 0.0
                    if self._last_yaw_meas is not None:
                        gyro_rate = core.nav_normalize_angle(
                            (self.current_yaw_rad - self._last_yaw_meas)) / dt
                    self._last_yaw_meas = self.current_yaw_rad
                    self.ekf_heading.predict(gyro_rate, dt)
                    heading_est = self.ekf_heading.update(self.current_yaw_rad or 0.0)
                    raw_corr = self._calculate_yaw_correction_gate(
                        best_gate, gate_distance)
                    raw_target = core.nav_normalize_angle(heading_est + raw_corr)
                    target_yaw = core.nav_normalize_angle(
                        core.complementary_filter(
                            self.config.COMPLEMENTARY_ALPHA,
                            self.comp_angle_prev, gyro_rate, dt, raw_target))
                    self.comp_angle_prev = target_yaw
                    status = f"VISION (Dist: {gate_distance:.1f}m)"
                else:
                    status = "WAYPOINT (On Track, No Gate)"
                    self.last_vision_correction_rad = 0.0
                    self.last_used_p_gain = 0.0
            else:
                status = f"GEOFENCE_CORR (Off: {cross_track:.1f}m)"
                self.last_vision_correction_rad = 0.0
                self.last_used_p_gain = 0.0
        else:
            status = ("PRE-TURN" if is_pre_turning else "TRANSIT (Vision OFF)")
            self.last_vision_correction_rad = 0.0
            self.last_used_p_gain = 0.0
            target_yaw = bearing_ke_wp
        return target_yaw, status

    # ------------------------------------------------------------------
    # Geometri / sudut (hitung di C)
    # ------------------------------------------------------------------
    def _normalize_angle(self, angle_rad):
        return core.nav_normalize_angle(angle_rad)

    def _control_dt(self):
        now = time.time()
        dt = now - self._last_loop_time
        self._last_loop_time = now
        self._loop_dt = max(dt, 1e-4)
        return self._loop_dt

    # ------------------------------------------------------------------
    # Failsafe / hindar-rintangan
    # ------------------------------------------------------------------
    def _avoidance_scan(self):
        try:
            boxes = self.yolo_worker.latest_boxes_any(max_age_s=0.5)
        except Exception:
            boxes = []
        w = float(getattr(self.config, "FRAME_WIDTH", 640) or 640.0)
        det = None
        if boxes and w > 0:
            best = None
            for _cls, _conf, xyxy in boxes:
                try:
                    x1, y1, x2, y2 = (float(v) for v in xyxy)
                    if x2 <= x1 or y2 <= y1:
                        continue
                    w_norm = (x2 - x1) / w
                    cx_norm = ((x1 + x2) / 2.0 - w / 2.0) / (w / 2.0)
                    if w_norm >= 0.25 and -0.8 <= cx_norm <= 0.8:
                        if best is None or w_norm > best[0]:
                            best = (w_norm, cx_norm)
                except Exception:
                    continue
            if best is not None:
                det = (best[1], best[0])
        now = time.monotonic()
        dt_s = max(1e-3, now - self._avoid_last_mono)
        self._avoid_last_mono = now
        try:
            if det is not None:
                return core.avoid_update(self.avoid_state, det[0], det[1], dt_s)
            return core.avoid_update(self.avoid_state, 0.0, 0.0, dt_s)
        except Exception:
            return {"yaw_bias": 0.0, "memory_s": 0.0, "active": False}

    def _pixhawk_param_check_worker(self):
        from .pixhawk_check import load_spec, check_params, summarize
        try:
            spec = load_spec()
        except Exception as e:
            self.pixhawk_check = f"spec gagal dibaca ({e})"
            log.warning("[PIXHAWK-CHECK] %s", self.pixhawk_check)
            return
        deadline = time.time() + 10.0
        while time.time() < deadline:
            mav = self.mav
            master = mav.master if mav is not None else None
            if master is not None and mav is not None and mav.connected:
                try:
                    results = check_params(master, spec)
                except Exception as e:
                    self.pixhawk_check = f"gagal ({e})"
                    log.warning("[PIXHAWK-CHECK] %s", self.pixhawk_check)
                    return
                self.pixhawk_check = summarize(results)
                log.info("[PIXHAWK-CHECK] %s", self.pixhawk_check)
                return
            time.sleep(0.5)
        self.pixhawk_check = "MAVLink tidak tersambung (cek dilewati)"

    # ------------------------------------------------------------------
    # Foto misi (LOKAL saja)
    # ------------------------------------------------------------------
    def _capture_secondary(self):
        cap = self.cam.secondary.cap if self.cam is not None else None
        if cap is None or not cap.isOpened():
            return None
        frame = None
        for _ in range(5):
            ok, f = cap.read()
            if ok and f is not None:
                frame = f
        return frame

    def _take_photo(self, frame):
        if not getattr(self.config, "SAVE_GREEN_BOX_PHOTO", True):
            return
        try:
            os.makedirs(cfg.CAPTURES_DIR, exist_ok=True)
            path = os.path.join(cfg.CAPTURES_DIR,
                                f"WP6_GreenBox_{int(time.time())}.jpg")
            if cv2.imwrite(path, frame):
                log.info("Foto box hijau disimpan: %s", path)
        except Exception as e:
            log.warning("Gagal simpan foto box hijau: %s", e)

    def _take_waypoint_photo(self):
        frame = self._capture_secondary()
        if frame is None:
            frame = self._last_frame
        if frame is None:
            log.warning("Foto WP dilewati: tak ada frame kamera.")
            return
        try:
            os.makedirs(cfg.WAYPOINT_PHOTO_DIR, exist_ok=True)
            path = os.path.join(
                cfg.WAYPOINT_PHOTO_DIR,
                f"wp_photo_WP{self.current_waypoint_index}_{int(time.time())}.jpg")
            if cv2.imwrite(path, frame):
                log.info("Foto waypoint disimpan: %s", path)
        except Exception as e:
            log.warning("Gagal simpan foto waypoint: %s", e)

    def _smart_capture_blue(self):
        """Foto bawah air WP8 (kamera secondary, lokal saja)."""
        frame = self._capture_secondary()
        if frame is None:
            log.warning("[WP8] Gagal baca kamera bawah air.")
            return
        try:
            os.makedirs(cfg.WAYPOINT_PHOTO_DIR, exist_ok=True)
            avg = float(np.mean(frame)) if frame is not None else 0.0
            tag = "OK" if avg > 1.0 else "DARK"
            path = os.path.join(
                cfg.WAYPOINT_PHOTO_DIR,
                f"WP8_BlueBox_{tag}_{int(time.time())}.jpg")
            if cv2.imwrite(path, frame):
                log.info("[WP8] Foto bawah air disimpan: %s", path)
        except Exception as e:
            log.warning("[WP8] Gagal simpan foto bawah air: %s", e)

    # ------------------------------------------------------------------
    # Rekaman sesi (opsional)
    # ------------------------------------------------------------------
    def _start_video_recording(self):
        if not getattr(self.config, "ENABLE_SESSION_RECORDING", False):
            self.video_writer = None
            return
        try:
            os.makedirs(cfg.SESSION_VIDEO_DIR, exist_ok=True)
            path = os.path.join(cfg.SESSION_VIDEO_DIR,
                                f"session_record_{int(time.time())}.avi")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.video_writer = cv2.VideoWriter(
                path, fourcc, self.config.SESSION_VIDEO_FPS,
                (self.processing_width, self.processing_height))
            log.info("Perekam sesi mulai: %s", path)
        except Exception as e:
            log.warning("Gagal mulai perekam sesi: %s", e)
            self.video_writer = None

    def _stop_video_recording(self):
        if self.video_writer is not None:
            try:
                self.video_writer.release()
            except Exception:
                pass
            self.video_writer = None

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _set_state(self, new_state):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        if new_state == "TAKE_WAYPOINT_PHOTO":
            self._wp_photo_taken = False
        elif new_state == "TAKE_BLUE_BOX_PHOTO":
            self._blue_photo_taken = False
        log.info("STATE -> %s", new_state)

    def _apply_transition(self, res, state_before):
        """Terapkan efek samping dari `sm_transition` (C)."""
        if res["wp_inc"]:
            old = self.current_waypoint_index
            if state_before in ("RETREAT", "BLUE_BOX_RETREAT",
                                "RED_BOX_DOCKED", "TAKE_WAYPOINT_PHOTO"):
                self.leg_start_lat, self.leg_start_lon = (
                    self.current_lat, self.current_lon)
            elif 0 <= old < len(self.waypoints):
                self.leg_start_lat = self.waypoints[old]['lat']
                self.leg_start_lon = self.waypoints[old]['lon']
            self.current_waypoint_index = old + 1
        if res["reset_task_timer"]:
            self.task_timer = time.time()
        if res["reset_vision"]:
            self.last_vision_correction_rad = 0.0
        self.retreat_step = core.sm_retreat_step_name(res["retreat_step"])
        self._set_state(core.sm_state_name(res["next"]))

    # ------------------------------------------------------------------
    # HUD (digambar pada frame agar GUI tampil beranotasi)
    # ------------------------------------------------------------------
    def _visualize(self, frame, detections, best_gate, best_box,
                   best_red_box, jarak_ke_wp, box_distance,
                   red_box_distance, print_status):
        cv2.putText(frame, f"STATE: {self.current_state}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        wp_text = (f"Next WP: #{self.current_waypoint_index} "
                   f"({jarak_ke_wp:.1f} m)")
        if (not self.waypoints
                or self.current_waypoint_index >= len(self.waypoints)):
            wp_text = "MISSION COMPLETE"
        cv2.putText(frame, wp_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 0), 2)
        cv2.putText(frame, f"NAV: {print_status}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, f"P-Gain: {self.last_used_p_gain:.2f}", (10, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        min_area = self.config.MIN_BUOY_AREA_PX
        if self.current_state in ("WAYPOINT_NAV", "WAYPOINT_TRANSITION"):
            for ball in detections.get(self.config.RED_BALL_CLASS_ID, []):
                if ball['area'] >= min_area:
                    cv2.rectangle(frame, ball['box'][0:2], ball['box'][2:],
                                  (0, 0, 255), 2)
            for ball in detections.get(self.config.GREEN_BALL_CLASS_ID, []):
                if ball['area'] >= min_area:
                    cv2.rectangle(frame, ball['box'][0:2], ball['box'][2:],
                                  (0, 255, 0), 2)
            if best_gate and self.gate_active_mid is not None:
                mx, my = int(self.gate_active_mid[0]), int(self.gate_active_mid[1])
                cv2.circle(frame, (mx, my), 10, (0, 255, 255), -1)
                cv2.circle(frame, (mx, my), 13, (255, 255, 255), 2)
        elif (self.current_state.startswith("APPROACH_BOX")
              or self.current_state == "RETREAT"):
            for box in detections.get(self.config.GREEN_BOX_CLASS_ID, []):
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:],
                              (0, 255, 128), 3)
            if best_box:
                cv2.circle(frame, (best_box['cx'], best_box['cy']), 7,
                           (0, 255, 128), -1)
        elif (self.current_state.startswith("APPROACH_BLUE_BOX")
              or self.current_state in ("TAKE_BLUE_BOX_PHOTO",
                                        "BLUE_BOX_RETREAT")):
            for box in detections.get(self.config.BLUE_BOX_CLASS_ID, []):
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:],
                              (255, 128, 0), 3)
            if best_box:
                cv2.circle(frame, (best_box['cx'], best_box['cy']), 7,
                           (255, 128, 0), -1)
        elif (self.current_state.startswith("APPROACH_RED_BOX")
              or self.current_state == "RED_BOX_DOCKED"):
            for box in detections.get(self.config.RED_BOX_CLASS_ID, []):
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:],
                              (0, 128, 255), 3)
            if best_red_box:
                cv2.circle(frame, (best_red_box['cx'], best_red_box['cy']), 7,
                           (0, 128, 255), -1)
        if self.video_writer is not None:
            self.video_writer.write(frame)

    # ------------------------------------------------------------------
    # Loop utama
    # ------------------------------------------------------------------
    def run(self, data_signal):
        log.info("Navigator PRODUKSI start (OFFBOARD via Pixhawk).")
        self.running = True
        self.yolo_worker.start()
        self._load_models_async()
        self._start_video_recording()
        deadline = time.monotonic()

        # Tunggu telemetri pertama (lat + heading) tanpa membekukan GUI.
        while self.running and (self.current_lat is None
                                or self.current_yaw_rad is None):
            self._poll_telemetry()
            frame, sec, cam_st = self._read_frames()
            self.output.send(0.0, self.current_yaw_rad or 0.0, force_send=True)
            self._emit(data_signal, frame, sec, cam_st, "WAITING_GPS",
                       0.0, 0.0, 0.0, "WAITING_GPS", "AUTO",
                       {"active": False}, {"active": False}, "", {"active": False})
            deadline = pace_to_fps(deadline)
            time.sleep(0.02)

        if self.running and self.waypoints:
            self.leg_start_lat, self.leg_start_lon = (
                self.current_lat, self.current_lon)
        if self.running:
            self.output.prepare_offboard(3.0, self.current_yaw_rad or 0.0)
        self._last_loop_time = time.time()

        while self.running:
            self._control_dt()
            self._sync_frame_dims()
            self._poll_telemetry()

            # Manual: RC/gamepad kapal + QGC (Xbox laptop).
            try:
                man = self.manual_link.update(
                    dt=self._loop_dt, kill=self.kill_request,
                    gui_manual=self.manual_gui_request) or {}
            except Exception:
                man = dict(self.manual_last)
            try:
                qgc = self.qgc_link.poll(dt=self._loop_dt) or {}
            except Exception:
                qgc = {}
            self.manual_last = man
            self.qgc_last = qgc
            try:
                arb2 = core.arb_manual2(
                    bool(qgc.get("active")),
                    float(qgc.get("surge", 0.0)), float(qgc.get("yaw", 0.0)),
                    bool(man.get("active")),
                    float(man.get("surge", 0.0)), float(man.get("yaw", 0.0)))
            except Exception:
                arb2 = {"surge": 0.0, "yaw": 0.0, "source": 0,
                        "source_name": "?", "conflict": False}
            op_mode = man.get("mode_name", "AUTO")
            if arb2.get("conflict"):
                man = dict(man, active=True, surge=0.0, yaw=0.0)
                op_mode = "MANUAL"
            elif qgc.get("active"):
                man = dict(man, active=True, mode_name="MANUAL",
                           surge=float(arb2.get("surge", 0.0)),
                           yaw=float(arb2.get("yaw", 0.0)),
                           rc_ok=bool(qgc.get("rc_ok", False)), source="QGC")
                op_mode = "MANUAL"

            frame, sec, cam_st = self._read_frames()
            now = time.time()
            n_wp = len(self.waypoints)

            # --- Guard loop (C) ---
            guard = core.sm_guard(
                self.current_lat is not None and self.current_lon is not None,
                self.current_yaw_rad is not None,
                bool(self.waypoints), self.current_waypoint_index, n_wp)
            if guard != core.SM_NONE:
                st = core.sm_state_name(guard)
                self._set_state(st)
                self.output.send(0.0, self.current_yaw_rad or 0.0)
                self._emit(data_signal, frame, sec, cam_st, st, 0.0,
                           float(self.current_groundspeed), 0.0,
                           "WAITING", op_mode, man, qgc,
                           self.safety.failsafe_reason,
                           self.avoid_state)
                deadline = pace_to_fps(deadline)
                continue

            # Telemetri/waypoint sudah tersedia: bila state masih state GUARD
            # (WAITING_GPS / NO_TELEM / NO_WAYPOINTS), masuk/kembali ke
            # WAYPOINT_NAV — mesin-state C tidak punya transisi keluar guard.
            if self.current_state in ("WAITING_GPS", "NO_TELEM",
                                      "NO_WAYPOINTS"):
                self._set_state("WAYPOINT_NAV")
                if self.leg_start_lat is None:
                    self.leg_start_lat, self.leg_start_lon = (
                        self.current_lat, self.current_lon)

            target_wp = self.waypoints[self.current_waypoint_index]
            jarak_ke_wp, bearing_ke_wp = core.nav_distance_bearing(
                self.current_lat, self.current_lon,
                target_wp['lat'], target_wp['lon'])
            self.dist_to_wp = jarak_ke_wp

            thrust = self.config.THRUST_VALUE
            target_yaw = bearing_ke_wp
            lateral = 0.0
            print_status = "???"
            detections = {}
            best_gate = best_box = best_red_box = None
            gate_distance = box_distance = red_box_distance = float('inf')

            if op_mode == "AUTO":
                self._request_detections(frame)
            st = self.current_state
            inp = {}

            # ------------------- Bukan AUTO: tahan, jangan jalankan aksi misi
            # (mencegah foto/aksi terpanggil berulang saat MANUAL/KILL). -----
            if op_mode != "AUTO":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = op_mode

            # ------------------- WAYPOINT_NAV -------------------
            elif st == "WAYPOINT_NAV":
                inp["wp_idx"] = self.current_waypoint_index
                inp["n_wp"] = n_wp
                inp["wp_in_photo_box_legs"] = (
                    self.current_waypoint_index in self.config.PHOTO_BOX_LEGS)
                inp["wp_in_blue_box_legs"] = (
                    self.current_waypoint_index in self.config.BLUE_BOX_PHOTO_LEGS)
                inp["wp_in_stop_and_photo"] = (
                    self.current_waypoint_index in self.config.STOP_AND_PHOTO_AT_WP)
                inp["at_waypoint"] = jarak_ke_wp < self.config.ACCEPTANCE_RADIUS_M
                inp["green_model"] = self.box_model is not None
                inp["blue_model"] = self.blue_box_model is not None
                do_gate = (not (inp["wp_in_photo_box_legs"] and inp["green_model"])
                           and not (inp["wp_in_blue_box_legs"] and inp["blue_model"])
                           and not inp["at_waypoint"])
                if do_gate:
                    detections = self._detect(frame, self.gate_model)
                    best_gate, gate_distance = self._track_gate(detections)
                    target_yaw, print_status = self._get_gate_nav_yaw(
                        bearing_ke_wp, best_gate, gate_distance)
                    thrust = self.config.THRUST_VALUE
                else:
                    thrust = 0.0
                    target_yaw = self.current_yaw_rad or 0.0
                    print_status = "WAYPOINT_NAV"

            # ------------------- WAYPOINT_TRANSITION -------------------
            elif st == "WAYPOINT_TRANSITION":
                print_status = "TRANSITION (GPS ONLY)"
                thrust = self.config.THRUST_VALUE
                target_yaw = bearing_ke_wp
                inp["transition_over"] = (
                    (now - self.task_timer) > self.config.TRANSITION_DURATION_S)

            # ------------------- TAKE_WAYPOINT_PHOTO -------------------
            elif st == "TAKE_WAYPOINT_PHOTO":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                elapsed = now - self.task_timer
                inp["wp_idx"] = self.current_waypoint_index
                inp["n_wp"] = n_wp
                inp["wp_is_red_after"] = (
                    self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP)
                inp["red_model"] = self.red_dock_model is not None
                inp["wp_photo_over"] = (
                    elapsed > self.config.WAYPOINT_PHOTO_STOP_DURATION_S)
                if elapsed < 0.5:
                    print_status = "PHOTO_WP (Stopping..)"
                elif elapsed < 1.0:
                    if not self._wp_photo_taken:
                        self._take_waypoint_photo()
                        self._wp_photo_taken = True
                    print_status = "PHOTO_WP (Snap!)"
                else:
                    print_status = f"PHOTO_WP (Waiting {elapsed:.1f}s)"

            # ------------------- APPROACH_BOX_SEARCH -------------------
            elif st == "APPROACH_BOX_SEARCH":
                detections = self._detect(frame, self.box_model)
                best_box = self._find_best_box(
                    detections, self.config.GREEN_BOX_CLASS_ID)
                if best_box:
                    if self.green_box_confirm_timer is None:
                        self.green_box_confirm_timer = now
                    elapsed_confirm = now - self.green_box_confirm_timer
                    inp["box_found"] = 1
                    inp["box_confirmed"] = (
                        elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S)
                    thrust = 0.0
                    target_yaw = self.current_yaw_rad or 0.0
                    print_status = (f"BOX_SEARCH (Confirmed!)" if inp["box_confirmed"]
                                    else f"BOX_SEARCH (Verifying {elapsed_confirm:.1f}s)")
                else:
                    self.green_box_confirm_timer = None
                    self.green_box_lost_timer = None
                    thrust = self.config.SEARCH_THRUST
                    lateral = 0.0
                    target_yaw = self._normalize_angle(
                        (self.current_yaw_rad or 0.0)
                        + math.radians(self.config.YAW_SEARCH_BOX))
                    print_status = "BOX_SEARCH (Rotating)"

            # ------------------- APPROACH_BOX_ALIGN -------------------
            elif st == "APPROACH_BOX_ALIGN":
                detections = self._detect(frame, self.box_model)
                best_box = self._find_best_box(
                    detections, self.config.GREEN_BOX_CLASS_ID)
                if best_box:
                    self.green_box_lost_timer = None
                    inp["box_found"] = 1
                    box_distance = self._get_distance_to_box(
                        best_box, self.config.BOX_WIDTH_METERS,
                        self.config.FOCAL_LENGTH_PX)
                    if box_distance > self.config.BOX_APPROACH_DISTANCE_M:
                        raw = self._calculate_yaw_correction_box(
                            best_box, box_distance)
                        alpha = self.config.VISION_SMOOTHING_ALPHA
                        smooth = alpha * raw + (1.0 - alpha) * self.last_vision_correction_rad
                        self.last_vision_correction_rad = smooth
                        target_yaw = self._normalize_angle(
                            (self.current_yaw_rad or 0.0) + smooth)
                        thrust = self.config.ALIGN_THRUST
                        print_status = f"BOX_ALIGN (Dist: {box_distance:.1f}m)"
                    else:
                        inp["box_aligned"] = 1
                        thrust = 0.0
                        print_status = "BOX_ALIGN (Reached!)"
                else:
                    if self.green_box_lost_timer is None:
                        self.green_box_lost_timer = now
                    elapsed_lost = now - self.green_box_lost_timer
                    inp["box_lost_over_2s"] = elapsed_lost >= 2.0
                    if not inp["box_lost_over_2s"]:
                        target_yaw = self._normalize_angle(
                            (self.current_yaw_rad or 0.0)
                            + self.last_vision_correction_rad)
                        thrust = self.config.ALIGN_THRUST
                        print_status = f"BOX_ALIGN (Lost {elapsed_lost:.1f}s)"
                    else:
                        thrust = 0.0
                        print_status = "BOX_ALIGN (Lost > 2s)"

            # ------------------- TAKE_PHOTO -------------------
            elif st == "TAKE_PHOTO":
                self.output.send(0.0, self.current_yaw_rad or 0.0,
                                 force_send=True)
                time.sleep(0.2)
                self._take_photo(frame)
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = "TAKE_PHOTO"

            # ------------------- RETREAT -------------------
            elif st == "RETREAT":
                elapsed_start = now - self.task_timer
                inp["brake_over"] = elapsed_start > 0.2
                inp["neutral_over"] = elapsed_start > 0.4
                inp["retreat_over"] = elapsed_start > self.config.RETREAT_DURATION_S
                target_yaw = self.current_yaw_rad or 0.0
                if self.retreat_step == "START_BRAKE":
                    thrust = self.config.RETREAT_THRUST
                    print_status = "RETREAT (Brake)"
                elif self.retreat_step == "GOTO_NEUTRAL":
                    thrust = 0.0
                    print_status = "RETREAT (Neutral)"
                elif self.retreat_step == "START_REVERSE":
                    thrust = self.config.RETREAT_THRUST
                    print_status = "RETREAT (Reverse)"
                else:
                    thrust = 0.0
                    print_status = "RETREAT"

            # ------------------- APPROACH_BLUE_BOX_SEARCH -------------------
            elif st == "APPROACH_BLUE_BOX_SEARCH":
                detections = self._detect(frame, self.blue_box_model)
                best_box = self._find_best_box(
                    detections, self.config.BLUE_BOX_CLASS_ID, True)
                if best_box:
                    if self.blue_box_confirm_timer is None:
                        self.blue_box_confirm_timer = now
                    elapsed_confirm = now - self.blue_box_confirm_timer
                    inp["blue_found"] = 1
                    inp["blue_confirmed"] = (
                        elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S)
                    thrust = 0.0
                    target_yaw = self.current_yaw_rad or 0.0
                    print_status = (f"BLUE_BOX_SEARCH (Confirmed!)"
                                    if inp["blue_confirmed"]
                                    else f"BLUE_BOX_SEARCH (Verifying {elapsed_confirm:.1f}s)")
                else:
                    self.blue_box_confirm_timer = None
                    self.blue_box_lost_timer = None
                    thrust = self.config.BLUE_BOX_SEARCH_THRUST
                    lateral = 0.0
                    target_yaw = self._normalize_angle(
                        (self.current_yaw_rad or 0.0)
                        + math.radians(self.config.BLUE_BOX_YAW_SEARCH))
                    print_status = "BLUE_BOX_SEARCH (Rotating)"

            # ------------------- APPROACH_BLUE_BOX_ALIGN -------------------
            elif st == "APPROACH_BLUE_BOX_ALIGN":
                detections = self._detect(frame, self.blue_box_model)
                best_box = self._find_best_box(
                    detections, self.config.BLUE_BOX_CLASS_ID, True)
                if best_box:
                    self.blue_box_lost_timer = None
                    inp["blue_found"] = 1
                    box_distance = self._get_distance_to_box(
                        best_box, self.config.BLUE_BOX_WIDTH_METERS,
                        self.config.FOCAL_LENGTH_PX)
                    if box_distance > self.config.BLUE_BOX_APPROACH_DISTANCE_M:
                        raw = self._calculate_yaw_correction_blue_box(
                            best_box, box_distance)
                        alpha = self.config.VISION_SMOOTHING_ALPHA
                        smooth = alpha * raw + (1.0 - alpha) * self.last_vision_correction_rad
                        self.last_vision_correction_rad = smooth
                        target_yaw = self._normalize_angle(
                            (self.current_yaw_rad or 0.0) + smooth)
                        thrust = self.config.BLUE_BOX_ALIGN_THRUST
                        print_status = f"BLUE_BOX_ALIGN (Dist: {box_distance:.1f}m)"
                    else:
                        inp["blue_aligned"] = 1
                        thrust = 0.0
                        print_status = "BLUE_BOX_ALIGN (Reached!)"
                else:
                    if self.blue_box_lost_timer is None:
                        self.blue_box_lost_timer = now
                    elapsed_lost = now - self.blue_box_lost_timer
                    inp["blue_lost_over_2s"] = elapsed_lost >= 2.0
                    if not inp["blue_lost_over_2s"]:
                        target_yaw = self._normalize_angle(
                            (self.current_yaw_rad or 0.0)
                            + self.last_vision_correction_rad)
                        thrust = self.config.BLUE_BOX_ALIGN_THRUST
                        print_status = f"BLUE_BOX_ALIGN (Lost {elapsed_lost:.1f}s)"
                    else:
                        thrust = 0.0
                        print_status = "BLUE_BOX_ALIGN (Lost > 2s)"

            # ------------------- TAKE_BLUE_BOX_PHOTO -------------------
            elif st == "TAKE_BLUE_BOX_PHOTO":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                elapsed = now - self.task_timer
                inp["blue_photo_over"] = (
                    elapsed > (self.config.WAYPOINT_PHOTO_STOP_DURATION_S + 2.0))
                if elapsed < 2.0:
                    print_status = "PHOTO_BLUE (Stabilizing..)"
                elif elapsed < 4.0:
                    if not self._blue_photo_taken:
                        self._smart_capture_blue()
                        self._blue_photo_taken = True
                    print_status = "PHOTO_BLUE (Snap!)"
                else:
                    print_status = f"PHOTO_BLUE (Holding {elapsed:.1f}s)"

            # ------------------- BLUE_BOX_RETREAT -------------------
            elif st == "BLUE_BOX_RETREAT":
                elapsed_start = now - self.task_timer
                inp["brake_over"] = elapsed_start > 0.2
                inp["neutral_over"] = elapsed_start > 0.4
                inp["retreat_over"] = elapsed_start > self.config.RETREAT_DURATION_S
                inp["red_model"] = self.red_dock_model is not None
                target_yaw = self.current_yaw_rad or 0.0
                if self.retreat_step == "START_BRAKE":
                    thrust = self.config.RETREAT_THRUST
                    print_status = "BLUE_RETREAT (Brake)"
                elif self.retreat_step == "GOTO_NEUTRAL":
                    thrust = 0.0
                    print_status = "BLUE_RETREAT (Neutral)"
                elif self.retreat_step == "START_REVERSE":
                    thrust = self.config.RETREAT_THRUST
                    print_status = "BLUE_RETREAT (Reverse)"
                else:
                    thrust = 0.0
                    print_status = "BLUE_RETREAT"

            # ------------------- APPROACH_RED_BOX_SEARCH -------------------
            elif st == "APPROACH_RED_BOX_SEARCH":
                detections = self._detect(frame, self.red_dock_model)
                best_red_box = self._find_best_box(
                    detections, self.config.RED_BOX_CLASS_ID, True)
                if best_red_box:
                    inp["red_found"] = 1
                    thrust = 0.0
                    target_yaw = self.current_yaw_rad or 0.0
                    print_status = "RED_DOCK_SEARCH (Found!)"
                else:
                    thrust = self.config.SEARCH_THRUST
                    lateral = 0.0
                    target_yaw = self._normalize_angle(
                        (self.current_yaw_rad or 0.0)
                        + math.radians(self.config.YAW_SEARCH_DOCK))
                    print_status = "RED_DOCK_SEARCH (Rotating)"

            # ------------------- APPROACH_RED_BOX_ALIGN -------------------
            elif st == "APPROACH_RED_BOX_ALIGN":
                detections = self._detect(frame, self.red_dock_model)
                best_red_box = self._find_best_box(
                    detections, self.config.RED_BOX_CLASS_ID, True)
                if best_red_box:
                    inp["red_found"] = 1
                    red_box_distance = self._get_distance_to_box(
                        best_red_box, self.config.RED_BOX_WIDTH_METERS,
                        self.config.FOCAL_LENGTH_PX)
                    if red_box_distance > self.config.RED_BOX_DOCK_DISTANCE_M:
                        raw = self._calculate_yaw_correction_red_box(
                            best_red_box, red_box_distance)
                        alpha = 0.5
                        smooth = alpha * raw + (1.0 - alpha) * self.last_vision_correction_rad
                        self.last_vision_correction_rad = smooth
                        target_yaw = self._normalize_angle(
                            (self.current_yaw_rad or 0.0) + smooth)
                        thrust = self.config.DOCK_ALIGN_THRUST
                        print_status = f"RED_DOCK_ALIGN (Dist: {red_box_distance:.2f}m)"
                    else:
                        inp["red_aligned"] = 1
                        thrust = 0.0
                        print_status = "RED_DOCK_ALIGN (Docked!)"
                else:
                    thrust = 0.0
                    print_status = "RED_DOCK_ALIGN (Lost Target!)"

            # ------------------- RED_BOX_DOCKED -------------------
            elif st == "RED_BOX_DOCKED":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                inp["wp_idx"] = self.current_waypoint_index
                inp["n_wp"] = n_wp
                inp["dock_hold_over"] = (
                    (now - self.task_timer) > self.config.DOCK_HOLD_DURATION_S)
                print_status = f"RED_BOX_DOCKED ({now - self.task_timer:.1f}s)"

            # --- Hindar-rintangan (AUTO, saat transit) ---
            avoid = self._avoidance_scan()
            if (avoid.get("active") and op_mode == "AUTO"
                    and st in ("WAYPOINT_NAV", "WAYPOINT_TRANSITION")):
                target_yaw = self._normalize_angle(
                    target_yaw
                    + float(avoid.get("yaw_bias", 0.0)) * _AVOID_MAX_DEFLECT_RAD)
                arah = "KIRI" if avoid.get("yaw_bias", 0.0) < 0 else "KANAN"
                print_status = f"{print_status} (AVOID-{arah})"

            # --- Transisi state (C) — hanya saat AUTO ---
            if op_mode == "AUTO":
                res = core.sm_transition(
                    core.sm_state_from_name(st),
                    _SM_STEP_BY_NAME.get(self.retreat_step, core.SM_STEP_IDLE),
                    inp)
                self._apply_transition(res, st)

            # --- Arbitrasi KILL > MANUAL > AUTO ---
            if self.kill_request or op_mode == "KILL":
                thrust = 0.0
                lateral = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = "KILL (E-stop)"
                op_mode = "KILL"
                self.output.send_disarm()
            elif op_mode == "MANUAL":
                thrust = max(-1.0, min(1.0, float(man.get("surge", 0.0))))
                lateral = 0.0
                target_yaw = self._normalize_angle(
                    (self.current_yaw_rad or 0.0)
                    + float(man.get("yaw", 0.0)) * 0.8)
                print_status = (f"MANUAL ({man.get('source', '?')}) "
                                f"surge {thrust:+.2f} yaw {float(man.get('yaw', 0.0)):+.2f}")

            # --- Failsafe (prioritas tertinggi) ---
            batt = {}
            if self.mav is not None and self.mav.has_battery:
                batt = {"battery_pct": self.mav.battery_pct,
                        "voltage_v": self.mav.voltage_v,
                        "current_a": self.mav.current_a}
            fs_reason = self.safety.check(batt.get("battery_pct"),
                                          batt.get("voltage_v"))
            if fs_reason:
                thrust = 0.0
                lateral = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = f"FAILSAFE ({fs_reason})"
                op_mode = "FAILSAFE"

            # --- Kirim setpoint + HUD + emit ---
            self.output.send(thrust, target_yaw, lateral_thrust=lateral)
            self._visualize(frame, detections, best_gate, best_box,
                            best_red_box, jarak_ke_wp, box_distance,
                            red_box_distance, print_status)
            self._emit(data_signal, frame, sec, cam_st, self.current_state,
                       jarak_ke_wp, float(self.current_groundspeed),
                       getattr(self.mav, "groundspeed", 0.0), print_status,
                       op_mode, man, qgc, fs_reason, avoid, batt)

            deadline = pace_to_fps(deadline)

        self._cleanup()

    # ------------------------------------------------------------------
    # Emit paket GUI
    # ------------------------------------------------------------------
    def _emit(self, data_signal, frame, sec, cam_st, state, dist_to_wp,
              groundspeed, _gd, status, op_mode, man, qgc, fs_reason, avoid,
              batt=None):
        mav = self.mav
        pkt = {
            "lat": self.current_lat, "lon": self.current_lon,
            "yaw_deg": math.degrees(self.current_yaw_rad or 0.0),
            "pitch_deg": self.current_pitch_deg,
            "roll_deg": self.current_roll_deg,
            "state": state,
            "target_wp_idx": self.current_waypoint_index,
            "dist_to_wp_m": dist_to_wp,
            "groundspeed": groundspeed,
            "mavlink_ok": bool(getattr(mav, "connected", False)) if mav else False,
            "telem_source": "REAL",
            "gps_fix": bool(self.current_lat is not None),
            "frame": frame,
            "frame_secondary": sec,
            "cam_status": cam_st,
            "op_mode": op_mode,
            "manual_active": bool(man.get("active", False)),
            "manual_surge": float(man.get("surge", 0.0)),
            "manual_yaw": float(man.get("yaw", 0.0)),
            "rc_ok": bool(man.get("rc_ok", False)),
            "manual_source": str(man.get("source", "AUTO")),
            "qgc_active": bool(qgc.get("active", False)),
            "gcs_forward": bool(getattr(mav, "gcs_active", False)) if mav else False,
            "failsafe_active": bool(self.safety.failsafe_active),
            "failsafe_reason": str(fs_reason or ""),
            "pixhawk_check": str(getattr(self, "pixhawk_check", "N/A")),
            "avoid_active": bool(avoid.get("active", False)),
            "avoid_bias": float(avoid.get("yaw_bias", 0.0)),
            "status": status,
        }
        batt = batt or {}
        pkt["voltage_v"] = batt.get("voltage_v")
        pkt["current_a"] = batt.get("current_a")
        pkt["battery_pct"] = batt.get("battery_pct")
        try:
            data_signal.emit(pkt)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def _cleanup(self):
        log.info("Navigator cleanup...")
        self._stop_video_recording()
        try:
            self.yolo_worker.stop_and_wait()
        except Exception:
            pass
        try:
            self.output.stop_stream(times=5, yaw_rad=self.current_yaw_rad or 0.0)
        except Exception:
            pass

    def stop(self):
        self.running = False
        try:
            self.yolo_worker.stop_and_wait()
        except Exception:
            pass
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
        self.cap = None

    # API GUI: Scan kamera + ganti index saat runtime (plug-and-play).
    def rescan_cameras(self):
        if self.cam is None:
            return []
        return self.cam.rescan()

    def select_camera(self, role, index):
        if self.cam is None:
            return False
        role = str(role).upper()
        if role in ("NAV", "PRIMARY", "UTAMA"):
            ok = self.cam.select_primary(index)
            self.cap = self.cam.cap
            return ok
        return self.cam.select_secondary(index)


class NavigatorThread(QThread):
    """Wrapper QThread untuk `VisionNavigator` (kontrak sama dgn simulator)."""

    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        self.config = cfg.Config()
        self.navigator = VisionNavigator(self.config)
        self.navigator.waypoints = waypoints

    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            log.info("[TUNING] %s = %s", key, value)
        else:
            log.error("Config key '%s' tidak ada!", key)

    def save_config_to_file(self):
        cfg.save_params(self.config)

    def run(self):
        try:
            self.navigator.run(self.newData)
        except Exception as e:
            log.error("Thread navigator error: %s", e)
            import traceback
            traceback.print_exc()

    def stop(self):
        self.navigator.stop()

    def update_waypoints(self, wps):
        self.navigator.waypoints = wps
