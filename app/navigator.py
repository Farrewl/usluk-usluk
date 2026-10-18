"""app/navigator.py — Navigator PRODUKSI (kapal asli, jalur kendali Pixhawk).

Posisi & heading datang dari Pixhawk NYATA (MAVLink); perintah gerak dikirim
lewat `SET_ATTITUDE_TARGET` (Pixhawk tetap pemegang ESC/mixer), dan keputusan
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
from . import geotag
from . import monitor_export
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
except Exception:
    mavlink_mod = None

# Inferensi YOLO lewat ONNX + cv2.dnn (TANPA torch/ultralytics). Lihat
# app/yolo_onnx.py. Ini memangkas ~250 MB RAM & waktu import. Model tetap
# dimuat di thread loader agar GUI + telemetri tampil lebih dulu.
try:
    from .yolo_onnx import OnnxDetector
    YOLO_AVAILABLE = True
except Exception:
    OnnxDetector = None
    YOLO_AVAILABLE = False


def _names_for(path):
    """Nama kelas untuk model ONNX (dari settings.YOLO_NAMES by basename)."""
    try:
        return cfg.YOLO_NAMES.get(os.path.basename(path), {})
    except Exception:
        return {}

log = get_logger()



FUZZY_ENABLED = core.C_AVAILABLE


_SM_STEP_BY_NAME = {
    core.sm_retreat_step_name(core.SM_STEP_IDLE): core.SM_STEP_IDLE,
    core.sm_retreat_step_name(core.SM_STEP_START_BRAKE): core.SM_STEP_START_BRAKE,
    core.sm_retreat_step_name(core.SM_STEP_GOTO_NEUTRAL):
        core.SM_STEP_GOTO_NEUTRAL,
    core.sm_retreat_step_name(core.SM_STEP_START_REVERSE):
        core.SM_STEP_START_REVERSE,
}



_AVOID_MAX_DEFLECT_RAD = math.radians(25.0)

STATE_ORDER = list(core.SM_STATES)


class VisionNavigator:
    """Navigator produksi: telemetri Pixhawk nyata + mesin-state C."""

    _MID_GATE = "gate"
    _MID_BOX = "box"
    _MID_BLUE = "blue"

    def __init__(self, config):
        self.config = config
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_state = "WAITING_GPS"
        self.running = False
        self._configure_cpu_threads()
        try:
            monitor_export.get_exporter(config)
        except Exception:
            pass


        self.mav = None
        self._mav_retry_mono = 0.0
        self._mav_port_logged = None
        self._mav_port = None
        self._mav_reason = ""
        self.current_lat = None
        self.current_lon = None
        self.current_yaw_rad = None
        self.current_roll_deg = 0.0
        self.current_pitch_deg = 0.0
        self.current_groundspeed = 0.0
        self.dist_to_wp = 0.0


        self.cam = CameraManager(config)
        self.cap = self.cam.cap
        self._sync_frame_dims()


        self.gate_model = None
        self.box_model = None
        self.blue_box_model = None
        self._models_loading = False
        self._models_loaded = False
        self.yolo_worker = YoloWorker(self.config)
        if YOLO_AVAILABLE:
            self._load_models_async()
        self._det_cache = {}
        self._frame_seq = 0
        self._last_frame = None
        self._sec_seq = 0
        self._last_sec_frame = None


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


        self.gate_seq = core.GateSequencerC(
            pass_distance_m=config.GATE_PASS_DISTANCE_M,
            lost_tolerance_frames=config.GATE_LOST_TOLERANCE_FRAMES,
            gate_width_m=config.GATE_WIDTH_METERS,
            focal_length_px=config.FOCAL_LENGTH_PX,
        )
        self.gate_active_mid = None
        self._gate_last_leg = -1
        self._gate_counted_leg = -1
        self._gate_passed_count = 0
        self._surface_taken = False
        self._under_taken = False
        self._dock_done = False


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

        self.manual_link = ManualLink(config)
        self.qgc_link = QgcOffboard(config)
        self.manual_gui_request = False
        self.kill_request = False
        # Startup aman: kapal HOLD dulu; operator wajib tekan MULAI (AUTO).
        self.auto_started = not bool(getattr(config, "STARTUP_HOLD", True))
        self.manual_last = {"surge": 0.0, "yaw": 0.0, "active": False,
                            "mode": 0, "mode_name": "AUTO", "rc_ok": False,
                            "source": "AUTO"}
        self.qgc_last = dict(self.manual_last)


        self.output = OutputLink(config)
        self.safety = SafetyManager(config, link=self.output)


        self.avoid_state = {"yaw_bias": 0.0, "memory_s": 0.0}
        # Instrumen loop (ASV_FPS_DEBUG=1): akumulasi ms per tahap + Hz.
        self._fps_dbg_on = os.environ.get("ASV_FPS_DEBUG", "") not in (
            "", "0", "false", "off", "no")
        self._fps_dbg = {"n": 0, "t0": time.monotonic(),
                         "read": 0.0, "detect": 0.0, "emit": 0.0}
        # Throttle video ke GUI (lihat _emit): ts kiriman pixmap terakhir.
        self._video_gui_last = 0.0
        self._avoid_last_mono = time.monotonic()


        self.pixhawk_check = "menunggu MAVLink"
        threading.Thread(target=self._pixhawk_param_check_worker,
                         daemon=True).start()


        self.video_writer = None




    def _sync_frame_dims(self):
        self.processing_width = int(self.config.FRAME_WIDTH)
        self.processing_height = int(self.config.FRAME_HEIGHT)
        self.image_center_x = self.processing_width / 2.0
        self.image_center_y = self.processing_height / 2.0
        self.roi_top_y_cutoff = int(
            self.processing_height * self.config.ROI_TOP_CUTOFF_PERCENT)




    def _load_models_async(self):
        if self._models_loading or self._models_loaded or not YOLO_AVAILABLE:
            return
        self._models_loading = True

        def _loader():
            if OnnxDetector is None:
                log.warning("yolo_onnx tidak tersedia — deteksi YOLO mati.")
                self._models_loading = False
                return
            for attr, path, label in (
                    ("gate_model", self.config.MODEL_PATH, "GATE BUOY"),
                    ("box_model", self.config.BOX_MODEL_PATH, "BOX HIJAU"),
                    ("blue_box_model", self.config.BLUE_BOX_MODEL_PATH,
                     "BOX BIRU")):
                try:
                    log.info("Memuat model %s (ONNX)...", label)
                    setattr(self, attr, OnnxDetector(path, _names_for(path)))
                except Exception as e:
                    log.warning("Gagal memuat model %s: %s", label, e)
                if attr == "gate_model" and self.yolo_worker is not None:
                    try:
                        self.yolo_worker.bind_gate_model(self.gate_model)
                    except Exception:
                        pass
            self._models_loaded = True
            self._models_loading = False
            log.info("Load model ONNX selesai.")

        threading.Thread(target=_loader, daemon=True, name="lazy-models").start()




    def _mav_ensure_connected(self):
        if self.mav is not None and self.mav.connected:
            return True
        if mavlink_mod is None or not mavlink_mod.MAVLINK_AVAILABLE:
            self._mav_reason = " NO-MAVLINK"
            return False
        now = time.monotonic()
        interval = float(getattr(self.config, "MAV_RETRY_INTERVAL_S", 5.0))
        if now - self._mav_retry_mono < interval:
            return False
        self._mav_retry_mono = now
        # Tutup koneksi lama sebelum ganti (cegah kebocoran handle serial).
        if self.mav is not None:
            try:
                self.mav.close()
            except Exception:
                pass
            self.mav = None
        try:
            self.mav = mavlink_mod.MavlinkTelemetry(
                port=self.config.SERIAL_PORT, baud=self.config.BAUD_RATE)
        except Exception:
            self._mav_reason = " ERR"
            return False
        self._mav_port = self.mav.port
        try:
            ok = self.mav.connect(
                timeout_s=float(getattr(self.config,
                                        "MAV_CONNECT_TIMEOUT_S", 2.0)))
        except Exception:
            ok = False
        if ok:
            self._mav_reason = ""
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
        elif getattr(self.mav, "busy", False):
            self._mav_reason = " BUSY(QGC?)"
        elif self.mav.port is None:
            self._mav_reason = " NO-PORT"
        else:
            self._mav_reason = " NO-HB"
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




    def _request_detections(self, frame):
        self._frame_seq += 1
        skip = int(getattr(self.config, "YOLO_FRAME_SKIP", 2))
        if skip >= 0 and self._frame_seq % (skip + 1) != 0:
            return
        st = self.current_state
        want = None
        if st in ("WAITING_GPS", "NO_TELEM", "NO_WAYPOINTS"):
            if getattr(self.config, "DETECT_CONTINUOUS", True):
                want = (self._MID_GATE, self.gate_model,
                        self.config.BUOY_CONF_SMALL_THRESHOLD)
        elif st == "WAYPOINT_NAV":
            want = (self._MID_GATE, self.gate_model,
                    self.config.BUOY_CONF_SMALL_THRESHOLD)
        elif st in ("APPROACH_BOX_SEARCH", "APPROACH_BOX_ALIGN"):
            want = (self._MID_BOX, self.box_model, 0.25)
        elif st in ("APPROACH_BLUE_BOX_SEARCH", "APPROACH_BLUE_BOX_ALIGN",
                    "APPROACH_BLUE_BOX_APPROACH"):
            # Docking biru = deteksi lewat KAMERA BAWAH (bukan depan).
            want = (self._MID_BLUE, self.blue_box_model, 0.25)
            bot = self._bottom_frame()
            if bot is not None:
                frame = bot
        if want is None:
            return
        mid, model, conf = want
        if model is None:
            return






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
        return {}




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
        if is_passed and self._gate_counted_leg != self.current_waypoint_index:
            self._gate_counted_leg = self.current_waypoint_index
            self._gate_passed_count = min(10, self._gate_passed_count + 1)
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

    def _blue_approach_thrust(self, distance_m):
        """Deselerasi maju: thrust makin kecil saat makin dekat kotak biru.

        thrust = clamp(map(distance, [DOCK_DIST, APPROACH_DIST],
                            [0, BLUE_BOX_ALIGN_THRUST]))
        """
        dock_d = float(getattr(self.config, "BLUE_BOX_DOCK_DISTANCE_M", 0.6))
        appr_d = float(getattr(self.config, "BLUE_BOX_APPROACH_DISTANCE_M", 1.5))
        max_t = float(getattr(self.config, "BLUE_BOX_ALIGN_THRUST", 0.3))
        if appr_d <= dock_d:
            return 0.0
        t = (distance_m - dock_d) / (appr_d - dock_d)
        t = max(0.0, min(1.0, t))
        return max_t * t

    def _single_buoy_avoid(self, detections, base_yaw):
        """Hanya 1 buoy terlihat (tak form gate): belok menjauh.

        - Buoy HIJAU terlihat sendiri -> belok KANAN (buang hijau ke kiri).
        - Buoy MERAH terlihat sendiri -> belok KIRI  (buang merah ke kanan).
        Aktif hanya bila buoy cukup dekat (<= SINGLE_BUOY_CLEARANCE_M).
        Return (yaw_baru, status) atau (None, None) bila tak aktif.
        """
        if not bool(getattr(self.config, "SINGLE_BUOY_ENABLED", True)):
            return None, None
        min_area = self.config.MIN_BUOY_AREA_PX
        red = [b for b in detections.get(self.config.RED_BALL_CLASS_ID, [])
               if b['area'] >= min_area]
        green = [b for b in detections.get(self.config.GREEN_BALL_CLASS_ID, [])
                 if b['area'] >= min_area]
        if (red and green) or (not red and not green):
            return None, None  # sudah gate penuh, atau tak ada buoy
        # Pilih buoy terdekat (area terbesar sebagai proksi jarak).
        solo = (red or green)[0]
        for b in (red or green):
            if b['area'] > solo['area']:
                solo = b
        # Estimasi jarak monokular dari lebar buoy (asumsi lebar fisik gate).
        dist = self._get_distance_to_box(
            solo, float(getattr(self.config, "GATE_WIDTH_METERS", 0.6)),
            self.config.FOCAL_LENGTH_PX)
        if dist > float(getattr(self.config, "SINGLE_BUOY_CLEARANCE_M", 2.5)):
            return None, None
        # Belok menjauh: hijau di kanan frame -> belok kanan; merah -> kiri.
        is_green = bool(green)
        steer = +1.0 if is_green else -1.0
        bias = math.radians(20.0) * steer
        yaw = self._normalize_angle(base_yaw + bias)
        color = "HIJAU" if is_green else "MERAH"
        return yaw, f"AVOID-{color} ({dist:.1f}m)"

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




    def _normalize_angle(self, angle_rad):
        return core.nav_normalize_angle(angle_rad)

    def _control_dt(self):
        now = time.time()
        dt = now - self._last_loop_time
        self._last_loop_time = now
        self._loop_dt = max(dt, 1e-4)
        return self._loop_dt




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
        from .pixhawk_check import (load_spec, check_params, summarize,
                                    detect_autopilot, spec_path_for,
                                    firmware_name)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            mav = self.mav
            master = mav.master if mav is not None else None
            if master is not None and mav is not None and mav.connected:
                ap = detect_autopilot(master)
                fw = firmware_name(ap)
                try:
                    spec = load_spec(spec_path_for(ap))
                except Exception as e:
                    self.pixhawk_check = f"spec gagal dibaca ({e})"
                    log.warning("[PIXHAWK-CHECK] %s", self.pixhawk_check)
                    return
                try:
                    results = check_params(master, spec)
                except Exception as e:
                    self.pixhawk_check = f"gagal ({e})"
                    log.warning("[PIXHAWK-CHECK] %s", self.pixhawk_check)
                    return
                self.pixhawk_check = f"[{fw}] {summarize(results)}"
                log.info("[PIXHAWK-CHECK] %s", self.pixhawk_check)
                return
            time.sleep(0.5)
        self.pixhawk_check = "MAVLink tidak tersambung (cek dilewati)"




    def _bottom_frame(self):
        """Frame kamera bawah (BAWAH) untuk deteksi docking biru, atau None.

        Membaca kamera bawah tiap dipanggil (tanpa throttle preview) supaya
        deteksi selalu pakai frame segar. Flip ikut CAMERA_FLIP_MODE.
        """
        if self.cam is None:
            return None
        try:
            ok, frame, _ = self.cam.read_secondary(
                max_width=int(getattr(self.config, "FRAME_WIDTH", 1280)))
            return frame if ok and frame is not None else None
        except Exception:
            return None

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

    def _geotag_and_register(self, kind, path):
        """Tulis sidecar geo-tag + daftarkan ke ekspor monitoring."""
        gs = float(getattr(self, "current_groundspeed", 0.0) or 0.0)
        cog = math.degrees(self.current_yaw_rad or 0.0)
        info = geotag.finalize_capture(
            kind, path, self.current_lat, self.current_lon,
            gs * 1.943844, gs * 3.6, cog)
        try:
            monitor_export.register_photo(info)
        except Exception:
            pass
        return info

    def _take_photo(self, frame):
        if not getattr(self.config, "SAVE_GREEN_BOX_PHOTO", True):
            return
        try:
            os.makedirs(cfg.CAPTURES_DIR, exist_ok=True)
            path = os.path.join(cfg.CAPTURES_DIR,
                                f"WP6_GreenBox_{int(time.time())}.jpg")
            if cv2.imwrite(path, frame):
                self._surface_taken = True
                self._geotag_and_register("surface", path)
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
                self._geotag_and_register("wp", path)
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
                self._under_taken = True
                self._geotag_and_register("under", path)
                log.info("[WP8] Foto bawah air disimpan: %s", path)
        except Exception as e:
            log.warning("[WP8] Gagal simpan foto bawah air: %s", e)




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




    def start_auto(self):
        """Operator tekan MULAI: minta Pixhawk masuk mode misi (AUTO/GUIDED).

        Sebelum ini kapal tetap HOLD (aman). Dipanggil dari GUI thread —
        hanya menyalakan flag & kirim DO_SET_MODE; loop yang bergerak.
        """
        self.auto_started = True
        try:
            self.output.request_offboard()
        except Exception as e:
            log.warning("Gagal kirim OFFBOARD: %s", e)
        log.info("Operator: MULAI AUTO — kapal lepas dari HOLD.")

    def stop_auto(self):
        """Operator tekan HOLD: kapal berhenti di tempat, kembali HOLD."""
        self.auto_started = False
        try:
            self.output.request_hold()
        except Exception as e:
            log.warning("Gagal kirim HOLD: %s", e)
        log.info("Operator: HOLD — kapal ditahan di tempat.")

    def _sync_auto_from_qgc(self):
        """Mirror custom_mode HEARTBEAT (QGC) -> auto_started.

        Bila operator mengubah mode lewat QGroundControl (AUTO/GUIDED),
        navigator ikut start; kalau ke MANUAL/HOLD, navigator ikut hold.
        """
        if self.mav is None:
            return
        mid = getattr(self.mav, "flight_mode_id", None)
        if mid is None:
            return
        auto_ids = {int(getattr(self.config, "FLIGHT_MODE_AUTO", 10)),
                    int(getattr(self.config, "FLIGHT_MODE_OFFBOARD", 15))}
        hold_ids = {int(getattr(self.config, "FLIGHT_MODE_HOLD", 4)),
                    int(getattr(self.config, "FLIGHT_MODE_MANUAL", 0))}
        if mid in auto_ids:
            self.auto_started = True
        elif mid in hold_ids:
            self.auto_started = False

    def _set_state(self, new_state):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        if new_state == "TAKE_WAYPOINT_PHOTO":
            self._wp_photo_taken = False
        log.info("STATE -> %s", new_state)

    def _apply_transition(self, res, state_before):
        """Terapkan efek samping dari `sm_transition` (C)."""
        if res["wp_inc"]:
            old = self.current_waypoint_index
            if state_before in ("RETREAT", "BLUE_BOX_DOCKED",
                                "TAKE_WAYPOINT_PHOTO"):
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




    def _visualize(self, frame, detections, best_gate, best_box,
                   jarak_ke_wp, box_distance, print_status):
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
              or self.current_state == "BLUE_BOX_DOCKED"):
            for box in detections.get(self.config.BLUE_BOX_CLASS_ID, []):
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:],
                              (255, 128, 0), 3)
            if best_box:
                cv2.circle(frame, (best_box['cx'], best_box['cy']), 7,
                           (255, 128, 0), -1)
        if self.video_writer is not None:
            self.video_writer.write(frame)




    def _configure_cpu_threads(self):
        """Batasi thread cv2 (torch diatur di thread loader model)."""
        n = int(getattr(self.config, "PERF_THREADS", 0) or 0)
        if n <= 0:
            try:
                n = max(1, (os.cpu_count() or 2) // 2)
            except Exception:
                n = 2
        self._cpu_threads = n
        try:
            cv2.setNumThreads(n)
        except Exception:
            pass
        log.info("CPU threads (cv2) = %d", n)




    def _fps_debug_tick(self, t_read, t_detect, t_emit):
        """Instrumentasi loop (hanya bila env ASV_FPS_DEBUG=1).

        Sekali/dtk cetak rata-rata ms tiap tahap + Hz loop nyata. Dipakai
        untuk tunjuk bottleneck (baca kamera vs deteksi vs emit/GUI) tanpa
        nebak. Diamkan bila env tak diset — nol biaya di produksi.
        """
        d = self._fps_dbg
        d["n"] += 1
        d["read"] += t_read
        d["detect"] += t_detect
        d["emit"] += t_emit
        now = time.monotonic()
        if now - d["t0"] < 1.0:
            return
        n = max(1, d["n"])
        elapsed = now - d["t0"]
        log.info(
            "[FPS-DEBUG] loop=%.1fHz | baca=%.1fms deteksi=%.1fms "
            "emit/GUI=%.1fms | sisa=%.1fms",
            d["n"] / elapsed,
            1000.0 * d["read"] / n, 1000.0 * d["detect"] / n,
            1000.0 * d["emit"] / n,
            max(0.0, 1000.0 * elapsed / n
                - 1000.0 * (d["read"] + d["detect"] + d["emit"]) / n))
        d["n"] = 0
        d["read"] = d["detect"] = d["emit"] = 0.0
        d["t0"] = now

    def run(self, data_signal):
        log.info("Navigator PRODUKSI start (OFFBOARD via Pixhawk).")
        self.running = True
        self.yolo_worker.start()
        self._load_models_async()
        self._start_video_recording()
        deadline = time.monotonic()


        while self.running and (self.current_lat is None
                                or self.current_yaw_rad is None):
            self._poll_telemetry()
            _t0 = time.monotonic() if self._fps_dbg_on else 0.0
            frame, sec, cam_st = self._read_frames()
            _t_read = (time.monotonic() - _t0) if self._fps_dbg_on else 0.0
            self.output.send(0.0, self.current_yaw_rad or 0.0, force_send=True)
            _t0e = time.monotonic() if self._fps_dbg_on else 0.0
            self._emit(data_signal, frame, sec, cam_st, "WAITING_GPS",
                       0.0, 0.0, 0.0, "WAITING_GPS", "AUTO",
                       {"active": False}, {"active": False}, "", {"active": False})
            _t_emit = (time.monotonic() - _t0e) if self._fps_dbg_on else 0.0
            if frame is not None and not cam_st.startswith("NO"):
                _t0d = time.monotonic() if self._fps_dbg_on else 0.0
                self._request_detections(frame)
                if self._fps_dbg_on:
                    self._fps_debug_tick(_t_read, time.monotonic() - _t0d,
                                         _t_emit)
            elif self._fps_dbg_on:
                self._fps_debug_tick(_t_read, 0.0, _t_emit)
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

            self._sync_auto_from_qgc()
            if not self.auto_started:
                # Startup aman / operator tekan HOLD: kapal ditahan.
                op_mode = "HOLD"

            _t0 = time.monotonic() if self._fps_dbg_on else 0.0
            frame, sec, cam_st = self._read_frames()
            _t_read = (time.monotonic() - _t0) if self._fps_dbg_on else 0.0
            _t_detect = 0.0
            now = time.time()
            n_wp = len(self.waypoints)


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
            best_gate = best_box = None
            gate_distance = box_distance = float('inf')

            if op_mode == "AUTO":
                _t0 = time.monotonic() if self._fps_dbg_on else 0.0
                self._request_detections(frame)
                if self._fps_dbg_on:
                    _t_detect = time.monotonic() - _t0
            st = self.current_state
            inp = {}



            if op_mode != "AUTO":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = op_mode


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
                    if best_gate is None:
                        # Tak ada pasangan gate — coba single-buoy avoid.
                        avoid_yaw, avoid_st = self._single_buoy_avoid(
                            detections, target_yaw)
                        if avoid_yaw is not None:
                            target_yaw, print_status = avoid_yaw, avoid_st
                    thrust = self.config.THRUST_VALUE
                else:
                    thrust = 0.0
                    target_yaw = self.current_yaw_rad or 0.0
                    print_status = "WAYPOINT_NAV"


            elif st == "WAYPOINT_TRANSITION":
                print_status = "TRANSITION (GPS ONLY)"
                thrust = self.config.THRUST_VALUE
                target_yaw = bearing_ke_wp
                inp["transition_over"] = (
                    (now - self.task_timer) > self.config.TRANSITION_DURATION_S)


            elif st == "TAKE_WAYPOINT_PHOTO":
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                elapsed = now - self.task_timer
                inp["wp_idx"] = self.current_waypoint_index
                inp["n_wp"] = n_wp
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


            elif st == "TAKE_PHOTO":
                self.output.send(0.0, self.current_yaw_rad or 0.0,
                                 force_send=True)
                time.sleep(0.2)
                self._take_photo(frame)
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                print_status = "TAKE_PHOTO"


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
                        print_status = "BLUE_BOX_ALIGN (Aligned!)"
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

            elif st == "APPROACH_BLUE_BOX_APPROACH":
                # Docking: maju DESELERASI ke kotak biru (kamera bawah),
                # parkir di samping (lateral offset) sampai jarak <= dock.
                detections = self._detect(frame, self.blue_box_model)
                best_box = self._find_best_box(
                    detections, self.config.BLUE_BOX_CLASS_ID, True)
                if best_box:
                    self.blue_box_lost_timer = None
                    inp["blue_found"] = 1
                    box_distance = self._get_distance_to_box(
                        best_box, self.config.BLUE_BOX_WIDTH_METERS,
                        self.config.FOCAL_LENGTH_PX)
                    raw = self._calculate_yaw_correction_blue_box(
                        best_box, box_distance)
                    alpha = self.config.VISION_SMOOTHING_ALPHA
                    smooth = alpha * raw + (1.0 - alpha) * self.last_vision_correction_rad
                    self.last_vision_correction_rad = smooth
                    target_yaw = self._normalize_angle(
                        (self.current_yaw_rad or 0.0) + smooth)
                    lateral = float(
                        getattr(self.config, "BLUE_BOX_LATERAL_THRUST", -0.2))
                    thrust = self._blue_approach_thrust(box_distance)
                    inp["blue_close"] = int(
                        box_distance <= self.config.BLUE_BOX_DOCK_DISTANCE_M)
                    print_status = (f"BLUE_DOCK (Dist {box_distance:.2f}m "
                                    f"Thrust {thrust:.2f})")
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
                        print_status = f"BLUE_DOCK (Lost {elapsed_lost:.1f}s)"
                    else:
                        thrust = 0.0
                        print_status = "BLUE_DOCK (Lost > 2s)"

            elif st == "BLUE_BOX_DOCKED":
                # Berhenti di samping kotak biru: thrust 0, TETAP armed.
                thrust = 0.0
                target_yaw = self.current_yaw_rad or 0.0
                inp["dock_hold_over"] = (
                    (now - self.task_timer) > self.config.DOCK_HOLD_DURATION_S)
                print_status = f"BLUE_BOX_DOCKED ({now - self.task_timer:.1f}s)"
            avoid = self._avoidance_scan()
            if (avoid.get("active") and op_mode == "AUTO"
                    and st in ("WAYPOINT_NAV", "WAYPOINT_TRANSITION")):
                target_yaw = self._normalize_angle(
                    target_yaw
                    + float(avoid.get("yaw_bias", 0.0)) * _AVOID_MAX_DEFLECT_RAD)
                arah = "KIRI" if avoid.get("yaw_bias", 0.0) < 0 else "KANAN"
                print_status = f"{print_status} (AVOID-{arah})"


            if op_mode == "AUTO":
                res = core.sm_transition(
                    core.sm_state_from_name(st),
                    _SM_STEP_BY_NAME.get(self.retreat_step, core.SM_STEP_IDLE),
                    inp)
                self._apply_transition(res, st)


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


            self.output.send(thrust, target_yaw, lateral_thrust=lateral)
            self._visualize(frame, detections, best_gate, best_box,
                            jarak_ke_wp, box_distance, print_status)
            _t0 = time.monotonic() if self._fps_dbg_on else 0.0
            self._emit(data_signal, frame, sec, cam_st, self.current_state,
                       jarak_ke_wp, float(self.current_groundspeed),
                       getattr(self.mav, "groundspeed", 0.0), print_status,
                       op_mode, man, qgc, fs_reason, avoid, batt)
            if self._fps_dbg_on:
                self._fps_debug_tick(_t_read, _t_detect,
                                     time.monotonic() - _t0)

            deadline = pace_to_fps(deadline)

        self._cleanup()




    def _annotated_frame(self, frame):
        """Frame ter-annotasi (kotak deteksi) bila ada hasil YOLO segar.

        Sebelumnya frame mentah selalu dipublish ke GUI/monitor meski worker
        menyimpan versi ter-annotasi — operator tidak pernah melihat kotak.
        Ambil hasil terbaru semua model yang umurnya <= 1 s; kalau ada,
        publish versi annotate (bukan frame mentah).
        """
        if frame is None or not getattr(self.config, "DETECT_CONTINUOUS", True):
            return frame
        try:
            now = time.monotonic()
            best = None
            best_age = None
            for mid in (self._MID_GATE, self._MID_BOX,
                        self._MID_BLUE):
                st = self.yolo_worker.latest_status(mid)
                if st is None:
                    continue
                _detected, annotated, _boxes, ts = st
                if annotated is None:
                    continue
                age = now - ts
                if age > 1.0:
                    continue
                if best_age is None or age < best_age:
                    best, best_age = annotated, age
            if best is not None:
                return best
        except Exception:
            pass
        return frame

    def _emit(self, data_signal, frame, sec, cam_st, state, dist_to_wp,
              groundspeed, _gd, status, op_mode, man, qgc, fs_reason, avoid,
              batt=None):
        frame = self._annotated_frame(frame)
        mav = self.mav
        # Throttle kiriman video ke GUI (VIDEO_GUI_HZ). Kerja berat ada di
        # thread Qt (resize INTER_AREA + QImage + setPixmap) — dikirim tiap
        # loop (30 Hz x 2 panel) membuat Qt thread jenuh dan loop navigator
        # berebut CPU. Telemetri/status TETAP full-rate; hanya frame video
        # yang di-throttle. Deteksi & navigasi tak terpengaruh (frame lokal
        # tetap dipakai penuh di loop).
        _now = time.monotonic()
        _hz = float(getattr(self.config, "VIDEO_GUI_HZ", 0) or 0)
        if _hz > 0.0 and (_now - self._video_gui_last) < (1.0 / _hz):
            video_due = False
        else:
            video_due = True
            self._video_gui_last = _now
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
            "mav_port": getattr(self, "_mav_port", None),
            "mav_reason": getattr(self, "_mav_reason", ""),
            "telem_source": "REAL",
            "gps_fix": bool(self.current_lat is not None),
            "frame": frame if video_due else None,
            "frame_secondary": sec if video_due else None,
            "video_due": video_due,
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

        gs = float(self.current_groundspeed or 0.0)
        pkt["sog_kn"] = gs * 1.943844
        pkt["sog_kmh"] = gs * 3.6
        pkt["cog_deg"] = pkt["yaw_deg"]
        pkt["wp_total"] = len(self.waypoints or [])
        self._publish_monitor(pkt, frame, sec, state, op_mode, status)
        try:
            data_signal.emit(pkt)
        except Exception:
            pass

    def _publish_monitor(self, pkt, frame, sec, state, op_mode, status):
        """Ekspor snapshot + frame ke file bridge (data/monitor) tanpa blokir."""
        try:
            if state == "BLUE_BOX_DOCKED":
                self._dock_done = True
            wp_idx = int(getattr(self, "current_waypoint_index", 0) or 0)
            mission = {
                "prep": bool(pkt.get("mavlink_ok") or pkt.get("gps_fix")),
                "start": (wp_idx >= 1),
                "balls": int(getattr(self, "_gate_passed_count", 0)),
                "surface": bool(getattr(self, "_surface_taken", False)),
                "under": bool(getattr(self, "_under_taken", False)),
                "dock": bool(getattr(self, "_dock_done", False)),
            }
            vision = {"gate_passed": mission["balls"], "models": {}}
            try:
                now = time.monotonic()
                for mid in (self._MID_GATE, self._MID_BOX,
                            self._MID_BLUE):
                    st = self.yolo_worker.latest_status(mid)
                    if st is None:
                        vision["models"][mid] = {"detected": False, "n": 0}
                        continue
                    detected, _annotated, boxes, ts = st
                    vision["models"][mid] = {
                        "detected": bool(detected),
                        "n": len(boxes or []),
                        "age_s": round(max(0.0, now - ts), 3),
                    }
            except Exception:
                pass
            camera = {"status": pkt.get("cam_status")}
            monitor_export.publish(pkt, frame=frame, frame_secondary=sec,
                                   mission=mission, vision=vision,
                                   camera=camera)
        except Exception:
            pass




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
        try:
            monitor_export.shutdown()
        except Exception:
            pass


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
    """Wrapper QThread untuk `VisionNavigator`."""

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
            if key == "SERIAL_PORT":
                # Paksa percobaan sambung ulang SEGERA (abaikan interval retry).
                self.navigator._mav_retry_mono = 0.0
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
