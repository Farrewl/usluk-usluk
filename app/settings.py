"""
app/settings.py — Konfigurasi tunggal seluruh program ASV.

Path data distandarkan ke struktur repo:
  config/   -> site.json + trackA/trackB.json + tuning_params.json
  weights/  -> model YOLO *.onnx    (IKUT ter-commit di repo)
  data/     -> output runtime: captures, video session, peta sementara

Catatan migrasi: file ini dulu bernama modules/config.py. Nilai di sini
di-set SEKALI di __init__ (duplikasi class-level lama sudah dihapus).
"""

import os
import json


APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(APP_DIR)
CONFIG_DIR = os.path.join(ROOT_DIR, "config")
WEIGHTS_DIR = os.path.join(ROOT_DIR, "weights")
DATA_DIR = os.path.join(ROOT_DIR, "data")

# Nama kelas tiap model ONNX (urutan index HARUS sama dengan saat training).
# Ditaruh di sini (bukan dibaca dari metadata .onnx) agar runtime tak butuh
# paket `onnx`. Kunci = nama file di weights/.
YOLO_NAMES = {
    "buoy.onnx": {0: "green_ball", 1: "red_ball"},
    "box_hijau.onnx": {0: "Box Hijau", 1: "Buoy Hijau", 2: "Buoy Merah"},
    "best_blue_dark.onnx": {0: "Blue-Box"},
    "best_red_new.onnx": {0: "Red-Box"},
}

TUNING_FILE = os.path.join(CONFIG_DIR, "tuning_params.json")
SITE_FILE = os.path.join(CONFIG_DIR, "site.json")


CAPTURES_DIR = os.path.join(DATA_DIR, "captures")
WAYPOINT_PHOTO_DIR = os.path.join(DATA_DIR, "waypoint_captures")
SESSION_VIDEO_DIR = os.path.join(DATA_DIR, "session_videos")


_DEFAULT_SITE = {
    "TEAM_NAME": "Aterkia",
    "TEAM_PT": "Universitas Diponegoro",
    "TRACK": "A",
    # Ekspor monitoring ke web terpisah (file bridge, lihat README.md)
    "MONITOR_ENABLED": True,
    "MONITOR_DIR": "data/monitor",
    "MONITOR_HZ": 5.0,
    "MONITOR_FRAME": True,
    "MONITOR_FRAME_HZ": 10.0,
    "MONITOR_FRAME_WIDTH": 640,
    "MONITOR_JPEG_QUALITY": 70,
    "MONITOR_EVENTS": True,
    "GRID_CELL_M": 5,
    "GRID_COLS": 5,
    "GRID_ROWS": 5,
    "SHOW_SPEED_KMH": True,
}

_SITE_ENV = {
    "ASV_TEAM": "TEAM_NAME",
    "ASV_PT": "TEAM_PT",
    "ASV_TRACK": "TRACK",
    "ASV_MONITOR": "MONITOR_ENABLED",
    "ASV_MONITOR_DIR": "MONITOR_DIR",
    "ASV_MONITOR_HZ": "MONITOR_HZ",
}


def _coerce(value, default):
    if isinstance(default, bool):
        return str(value).strip().lower() in ("1", "true", "yes", "on")
    if isinstance(default, int):
        try:
            return int(value)
        except (TypeError, ValueError):
            return default
    if isinstance(default, float):
        try:
            return float(value)
        except (TypeError, ValueError):
            return default
    return value


def load_site():
    """Baca config/site.json + override env (ASV_*). Selalu ada nilai default."""
    data = dict(_DEFAULT_SITE)
    if os.path.exists(SITE_FILE):
        try:
            with open(SITE_FILE, "r", encoding="utf-8") as f:
                data.update(json.load(f))
        except Exception as e:
            print(f"PERINGATAN: gagal baca {SITE_FILE}, pakai default. {e}")
    for env, key in _SITE_ENV.items():
        raw = os.environ.get(env)
        if raw is not None and raw != "":
            data[key] = _coerce(raw, _DEFAULT_SITE.get(key))
    return data


SITE = load_site()


def save_site(patch):
    """Gabung `patch` ke config/site.json tanpa menimpa key lain."""
    data = {}
    if os.path.exists(SITE_FILE):
        try:
            with open(SITE_FILE, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            data = {}
    data.update(patch)
    try:
        os.makedirs(CONFIG_DIR, exist_ok=True)
        with open(SITE_FILE, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4)
    except Exception as e:
        print(f"ERROR: gagal simpan {SITE_FILE}: {e}")
    return data


def load_track(name=None):
    """Baca config/track<NAME>.json. `name` None -> site.json TRACK."""
    name = str(name or SITE.get("TRACK") or "A").strip().upper()
    path = os.path.join(CONFIG_DIR, f"track{name}.json")
    trk = {"name": name, "start_color": "red", "waypoints": [],
           "vision_legs": [], "photo_box_legs": [6],
           "blue_box_photo_legs": [8], "red_box_nav_after_wp": -1,
           "stop_and_photo_at_wp": [],
           "grid": {"origin": "START", "cell_m": 5, "cols": 5, "rows": 5}}
    if os.path.exists(path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                trk.update(json.load(f))
        except Exception as e:
            print(f"PERINGATAN: gagal baca {path}, pakai default. {e}")
    else:
        print(f"PERINGATAN: {path} tidak ada, pakai default lintasan {name}.")
    trk["name"] = name
    if not trk.get("vision_legs"):
        trk["vision_legs"] = list(range(1, max(1, len(trk.get("waypoints", [])))))
    return trk


def track_waypoints(name=None):
    """Daftar waypoint [{lat,lon}] untuk lintasan `name`."""
    clean = []
    for wp in load_track(name).get("waypoints", []):
        try:
            clean.append({"lat": float(wp["lat"]), "lon": float(wp["lon"])})
        except (KeyError, TypeError, ValueError):
            continue
    return clean












CAMERA_TARGET_FPS = 30
NAV_LOOP_FPS = 30
CAMERA_MAX_AUTO_WIDTH = 1920
CAMERA_MAX_AUTO_HEIGHT = 1080
CAMERA_MIN_ACCEPT_FPS = 25
GUI_TELEMETRY_HZ = 5
GUI_MAP_HZ = 1
# Rate kirim frame video ke panel GUI (resize + QImage + setPixmap di thread
# Qt = kerja berat). Loop navigasi/deteksi TETAP di NAV_LOOP_FPS; hanya
# kiriman pixmap ke GUI yang di-throttle. 0 = tanpa throttle.
VIDEO_GUI_HZ = 12
CAMERA_FLIP_MODE = 1


def _load_saved_params():
    """Baca parameter tuning tersimpan (kalau ada)."""
    if not os.path.exists(TUNING_FILE):
        return {}
    try:
        with open(TUNING_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"PERINGATAN: Gagal membaca {TUNING_FILE}, pakai default. Error: {e}")
        return {}






TUNING_PARAM_KEYS = frozenset({

    "SESSION_VIDEO_FPS", "ACCEPTANCE_RADIUS_M", "THRUST_VALUE",
    "TRANSITION_DURATION_S", "GEOFENCE_WIDTH_METERS",

    "FOCAL_LENGTH_PX", "VISION_P_GAIN", "VISION_SMOOTHING_ALPHA",
    "ROI_TOP_CUTOFF_PERCENT",

    "GATE_WIDTH_METERS", "VISION_ENABLED_LEGS", "MIN_BUOY_AREA_PX",
    "GATE_AREA_SIMILARITY_RATIO", "GATE_VERTICAL_ALIGN_PX",
    "GATE_PASS_DISTANCE_M", "GATE_LOST_TOLERANCE_FRAMES",
    "CAMERA_FLIP_MODE",

    "BUOY_CONF_THRESHOLD", "BUOY_MIN_COLOR_FRACTION",
    "BUOY_MIN_SATURATION", "BUOY_MAX_ASPECT_DEVIATION",

    "BUOY_CONF_SMALL_THRESHOLD", "BUOY_SMALL_AREA_PX",
    "SINGLE_BUOY_ENABLED", "SINGLE_BUOY_CLEARANCE_M",
    "BUOY_TRACK_MATCH_PX", "BUOY_TRACK_BOOST",

    "BUOY_ADAPTIVE_ENABLED", "BUOY_BRIGHTNESS_THRESHOLD",
    "BUOY_ADAPTIVE_MIN_SATURATION", "BUOY_ADAPTIVE_MIN_VALUE",
    "BUOY_ADAPTIVE_COLOR_FRACTION_MULT",

    "BUOY_CONF_THRESHOLD_GREEN", "BUOY_CONF_SMALL_THRESHOLD_GREEN",
    "BUOY_CONF_THRESHOLD_RED", "BUOY_CONF_SMALL_THRESHOLD_RED",
    "BUOY_MIN_COLOR_FRACTION_GREEN", "BUOY_MIN_COLOR_FRACTION_RED",
    "BUOY_MIN_SATURATION_GREEN", "BUOY_MIN_SATURATION_RED",
    "DETECTION_DEBUG",

    "PID_KP", "PID_KI", "PID_KD", "PID_DEADBAND", "PID_OUTPUT_LIMIT",
    "PID_INTEGRAL_LIMIT", "COMPLEMENTARY_ALPHA",
    "EKF_PROCESS_NOISE", "EKF_MEAS_NOISE",

    "STOP_AND_PHOTO_AT_WP", "WAYPOINT_PHOTO_STOP_DURATION_S",
    "DETECTION_CONFIRM_DURATION_S",

    "PHOTO_BOX_LEGS", "SEARCH_THRUST", "ALIGN_THRUST", "RETREAT_THRUST",
    "RETREAT_DURATION_S", "BOX_WIDTH_METERS", "BOX_APPROACH_DISTANCE_M",
    "YAW_SEARCH_BOX", "BOX_SEARCH_LATERAL_THRUST",

    "BLUE_BOX_PHOTO_LEGS", "BLUE_BOX_WIDTH_METERS",
    "BLUE_BOX_APPROACH_DISTANCE_M", "BLUE_BOX_LATERAL_OFFSET_M",
    "BLUE_BOX_ALIGN_THRUST", "BLUE_BOX_SEARCH_THRUST", "BLUE_BOX_YAW_SEARCH",
    "BLUE_BOX_DOCK_DISTANCE_M", "BLUE_BOX_LATERAL_THRUST",

    "RED_BOX_WIDTH_METERS", "RED_BOX_DOCK_DISTANCE_M", "DOCK_ALIGN_THRUST",
    "DOCK_HOLD_DURATION_S", "YAW_SEARCH_DOCK",

    "MANUAL_ENABLED", "MANUAL_MAX_SURGE", "MANUAL_MAX_YAW",
    "MANUAL_DEADBAND", "MANUAL_EXPO", "MANUAL_RATE_LIMIT",
    "RC_TIMEOUT_MS", "RC_CH_THROTTLE", "RC_CH_YAW", "RC_CH_MODE",
    "RC_CH_DEADMAN", "MANUAL_LOST_HOLD_S",

    "QGC_MANUAL_TIMEOUT_S",

    "FAILSAFE_ENABLED", "FAILSAFE_TELEM_TIMEOUT_S",
    "FAILSAFE_LOW_BATT_PCT", "FAILSAFE_LOW_VOLT_V",
    "FAILSAFE_LOW_BATT_HOLD_S",

    "YOLO_FRAME_SKIP", "YOLO_INFERENCE_SIZE", "YOLO_RESULT_MAX_AGE_S",

    "SECONDARY_PREVIEW_INTERVAL", "SECONDARY_FPS", "SECONDARY_CACHE_WIDTH",
    "VIDEO_GUI_HZ",

    "MAV_CONNECT_TIMEOUT_S", "MAV_RETRY_INTERVAL_S",

    "PERF_THREADS",
})


def save_params(config):
    """Simpan parameter tuning (hanya key di TUNING_PARAM_KEYS) ke file JSON.

    Dipanggil saat GUI ditutup. Atribut internal
    (path model, port, host, index kamera) sengaja TIDAK ikut tersimpan
    supaya tuning_params.json tetap portable antar mesin & antar tim.
    """
    editable = {
        key: value
        for key, value in vars(config).items()
        if key in TUNING_PARAM_KEYS
    }
    try:
        os.makedirs(CONFIG_DIR, exist_ok=True)
        with open(TUNING_FILE, 'w') as f:
            json.dump(editable, f, indent=4)
        print(f"Parameter tuning disimpan ke {TUNING_FILE}")
    except Exception as e:
        print(f"ERROR: Gagal menyimpan parameter tuning: {e}")


class Config:
    def __init__(self):
        saved = _load_saved_params()


        self.TEAM_NAME = SITE["TEAM_NAME"]
        self.TEAM_PT = SITE["TEAM_PT"]
        self.TRACK = SITE["TRACK"]
        self.MONITOR_ENABLED = bool(SITE["MONITOR_ENABLED"])
        self.MONITOR_DIR = str(SITE["MONITOR_DIR"])
        self.MONITOR_HZ = float(SITE["MONITOR_HZ"])
        self.MONITOR_FRAME = bool(SITE["MONITOR_FRAME"])
        self.MONITOR_FRAME_HZ = float(SITE["MONITOR_FRAME_HZ"])
        self.MONITOR_FRAME_WIDTH = int(SITE["MONITOR_FRAME_WIDTH"])
        self.MONITOR_JPEG_QUALITY = int(SITE["MONITOR_JPEG_QUALITY"])
        self.MONITOR_EVENTS = bool(SITE["MONITOR_EVENTS"])
        self.GRID_CELL_M = float(SITE["GRID_CELL_M"])
        self.GRID_COLS = int(SITE["GRID_COLS"])
        self.GRID_ROWS = int(SITE["GRID_ROWS"])
        self.SHOW_SPEED_KMH = bool(SITE["SHOW_SPEED_KMH"])
        _trk = load_track(self.TRACK)
        self.TRACK_DATA = _trk
        self.TRACK_START = {"lat": _trk.get("start_lat"),
                            "lon": _trk.get("start_lon"),
                            "color": _trk.get("start_color", "red")}
        self.TRACK_GRID = _trk.get("grid", {})
        self.WAYPOINTS = track_waypoints(self.TRACK)



        self.PERF_THREADS = int(saved.get('PERF_THREADS', 0) or 0)


        self.MODEL_PATH = os.path.join(WEIGHTS_DIR, 'buoy.onnx')
        self.BOX_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'box_hijau.onnx')
        self.BLUE_BOX_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'best_blue_dark.onnx')
        self.RED_DOCK_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'best_red_new.onnx')
        self.CAMERA_INDEX = saved.get('CAMERA_INDEX', 1)



        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CAMERA_TARGET_FPS = CAMERA_TARGET_FPS
        try:
            self.NAV_LOOP_FPS = saved.get('NAV_LOOP_FPS', NAV_LOOP_FPS)
        except Exception:
            self.NAV_LOOP_FPS = CAMERA_TARGET_FPS
        self.CAMERA_MAX_AUTO_WIDTH = CAMERA_MAX_AUTO_WIDTH
        self.CAMERA_MAX_AUTO_HEIGHT = CAMERA_MAX_AUTO_HEIGHT
        self.CAMERA_MIN_ACCEPT_FPS = CAMERA_MIN_ACCEPT_FPS

        # Fast-start kamera: buka sekali (mode cache/1280x720 MJPG) tanpa
        # sweep resolusi/codec berulang. Negosiasi penuh
        # tetap bisa diminta manual via tombol Scan GUI. Matikan: ASV_CAM_FAST=0.
        _env_fast = os.environ.get("ASV_CAM_FAST", "").strip().lower()
        self.CAMERA_FAST_START = _env_fast not in ("0", "false", "off", "no")

        # Deteksi kontinu: model GATE ikut dijalankan saat pra-misi
        # (WAITING_GPS dst) dan frame ter-annotasi dipublish ke GUI + monitor.
        # Matikan: ASV_DETECT=0.
        _env_det = os.environ.get("ASV_DETECT", "").strip().lower()
        self.DETECT_CONTINUOUS = _env_det not in ("0", "false", "off", "no")


        self.CAMERA_FLIP_MODE = saved.get('CAMERA_FLIP_MODE', CAMERA_FLIP_MODE)
        self.ENABLE_SESSION_RECORDING = False
        self.SESSION_VIDEO_FPS = saved.get('SESSION_VIDEO_FPS', 10.0)
        self.NAV_LOOP_FPS = saved.get('NAV_LOOP_FPS', NAV_LOOP_FPS)

        self.SERIAL_PORT = saved.get('SERIAL_PORT', '')
        self.BAUD_RATE = 57600
        self.OFFBOARD_STREAM_RATE_HZ = 30





        self.MAV_CONNECT_TIMEOUT_S = saved.get('MAV_CONNECT_TIMEOUT_S', 2.0)
        self.MAV_RETRY_INTERVAL_S = saved.get('MAV_RETRY_INTERVAL_S', 5.0)


        self.WAYPOINT_PHOTO_CAMERA_INDEX = saved.get('WAYPOINT_PHOTO_CAMERA_INDEX', 0)


        self.ACCEPTANCE_RADIUS_M = saved.get('ACCEPTANCE_RADIUS_M', 2.0)
        self.THRUST_VALUE = saved.get('THRUST_VALUE', 0.9)
        self.TRANSITION_DURATION_S = saved.get('TRANSITION_DURATION_S', 0.5)
        self.GEOFENCE_WIDTH_METERS = saved.get('GEOFENCE_WIDTH_METERS', 1.0)


        self.FOCAL_LENGTH_PX = saved.get('FOCAL_LENGTH_PX', 400)
        self.VISION_P_GAIN = saved.get('VISION_P_GAIN', 1.9)
        self.VISION_SMOOTHING_ALPHA = saved.get('VISION_SMOOTHING_ALPHA', 0.6)
        self.ROI_TOP_CUTOFF_PERCENT = saved.get('ROI_TOP_CUTOFF_PERCENT', 0.20)


        self.GATE_WIDTH_METERS = saved.get('GATE_WIDTH_METERS', 1.0)
        self.VISION_ENABLED_LEGS = _trk.get(
            "vision_legs", saved.get('VISION_ENABLED_LEGS', [1, 3, 5]))
        self.MIN_BUOY_AREA_PX = saved.get('MIN_BUOY_AREA_PX', 16)
        self.GATE_AREA_SIMILARITY_RATIO = saved.get('GATE_AREA_SIMILARITY_RATIO', 0.5)


        self.GATE_VERTICAL_ALIGN_PX = saved.get('GATE_VERTICAL_ALIGN_PX', 75)


        self.GATE_PASS_DISTANCE_M = saved.get('GATE_PASS_DISTANCE_M', 1.2)
        self.GATE_LOST_TOLERANCE_FRAMES = saved.get('GATE_LOST_TOLERANCE_FRAMES', 5)




        self.BUOY_CONF_THRESHOLD = saved.get('BUOY_CONF_THRESHOLD', 0.35)
        self.BUOY_MIN_COLOR_FRACTION = saved.get('BUOY_MIN_COLOR_FRACTION', 0.05)
        self.BUOY_MIN_SATURATION = saved.get('BUOY_MIN_SATURATION', 0.35)
        self.BUOY_MAX_ASPECT_DEVIATION = saved.get('BUOY_MAX_ASPECT_DEVIATION', 0.5)

        self.BUOY_CONF_THRESHOLD_GREEN = saved.get('BUOY_CONF_THRESHOLD_GREEN', 0.30)
        self.BUOY_CONF_SMALL_THRESHOLD_GREEN = saved.get('BUOY_CONF_SMALL_THRESHOLD_GREEN', 0.15)
        self.BUOY_MIN_COLOR_FRACTION_GREEN = saved.get('BUOY_MIN_COLOR_FRACTION_GREEN', 0.04)
        self.BUOY_MIN_SATURATION_GREEN = saved.get('BUOY_MIN_SATURATION_GREEN', 0.30)
        self.BUOY_CONF_THRESHOLD_RED = saved.get('BUOY_CONF_THRESHOLD_RED', 0.35)
        self.BUOY_CONF_SMALL_THRESHOLD_RED = saved.get('BUOY_CONF_SMALL_THRESHOLD_RED', 0.20)
        self.BUOY_MIN_COLOR_FRACTION_RED = saved.get('BUOY_MIN_COLOR_FRACTION_RED', 0.05)
        self.BUOY_MIN_SATURATION_RED = saved.get('BUOY_MIN_SATURATION_RED', 0.35)

        self.BUOY_CONF_SMALL_THRESHOLD = saved.get('BUOY_CONF_SMALL_THRESHOLD', 0.20)
        self.SINGLE_BUOY_ENABLED = saved.get('SINGLE_BUOY_ENABLED', True)
        self.SINGLE_BUOY_CLEARANCE_M = saved.get('SINGLE_BUOY_CLEARANCE_M', 2.5)
        self.BUOY_SMALL_AREA_PX = saved.get('BUOY_SMALL_AREA_PX', 80)


        self.BUOY_TRACK_MATCH_PX = saved.get('BUOY_TRACK_MATCH_PX', 30)
        self.BUOY_TRACK_BOOST = saved.get('BUOY_TRACK_BOOST', 1.5)

        self.BUOY_ADAPTIVE_ENABLED = saved.get('BUOY_ADAPTIVE_ENABLED', True)

        self.BUOY_BRIGHTNESS_THRESHOLD = saved.get('BUOY_BRIGHTNESS_THRESHOLD', 80)

        self.BUOY_ADAPTIVE_MIN_SATURATION = saved.get('BUOY_ADAPTIVE_MIN_SATURATION', 0.20)
        self.BUOY_ADAPTIVE_MIN_VALUE = saved.get('BUOY_ADAPTIVE_MIN_VALUE', 20)

        self.BUOY_ADAPTIVE_COLOR_FRACTION_MULT = saved.get(
            'BUOY_ADAPTIVE_COLOR_FRACTION_MULT', 0.5)



        self.DETECTION_DEBUG = saved.get('DETECTION_DEBUG', False)



        self.PID_KP = saved.get('PID_KP', 1.0)
        self.PID_KI = saved.get('PID_KI', 0.0)
        self.PID_KD = saved.get('PID_KD', 0.0)
        self.PID_DEADBAND = saved.get('PID_DEADBAND', 0.01)
        self.PID_OUTPUT_LIMIT = saved.get('PID_OUTPUT_LIMIT', 0.6)
        self.PID_INTEGRAL_LIMIT = saved.get('PID_INTEGRAL_LIMIT', 0.3)


        self.COMPLEMENTARY_ALPHA = saved.get('COMPLEMENTARY_ALPHA', 0.6)

        self.EKF_PROCESS_NOISE = saved.get('EKF_PROCESS_NOISE', 0.05)
        self.EKF_MEAS_NOISE = saved.get('EKF_MEAS_NOISE', 0.10)


        self.STOP_AND_PHOTO_AT_WP = _trk.get(
            "stop_and_photo_at_wp", saved.get('STOP_AND_PHOTO_AT_WP', []))
        self.WAYPOINT_PHOTO_STOP_DURATION_S = saved.get('WAYPOINT_PHOTO_STOP_DURATION_S', 3.0)
        self.DETECTION_CONFIRM_DURATION_S = saved.get('DETECTION_CONFIRM_DURATION_S', 0.5)


        self.PHOTO_BOX_LEGS = _trk.get(
            "photo_box_legs", saved.get('PHOTO_BOX_LEGS', [6]))
        self.SEARCH_THRUST = saved.get('SEARCH_THRUST', 0.4)
        self.ALIGN_THRUST = saved.get('ALIGN_THRUST', 0.3)
        self.RETREAT_THRUST = saved.get('RETREAT_THRUST', -0.7)
        self.RETREAT_DURATION_S = saved.get('RETREAT_DURATION_S', 0.5)
        self.BOX_WIDTH_METERS = saved.get('BOX_WIDTH_METERS', 0.4)
        self.BOX_APPROACH_DISTANCE_M = saved.get('BOX_APPROACH_DISTANCE_M', 2.0)
        self.SAVE_GREEN_BOX_PHOTO = True
        self.YAW_SEARCH_BOX = saved.get('YAW_SEARCH_BOX', -90)
        self.BOX_SEARCH_LATERAL_THRUST = saved.get('BOX_SEARCH_LATERAL_THRUST', -0.2)


        self.BLUE_BOX_PHOTO_LEGS = _trk.get(
            "blue_box_photo_legs", saved.get('BLUE_BOX_PHOTO_LEGS', [8]))
        self.BLUE_BOX_CLASS_ID = 0
        self.BLUE_BOX_WIDTH_METERS = saved.get('BLUE_BOX_WIDTH_METERS', 0.6)
        self.BLUE_BOX_APPROACH_DISTANCE_M = saved.get('BLUE_BOX_APPROACH_DISTANCE_M', 1.5)
        self.BLUE_BOX_LATERAL_OFFSET_M = saved.get('BLUE_BOX_LATERAL_OFFSET_M', -1.0)
        self.BLUE_BOX_ALIGN_THRUST = saved.get('BLUE_BOX_ALIGN_THRUST', 0.3)
        self.BLUE_BOX_SEARCH_THRUST = saved.get('BLUE_BOX_SEARCH_THRUST', 0.2)
        self.BLUE_BOX_YAW_SEARCH = saved.get('BLUE_BOX_YAW_SEARCH', -90)
        self.BLUE_BOX_DOCK_DISTANCE_M = saved.get('BLUE_BOX_DOCK_DISTANCE_M', 0.6)
        self.BLUE_BOX_LATERAL_THRUST = saved.get('BLUE_BOX_LATERAL_THRUST', -0.2)
        self.DOCK_HOLD_DURATION_S = saved.get('DOCK_HOLD_DURATION_S', 2.0)

        # Mode ArduRover (ROVER): MANUAL=0 HOLD=4 AUTO=10 GUIDED=15 RTL=11.
        # OFFBOARD disetarakan dengan GUIDED (ArduRover tak punya OFFBOARD).
        self.FLIGHT_MODE_MANUAL = int(saved.get('FLIGHT_MODE_MANUAL', 0))
        self.FLIGHT_MODE_HOLD = int(saved.get('FLIGHT_MODE_HOLD', 4))
        self.FLIGHT_MODE_AUTO = int(saved.get('FLIGHT_MODE_AUTO', 10))
        self.FLIGHT_MODE_OFFBOARD = int(saved.get('FLIGHT_MODE_OFFBOARD', 15))
        # Startup aman: Kapal HOLD dulu; operator wajib tekan MULAI (AUTO).
        self.STARTUP_HOLD = bool(saved.get('STARTUP_HOLD', True))


        self.RED_BOX_CLASS_ID = 0
        self.RED_BOX_NAV_AFTER_WP = _trk.get(
            "red_box_nav_after_wp", 8)
        self.RED_BOX_WIDTH_METERS = saved.get('RED_BOX_WIDTH_METERS', 0.6)
        self.RED_BOX_DOCK_DISTANCE_M = saved.get('RED_BOX_DOCK_DISTANCE_M', 1.0)
        self.DOCK_ALIGN_THRUST = saved.get('DOCK_ALIGN_THRUST', 0.4)
        self.YAW_SEARCH_DOCK = saved.get('YAW_SEARCH_DOCK', 90)





        self.MANUAL_ENABLED = saved.get('MANUAL_ENABLED', True)
        self.MANUAL_MAX_SURGE = saved.get('MANUAL_MAX_SURGE', 0.6)
        self.MANUAL_MAX_YAW = saved.get('MANUAL_MAX_YAW', 0.7)
        self.MANUAL_DEADBAND = saved.get('MANUAL_DEADBAND', 0.05)
        self.MANUAL_EXPO = saved.get('MANUAL_EXPO', 0.3)
        self.MANUAL_RATE_LIMIT = saved.get('MANUAL_RATE_LIMIT', 2.0)
        self.RC_TIMEOUT_MS = saved.get('RC_TIMEOUT_MS', 500)
        self.RC_CH_THROTTLE = saved.get('RC_CH_THROTTLE', 3)
        self.RC_CH_YAW = saved.get('RC_CH_YAW', 4)
        self.RC_CH_MODE = saved.get('RC_CH_MODE', 5)
        self.RC_CH_DEADMAN = saved.get('RC_CH_DEADMAN', 7)
        self.MANUAL_LOST_HOLD_S = saved.get('MANUAL_LOST_HOLD_S', 1.0)



        self.QGC_MANUAL_TIMEOUT_S = saved.get('QGC_MANUAL_TIMEOUT_S', 0.5)






        self.FAILSAFE_ENABLED = saved.get('FAILSAFE_ENABLED', True)
        self.FAILSAFE_TELEM_TIMEOUT_S = saved.get(
            'FAILSAFE_TELEM_TIMEOUT_S', 2.0)
        self.FAILSAFE_LOW_BATT_PCT = saved.get(
            'FAILSAFE_LOW_BATT_PCT', 20.0)
        self.FAILSAFE_LOW_VOLT_V = saved.get(
            'FAILSAFE_LOW_VOLT_V', 13.2)
        self.FAILSAFE_LOW_BATT_HOLD_S = saved.get(
            'FAILSAFE_LOW_BATT_HOLD_S', 3.0)


        self.YOLO_FRAME_SKIP = saved.get('YOLO_FRAME_SKIP', 2)
        self.YOLO_INFERENCE_SIZE = saved.get('YOLO_INFERENCE_SIZE', 256)


        self.SECONDARY_PREVIEW_INTERVAL = saved.get(
            'SECONDARY_PREVIEW_INTERVAL', 3)
        self.SECONDARY_FPS = saved.get('SECONDARY_FPS', 15)
        self.SECONDARY_CACHE_WIDTH = saved.get('SECONDARY_CACHE_WIDTH', 640)
        self.VIDEO_GUI_HZ = saved.get('VIDEO_GUI_HZ', VIDEO_GUI_HZ)



        self.YOLO_RESULT_MAX_AGE_S = saved.get('YOLO_RESULT_MAX_AGE_S', 0.5)
        self.RED_BALL_CLASS_ID = 1
        self.GREEN_BALL_CLASS_ID = 0
        self.GREEN_BOX_CLASS_ID = 0