"""
app/settings.py — Konfigurasi tunggal seluruh program ASV.

Path data distandarkan ke struktur repo:
  config/   -> tuning_params.json (disimpan GUI saat close)
  weights/  -> model YOLO *.pt      (tidak di-git; lihat scripts/download_weights.sh)
  data/     -> output runtime: captures, video session, peta sementara

Catatan migrasi: file ini dulu bernama modules/config.py. Nilai di sini
di-set SEKALI di __init__ (duplikasi class-level lama sudah dihapus).
"""

import os
import json

# --- Path standar (relative terhadap root repo, bukan folder ini) ---
APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(APP_DIR)
CONFIG_DIR = os.path.join(ROOT_DIR, "config")
WEIGHTS_DIR = os.path.join(ROOT_DIR, "weights")
DATA_DIR = os.path.join(ROOT_DIR, "data")

TUNING_FILE = os.path.join(CONFIG_DIR, "tuning_params.json")

# Output runtime (folder dibuat otomatis saat dipakai)
CAPTURES_DIR = os.path.join(DATA_DIR, "captures")
WAYPOINT_PHOTO_DIR = os.path.join(DATA_DIR, "waypoint_captures")
SESSION_VIDEO_DIR = os.path.join(DATA_DIR, "session_videos")

# --- Negosiasi kamera (dipakai app/camera.py & kelas Config) ---
# Resolusi "tertinggi" sebenarnya ditentukan kamera: permintaan di atas
# kemampuan aslinya di-clamp sendiri oleh driver. CAMERA_MAX_AUTO_* hanya
# membatasi ukuran yang boleh diminta saat probing.
CAMERA_TARGET_FPS = 30
CAMERA_MAX_AUTO_WIDTH = 1920
CAMERA_MAX_AUTO_HEIGHT = 1080
CAMERA_MIN_ACCEPT_FPS = 25


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


# Parameter yang BOLEH disimpan/upload ke tuning_params.json.
# Kunci ini dipakai bersama _load_saved_params(): key lain (path model,
# port serial, host redis, index kamera, ...) TIDAK ikut tersimpan agar
# file tetap portable antar mesin (Windows dev / Pi / laptop mana pun).
TUNING_PARAM_KEYS = frozenset({
    # video & navigasi umum
    "SESSION_VIDEO_FPS", "ACCEPTANCE_RADIUS_M", "THRUST_VALUE",
    "TRANSITION_DURATION_S", "GEOFENCE_WIDTH_METERS",
    # vision global
    "FOCAL_LENGTH_PX", "VISION_P_GAIN", "VISION_SMOOTHING_ALPHA",
    "ROI_TOP_CUTOFF_PERCENT",
    # misi gate (buoy)
    "GATE_WIDTH_METERS", "VISION_ENABLED_LEGS", "MIN_BUOY_AREA_PX",
    "GATE_AREA_SIMILARITY_RATIO",
    # misi foto waypoint
    "STOP_AND_PHOTO_AT_WP", "WAYPOINT_PHOTO_STOP_DURATION_S",
    "DETECTION_CONFIRM_DURATION_S",
    # misi box hijau
    "PHOTO_BOX_LEGS", "SEARCH_THRUST", "ALIGN_THRUST", "RETREAT_THRUST",
    "RETREAT_DURATION_S", "BOX_WIDTH_METERS", "BOX_APPROACH_DISTANCE_M",
    "YAW_SEARCH_BOX", "BOX_SEARCH_LATERAL_THRUST",
    # misi box biru
    "BLUE_BOX_PHOTO_LEGS", "BLUE_BOX_WIDTH_METERS",
    "BLUE_BOX_APPROACH_DISTANCE_M", "BLUE_BOX_LATERAL_OFFSET_M",
    "BLUE_BOX_ALIGN_THRUST", "BLUE_BOX_SEARCH_THRUST", "BLUE_BOX_YAW_SEARCH",
    # misi docking (box merah)
    "RED_BOX_WIDTH_METERS", "RED_BOX_DOCK_DISTANCE_M", "DOCK_ALIGN_THRUST",
    "DOCK_HOLD_DURATION_S", "YAW_SEARCH_DOCK",
    # YOLO
    "YOLO_FRAME_SKIP", "YOLO_INFERENCE_SIZE",
})


def save_params(config):
    """Simpan parameter tuning (hanya key di TUNING_PARAM_KEYS) ke file JSON.

    Dipanggil saat GUI ditutup dan oleh thread simulator. Atribut internal
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

        # ---------- Model YOLO & kamera ----------
        self.MODEL_PATH = os.path.join(WEIGHTS_DIR, 'buoy.pt')          # gate (2 buoy)
        self.BOX_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'box_hijau.pt')  # foto box hijau
        self.BLUE_BOX_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'best_blue_dark.pt')  # box biru (foto samping)
        self.RED_DOCK_MODEL_PATH = os.path.join(WEIGHTS_DIR, 'best_red_new.pt')  # docking box merah
        self.CAMERA_INDEX = 1                     # kamera navigasi (depan)
        self.WAYPOINT_PHOTO_CAMERA_INDEX = 1       # kamera bawah air (foto WP 8)
        # Ukuran ini hanya *default* untuk frame sintetis (kamera mati).
        # Saat kamera dibuka dengan auto_highest=True, FRAME_WIDTH/HEIGHT
        # ditimpa oleh hasil negosiasi mode (lihat app/camera.py).
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CAMERA_TARGET_FPS = CAMERA_TARGET_FPS      # fps yang diusahakan
        self.CAMERA_MAX_AUTO_WIDTH = CAMERA_MAX_AUTO_WIDTH  # batas negosiasi resolusi
        self.CAMERA_MAX_AUTO_HEIGHT = CAMERA_MAX_AUTO_HEIGHT
        self.CAMERA_MIN_ACCEPT_FPS = CAMERA_MIN_ACCEPT_FPS  # mode di bawah ini ditolak
        self.ENABLE_SESSION_RECORDING = False
        self.SESSION_VIDEO_FPS = saved.get('SESSION_VIDEO_FPS', 10.0)

        # ---------- Hardware / telemetri ----------
        self.SERIAL_PORT = 'COM8'                  # Windows dev; di Pi pakai /dev/ttyACM0
        self.BAUD_RATE = 57600
        self.OFFBOARD_STREAM_RATE_HZ = 30

        # ---------- Redis / web dashboard ----------
        self.REDIS_HOST = "localhost"
        self.REDIS_PORT = 6379
        self.TELEMETRY_CHANNEL = "asv_telemetry"
        self.VISION_CHANNEL = "asv_vision"
        self.MISSION_CHANNEL = "asv_mission"
        self.NGROK_HOST = "dashboardaterolas.app"
        self.SERVER_UPLOAD_URL = f"https://{self.NGROK_HOST}/api/images/upload"

        # ---------- Navigasi umum ----------
        self.ACCEPTANCE_RADIUS_M = saved.get('ACCEPTANCE_RADIUS_M', 2.0)
        self.THRUST_VALUE = saved.get('THRUST_VALUE', 0.9)
        self.TRANSITION_DURATION_S = saved.get('TRANSITION_DURATION_S', 0.5)
        self.GEOFENCE_WIDTH_METERS = saved.get('GEOFENCE_WIDTH_METERS', 1.0)

        # ---------- Vision global ----------
        self.FOCAL_LENGTH_PX = saved.get('FOCAL_LENGTH_PX', 400)
        self.VISION_P_GAIN = saved.get('VISION_P_GAIN', 1.9)
        self.VISION_SMOOTHING_ALPHA = saved.get('VISION_SMOOTHING_ALPHA', 0.6)
        self.ROI_TOP_CUTOFF_PERCENT = saved.get('ROI_TOP_CUTOFF_PERCENT', 0.20)

        # ---------- Misi gate (buoy) ----------
        self.GATE_WIDTH_METERS = saved.get('GATE_WIDTH_METERS', 1.0)
        self.VISION_ENABLED_LEGS = saved.get('VISION_ENABLED_LEGS', [1, 3, 5])
        self.MIN_BUOY_AREA_PX = saved.get('MIN_BUOY_AREA_PX', 80)
        self.GATE_AREA_SIMILARITY_RATIO = saved.get('GATE_AREA_SIMILARITY_RATIO', 0.5)

        # ---------- Misi foto waypoint ----------
        self.STOP_AND_PHOTO_AT_WP = saved.get('STOP_AND_PHOTO_AT_WP', [])
        self.WAYPOINT_PHOTO_STOP_DURATION_S = saved.get('WAYPOINT_PHOTO_STOP_DURATION_S', 3.0)
        self.DETECTION_CONFIRM_DURATION_S = saved.get('DETECTION_CONFIRM_DURATION_S', 0.5)

        # ---------- Misi box hijau ----------
        self.PHOTO_BOX_LEGS = saved.get('PHOTO_BOX_LEGS', [6])
        self.SEARCH_THRUST = saved.get('SEARCH_THRUST', 0.4)
        self.ALIGN_THRUST = saved.get('ALIGN_THRUST', 0.3)
        self.RETREAT_THRUST = saved.get('RETREAT_THRUST', -0.7)
        self.RETREAT_DURATION_S = saved.get('RETREAT_DURATION_S', 0.5)
        self.BOX_WIDTH_METERS = saved.get('BOX_WIDTH_METERS', 0.4)
        self.BOX_APPROACH_DISTANCE_M = saved.get('BOX_APPROACH_DISTANCE_M', 2.0)
        self.SAVE_GREEN_BOX_PHOTO = True
        self.YAW_SEARCH_BOX = saved.get('YAW_SEARCH_BOX', -90)
        self.BOX_SEARCH_LATERAL_THRUST = saved.get('BOX_SEARCH_LATERAL_THRUST', -0.2)

        # ---------- Misi box biru ----------
        self.BLUE_BOX_PHOTO_LEGS = saved.get('BLUE_BOX_PHOTO_LEGS', [8])
        self.BLUE_BOX_CLASS_ID = 0
        self.BLUE_BOX_WIDTH_METERS = saved.get('BLUE_BOX_WIDTH_METERS', 0.6)
        self.BLUE_BOX_APPROACH_DISTANCE_M = saved.get('BLUE_BOX_APPROACH_DISTANCE_M', 1.5)
        self.BLUE_BOX_LATERAL_OFFSET_M = saved.get('BLUE_BOX_LATERAL_OFFSET_M', -1.0)
        self.BLUE_BOX_ALIGN_THRUST = saved.get('BLUE_BOX_ALIGN_THRUST', 0.3)
        self.BLUE_BOX_SEARCH_THRUST = saved.get('BLUE_BOX_SEARCH_THRUST', 0.2)
        self.BLUE_BOX_YAW_SEARCH = saved.get('BLUE_BOX_YAW_SEARCH', -90)

        # ---------- Misi docking (box merah) ----------
        self.RED_BOX_CLASS_ID = 0
        self.RED_BOX_NAV_AFTER_WP = 8
        self.RED_BOX_WIDTH_METERS = saved.get('RED_BOX_WIDTH_METERS', 0.6)
        self.RED_BOX_DOCK_DISTANCE_M = saved.get('RED_BOX_DOCK_DISTANCE_M', 1.0)
        self.DOCK_ALIGN_THRUST = saved.get('DOCK_ALIGN_THRUST', 0.4)
        self.DOCK_HOLD_DURATION_S = saved.get('DOCK_HOLD_DURATION_S', 5.0)
        self.YAW_SEARCH_DOCK = saved.get('YAW_SEARCH_DOCK', 90)

        # ---------- YOLO & video ----------
        self.YOLO_FRAME_SKIP = saved.get('YOLO_FRAME_SKIP', 1)
        self.YOLO_INFERENCE_SIZE = saved.get('YOLO_INFERENCE_SIZE', 320)
        self.YOLO_HALF_PRECISION = False           # CPU tidak mendukung half precision
        self.YOLO_DEVICE = 'cpu'                   # paksa jalan di CPU
        self.RED_BALL_CLASS_ID = 1
        self.GREEN_BALL_CLASS_ID = 0