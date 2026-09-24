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
#
# CAMERA_TARGET_FPS = 20: loop video & pacing menargetkan 20 fps stabil
# (negosiasi tetap boleh memilih mode yang nyata-nya > 20, mis. 1280x720
# MJPG ~30 fps — pacing loop yang menurunkannya ke 20, bukan kamera).
CAMERA_TARGET_FPS = 23
CAMERA_MAX_AUTO_WIDTH = 1920
CAMERA_MAX_AUTO_HEIGHT = 1080
# Minimum fps yang MASIH diterima saat negosiasi mode. Diturunkan ke 15
# supaya resolusi tinggi tetap lolos walau target pacing hanya 20.
CAMERA_MIN_ACCEPT_FPS = 18

# Orientasi frame kamera: 0 = normal, 1 = mirror kiri-kanan (flip
# horizontal), 2 = atas-bawah (flip vertikal), 3 = 180 derajat. Nilai 0
# berarti "tidak reverse" — gambar persis seperti keluar dari sensor.
# Operator yang merasa feed tampak seperti selfie bisa set 1 di
# config/tuning_params.json tanpa mengubah kode.
CAMERA_FLIP_MODE = 0


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
    "GATE_AREA_SIMILARITY_RATIO", "GATE_VERTICAL_ALIGN_PX",
    "GATE_PASS_DISTANCE_M", "GATE_LOST_TOLERANCE_FRAMES",
    # filter warna/bentuk buoy (lihat app/detection_validation.py)
    "BUOY_CONF_THRESHOLD", "BUOY_MIN_COLOR_FRACTION",
    "BUOY_MIN_SATURATION", "BUOY_MAX_ASPECT_DEVIATION",
    # adaptif deteksi jauh
    "BUOY_CONF_SMALL_THRESHOLD", "BUOY_SMALL_AREA_PX",
    "BUOY_TRACK_MATCH_PX", "BUOY_TRACK_BOOST",
    "DETECTION_DEBUG",
    # filter & kontroler galat (lihat app/filtering.py == core C)
    "PID_KP", "PID_KI", "PID_KD", "PID_DEADBAND", "PID_OUTPUT_LIMIT",
    "PID_INTEGRAL_LIMIT", "COMPLEMENTARY_ALPHA",
    "EKF_PROCESS_NOISE", "EKF_MEAS_NOISE",
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
        self.CAMERA_FLIP_MODE = CAMERA_FLIP_MODE  # orientasi frame (0 = tidak reverse)
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
        self.MIN_BUOY_AREA_PX = saved.get('MIN_BUOY_AREA_PX', 16)
        self.GATE_AREA_SIMILARITY_RATIO = saved.get('GATE_AREA_SIMILARITY_RATIO', 0.5)
        # Maksimum selisih sumbu-Y (piksel) antara buoy merah & hijau agar
        # keduanya dianggap satu "gate" sejajar di frame.
        self.GATE_VERTICAL_ALIGN_PX = saved.get('GATE_VERTICAL_ALIGN_PX', 75)
        # Ambang lolos/tenggelamnya target gate aktif: saat kapal melewati
        # titik tengah buoy, target otomatis pindah ke berikutnya.
        self.GATE_PASS_DISTANCE_M = saved.get('GATE_PASS_DISTANCE_M', 1.2)
        self.GATE_LOST_TOLERANCE_FRAMES = saved.get('GATE_LOST_TOLERANCE_FRAMES', 5)
        # Filter false-positive buoy: warna + bentuk + confidence
        # (lihat app/detection_validation.py). Nilai longgar (0.35/0.05/0.35)
        # supaya buoy ASLI tetap lolos walau lighting buruk; wajah operator
        # tetap tertolak karena hue kulit bukan merah/hijau.
        self.BUOY_CONF_THRESHOLD = saved.get('BUOY_CONF_THRESHOLD', 0.35)
        self.BUOY_MIN_COLOR_FRACTION = saved.get('BUOY_MIN_COLOR_FRACTION', 0.05)
        self.BUOY_MIN_SATURATION = saved.get('BUOY_MIN_SATURATION', 0.35)
        self.BUOY_MAX_ASPECT_DEVIATION = saved.get('BUOY_MAX_ASPECT_DEVIATION', 0.5)
        # Adaptif deteksi jauh: conf & area minimum lebih longgar untuk box kecil
        self.BUOY_CONF_SMALL_THRESHOLD = saved.get('BUOY_CONF_SMALL_THRESHOLD', 0.20)
        self.BUOY_SMALL_AREA_PX = saved.get('BUOY_SMALL_AREA_PX', 80)
        # Tracking boost: buoy yang sudah terdeteksi tapi mengecil (menjauh)
        # atau membesar (mendekat) di-latch beberapa frame.
        self.BUOY_TRACK_MATCH_PX = saved.get('BUOY_TRACK_MATCH_PX', 30)
        self.BUOY_TRACK_BOOST = saved.get('BUOY_TRACK_BOOST', 1.5)

        # Debug deteksi: cetak alasan buoy ditolak (maks 1x per detik) ke
        # konsol — berguna untuk tuning live lewat scripts/test_deteksi.py.
        self.DETECTION_DEBUG = saved.get('DETECTION_DEBUG', False)

        # ---------- Filter & kontroler galat (app/filtering.py == core C) ----------
        # PID + deadband pengganti gain-P murni untuk koreksi yaw gate/box.
        self.PID_KP = saved.get('PID_KP', 1.0)
        self.PID_KI = saved.get('PID_KI', 0.0)
        self.PID_KD = saved.get('PID_KD', 0.0)
        self.PID_DEADBAND = saved.get('PID_DEADBAND', 0.01)        # radian
        self.PID_OUTPUT_LIMIT = saved.get('PID_OUTPUT_LIMIT', 0.6)  # radian
        self.PID_INTEGRAL_LIMIT = saved.get('PID_INTEGRAL_LIMIT', 0.3)
        # Complementary filter untuk memuluskan heading target (fusi
        # heading terukur + laju yaw gyro). Alpha 1 = percaya gyro, 0 = terukur.
        self.COMPLEMENTARY_ALPHA = saved.get('COMPLEMENTARY_ALPHA', 0.6)
        # EKF heading: process (gyro) & measurement (heading visi/GPS) noise.
        self.EKF_PROCESS_NOISE = saved.get('EKF_PROCESS_NOISE', 0.05)
        self.EKF_MEAS_NOISE = saved.get('EKF_MEAS_NOISE', 0.10)

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