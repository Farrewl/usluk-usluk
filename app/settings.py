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
# CAMERA_TARGET_FPS = 30: capture + loop video + render GUI dikunci 30 Hz
# (monotonic pacing). Negosiasi memilih mode resolusi TERTINGGI yang
# nyata >= 25 fps (prioritas 1280x720@30 MJPG, fallback 640x480@30);
# bila kamera hanya mampu lambat (mis. 10 fps), loop tetap pacing 30
# via frame terakhir (chip GUI jujur tulis real/display). Deteksi YOLO
# async tak diblokir.
CAMERA_TARGET_FPS = 30
CAMERA_MAX_AUTO_WIDTH = 1920
CAMERA_MAX_AUTO_HEIGHT = 1080
# Minimum fps nyata agar mode lolos negosiasi. 25 = toleransi jitter
# dari target 30 (kamera sehat 720p MJPG umumnya ~30 fps).
CAMERA_MIN_ACCEPT_FPS = 25
# Throttle GUI (agar start & render ringan di RPi/laptop):
# teks telemetri 5 Hz, peta 1 Hz, chip FPS 1 Hz.
GUI_TELEMETRY_HZ = 5
GUI_MAP_HZ = 1

# Orientasi frame kamera: 0 = normal, 1 = mirror kiri-kanan (flip
# horizontal), 2 = atas-bawah (flip vertikal), 3 = 180 derajat. Default 1
# (mirror) karena umpan kamera navigasi terpasang terbalik — objek di kanan
# kapal tampil di kiri GUI bila 0. Bisa diubah via GUI tuning /
# config/tuning_params.json (key CAMERA_FLIP_MODE) tanpa mengubah kode.
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
    "CAMERA_FLIP_MODE",
    # filter warna/bentuk buoy (lihat app/detection_validation.py)
    "BUOY_CONF_THRESHOLD", "BUOY_MIN_COLOR_FRACTION",
    "BUOY_MIN_SATURATION", "BUOY_MAX_ASPECT_DEVIATION",
    # adaptif deteksi jauh
    "BUOY_CONF_SMALL_THRESHOLD", "BUOY_SMALL_AREA_PX",
    "BUOY_TRACK_MATCH_PX", "BUOY_TRACK_BOOST",
    # P4-A: threshold adaptif gelap/terang (global frame brightness)
    "BUOY_ADAPTIVE_ENABLED", "BUOY_BRIGHTNESS_THRESHOLD",
    "BUOY_ADAPTIVE_MIN_SATURATION", "BUOY_ADAPTIVE_MIN_VALUE",
    "BUOY_ADAPTIVE_COLOR_FRACTION_MULT",
    # P4-D: ambang per-class (hijau vs merah)
    "BUOY_CONF_THRESHOLD_GREEN", "BUOY_CONF_SMALL_THRESHOLD_GREEN",
    "BUOY_CONF_THRESHOLD_RED", "BUOY_CONF_SMALL_THRESHOLD_RED",
    "BUOY_MIN_COLOR_FRACTION_GREEN", "BUOY_MIN_COLOR_FRACTION_RED",
    "BUOY_MIN_SATURATION_GREEN", "BUOY_MIN_SATURATION_RED",
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
    # kendali manual RC/gamepad (lihat core/src/manual_control.c)
    "MANUAL_ENABLED", "MANUAL_MAX_SURGE", "MANUAL_MAX_YAW",
    "MANUAL_DEADBAND", "MANUAL_EXPO", "MANUAL_RATE_LIMIT",
    "RC_TIMEOUT_MS", "RC_CH_THROTTLE", "RC_CH_YAW", "RC_CH_MODE",
    "RC_CH_DEADMAN", "MANUAL_LOST_HOLD_S",
    # manual dari QGC (app/qgc_offboard.py — Xbox via MAVLink)
    "QGC_MANUAL_TIMEOUT_S",
    # failsafe otomatis (lihat app/navigator.py _check_failsafe)
    "FAILSAFE_ENABLED", "FAILSAFE_TELEM_TIMEOUT_S",
    "FAILSAFE_LOW_BATT_PCT", "FAILSAFE_LOW_VOLT_V",
    "FAILSAFE_LOW_BATT_HOLD_S",
    # YOLO async (lihat app/yolo_async.py)
    "YOLO_FRAME_SKIP", "YOLO_INFERENCE_SIZE", "YOLO_RESULT_MAX_AGE_S",
    # P6-B: Pixhawk non-blocking + histeresis telemetri
    "MAV_CONNECT_TIMEOUT_S", "MAV_RETRY_INTERVAL_S",
    "TELEM_HYSTERESIS_FRAMES",
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
        # Ukuran ini hanya *default* untuk frame sintetis (kamera mati).
        # Saat kamera dibuka dengan auto_highest=True, FRAME_WIDTH/HEIGHT
        # ditimpa oleh hasil negosiasi mode (lihat app/camera.py).
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.CAMERA_TARGET_FPS = CAMERA_TARGET_FPS      # fps yang diusahakan
        self.CAMERA_MAX_AUTO_WIDTH = CAMERA_MAX_AUTO_WIDTH  # batas negosiasi resolusi
        self.CAMERA_MAX_AUTO_HEIGHT = CAMERA_MAX_AUTO_HEIGHT
        self.CAMERA_MIN_ACCEPT_FPS = CAMERA_MIN_ACCEPT_FPS  # mode di bawah ini ditolak
        # Flip dibaca dari tuning (disimpan GUI) dulu, fallback ke konstanta
        # modul — tanpa ini perubahan via GUI tidak pernah dipakai.
        self.CAMERA_FLIP_MODE = saved.get('CAMERA_FLIP_MODE', CAMERA_FLIP_MODE)
        self.ENABLE_SESSION_RECORDING = False
        self.SESSION_VIDEO_FPS = saved.get('SESSION_VIDEO_FPS', 10.0)

        # ---------- Hardware / telemetri ----------
        self.SERIAL_PORT = 'COM8'                  # Windows dev; di Pi pakai /dev/ttyACM0
        self.BAUD_RATE = 57600
        self.OFFBOARD_STREAM_RATE_HZ = 30
        # P6-B: Pixhawk non-blocking + histeresis (GUI tak lompat).
        # MAV_CONNECT_TIMEOUT_S: sekali coba connect (detik); loop tak freeze.
        # MAV_RETRY_INTERVAL_S: jeda antar percobaan (detik).
        # TELEM_HYSTERESIS_FRAMES: frame bagus/gagal beruntun sebelum
        # sumber telemetri pindah MOCK<->REAL.
        self.MAV_CONNECT_TIMEOUT_S = saved.get('MAV_CONNECT_TIMEOUT_S', 2.0)
        self.MAV_RETRY_INTERVAL_S = saved.get('MAV_RETRY_INTERVAL_S', 5.0)
        self.TELEM_HYSTERESIS_FRAMES = saved.get(
            'TELEM_HYSTERESIS_FRAMES', 5)
        # P6-C: kamera bawah air default index 2 (NAV=0/1 via negosiasi).
        # Dulu dua-duanya 1 (konflik: rebutan 1 device).
        self.WAYPOINT_PHOTO_CAMERA_INDEX = 2

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
        # P4-D: ambang per-class (hijau lebih longgar karena tenggelam duluan di gelap)
        self.BUOY_CONF_THRESHOLD_GREEN = saved.get('BUOY_CONF_THRESHOLD_GREEN', 0.30)
        self.BUOY_CONF_SMALL_THRESHOLD_GREEN = saved.get('BUOY_CONF_SMALL_THRESHOLD_GREEN', 0.15)
        self.BUOY_MIN_COLOR_FRACTION_GREEN = saved.get('BUOY_MIN_COLOR_FRACTION_GREEN', 0.04)
        self.BUOY_MIN_SATURATION_GREEN = saved.get('BUOY_MIN_SATURATION_GREEN', 0.30)
        self.BUOY_CONF_THRESHOLD_RED = saved.get('BUOY_CONF_THRESHOLD_RED', 0.35)
        self.BUOY_CONF_SMALL_THRESHOLD_RED = saved.get('BUOY_CONF_SMALL_THRESHOLD_RED', 0.20)
        self.BUOY_MIN_COLOR_FRACTION_RED = saved.get('BUOY_MIN_COLOR_FRACTION_RED', 0.05)
        self.BUOY_MIN_SATURATION_RED = saved.get('BUOY_MIN_SATURATION_RED', 0.35)
        # Adaptif deteksi jauh: conf & area minimum lebih longgar untuk box kecil
        self.BUOY_CONF_SMALL_THRESHOLD = saved.get('BUOY_CONF_SMALL_THRESHOLD', 0.20)
        self.BUOY_SMALL_AREA_PX = saved.get('BUOY_SMALL_AREA_PX', 80)
        # Tracking boost: buoy yang sudah terdeteksi tapi mengecil (menjauh)
        # atau membesar (mendekat) di-latch beberapa frame.
        self.BUOY_TRACK_MATCH_PX = saved.get('BUOY_TRACK_MATCH_PX', 30)
        self.BUOY_TRACK_BOOST = saved.get('BUOY_TRACK_BOOST', 1.5)
        # P4-A: threshold adaptif gelap/terang (global frame brightness)
        self.BUOY_ADAPTIVE_ENABLED = saved.get('BUOY_ADAPTIVE_ENABLED', True)
        # Mean V (0..255) di bawah ini = frame "gelap" → longgarkan ambang
        self.BUOY_BRIGHTNESS_THRESHOLD = saved.get('BUOY_BRIGHTNESS_THRESHOLD', 80)
        # Saat gelap: turunkan ke nilai ini
        self.BUOY_ADAPTIVE_MIN_SATURATION = saved.get('BUOY_ADAPTIVE_MIN_SATURATION', 0.20)
        self.BUOY_ADAPTIVE_MIN_VALUE = saved.get('BUOY_ADAPTIVE_MIN_VALUE', 20)
        # Pengali fraksi warna saat gelap (0.5 = setengah dari normal)
        self.BUOY_ADAPTIVE_COLOR_FRACTION_MULT = saved.get(
            'BUOY_ADAPTIVE_COLOR_FRACTION_MULT', 0.5)

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

        # ---------- Kendali manual RC/gamepad (lihat core/src/manual_control.c) ----------
        # MANUAL_MAX_* membatasi gas/belok manual (default 0.6/0.7 = aman).
        # RC_CH_* = nomor channel (1-based) di RC_CHANNELS Pixhawk.
        # Deadman: MANUAL hanya aktif selama CH_DEADMAN > 1500.
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

        # Failsafe manual QGC: paket MANUAL_CONTROL basi > ini = lepas
        # kendali -> netral (lihat app/qgc_offboard.py).
        self.QGC_MANUAL_TIMEOUT_S = saved.get('QGC_MANUAL_TIMEOUT_S', 0.5)

        # ---------- Failsafe otomatis (lihat _check_failsafe) ----------
        # Telemetri stale (tak ada ATTITUDE/GLOBAL_POSITION > batas) ATAU
        # baterai rendah (persen ATAU tegangan di bawah ambang, ditahan
        # selama FAILSAFE_LOW_BATT_HOLD_S agar spike sesaat tak memicu) ->
        # thrust 0 + coba set mode RTL via MAV_CMD_DO_SET_MODE (best-effort).
        self.FAILSAFE_ENABLED = saved.get('FAILSAFE_ENABLED', True)
        self.FAILSAFE_TELEM_TIMEOUT_S = saved.get(
            'FAILSAFE_TELEM_TIMEOUT_S', 2.0)
        self.FAILSAFE_LOW_BATT_PCT = saved.get(
            'FAILSAFE_LOW_BATT_PCT', 20.0)
        self.FAILSAFE_LOW_VOLT_V = saved.get(
            'FAILSAFE_LOW_VOLT_V', 13.2)
        self.FAILSAFE_LOW_BATT_HOLD_S = saved.get(
            'FAILSAFE_LOW_BATT_HOLD_S', 3.0)

        # ---------- YOLO & video ----------
        self.YOLO_FRAME_SKIP = saved.get('YOLO_FRAME_SKIP', 2)
        self.YOLO_INFERENCE_SIZE = saved.get('YOLO_INFERENCE_SIZE', 256)
        # Umur maks hasil async (detik): hasil lebih tua dianggap basi dan
        # loop memakai cache terakhir / kosong. 0.5 s ~= 1-2 frame @ inferensi
        # lambat; cukup segar untuk kontrol, cukup longgar untuk CPU RPi.
        self.YOLO_RESULT_MAX_AGE_S = saved.get('YOLO_RESULT_MAX_AGE_S', 0.5)
        self.YOLO_HALF_PRECISION = False           # CPU tidak mendukung half precision
        self.YOLO_DEVICE = 'cpu'                   # paksa jalan di CPU
        self.RED_BALL_CLASS_ID = 1
        self.GREEN_BALL_CLASS_ID = 0
        self.GREEN_BOX_CLASS_ID = 0                # model box hijau: 1 class