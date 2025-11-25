import os, json
cur_dir = os.path.dirname(os.path.abspath(__file__))
TUNING_FILE = cur_dir + "/config/tuning_params.json"
WEIGHTS_DIR = cur_dir + "/weights/"

class Config:
    def __init__(self):
        saved_params = {}
        if os.path.exists(TUNING_FILE):
            try:
                with open(TUNING_FILE, 'r') as f:
                    saved_params = json.load(f)
                print(f"Berhasil memuat parameter tuning dari {TUNING_FILE}")
            except Exception as e:
                print(f"PERINGATAN: Gagal membaca {TUNING_FILE}. Menggunakan nilai default. Error: {e}")
        
        # --- MODEL & PORT (Tidak dapat di-tuning) ---
        self.MODEL_PATH = WEIGHTS_DIR + 'buoy.pt'    
        self.CAMERA_INDEX = 1
        self.WAYPOINT_PHOTO_DIR = "waypoint_captures" # <-- PERBAIKAN 1
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.SERIAL_PORT = 'COM9' 
        self.BAUD_RATE = 57600
        self.OFFBOARD_STREAM_RATE_HZ = 30
        self.RED_BALL_CLASS_ID = 1
        self.GREEN_BALL_CLASS_ID = 0

        self.BOX_MODEL_PATH = WEIGHTS_DIR + 'box_hijau.pt' 
        self.GREEN_BOX_CLASS_ID = 0
        self.PHOTO_BOX_LEGS = [6]
        self.BLUE_BOX_MODEL_PATH = WEIGHTS_DIR + 'box_biru.pt'
        self.BLUE_BOX_CLASS_ID = 0
        self.BLUE_BOX_PHOTO_LEGS = [8]
        self.WAYPOINT_PHOTO_CAMERA_INDEX = 0
        self.RED_DOCK_MODEL_PATH = WEIGHTS_DIR + 'box_merah.pt'
        self.RED_BOX_CLASS_ID = 0
        self.RED_BOX_NAV_AFTER_WP = 8

        self.ENABLE_SESSION_RECORDING = False
        self.SESSION_VIDEO_DIR = "session_videos"
        self.NGROK_HOST = "dashboardaterolas.app"
        self.SERVER_UPLOAD_URL = f"https://{self.NGROK_HOST}/api/images/upload"
        self.REDIS_HOST = "localhost"
        self.REDIS_PORT = 6379
        self.TELEMETRY_CHANNEL = "asv_telemetry"
        self.VISION_CHANNEL = "asv_vision"
        self.MISSION_CHANNEL = "asv_mission"
        self.YOLO_HALF_PRECISION = False
        self.YOLO_DEVICE = 'cpu'
        
        # --- PARAMETER YANG DAPAT DI-TUNING ---
        # (Menggunakan saved_params.get() untuk memuat nilai atau menggunakan default)

        # Navigasi Umum
        self.ACCEPTANCE_RADIUS_M = saved_params.get('ACCEPTANCE_RADIUS_M', 2.0)
        self.THRUST_VALUE = saved_params.get('THRUST_VALUE', 0.9)
        self.TRANSITION_DURATION_S = saved_params.get('TRANSITION_DURATION_S', 0.5)
        self.GEOFENCE_WIDTH_METERS = saved_params.get('GEOFENCE_WIDTH_METERS', 1.0)

        # Vision Global
        self.FOCAL_LENGTH_PX = saved_params.get('FOCAL_LENGTH_PX', 400)
        self.VISION_P_GAIN = saved_params.get('VISION_P_GAIN', 1.9)
        self.VISION_SMOOTHING_ALPHA = saved_params.get('VISION_SMOOTHING_ALPHA', 0.6)
        self.ROI_TOP_CUTOFF_PERCENT = saved_params.get('ROI_TOP_CUTOFF_PERCENT', 0.20)

        # Misi Gate (Buoy)
        self.GATE_WIDTH_METERS = saved_params.get('GATE_WIDTH_METERS', 1.0)
        self.VISION_ENABLED_LEGS = [1,3, 5] # Tidak di-tuning, dianggap konstan
        self.MIN_BUOY_AREA_PX = saved_params.get('MIN_BUOY_AREA_PX', 80)
        self.GATE_AREA_SIMILARITY_RATIO = saved_params.get('GATE_AREA_SIMILARITY_RATIO', 0.5)

        # --- PERBAIKAN 2: TAMBAHKAN BLOK INI ---
        # Misi Foto Waypoint
        self.STOP_AND_PHOTO_AT_WP = saved_params.get('STOP_AND_PHOTO_AT_WP', []) # Default: list kosong
        self.WAYPOINT_PHOTO_STOP_DURATION_S = saved_params.get('WAYPOINT_PHOTO_STOP_DURATION_S', 3.0)
        # --- AKHIR PERBAIKAN 2 ---

        # Misi Box Hijau
        self.SEARCH_THRUST = saved_params.get('SEARCH_THRUST', 0.4)
        self.ALIGN_THRUST = saved_params.get('ALIGN_THRUST', 0.3)
        self.RETREAT_THRUST = saved_params.get('RETREAT_THRUST', -0.7)
        self.RETREAT_DURATION_S = saved_params.get('RETREAT_DURATION_S', 0.5)
        self.BOX_WIDTH_METERS = saved_params.get('BOX_WIDTH_METERS', 0.4)
        self.BOX_APPROACH_DISTANCE_M = saved_params.get('BOX_APPROACH_DISTANCE_M', 2.0)
        self.SAVE_GREEN_BOX_PHOTO = True # Tidak di-tuning
        self.YAW_SEARCH_BOX = saved_params.get('YAW_SEARCH_BOX', -90)
        self.BOX_SEARCH_LATERAL_THRUST = saved_params.get('BOX_SEARCH_LATERAL_THRUST', -0.2)
        
        # Misi Box Biru
        self.BLUE_BOX_WIDTH_METERS = saved_params.get('BLUE_BOX_WIDTH_METERS', 0.6)
        self.BLUE_BOX_APPROACH_DISTANCE_M = saved_params.get('BLUE_BOX_APPROACH_DISTANCE_M', 1.5)
        self.BLUE_BOX_LATERAL_OFFSET_M = saved_params.get('BLUE_BOX_LATERAL_OFFSET_M', -1.0)
        self.BLUE_BOX_ALIGN_THRUST = saved_params.get('BLUE_BOX_ALIGN_THRUST', 0.3)
        self.BLUE_BOX_SEARCH_THRUST = saved_params.get('BLUE_BOX_SEARCH_THRUST', 0.2)
        self.BLUE_BOX_YAW_SEARCH = saved_params.get('BLUE_BOX_YAW_SEARCH', -90)
        
        # Misi Docking (Box Merah)
        self.RED_BOX_WIDTH_METERS = saved_params.get('RED_BOX_WIDTH_METERS', 0.6)
        self.RED_BOX_DOCK_DISTANCE_M = saved_params.get('RED_BOX_DOCK_DISTANCE_M', 1.0)
        self.DOCK_ALIGN_THRUST = saved_params.get('DOCK_ALIGN_THRUST', 0.4)
        self.DOCK_HOLD_DURATION_S = saved_params.get('DOCK_HOLD_DURATION_S', 5.0)
        self.YAW_SEARCH_DOCK = saved_params.get('YAW_SEARCH_DOCK', 90)

        # YOLO & Video
        self.YOLO_FRAME_SKIP = saved_params.get('YOLO_FRAME_SKIP', 1)
        self.YOLO_INFERENCE_SIZE = saved_params.get('YOLO_INFERENCE_SIZE', 320)
        self.SESSION_VIDEO_FPS = saved_params.get('SESSION_VIDEO_FPS', 10.0)
    
    YOLO_HALF_PRECISION = False   # CPU tidak mendukung half precision
    YOLO_DEVICE = 'cpu'           # Paksa jalan di CPU
    ENABLE_SESSION_RECORDING = False # Set ke False untuk NONAKTIF merekam video sesi
    SESSION_VIDEO_DIR = "session_videos"    # Folder baru untuk video
    SESSION_VIDEO_FPS = 10.0                # Simpan video pada 10 FPS (lebih tinggi akan boros disk)
    NGROK_HOST = "dashboardaterolas.app" # Ambil dari web_gateway.txt
    SERVER_UPLOAD_URL = f"https://{NGROK_HOST}/api/images/upload"
    REDIS_HOST = "localhost"
    REDIS_PORT = 6379
    TELEMETRY_CHANNEL = "asv_telemetry"
    VISION_CHANNEL = "asv_vision"
    MISSION_CHANNEL = "asv_mission"
