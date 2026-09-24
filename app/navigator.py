from . import settings as cfg
from . import geo
from .camera import open_camera, flip_frame_if_needed
from .detection_validation import validate_buoy
from .filtering import (
    PidController, complementary_filter, HeadingEkf,
    normalize_wrap, cross_track_error,
)
from .gate_sequencer import GateSequencer, collect_gate_pairs
from ultralytics import YOLO
from pymavlink import mavutil
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import sys, time, math, cv2, csv, redis, base64, threading, requests, random, subprocess, re
import os, json

# Fuzzy Sugeno singleton (gain P dinamis) memakai app/fuzzy.py — implementasi
# murni Python. scikit-fuzzy >= 0.5 tidak bisa bikin singleton output
# (ValueError: "...must be equivalent in length to the universe variable"),
# jadi NAH itu yang membuat fuzzy diam-diam mati. app/fuzzy.py menggantikannya
# tanpa dependensi eksternal tambahan.
from .fuzzy import gate_p_gain, docking_p_gain
FUZZY_ENABLED = True
print("Fuzzy Logic (app/fuzzy.py) SIAP — gate_p_gain & docking_p_gain.")


WAYPOINTS = [] 

TUNING_FILE = cfg.TUNING_FILE


class VisionOffboardNavigator:

    # NOTE: Factory fuzzy skfuzzy (_create_gate_controller/_create_docking_controller)
    # DIHAPUS — scikit-fuzzy >= 0.5 gagal membuat singleton output. Desain
    # Sugeno yang sama (trapmf, AND=min, rata-rata tertimbang) sekarang ada di
    # app/fuzzy.py (gate_p_gain/docking_p_gain) dengan referensi C 1:1 di
    # core/src/fuzzy.c.


    def __init__(self, config):
        self.config = config
        self.waypoints = WAYPOINTS
        self.current_waypoint_index = 0
        
        self.gate_model = None
        self.box_model = None       
        self.red_dock_model = None  
        self.blue_box_model = None 
        
        self.cap = None
        self.master = None
        self.wp_photo_cap = None
        
        self.current_state = "WAYPOINT_NAV"
        self.task_timer = 0.0
        
        self.green_box_lost_timer = None
        self.blue_box_lost_timer = None
        self.green_box_confirm_timer = None
        self.blue_box_confirm_timer = None
        
        self.running = False
        self.last_attitude_msg = None 
        self.current_groundspeed = 0.0

        self.last_used_p_gain = 0.0 

        # --- Filter & kontroler galat (app/filtering.py == core C) ---
        # PID + deadband pengganti gain-P murni untuk koreksi yaw.
        self.pid_gate = PidController(
            kp=self.config.PID_KP, ki=self.config.PID_KI, kd=self.config.PID_KD,
            deadband=self.config.PID_DEADBAND,
            output_limit=self.config.PID_OUTPUT_LIMIT,
            integral_limit=self.config.PID_INTEGRAL_LIMIT,
        )
        self.ekf_heading = HeadingEkf(
            process_noise=self.config.EKF_PROCESS_NOISE,
            meas_noise=self.config.EKF_MEAS_NOISE,
        )
        # State filter komplementer untuk heading target.
        self.comp_angle_prev = 0.0
        self._last_loop_time = time.time()

        # --- Antrean target gate (titik tengah merah+hijau) ---
        self.gate_seq = GateSequencer(
            pass_distance_m=self.config.GATE_PASS_DISTANCE_M,
            lost_tolerance_frames=self.config.GATE_LOST_TOLERANCE_FRAMES,
            gate_width_m=self.config.GATE_WIDTH_METERS,
            focal_length_px=self.config.FOCAL_LENGTH_PX,
        )
        self.gate_active_mid = None  # midpoint gate aktif (untuk visualisasi)

        # Fuzzy Sugeno (app/fuzzy.py): tidak butuh inisialisasi objek controller,
        # cukup fungsi murni. Error hanya terjadi bila tabel tak konsisten.
        if FUZZY_ENABLED:
            print("Fuzzy Logic Controllers SIAP (app/fuzzy.py).")
        
        self.redis_client = None
        try:
            print(f"Menghubungkan ke Redis di {self.config.REDIS_HOST}:{self.config.REDIS_PORT}...")
            self.redis_client = redis.Redis(host=self.config.REDIS_HOST, port=self.config.REDIS_PORT, decode_responses=True)
            self.redis_client.ping()
            print("Berhasil terhubung ke Redis.")
        except Exception as e:
            print(f"PERINGATAN: Gagal terhubung ke Redis: {e}. Web dashboard tidak akan berfungsi.")
            self.redis_client = None
        
        try:
            print(f"Memuat model GATE: {self.config.MODEL_PATH}..."); 
            self.gate_model = YOLO(self.config.MODEL_PATH).to(self.config.YOLO_DEVICE)
        except Exception as e:
            raise FileNotFoundError(f"FATAL: Gagal memuat model GATE: {e}")
            
        try:
            print(f"Memuat model BOX HIJAU: {self.config.BOX_MODEL_PATH}..."); 
            self.box_model = YOLO(self.config.BOX_MODEL_PATH).to(self.config.YOLO_DEVICE)
        except Exception as e:
            print(f"PERINGATAN: Gagal memuat model BOX HIJAU: {e}. Misi foto tidak akan berfungsi.")
            self.box_model = None

        try:
            print(f"Memuat model BOX MERAH: {self.config.RED_DOCK_MODEL_PATH}..."); 
            self.red_dock_model = YOLO(self.config.RED_DOCK_MODEL_PATH).to(self.config.YOLO_DEVICE)
        except Exception as e:
            print(f"PERINGATAN: Gagal memuat model BOX MERAH: {e}. Misi docking tidak akan berfungsi.")
            self.red_dock_model = None
        
        try:
            print(f"Memuat model BOX BIRU: {self.config.BLUE_BOX_MODEL_PATH}..."); 
            self.blue_box_model = YOLO(self.config.BLUE_BOX_MODEL_PATH).to(self.config.YOLO_DEVICE)
        except Exception as e:
            print(f"PERINGATAN: Gagal memuat model BOX BIRU: {e}. Misi foto WP 8 tidak akan berfungsi.")
            self.blue_box_model = None

        print(f"Membuka kamera utama (Indeks {self.config.CAMERA_INDEX})...");
        self.cap = open_camera(self.config.CAMERA_INDEX,
                               target_fps=self.config.CAMERA_TARGET_FPS,
                               auto_highest=True)
        
        if self.cap is None: 
            raise IOError(f"FATAL: Tidak bisa membuka kamera utama di indeks {self.config.CAMERA_INDEX}.")

        # Ukuran hasil negosiasi menimpa default (frame sintetis & tampilan
        # video ikut konsisten dengan kamera asli).
        real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.config.FRAME_WIDTH = real_w
        self.config.FRAME_HEIGHT = real_h
        print(f"[CAM] Mode aktif kamera utama: {real_w}x{real_h}.")
        
        print("Kamera utama terbuka. Melakukan 'warm-up'...")
        start_warmup = time.time()
        warmup_success = False
        while time.time() - start_warmup < 3.0:
            ret, _ = self.cap.read()
            if ret:
                warmup_success = True
                break
            time.sleep(0.1)
        if not warmup_success: print("PERINGATAN: Kamera utama tidak mengirimkan frame.")
        
        print(f"Kamera foto WP (Indeks {self.config.WAYPOINT_PHOTO_CAMERA_INDEX}) akan dibuka nanti saat diperlukan.")
        self.wp_photo_cap = None 

        print(f"Menghubungkan ke PX4 di: {self.config.SERIAL_PORT}...");
        try:
            self.master = mavutil.mavlink_connection(self.config.SERIAL_PORT, baud=self.config.BAUD_RATE, autoreconnect=True)
            self.master.wait_heartbeat()
            print("Heartbeat diterima.")
        except Exception as e:
            if self.cap and self.cap.isOpened(): self.cap.release()
            if self.wp_photo_cap and self.wp_photo_cap.isOpened(): self.wp_photo_cap.release()
            raise ConnectionError(f"FATAL: Gagal terhubung: {e}")

        self.current_lat, self.current_lon, self.current_yaw_rad = None, None, None
        
        self.processing_width = 640
        self.processing_height = 360
        self.image_center_x = self.processing_width / 2.0 
        self.image_center_y = self.processing_height / 2.0 
        # Leg terakhir yang disapu GateSequencer (reset target saat ganti leg).
        self._gate_last_leg = -1 
        
        self.roi_top_y_cutoff = int(self.processing_height * self.config.ROI_TOP_CUTOFF_PERCENT)
        if self.roi_top_y_cutoff > 0:
            print(f"ROI diaktifkan: Mengabaikan {self.roi_top_y_cutoff} piksel teratas ({self.config.ROI_TOP_CUTOFF_PERCENT*100}% dari {self.processing_height})")
        
        self.frame_counter = 0
        self.last_detections = {}
        self.last_stream_time = 0
        self.leg_start_lat, self.leg_start_lon = None, None 
        self.last_vision_correction_rad = 0.0
        
        self.retreat_step = "IDLE"
        
        self.video_writer = None

        self.stream_display_mode = "raw" # Default: Raw Mode
        self.command_stop_event = threading.Event()
        self.command_thread = threading.Thread(target=self._redis_command_listener, daemon=True)
        self.command_thread.start()

        self.current_nuc_signal_ms = 0 # Menyimpan latensi dalam ms
        self.signal_stop_event = threading.Event()
        self.signal_thread = threading.Thread(target=self._signal_monitor_loop, daemon=True)
        
        self.redis_publish_data = None
        self.redis_frame_lock = threading.Lock()
        self.redis_stop_event = threading.Event()
        self.redis_publish_thread = threading.Thread(target=self._redis_publish_loop, daemon=True)

    def _redis_publish_loop(self):
        print("THREAD REDIS: Dimulai.")
        while not self.redis_stop_event.is_set():
            frame_to_publish = None
            counts_to_publish = None
            extra_info = None # Variabel baru
            
            with self.redis_frame_lock:
                if self.redis_publish_data is not None:
                    # Unpack 3 item sekarang
                    frame_to_publish, counts_to_publish, extra_info = self.redis_publish_data
                    frame_to_publish = frame_to_publish.copy() 
                    self.redis_publish_data = None 
            
            if frame_to_publish is not None and self.redis_client:
                try:
                    _, buffer = cv2.imencode('.jpg', frame_to_publish, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    jpg_as_base64 = base64.b64encode(buffer).decode('utf-8')
                    
                    payload = {
                        "type": "vision_update",
                        "frame_base64": jpg_as_base64,
                        "buoy_counts": counts_to_publish,
                        # Masukkan info tambahan ke payload JSON
                        "info": extra_info 
                    }
                    self.redis_client.publish(self.config.VISION_CHANNEL, json.dumps(payload))
                except Exception as e:
                    pass
            time.sleep(0.03)
        print("THREAD REDIS: Berhenti.")

    def _set_state_and_publish(self, new_state):
        if self.current_state == new_state: return
        self.current_state = new_state
        print(f"\nSTATE CHANGE: -> {new_state}")
        if not self.redis_client: return
        try:
            payload = {"type": "mission_update", "state_name": new_state}
            self.redis_client.publish(self.config.MISSION_CHANNEL, json.dumps(payload))
        except Exception as e:
            print(f"PERINGATAN: Gagal publish status '{new_state}' ke Redis: {e}")

    def _signal_monitor_loop(self):
        print("THREAD SINYAL: Dimulai.")
        while not self.signal_stop_event.is_set():
            try:
                param = '-n' if os.name == 'nt' else '-c'
                command = ['ping', param, '1', '8.8.8.8']
                result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=2)
                
                if result.returncode == 0:
                    match = re.search(r'time[=<](\d+)', result.stdout)
                    if match:
                        self.current_nuc_signal_ms = int(match.group(1))
                    else:
                        self.current_nuc_signal_ms = 10 
                else:
                    self.current_nuc_signal_ms = 999 
            except Exception as e:
                self.current_nuc_signal_ms = 0
            
            time.sleep(2)
        print("THREAD SINYAL: Berhenti.")

    def _redis_command_listener(self):
        """Mendengarkan perintah dari Web via Redis."""
        if not self.redis_client: return
        print("THREAD COMMAND: Mendengarkan channel 'asv_commands'...")
        pubsub = self.redis_client.pubsub()
        pubsub.subscribe("asv_commands")
        
        while not self.command_stop_event.is_set():
            message = pubsub.get_message()
            if message and message['type'] == 'message':
                try:
                    data = json.loads(message['data'])
                    if 'mode' in data:
                        new_mode = data['mode']
                        if new_mode in ["raw", "processed"]:
                            self.stream_display_mode = new_mode
                            print(f"\n[COMMAND] Mode Stream diubah ke: {new_mode.upper()}")
                except Exception:
                    pass
            time.sleep(0.1)

    def run(self, data_signal):
        print("Mempersiapkan mode Offboard..."); self._prepare_for_offboard()
        print("Menunggu data telemetri pertama (GPS 3D Fix & Attitude)...")

        if self.redis_client:
            print("Memulai thread publisher Redis...")
            self.redis_publish_thread.start()

        print("Memulai thread monitor sinyal (Ping)...")
        self.signal_thread.start()

        self.running = True 

        while self.running and (self.current_lat is None or self.current_yaw_rad is None):
            self._update_telemetry()
            self._stream_offboard_command(0.0, self.current_yaw_rad or 0, "INIT")
            time.sleep(0.1)
            frame_kosong = np.zeros((self.processing_height, self.processing_width, 3), dtype=np.uint8)
            cv2.putText(frame_kosong, "Waiting for GPS 3D Fix...", (30, self.processing_height // 2), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            data_packet = {"lat": 0.0, "lon": 0.0, "yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0, "state": "WAITING_GPS", "target_wp_idx": 0, "dist_to_wp_m": 0.0, "groundspeed": 0.0, "mavlink_ok": True, "gps_fix": False, "frame": frame_kosong}
            data_packet.update(self._battery_fields())
            data_signal.emit(data_packet)

        if self.running and self.waypoints:
            self.leg_start_lat = self.current_lat
            self.leg_start_lon = self.current_lon
            print(f"Posisi awal leg diatur ke: {self.leg_start_lat:.6f}, {self.leg_start_lon:.6f}")
        
        self._start_video_recording()

        try:
            while self.running:
                self._update_telemetry()
                self._publish_telemetry()

                ret, frame_high_res = self.cap.read()

                frame_raw = None

                
                if not ret:
                    print("Frame kamera gagal dibaca! Menggunakan frame hitam.")
                    frame = np.zeros((self.processing_height, self.processing_width, 3), dtype=np.uint8)
                    time.sleep(0.1)
                    frame_raw = frame.copy()
                else:
                    # Orientasi kamera (CAMERA_FLIP_MODE): default 0 = TIDAK
                    # dibalik — gambar persis output sensor (tidak reverse).
                    frame_high_res = flip_frame_if_needed(
                        frame_high_res, self.config.CAMERA_FLIP_MODE)
                    frame = cv2.resize(frame_high_res, (self.processing_width, self.processing_height), interpolation=cv2.INTER_LINEAR)
                    frame_raw = frame.copy()

                if self.roi_top_y_cutoff > 0:
                    cv2.rectangle(frame, (0, 0), (self.processing_width, self.roi_top_y_cutoff), (0, 0, 0), -1)

                if self.current_lat is None or self.current_yaw_rad is None:
                    print_status = "NO_TELEM (IDLE)"
                    self._stream_offboard_command(0.0, self.current_yaw_rad or 0, "NO_TELEM (IDLE)")
                    self._set_state_and_publish("NO_TELEM")
                    data_packet = {"lat": 0.0, "lon": 0.0, "yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0, "state": self.current_state, "target_wp_idx": self.current_waypoint_index, "dist_to_wp_m": 0.0, "frame": frame}
                    data_packet.update(self._battery_fields())
                    data_signal.emit(data_packet)
                    time.sleep(0.5)
                    continue

                if not self.waypoints or self.current_waypoint_index >= len(self.waypoints):
                    if not self.waypoints:
                        self._set_state_and_publish("NO_WAYPOINTS")
                        print_status = "NO_WAYPOINTS (IDLE)"
                    else:
                        self._set_state_and_publish("MISSION_COMPLETE")
                        print_status = "MISSION_COMPLETE (IDLE)"

                    self._stream_offboard_command(0.0, self.current_yaw_rad or 0, print_status)
                    current_pitch_deg = 0.0; current_roll_deg = 0.0
                    if self.last_attitude_msg:
                        current_pitch_deg = math.degrees(self.last_attitude_msg.pitch)
                        current_roll_deg = math.degrees(self.last_attitude_msg.roll)
                    data_packet = {"lat": self.current_lat, "lon": self.current_lon, "yaw_deg": math.degrees(self.current_yaw_rad or 0.0), "pitch_deg": current_pitch_deg, "roll_deg": current_roll_deg, "state": self.current_state, "target_wp_idx": max(0, len(self.waypoints) - 1), "dist_to_wp_m": 0.0, "frame": frame}
                    data_packet.update(self._battery_fields())
                    data_signal.emit(data_packet)
                    time.sleep(1.0 / self.config.OFFBOARD_STREAM_RATE_HZ)
                    continue

                target_wp = self.waypoints[self.current_waypoint_index]
                jarak_ke_wp, bearing_ke_wp = self._get_distance_and_bearing(self.current_lat, self.current_lon, target_wp['lat'], target_wp['lon'])
                thrust = self.config.THRUST_VALUE
                target_yaw_angle_rad = bearing_ke_wp
                lateral_thrust = 0.0
                print_status = "???"
                detections = {}
                best_gate = None; best_box = None; best_red_box = None
                gate_distance = float('inf'); box_distance = float('inf'); red_box_distance = float('inf')

                if self.current_state == "WAYPOINT_NAV":
                    print_status = "WAYPOINT_NAV"
                    if self.current_waypoint_index in self.config.PHOTO_BOX_LEGS:
                        if self.box_model is not None:
                            print("\n=== MEMULAI MISI FOTO BOX HIJAU ===")
                            self._set_state_and_publish("APPROACH_BOX_SEARCH")
                            self.last_vision_correction_rad = 0.0
                            self.green_box_confirm_timer = None 
                            continue
                        else:
                            print("PERINGATAN: Misi Foto Box Hijau leg terpicu, tapi model box hijau tidak ada!")
                            
                    if self.current_waypoint_index in self.config.BLUE_BOX_PHOTO_LEGS:
                        if self.blue_box_model is not None:
                            print(f"\n=== MEMULAI MISI FOTO SAMPING BOX BIRU (Leg {self.current_waypoint_index}) ===")
                            self._set_state_and_publish("APPROACH_BLUE_BOX_SEARCH")
                            self.last_vision_correction_rad = 0.0
                            self.blue_box_confirm_timer = None 
                            continue
                        else:
                            print("PERINGATAN: Misi Foto Box Biru leg terpicu, tapi model box biru tidak ada!")

                    if jarak_ke_wp < self.config.ACCEPTANCE_RADIUS_M:
                        print(f"\nWaypoint #{self.current_waypoint_index} tercapai.")
                        if self.current_waypoint_index in self.config.STOP_AND_PHOTO_AT_WP:
                            print(f"\n=== MEMULAI MISI FOTO WAYPOINT #{self.current_waypoint_index} ===")
                            self._set_state_and_publish("TAKE_WAYPOINT_PHOTO")
                            self.task_timer = time.time() 
                            continue 
                        
                        current_target_wp = self.waypoints[self.current_waypoint_index]
                        self.leg_start_lat = current_target_wp['lat']
                        self.leg_start_lon = current_target_wp['lon']
                        self.current_waypoint_index += 1
                        self.last_vision_correction_rad = 0.0
                        
                        if self.current_waypoint_index >= len(self.waypoints):
                            self._set_state_and_publish("MISSION_COMPLETE")
                            continue
                        else:
                            self._set_state_and_publish("WAYPOINT_TRANSITION")
                            self.task_timer = time.time()
                            continue 
                        
                    detections = self._detect_objects(frame, self.gate_model, force_run=False)
                    # Pemilihan gate TIDAK lagi per-frame murni (_find_best_gate):
                    # GateSequencer mengumpulkan semua pasangan, mengunci target
                    # aktif & melompat cepat ke gate berikutnya saat yang aktif
                    # dilewati -> kapal tidak "mikir kelamaan" di tengah gate.
                    best_gate, gate_distance = self._track_gate(detections)
                    target_yaw_angle_rad, print_status = self._get_gate_nav_yaw(bearing_ke_wp, best_gate, gate_distance)
                
                elif self.current_state == "WAYPOINT_TRANSITION":
                    print_status = "TRANSITION (GPS ONLY)"
                    thrust = self.config.THRUST_VALUE
                    target_yaw_angle_rad = bearing_ke_wp
                    elapsed = time.time() - self.task_timer
                    if elapsed > self.config.TRANSITION_DURATION_S:
                        print(f"Masa tenang transisi selesai ({elapsed:.1f}s). Kembali ke WAYPOINT_NAV.")
                        self._set_state_and_publish("WAYPOINT_NAV")
                    else:
                        print_status = f"TRANSITION (GPS {elapsed:.1f}s)"
                
                elif self.current_state == "TAKE_WAYPOINT_PHOTO":
                    print_status = f"PHOTO_WP_{self.current_waypoint_index}"
                    thrust = 0.0 
                    target_yaw_angle_rad = self.current_yaw_rad
                    elapsed = time.time() - self.task_timer
                    if elapsed < 0.5: print_status = f"PHOTO_WP (Stopping..)"
                    elif elapsed < 1.0: 
                        if not hasattr(self, 'wp_photo_taken'):
                            self._take_waypoint_photo()
                            self.wp_photo_taken = True
                        print_status = f"PHOTO_WP (Snap!)"
                    else:
                        if elapsed > self.config.WAYPOINT_PHOTO_STOP_DURATION_S:
                            print(f"Foto Selesai. Melanjutkan misi...")
                            if hasattr(self, 'wp_photo_taken'): del self.wp_photo_taken

                            if self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP:
                                if self.red_dock_model is not None:
                                    print("\n=== FOTO WP SELESAI, MEMULAI MISI DOCKING RED BOX ===")
                                    self._set_state_and_publish("APPROACH_RED_BOX_SEARCH")
                                    self.last_vision_correction_rad = 0.0
                                    continue 
                                else:
                                    print(f"PERINGATAN: Misi Docking Red Box setelah WP {self.current_waypoint_index} tidak bisa dimulai (Model tidak ada).")

                            current_target_wp = self.waypoints[self.current_waypoint_index]
                            self.leg_start_lat = current_target_wp['lat']
                            self.leg_start_lon = current_target_wp['lon']
                            self.current_waypoint_index += 1
                            self.last_vision_correction_rad = 0.0
                            if self.current_waypoint_index >= len(self.waypoints): self._set_state_and_publish("MISSION_COMPLETE")
                            else:
                                self._set_state_and_publish("WAYPOINT_TRANSITION")
                                self.task_timer = time.time()
                        else:
                            print_status = f"PHOTO_WP (Waiting {elapsed:.1f}s)"

                elif self.current_state == "APPROACH_BOX_SEARCH":
                    detections = self._detect_objects(frame, self.box_model, force_run=True)
                    best_box = self._find_best_box(detections, self.config.GREEN_BOX_CLASS_ID)
                    if best_box:
                        if self.green_box_confirm_timer is None:
                            self.green_box_confirm_timer = time.time()
                            print("BOX_SEARCH: Potensi deteksi... Verifikasi dimulai.")

                        elapsed_confirm = time.time() - self.green_box_confirm_timer
                        
                        if elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S:
                            print_status = "BOX_SEARCH (Confirmed!)"
                            print(f"Konfirmasi Berhasil ({elapsed_confirm:.2f}s). Pindah ke ALIGN.")
                            self._set_state_and_publish("APPROACH_BOX_ALIGN")
                            self.last_vision_correction_rad = 0.0
                            self.green_box_lost_timer = None 
                            self.green_box_confirm_timer = None 
                            continue
                        else:
                            print_status = f"BOX_SEARCH (Verifying {elapsed_confirm:.1f}s)"
                            thrust = 0.0
                            lateral_thrust = 0.0
                            target_yaw_angle_rad = self.current_yaw_rad 
                    else:
                        if self.green_box_confirm_timer is not None:
                            print("BOX_SEARCH: Deteksi Gagal/Hilang saat verifikasi. Reset.")
                            self.green_box_confirm_timer = None

                        print_status = "BOX_SEARCH (Rotating)"
                        target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + math.radians(self.config.YAW_SEARCH_BOX))
                        thrust = self.config.SEARCH_THRUST 
                        lateral_thrust = 0.0
                        self.last_vision_correction_rad = 0.0 
                        self.green_box_lost_timer = None 
                
                elif self.current_state == "APPROACH_BOX_ALIGN":
                    detections = self._detect_objects(frame, self.box_model, force_run=True)
                    best_box = self._find_best_box(detections, self.config.GREEN_BOX_CLASS_ID)
                    if best_box:
                        self.green_box_lost_timer = None 
                        box_distance = self._get_distance_to_box(best_box, self.config.BOX_WIDTH_METERS, self.config.FOCAL_LENGTH_PX)
                        if box_distance > self.config.BOX_APPROACH_DISTANCE_M:
                            raw_correction_rad = self._calculate_yaw_correction_box(best_box, box_distance)
                            alpha = self.config.VISION_SMOOTHING_ALPHA
                            smooth_correction_rad = (alpha * raw_correction_rad) + (1.0 - alpha) * self.last_vision_correction_rad
                            self.last_vision_correction_rad = smooth_correction_rad
                            target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + smooth_correction_rad)
                            print_status = f"BOX_ALIGN (Dist: {box_distance:.1f}m)"
                            thrust = self.config.ALIGN_THRUST
                        else:
                            print_status = "BOX_ALIGN (Reached!)"
                            self._set_state_and_publish("TAKE_PHOTO")
                            thrust = 0.0
                            self.last_vision_correction_rad = 0.0 
                    else:
                        if self.green_box_lost_timer is None:
                            print("BOX_ALIGN (Lost! Starting patience timer...)")
                            self.green_box_lost_timer = time.time()
                        elapsed_lost = time.time() - self.green_box_lost_timer
                        if elapsed_lost < 2.0: 
                            print_status = f"BOX_ALIGN (Lost {elapsed_lost:.1f}s)"
                            target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + self.last_vision_correction_rad)
                            thrust = self.config.ALIGN_THRUST
                        else:
                            print_status = "BOX_ALIGN (Lost > 2s! Assuming position...)"
                            self._set_state_and_publish("TAKE_PHOTO") 
                            self.green_box_lost_timer = None
                            self.last_vision_correction_rad = 0.0
                            thrust = 0.0

                elif self.current_state == "TAKE_PHOTO":
                    print_status = "TAKE_PHOTO"
                    self._stream_offboard_command(0.0, self.current_yaw_rad, print_status, force_send=True)
                    time.sleep(0.2)
                    self._take_photo(frame)
                    self.task_timer = time.time()
                    self._set_state_and_publish("RETREAT")
                    self.retreat_step = "START_BRAKE"
                    thrust = 0.0

                elif self.current_state == "RETREAT":
                    elapsed_from_start = time.time() - self.task_timer
                    if self.retreat_step == "START_BRAKE":
                        print_status = "RETREAT (Brake)"
                        thrust = self.config.RETREAT_THRUST
                        target_yaw_angle_rad = self.current_yaw_rad
                        if elapsed_from_start > 0.2:
                            self.retreat_step = "GOTO_NEUTRAL"
                    elif self.retreat_step == "GOTO_NEUTRAL":
                        print_status = "RETREAT (Neutral)"
                        thrust = 0.0
                        target_yaw_angle_rad = self.current_yaw_rad
                        if elapsed_from_start > 0.4:
                            self.retreat_step = "START_REVERSE"
                            self.task_timer = time.time()
                    elif self.retreat_step == "START_REVERSE":
                        elapsed_actual_retreat = time.time() - self.task_timer
                        if elapsed_actual_retreat < self.config.RETREAT_DURATION_S:
                            print_status = f"RETREAT (Actual: {elapsed_actual_retreat:.1f}s)"
                            thrust = self.config.RETREAT_THRUST
                            target_yaw_angle_rad = self.current_yaw_rad
                        else:
                            print_status = "RETREAT (Done)"
                            self._set_state_and_publish("WAYPOINT_NAV")
                            self.retreat_step = "IDLE"
                            print(f"\n=== MISI FOTO BOX SELESAI ===")
                            self.current_waypoint_index += 1
                            if self.current_waypoint_index < len(self.waypoints):
                                self.leg_start_lat = self.current_lat
                                self.leg_start_lon = self.current_lon
                            self.last_vision_correction_rad = 0.0
                    else:
                        print_status = "RETREAT (ERR_STATE)"
                        thrust = 0.0
                        self._set_state_and_publish("WAYPOINT_NAV")
                        self.retreat_step = "IDLE"

                elif self.current_state == "APPROACH_BLUE_BOX_SEARCH":
                    detections = self._detect_objects(frame, self.blue_box_model, force_run=True)
                    best_box = self._find_best_box(detections, self.config.BLUE_BOX_CLASS_ID, True)
                    
                    if best_box:
                        if self.blue_box_confirm_timer is None:
                            self.blue_box_confirm_timer = time.time()
                            print("BLUE_BOX_SEARCH: Potensi deteksi... Verifikasi dimulai.")
                        
                        elapsed_confirm = time.time() - self.blue_box_confirm_timer
                        
                        if elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S:
                            print_status = "BLUE_BOX_SEARCH (Confirmed!)"
                            print(f"Konfirmasi Box Biru Berhasil ({elapsed_confirm:.2f}s). Pindah ke ALIGN.")
                            self._set_state_and_publish("APPROACH_BLUE_BOX_ALIGN")
                            self.last_vision_correction_rad = 0.0 
                            self.blue_box_lost_timer = None 
                            self.blue_box_confirm_timer = None 
                            continue
                        else:
                            print_status = f"BLUE_BOX_SEARCH (Verifying {elapsed_confirm:.1f}s)"
                            thrust = 0.0
                            lateral_thrust = 0.0
                            target_yaw_angle_rad = self.current_yaw_rad
                    else:
                        if self.blue_box_confirm_timer is not None:
                            print("BLUE_BOX_SEARCH: Deteksi Gagal/Hilang saat verifikasi. Reset.")
                            self.blue_box_confirm_timer = None

                        print_status = "BLUE_BOX_SEARCH (Rotating)"
                        target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + math.radians(self.config.BLUE_BOX_YAW_SEARCH))
                        thrust = self.config.BLUE_BOX_SEARCH_THRUST
                        lateral_thrust = 0.0
                        self.last_vision_correction_rad = 0.0 
                        self.blue_box_lost_timer = None 

                elif self.current_state == "APPROACH_BLUE_BOX_ALIGN":
                    detections = self._detect_objects(frame, self.blue_box_model, force_run=True)
                    best_box = self._find_best_box(detections, self.config.BLUE_BOX_CLASS_ID, True)
                    if best_box:
                        self.blue_box_lost_timer = None 
                        box_distance = self._get_distance_to_box(best_box, self.config.BLUE_BOX_WIDTH_METERS, self.config.FOCAL_LENGTH_PX)
                        if box_distance > self.config.BLUE_BOX_APPROACH_DISTANCE_M: 
                            raw_correction_rad = self._calculate_yaw_correction_blue_box(best_box, box_distance)
                            alpha = self.config.VISION_SMOOTHING_ALPHA
                            smooth_correction_rad = (alpha * raw_correction_rad) + (1.0 - alpha) * self.last_vision_correction_rad
                            self.last_vision_correction_rad = smooth_correction_rad
                            target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + smooth_correction_rad)
                            print_status = f"BLUE_BOX_ALIGN (Dist: {box_distance:.1f}m)"
                            thrust = self.config.BLUE_BOX_ALIGN_THRUST
                        else:
                            print_status = "BLUE_BOX_ALIGN (Reached!)"
                            self._set_state_and_publish("TAKE_BLUE_BOX_PHOTO")
                            self.task_timer = time.time() 
                            thrust = 0.0
                            self.last_vision_correction_rad = 0.0 
                    else:
                        if self.blue_box_lost_timer is None:
                            print("BLUE_BOX_ALIGN (Lost! Starting patience timer...)")
                            self.blue_box_lost_timer = time.time()
                        elapsed_lost = time.time() - self.blue_box_lost_timer
                        if elapsed_lost < 2.0: 
                            print_status = f"BLUE_BOX_ALIGN (Lost {elapsed_lost:.1f}s)"
                            target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + self.last_vision_correction_rad)
                            thrust = self.config.BLUE_BOX_ALIGN_THRUST
                        else:
                            print_status = "BLUE_BOX_ALIGN (Lost > 2s! Assuming position...)"
                            self._set_state_and_publish("TAKE_BLUE_BOX_PHOTO") 
                            self.task_timer = time.time() 
                            self.blue_box_lost_timer = None
                            self.last_vision_correction_rad = 0.0
                            thrust = 0.0

                # --- [MODIFIKASI FINAL: WP 8 SMART CAPTURE + FORCE UPLOAD] ---
                elif self.current_state == "TAKE_BLUE_BOX_PHOTO":
                    print_status = f"PHOTO_BLUE_BOX"
                    thrust = 0.0 
                    target_yaw_angle_rad = self.current_yaw_rad
                    elapsed = time.time() - self.task_timer
                    
                    # Fase 1: Stabilisasi (2 Detik)
                    if elapsed < 2.0: 
                        print_status = f"PHOTO_BLUE (Stabilizing..)"
                    
                    # Fase 2: Eksekusi Smart Capture
                    elif elapsed < 4.0: 
                        if not hasattr(self, 'blue_box_photo_taken'):
                            print("\n=== [WP 8] MEMULAI PROSEDUR SMART PHOTO ===")

                            # A. Matikan Kamera Navigasi
                            if self.cap.isOpened():
                                self.cap.release()
                                print("[WP 8] Kamera Navigasi dipause.")
                            time.sleep(1.0) 

                            # B. Buka Kamera Bawah (Index 1) dengan Settingan Terang
                            cam_bawah = open_camera(self.config.WAYPOINT_PHOTO_CAMERA_INDEX, 640, 480)
                            
                            if cam_bawah and cam_bawah.isOpened():
                                # Settingan "Obat Kuat"
                                cam_bawah.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                                cam_bawah.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                                cam_bawah.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Force Auto ON (Value 1)

                                print("[WP 8] Warming up sensor (20 frames)...")
                                for _ in range(20): cam_bawah.read()

                                foto_sukses = False
                                last_frame_bawah = None # Simpan frame terakhir

                                # C. Loop Percobaan
                                for percobaan in range(1, 6):
                                    ret_b, frame_b = cam_bawah.read()
                                    if ret_b:
                                        last_frame_bawah = frame_b.copy() # Backup frame
                                        avg_bright = np.mean(frame_b)
                                        print(f"[WP 8] Try {percobaan}: Brightness = {avg_bright:.2f}")
                                        
                                        # Validasi Brightness > 1.0 (Bisa dinaikkan jika perlu)
                                        if avg_bright > 1.0:
                                            timestamp = int(time.time())
                                            fname_short = f"WP8_BlueBox_OK_{timestamp}.jpg"
                                            fname_full = os.path.join(self.config.WAYPOINT_PHOTO_DIR, fname_short)
                                            
                                            if not os.path.exists(self.config.WAYPOINT_PHOTO_DIR): 
                                                os.makedirs(self.config.WAYPOINT_PHOTO_DIR)
                                                
                                            cv2.imwrite(fname_full, frame_b)
                                            print(f"[WP 8] FOTO SUKSES: {fname_full}")
                                            
                                            threading.Thread(target=self._upload_snapshot_to_server, args=(fname_full, fname_short), daemon=True).start()
                                            foto_sukses = True
                                            break
                                        else:
                                            print("[WP 8] Foto Gelap. Retry...")
                                            time.sleep(0.2)
                                
                                # --- FALLBACK: UPLOAD MESKIPUN GELAP ---
                                if not foto_sukses and last_frame_bawah is not None:
                                    print("[PERINGATAN] Gagal mendapatkan foto terang setelah 5x percobaan.")
                                    print("[ACTION] Mengupload foto terakhir (meskipun gelap) sebagai bukti data.")
                                    
                                    timestamp = int(time.time())
                                    fname_short = f"WP8_BlueBox_DARK_{timestamp}.jpg"
                                    fname_full = os.path.join(self.config.WAYPOINT_PHOTO_DIR, fname_short)
                                    
                                    if not os.path.exists(self.config.WAYPOINT_PHOTO_DIR): 
                                        os.makedirs(self.config.WAYPOINT_PHOTO_DIR)
                                    
                                    cv2.imwrite(fname_full, last_frame_bawah)
                                    # Tetap upload ke web
                                    threading.Thread(target=self._upload_snapshot_to_server, args=(fname_full, fname_short), daemon=True).start()

                                cam_bawah.release()
                            else:
                                print("[WP 8] Gagal buka kamera bawah air!")
                            
                            self.blue_box_photo_taken = True
                            
                            # D. Nyalakan Lagi Kamera Navigasi
                            print("[WP 8] Restarting Nav Camera...")
                            self.cap = open_camera(self.config.CAMERA_INDEX,
                                                   target_fps=self.config.CAMERA_TARGET_FPS,
                                                   auto_highest=True)
                            for _ in range(5): self.cap.read() 

                        print_status = f"PHOTO_BLUE (Snap!)"
                    
                    # Fase 3: Selesai
                    else:
                        if elapsed > (self.config.WAYPOINT_PHOTO_STOP_DURATION_S + 2.0):
                            if hasattr(self, 'blue_box_photo_taken'): del self.blue_box_photo_taken
                            self._set_state_and_publish("BLUE_BOX_RETREAT")
                            self.task_timer = time.time()
                            self.retreat_step = "START_BRAKE"
                        else:
                            print_status = f"PHOTO_BLUE (Holding {elapsed:.1f}s)"

                elif self.current_state == "BLUE_BOX_RETREAT":
                    elapsed_from_start = time.time() - self.task_timer
                    if self.retreat_step == "START_BRAKE":
                        print_status = "BLUE_RETREAT (Brake)"
                        thrust = self.config.RETREAT_THRUST
                        target_yaw_angle_rad = self.current_yaw_rad
                        if elapsed_from_start > 0.2: self.retreat_step = "GOTO_NEUTRAL"
                    elif self.retreat_step == "GOTO_NEUTRAL":
                        print_status = "BLUE_RETREAT (Neutral)"
                        thrust = 0.0
                        target_yaw_angle_rad = self.current_yaw_rad
                        if elapsed_from_start > 0.4:
                            self.retreat_step = "START_REVERSE"
                            self.task_timer = time.time()
                    elif self.retreat_step == "START_REVERSE":
                        elapsed_actual_retreat = time.time() - self.task_timer
                        if elapsed_actual_retreat < self.config.RETREAT_DURATION_S:
                            print_status = f"BLUE_RETREAT (Actual: {elapsed_actual_retreat:.1f}s)"
                            thrust = self.config.RETREAT_THRUST
                            target_yaw_angle_rad = self.current_yaw_rad
                        else:
                            print_status = "BLUE_RETREAT (Done)"
                            self.retreat_step = "IDLE"
                            print(f"\n=== MISI FOTO BOX BIRU SELESAI ===")
                            if self.red_dock_model is not None:
                                print("\n=== MEMULAI MISI DOCKING RED BOX (Setelah Box Biru) ===")
                                self._set_state_and_publish("APPROACH_RED_BOX_SEARCH")
                                self.last_vision_correction_rad = 0.0
                            else:
                                print("PERINGATAN: Model docking tidak ada, melanjutkan ke WP Nav...")
                                self._set_state_and_publish("WAYPOINT_NAV")
                                self.current_waypoint_index += 1
                                if self.current_waypoint_index < len(self.waypoints):
                                    self.leg_start_lat = self.current_lat
                                    self.leg_start_lon = self.current_lon
                                self.last_vision_correction_rad = 0.0
                    else:
                        print_status = "BLUE_RETREAT (ERR_STATE)"
                        thrust = 0.0
                        self._set_state_and_publish("WAYPOINT_NAV")
                        self.retreat_step = "IDLE"

                elif self.current_state == "APPROACH_RED_BOX_SEARCH":
                    detections = self._detect_objects(frame, self.red_dock_model, force_run=True)
                    best_red_box = self._find_best_box(detections, self.config.RED_BOX_CLASS_ID, True)
                    if best_red_box:
                        print_status = "RED_DOCK_SEARCH (Found!)"
                        self._set_state_and_publish("APPROACH_RED_BOX_ALIGN")
                        self.last_vision_correction_rad = 0.0
                        continue
                    else:
                        print_status = "RED_DOCK_SEARCH (Rotating)"
                        target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + math.radians(self.config.YAW_SEARCH_DOCK))
                        thrust = self.config.SEARCH_THRUST
                        lateral_thrust = 0.0

                elif self.current_state == "APPROACH_RED_BOX_ALIGN":
                    detections = self._detect_objects(frame, self.red_dock_model, force_run=True)
                    best_red_box = self._find_best_box(detections, self.config.RED_BOX_CLASS_ID, True)
                    if not best_red_box:
                        print_status = "RED_DOCK_ALIGN (Lost Target!)"
                        self._set_state_and_publish("APPROACH_RED_BOX_SEARCH")
                        continue
                    red_box_distance = self._get_distance_to_box(best_red_box, self.config.RED_BOX_WIDTH_METERS, self.config.FOCAL_LENGTH_PX)
                    if red_box_distance > self.config.RED_BOX_DOCK_DISTANCE_M:
                        raw_correction_rad = self._calculate_yaw_correction_red_box(best_red_box, red_box_distance)
                        alpha = 0.5 
                        smooth_correction_rad = (alpha * raw_correction_rad) + (1.0 - alpha) * self.last_vision_correction_rad
                        self.last_vision_correction_rad = smooth_correction_rad
                        target_yaw_angle_rad = self._normalize_angle(self.current_yaw_rad + smooth_correction_rad)
                        thrust = self.config.DOCK_ALIGN_THRUST 
                        print_status = f"RED_DOCK_ALIGN (Dist: {red_box_distance:.2f}m)"
                    else:
                        print_status = "RED_DOCK_ALIGN (Docked!)"
                        self._set_state_and_publish("RED_BOX_DOCKED")
                        thrust = 0.0
                        self.task_timer = time.time()

                elif self.current_state == "RED_BOX_DOCKED":
                    print_status = f"RED_BOX_DOCKED ({time.time() - self.task_timer:.1f}s)"
                    thrust = 0.0
                    target_yaw_angle_rad = self.current_yaw_rad
                    if time.time() - self.task_timer > self.config.DOCK_HOLD_DURATION_S: 
                        print("\n=== MISI DOCKING RED BOX SELESAI ===")
                        current_target_wp = self.waypoints[self.current_waypoint_index]
                        self.leg_start_lat = current_target_wp['lat']
                        self.leg_start_lon = current_target_wp['lon']
                        self.current_waypoint_index += 1 
                        self.last_vision_correction_rad = 0.0
                        if self.current_waypoint_index >= len(self.waypoints): self._set_state_and_publish("MISSION_COMPLETE")
                        else:
                            self._set_state_and_publish("WAYPOINT_TRANSITION")
                            self.task_timer = time.time()
                        continue
                
                if self.current_waypoint_index >= len(self.waypoints):
                    thrust = 0.0
                    print_status = "MISSION_COMPLETE"

                self._stream_offboard_command(thrust, target_yaw_angle_rad, print_status, lateral_thrust=lateral_thrust)
                
                buoy_counts = self._visualize(
                    frame, detections, best_gate, best_box, best_red_box, 
                    jarak_ke_wp, box_distance, red_box_distance, print_status
                )

                if self.redis_client:
                    with self.redis_frame_lock:
                        if self.redis_publish_data is None:
                            # Pastikan frame_raw tersedia, jika tidak gunakan frame sebagai fallback
                            if self.stream_display_mode == "processed":
                                img_to_send = frame
                            else:
                                img_to_send = frame_raw if 'frame_raw' in locals() and frame_raw is not None else frame
                            
                            # --- KUMPULKAN DATA TEXT DI SINI ---
                            # Siapkan string untuk P-Gain
                            gain_str = f"{self.last_used_p_gain:.2f}"
                            if FUZZY_ENABLED and self.last_used_p_gain != self.config.VISION_P_GAIN and self.last_used_p_gain > 0.0:
                                gain_str += " (Fuzzy)"
                            
                            # Bungkus data penting
                            extra_data = {
                                "state": self.current_state,
                                "wp_idx": self.current_waypoint_index,
                                "wp_dist": f"{jarak_ke_wp:.1f}", # Jarak ke WP
                                "nav_status": print_status,      # Status navigasi (VISION/TRANSIT/dll)
                                "p_gain": gain_str,
                                "box_dist": f"{box_distance:.2f}" if box_distance != float('inf') else "-",
                                "dock_dist": f"{red_box_distance:.2f}" if red_box_distance != float('inf') else "-"
                            }
                            
                            # Masukkan ke antrian (Tuple 3 item)
                            self.redis_publish_data = (img_to_send, buoy_counts, extra_data)
                
                current_pitch_deg = 0.0; current_roll_deg = 0.0
                if self.last_attitude_msg:
                    current_pitch_deg = math.degrees(self.last_attitude_msg.pitch)
                    current_roll_deg = math.degrees(self.last_attitude_msg.roll)

                data_packet = {
                    "lat": self.current_lat, "lon": self.current_lon,
                    "yaw_deg": math.degrees(self.current_yaw_rad),
                    "pitch_deg": current_pitch_deg, "roll_deg": current_roll_deg,
                    "state": self.current_state,
                    "target_wp_idx": self.current_waypoint_index,
                    "dist_to_wp_m": jarak_ke_wp,
                    "groundspeed": self.current_groundspeed,
                    "mavlink_ok": bool(self.master is not None
                                       and self.master.port is not None),
                    "gps_fix": bool(self.current_lat is not None
                                    and self.current_lon is not None),
                    "frame": frame
                }
                data_packet.update(self._battery_fields())
                data_signal.emit(data_packet)

        finally:
            self._cleanup()

    def stop(self):
        print("Navigator backend menerima sinyal stop...")
        self.running = False

    def _get_gate_nav_yaw(self, bearing_ke_wp, best_gate, gate_distance):
        start_lat, start_lon = self.leg_start_lat, self.leg_start_lon

        if self.current_waypoint_index > 0:
            if self.current_waypoint_index < len(self.waypoints):
                prev_wp_index = self.current_waypoint_index - 1
                prev_wp = self.waypoints[prev_wp_index]
                start_lat, start_lon = prev_wp['lat'], prev_wp['lon']
            else:
                start_lat, start_lon = self.current_lat, self.current_lon

        if self.current_waypoint_index >= len(self.waypoints):
            return self.current_yaw_rad or 0.0, "END_OF_MISSION"

        target_wp = self.waypoints[self.current_waypoint_index]
        
        jarak_ke_wp_saat_ini, _ = self._get_distance_and_bearing(self.current_lat, self.current_lon, target_wp['lat'], target_wp['lon'])

        is_pre_turning = False
        PRE_TURN_RADIUS_M = 1 

        next_wp_index = self.current_waypoint_index + 1
        if next_wp_index < len(self.waypoints):
            if jarak_ke_wp_saat_ini < PRE_TURN_RADIUS_M:
                next_wp = self.waypoints[next_wp_index]
                _, bearing_ke_wp_berikutnya = self._get_distance_and_bearing(self.current_lat, self.current_lon, next_wp['lat'], next_wp['lon'])
                
                bearing_ke_wp = bearing_ke_wp_berikutnya
                is_pre_turning = True
                print(f"PRE-TURN: Navigating to WP #{next_wp_index} bearing ({math.degrees(bearing_ke_wp):.1f} deg) at {jarak_ke_wp_saat_ini:.1f}m from current WP (Radius: {PRE_TURN_RADIUS_M:.1f}m).")
        
        cross_track_dist = self._get_cross_track_distance(self.current_lat, self.current_lon, start_lat, start_lon, target_wp['lat'], target_wp['lon'])

        target_yaw_angle_rad = bearing_ke_wp
        print_status = "WAYPOINT (Default)"
        
        is_vision_leg = self.current_waypoint_index in self.config.VISION_ENABLED_LEGS
        is_on_track = abs(cross_track_dist) < self.config.GEOFENCE_WIDTH_METERS

        if is_vision_leg and not is_pre_turning:
            if is_on_track:
                if best_gate:
                    raw_correction_rad = self._calculate_yaw_correction_gate(best_gate, gate_distance)
                    dt = self._control_dt()

                    # EKF heading (app/filtering.py == core C): prediksi dengan
                    # laju yaw (gyro pendek) lalu koreksi heading terukur —
                    # heading estimasi lebih halus dari pengukuran mentah.
                    gyro_rate = 0.0
                    if getattr(self, '_last_yaw_meas', None) is not None:
                        gyro_rate = self._normalize_angle(
                            self.current_yaw_rad - self._last_yaw_meas) / dt
                    self._last_yaw_meas = self.current_yaw_rad
                    self.ekf_heading.predict(gyro_rate, dt)
                    heading_est = self.ekf_heading.update(self.current_yaw_rad or 0.0)

                    # Complementary filter: extrapolasi heading dari gyro lalu
                    # fusi dengan (heading estimasi + koreksi visi) — halus &
                    # tak hanyut.
                    raw_target = self._normalize_angle(heading_est + raw_correction_rad)
                    target_yaw_angle_rad = self._normalize_angle(complementary_filter(
                        self.config.COMPLEMENTARY_ALPHA, self.comp_angle_prev,
                        gyro_rate, dt, raw_target))
                    self.comp_angle_prev = target_yaw_angle_rad
                    print_status = f"VISION (Dist: {gate_distance:.1f}m)"
                else:
                    print_status = "WAYPOINT (On Track, No Gate)"
                    self.last_vision_correction_rad = 0.0
                    self.last_used_p_gain = 0.0
            else:
                print_status = f"GEOFENCE_CORR (Off: {cross_track_dist:.1f}m)"
                self.last_vision_correction_rad = 0.0
                self.last_used_p_gain = 0.0
        else:
            if is_pre_turning: print_status = f"PRE-TURN to WP #{next_wp_index}"
            else:
                print_status = "TRANSIT (Vision OFF)"

            self.last_vision_correction_rad = 0.0
            self.last_used_p_gain = 0.0
            target_yaw_angle_rad = bearing_ke_wp

        return target_yaw_angle_rad, print_status

    def _update_telemetry(self):
        msg = self.master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS'], blocking=False)
        while msg:
            match msg.get_type():
                case 'GLOBAL_POSITION_INT':
                    self.current_lat, self.current_lon = msg.lat / 1e7, msg.lon / 1e7
                case 'ATTITUDE':
                    self.current_yaw_rad = msg.yaw
                    self.last_attitude_msg = msg
                case 'VFR_HUD':
                    self.current_groundspeed = msg.groundspeed
                case 'SYS_STATUS':
                    # voltage_battery satuannya millivolt; 0 / 0xFFFF artinya
                    # Pixhawk tidak punya sensor tegangan -> pertahankan nilai
                    # terakhir, jangan diacak (data palsu menyesatkan operator).
                    vbatt = getattr(msg, "voltage_battery", 0) or 0
                    if vbatt not in (0, 0xFFFF, 65535):
                        self.current_voltage = vbatt / 1000.0
                    curr = getattr(msg, "current_battery", -1)
                    if curr is not None and curr >= 0:
                        self.current_current = curr / 100.0
                    rem = getattr(msg, "battery_remaining", -1)
                    if rem is not None and 0 <= rem <= 100:
                        self.current_battery_pct = float(rem)
                    else:
                        self.current_battery_pct = None
            msg = self.master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS'], blocking=False)

    def _detect_objects(self, frame, model_to_use, force_run=False):
        self.frame_counter += 1
        if not force_run and (self.frame_counter % (self.config.YOLO_FRAME_SKIP + 1) != 0): return self.last_detections
        if model_to_use is None:
            self.last_detections = {}
            return {}
        is_gate = model_to_use is self.gate_model
        # Gate: gunakan conf rendah (0.25) agar box kecil (jauh) keluar dari model,
        # lalu filter adaptif di validate_buoy.
        conf = 0.25 if is_gate else 0.25
        results = model_to_use(frame, verbose=False, imgsz=self.config.YOLO_INFERENCE_SIZE,
                               half=self.config.YOLO_HALF_PRECISION, device=self.config.YOLO_DEVICE,
                               conf=conf)
        detections = {}
        for r in results:
            for box in r.boxes:
                cls_tensor = box.cls
                if cls_tensor is None or len(cls_tensor) == 0: continue 
                cls = int(cls_tensor[0])
                xyxy_tensor = box.xyxy
                if xyxy_tensor is None or len(xyxy_tensor) == 0: continue
                x1, y1, x2, y2 = map(int, xyxy_tensor[0])
                w = x2 - x1
                h = y2 - y1
                area = w * h
                if is_gate and cls in (self.config.RED_BALL_CLASS_ID,
                                       self.config.GREEN_BALL_CLASS_ID):
                    # Ambang conf adaptif: box kecil (jauh) pakai threshold longgar
                    conf_thresh = (self.config.BUOY_CONF_SMALL_THRESHOLD
                                   if area < self.config.BUOY_SMALL_AREA_PX
                                   else self.config.BUOY_CONF_THRESHOLD)
                    if float(box.conf[0]) < conf_thresh:
                        continue
                    # Debug: panggil validate_buoy(..., debug=True) & cetak
                    # alasan tolak (throttle maks 1x/detik agar tidak spam).
                    if self.config.DETECTION_DEBUG:
                        ok, reasons = validate_buoy(
                            frame, cls, (x1, y1, x2, y2),
                            min_area=self.config.MIN_BUOY_AREA_PX,
                            min_color_fraction=self.config.BUOY_MIN_COLOR_FRACTION,
                            min_saturation=self.config.BUOY_MIN_SATURATION,
                            max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION,
                            debug=True)
                        if not ok:
                            now = time.time()
                            if not hasattr(self, '_last_debug_print') or now - self._last_debug_print > 1.0:
                                print(f"[DETECT DEBUG] buoy cls={cls} area={area} ditolak: {'; '.join(reasons)}")
                                self._last_debug_print = now
                        if not ok:
                            continue
                    else:
                        if not validate_buoy(
                                frame, cls, (x1, y1, x2, y2),
                                min_area=self.config.MIN_BUOY_AREA_PX,
                                min_color_fraction=self.config.BUOY_MIN_COLOR_FRACTION,
                                min_saturation=self.config.BUOY_MIN_SATURATION,
                                max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION):
                            continue
                if cls not in detections: detections[cls] = []
                detections[cls].append({'cx':(x1+x2)//2, 'cy':(y1+y2)//2, 'box':(x1,y1,x2,y2), 'area':area})
        self.last_detections = detections
        return detections

    def _find_best_box(self, detections, target_class_id, use_roi=False):
        boxes = detections.get(target_class_id, [])
        if use_roi and self.roi_top_y_cutoff > 0:
            boxes = [b for b in boxes if b['cy'] >= self.roi_top_y_cutoff]
        if not boxes: return None
        best_box = max(boxes, key=lambda b: b['area'])
        return best_box

    def _get_distance_to_box(self, box, bwm, flp):
        try:
            if not isinstance(box, dict) or 'box' not in box: return float('inf')
            pixel_width = box['box'][2] - box['box'][0]
            if pixel_width < 1 or self.config.FOCAL_LENGTH_PX == 0: return float('inf')
            distance_m = (bwm * flp) / pixel_width
            return distance_m
        except (ZeroDivisionError, TypeError, KeyError):
            return float('inf')
            
    def _calculate_yaw_correction_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box: return 0.0
        midpoint_x = best_box['cx']
        error_px = midpoint_x - self.image_center_x
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0: return 0.0
            error_m = (error_px * distance_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError: 
            print("ERROR: FOCAL_LENGTH_PX di Config adalah 0.")
            return 0.0
        raw_correction_rad = math.atan2(error_m, distance_m)
        self.last_used_p_gain = self.config.VISION_P_GAIN 
        
        # PID + deadband (pengganti raw * VISION_P_GAIN).
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN)

    def _take_photo(self, frame):
        if not self.config.SAVE_GREEN_BOX_PHOTO: return
        try:
            if not os.path.exists(cfg.CAPTURES_DIR): os.makedirs(cfg.CAPTURES_DIR)
            # Nama file unik dengan timestamp
            filename_short = f"green_box_{int(time.time())}.jpg"
            filename_full = os.path.join(cfg.CAPTURES_DIR, filename_short)
            
            success = cv2.imwrite(filename_full, frame)
            if success:
                print(f"--- Foto Box Hijau disimpan: {filename_full} ---")
                # Trigger Upload di Thread terpisah
                threading.Thread(target=self._upload_snapshot_to_server, args=(filename_full, filename_short), daemon=True).start()
            else:
                print(f"Gagal menyimpan foto ke {filename_full}")
        except Exception as e:
            print(f"Gagal menyimpan foto: {e}")

    # --- [MODIFIKASI 2: Update _take_waypoint_photo dengan Settingan Terang] ---
    def _take_waypoint_photo(self):
        print(f"Membuka Kamera WP (Index {self.config.WAYPOINT_PHOTO_CAMERA_INDEX})...")
        cap = None 
        try:
            cam_idx = self.config.WAYPOINT_PHOTO_CAMERA_INDEX
            cap = open_camera(cam_idx, 640, 480) 
            if cap is None:
                print(f"ERROR: Gagal membuka kamera foto waypoint di indeks {cam_idx}.")
                return
            
            # == SETTINGAN OBAT KUAT ==
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Force Brightness
            
            # Warmup 20 frame (Wajib)
            for _ in range(20): cap.read()
            
            ret, frame = cap.read()
            if not ret:
                print("ERROR: Gagal mengambil frame WP.")
                cap.release(); return
                
            print("Frame foto WP berhasil diambil.")
            cap.release()

            # Simpan & Upload
            dir_name = self.config.WAYPOINT_PHOTO_DIR
            if not os.path.exists(dir_name): os.makedirs(dir_name)
            
            filename_short = f"wp_photo_WP{self.current_waypoint_index}_{int(time.time())}.jpg"
            filename_full = os.path.join(dir_name, filename_short)
            
            success = cv2.imwrite(filename_full, frame)
            if success:
                print(f"--- Foto Waypoint disimpan: {filename_full} ---")
                threading.Thread(target=self._upload_snapshot_to_server, args=(filename_full, filename_short), daemon=True).start()
        except Exception as e:
            print(f"ERROR saat akses kamera foto: {e}")
            if cap and cap.isOpened(): cap.release()

    def _control_dt(self):
        """Selang waktu loop nyata (detik) untuk PID/filter.

        dt diukur dari `self._last_loop_time` (di-set di __init__), bukan
        tebakan — koreksi tetap benar walau loop berjalan 20 fps (fps turun
        dari 30) atau tersendat oleh YOLO.
        """
        now = time.time()
        dt = now - self._last_loop_time
        self._last_loop_time = now
        return max(dt, 1e-4)

    def _track_gate(self, detections):
        """Pilih target gate aktif lewat GateSequencer (titik tengah).

        Menggantikan `_find_best_gate` per-frame: sequencer mengumpulkan
        SEMUA pasangan, latch target aktif (tahan walau sempat hilang), dan
        melompat cepat ke gate berikutnya saat yang aktif dilewati.

        Return: (best_gate_pair, distance_m); (None, inf) bila tidak ada.
        Midpoint aktif disimpan di `self.gate_active_mid` untuk visualisasi.
        """
        # Reset antrean setiap berganti leg (cara terbit memori antar gate).
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
            pairs = collect_gate_pairs(
                red_balls, green_balls,
                self.config.GATE_VERTICAL_ALIGN_PX,
                self.config.GATE_AREA_SIMILARITY_RATIO)

        mid_x, mid_y, dist, is_passed = self.gate_seq.update(
            pairs, self.image_center_y)

        if mid_x is None:
            self.gate_active_mid = None
            return None, float('inf')

        self.gate_active_mid = (mid_x, mid_y)
        pair = self.gate_seq.active_pair
        if pair is None or is_passed:
            # Target aktif sudah dilewati / tidak ada pasangan valid lagi —
            # jangan koreksi arah; sequencer sudah menyiapkan gate berikutnya.
            return None, float('inf')
        return pair, dist

    def _find_best_gate(self, detections):
        red_balls = detections.get(self.config.RED_BALL_CLASS_ID, [])
        green_balls = detections.get(self.config.GREEN_BALL_CLASS_ID, [])
        min_area = self.config.MIN_BUOY_AREA_PX
        red_balls = [b for b in red_balls if b['area'] >= min_area]
        green_balls = [b for b in green_balls if b['area'] >= min_area]

        if not red_balls or not green_balls: return None, float('inf') 

        FOCAL_LENGTH_PX = self.config.FOCAL_LENGTH_PX
        GATE_WIDTH_METERS = self.config.GATE_WIDTH_METERS
        AREA_SIMILARITY_RATIO_THRESHOLD = self.config.GATE_AREA_SIMILARITY_RATIO
        vertical_alignment_threshold = self.config.GATE_VERTICAL_ALIGN_PX 
        plausible_pairs = [] 

        for r_ball in red_balls:
            for g_ball in green_balls:
                if abs(r_ball['cy'] - g_ball['cy']) > vertical_alignment_threshold: continue 
                r_area = r_ball['area']; g_area = g_ball['area']
                if r_area == 0 or g_area == 0: continue
                max_area = max(r_area, g_area)
                if max_area == 0: continue 
                if min(r_area, g_area) / max_area < AREA_SIMILARITY_RATIO_THRESHOLD: continue 
                plausible_pairs.append((r_ball, g_ball))

        if not plausible_pairs: return None, float('inf') 
        best_pair = None
        min_estimated_distance = float('inf') 
        for pair in plausible_pairs:
            r_ball, g_ball = pair
            current_distance_m = float('inf')
            pixel_width = abs(r_ball['cx'] - g_ball['cx'])
            if pixel_width > 5 and FOCAL_LENGTH_PX > 0: 
                try:
                    current_distance_m = (GATE_WIDTH_METERS * FOCAL_LENGTH_PX) / pixel_width
                except ZeroDivisionError:
                    current_distance_m = float('inf')
            if current_distance_m < min_estimated_distance:
                min_estimated_distance = current_distance_m
                best_pair = pair
        if best_pair is None:
            return None, float('inf')
        return best_pair, min_estimated_distance

    def _apply_yaw_pid(self, raw_correction_rad, dynamic_gain):
        """PID + deadband untuk koreksi yaw (app/filtering.py == core C).

        Pengganti pola lama `raw * gain`. Gain P tetap bisa dinamis
        (fuzzy / VISION_P_GAIN) — di-set ulang tiap panggilan; term
        integral & derivatif memakai dt loop nyata (`_control_dt`) supaya
        perilaku tidak berubah saat fps turun (20 fps).
        """
        self.pid_gate.kp = dynamic_gain
        return self.pid_gate.update(raw_correction_rad, self._control_dt())

    def _calculate_yaw_correction_gate(self, best_gate, distance_m):
        if not isinstance(best_gate, (list, tuple)) or len(best_gate) != 2: return 0.0
        r_ball, g_ball = best_gate
        if not isinstance(r_ball, dict) or not isinstance(g_ball, dict) or 'cx' not in r_ball or 'cx' not in g_ball: return 0.0
            
        midpoint_x = (r_ball['cx'] + g_ball['cx']) / 2.0
        error_px = midpoint_x - self.image_center_x
        
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0: return 0.0
            error_m = (error_px * distance_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError: 
            print("ERROR: FOCAL_LENGTH_PX di Config adalah 0.")
            return 0.0 
            
        raw_correction_rad = math.atan2(error_m, distance_m)
        dynamic_p_gain = self.config.VISION_P_GAIN
        
        if FUZZY_ENABLED:
            # Sugeno singleton murni (app/fuzzy.py == core C). Input diklem
            # ke semesta 0..2 m; error_px dipakai abs langsung (< 320 piksel
            # karena offset dari pusat frame ±160 px).
            dynamic_p_gain = gate_p_gain(min(max(distance_m, 0.0), 2.0),
                                         abs(error_px))
        
        self.last_used_p_gain = dynamic_p_gain
        # PID + deadband (bukan raw * gain): lihat _apply_yaw_pid.
        return self._apply_yaw_pid(raw_correction_rad, dynamic_p_gain)

    def _calculate_yaw_correction_blue_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box: return 0.0
        box_center_x_px = best_box['cx']
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0: return 0.0
            offset_in_pixels = (self.config.BLUE_BOX_LATERAL_OFFSET_M * self.config.FOCAL_LENGTH_PX) / distance_m
            target_x_in_frame = box_center_x_px + offset_in_pixels
            error_px = target_x_in_frame - self.image_center_x
            error_m = (error_px * distance_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError: 
            print("ERROR: FOCAL_LENGTH_PX di Config adalah 0.")
            return 0.0
        
        raw_correction_rad = math.atan2(error_m, distance_m)
        self.last_used_p_gain = self.config.VISION_P_GAIN
        # PID + deadband (pengganti raw * VISION_P_GAIN).
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN)

    def _calculate_yaw_correction_red_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box: return 0.0
            
        midpoint_x = best_box['cx']
        error_px = midpoint_x - self.image_center_x
        
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0: return 0.0
            error_m = (error_px * distance_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError: 
            print("ERROR: FOCAL_LENGTH_PX di Config adalah 0.")
            return 0.0
            
        raw_correction_rad = math.atan2(error_m, distance_m)
        dynamic_p_gain = self.config.VISION_P_GAIN
        
        if FUZZY_ENABLED:
            # Sugeno singleton murni (app/fuzzy.py == core C). Input diklem
            # ke semesta 0..10 m; error_px dipakai abs langsung.
            dynamic_p_gain = docking_p_gain(min(max(distance_m, 0.0), 10.0),
                                            abs(error_px))
        
        self.last_used_p_gain = dynamic_p_gain

        # PID + deadband (pengganti raw * dynamic_p_gain) — dock merah.
        return self._apply_yaw_pid(raw_correction_rad, dynamic_p_gain)

    def _visualize(self, frame, detections, best_gate, best_box, best_red_box, jarak_ke_wp, box_distance, red_box_distance, print_status="N/A"):
        cv2.putText(frame, f"STATE: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        wp_text = f"Next WP: #{self.current_waypoint_index} ({jarak_ke_wp:.1f} m)"
        if not self.waypoints or self.current_waypoint_index >= len(self.waypoints): wp_text = "MISSION COMPLETE"
        cv2.putText(frame, wp_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        status_color = (0, 255, 0)
        if print_status.startswith("VISION"):
            status_color = (0, 255, 0)
        elif print_status.startswith("WAYPOINT") or print_status.startswith("TRANSIT"):
            status_color = (0, 165, 255)
        elif print_status.startswith("GEOFENCE"):
            status_color = (0, 0, 255)
        elif print_status == "N/A" and self.current_state != "WAYPOINT_NAV":
             status_color = (0, 255, 255)
        
        nav_text = print_status
        if self.current_state not in ["WAYPOINT_NAV", "WAYPOINT_TRANSITION"]:
            nav_text = "MISSION_TASK"
            status_color = (0, 255, 255)

        cv2.putText(frame, f"NAV: {nav_text}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)

        gain_text = f"P-Gain: {self.last_used_p_gain:.2f}"
        gain_color = (0, 255, 0) if self.last_used_p_gain > 0.0 else (100, 100, 100)
        if FUZZY_ENABLED and self.last_used_p_gain != self.config.VISION_P_GAIN and self.last_used_p_gain > 0.0: gain_text += " (Fuzzy)"
        
        cv2.putText(frame, gain_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, gain_color, 2)

        min_area = self.config.MIN_BUOY_AREA_PX
        buoy_counts = { "red": 0, "green": 0 } 
        
        if self.current_state == "WAYPOINT_NAV" or self.current_state == "WAYPOINT_TRANSITION":
            red_balls = detections.get(self.config.RED_BALL_CLASS_ID, [])
            green_balls = detections.get(self.config.GREEN_BALL_CLASS_ID, [])
            buoy_counts = { "red": len(red_balls), "green": len(green_balls) } 
            for ball in red_balls: 
                if ball['area'] >= min_area: cv2.rectangle(frame, ball['box'][0:2], ball['box'][2:], (0, 0, 255), 2)
            for ball in green_balls: 
                if ball['area'] >= min_area: cv2.rectangle(frame, ball['box'][0:2], ball['box'][2:], (0, 255, 0), 2)
            if best_gate:
                if self.gate_active_mid is not None:
                    midpoint_x = int(self.gate_active_mid[0])
                    midpoint_y = int(self.gate_active_mid[1])
                else:
                    midpoint_x = int((best_gate[0]['cx'] + best_gate[1]['cx']) / 2.0)
                    midpoint_y = int((best_gate[0]['cy'] + best_gate[1]['cy']) / 2.0)
                cv2.line(frame, (midpoint_x, 0), (midpoint_x, self.processing_height), (255, 255, 0), 1)
                cv2.circle(frame, (midpoint_x, midpoint_y), 7, (0, 255, 255), -1) 
                cv2.putText(frame, "TENGAH", (midpoint_x + 10, max(12, midpoint_y - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2) 

        elif self.current_state.startswith("APPROACH_BOX") or self.current_state == "RETREAT":
            for box in detections.get(self.config.GREEN_BOX_CLASS_ID, []): 
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:], (0, 255, 128), 3)
            if best_box:
                midpoint_x = best_box['cx']
                midpoint_y = best_box['cy']
                cv2.circle(frame, (midpoint_x, midpoint_y), 7, (0, 255, 128), -1)
                if box_distance != float('inf'):
                    cv2.putText(frame, f"BOX GREEN: {box_distance:.2f} m", (midpoint_x + 10, midpoint_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 128), 2)
        
        elif self.current_state.startswith("APPROACH_BLUE_BOX") or self.current_state == "TAKE_BLUE_BOX_PHOTO" or self.current_state == "BLUE_BOX_RETREAT":
            blue_box_distance = float('inf')
            if best_box: blue_box_distance = self._get_distance_to_box(best_box, self.config.BLUE_BOX_WIDTH_METERS, self.config.FOCAL_LENGTH_PX)
            for box in detections.get(self.config.BLUE_BOX_CLASS_ID, []): 
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:], (255, 128, 0), 3) 
            if best_box: 
                midpoint_x = best_box['cx']
                midpoint_y = best_box['cy']
                cv2.circle(frame, (midpoint_x, midpoint_y), 7, (255, 128, 0), -1)
                if blue_box_distance != float('inf') and blue_box_distance > 0.1:
                    offset_in_pixels = (self.config.BLUE_BOX_LATERAL_OFFSET_M * self.config.FOCAL_LENGTH_PX) / blue_box_distance
                    target_x_in_frame = int(midpoint_x + offset_in_pixels)
                    cv2.drawMarker(frame, (target_x_in_frame, midpoint_y), (0, 0, 255), cv2.MARKER_CROSS, 20, 3)
                    cv2.line(frame, (int(self.image_center_x), self.processing_height), (target_x_in_frame, midpoint_y), (0, 0, 255), 2)
                    cv2.putText(frame, f"BOX BLUE: {blue_box_distance:.2f} m", (midpoint_x + 10, midpoint_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 2)

        elif self.current_state.startswith("APPROACH_RED_BOX") or self.current_state == "RED_BOX_DOCKED":
            for box in detections.get(self.config.RED_BOX_CLASS_ID, []): 
                cv2.rectangle(frame, box['box'][0:2], box['box'][2:], (0, 128, 255), 3) 
            if best_red_box:
                midpoint_x = best_red_box['cx']
                midpoint_y = best_red_box['cy']
                cv2.circle(frame, (midpoint_x, midpoint_y), 7, (0, 128, 255), -1)
                if red_box_distance != float('inf'):
                    cv2.putText(frame, f"BOX RED: {red_box_distance:.2f} m", (midpoint_x + 10, midpoint_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 2)
        
        if self.video_writer is not None: self.video_writer.write(frame)
        
        # if self.redis_client:
        #     with self.redis_frame_lock:
        #         if self.redis_publish_data is None:
        #             img_to_send = frame if self.stream_display_mode == "processed" else frame_raw
        #             self.redis_publish_data = (img_to_send, buoy_counts)
        return buoy_counts

    def _upload_snapshot_to_server(self, file_path, filename):
        if not os.path.exists(file_path):
            print(f"UPLOAD GAGAL: File tidak ditemukan di {file_path}")
            return
        print(f"Meng-upload {filename} ke {self.config.SERVER_UPLOAD_URL}...")
        try:
            with open(file_path, 'rb') as f:
                files_payload = {'file': (filename, f, 'image/jpeg')}
                response = requests.post(self.config.SERVER_UPLOAD_URL, files=files_payload, timeout=10)
                if response.status_code == 200:
                    print(f"UPLOAD SUKSES: {filename} (Server merespon: {response.text})")
                else:
                    print(f"UPLOAD GAGAL: Server merespon {response.status_code} - {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"UPLOAD ERROR: Koneksi ke server gagal: {e}")
        except Exception as e:
            print(f"UPLOAD ERROR: Terjadi error: {e}")

    def _start_video_recording(self):
        if not self.config.ENABLE_SESSION_RECORDING:
            print("--- [Config] ENABLE_SESSION_RECORDING di-set False. Perekaman video sesi NONAKTIF. ---")
            self.video_writer = None
            return
        try:
            dir_name = self.config.SESSION_VIDEO_DIR
            if not os.path.exists(dir_name): os.makedirs(dir_name)
            filename = os.path.join(dir_name, f"session_record_{int(time.time())}.avi")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            frame_size = (self.processing_width, self.processing_height)
            self.video_writer = cv2.VideoWriter(filename, fourcc, self.config.SESSION_VIDEO_FPS, frame_size)
            print(f"\n--- [Perekam Sesi] Mulai merekam ke: {filename} ---")
        except Exception as e:
            print(f"PERINGATAN: Gagal memulai perekam video sesi: {e}")
            self.video_writer = None

    def _stop_video_recording(self):
        if self.video_writer is not None:
            print(f"--- [Perekam Sesi] Berhenti merekam. Menyimpan file... ---")
            self.video_writer.release()
            self.video_writer = None

    def _set_attitude_target(self, thrust, target_yaw_rad, lateral_thrust=0.0):
        thrust_limited = max(-1.0, min(1.0, thrust))
        lateral_thrust_limited = max(-1.0, min(1.0, lateral_thrust))
        type_mask = (1 << 1) | (1 << 2) 
        body_roll_rate_cmd = 0.0
        if abs(lateral_thrust_limited) < 0.01:
            type_mask = type_mask | (1 << 0) 
        else:
            body_roll_rate_cmd = lateral_thrust_limited
        quat = self._yaw_to_quaternion(target_yaw_rad)
        if self.master:
            try:
                self.master.mav.set_attitude_target_send(0, self.master.target_system, self.master.target_component, type_mask, quat, body_roll_rate_cmd, 0, 0, thrust_limited)
            except Exception as e:
                print(f"Error sending attitude target: {e}")

    def _stream_offboard_command(self, thrust, target_yaw_rad, status="N/A", force_send=False, lateral_thrust=0.0): 
        current_time = time.time()
        time_since_last_stream = current_time - self.last_stream_time
        stream_interval = 1.0 / self.config.OFFBOARD_STREAM_RATE_HZ if self.config.OFFBOARD_STREAM_RATE_HZ > 0 else float('inf')
        if force_send or time_since_last_stream >= stream_interval:
            if not isinstance(target_yaw_rad, (int, float)) or math.isnan(target_yaw_rad):
                print(f"Invalid target_yaw_rad: {target_yaw_rad}, using current yaw.")
                target_yaw_rad = self.current_yaw_rad or 0.0 
            self._set_attitude_target(thrust, target_yaw_rad, lateral_thrust)
            self.last_stream_time = current_time
            pass

    def _prepare_for_offboard(self):
        print("Mengirim stream awal (3 detik)...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self._set_attitude_target(0.0, 0)
            sleep_duration = max(0, (1.0 / self.config.OFFBOARD_STREAM_RATE_HZ) - 0.001)
            time.sleep(sleep_duration) 
        print("SISTEM SIAP. Silakan ARMING dan ganti ke mode OFFBOARD.")

    def _cleanup(self):
        print("\nMembersihkan resource...")
        self._stop_video_recording() 
        if hasattr(self, 'redis_publish_thread') and self.redis_publish_thread.is_alive():
            print("Menghentikan thread publisher Redis...")
            self.redis_stop_event.set()
            self.redis_publish_thread.join(timeout=1.0) 
        if hasattr(self, 'master') and self.master:
            print("Mengirim perintah berhenti...")
            for _ in range(10): 
                current_yaw = self.current_yaw_rad or 0.0
                self._set_attitude_target(0.0, current_yaw, 0.0) 
                time.sleep(1.0 / self.config.OFFBOARD_STREAM_RATE_HZ + 0.01)
            try:
                self.master.close()
            except Exception as e:
                print(f"Error closing MAVLink connection: {e}")
        if hasattr(self, 'cap') and self.cap and self.cap.isOpened():
            self.cap.release()
            print("Kamera utama dilepaskan.")
        if hasattr(self, 'wp_photo_cap') and self.wp_photo_cap and self.wp_photo_cap.isOpened():
            self.wp_photo_cap.release()
            print("Kamera foto waypoint dilepaskan.")
        if hasattr(self, 'redis_client') and self.redis_client:
            print("Menutup koneksi Redis...")
            self.redis_client.close()
        print("Selesai.")

    def _normalize_angle(self, angle_rad):
        """Bawa sudut ke [-pi, pi]. Implementasi terpusat di app.geo."""
        return geo.normalize_angle(angle_rad)

    def _get_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        """Jarak (m) & bearing (rad) antar koordinat — lihat app.geo."""
        return geo.distance_bearing(lat1, lon1, lat2, lon2)

    def _get_cross_track_distance(self, lat_p, lon_p, lat_wp1, lon_wp1, lat_wp2, lon_wp2):
        """Simpangan titik P dari garis WP1->WP2 (m) — lihat app.geo."""
        return geo.cross_track_distance(lat_p, lon_p, lat_wp1, lon_wp1, lat_wp2, lon_wp2)

    def _yaw_to_quaternion(self, yaw_rad):
        # (Fungsi _yaw_to_quaternion tidak berubah)
        if not isinstance(yaw_rad, (int, float)) or math.isnan(yaw_rad):
            print(f"Invalid yaw_rad for quaternion: {yaw_rad}, using 0.0")
            yaw_rad = 0.0
        cy = math.cos(yaw_rad * 0.5); sy = math.sin(yaw_rad * 0.5)
        cr = 1.0; sr = 0.0
        cp = 1.0; sp = 0.0
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        return [w, x, y, z]

    def _battery_fields(self):
        """Field baterai untuk paket GUI — None = belum ada data (GUI tulis '--').

        LiPO 4S penuh 16.8 V / kosong ~12.8 V. Persen dihitung linear dari
        tegangan bila firmware tidak mengirim battery_remaining.
        """
        volt = getattr(self, "current_voltage", None)
        curr = getattr(self, "current_current", None)
        pct = getattr(self, "current_battery_pct", None)
        if pct is None and volt is not None:
            pct = max(0.0, min(100.0, (volt - 12.8) / (16.8 - 12.8) * 100.0))
        return {"voltage_v": volt, "current_a": curr, "battery_pct": pct}

    def _publish_telemetry(self):
        if self.current_lat is None or self.current_yaw_rad is None: return
        current_roll = 0.0; current_pitch = 0.0
        if self.last_attitude_msg:
            current_roll = self.last_attitude_msg.roll
            current_pitch = self.last_attitude_msg.pitch
        try:
            telemetry_data = {
                "roll": current_roll, "pitch": current_pitch, "yaw": self.current_yaw_rad,
                "lat": self.current_lat, "lon": self.current_lon,
                "groundspeed": self.current_groundspeed, "heading": math.degrees(self.current_yaw_rad),
                "voltage": getattr(self, 'current_voltage', None),
                "nuc_signal": self.current_nuc_signal_ms,
                "target_wp_idx": self.current_waypoint_index,
                "state": self.current_state
            }
            payload = { "type": "telemetry", "data": telemetry_data }
            if self.redis_client: self.redis_client.publish(self.config.TELEMETRY_CHANNEL, json.dumps(payload))
        except Exception as e:
            print(f"PERINGATAN: Gagal publish telemetri ke Redis: {e}")
    
    def _publish_vision_frame(self, frame, status_message, buoy_counts=None):
        pass 

class NavigatorThread(QThread):
    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        global WAYPOINTS
        WAYPOINTS = waypoints
        self.config = cfg.Config()
        self.navigator = VisionOffboardNavigator(self.config)

    def save_config_to_file(self):
        if not hasattr(self, 'config'):
            print("Penyimpanan Gagal: Objek config belum ada.")
            return
        print(f"Menyimpan parameter tuning ke {TUNING_FILE}...")
        params_to_save = {
            'ACCEPTANCE_RADIUS_M': self.config.ACCEPTANCE_RADIUS_M,
            'THRUST_VALUE': self.config.THRUST_VALUE,
            'TRANSITION_DURATION_S': self.config.TRANSITION_DURATION_S,
            'GEOFENCE_WIDTH_METERS': self.config.GEOFENCE_WIDTH_METERS,
            'FOCAL_LENGTH_PX': self.config.FOCAL_LENGTH_PX,
            'VISION_P_GAIN': self.config.VISION_P_GAIN,
            'VISION_SMOOTHING_ALPHA': self.config.VISION_SMOOTHING_ALPHA,
            'ROI_TOP_CUTOFF_PERCENT': self.config.ROI_TOP_CUTOFF_PERCENT,
            'GATE_WIDTH_METERS': self.config.GATE_WIDTH_METERS,
            'MIN_BUOY_AREA_PX': self.config.MIN_BUOY_AREA_PX,
            'GATE_AREA_SIMILARITY_RATIO': self.config.GATE_AREA_SIMILARITY_RATIO,
            'STOP_AND_PHOTO_AT_WP': self.config.STOP_AND_PHOTO_AT_WP,
            'WAYPOINT_PHOTO_STOP_DURATION_S': self.config.WAYPOINT_PHOTO_STOP_DURATION_S,
            'SEARCH_THRUST': self.config.SEARCH_THRUST,
            'ALIGN_THRUST': self.config.ALIGN_THRUST,
            'RETREAT_THRUST': self.config.RETREAT_THRUST,
            'RETREAT_DURATION_S': self.config.RETREAT_DURATION_S,
            'BOX_WIDTH_METERS': self.config.BOX_WIDTH_METERS,
            'BOX_APPROACH_DISTANCE_M': self.config.BOX_APPROACH_DISTANCE_M,
            'YAW_SEARCH_BOX': self.config.YAW_SEARCH_BOX,
            'BOX_SEARCH_LATERAL_THRUST': self.config.BOX_SEARCH_LATERAL_THRUST,
            'BLUE_BOX_WIDTH_METERS': self.config.BLUE_BOX_WIDTH_METERS,
            'BLUE_BOX_APPROACH_DISTANCE_M': self.config.BLUE_BOX_APPROACH_DISTANCE_M,
            'BLUE_BOX_LATERAL_OFFSET_M': self.config.BLUE_BOX_LATERAL_OFFSET_M,
            'BLUE_BOX_ALIGN_THRUST': self.config.BLUE_BOX_ALIGN_THRUST,
            'BLUE_BOX_SEARCH_THRUST': self.config.BLUE_BOX_SEARCH_THRUST,
            'BLUE_BOX_YAW_SEARCH': self.config.BLUE_BOX_YAW_SEARCH,
            'RED_BOX_WIDTH_METERS': self.config.RED_BOX_WIDTH_METERS,
            'RED_BOX_DOCK_DISTANCE_M': self.config.RED_BOX_DOCK_DISTANCE_M,
            'DOCK_ALIGN_THRUST': self.config.DOCK_ALIGN_THRUST,
            'DOCK_HOLD_DURATION_S': self.config.DOCK_HOLD_DURATION_S,
            'YAW_SEARCH_DOCK': self.config.YAW_SEARCH_DOCK,
            'YOLO_FRAME_SKIP': self.config.YOLO_FRAME_SKIP,
            'YOLO_INFERENCE_SIZE': self.config.YOLO_INFERENCE_SIZE,
            'SESSION_VIDEO_FPS': self.config.SESSION_VIDEO_FPS,
            'VISION_ENABLED_LEGS': self.config.VISION_ENABLED_LEGS,
            'PHOTO_BOX_LEGS': self.config.PHOTO_BOX_LEGS,
            'BLUE_BOX_PHOTO_LEGS': self.config.BLUE_BOX_PHOTO_LEGS,
            'STOP_AND_PHOTO_AT_WP': self.config.STOP_AND_PHOTO_AT_WP
        }
        try:
            with open(TUNING_FILE, 'w') as f:
                json.dump(params_to_save, f, indent=4)
            print("Parameter berhasil disimpan.")
        except Exception as e:
            print(f"ERROR: Gagal menyimpan parameter tuning: {e}")

    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            print(f"[TUNING] {key} updated to {value}")
        else:
            print(f"[ERROR] Config key '{key}' not found!")
        
    def run(self):
        print("Starting NavigatorThread (ASLI DENGAN PX4 & REDIS)...")
        try:
            self.navigator.run(self.newData) 
        except Exception as e:
            print(f"FATAL ERROR IN NAVIGATOR THREAD: {e}")
            import traceback
            traceback.print_exc()
        print("NavigatorThread finished.")

    def stop(self):
        if self.navigator:
            self.navigator.stop()
            
    def update_waypoints(self, new_waypoints):
        global WAYPOINTS
        WAYPOINTS = new_waypoints
        if not self.navigator: return
        print("\n--- [RESET MISSION] Perintah Save/Reload diterima ---")
        self.navigator.waypoints = new_waypoints
        if not new_waypoints or len(new_waypoints) == 0:
            self.navigator.current_waypoint_index = 0
            self.navigator._set_state_and_publish("NO_WAYPOINTS")
            print("[RESET MISSION] Tidak ada waypoint baru. Berhenti (IDLE).")
        else:
            self.navigator.current_waypoint_index = 0
            if self.navigator.current_lat is not None:
                 self.navigator.leg_start_lat = self.navigator.current_lat
                 self.navigator.leg_start_lon = self.navigator.current_lon
            else:
                 self.navigator.leg_start_lat = new_waypoints[0]['lat']
                 self.navigator.leg_start_lon = new_waypoints[0]['lon']
            self.navigator.task_timer = time.time() 
            self.navigator.retreat_step = "IDLE"
            self.navigator.last_vision_correction_rad = 0.0
            self.navigator._set_state_and_publish("WAYPOINT_TRANSITION")
            print(f"[RESET MISSION] Misi direset. Menuju ke WP 0 baru.")
        print(f"Navigator waypoints updated (HARD RESET).")
