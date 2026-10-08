from . import settings as cfg
from . import aterkia_core as core
from .manual_link import ManualLink
from .camera import open_camera, flip_frame_if_needed, pace_to_fps
from .detection_validation import validate_buoy, frame_brightness
from .yolo_async import YoloAsyncWorker
from .uploader import UploadWorker
from .logutil import get_logger, throttled
# Sequencer gate + koleksi pasangan: hitung di C (gate_vision via
# aterkia_core). Modul Python app/gate_sequencer.py dipertahankan HANYA
# sebagai referensi uji (tests/test_gate_sequencer.py) — jalur produksi
# di file ini TIDAK mengimpornya langsung.
from ultralytics import YOLO
from pymavlink import mavutil
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import time
import math
import cv2
import redis
import base64
import threading
import os
import json

log = get_logger()

# Hitungan navigasi (PID/EKF/komplementer/geodesi/fuzzy) berjalan di C via
# app/aterkia_core.py (core/libaterkia.so Linux | aterkia_core.dll Windows). Modul Python app/filtering.py,
# app/geo.py & app/fuzzy.py dipertahankan HANYA sebagai referensi uji
# (tests/test_*.py cross-check bit-per-bit) — jalur produksi di file ini
# TIDAK mengimpornya langsung.
FUZZY_ENABLED = True
log.info("Fuzzy Logic (core C fuzzy_gate/docking_p_gain) SIAP via aterkia_core.")


WAYPOINTS = [] 

TUNING_FILE = cfg.TUNING_FILE


class VisionOffboardNavigator:

    # NOTE: Factory fuzzy skfuzzy (_create_gate_controller/_create_docking_controller)
    # DIHAPUS — scikit-fuzzy >= 0.5 gagal membuat singleton output. Desain
    # Sugeno yang sama (trapmf, AND=min, rata-rata tertimbang) sekarang jalan
    # di C (core/src/fuzzy.c) via app/aterkia_core (fuzzy_gate/docking_gain).


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

        # --- Filter & kontroler galat (hitung di C via aterkia_core) ---
        # PID + deadband pengganti gain-P murni untuk koreksi yaw.
        # app/filtering.py dipertahankan hanya sebagai referensi uji.
        self.pid_gate = core.PidState(
            kp=self.config.PID_KP, ki=self.config.PID_KI, kd=self.config.PID_KD,
            deadband=self.config.PID_DEADBAND,
            output_limit=self.config.PID_OUTPUT_LIMIT,
            integral_limit=self.config.PID_INTEGRAL_LIMIT,
        )
        self.ekf_heading = core.EkfState(
            process_noise=self.config.EKF_PROCESS_NOISE,
            meas_noise=self.config.EKF_MEAS_NOISE,
        )
        # State filter komplementer untuk heading target.
        self.comp_angle_prev = 0.0
        self._last_loop_time = time.time()
        # Jembatan kendali manual RC/gamepad (hitung di C; kabel di
        # app/manual_link.py). Dipasang ke master MAVLink setelah konek.
        self.manual_link = ManualLink(config)
        self.manual_gui_request = False  # tombol "Manual" GUI (backup darat)
        self.manual_last = {"surge": 0.0, "yaw": 0.0, "active": False,
                            "mode": 0, "mode_name": "AUTO", "rc_ok": False,
                            "source": "AUTO"}

        # --- Antrean target gate (titik tengah merah+hijau, hitung di C) ---
        self.gate_seq = core.GateSequencerC(
            pass_distance_m=self.config.GATE_PASS_DISTANCE_M,
            lost_tolerance_frames=self.config.GATE_LOST_TOLERANCE_FRAMES,
            gate_width_m=self.config.GATE_WIDTH_METERS,
            focal_length_px=self.config.FOCAL_LENGTH_PX,
        )
        self.gate_active_mid = None  # midpoint gate aktif (untuk visualisasi)

        # Fuzzy Sugeno (core C via aterkia_core): tidak butuh inisialisasi
        # objek controller, cukup fungsi murni.
        if FUZZY_ENABLED:
            log.info("Fuzzy Logic Controllers SIAP (C).")

        self.redis_client = None
        try:
            log.info("Menghubungkan ke Redis di %s:%s...",
                     self.config.REDIS_HOST, self.config.REDIS_PORT)
            self.redis_client = redis.Redis(
                host=self.config.REDIS_HOST,
                port=self.config.REDIS_PORT,
                decode_responses=True,
                socket_connect_timeout=0.5)
            self.redis_client.ping()
            log.info("Berhasil terhubung ke Redis.")
        except Exception as e:
            log.warning("Gagal terhubung ke Redis: %s. Web dashboard tidak akan berfungsi.", e)
            self.redis_client = None

        # Hanya model GATE (buoy) yang dimuat saat start — model lain
        # dimuat lazy (thread latar) agar startup < 5 dtk.
        try:
            log.info("Memuat model GATE: %s...", self.config.MODEL_PATH)
            self.gate_model = YOLO(self.config.MODEL_PATH).to(self.config.YOLO_DEVICE)
        except Exception as e:
            raise FileNotFoundError(f"FATAL: Gagal memuat model GATE: {e}")

        # 3 model lain: dimuat di thread latar saat loop pertama / misi butuh.
        self.box_model = None
        self.red_dock_model = None
        self.blue_box_model = None
        self._models_loading = False
        self._models_loaded = False

    def _load_models_async(self):
        """Muat 3 model non-gate di thread latar."""
        if self._models_loading or self._models_loaded:
            return
        self._models_loading = True

        def _loader():
            try:
                log.info("Memuat model BOX HIJAU (lazy)...")
                self.box_model = YOLO(self.config.BOX_MODEL_PATH).to(self.config.YOLO_DEVICE)
            except Exception as e:
                log.warning("Gagal memuat model BOX HIJAU: %s. Misi foto tidak akan berfungsi.", e)
            try:
                log.info("Memuat model BOX MERAH (lazy)...")
                self.red_dock_model = YOLO(self.config.RED_DOCK_MODEL_PATH).to(self.config.YOLO_DEVICE)
            except Exception as e:
                log.warning("Gagal memuat model BOX MERAH: %s. Misi docking tidak akan berfungsi.", e)
            try:
                log.info("Memuat model BOX BIRU (lazy)...")
                self.blue_box_model = YOLO(self.config.BLUE_BOX_MODEL_PATH).to(self.config.YOLO_DEVICE)
            except Exception as e:
                log.warning("Gagal memuat model BOX BIRU: %s. Misi foto WP 8 tidak akan berfungsi.", e)
            self._models_loaded = True
            self._models_loading = False
            log.info("Lazy-load model selesai.")

        threading.Thread(target=_loader, daemon=True, name="lazy-models").start()

    def _init_hardware_dan_lanjut(self):
        """Lanjutan __init__ (kamera, worker, dsb) — dipisah agar rapih."""
        self.roi_top_y_cutoff = int(self.processing_height * self.config.ROI_TOP_CUTOFF_PERCENT)
        if self.roi_top_y_cutoff > 0:
            log.info("ROI diaktifkan: Mengabaikan %s piksel teratas (%s%% dari %s)",
                     self.roi_top_y_cutoff, self.config.ROI_TOP_CUTOFF_PERCENT * 100,
                     self.processing_height)

        # Hasil deteksi TERAKHIR per model async (model_id -> dict). Loop tak
        # pernah menunggu inferensi: `_request_detections` submit job tiap
        # frame-skip, `_take_latest` memakai hasil bila segar.
        self._det_cache = {}
        self._frame_seq = 0
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

        # --- State failsafe otomatis (lihat _check_failsafe) ---
        # Aktif bila telemetri stale ATAU baterai rendah (ditahan agar spike
        # sesaat tak memicu). Saat aktif: thrust 0 + coba RTL best-effort.
        self.failsafe_active = False
        self.failsafe_reason = ""
        self._lowbatt_since = None  # monotonic awal kondisi low-batt
        self._rtl_sent_mono = 0.0   # throttle perintah RTL (maks 1x/5 detik)

        self.redis_publish_data = None
        self.redis_frame_lock = threading.Lock()
        self.redis_stop_event = threading.Event()
        self.redis_publish_thread = threading.Thread(target=self._redis_publish_loop, daemon=True)

    def _redis_publish_loop(self):
        log.info("THREAD REDIS: Dimulai.")
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
                    if throttled("redis_pub", 5.0):
                        log.warning("Gagal publish frame ke Redis: %s", e)
            time.sleep(0.03)
        log.info("THREAD REDIS: Berhenti.")

    def _set_state_and_publish(self, new_state):
        if self.current_state == new_state: return
        self.current_state = new_state
        log.info("STATE CHANGE: -> %s", new_state)
        if not self.redis_client: return
        try:
            payload = {"type": "mission_update", "state_name": new_state}
            self.redis_client.publish(self.config.MISSION_CHANNEL, json.dumps(payload))
        except Exception as e:
            log.warning("Gagal publish status '%s' ke Redis: %s", new_state, e)

    def _signal_monitor_loop(self):
        """Pantau link NUC->internet via TCP connect (bukan ping subprocess).

        Spawn `ping` tiap 2 detik = 1 proses baru terus-menerus (boros di
        RPi + gagal di Windows tanpa ping). TCP connect ke DNS publik dengan
        timeout 2 detik memberi sinyal "internet OK" yang setara untuk
        kebutuhan dashboard, tanpa melahirkan proses.
        """
        import socket
        log.info("THREAD SINYAL: Dimulai.")
        while not self.signal_stop_event.is_set():
            try:
                start = time.monotonic()
                sock = socket.create_connection(("8.8.8.8", 53), timeout=2.0)
                sock.close()
                self.current_nuc_signal_ms = int((time.monotonic() - start) * 1000)
            except Exception:
                self.current_nuc_signal_ms = 999
            time.sleep(2)
        log.info("THREAD SINYAL: Berhenti.")

    def _redis_command_listener(self):
        """Mendengarkan perintah dari Web via Redis."""
        if not self.redis_client: return
        log.info("THREAD COMMAND: Mendengarkan channel 'asv_commands'...")
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
                            log.info("[COMMAND] Mode Stream diubah ke: %s",
                                     new_mode.upper())
                except Exception:
                    pass
            time.sleep(0.1)

    def run(self, data_signal):
        log.info("Mempersiapkan mode Offboard...")
        self._prepare_for_offboard()
        log.info("Menunggu data telemetri pertama (GPS 3D Fix & Attitude)...")

        if self.redis_client:
            log.info("Memulai thread publisher Redis...")
            self.redis_publish_thread.start()

        log.info("Memulai thread monitor sinyal...")
        self.signal_thread.start()

        # Worker YOLO async + uploader: hidup selama misi, berhenti di
        # _cleanup (finally) agar tak ada thread nyangkut.
        self.yolo_worker.start()
        self.upload_worker.start()

        # Lazy-load 3 model non-gate di thread latar (startup cepat).
        self._load_models_async()

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
            log.info("Posisi awal leg diatur ke: %.6f, %.6f",
                     self.leg_start_lat, self.leg_start_lon)

        self._start_video_recording()

        try:
            # Pacing 30 Hz monotonic (sama seperti simulator ground):
            # capture + kontrol + OFFBOARD stream terkunci 30 fps.
            # Inferensi YOLO ASYNC (submit + latest) — loop tak menunggu.
            deadline = time.monotonic()
            while self.running:
                # dt loop tunggal per frame: dipakai manual_link, PID, dan
                # EKF di bawah (dulu _control_dt dipanggil 3x/frame sehingga
                # tiap konsumen dapat potongan dt berbeda).
                loop_dt = self._control_dt()
                self._update_telemetry()
                self._publish_telemetry()

                ret, frame_high_res = self.cap.read()

                frame_raw = None


                if not ret:
                    if throttled("cam_fail", 5.0):
                        log.warning("Frame kamera gagal dibaca! Menggunakan frame hitam.")
                    frame = np.zeros((self.processing_height, self.processing_width, 3), dtype=np.uint8)
                    time.sleep(0.1)
                    frame_raw = frame.copy()
                else:
                    # Orientasi kamera (CAMERA_FLIP_MODE, default 1 = mirror):
                    # objek kanan kapal tampil kanan di GUI.
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
                # Submit job YOLO async (tak menunggu): hasil dibaca tiap
                # state via _detect_objects -> cache worker.
                self._request_detections(frame)
                detections = {}
                best_gate = None; best_box = None; best_red_box = None
                gate_distance = float('inf'); box_distance = float('inf'); red_box_distance = float('inf')

                if self.current_state == "WAYPOINT_NAV":
                    print_status = "WAYPOINT_NAV"
                    if self.current_waypoint_index in self.config.PHOTO_BOX_LEGS:
                        if self.box_model is not None:
                            log.info("\n=== MEMULAI MISI FOTO BOX HIJAU ===")
                            self._set_state_and_publish("APPROACH_BOX_SEARCH")
                            self.last_vision_correction_rad = 0.0
                            self.green_box_confirm_timer = None 
                            continue
                        else:
                            log.info("PERINGATAN: Misi Foto Box Hijau leg terpicu, tapi model box hijau tidak ada!")
                            
                    if self.current_waypoint_index in self.config.BLUE_BOX_PHOTO_LEGS:
                        if self.blue_box_model is not None:
                            log.info("\n=== MEMULAI MISI FOTO SAMPING BOX BIRU (Leg %s) ===", self.current_waypoint_index)
                            self._set_state_and_publish("APPROACH_BLUE_BOX_SEARCH")
                            self.last_vision_correction_rad = 0.0
                            self.blue_box_confirm_timer = None 
                            continue
                        else:
                            log.info("PERINGATAN: Misi Foto Box Biru leg terpicu, tapi model box biru tidak ada!")

                    if jarak_ke_wp < self.config.ACCEPTANCE_RADIUS_M:
                        log.info("\nWaypoint #%s tercapai.", self.current_waypoint_index)
                        if self.current_waypoint_index in self.config.STOP_AND_PHOTO_AT_WP:
                            log.info("\n=== MEMULAI MISI FOTO WAYPOINT #%s ===", self.current_waypoint_index)
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
                    # Pemilihan gate TIDAK lagi per-frame murni (dulu
                    # `_find_best_gate`): GateSequencer mengumpulkan semua
                    # pasangan, mengunci target
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
                        log.info("Masa tenang transisi selesai (%ss). Kembali ke WAYPOINT_NAV.", elapsed)
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
                            log.info("Foto Selesai. Melanjutkan misi...")
                            if hasattr(self, 'wp_photo_taken'): del self.wp_photo_taken

                            if self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP:
                                if self.red_dock_model is not None:
                                    log.info("\n=== FOTO WP SELESAI, MEMULAI MISI DOCKING RED BOX ===")
                                    self._set_state_and_publish("APPROACH_RED_BOX_SEARCH")
                                    self.last_vision_correction_rad = 0.0
                                    continue 
                                else:
                                    log.info("PERINGATAN: Misi Docking Red Box setelah WP %s tidak bisa dimulai (Model tidak ada).", self.current_waypoint_index)

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
                            log.info("BOX_SEARCH: Potensi deteksi... Verifikasi dimulai.")

                        elapsed_confirm = time.time() - self.green_box_confirm_timer
                        
                        if elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S:
                            print_status = "BOX_SEARCH (Confirmed!)"
                            log.info("Konfirmasi Berhasil (%ss). Pindah ke ALIGN.", elapsed_confirm)
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
                            log.info("BOX_SEARCH: Deteksi Gagal/Hilang saat verifikasi. Reset.")
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
                            log.info("BOX_ALIGN (Lost! Starting patience timer...)")
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
                            log.info("\n=== MISI FOTO BOX SELESAI ===")
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
                            log.info("BLUE_BOX_SEARCH: Potensi deteksi... Verifikasi dimulai.")
                        
                        elapsed_confirm = time.time() - self.blue_box_confirm_timer
                        
                        if elapsed_confirm >= self.config.DETECTION_CONFIRM_DURATION_S:
                            print_status = "BLUE_BOX_SEARCH (Confirmed!)"
                            log.info("Konfirmasi Box Biru Berhasil (%ss). Pindah ke ALIGN.", elapsed_confirm)
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
                            log.info("BLUE_BOX_SEARCH: Deteksi Gagal/Hilang saat verifikasi. Reset.")
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
                            log.info("BLUE_BOX_ALIGN (Lost! Starting patience timer...)")
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
                            log.info("\n=== [WP 8] MEMULAI PROSEDUR SMART PHOTO ===")

                            # A. Matikan Kamera Navigasi
                            if self.cap.isOpened():
                                self.cap.release()
                                log.info("[WP 8] Kamera Navigasi dipause.")
                            time.sleep(1.0) 

                            # B. Buka Kamera Bawah (Index 1) dengan Settingan Terang
                            cam_bawah = open_camera(self.config.WAYPOINT_PHOTO_CAMERA_INDEX, 640, 480)
                            
                            if cam_bawah and cam_bawah.isOpened():
                                # Settingan "Obat Kuat"
                                cam_bawah.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                                cam_bawah.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                                cam_bawah.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Force Auto ON (Value 1)

                                log.info("[WP 8] Warming up sensor (20 frames)...")
                                for _ in range(20): cam_bawah.read()

                                foto_sukses = False
                                last_frame_bawah = None # Simpan frame terakhir

                                # C. Loop Percobaan
                                for percobaan in range(1, 6):
                                    ret_b, frame_b = cam_bawah.read()
                                    if ret_b:
                                        last_frame_bawah = frame_b.copy() # Backup frame
                                        avg_bright = np.mean(frame_b)
                                        log.info("[WP 8] Try %s: Brightness = %s", percobaan, avg_bright)
                                        
                                        # Validasi Brightness > 1.0 (Bisa dinaikkan jika perlu)
                                        if avg_bright > 1.0:
                                            timestamp = int(time.time())
                                            fname_short = f"WP8_BlueBox_OK_{timestamp}.jpg"
                                            fname_full = os.path.join(self.config.WAYPOINT_PHOTO_DIR, fname_short)
                                            
                                            if not os.path.exists(self.config.WAYPOINT_PHOTO_DIR): 
                                                os.makedirs(self.config.WAYPOINT_PHOTO_DIR)
                                                
                                            cv2.imwrite(fname_full, frame_b)
                                            log.info("[WP 8] FOTO SUKSES: %s", fname_full)
                                            
                                            self.upload_worker.enqueue(fname_full, fname_short)
                                            foto_sukses = True
                                            break
                                        else:
                                            log.info("[WP 8] Foto Gelap. Retry...")
                                            time.sleep(0.2)
                                
                                # --- FALLBACK: UPLOAD MESKIPUN GELAP ---
                                if not foto_sukses and last_frame_bawah is not None:
                                    log.info("[PERINGATAN] Gagal mendapatkan foto terang setelah 5x percobaan.")
                                    log.info("[ACTION] Mengupload foto terakhir (meskipun gelap) sebagai bukti data.")
                                    
                                    timestamp = int(time.time())
                                    fname_short = f"WP8_BlueBox_DARK_{timestamp}.jpg"
                                    fname_full = os.path.join(self.config.WAYPOINT_PHOTO_DIR, fname_short)
                                    
                                    if not os.path.exists(self.config.WAYPOINT_PHOTO_DIR): 
                                        os.makedirs(self.config.WAYPOINT_PHOTO_DIR)
                                    
                                    cv2.imwrite(fname_full, last_frame_bawah)
                                    # Tetap upload ke web (via antrean 1 worker)
                                    self.upload_worker.enqueue(fname_full, fname_short)

                                cam_bawah.release()
                            else:
                                log.info("[WP 8] Gagal buka kamera bawah air!")
                            
                            self.blue_box_photo_taken = True
                            
                            # D. Nyalakan Lagi Kamera Navigasi
                            log.info("[WP 8] Restarting Nav Camera...")
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
                            log.info("\n=== MISI FOTO BOX BIRU SELESAI ===")
                            if self.red_dock_model is not None:
                                log.info("\n=== MEMULAI MISI DOCKING RED BOX (Setelah Box Biru) ===")
                                self._set_state_and_publish("APPROACH_RED_BOX_SEARCH")
                                self.last_vision_correction_rad = 0.0
                            else:
                                log.info("PERINGATAN: Model docking tidak ada, melanjutkan ke WP Nav...")
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
                        log.info("\n=== MISI DOCKING RED BOX SELESAI ===")
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

                # --- Arbitrasi KILL > MANUAL > AUTO (hitung di C) ---
                # ManualLink membaca RC_CHANNELS Pixhawk (receiver -> RCIN)
                # + gamepad USB, olah kurva stick & mode di C. Bila mode
                # MANUAL: thrust = surge manual, yaw = yaw saat ini +
                # offset proporsional stick (±0.8 rad ≈ ±45°).
                # dt = loop_dt (dt loop tunggal, bukan _control_dt baru).
                try:
                    self.manual_last = self.manual_link.update(
                        dt=loop_dt, kill=False,
                        gui_manual=self.manual_gui_request)
                except Exception:
                    pass
                man = getattr(self, "manual_last", {}) or {}
                if man.get("mode_name") == "MANUAL" and man.get("active"):
                    arb = core.arbitrate(False, True, float(man.get("surge", 0.0)),
                                         float(man.get("yaw", 0.0)),
                                         float(thrust), 0.0)
                    thrust = float(arb["surge"])
                    target_yaw_angle_rad = self._normalize_angle(
                        (self.current_yaw_rad or 0.0)
                        + float(arb["yaw"]) * 0.8)
                    lateral_thrust = 0.0
                    print_status = (f"MANUAL (surge {thrust:+.2f} "
                                    f"yaw {float(man.get('yaw', 0.0)):+.2f})")
                elif man.get("mode_name") == "KILL":
                    thrust = 0.0
                    lateral_thrust = 0.0
                    print_status = "KILL (E-stop)"

                # --- Failsafe otomatis (stale-link / low-batt): timpa thrust
                # & yaw sebelum perintah dikirim ke Pixhawk. Lihat
                # _check_failsafe (aksi + RTL best-effort di dalamnya).
                fs_reason = self._check_failsafe()
                if fs_reason:
                    thrust = 0.0
                    lateral_thrust = 0.0
                    target_yaw_angle_rad = self.current_yaw_rad or 0.0
                    print_status = f"FAILSAFE ({fs_reason})"

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
                    "frame": frame,
                    # Kendali manual (RC/gamepad via C): GUI baca chip mode.
                    "op_mode": (getattr(self, "manual_last", {}) or {}
                                ).get("mode_name", "AUTO"),
                    "manual_active": bool((getattr(self, "manual_last", {})
                                           or {}).get("active", False)),
                    "manual_surge": float((getattr(self, "manual_last", {})
                                           or {}).get("surge", 0.0)),
                    "manual_yaw": float((getattr(self, "manual_last", {})
                                         or {}).get("yaw", 0.0)),
                    "rc_ok": bool((getattr(self, "manual_last", {})
                                   or {}).get("rc_ok", False)),
                    # Failsafe + antrean upload: GUI baca chip/link status.
                    "failsafe_active": bool(getattr(self, "failsafe_active",
                                                   False)),
                    "failsafe_reason": str(getattr(self, "failsafe_reason",
                                                  "") or ""),
                    "upload_pending": int(getattr(getattr(
                        self, "upload_worker", None), "pending", 0) or 0),
                }
                data_packet.update(self._battery_fields())
                data_signal.emit(data_packet)
                deadline = pace_to_fps(deadline)

        finally:
            self._cleanup()

    def stop(self):
        log.info("Navigator backend menerima sinyal stop...")
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
                log.info("PRE-TURN: Navigating to WP #%s bearing (%s deg) at %sm from current WP (Radius: %sm).", next_wp_index, math.degrees(bearing_ke_wp), jarak_ke_wp_saat_ini, PRE_TURN_RADIUS_M)
        
        cross_track_dist = self._get_cross_track_distance(self.current_lat, self.current_lon, start_lat, start_lon, target_wp['lat'], target_wp['lon'])

        target_yaw_angle_rad = bearing_ke_wp
        print_status = "WAYPOINT (Default)"
        
        is_vision_leg = self.current_waypoint_index in self.config.VISION_ENABLED_LEGS
        is_on_track = abs(cross_track_dist) < self.config.GEOFENCE_WIDTH_METERS

        if is_vision_leg and not is_pre_turning:
            if is_on_track:
                if best_gate:
                    raw_correction_rad = self._calculate_yaw_correction_gate(best_gate, gate_distance)
                    # dt loop frame ini (bukan ukur baru): EKF & PID di bawah
                    # memakai potongan waktu yang sama dengan manual_link.
                    dt = self._loop_dt_now()

                    # EKF heading (hitung di C): prediksi dengan laju yaw
                    # (gyro pendek) lalu koreksi heading terukur — heading
                    # estimasi lebih halus dari pengukuran mentah.
                    gyro_rate = 0.0
                    if getattr(self, '_last_yaw_meas', None) is not None:
                        gyro_rate = self._normalize_angle(
                            self.current_yaw_rad - self._last_yaw_meas) / dt
                    self._last_yaw_meas = self.current_yaw_rad
                    self.ekf_heading.predict(gyro_rate, dt)
                    heading_est = self.ekf_heading.update(self.current_yaw_rad or 0.0)

                    # Complementary filter (hitung di C): extrapolasi heading
                    # dari gyro lalu fusi dengan (heading estimasi + koreksi
                    # visi) — halus & tak hanyut.
                    raw_target = self._normalize_angle(heading_est + raw_correction_rad)
                    target_yaw_angle_rad = self._normalize_angle(core.complementary_filter(
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
        """Tarik pesan MAVLink non-blokir + cap waktu monotonic.

        Cap (`_last_telem_mono`) diperbarui tiap ATTITUDE / GLOBAL_POSITION
        tiba — dipakai failsafe stale-link. SYS_STATUS tak dihitung (bisa
        jarang dikirim firmware).
        """
        msg = self.master.recv_match(type=['ATTITUDE', 'GLOBAL_POSITION_INT', 'VFR_HUD', 'SYS_STATUS'], blocking=False)
        while msg:
            mtype = msg.get_type()
            if mtype in ('ATTITUDE', 'GLOBAL_POSITION_INT'):
                self._last_telem_mono = time.monotonic()
            match mtype:
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

    # --- Deteksi YOLO ASYNC (loop tak menunggu inferensi) ---
    # Pola: tiap frame (kecuali frame-skip) submit 1 job per model AKTIF ke
    # YoloAsyncWorker; hasil dibaca via latest() bila segar (<=
    # YOLO_RESULT_MAX_AGE_S), kalau tidak pakai cache terakhir. Filter
    # warna/bentuk/conf adaptif (validate_buoy) tetap di sini supaya perilaku
    # identik dengan jalur sinkron lama — hanya inferensi yang pindah thread.

    # ID model untuk worker async (string stabil, bukan id(objek)).
    _MID_GATE = "gate"
    _MID_BOX = "box"
    _MID_BLUE = "blue"
    _MID_RED = "red"

    def _request_detections(self, frame):
        """Submit job inferensi untuk model yang AKTIF di state saat ini.

        Dipanggil tiap frame; frame-skip (YOLO_FRAME_SKIP) mengatur seberapa
        sering job diserahkan. Model None (gagal dimuat) dilewati.
        """
        self._frame_seq += 1
        if self._frame_seq % (self.config.YOLO_FRAME_SKIP + 1) != 0:
            return
        st = self.current_state
        want = set()
        if st == "WAYPOINT_NAV":
            want.add((self._MID_GATE, self.gate_model,
                      self.config.BUOY_CONF_SMALL_THRESHOLD))
        elif st in ("APPROACH_BOX_SEARCH", "APPROACH_BOX_ALIGN"):
            want.add((self._MID_BOX, self.box_model, 0.25))
        elif st in ("APPROACH_BLUE_BOX_SEARCH", "APPROACH_BLUE_BOX_ALIGN"):
            want.add((self._MID_BLUE, self.blue_box_model, 0.25))
        elif st in ("APPROACH_RED_BOX_SEARCH", "APPROACH_RED_BOX_ALIGN"):
            want.add((self._MID_RED, self.red_dock_model, 0.25))
        for mid, model, conf in want:
            if model is not None:
                self.yolo_worker.submit(
                    mid, model, frame, conf,
                    self.config.YOLO_INFERENCE_SIZE,
                    self.config.YOLO_HALF_PRECISION,
                    self.config.YOLO_DEVICE)

    def _take_latest(self, model_id):
        """Hasil mentah worker: dict {cls: [...]} atau {} bila belum/basi.

        Basi (> YOLO_RESULT_MAX_AGE_S) -> cache terakhir model itu; belum
        pernah ada -> {} (caller berlaku seperti "tak ada deteksi").
        """
        max_age = getattr(self.config, "YOLO_RESULT_MAX_AGE_S", 0.5)
        raw, _age = self.yolo_worker.latest(model_id, max_age_s=max_age)
        if raw is None:
            return dict(self._det_cache.get(model_id, {}))
        if raw:
            self._det_cache[model_id] = raw
            return raw
        return dict(self._det_cache.get(model_id, {}))

    def _parse_detections(self, raw, validate_gate=False):
        """Hasil mentah -> format navigator {cls: [{cx,cy,box,area,conf}]}.

        `validate_gate=True` (model gate): terapkan ambang conf adaptif +
        validate_buoy (warna/bentuk) — SATU panggilan per box (debug=True
        pun dipakai untuk keputusan, tanpa komputasi ganda).

        P4-D: ambang conf/warna/saturasi PER-CLASS (hijau lebih longgar
        karena tenggelam duluan di gelap); fallback ke ambang global bila
        key baru belum ada di config lama.
        P4-A: kecerahan frame diukur 1x (`_last_brightness`, via
        frame_brightness) lalu ambang HSV dilonggarkan otomatis saat
        gelap — lihat app/detection_validation.py.
        """
        detections = {}
        is_debug = bool(getattr(self.config, "DETECTION_DEBUG", False))
        # Kecerahan 1x per panggilan (bukan per box): 64x64 thumbnail.
        bright = None
        if validate_gate and getattr(self.config, "BUOY_ADAPTIVE_ENABLED",
                                     False):
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
                if validate_gate and cls in (
                        self.config.RED_BALL_CLASS_ID,
                        self.config.GREEN_BALL_CLASS_ID):
                    is_green = (cls == self.config.GREEN_BALL_CLASS_ID)
                    # Ambang conf per-class + box kecil (jauh) lebih longgar.
                    if is_green:
                        conf_big = getattr(
                            self.config, "BUOY_CONF_THRESHOLD_GREEN",
                            self.config.BUOY_CONF_THRESHOLD)
                        conf_small = getattr(
                            self.config, "BUOY_CONF_SMALL_THRESHOLD_GREEN",
                            self.config.BUOY_CONF_SMALL_THRESHOLD)
                    else:
                        conf_big = getattr(
                            self.config, "BUOY_CONF_THRESHOLD_RED",
                            self.config.BUOY_CONF_THRESHOLD)
                        conf_small = getattr(
                            self.config, "BUOY_CONF_SMALL_THRESHOLD_RED",
                            self.config.BUOY_CONF_SMALL_THRESHOLD)
                    conf_thresh = (conf_small
                                   if area < self.config.BUOY_SMALL_AREA_PX
                                   else conf_big)
                    if conf_val < conf_thresh:
                        continue
                    if is_green:
                        frac_cfg = getattr(
                            self.config, "BUOY_MIN_COLOR_FRACTION_GREEN",
                            self.config.BUOY_MIN_COLOR_FRACTION)
                        sat_cfg = getattr(
                            self.config, "BUOY_MIN_SATURATION_GREEN",
                            self.config.BUOY_MIN_SATURATION)
                    else:
                        frac_cfg = getattr(
                            self.config, "BUOY_MIN_COLOR_FRACTION_RED",
                            self.config.BUOY_MIN_COLOR_FRACTION)
                        sat_cfg = getattr(
                            self.config, "BUOY_MIN_SATURATION_RED",
                            self.config.BUOY_MIN_SATURATION)
                    ok, reasons = validate_buoy(
                        self._last_frame, cls, (x1, y1, x2, y2),
                        min_area=self.config.MIN_BUOY_AREA_PX,
                        min_color_fraction=frac_cfg,
                        min_saturation=sat_cfg,
                        max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION,
                        debug=True,
                        brightness=bright,
                        adaptive_enabled=getattr(
                            self.config, "BUOY_ADAPTIVE_ENABLED", False),
                        brightness_threshold=getattr(
                            self.config, "BUOY_BRIGHTNESS_THRESHOLD", 80),
                        dark_saturation=getattr(
                            self.config, "BUOY_ADAPTIVE_MIN_SATURATION",
                            0.20),
                        color_fraction_mult=getattr(
                            self.config, "BUOY_ADAPTIVE_COLOR_FRACTION_MULT",
                            0.5))
                    if not ok:
                        if is_debug and throttled("detect_dbg", 1.0):
                            log.info("[DETECT DEBUG] buoy cls=%s area=%s "
                                     "ditolak: %s", cls, area,
                                     "; ".join(reasons))
                        continue
                ent = {"cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2,
                       "box": (x1, y1, x2, y2), "area": area,
                       "conf": conf_val}
                detections.setdefault(cls, []).append(ent)
        return detections

    def _detect_objects(self, frame, model_to_use, force_run=False):
        """Kompatibilitas: deteksi sinkron dari cache async + frame terbaru.

        `model_to_use` dipetakan ke ID worker (gate/box/blue/red); `frame`
        disimpan sebagai `_last_frame` untuk validate_buoy. `force_run`
        diabaikan (kesegaran diatur YOLO_RESULT_MAX_AGE_S) — dipertahankan
        agar call-site lama tak perlu diubah serentak.
        """
        self._last_frame = frame
        if model_to_use is self.gate_model:
            return self._parse_detections(
                self._take_latest(self._MID_GATE), validate_gate=True)
        if model_to_use is self.box_model:
            return self._parse_detections(self._take_latest(self._MID_BOX))
        if model_to_use is self.blue_box_model:
            return self._parse_detections(self._take_latest(self._MID_BLUE))
        if model_to_use is self.red_dock_model:
            return self._parse_detections(self._take_latest(self._MID_RED))
        return {}

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
        # Rumus di C (gv_yaw_correction): 0.0 bila tak reliabel.
        raw_correction_rad = core.yaw_correction_c(
            best_box['cx'], self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        self.last_used_p_gain = self.config.VISION_P_GAIN

        # PID + deadband (pengganti raw * VISION_P_GAIN).
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN,
                                   self._loop_dt_now())

    def _take_photo(self, frame):
        if not self.config.SAVE_GREEN_BOX_PHOTO: return
        try:
            if not os.path.exists(cfg.CAPTURES_DIR): os.makedirs(cfg.CAPTURES_DIR)
            # Nama file unik dengan timestamp
            filename_short = f"green_box_{int(time.time())}.jpg"
            filename_full = os.path.join(cfg.CAPTURES_DIR, filename_short)
            
            success = cv2.imwrite(filename_full, frame)
            if success:
                log.info("--- Foto Box Hijau disimpan: %s ---", filename_full)
                # Trigger Upload di Thread terpisah
                self.upload_worker.enqueue(filename_full, filename_short)
            else:
                log.info("Gagal menyimpan foto ke %s", filename_full)
        except Exception as e:
            log.info("Gagal menyimpan foto: %s", e)

    # --- [MODIFIKASI 2: Update _take_waypoint_photo dengan Settingan Terang] ---
    def _take_waypoint_photo(self):
        log.info("Membuka Kamera WP (Index %s)...", self.config.WAYPOINT_PHOTO_CAMERA_INDEX)
        cap = None 
        try:
            cam_idx = self.config.WAYPOINT_PHOTO_CAMERA_INDEX
            cap = open_camera(cam_idx, 640, 480) 
            if cap is None:
                log.info("ERROR: Gagal membuka kamera foto waypoint di indeks %s.", cam_idx)
                return
            
            # == SETTINGAN OBAT KUAT ==
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # Force Brightness
            
            # Warmup 20 frame (Wajib)
            for _ in range(20): cap.read()
            
            ret, frame = cap.read()
            if not ret:
                log.info("ERROR: Gagal mengambil frame WP.")
                cap.release(); return
                
            log.info("Frame foto WP berhasil diambil.")
            cap.release()

            # Simpan & Upload
            dir_name = self.config.WAYPOINT_PHOTO_DIR
            if not os.path.exists(dir_name): os.makedirs(dir_name)
            
            filename_short = f"wp_photo_WP{self.current_waypoint_index}_{int(time.time())}.jpg"
            filename_full = os.path.join(dir_name, filename_short)
            
            success = cv2.imwrite(filename_full, frame)
            if success:
                log.info("--- Foto Waypoint disimpan: %s ---", filename_full)
                self.upload_worker.enqueue(filename_full, filename_short)
        except Exception as e:
            log.info("ERROR saat akses kamera foto: %s", e)
            if cap and cap.isOpened(): cap.release()

    def _control_dt(self):
        """Selang waktu loop nyata (detik) untuk PID/filter.

        dt diukur dari `self._last_loop_time` (di-set di __init__), bukan
        tebakan — koreksi tetap benar walau loop melambat. Hasilnya juga
        disimpan di `self._loop_dt` agar pemanggil dalam frame yang sama
        (manual_link, PID, EKF) memakai dt yang SAMA, bukan potongan baru.
        """
        now = time.time()
        dt = now - self._last_loop_time
        self._last_loop_time = now
        self._loop_dt = max(dt, 1e-4)
        return self._loop_dt

    def _loop_dt_now(self):
        """dt loop frame ini (tanpa mengukur ulang)."""
        return getattr(self, "_loop_dt", 0.033)

    def _track_gate(self, detections):
        """Pilih target gate aktif lewat sequencer C (titik tengah).

        Menggantikan versi per-frame lama (`_find_best_gate`, sudah dihapus):
        sequencer C mengumpulkan
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
            # Koleksi pasangan di C (gv_collect_pairs via aterkia_core).
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
            # Target aktif sudah dilewati / tidak ada pasangan valid lagi —
            # jangan koreksi arah; sequencer sudah menyiapkan gate berikutnya.
            return None, float('inf')
        return pair, dist

    def _apply_yaw_pid(self, raw_correction_rad, dynamic_gain, dt=None):
        """PID + deadband untuk koreksi yaw (hitung di C via aterkia_core).

        Pengganti pola lama `raw * gain`. Gain P tetap bisa dinamis
        (fuzzy / VISION_P_GAIN) — di-set ulang tiap panggilan; term
        integral & derivatif memakai dt loop frame ini (`_loop_dt_now`)
        supaya perilaku tidak berubah saat fps turun.
        """
        self.pid_gate.kp = dynamic_gain
        return self.pid_gate.update(raw_correction_rad,
                                    self._loop_dt_now() if dt is None else dt)

    def _calculate_yaw_correction_gate(self, best_gate, distance_m):
        if not isinstance(best_gate, (list, tuple)) or len(best_gate) != 2: return 0.0
        r_ball, g_ball = best_gate
        if not isinstance(r_ball, dict) or not isinstance(g_ball, dict) or 'cx' not in r_ball or 'cx' not in g_ball: return 0.0

        midpoint_x = (r_ball['cx'] + g_ball['cx']) / 2.0
        # Rumus di C (gv_yaw_correction): 0.0 bila tak reliabel.
        raw_correction_rad = core.yaw_correction_c(
            midpoint_x, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        dynamic_p_gain = self.config.VISION_P_GAIN

        if FUZZY_ENABLED:
            # Sugeno singleton (hitung di C via aterkia_core). Input diklem
            # ke semesta 0..2 m; error_px dipakai abs langsung (< 320 piksel
            # karena offset dari pusat frame ±160 px).
            dynamic_p_gain = core.fuzzy_gate_gain(
                min(max(distance_m, 0.0), 2.0), abs(midpoint_x - self.image_center_x))

        self.last_used_p_gain = dynamic_p_gain
        # PID + deadband (bukan raw * gain): lihat _apply_yaw_pid.
        return self._apply_yaw_pid(raw_correction_rad, dynamic_p_gain)

    def _calculate_yaw_correction_blue_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box: return 0.0
        box_center_x_px = best_box['cx']
        # Offset lateral (m) -> piksel; target di C seperti gate.
        try:
            if distance_m < 0.1 or self.config.FOCAL_LENGTH_PX == 0: return 0.0
            offset_in_pixels = (self.config.BLUE_BOX_LATERAL_OFFSET_M * self.config.FOCAL_LENGTH_PX) / distance_m
            target_x_in_frame = box_center_x_px + offset_in_pixels
        except ZeroDivisionError:
            return 0.0

        raw_correction_rad = core.yaw_correction_c(
            target_x_in_frame, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        self.last_used_p_gain = self.config.VISION_P_GAIN
        # PID + deadband (pengganti raw * VISION_P_GAIN).
        return self._apply_yaw_pid(raw_correction_rad, self.config.VISION_P_GAIN)

    def _calculate_yaw_correction_red_box(self, best_box, distance_m):
        if not isinstance(best_box, dict) or 'cx' not in best_box: return 0.0

        midpoint_x = best_box['cx']
        # Rumus di C (gv_yaw_correction): 0.0 bila tak reliabel.
        raw_correction_rad = core.yaw_correction_c(
            midpoint_x, self.image_center_x, distance_m,
            self.config.FOCAL_LENGTH_PX)
        dynamic_p_gain = self.config.VISION_P_GAIN

        if FUZZY_ENABLED:
            # Sugeno singleton (hitung di C via aterkia_core). Input diklem
            # ke semesta 0..10 m; error_px dipakai abs langsung.
            dynamic_p_gain = core.fuzzy_docking_gain(
                min(max(distance_m, 0.0), 10.0),
                abs(midpoint_x - self.image_center_x))

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
                # Jalur tengah gate DIPERKUAT (tanpa garis vertikal penuh —
                # kamera tetap jernih): lingkaran isi + ring putih + label.
                cv2.circle(frame, (midpoint_x, midpoint_y), 10, (0, 255, 255), -1)
                cv2.circle(frame, (midpoint_x, midpoint_y), 13, (255, 255, 255), 2)
                cv2.putText(frame, "TENGAH", (midpoint_x + 16, max(12, midpoint_y - 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

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
        """Kompatibilitas: arahkan ke antrean UploadWorker (P1).

        Dipertahankan agar kode lama/simulator yang memanggil langsung tetap
        jalan — TIDAK spawn thread baru, hanya enqueue.
        """
        worker = getattr(self, "upload_worker", None)
        if worker is None:
            log.warning("UPLOAD: worker belum siap, %s dibuang.", filename)
            return
        if not worker.enqueue(file_path, filename):
            log.warning("UPLOAD: antrean penuh/bermasalah, %s dibuang.",
                        filename)

    def _start_video_recording(self):
        if not self.config.ENABLE_SESSION_RECORDING:
            log.info("--- [Config] ENABLE_SESSION_RECORDING di-set False. Perekaman video sesi NONAKTIF. ---")
            self.video_writer = None
            return
        try:
            dir_name = self.config.SESSION_VIDEO_DIR
            if not os.path.exists(dir_name): os.makedirs(dir_name)
            filename = os.path.join(dir_name, f"session_record_{int(time.time())}.avi")
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            frame_size = (self.processing_width, self.processing_height)
            self.video_writer = cv2.VideoWriter(filename, fourcc, self.config.SESSION_VIDEO_FPS, frame_size)
            log.info("\n--- [Perekam Sesi] Mulai merekam ke: %s ---", filename)
        except Exception as e:
            log.info("PERINGATAN: Gagal memulai perekam video sesi: %s", e)
            self.video_writer = None

    def _stop_video_recording(self):
        if self.video_writer is not None:
            log.info("--- [Perekam Sesi] Berhenti merekam. Menyimpan file... ---")
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
                log.info("Error sending attitude target: %s", e)

    def _stream_offboard_command(self, thrust, target_yaw_rad, status="N/A", force_send=False, lateral_thrust=0.0): 
        current_time = time.time()
        time_since_last_stream = current_time - self.last_stream_time
        stream_interval = 1.0 / self.config.OFFBOARD_STREAM_RATE_HZ if self.config.OFFBOARD_STREAM_RATE_HZ > 0 else float('inf')
        if force_send or time_since_last_stream >= stream_interval:
            if not isinstance(target_yaw_rad, (int, float)) or math.isnan(target_yaw_rad):
                log.info("Invalid target_yaw_rad: %s, using current yaw.", target_yaw_rad)
                target_yaw_rad = self.current_yaw_rad or 0.0 
            self._set_attitude_target(thrust, target_yaw_rad, lateral_thrust)
            self.last_stream_time = current_time
            pass

    def _prepare_for_offboard(self):
        log.info("Mengirim stream awal (3 detik)...")
        start_time = time.time()
        while time.time() - start_time < 3.0:
            self._set_attitude_target(0.0, 0)
            sleep_duration = max(0, (1.0 / self.config.OFFBOARD_STREAM_RATE_HZ) - 0.001)
            time.sleep(sleep_duration) 
        log.info("SISTEM SIAP. Silakan ARMING dan ganti ke mode OFFBOARD.")

    def _cleanup(self):
        log.info("\nMembersihkan resource...")
        self._stop_video_recording()
        # Hentikan worker async dulu (YOLO + uploader) agar tak ada thread
        # nyangkut saat kamera/MAVLink ditutup.
        for _worker in (getattr(self, "yolo_worker", None),
                        getattr(self, "upload_worker", None)):
            try:
                if _worker is not None:
                    _worker.stop_and_wait()
            except Exception:
                pass
        # Hentikan thread pendengar Redis/sinyal (jangan bocor antar sesi).
        for _ev, _thr in ((getattr(self, "command_stop_event", None),
                           getattr(self, "command_thread", None)),
                          (getattr(self, "signal_stop_event", None),
                           getattr(self, "signal_thread", None))):
            try:
                if _ev is not None:
                    _ev.set()
                if _thr is not None and _thr.is_alive():
                    _thr.join(timeout=1.0)
            except Exception:
                pass
        if hasattr(self, 'redis_publish_thread') and self.redis_publish_thread.is_alive():
            log.info("Menghentikan thread publisher Redis...")
            self.redis_stop_event.set()
            self.redis_publish_thread.join(timeout=1.0) 
        if hasattr(self, 'master') and self.master:
            log.info("Mengirim perintah berhenti...")
            for _ in range(10): 
                current_yaw = self.current_yaw_rad or 0.0
                self._set_attitude_target(0.0, current_yaw, 0.0) 
                time.sleep(1.0 / self.config.OFFBOARD_STREAM_RATE_HZ + 0.01)
            try:
                self.master.close()
            except Exception as e:
                log.info("Error closing MAVLink connection: %s", e)
        if hasattr(self, 'cap') and self.cap and self.cap.isOpened():
            self.cap.release()
            log.info("Kamera utama dilepaskan.")
        if hasattr(self, 'wp_photo_cap') and self.wp_photo_cap and self.wp_photo_cap.isOpened():
            self.wp_photo_cap.release()
            log.info("Kamera foto waypoint dilepaskan.")
        if hasattr(self, 'redis_client') and self.redis_client:
            log.info("Menutup koneksi Redis...")
            self.redis_client.close()
        log.info("Selesai.")

    def _normalize_angle(self, angle_rad):
        """Bawa sudut ke [-pi, pi]. Hitung di C (nav_math) via aterkia_core."""
        return core.nav_normalize_angle(angle_rad)

    def _get_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        """Jarak (m) & bearing (rad) antar koordinat — hitung di C."""
        return core.nav_distance_bearing(lat1, lon1, lat2, lon2)

    def _get_cross_track_distance(self, lat_p, lon_p, lat_wp1, lon_wp1, lat_wp2, lon_wp2):
        """Simpangan titik P dari garis WP1->WP2 (m) — hitung di C."""
        return core.nav_cross_track_distance(lat_p, lon_p, lat_wp1, lon_wp1,
                                             lat_wp2, lon_wp2)

    def _yaw_to_quaternion(self, yaw_rad):
        # (Fungsi _yaw_to_quaternion tidak berubah)
        if not isinstance(yaw_rad, (int, float)) or math.isnan(yaw_rad):
            log.info("Invalid yaw_rad for quaternion: %s, using 0.0", yaw_rad)
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

    # ID mode RTL ArduPilot Rover (lihat mode_mapping_rover pymavlink).
    _ROVER_RTL_MODE = 11
    # Jeda minimal antar percobaan kirim RTL (detik, monotonic).
    _RTL_RETRY_INTERVAL_S = 5.0

    def _request_rtl(self):
        """Minta mode RTL via MAV_CMD_DO_SET_MODE (best-effort).

        Pixhawk TETAP pemegang failsafe utama (RCIN + geofence bawaan);
        perintah ini hanya usaha tambahan dari NUC, di-throttle maks 1x per
        5 detik agar tak membanjiri link MAVLink yang sedang bermasalah.
        """
        now = time.monotonic()
        if now - self._rtl_sent_mono < self._RTL_RETRY_INTERVAL_S:
            return
        self._rtl_sent_mono = now
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self._ROVER_RTL_MODE, 0, 0, 0, 0, 0)
            log.warning("FAILSAFE: perintah RTL dikirim (best-effort).")
        except Exception as e:
            if throttled("rtl_fail", 10.0):
                log.warning("FAILSAFE: gagal kirim perintah RTL: %s", e)

    def _check_failsafe(self):
        """Evaluasi failsafe tiap frame sebelum stream OFFBOARD.

        Return "" bila aman (loop lanjut normal), atau string alasan bila
        aktif — caller MENIMPA thrust=0 & yaw=current lalu label status
        "FAILSAFE (alasan)". Kondisi pemicu (ambang di Config, tab
        Failsafe di Settings):
          1. stale-link: tak ada ATTITUDE/GLOBAL_POSITION > batas detik;
          2. low-batt: persen ATAU tegangan di bawah ambang, DITAHAN selama
             hold agar spike sesaat (arus dud) tak memicu RTL palsu.
        Saat aktif: thrust 0 + coba RTL via _request_rtl (best-effort).
        Pulih otomatis bila kondisi normal kembali (log 1x).
        """
        if not getattr(self.config, "FAILSAFE_ENABLED", True):
            if self.failsafe_active:
                self.failsafe_active = False
                self.failsafe_reason = ""
            return ""
        now = time.monotonic()
        reason = ""
        # 1. Stale-link (_last_telem_mono dicap di _update_telemetry).
        timeout = float(getattr(self.config, "FAILSAFE_TELEM_TIMEOUT_S", 2.0))
        last = float(getattr(self, "_last_telem_mono", 0.0) or 0.0)
        if last > 0.0 and (now - last) > timeout:
            reason = f"telemetri basi {now - last:.1f}s"
        # 2. Baterai rendah (ditahan).
        if not reason:
            fields = self._battery_fields()
            pct = fields.get("battery_pct")
            volt = fields.get("voltage_v")
            low_pct = float(getattr(self.config, "FAILSAFE_LOW_BATT_PCT",
                                    20.0))
            low_v = float(getattr(self.config, "FAILSAFE_LOW_VOLT_V", 13.2))
            hold = float(getattr(self.config, "FAILSAFE_LOW_BATT_HOLD_S",
                                 3.0))
            low = ((pct is not None and pct < low_pct)
                   or (volt is not None and volt < low_v))
            if low:
                if self._lowbatt_since is None:
                    self._lowbatt_since = now
                elif now - self._lowbatt_since >= hold:
                    detail = (f"{pct:.0f}%" if pct is not None
                              else f"{volt:.1f}V")
                    reason = f"baterai rendah {detail}"
            else:
                self._lowbatt_since = None
        if reason:
            if not self.failsafe_active:
                log.warning("FAILSAFE AKTIF: %s — thrust 0 + coba RTL.",
                            reason)
            self.failsafe_active = True
            self.failsafe_reason = reason
            self._request_rtl()
            return reason
        if self.failsafe_active:
            log.info("FAILSAFE pulih: kondisi normal kembali.")
        self.failsafe_active = False
        self.failsafe_reason = ""
        return ""

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
            log.info("PERINGATAN: Gagal publish telemetri ke Redis: %s", e)
    
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
            log.info("Penyimpanan Gagal: Objek config belum ada.")
            return
        log.info("Menyimpan parameter tuning ke %s...", TUNING_FILE)
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
            'BUOY_CONF_THRESHOLD_GREEN': self.config.BUOY_CONF_THRESHOLD_GREEN,
            'BUOY_CONF_SMALL_THRESHOLD_GREEN':
                self.config.BUOY_CONF_SMALL_THRESHOLD_GREEN,
            'BUOY_CONF_THRESHOLD_RED': self.config.BUOY_CONF_THRESHOLD_RED,
            'BUOY_CONF_SMALL_THRESHOLD_RED':
                self.config.BUOY_CONF_SMALL_THRESHOLD_RED,
            'BUOY_MIN_COLOR_FRACTION_GREEN':
                self.config.BUOY_MIN_COLOR_FRACTION_GREEN,
            'BUOY_MIN_COLOR_FRACTION_RED':
                self.config.BUOY_MIN_COLOR_FRACTION_RED,
            'BUOY_MIN_SATURATION_GREEN':
                self.config.BUOY_MIN_SATURATION_GREEN,
            'BUOY_MIN_SATURATION_RED':
                self.config.BUOY_MIN_SATURATION_RED,
            'BUOY_ADAPTIVE_ENABLED': self.config.BUOY_ADAPTIVE_ENABLED,
            'BUOY_BRIGHTNESS_THRESHOLD':
                self.config.BUOY_BRIGHTNESS_THRESHOLD,
            'BUOY_ADAPTIVE_MIN_SATURATION':
                self.config.BUOY_ADAPTIVE_MIN_SATURATION,
            'BUOY_ADAPTIVE_MIN_VALUE':
                self.config.BUOY_ADAPTIVE_MIN_VALUE,
            'BUOY_ADAPTIVE_COLOR_FRACTION_MULT':
                self.config.BUOY_ADAPTIVE_COLOR_FRACTION_MULT,
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
            'YOLO_RESULT_MAX_AGE_S': getattr(
                self.config, 'YOLO_RESULT_MAX_AGE_S', 0.5),
            'UPLOAD_QUEUE_SIZE': getattr(self.config, 'UPLOAD_QUEUE_SIZE', 3),
            'UPLOAD_MAX_RETRIES': getattr(
                self.config, 'UPLOAD_MAX_RETRIES', 1),
            'FAILSAFE_ENABLED': getattr(self.config, 'FAILSAFE_ENABLED',
                                       True),
            'FAILSAFE_TELEM_TIMEOUT_S': getattr(
                self.config, 'FAILSAFE_TELEM_TIMEOUT_S', 2.0),
            'FAILSAFE_LOW_BATT_PCT': getattr(
                self.config, 'FAILSAFE_LOW_BATT_PCT', 20.0),
            'FAILSAFE_LOW_VOLT_V': getattr(
                self.config, 'FAILSAFE_LOW_VOLT_V', 13.2),
            'FAILSAFE_LOW_BATT_HOLD_S': getattr(
                self.config, 'FAILSAFE_LOW_BATT_HOLD_S', 3.0),
            'SESSION_VIDEO_FPS': self.config.SESSION_VIDEO_FPS,
            'CAMERA_FLIP_MODE': self.config.CAMERA_FLIP_MODE,
            'MANUAL_ENABLED': self.config.MANUAL_ENABLED,
            'MANUAL_MAX_SURGE': self.config.MANUAL_MAX_SURGE,
            'MANUAL_MAX_YAW': self.config.MANUAL_MAX_YAW,
            'MANUAL_DEADBAND': self.config.MANUAL_DEADBAND,
            'MANUAL_EXPO': self.config.MANUAL_EXPO,
            'MANUAL_RATE_LIMIT': self.config.MANUAL_RATE_LIMIT,
            'RC_TIMEOUT_MS': self.config.RC_TIMEOUT_MS,
            'RC_CH_THROTTLE': self.config.RC_CH_THROTTLE,
            'RC_CH_YAW': self.config.RC_CH_YAW,
            'RC_CH_MODE': self.config.RC_CH_MODE,
            'RC_CH_DEADMAN': self.config.RC_CH_DEADMAN,
            'MANUAL_LOST_HOLD_S': self.config.MANUAL_LOST_HOLD_S,
            'VISION_ENABLED_LEGS': self.config.VISION_ENABLED_LEGS,
            'PHOTO_BOX_LEGS': self.config.PHOTO_BOX_LEGS,
            'BLUE_BOX_PHOTO_LEGS': self.config.BLUE_BOX_PHOTO_LEGS,
        }
        try:
            with open(TUNING_FILE, 'w') as f:
                json.dump(params_to_save, f, indent=4)
            log.info("Parameter berhasil disimpan.")
        except Exception as e:
            log.info("ERROR: Gagal menyimpan parameter tuning: %s", e)

    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            log.info("[TUNING] %s updated to %s", key, value)
        else:
            log.info("[ERROR] Config key '%s' not found!", key)
        
    def run(self):
        log.info("Starting NavigatorThread (ASLI DENGAN PX4 & REDIS)...")
        try:
            self.navigator.run(self.newData) 
        except Exception as e:
            log.info("FATAL ERROR IN NAVIGATOR THREAD: %s", e)
            import traceback
            traceback.print_exc()
        log.info("NavigatorThread finished.")

    def stop(self):
        if self.navigator:
            self.navigator.stop()
            
    def update_waypoints(self, new_waypoints):
        global WAYPOINTS
        WAYPOINTS = new_waypoints
        if not self.navigator: return
        log.info("\n--- [RESET MISSION] Perintah Save/Reload diterima ---")
        self.navigator.waypoints = new_waypoints
        if not new_waypoints or len(new_waypoints) == 0:
            self.navigator.current_waypoint_index = 0
            self.navigator._set_state_and_publish("NO_WAYPOINTS")
            log.info("[RESET MISSION] Tidak ada waypoint baru. Berhenti (IDLE).")
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
            log.info("[RESET MISSION] Misi direset. Menuju ke WP 0 baru.")
        log.info("Navigator waypoints updated (HARD RESET).")
