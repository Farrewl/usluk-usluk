"""
app/simulator.py — Simulator untuk menguji GUI.

Dua mode eksperimen yang TIDAK perlu kapal asli:
  1. GroundSimNavigator  — kamera ASLI + model YOLO asli, posisi GPS di-mock.
                           Bila Pixhawk terhubung (USB/MAVLink), roll/pitch/yaw
                           ikut IMU NYATA (lihat app/mavlink_telemetry.py).
                           (dulu: modules/simulation_in_ground.py)
  2. MockSimNavigator    — tanpa kamera: objek vision & fisika dibuat sintetis.
                           (dulu: modules/simulator_backend.py)

Keduanya dibungkus QThread agar tidak membekukan GUI (PyQt):
  - NavigatorThread     -> ground sim (kamera asli, default di main.py)
  - MockNavigatorThread -> mock penuh (paling cepat untuk uji logika GUI)

Kontrak antarmuka thread (wajib ada, dipakai panel tuning main.py):
  update_config_param(key, value)   : ganti 1 parameter config saat runtime
  save_config_to_file()             : simpan parameter ke config/tuning_params.json
  update_waypoints(list)            : reset misi dengan daftar waypoint baru
  stop()                            : hentikan loop dengan aman
"""

import math
import os
import json
import time
import random
import queue
import threading

from PyQt5.QtCore import QThread, pyqtSignal

from . import geo
from . import settings as cfg

try:
    import numpy as np
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False

try:
    from .camera import open_camera, make_fallback_frame
    CAMERA_HELPER = True
except ImportError:
    CAMERA_HELPER = False

from . import mavlink_telemetry as mavlink_mod

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False

try:
    import skfuzzy as fuzz
    from skfuzzy import control as ctrl
    FUZZY_ENABLED = True
except ImportError:
    FUZZY_ENABLED = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False


# ---------------------------------------------------------------------------
# _YoloWorker — inferensi YOLO paralel (video loop tak pernah menunggu)
# ---------------------------------------------------------------------------
class _YoloWorker(QThread):
    """Jalankan inferensi YOLO di thread terpisah.

    Loop video menyerahkan frame ke worker lalu langsung melanjutkan
    (tidak menunggu). Hasil terbaru per model diambil lewat `latest()`.
    Dengan begitu, biarpun inferensi lambat di CPU, capture & tampilan
    kamera tetap stabil di ~30 fps (frame-skip hanya mengatur seberapa
    sering frame diserahkan ke worker).
    """

    def __init__(self, config, parent=None):
        super().__init__(parent)
        self.config = config
        self._jobs = queue.Queue(maxsize=2)  # job lama dibuang, yang baru penting
        self._latest = {}                    # id(model) -> (detected, annotated)
        self._gate_model = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()

    def bind_gate_model(self, model):
        """Tandai model gate: hasilnya butuh >= 2 objek (2 buoy)."""
        self._gate_model = model

    def submit(self, model, frame, conf):
        """Serahkan frame untuk di-infer. Antrean lama di-geser bila penuh."""
        if model is None:
            return
        try:
            self._jobs.get_nowait()
        except queue.Empty:
            pass
        self._jobs.put_nowait((id(model), model, frame, conf))

    def latest(self, model):
        """Hasil terbaru untuk `model`; None bila belum pernah selesai."""
        with self._lock:
            return self._latest.get(id(model))

    def run(self):
        while not self._stop_event.is_set():
            try:
                mid, model, frame, conf = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                results = model(frame, verbose=False, conf=conf,
                                imgsz=self.config.YOLO_INFERENCE_SIZE)
                annotated = results[0].plot()
                detected = len(results[0].boxes) > 0
                if model is self._gate_model and self._gate_model is not None:
                    detected = len(results[0].boxes) >= 2  # gate butuh 2 buoy
            except Exception as exc:  # model rusak / frame tak terduga
                print(f"[YOLO-W] Inferensi gagal: {exc}")
                detected, annotated = False, frame
            with self._lock:
                self._latest[mid] = (detected, annotated)

    def stop_and_wait(self):
        """Hentikan loop worker dan tunggu sampai threadnya selesai."""
        self._stop_event.set()
        self.wait(3000)


# ---------------------------------------------------------------------------
# GroundSimNavigator — kamera asli + YOLO asli, GPS dimock
# ---------------------------------------------------------------------------
class GroundSimNavigator:
    """Menjalankan pipeline kamera-nyata + YOLO-nyata, tetapi posisi kapal
    diperbarui sintetis (mock). Berguna menguji deteksi tanpa ke laut."""

    def __init__(self, config):
        self.config = config
        self.waypoints = []
        self.current_waypoint_index = 0
        self.current_state = "IDLE"
        self.running = False

        # --- MOCK TELEMETRY ---
        self.current_lat = -7.282356
        self.current_lon = 112.794925
        self.current_yaw_rad = 0.0
        self.current_groundspeed = 0.0
        self.dist_to_wp = 0.0

        # --- KAMERA ASLI (resolusi tertinggi + fps stabil, lihat camera.py) ---
        print(f"[SIM] Membuka Kamera Utama (Index {self.config.CAMERA_INDEX})...")
        self.cap = (open_camera(self.config.CAMERA_INDEX,
                                target_fps=self.config.CAMERA_TARGET_FPS,
                                auto_highest=True)
                    if CAMERA_HELPER else None)
        if self.cap is not None:
            # Ukuran hasil negosiasi menimpa default agar frame sintetis,
            # ROI, dan tampilan video konsisten dengan kamera asli.
            real_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.config.FRAME_WIDTH = real_w
            self.config.FRAME_HEIGHT = real_h
            print(f"[CAM] Mode aktif kamera utama: {real_w}x{real_h}.")
        else:
            print("[SIM] Kamera utama TIDAK terbuka — pakai frame sintetis.")

        # --- MODEL YOLO ASLI ---
        if YOLO_AVAILABLE:
            try:
                self.gate_model = YOLO(self.config.MODEL_PATH, task='detect')
                self.box_model = YOLO(self.config.BOX_MODEL_PATH, task='detect')
                self.blue_box_model = YOLO(self.config.BLUE_BOX_MODEL_PATH, task='detect')
                self.red_dock_model = YOLO(self.config.RED_DOCK_MODEL_PATH, task='detect')
                print("[SIM] Semua model YOLO berhasil dimuat!")
            except Exception as e:
                print(f"[SIM][ERROR] Gagal memuat model YOLO: {e}")
        else:
            self.gate_model = self.box_model = None
            self.blue_box_model = self.red_dock_model = None

        # --- REDIS (opsional) ---
        self.redis_client = None
        if REDIS_AVAILABLE:
            try:
                self.redis_client = redis.Redis(host=self.config.REDIS_HOST,
                                                port=self.config.REDIS_PORT,
                                                decode_responses=True)
                self.redis_client.ping()
            except Exception:
                print("[SIM] Redis tidak bisa dihubungi — fitur publikasi dimatikan.")

        self.state_timer = time.time()
        self.sim_step = 1
        self._loop_start = time.time()
        self._fps_window = time.time()
        self._fps_count = 0
        self._yolo_subcount = 0
        self._last_detection = {}   # id(model) -> (detected, annotated)

    # ------------------- Geolokasi mock -------------------
    def _update_mock_position(self, target_lat, target_lon, speed_mps=1.5):
        """Geser posisi mock mendekati target; True jika sudah tiba (<2 m)."""
        dist, bearing = geo.distance_bearing(self.current_lat, self.current_lon,
                                             target_lat, target_lon)
        self.dist_to_wp = dist
        self.current_yaw_rad = bearing
        if dist > 2.0:
            delta = speed_mps * 0.1
            self.current_lat, self.current_lon = geo.destination(
                self.current_lat, self.current_lon, delta, bearing)
            self.current_groundspeed = speed_mps
            return False
        self.current_groundspeed = 0.5
        return True

    # ------------------- Deteksi -------------------
    def _run_yolo_detection(self, model, frame, run_now=True):
        """Inferensi YOLO ASINKRON — loop video tidak pernah menunggu.

        `run_now=True` menyerahkan frame terbaru ke `_YoloWorker`.
        Hasil yang sudah siap dipakai untuk update status; saat belum ada
        hasil baru, hasil terakhir model yang sama dipakai. Dengan ini
        capture & tampilan kamera tetap ~30 fps walaupun inferensi lambat.
        """
        if model is None or not YOLO_AVAILABLE:
            return False, frame
        if run_now:
            self._yolo_worker.submit(model, frame.copy(), 0.5)
        result = self._yolo_worker.latest(model)
        if result is None:
            return self._last_detection.get(id(model), (False, frame))
        detected, annotated = result
        self._last_detection[id(model)] = (detected, annotated)
        return detected, annotated

    # ------------------- Foto bawah air (WP 8) -------------------
    def _execute_smart_capture_procedure(self):
        """Ganti ke kamera bawah air, hangatkan 20 frame, simpan & upload foto."""
        print("\n=== [WP 8] PROSEDUR SMART PHOTO (REAL HARDWARE) ===")
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("[WP 8] Kamera navigasi dipause.")
        time.sleep(1.0)

        cam_bawah = (open_camera(self.config.WAYPOINT_PHOTO_CAMERA_INDEX, 640, 480)
                     if CAMERA_HELPER else None)
        if cam_bawah and cam_bawah.isOpened():
            cam_bawah.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)  # paksa auto-exposure ON

            for _ in range(20):
                cam_bawah.read()  # warm-up sensor

            for percobaan in range(1, 6):
                ret, frame = cam_bawah.read()
                if not ret:
                    continue
                avg = float(np.mean(frame))
                print(f"[WP 8] Percobaan {percobaan}: brightness = {avg:.2f}")
                if avg > 1.0:
                    os.makedirs(cfg.CAPTURES_DIR, exist_ok=True)
                    path = os.path.join(cfg.CAPTURES_DIR, f"WP8_RealTest_{int(time.time())}.jpg")
                    cv2.imwrite(path, frame)
                    threading.Thread(target=self._upload_snapshot_to_server,
                                     args=(path, os.path.basename(path)),
                                     daemon=True).start()
                    time.sleep(1.0)
                    break
        if cam_bawah:
            cam_bawah.release()

        # hidupkan kembali kamera navigasi (auto-negosiasi, sama seperti init)
        self.cap = (open_camera(self.config.CAMERA_INDEX,
                                target_fps=self.config.CAMERA_TARGET_FPS,
                                auto_highest=True)
                    if CAMERA_HELPER else None)
        if self.cap is not None:
            self.config.FRAME_WIDTH = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.config.FRAME_HEIGHT = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # ------------------- Redis / upload -------------------
    def _upload_snapshot_to_server(self, file_path, filename):
        """Kirim foto ke dashboard web (Laravel/Ngrok) via HTTP POST."""
        if not os.path.exists(file_path):
            print(f"[UPLOAD] Gagal: {filename} tidak ditemukan.")
            return
        try:
            import requests
            with open(file_path, 'rb') as f:
                resp = requests.post(self.config.SERVER_UPLOAD_URL,
                                     files={'file': (filename, f, 'image/jpeg')}, timeout=10)
            print(f"[UPLOAD] {'Sukses' if resp.status_code == 200 else 'Gagal'} ({resp.status_code})")
        except Exception as e:
            print(f"[UPLOAD] Error koneksi (pastikan gateway jalan): {e}")

    def _publish_redis(self, frame, state, nav_status):
        if not self.redis_client:
            return
        try:
            import base64
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            payload_vis = {
                "type": "vision_update",
                "frame_base64": base64.b64encode(buffer).decode('utf-8'),
                "buoy_counts": {"red": 0, "green": 0},
                "info": {"state": state, "nav_status": nav_status, "p_gain": "SIM-REAL-YOLO"},
            }
            self.redis_client.publish(self.config.VISION_CHANNEL, json.dumps(payload_vis))
        except Exception:
            pass

    # ------------------- Loop utama -------------------
    def run(self, data_signal):
        print("--- [SIM] GROUND SIMULATOR (kamera + YOLO asli) START ---")
        self.running = True

        # --- Telemetri NYATA dari Pixhawk (kalau terhubung) ---
        self.mav = mavlink_mod.MavlinkTelemetry(port=self.config.SERIAL_PORT,
                                                baud=self.config.BAUD_RATE)
        if not mavlink_mod.MAVLINK_AVAILABLE:
            print("[SIM] pymavlink belum terpasang — attitude dari mock.")
        elif self.mav.connect(timeout_s=5.0):
            print("[SIM] Telemetri MAVLink AKTIF — roll/pitch/yaw dari Pixhawk.")
        else:
            print("[SIM] Pixhawk offline — roll/pitch/yaw dari mock.")
        self._last_mav_log = 0.0

        # Worker YOLO asinkron: video loop jalan 30 fps terlepas dari
        # kecepatan inferensi (deteksi tersedia di saatnya).
        self._yolo_worker = _YoloWorker(self.config)
        self._yolo_worker.bind_gate_model(self.gate_model)
        self._yolo_worker.start()

        while self.running:
            # --- Baca frame kamera (None-safe bila kamera tidak ada) ---
            if self.cap is not None:
                ret, frame = self.cap.read()
            else:
                ret, frame = False, None
            if not ret or frame is None:
                frame = np.zeros((self.config.FRAME_HEIGHT, self.config.FRAME_WIDTH, 3),
                                 dtype=np.uint8)
                cv2.putText(frame, "CAMERA ERROR", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 0, 255), 2)

            elapsed = time.time() - self.state_timer
            status_txt = "Transit"
            is_detected = False
            processed_frame = frame

            # Frame-skip: inferensi YOLO tidak tiap frame (bisa ratusan ms di
            # CPU), agar capture & tampilan video tetap mengalir ~30 fps.
            self._yolo_subcount += 1
            _run_now = (self._yolo_subcount % (self.config.YOLO_FRAME_SKIP + 1)) == 0

            # Alur skenario darat: gate1 -> gate2 -> gate3 -> kotak hijau
            # -> kotak biru (foto bawah air) -> docking -> MISSION_COMPLETE
            if self.sim_step == 1:
                self.current_state = "ENTER_TRACK_1"
                status_txt = "Navigating to Gate 1..."
                if elapsed > 3.0:
                    self.sim_step = 2
                    self.state_timer = time.time()
            elif self.sim_step == 2:
                self.current_state = "VISION_TRACK_1"
                status_txt = "Scanning Buoy..."
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame, run_now=_run_now)
                if is_detected:
                    status_txt = "GATE DETECTED!"
                    if elapsed > 2.0:
                        self.sim_step = 3
                        self.state_timer = time.time()
                else:
                    self.state_timer = time.time()
            elif self.sim_step == 3:
                self.current_state = "ENTER_TRACK_2"
                status_txt = "Moving to Track 2..."
                if elapsed > 3.0:
                    self.sim_step = 4
                    self.state_timer = time.time()
            elif self.sim_step == 4:
                self.current_state = "VISION_TRACK_2"
                status_txt = "Scanning Buoy..."
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame, run_now=_run_now)
                if is_detected and elapsed > 2.0:
                    self.sim_step = 5
                    self.state_timer = time.time()
                else:
                    self.state_timer = time.time()
            elif self.sim_step == 5:
                self.current_state = "ENTER_TRACK_3"
                status_txt = "Moving to Track 3..."
                if elapsed > 3.0:
                    self.sim_step = 6
                    self.state_timer = time.time()
            elif self.sim_step == 6:
                self.current_state = "VISION_TRACK_3"
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame, run_now=_run_now)
                if is_detected and elapsed > 2.0:
                    self.sim_step = 7
                    self.state_timer = time.time()
                else:
                    self.state_timer = time.time()
            elif self.sim_step == 7:
                self.current_state = "APPROACH_BOX_GREEN"
                status_txt = "Looking for Green Box..."
                is_detected, processed_frame = self._run_yolo_detection(self.box_model, frame, run_now=_run_now)
                if is_detected:
                    status_txt = "GREEN BOX FOUND!"
                    if elapsed > 3.0:
                        os.makedirs(cfg.CAPTURES_DIR, exist_ok=True)
                        path = os.path.join(cfg.CAPTURES_DIR,
                                            f"WP6_GreenBox_{int(time.time())}.jpg")
                        cv2.imwrite(path, processed_frame)
                        threading.Thread(target=self._upload_snapshot_to_server,
                                         args=(path, os.path.basename(path)),
                                         daemon=True).start()
                        self.sim_step = 8
                        self.state_timer = time.time()
                else:
                    self.state_timer = time.time()
            elif self.sim_step == 8:
                self.current_state = "APPROACH_BLUE_BOX"
                is_detected, processed_frame = self._run_yolo_detection(self.blue_box_model, frame, run_now=_run_now)
                if is_detected and elapsed > 2.0:
                    status_txt = "SWITCHING CAMERA..."
                    self._execute_smart_capture_procedure()
                    self.sim_step = 9
                    self.state_timer = time.time()
                else:
                    self.state_timer = time.time()
            elif self.sim_step == 9:
                self.current_state = "DOCKING"
                status_txt = "Scanning Dock..."
                is_detected, processed_frame = self._run_yolo_detection(self.red_dock_model, frame, run_now=_run_now)
                if is_detected and elapsed > 5.0:
                    self.current_state = "MISSION_COMPLETE"
                    print("[SIM] MISI SELESAI!")

            # mock gerakan ke waypoint aktif
            target = (self.waypoints[min(self.current_waypoint_index, len(self.waypoints) - 1)]
                      if self.waypoints
                      else {'lat': self.current_lat, 'lon': self.current_lon})
            speed = 1.5 if self.sim_step in (1, 3, 5) else 0.2
            self._update_mock_position(target['lat'], target['lon'], speed_mps=speed)

            # --- Telemetri: utamakan data NYATA dari Pixhawk bila ada ---
            if self.mav is not None:
                self.mav.poll()
            use_real = (self.mav is not None and self.mav.connected
                        and self.mav.has_attitude)
            if use_real:
                yaw_deg = self.mav.yaw_deg
                pitch_deg = self.mav.pitch_deg
                roll_deg = self.mav.roll_deg
                lat = self.mav.lat if self.mav.lat is not None else self.current_lat
                lon = self.mav.lon if self.mav.lon is not None else self.current_lon
                now = time.time()
                if now - self._last_mav_log > 10.0:
                    print(f"[MAV] roll={roll_deg:6.1f} pitch={pitch_deg:6.1f} "
                          f"yaw={yaw_deg:6.1f} lat={lat:.6f} lon={lon:.6f}")
                    self._last_mav_log = now
            else:
                yaw_deg = math.degrees(self.current_yaw_rad)
                pitch_deg = 0.0
                roll_deg = 0.0
                lat, lon = self.current_lat, self.current_lon

            data_signal.emit({
                "lat": lat, "lon": lon,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg, "roll_deg": roll_deg,
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": self.dist_to_wp,
                "groundspeed": self.mav.groundspeed if use_real else self.current_groundspeed,
                "mavlink_ok": bool(self.mav.connected) if self.mav else False,
                "gps_fix": bool(self.mav.lat is not None) if use_real else True,
                "frame": processed_frame,
            })

            if self.dist_to_wp < 3.0 and self.current_waypoint_index < len(self.waypoints) - 1:
                if self.sim_step in (1, 3, 5):
                    self.current_waypoint_index += 1

            # Pacing ke target fps (default 30). Bila kerja frame lebih lama
            # dari interval, tidak ada sleep tambahan (loop sesegera mungkin).
            now = time.time()
            self._fps_count += 1
            if now - self._fps_window >= 5.0:
                actual = self._fps_count / (now - self._fps_window)
                print(f"[SIM] FPS aktual: {actual:.1f} "
                      f"(target {self.config.CAMERA_TARGET_FPS})")
                self._fps_count = 0
                self._fps_window = now
            target_interval = 1.0 / max(1, int(self.config.CAMERA_TARGET_FPS))
            sleep_needed = target_interval - (now - self._loop_start)
            if sleep_needed > 0:
                time.sleep(sleep_needed)
            self._loop_start = time.time()

    def stop(self):
        self.running = False
        yolo_worker = getattr(self, "_yolo_worker", None)
        if yolo_worker is not None:
            yolo_worker.stop_and_wait()
        if getattr(self, 'mav', None):
            self.mav.close()
        if self.cap:
            self.cap.release()


class GroundSimNavigatorThread(QThread):
    """Wrapper QThread untuk GroundSimNavigator (kamera asli)."""
    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        self.config = cfg.Config()
        self.navigator = GroundSimNavigator(self.config)
        self.navigator.waypoints = waypoints

    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            print(f"[TUNING] {key} = {value}")
        else:
            print(f"[ERROR] Config key '{key}' tidak ada!")

    def save_config_to_file(self):
        cfg.save_params(self.config)

    def run(self):
        try:
            self.navigator.run(self.newData)
        except Exception as e:
            print(f"[SIM] Thread ground error: {e}")
            import traceback
            traceback.print_exc()

    def stop(self):
        self.navigator.stop()

    def update_waypoints(self, wps):
        self.navigator.waypoints = wps


# ---------------------------------------------------------------------------
# MockSimNavigator — tanpa kamera: vision & fisika disintesis
# ---------------------------------------------------------------------------
class MockSimNavigator:
    """Simulator penuh tanpa hardware: objek vision digenerate, posisi kapal
    digerakkan dengan rumus destination(). Tidak butuh kamera maupun model."""

    def __init__(self, config):
        self.config = config
        self.waypoints = []
        self.current_waypoint_index = 0
        self.running = False

        self.current_lat = -7.0476989
        self.current_lon = 110.4419256
        self.current_yaw_rad = 0.0
        self.current_groundspeed = 5.0

        self.current_state = "INIT"
        self.task_timer = 0.0
        self.last_loop_time = time.time()
        self.last_used_p_gain = config.VISION_P_GAIN
        self.image_center_x = config.FRAME_WIDTH / 2.0
        self.mock_box = None
        self.fuzzy_ctrl = self._create_fuzzy_controller()

        self.redis_client = None
        if REDIS_AVAILABLE:
            try:
                self.redis_client = redis.Redis(host=self.config.REDIS_HOST,
                                                port=self.config.REDIS_PORT,
                                                decode_responses=True)
            except Exception:
                pass

    def _create_fuzzy_controller(self):
        if not FUZZY_ENABLED:
            return None
        jarak = ctrl.Antecedent(np.arange(0, 10.01, 0.1), 'Jarak')
        error = ctrl.Antecedent(np.arange(0, 321, 1), 'Error')
        p_gain = ctrl.Consequent(np.arange(0, 3.01, 0.1), 'P_GAIN')
        jarak['DEKAT'] = fuzz.trapmf(jarak.universe, [0, 0, 0.5, 1.0])
        jarak['JAUH'] = fuzz.trapmf(jarak.universe, [0.5, 1.0, 10, 10])
        error['KECIL'] = fuzz.trapmf(error.universe, [0, 0, 50, 100])
        error['BESAR'] = fuzz.trapmf(error.universe, [50, 100, 320, 320])
        p_gain['RENDAH'] = fuzz.trimf(p_gain.universe, [0, 0.5, 1.0])
        p_gain['TINGGI'] = fuzz.trimf(p_gain.universe, [1.5, 2.0, 3.0])
        rules = [
            ctrl.Rule(jarak['DEKAT'], p_gain['RENDAH']),
            ctrl.Rule(jarak['JAUH'] & error['KECIL'], p_gain['RENDAH']),
            ctrl.Rule(jarak['JAUH'] & error['BESAR'], p_gain['TINGGI']),
        ]
        return ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

    def _generate_mock_vision(self, elapsed):
        """Objek tiruan yang 'menjauh' ke tengah frame seiring waktu (simulasi align)."""
        w = self.config.FRAME_WIDTH
        progress = min(1.0, elapsed / 3.0)
        current_error = 200 * (1.0 - progress)   # px: 200 -> 0
        cx = (w / 2) + current_error + random.uniform(-5, 5)
        cy = (self.config.FRAME_HEIGHT / 2) + random.uniform(-5, 5)
        dist = max(0.5, 5.0 - elapsed)
        size = int(100 / dist)
        return {'cx': cx, 'cy': cy,
                'box': [int(cx - size), int(cy - size), int(cx + size), int(cy + size)],
                'dist': dist}

    def _calculate_correction(self, mock_obj):
        """Koreksi yaw (rad) dari selisih pixel objek vs pusat frame, dengan
        gain statis (VISION_P_GAIN) atau gain fuzzy bila tersedia."""
        if not mock_obj:
            return 0.0
        error_px = mock_obj['cx'] - self.image_center_x
        dist_m = mock_obj['dist']
        try:
            error_m = (error_px * dist_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError:
            return 0.0
        raw_rad = math.atan2(error_m, dist_m)

        gain = self.config.VISION_P_GAIN
        if FUZZY_ENABLED and self.fuzzy_ctrl:
            try:
                self.fuzzy_ctrl.input['Jarak'] = min(dist_m, 10.0)
                self.fuzzy_ctrl.input['Error'] = abs(error_px)
                self.fuzzy_ctrl.compute()
                gain = self.fuzzy_ctrl.output['P_GAIN']
            except Exception:
                pass
        self.last_used_p_gain = gain
        return raw_rad * gain

    def _set_state(self, new_state):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self.task_timer = time.time()
        print(f"[SIM] -> {new_state}")

    def run(self, data_signal):
        if not self.waypoints:
            self.current_state = "NO_WAYPOINTS"
        else:
            self.current_state = "WAYPOINT_NAV"
            self.current_lat = self.waypoints[0]['lat']
            self.current_lon = self.waypoints[0]['lon']

        print("--- [SIM] MOCK SIMULATOR (tanpa kamera) START ---")
        self.running = True
        while self.running:
            dt = time.time() - self.last_loop_time
            self.last_loop_time = time.time()
            elapsed = time.time() - self.task_timer

            target_yaw = self.current_yaw_rad
            move = False
            jarak_wp = 0.0
            correction_rad = 0.0
            bear_wp = 0.0

            if self.current_waypoint_index < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint_index]
                jarak_wp, bear_wp = geo.distance_bearing(self.current_lat, self.current_lon,
                                                         wp['lat'], wp['lon'])

            # ----- state machine sederhana -----
            if self.current_state == "WAYPOINT_NAV":
                self.mock_box = None
                target_yaw = bear_wp
                move = True
                if jarak_wp < self.config.ACCEPTANCE_RADIUS_M:
                    if self.current_waypoint_index in self.config.PHOTO_BOX_LEGS:
                        self._set_state("APPROACH_BOX_SEARCH")
                    elif self.current_waypoint_index in self.config.BLUE_BOX_PHOTO_LEGS:
                        self._set_state("APPROACH_BLUE_BOX_SEARCH")
                    elif self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP:
                        self._set_state("APPROACH_RED_BOX_SEARCH")
                    else:
                        self.current_waypoint_index += 1
                        if self.current_waypoint_index >= len(self.waypoints):
                            self._set_state("MISSION_COMPLETE")
                        else:
                            self._set_state("WAYPOINT_TRANSITION")

            elif self.current_state == "WAYPOINT_TRANSITION":
                target_yaw = bear_wp
                move = True
                if elapsed > self.config.TRANSITION_DURATION_S:
                    self._set_state("WAYPOINT_NAV")

            elif "SEARCH" in self.current_state:
                if elapsed > 1.5:
                    self._set_state(self.current_state.replace("SEARCH", "ALIGN"))

            elif "ALIGN" in self.current_state:
                move = True
                self.mock_box = self._generate_mock_vision(elapsed)
                correction_rad = self._calculate_correction(self.mock_box)
                target_yaw = geo.normalize_angle(self.current_yaw_rad + correction_rad)
                if self.mock_box['dist'] < 1.0:
                    if "BLUE" in self.current_state:
                        self._set_state("TAKE_BLUE_BOX_PHOTO")
                    elif "RED" in self.current_state:
                        self._set_state("RED_BOX_DOCKED")
                    else:
                        self._set_state("TAKE_PHOTO")

            elif "TAKE" in self.current_state or "DOCKED" in self.current_state:
                if elapsed > 2.0:
                    if "DOCKED" in self.current_state:
                        self.current_waypoint_index += 1
                        self._set_state("WAYPOINT_NAV")
                    else:
                        self._set_state("RETREAT")

            elif "RETREAT" in self.current_state:
                if elapsed > self.config.RETREAT_DURATION_S:
                    if "BLUE" not in self.current_state:
                        self.current_waypoint_index += 1
                    self._set_state("WAYPOINT_NAV")

            # ----- fisika sederhana -----
            if move:
                diff = geo.normalize_angle(target_yaw - self.current_yaw_rad)
                self.current_yaw_rad += diff * dt * 2.0
                self.current_lat, self.current_lon = geo.destination(
                    self.current_lat, self.current_lon, 5.0 * dt, self.current_yaw_rad)

            # ----- render frame tiruan -----
            frame = np.zeros((self.config.FRAME_HEIGHT, self.config.FRAME_WIDTH, 3),
                             dtype=np.uint8)
            cv2.putText(frame, f"SIM STATE: {self.current_state}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame, f"GAIN: {self.last_used_p_gain:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            if self.mock_box:
                b = self.mock_box['box']
                cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                cv2.line(frame, (int(self.config.FRAME_WIDTH / 2), self.config.FRAME_HEIGHT),
                         (int(self.mock_box['cx']), int(self.mock_box['cy'])), (0, 0, 255), 2)

            data_signal.emit({
                "lat": self.current_lat, "lon": self.current_lon,
                "yaw_deg": math.degrees(self.current_yaw_rad),
                "pitch_deg": 0.0, "roll_deg": 0.0,
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": jarak_wp,
                "groundspeed": 0.0,
                "mavlink_ok": False,
                "gps_fix": True,
                "frame": frame,
            })
            time.sleep(1.0 / 20.0)

    def stop(self):
        self.running = False


class MockSimNavigatorThread(QThread):
    """Wrapper QThread untuk MockSimNavigator (tanpa hardware)."""
    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        self.config = cfg.Config()
        self.navigator = MockSimNavigator(self.config)
        self.navigator.waypoints = waypoints

    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            print(f"[TUNING] {key} = {value}")
        else:
            print(f"[ERROR] Config key '{key}' tidak ada!")

    def save_config_to_file(self):
        cfg.save_params(self.config)

    def run(self):
        try:
            self.navigator.run(self.newData)
        except Exception as e:
            print(f"[SIM] Thread mock error: {e}")
            import traceback
            traceback.print_exc()

    def stop(self):
        self.navigator.stop()

    def update_waypoints(self, new_waypoints):
        self.navigator.waypoints = new_waypoints
        self.navigator.current_waypoint_index = 0
        if new_waypoints:
            self.navigator.current_lat = new_waypoints[0]['lat']
            self.navigator.current_lon = new_waypoints[0]['lon']
        self.navigator.task_timer = time.time()
        self.navigator.current_state = "WAYPOINT_NAV" if new_waypoints else "NO_WAYPOINTS"
        print("[SIM] Misi di-reset.")


# Nama ramah yang dipakai main.py (default: ground sim dengan kamera asli).
NavigatorThread = GroundSimNavigatorThread