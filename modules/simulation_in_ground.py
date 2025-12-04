# FILE: modules/simulator_backend.py
from . import config as cfg
from PyQt5.QtCore import QThread, pyqtSignal
from ultralytics import YOLO
import numpy as np
import time, math, cv2, json, redis, base64, threading, os, requests

class VisionOffboardNavigator:
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
        
        # --- REAL CAMERA SETUP ---
        print(f"[SIMULATOR] Membuka Kamera Utama (Index {self.config.CAMERA_INDEX})...")
        self.cap = cv2.VideoCapture(self.config.CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.FRAME_HEIGHT)
        
        # --- LOAD REAL MODELS ---
        print("[SIMULATOR] Memuat Model YOLO Asli...")
        try:
            self.gate_model = YOLO(self.config.MODEL_PATH, task='detect')
            self.box_model = YOLO(self.config.BOX_MODEL_PATH, task='detect')
            self.blue_box_model = YOLO(self.config.BLUE_BOX_MODEL_PATH, task='detect')
            self.red_dock_model = YOLO(self.config.RED_DOCK_MODEL_PATH, task='detect')
            print("[SIMULATOR] Semua Model Berhasil Dimuat!")
        except Exception as e:
            print(f"[ERROR] Gagal memuat model YOLO: {e}")

        # --- REDIS ---
        self.redis_client = None
        try:
            self.redis_client = redis.Redis(host=self.config.REDIS_HOST, port=self.config.REDIS_PORT, decode_responses=True)
            self.redis_client.ping()
        except:
            print("[SIMULATOR] Redis Connection Failed!")

        self.state_timer = time.time()
        self.sim_step = 1

    # --- FUNGSI BARU: UPLOAD KE WEB ---
    def _upload_snapshot_to_server(self, file_path, filename):
        """ Mengirim file foto ke endpoint Laravel/Web via HTTP POST """
        if not os.path.exists(file_path):
            print(f"[UPLOAD] GAGAL: File {filename} tidak ditemukan.")
            return
        
        print(f"[UPLOAD] Mengirim {filename} ke {self.config.SERVER_UPLOAD_URL}...")
        try:
            with open(file_path, 'rb') as f:
                # Key 'file' harus sesuai dengan request controller di Laravel
                files_payload = {'file': (filename, f, 'image/jpeg')}
                response = requests.post(self.config.SERVER_UPLOAD_URL, files=files_payload, timeout=10)
                
                if response.status_code == 200:
                    print(f"[UPLOAD] SUKSES! Server merespon: {response.text}")
                else:
                    print(f"[UPLOAD] GAGAL: Error {response.status_code} - {response.text}")
        except requests.exceptions.RequestException as e:
            print(f"[UPLOAD] ERROR KONEKSI: Pastikan Web Gateway/Ngrok jalan! ({e})")
        except Exception as e:
            print(f"[UPLOAD] ERROR LAIN: {e}")

    def _get_distance_and_bearing(self, lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dlambda = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
        c = 2*math.atan2(math.sqrt(a), math.sqrt(1-a))
        dist = R*c
        y = math.sin(dlambda) * math.cos(phi2)
        x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)
        bearing = math.atan2(y, x)
        return dist, bearing

    def _update_mock_position(self, target_lat, target_lon, speed_mps=1.5):
        dist, bearing = self._get_distance_and_bearing(self.current_lat, self.current_lon, target_lat, target_lon)
        self.dist_to_wp = dist
        self.current_yaw_rad = bearing
        if dist > 2.0:
            delta_dist = speed_mps * 0.1
            self.current_lat += (delta_dist * math.cos(bearing)) / 111320
            self.current_lon += (delta_dist * math.sin(bearing)) / (40075000 * math.cos(self.current_lat) / 360)
            self.current_groundspeed = speed_mps
            return False 
        else:
            self.current_groundspeed = 0.5
            return True 

    def _run_yolo_detection(self, model, frame, conf_threshold=0.5):
        if model is None: return False, frame
        results = model(frame, verbose=False, conf=conf_threshold)
        annotated_frame = results[0].plot()
        detected = len(results[0].boxes) > 0
        if model == self.gate_model: detected = len(results[0].boxes) >= 2
        return detected, annotated_frame

    def _execute_smart_capture_procedure(self):
        print("\n=== [WP 8] MEMULAI PROSEDUR SMART PHOTO (REAL HARDWARE) ===")
        if self.cap.isOpened():
            self.cap.release()
            print("[WP 8] Kamera Navigasi dipause.")
        time.sleep(1.0) 

        print(f"[WP 8] Membuka Kamera Bawah Index {self.config.WAYPOINT_PHOTO_CAMERA_INDEX}...")
        cam_bawah = cv2.VideoCapture(self.config.WAYPOINT_PHOTO_CAMERA_INDEX, cv2.CAP_DSHOW)
        
        if cam_bawah.isOpened():
            # SETTINGAN KAMERA BAWAH AIR AGAR TERANG
            cam_bawah.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cam_bawah.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            print("[WP 8] Menerapkan Auto-Exposure: 1 (Force ON)")
            cam_bawah.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 

            print("[WP 8] Warming up sensor (20 frames)...")
            for _ in range(20): cam_bawah.read()

            foto_sukses = False
            for percobaan in range(1, 6):
                ret, frame_bawah = cam_bawah.read()
                if ret:
                    avg = np.mean(frame_bawah)
                    print(f"[WP 8] Percobaan {percobaan}: Brightness = {avg:.2f}")
                    self._publish_redis(frame_bawah, "PHOTO_ATTEMPT", f"Try {percobaan}")

                    if avg > 1.0: # Ambang batas rendah untuk tes indoor
                        timestamp = int(time.time())
                        if not os.path.exists("captures"): os.makedirs("captures")
                        
                        # Nama file unik
                        filename_short = f"WP8_RealTest_{timestamp}.jpg"
                        filename_full = os.path.join("captures", filename_short)
                        
                        cv2.imwrite(filename_full, frame_bawah)
                        print(f"[WP 8] FOTO SUKSES: {filename_full}")
                        
                        # --- TRIGGER UPLOAD DISINI ---
                        threading.Thread(target=self._upload_snapshot_to_server, args=(filename_full, filename_short), daemon=True).start()
                        
                        foto_sukses = True
                        time.sleep(1.0) 
                        break
                    else:
                        print("[WP 8] Foto gelap. Retrying...")
                        time.sleep(0.5)
            
            cam_bawah.release()
        else:
            print(f"[WP 8] ERROR: Gagal membuka kamera index {self.config.WAYPOINT_PHOTO_CAMERA_INDEX}!")

        print("[WP 8] Restarting Nav Camera...")
        self.cap = cv2.VideoCapture(self.config.CAMERA_INDEX, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.FRAME_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.FRAME_HEIGHT)

    def _publish_redis(self, frame, state, nav_status):
        if not self.redis_client: return
        try:
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            jpg_as_base64 = base64.b64encode(buffer).decode('utf-8')
            payload_vis = {
                "type": "vision_update",
                "frame_base64": jpg_as_base64,
                "buoy_counts": {"red": 0, "green": 0},
                "info": {"state": state, "nav_status": nav_status, "p_gain": "SIM-REAL-YOLO"}
            }
            self.redis_client.publish(self.config.VISION_CHANNEL, json.dumps(payload_vis))
            payload_telem = {
                "type": "telemetry",
                "data": {
                    "lat": self.current_lat, "lon": self.current_lon,
                    "yaw": self.current_yaw_rad, "heading": math.degrees(self.current_yaw_rad),
                    "roll": 0, "pitch": 0, "groundspeed": self.current_groundspeed,
                    "voltage": 16.0, "state": state, "target_wp_idx": self.current_waypoint_index,
                    "dist_to_wp_m": self.dist_to_wp
                }
            }
            self.redis_client.publish(self.config.TELEMETRY_CHANNEL, json.dumps(payload_telem))
        except: pass

    def run(self, data_signal):
        print("--- SIMULATOR REAL VISION STARTED ---")
        self.running = True
        
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                frame = np.zeros((self.config.FRAME_HEIGHT, self.config.FRAME_WIDTH, 3), dtype=np.uint8)
                cv2.putText(frame, "CAMERA ERROR", (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)

            elapsed = time.time() - self.state_timer
            status_txt = "Transit"
            is_detected = False
            processed_frame = frame
            
            # --- LOGIKA STEPS ---
            if self.sim_step == 1: 
                self.current_state = "ENTER_TRACK_1"
                status_txt = "Navigating to Gate 1..."
                if elapsed > 3.0: 
                    self.sim_step = 2; self.state_timer = time.time()
                    print("[SIM] Masuk Step 2: TUNJUKKAN 2 BUOY!")

            elif self.sim_step == 2: 
                self.current_state = "VISION_TRACK_1"
                status_txt = "Scanning Buoy..."
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame)
                if is_detected:
                    status_txt = "GATE DETECTED!"
                    if elapsed > 2.0:
                        self.sim_step = 3; self.state_timer = time.time()
                        print("[SIM] Gate Terdeteksi! Lanjut Track 2...")
                else: self.state_timer = time.time()

            elif self.sim_step == 3:
                self.current_state = "ENTER_TRACK_2"
                status_txt = "Moving to Track 2..."
                if elapsed > 3.0: 
                    self.sim_step = 4; self.state_timer = time.time()
                    print("[SIM] Masuk Step 4: TUNJUKKAN 2 BUOY LAGI!")

            elif self.sim_step == 4:
                self.current_state = "VISION_TRACK_2"
                status_txt = "Scanning Buoy..."
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame)
                if is_detected:
                    status_txt = "GATE 2 DETECTED!"
                    if elapsed > 2.0:
                        self.sim_step = 5; self.state_timer = time.time()
                        print("[SIM] Lanjut Track 3...")
                else: self.state_timer = time.time()

            elif self.sim_step == 5:
                self.current_state = "ENTER_TRACK_3"
                if elapsed > 3.0: 
                    self.sim_step = 6; self.state_timer = time.time()
                    print("[SIM] Masuk Step 6: TUNJUKKAN 2 BUOY (Gate 3)!")

            elif self.sim_step == 6:
                self.current_state = "VISION_TRACK_3"
                is_detected, processed_frame = self._run_yolo_detection(self.gate_model, frame)
                if is_detected and elapsed > 2.0:
                    self.sim_step = 7; self.state_timer = time.time()
                    print("[SIM] Masuk Step 7: TUNJUKKAN BOX HIJAU!")
                elif not is_detected: self.state_timer = time.time()

            elif self.sim_step == 7:
                self.current_state = "APPROACH_BOX_GREEN"
                status_txt = "Looking for Green Box..."
                is_detected, processed_frame = self._run_yolo_detection(self.box_model, frame)
                if is_detected:
                    status_txt = "GREEN BOX FOUND!"
                    if elapsed > 3.0:
                        # --- SIMPAN & UPLOAD FOTO HIJAU ---
                        timestamp = int(time.time())
                        if not os.path.exists("captures"): os.makedirs("captures")
                        filename_short = f"WP6_GreenBox_{timestamp}.jpg"
                        filename_full = os.path.join("captures", filename_short)
                        cv2.imwrite(filename_full, processed_frame)
                        print(f"[SIM] Foto Hijau Disimpan: {filename_full}")
                        
                        # Trigger Upload
                        threading.Thread(target=self._upload_snapshot_to_server, args=(filename_full, filename_short), daemon=True).start()
                        
                        self.sim_step = 8; self.state_timer = time.time()
                        print("[SIM] Foto Hijau Oke. Step 8: TUNJUKKAN BOX BIRU!")
                else: self.state_timer = time.time()

            elif self.sim_step == 8:
                self.current_state = "APPROACH_BLUE_BOX"
                is_detected, processed_frame = self._run_yolo_detection(self.blue_box_model, frame)
                if is_detected and elapsed > 2.0:
                    status_txt = "SWITCHING CAMERA..."
                    self._publish_redis(processed_frame, self.current_state, status_txt)
                    
                    # Logika Foto Bawah Air (Upload sudah di dalam fungsi ini)
                    self._execute_smart_capture_procedure()
                    
                    self.sim_step = 9; self.state_timer = time.time()
                    print("[SIM] Foto Bawah Air Selesai. Step 9: TUNJUKKAN DOCKING!")
                elif not is_detected: self.state_timer = time.time()

            elif self.sim_step == 9:
                self.current_state = "DOCKING"
                status_txt = "Scanning Dock..."
                is_detected, processed_frame = self._run_yolo_detection(self.red_dock_model, frame)
                if is_detected:
                    status_txt = "DOCK FOUND! HOLDING..."
                    if elapsed > 5.0:
                        self.current_state = "MISSION_COMPLETE"
                        print("[SIM] MISI SELESAI!")

            target = self.waypoints[min(self.current_waypoint_index, len(self.waypoints)-1)] if self.waypoints else {'lat': self.current_lat, 'lon': self.current_lon}
            if self.sim_step in [1, 3, 5]: 
                self._update_mock_position(target['lat'], target['lon'], speed_mps=1.5)
            else:
                self._update_mock_position(target['lat'], target['lon'], speed_mps=0.2)

            self._publish_redis(processed_frame, self.current_state, status_txt)
            data_packet = {
                "lat": self.current_lat, "lon": self.current_lon,
                "yaw_deg": math.degrees(self.current_yaw_rad),
                "pitch_deg": 0.0, "roll_deg": 0.0,
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": self.dist_to_wp,
                "frame": processed_frame
            }
            data_signal.emit(data_packet)
            
            if self.dist_to_wp < 3.0 and self.current_waypoint_index < len(self.waypoints) - 1:
                if self.sim_step in [1, 3, 5]: self.current_waypoint_index += 1
            time.sleep(0.05) 

    def stop(self):
        self.running = False
        if self.cap: self.cap.release()

# ... (Kode VisionOffboardNavigator di atasnya) ...

# PASTIKAN CLASS INI ADA DI FILE simulation_in_ground.py
class NavigatorThread(QThread):
    newData = pyqtSignal(dict)
    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        self.config = cfg.Config()
        self.navigator = VisionOffboardNavigator(self.config)
        self.navigator.waypoints = waypoints 

    def run(self):
        self.navigator.run(self.newData)

    def stop(self):
        self.navigator.stop()
        
    def update_waypoints(self, wps):
        self.navigator.waypoints = wps