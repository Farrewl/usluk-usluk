from . import config as cfg
from ultralytics import YOLO
from pymavlink import mavutil
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import sys, time, math, cv2, csv, redis, base64, threading, requests, random


START_LAT, START_LON = -7.0476989, 110.4419256
SPEED_MS = 0.5
SEND_RATE_HZ = 20

# ================================================================
# KONFIGURASI SIMULATOR (DIPINDAH DARI CONFIG ASLI)
# ================================================================


# ================================================================
# VARIABEL GLOBAL & HELPER (SIMULASI PERHITUNGAN)
# ================================================================

WAYPOINTS = [] 

def _get_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000.0
    if None in [lat1, lon1, lat2, lon2]:
        return float('inf'), 0.0
    lat1_rad, lon1_rad = math.radians(lat1), math.radians(lon1)
    lat2_rad, lon2_rad = math.radians(lat2), math.radians(lon2)
    dLon = lon2_rad - lon1_rad
    y = math.sin(dLon) * math.cos(lat2_rad)
    x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon)
    bearing = math.atan2(y, x)
    dLat = lat2_rad - lat1_rad
    a = math.sin(dLat / 2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dLon / 2)**2
    c = 2 * math.atan2(math.sqrt(max(0, a)), math.sqrt(max(0, 1 - a)))
    return R * c, bearing

def _haversine_destination(lat, lon, distance_m, bearing_rad):
    R = 6371000.0
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    
    lat2_rad = math.asin(math.sin(lat_rad) * math.cos(distance_m / R) +
                        math.cos(lat_rad) * math.sin(distance_m / R) * math.cos(bearing_rad))
    lon2_rad = lon_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance_m / R) * math.cos(lat_rad),
                                 math.cos(distance_m / R) - math.sin(lat_rad) * math.sin(lat2_rad))
    
    return math.degrees(lat2_rad), math.degrees(lon2_rad)

def _normalize_angle(angle_rad):
    while angle_rad > math.pi: angle_rad -= 2 * math.pi
    while angle_rad < -math.pi: angle_rad += 2 * math.pi
    return angle_rad

def _create_dummy_frame(state, idx, jarak_ke_wp, w, h):
    """Membuat frame video palsu dengan visualisasi simulasi."""
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    
    # Text Status
    cv2.putText(frame, f"STATE: {state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    wp_text = f"Next WP: #{idx + 1} ({jarak_ke_wp:.1f} m)"
    if state == "MISSION_COMPLETE":
        wp_text = "MISSION COMPLETE"
    cv2.putText(frame, wp_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    
    # Visualisasi Simulasi Task
    mid_w, mid_h = w // 2, h // 2
    
    if state == "WAYPOINT_NAV" and (idx + 1) in [1, 3, 5]:
        cv2.circle(frame, (mid_w - 50, mid_h), 20, (0, 0, 255), -1) # Merah
        cv2.circle(frame, (mid_w + 50, mid_h), 20, (0, 255, 0), -1) # Hijau
        cv2.putText(frame, "SIMULATED GATE", (mid_w - 70, mid_h + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
    elif state.startswith("APPROACH_BOX_SEARCH") or state.startswith("APPROACH_BOX_ALIGN"):
        cv2.rectangle(frame, (mid_w - 40, mid_h - 40), (mid_w + 40, mid_h + 40), (0, 255, 0), 3)
        cv2.putText(frame, "SIMULATED GREEN BOX", (mid_w - 80, mid_h + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
    elif state == "TAKE_PHOTO":
        cv2.putText(frame, "SNAP! (Green Box)", (mid_w - 100, mid_h), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 3)
        
    elif state.startswith("APPROACH_RED_BOX"):
        cv2.rectangle(frame, (mid_w - 40, mid_h - 40), (mid_w + 40, mid_h + 40), (0, 128, 255), 3)
        cv2.putText(frame, "SIMULATED RED DOCK", (mid_w - 80, mid_h + 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 128, 255), 1)
        
    elif state == "RED_BOX_DOCKED":
        cv2.putText(frame, "DOCKED!", (mid_w - 50, mid_h), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 3)
        
    return frame

# ================================================================
# CLASS NAVIGATOR SIMULASI (PENGGANTI VisionOffboardNavigator)
# ================================================================

class SimOffboardNavigator:

    def __init__(self, config):
        self.config = config
        self.waypoints = WAYPOINTS 
        self.current_waypoint_index = 0
        self.running = False
        
        # State simulasi
        self.current_lat = START_LAT
        self.current_lon = START_LON
        self.current_yaw_rad = 0.0 # Heading
        self.current_pitch_rad = 0.0
        self.current_roll_rad = 0.0
        self.current_groundspeed = SPEED_MS
        self.current_voltage = 12.5

        self.current_state = "INIT"
        self.task_timer = 0.0
        self.last_loop_time = time.time()
        
        # Variabel dummy untuk GUI
        self.last_vision_correction_rad = 0.0
        self.last_used_p_gain = self.config.VISION_P_GAIN
        self.retreat_step = "IDLE"

    def _update_telemetry(self, loop_delta_time):
        """Memperbarui data telemetri simulasi."""
        
        current_idx = self.current_waypoint_index
        jarak_ke_wp = float('inf')
        
        # 1. Logic Pergerakan
        if current_idx < len(self.waypoints) and self.current_state not in ["INIT", "NO_WAYPOINTS", "MISSION_COMPLETE", "TAKE_PHOTO", "RED_BOX_DOCKED", "TAKE_WAYPOINT_PHOTO", "BLUE_BOX_RETREAT", "RETREAT", "WAYPOINT_TRANSITION"]:
            
            target_wp = self.waypoints[current_idx]
            jarak_ke_wp, bearing_rad = _get_distance_and_bearing(
                self.current_lat, self.current_lon, 
                target_wp['lat'], target_wp['lon']
            )
            
            # Update heading dan groundspeed
            self.current_yaw_rad = _normalize_angle(bearing_rad)
            self.current_groundspeed = SPEED_MS
            
            # Kalkulasi pergerakan
            dist_to_move_m = self.current_groundspeed * loop_delta_time
            dist_to_move_m = min(dist_to_move_m, jarak_ke_wp) # Jangan overshoot
            
            # Dapatkan lat/lon baru
            self.current_lat, self.current_lon = _haversine_destination(
                self.current_lat, self.current_lon, 
                dist_to_move_m, bearing_rad
            )
        else:
            self.current_groundspeed = 0.0

        return jarak_ke_wp
    
    def _set_state(self, new_state):
        if self.current_state == new_state:
            return
        self.current_state = new_state
        self.task_timer = time.time()
        print(f"\nSTATE CHANGE: -> {new_state}")

    def run(self, data_signal):
        
        if not self.waypoints:
            self.current_state = "NO_WAYPOINTS"
        else:
            self.current_state = "WAYPOINT_NAV"
            self.current_lat = self.waypoints[0]['lat'] + random.uniform(-0.00005, 0.00005)
            self.current_lon = self.waypoints[0]['lon'] + random.uniform(-0.00005, 0.00005)
            self.current_waypoint_index = 0
            self.task_timer = time.time()
        
        print(f"Starting Sim Navigator. Initial State: {self.current_state}")
        self.running = True
        
        loop_interval = 1.0 / SEND_RATE_HZ
        
        while self.running:
            loop_start_time = time.time()
            loop_delta_time = loop_start_time - self.last_loop_time
            self.last_loop_time = loop_start_time

            # Dapatkan jarak sebelum simulasi pergerakan
            try:
                jarak_ke_wp_m = self._update_telemetry(loop_delta_time)
            except Exception as e:
                print(f"Error di _update_telemetry: {e}")
                jarak_ke_wp_m = float('inf')

            time_since_state_change = time.time() - self.task_timer
            current_idx = self.current_waypoint_index
            
            # --- STATE MACHINE SIMULASI ---

            if self.current_state == "NO_WAYPOINTS" or self.current_state == "MISSION_COMPLETE":
                pass
            
            elif self.current_state == "WAYPOINT_NAV" or self.current_state == "WAYPOINT_TRANSITION":
                
                if jarak_ke_wp_m < self.config.ACCEPTANCE_RADIUS_M and self.current_state == "WAYPOINT_NAV":
                    print(f"\nWaypoint #{current_idx + 1} tercapai (Simulasi).")
                    
                    if current_idx in self.config.PHOTO_BOX_LEGS:
                        self._set_state("APPROACH_BOX_SEARCH")
                        continue
                    
                    if current_idx in self.config.BLUE_BOX_PHOTO_LEGS:
                        self._set_state("APPROACH_BLUE_BOX_SEARCH")
                        continue
                        
                    if current_idx in self.config.STOP_AND_PHOTO_AT_WP:
                        self._set_state("TAKE_WAYPOINT_PHOTO")
                        continue
                        
                    # Lanjut ke WP berikutnya
                    self.current_waypoint_index += 1
                    if self.current_waypoint_index >= len(self.waypoints):
                        self._set_state("MISSION_COMPLETE")
                    else:
                        self._set_state("WAYPOINT_TRANSITION")
                
                # Logic Transisi
                if self.current_state == "WAYPOINT_TRANSITION" and time_since_state_change > self.config.TRANSITION_DURATION_S:
                    self._set_state("WAYPOINT_NAV")
                    
            # --- MISI HIJAU/BIRU ---
            elif self.current_state == "APPROACH_BOX_SEARCH":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("APPROACH_BOX_ALIGN")
            
            elif self.current_state == "APPROACH_BOX_ALIGN":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("TAKE_PHOTO")

            elif self.current_state == "TAKE_PHOTO":
                if time_since_state_change > 1.0: # Waktu "jepret"
                    self._set_state("RETREAT")
                    
            elif self.current_state == "RETREAT":
                if time_since_state_change > self.config.RETREAT_DURATION_S:
                    self._set_state("WAYPOINT_NAV")
                    self.current_waypoint_index += 1 # Lanjut ke WP setelah misi box

            # --- MISI BIRU Samping (contoh alur) ---
            elif self.current_state == "APPROACH_BLUE_BOX_SEARCH":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("APPROACH_BLUE_BOX_ALIGN")
            
            elif self.current_state == "APPROACH_BLUE_BOX_ALIGN":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("TAKE_BLUE_BOX_PHOTO")

            elif self.current_state == "TAKE_BLUE_BOX_PHOTO":
                if time_since_state_change > 1.0: # Waktu "jepret"
                    self._set_state("BLUE_BOX_RETREAT")

            elif self.current_state == "BLUE_BOX_RETREAT":
                if time_since_state_change > self.config.RETREAT_DURATION_S:
                    if self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP:
                        self._set_state("APPROACH_RED_BOX_SEARCH")
                    else:
                        self._set_state("WAYPOINT_NAV")
                        self.current_waypoint_index += 1 # Lanjut ke WP setelah misi box

            # --- MISI DOCKING MERAH ---
            elif self.current_state == "APPROACH_RED_BOX_SEARCH":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("APPROACH_RED_BOX_ALIGN")
            
            elif self.current_state == "APPROACH_RED_BOX_ALIGN":
                if time_since_state_change > self.config.TASK_WAIT_S:
                    self._set_state("RED_BOX_DOCKED")

            elif self.current_state == "RED_BOX_DOCKED":
                if time_since_state_change > self.config.DOCK_HOLD_DURATION_S:
                    self._set_state("WAYPOINT_NAV")
                    self.current_waypoint_index += 1 # Lanjut ke WP setelah docking

            # --- FOTO WP ---
            elif self.current_state == "TAKE_WAYPOINT_PHOTO":
                if time_since_state_change > self.config.WAYPOINT_PHOTO_STOP_DURATION_S:
                    if current_idx == self.config.RED_BOX_NAV_AFTER_WP:
                         self._set_state("APPROACH_RED_BOX_SEARCH")
                    else:
                         self._set_state("WAYPOINT_NAV")
                         self.current_waypoint_index += 1 # Lanjut ke WP setelah foto

            # --- KIRIM DATA KE GUI ---
            frame = _create_dummy_frame(
                self.current_state, 
                self.current_waypoint_index, 
                jarak_ke_wp_m,
                self.config.FRAME_WIDTH, 
                self.config.FRAME_HEIGHT
            )
            
            safe_frame_copy = frame.copy()

            data_packet = {
                "lat": self.current_lat, 
                "lon": self.current_lon,
                "yaw_deg": math.degrees(self.current_yaw_rad),
                "pitch_deg": math.degrees(self.current_pitch_rad), 
                "roll_deg": math.degrees(self.current_roll_rad),
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": jarak_ke_wp_m if jarak_ke_wp_m != float('inf') else 0.0,
                "frame": safe_frame_copy
            }
            data_signal.emit(data_packet)

            # Atur kecepatan loop
            sleep_duration = loop_interval - (time.time() - loop_start_time)
            if sleep_duration > 0:
                time.sleep(sleep_duration)
        
        print("Sim Navigator loop finished.")

    def stop(self):
        print("Sim Navigator menerima sinyal stop...")
        self.running = False

# ================================================================
# CLASS THREAD UTAMA (PENGGANTI NavigatorThread)
# ================================================================

class NavigatorThread(QThread):
    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        global WAYPOINTS
        WAYPOINTS = waypoints
        self.config = cfg.Config()
        self.navigator = SimOffboardNavigator(self.config)
        
    def save_config_to_file(self):
        # Dummy save
        print("Parameter tuning simulasi tidak disimpan ke file.")
        
    # Fungsi update_* perlu tetap ada karena dihubungkan oleh main.py
    def update_acceptance_radius(self, value): self.config.ACCEPTANCE_RADIUS_M = value
    def update_thrust(self, value): self.config.THRUST_VALUE = value
    def update_transition_duration(self, value): self.config.TRANSITION_DURATION_S = value
    def update_geofence_width(self, value): self.config.GEOFENCE_WIDTH_METERS = value
    def update_focalpx(self, value): self.config.FOCAL_LENGTH_PX = value
    def update_p_gain(self, value): self.config.VISION_P_GAIN = value
    def update_smoothing_alpha(self, value): self.config.VISION_SMOOTHING_ALPHA = value
    def update_roi_cutoff(self, value): self.config.ROI_TOP_CUTOFF_PERCENT = value
    def update_gate_width(self, value): self.config.GATE_WIDTH_METERS = value
    def update_min_buoy_area(self, value): self.config.MIN_BUOY_AREA_PX = value
    def update_gate_area_ratio(self, value,): self.config.GATE_AREA_SIMILARITY_RATIO = value
    def update_search_thrust(self, value): self.config.SEARCH_THRUST = value
    def update_align_thrust(self, value): self.config.ALIGN_THRUST = value
    def update_retreat_thrust(self, value): self.config.RETREAT_THRUST = value
    def update_retreat_duration(self, value): self.config.RETREAT_DURATION_S = value
    def update_box_width(self, value): self.config.BOX_WIDTH_METERS = value
    def update_box_approach_dist(self, value): self.config.BOX_APPROACH_DISTANCE_M = value
    def update_yaw_search_box(self, value): self.config.YAW_SEARCH_BOX = value
    def update_box_lat_thrust(self, value): self.config.BOX_SEARCH_LATERAL_THRUST = value
    def update_blue_box_search_thrust(self, value): self.config.BLUE_BOX_SEARCH_THRUST = value
    def update_blue_box_align_thrust(self, value): self.config.BLUE_BOX_ALIGN_THRUST = value
    def update_blue_box_width(self, value): self.config.BLUE_BOX_WIDTH_METERS = value
    def update_blue_box_approach_dist(self, value): self.config.BLUE_BOX_APPROACH_DISTANCE_M = value
    def update_blue_box_lat_offset(self, value): self.config.BLUE_BOX_LATERAL_OFFSET_M = value
    def update_blue_box_yaw_search(self, value): self.config.BLUE_BOX_YAW_SEARCH = value
    def update_dock_align_thrust(self, value): self.config.DOCK_ALIGN_THRUST = value
    def update_dock_hold_dur(self, value): self.config.DOCK_HOLD_DURATION_S = value
    def update_red_box_width(self, value): self.config.RED_BOX_WIDTH_METERS = value
    def update_red_box_dock_dist(self, value): self.config.RED_BOX_DOCK_DISTANCE_M = value
    def update_yaw_search_dock(self, value): self.config.YAW_SEARCH_DOCK = value
    def update_yolo_frame_skip(self, value): self.config.YOLO_FRAME_SKIP = value
    def update_yolo_inf_size(self, value): self.config.YOLO_INFERENCE_SIZE = value
    def update_session_video_fps(self, value): self.config.SESSION_VIDEO_FPS = value
    
    def run(self):
        print("Starting NavigatorThread (SIMULASI)...")
        self.navigator.run(self.newData) 

    def stop(self):
        if self.navigator:
            self.navigator.stop()
            
    def update_waypoints(self, new_waypoints):
        global WAYPOINTS
        WAYPOINTS = new_waypoints
        if not self.navigator:
            return
        
        print("\n--- [RESET MISSION] Perintah Save/Reload diterima ---")
        self.navigator.waypoints = new_waypoints
        if not new_waypoints or len(new_waypoints) == 0:
            self.navigator.current_waypoint_index = 0
            self.navigator._set_state("NO_WAYPOINTS")
            print("[RESET MISSION] Tidak ada waypoint baru. Berhenti (IDLE).")
        else:
            self.navigator.current_waypoint_index = 0
            self.navigator._set_state("WAYPOINT_NAV")
            self.navigator.current_lat = new_waypoints[0]['lat'] + random.uniform(-0.00005, 0.00005)
            self.navigator.current_lon = new_waypoints[0]['lon'] + random.uniform(-0.00005, 0.00005)
            print(f"[RESET MISSION] Misi direset. Menuju ke WP 0 baru.")
