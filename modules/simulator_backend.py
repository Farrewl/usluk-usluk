from . import config as cfg
from PyQt5.QtCore import QThread, pyqtSignal
import numpy as np
import sys, time, math, cv2, random, json, threading

try:
    import redis
    REDIS_AVAILABLE = True
except ImportError:
    REDIS_AVAILABLE = False

try:
    import skfuzzy as fuzz
    from skfuzzy import control as ctrl
    FUZZY_ENABLED = True
    print("[SIMULATOR] Fuzzy Logic: ON")
except ImportError:
    print("[SIMULATOR] Fuzzy Logic: OFF (Install scikit-fuzzy)")
    FUZZY_ENABLED = False

START_LAT, START_LON = -7.0476989, 110.4419256
SPEED_MS = 5
SEND_RATE_HZ = 20
WAYPOINTS = [] 

def _get_distance_and_bearing(lat1, lon1, lat2, lon2):
    R = 6371000.0
    if None in [lat1, lon1, lat2, lon2]: return float('inf'), 0.0
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

# ==================== #
# SIMULATOR CORE LOGIC #
# ==================== #

class SimOffboardNavigator:
    def _create_fuzzy_controller(self, type='gate'):
        if not FUZZY_ENABLED: return None
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
            ctrl.Rule(jarak['JAUH'] & error['BESAR'], p_gain['TINGGI'])
        ]
        return ctrl.ControlSystemSimulation(ctrl.ControlSystem(rules))

    def __init__(self, config):
        self.config = config
        self.waypoints = WAYPOINTS 
        self.current_waypoint_index = 0
        self.running = False
        
        self.current_lat = START_LAT; self.current_lon = START_LON
        self.current_yaw_rad = 0.0; self.current_groundspeed = SPEED_MS
        self.current_voltage = 15.5
        
        self.current_state = "INIT"
        self.task_timer = 0.0
        self.last_loop_time = time.time()
        
        self.last_used_p_gain = self.config.VISION_P_GAIN
        self.image_center_x = self.config.FRAME_WIDTH / 2.0
        
        self.mock_box = None
        
        self.fuzzy_ctrl = self._create_fuzzy_controller()
        
        self.redis_client = None
        if REDIS_AVAILABLE:
            try: self.redis_client = redis.Redis(host=self.config.REDIS_HOST, port=self.config.REDIS_PORT, decode_responses=True)
            except: pass

    def _generate_mock_vision(self, state, elapsed):
        w, h = self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT
        
        progress = min(1.0, elapsed / 3.0) 
        start_error = 200 # pixel
        current_error = start_error * (1.0 - progress)
        
        noise = random.uniform(-5, 5)
        
        cx = (w / 2) + current_error + noise
        cy = (h / 2) + noise
        
        dist = max(0.5, 5.0 - elapsed)
        
        size = int(100 / dist)
        
        return {'cx': cx, 'cy': cy, 'box': [int(cx-size), int(cy-size), int(cx+size), int(cy+size)], 'dist': dist}

    def _calculate_correction(self, mock_obj):
        if not mock_obj: return 0.0
        
        error_px = mock_obj['cx'] - self.image_center_x
        dist_m = mock_obj['dist']
        
        # Hitung Error Meter
        try: error_m = (error_px * dist_m) / self.config.FOCAL_LENGTH_PX
        except: return 0.0
        
        raw_rad = math.atan2(error_m, dist_m)
        
        # Fuzzy / Static P-Gain
        gain = self.config.VISION_P_GAIN
        if FUZZY_ENABLED and self.fuzzy_ctrl:
            try:
                self.fuzzy_ctrl.input['Jarak'] = min(dist_m, 10.0)
                self.fuzzy_ctrl.input['Error'] = abs(error_px)
                self.fuzzy_ctrl.compute()
                gain = self.fuzzy_ctrl.output['P_GAIN']
            except: pass
        
        self.last_used_p_gain = gain
        return raw_rad * gain

    def run(self, data_signal):
        if not self.waypoints: self.current_state = "NO_WAYPOINTS"
        else:
            self.current_state = "WAYPOINT_NAV"
            self.current_lat = self.waypoints[0]['lat']; self.current_lon = self.waypoints[0]['lon']
            
        print("[SIMULATOR] Loop Dimulai...")
        self.running = True
        
        while self.running:
            loop_start = time.time()
            dt = loop_start - self.last_loop_time
            self.last_loop_time = loop_start
            elapsed = time.time() - self.task_timer
            
            # --- LOGIC SIMULATOR ---
            target_yaw = self.current_yaw_rad
            move = False
            jarak_wp = 0.0
            correction_rad = 0.0
            
            if self.current_waypoint_index < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint_index]
                jarak_wp, bear_wp = _get_distance_and_bearing(self.current_lat, self.current_lon, wp['lat'], wp['lon'])
            
            # State Machine Simple
            if self.current_state == "WAYPOINT_NAV":
                self.mock_box = None 
                target_yaw = bear_wp
                move = True
                if jarak_wp < self.config.ACCEPTANCE_RADIUS_M:
                    # Cek Trigger Misi (Hijau/Biru/Merah)
                    if self.current_waypoint_index in self.config.PHOTO_BOX_LEGS: self._set_state("APPROACH_BOX_SEARCH")
                    elif self.current_waypoint_index in self.config.BLUE_BOX_PHOTO_LEGS: self._set_state("APPROACH_BLUE_BOX_SEARCH")
                    elif self.current_waypoint_index == self.config.RED_BOX_NAV_AFTER_WP: self._set_state("APPROACH_RED_BOX_SEARCH")
                    else:
                        self.current_waypoint_index += 1
                        if self.current_waypoint_index >= len(self.waypoints): self._set_state("MISSION_COMPLETE")
                        else: self._set_state("WAYPOINT_TRANSITION")

            elif self.current_state == "WAYPOINT_TRANSITION":
                target_yaw = bear_wp; move = True
                if elapsed > self.config.TRANSITION_DURATION_S: self._set_state("WAYPOINT_NAV")

            elif "SEARCH" in self.current_state:
                if elapsed > 1.5: self._set_state(self.current_state.replace("SEARCH", "ALIGN")) # Ketemu dlm 1.5s
            
            elif "ALIGN" in self.current_state:
                move = True
                self.mock_box = self._generate_mock_vision(self.current_state, elapsed)
                correction_rad = self._calculate_correction(self.mock_box)
                target_yaw = _normalize_angle(self.current_yaw_rad + correction_rad)
                
                # Selesai align jika dekat
                if self.mock_box['dist'] < 1.0: 
                    if "BLUE" in self.current_state: self._set_state("TAKE_BLUE_BOX_PHOTO")
                    elif "RED" in self.current_state: self._set_state("RED_BOX_DOCKED")
                    else: self._set_state("TAKE_PHOTO")

            elif "TAKE" in self.current_state or "DOCKED" in self.current_state:
                if elapsed > 2.0: 
                    if "DOCKED" in self.current_state: 
                        self.current_waypoint_index += 1
                        self._set_state("WAYPOINT_NAV")
                    else: self._set_state("RETREAT") # Mundur

            elif "RETREAT" in self.current_state:
                if elapsed > self.config.RETREAT_DURATION_S:
                    if "BLUE" not in self.current_state: self.current_waypoint_index += 1
                    self._set_state("WAYPOINT_NAV")

            # Physics Update
            if move:
                yaw_diff = _normalize_angle(target_yaw - self.current_yaw_rad)
                self.current_yaw_rad += yaw_diff * dt * 2.0 # Turn rate
                dist = SPEED_MS * dt
                self.current_lat, self.current_lon = _haversine_destination(self.current_lat, self.current_lon, dist, self.current_yaw_rad)

            # --- RENDER FRAME ---
            frame = np.zeros((self.config.FRAME_HEIGHT, self.config.FRAME_WIDTH, 3), dtype=np.uint8)
            cv2.putText(frame, f"SIM STATE: {self.current_state}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame, f"GAIN: {self.last_used_p_gain:.2f}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
            if self.mock_box:
                b = self.mock_box['box']
                cv2.rectangle(frame, (b[0], b[1]), (b[2], b[3]), (0,255,0), 2)
                cv2.line(frame, (int(self.config.FRAME_WIDTH/2), self.config.FRAME_HEIGHT), (int(self.mock_box['cx']), int(self.mock_box['cy'])), (0,0,255), 2)

            # Emit Data
            packet = {
                "lat": self.current_lat, "lon": self.current_lon,
                "yaw_deg": math.degrees(self.current_yaw_rad),
                "pitch_deg": 0.0, "roll_deg": 0.0,
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": jarak_wp, "frame": frame
            }
            data_signal.emit(packet)
            time.sleep(1.0/SEND_RATE_HZ)

    def _set_state(self, new_state):
        if self.current_state != new_state:
            self.current_state = new_state
            self.task_timer = time.time()
            print(f"[SIM] -> {new_state}")

    def stop(self): self.running = False

# ============== #
# THREAD WRAPPER #
# ============== #

class NavigatorThread(QThread):
    newData = pyqtSignal(dict)

    def __init__(self, waypoints, parent=None):
        super().__init__(parent)
        global WAYPOINTS
        WAYPOINTS = waypoints
        self.config = cfg.Config()
        self.navigator = SimOffboardNavigator(self.config)

    # --- INI SOLUSINYA: GANTI 50 FUNGSI DENGAN SATU FUNGSI ---
    def update_config_param(self, key, value):
        if hasattr(self.config, key):
            setattr(self.config, key, value)
            print(f"[TUNING] {key} updated to {value}")
        else:
            print(f"[ERROR] Config key '{key}' not found!")

    def save_config_to_file(self):
        print("[SIMULATOR] Config save simulation called.")

    def run(self):
        try: self.navigator.run(self.newData)
        except Exception as e: print(f"Thread Error: {e}")

    def stop(self):
        if self.navigator: self.navigator.stop()
            
    def update_waypoints(self, new_waypoints):
        global WAYPOINTS
        WAYPOINTS = new_waypoints
        if self.navigator:
            self.navigator.waypoints = new_waypoints
            self.navigator.current_waypoint_index = 0
            self.navigator.current_state = "WAYPOINT_NAV"
            # self.navigator.mock_box = None
            self.navigator.task_timer = time.time()
            if new_waypoints:
                self.navigator.current_lat = new_waypoints[0]['lat']
                self.navigator.current_lon = new_waypoints[0]['lon']
            print("[SIM] Mission Reset.")
