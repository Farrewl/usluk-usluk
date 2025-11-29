import sys, os, math, time, folium, csv, cv2
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial, QFormLayout, QTableWidget, QTableWidgetItem, QDoubleSpinBox, QSpinBox, QScrollArea, QSplitter, QFrame, QPushButton)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QThread, pyqtSignal, QUrl, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

# uncomment aja salah satu
from modules.simulator_backend import NavigatorThread
#from modules.navigator_backend import NavigatorThread

CFG_PATH = "modules/config/"
WAYPOINT_FILE = CFG_PATH + "plan.csv"
TMP_MAP_FILE = CFG_PATH + "temp_map.html"

WAYPOINTS = [] 
try:
    with open(WAYPOINT_FILE, mode='r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            clean_row = {key.strip(): value for key, value in row.items() if key is not None}
            if not clean_row or not clean_row.get('lat') or not clean_row.get('lon'):
                continue
            WAYPOINTS.append({'lat': float(clean_row['lat']), 'lon': float(clean_row['lon'])})
except Exception as e:
    print(f"Warning: Could not load {WAYPOINT_FILE}. {e}")

class MainWindow(QMainWindow):
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ASV Live Dashboard (PyQt5 Version)")
        self.setGeometry(100, 100, 1600, 900)
        self.map_js_ready = False
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.recorded_waypoints = list(WAYPOINTS) 
        self.nav_thread = NavigatorThread(WAYPOINTS, self) 
        map_frame = self._create_map_frame()
        video_widget = self._create_video_widget()
        ahrs_widget = self._create_ahrs_widget()
        monitoring_widget = self._create_monitoring_widget()
        tuning_widget = self._create_tuning_widget()
        recording_widget = self._create_recording_widget()
        left_splitter = QSplitter(Qt.Vertical)
        left_splitter.addWidget(map_frame)
        left_splitter.addWidget(video_widget)
        left_splitter.setSizes([int(self.height() * 0.6), int(self.height() * 0.4)])
        right_splitter = QSplitter(Qt.Vertical)
        right_splitter.addWidget(tuning_widget)
        right_splitter.addWidget(monitoring_widget)
        right_splitter.addWidget(recording_widget) 
        right_splitter.addWidget(ahrs_widget)
        right_splitter.setSizes([int(self.height() * 0.4), int(self.height() * 0.25), int(self.height() * 0.15), int(self.height() * 0.2)])
        main_splitter = QSplitter(Qt.Horizontal)
        main_splitter.addWidget(left_splitter)
        main_splitter.addWidget(right_splitter)
        main_splitter.setSizes([int(self.width() * 0.7), int(self.width() * 0.3)])
        self.setCentralWidget(main_splitter)
        self.nav_thread.newData.connect(self.update_ui)
        self.nav_thread.start()
        
        print("Window created. Loading map widget in 0.5 seconds...")
        QTimer.singleShot(500, self.initialize_map_widget)

    def _create_map_frame(self):
        self.map_frame = QFrame()
        layout = QVBoxLayout(self.map_frame)
        layout.setContentsMargins(0,0,0,0)
        
        self.map_loading_label = QLabel("Loading map...")
        self.map_loading_label.setAlignment(Qt.AlignCenter)
        self.map_loading_label.setStyleSheet("font-size: 16px; color: #888;")
        layout.addWidget(self.map_loading_label)
        
        return self.map_frame

    def initialize_map_widget(self):
        try:
            print("Initializing QWebEngineView...")
            
            if hasattr(self, 'map_view'):
                self.map_view.deleteLater()
            
            self.map_loading_label.show()
            
            self.map_view = QWebEngineView()
            self.map_frame.layout().addWidget(self.map_view)
            
            self._generate_folium_map()
            
            abs_path = os.path.abspath(TMP_MAP_FILE)
            self.map_view.setUrl(QUrl.fromLocalFile(abs_path))
            self.map_view.loadFinished.connect(self._on_map_loaded)
            
            self.map_loading_label.hide()
            print("Map widget loaded successfully.")
            
        except Exception as e:
            print(f"FATAL: Failed to initialize QWebEngineView: {e}")
            self.map_loading_label.setText(f"Error: Failed to load map.\n{e}")
            self.map_loading_label.setStyleSheet("color: red; font-size: 16px;")
            self.map_loading_label.show()

    def _generate_folium_map(self):
        VISION_ENABLED_LEGS = self.nav_thread.config.VISION_ENABLED_LEGS
        
        current_wps = self.nav_thread.navigator.waypoints
        
        if not current_wps:
            avg_lat, avg_lon = -6.9834, 110.4098
            print("Generating empty map at default location.")
        else:
            avg_lat = sum(wp['lat'] for wp in current_wps) / len(current_wps)
            avg_lon = sum(wp['lon'] for wp in current_wps) / len(current_wps)
            
        m = folium.Map(location=[avg_lat, avg_lon], zoom_start=18, tiles="CartoDB positron")
        
        for i, wp in enumerate(current_wps, start=1):
            folium.Marker(
                location=[wp['lat'], wp['lon']], 
                popup=f"Waypoint #{i}\n({wp['lat']:.6f}, {wp['lon']:.6f})", 
                icon=folium.Icon(color='blue', icon='flag')
            ).add_to(m)
            
        if len(current_wps) > 1:
            for i in range(1, len(current_wps)): 
                wp_prev = current_wps[i-1]
                wp_curr = current_wps[i]
                
                locs = [(wp_prev['lat'], wp_prev['lon']), (wp_curr['lat'], wp_curr['lon'])]
                
                is_vision_leg = i in VISION_ENABLED_LEGS 
                
                if is_vision_leg:
                    folium.PolyLine(
                        locations=locs, 
                        color='green', 
                        weight=4, 
                        opacity=0.8,
                        popup=f"Leg (Menuju WP #{i}) (VISION ON)"
                    ).add_to(m)
                else:
                    folium.PolyLine(
                        locations=locs, 
                        color='gray', 
                        weight=3, 
                        opacity=0.8, 
                        dash_array='5, 10',
                        popup=f"Leg (Menuju WP #{i}) (Transit)"
                    ).add_to(m)
            
        m.save(TMP_MAP_FILE)
        print(f"Generated {TMP_MAP_FILE} with vision legs")

    def _on_map_loaded(self):
        start_lat = -6.9834
        start_lon = 110.4098
        
        if self.nav_thread.navigator.waypoints:
            start_lat = self.nav_thread.navigator.waypoints[0]['lat']
            start_lon = self.nav_thread.navigator.waypoints[0]['lon']
            
        js_code = f"""
        (function() {{
            var fa_css = document.createElement('link');
            fa_css.rel = 'stylesheet';
            fa_css.href = 'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css';
            document.head.appendChild(fa_css);

            var vehicleIcon = L.divIcon({{
                html: '<i class="fa fa-arrow-up" style="font-size: 24px; color: #ff0000;"></i>',
                className: 'vehicle-icon',
                iconSize: [24, 24],
                iconAnchor: [12, 12]
            }});

            window.vehicleMarker = L.marker([{start_lat}, {start_lon}], {{
                icon: vehicleIcon,
                rotationAngle: 0,
                rotationOrigin: 'center center'
            }}).addTo(map);

            window.updateVehiclePosition = function(lat, lon, yaw_deg) {{
                var newLatLng = L.latLng(lat, lon);
                window.vehicleMarker.setLatLng(newLatLng);
                window.vehicleMarker.setRotationAngle(yaw_deg);
            }};

            window.addWaypointMarker = function(lat, lon, text) {{
                L.marker([lat, lon], {{
                    icon: L.icon({{
                        iconUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-icon.png',
                        shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.7.1/images/marker-shadow.png',
                        iconAnchor: [12, 41],
                        popupAnchor: [1, -34]
                    }})
                }}).addTo(map).bindPopup(text);
            }};
            
        }})();
        """
        if hasattr(self, 'map_view'):
            self.map_view.page().runJavaScript(js_code)
        
        self.map_js_ready = True
# (Class Config Anda tidak berubah, ini sudah benar)
        print("Map JavaScript is ready.")

    def _create_video_widget(self):
        self.video_label = QLabel("Waiting for video feed...")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.video_label.setStyleSheet("background-color: #000; color: #FFF;")
        return self.video_label

    def _create_ahrs_widget(self):
        widget = QWidget()
        layout = QFormLayout(widget)
        
        self.yaw_dial = QDial()
        self.yaw_dial.setRange(-180, 180); self.yaw_dial.setWrapping(True); self.yaw_dial.setEnabled(False); self.yaw_dial.setNotchesVisible(True)
        layout.addRow("Yaw:", self.yaw_dial)
        
        self.pitch_dial = QDial()
        self.pitch_dial.setRange(-90, 90); self.pitch_dial.setEnabled(False); self.pitch_dial.setNotchesVisible(True)
        layout.addRow("Pitch:", self.pitch_dial)
        
        self.roll_dial = QDial()
        self.roll_dial.setRange(-180, 180); self.roll_dial.setWrapping(True); self.roll_dial.setEnabled(False); self.roll_dial.setNotchesVisible(True)
        layout.addRow("Roll:", self.roll_dial)
        
        widget.setMinimumHeight(200)
        return widget

    def _create_monitoring_widget(self):
        self.monitor_table = QTableWidget()
        self.monitor_table.setRowCount(5)
        self.monitor_table.setColumnCount(1)
        self.monitor_table.setVerticalHeaderLabels(["State", "Latitude", "Longitude", "Target WP", "Dist to WP (m)"])
        self.monitor_table.horizontalHeader().setVisible(False)
        self.monitor_table.horizontalHeader().setStretchLastSection(True)
        return self.monitor_table

    # =======================================================================
    # === FUNGSI DI BAWAH INI TELAH DIGANTI DENGAN VERSI LENGKAP ===
    # =======================================================================
    def _create_tuning_widget(self):
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        widget = QWidget()
        self.tuning_layout = QFormLayout(widget)
        self.tuning_layout.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)
    
        # Helper untuk membuat header
        def add_header(text):
            label = QLabel(text)
            label.setStyleSheet("font-size: 14px; font-weight: bold; margin-top: 10px;")
            self.tuning_layout.addRow(label)
    
        # === NAVIGASI UMUM ===
        add_header("Navigasi Umum & GPS")
        self.tune_acceptance_radius = QDoubleSpinBox()
        self.tune_acceptance_radius.setRange(0.5, 10.0); self.tune_acceptance_radius.setSingleStep(0.1)
        self.tune_acceptance_radius.setValue(self.nav_thread.config.ACCEPTANCE_RADIUS_M)
        self.tuning_layout.addRow("ACCEPTANCE_RADIUS_M:", self.tune_acceptance_radius)
    
        self.tune_thrust = QDoubleSpinBox()
        self.tune_thrust.setRange(0, 1.0); self.tune_thrust.setSingleStep(0.05)
        self.tune_thrust.setValue(self.nav_thread.config.THRUST_VALUE)
        self.tuning_layout.addRow("THRUST_VALUE (Transit):", self.tune_thrust)
    
        self.tune_transition_duration = QDoubleSpinBox()
        self.tune_transition_duration.setRange(0, 5.0); self.tune_transition_duration.setSingleStep(0.1)
        self.tune_transition_duration.setValue(self.nav_thread.config.TRANSITION_DURATION_S)
        self.tuning_layout.addRow("TRANSITION_DURATION_S:", self.tune_transition_duration)
    
        self.tune_geofence_width = QDoubleSpinBox()
        self.tune_geofence_width.setRange(0.5, 10.0); self.tune_geofence_width.setSingleStep(0.1)
        self.tune_geofence_width.setValue(self.nav_thread.config.GEOFENCE_WIDTH_METERS)
        self.tuning_layout.addRow("GEOFENCE_WIDTH_METERS:", self.tune_geofence_width)
    
        # === PARAMETER VISION BERSAMA ===
        add_header("Parameter Vision (Global)")
        self.tune_focalpx = QSpinBox() # Focal length biasanya integer
        self.tune_focalpx.setRange(100, 2000); self.tune_focalpx.setSingleStep(10)
        self.tune_focalpx.setValue(self.nav_thread.config.FOCAL_LENGTH_PX)
        self.tuning_layout.addRow("FOCAL_LENGTH_PX:", self.tune_focalpx)
    
        self.tune_p_gain = QDoubleSpinBox()
        self.tune_p_gain.setRange(0.0, 10.0); self.tune_p_gain.setSingleStep(0.1)
        self.tune_p_gain.setValue(self.nav_thread.config.VISION_P_GAIN)
        self.tuning_layout.addRow("VISION_P_GAIN:", self.tune_p_gain)
    
        self.tune_smoothing_alpha = QDoubleSpinBox()
        self.tune_smoothing_alpha.setRange(0.0, 1.0); self.tune_smoothing_alpha.setSingleStep(0.05)
        self.tune_smoothing_alpha.setValue(self.nav_thread.config.VISION_SMOOTHING_ALPHA)
        self.tuning_layout.addRow("VISION_SMOOTHING_ALPHA:", self.tune_smoothing_alpha)
    
        self.tune_roi_cutoff = QDoubleSpinBox()
        self.tune_roi_cutoff.setRange(0.0, 1.0); self.tune_roi_cutoff.setSingleStep(0.05)
        self.tune_roi_cutoff.setValue(self.nav_thread.config.ROI_TOP_CUTOFF_PERCENT)
        self.tuning_layout.addRow("ROI_TOP_CUTOFF_PERCENT:", self.tune_roi_cutoff)
    
        # === MISI GATE (BUOY) ===
        add_header("Misi Gate (Buoy Merah/Hijau)")
        self.tune_gate_width = QDoubleSpinBox()
        self.tune_gate_width.setRange(0.5, 5.0); self.tune_gate_width.setSingleStep(0.1)
        self.tune_gate_width.setValue(self.nav_thread.config.GATE_WIDTH_METERS)
        self.tuning_layout.addRow("GATE_WIDTH_METERS:", self.tune_gate_width)
    
        self.tune_min_buoy_area = QSpinBox()
        self.tune_min_buoy_area.setRange(0, 5000); self.tune_min_buoy_area.setSingleStep(10)
        self.tune_min_buoy_area.setValue(self.nav_thread.config.MIN_BUOY_AREA_PX)
        self.tuning_layout.addRow("MIN_BUOY_AREA_PX:", self.tune_min_buoy_area)
    
        self.tune_gate_area_ratio = QDoubleSpinBox()
        self.tune_gate_area_ratio.setRange(0.0, 1.0); self.tune_gate_area_ratio.setSingleStep(0.05)
        self.tune_gate_area_ratio.setValue(self.nav_thread.config.GATE_AREA_SIMILARITY_RATIO)
        self.tuning_layout.addRow("GATE_AREA_SIMILARITY_RATIO:", self.tune_gate_area_ratio)
    
        # === MISI BOX HIJAU ===
        add_header("Misi Foto Box Hijau")
        self.tune_search_thrust = QDoubleSpinBox()
        self.tune_search_thrust.setRange(0, 1.0); self.tune_search_thrust.setSingleStep(0.05)
        self.tune_search_thrust.setValue(self.nav_thread.config.SEARCH_THRUST)
        self.tuning_layout.addRow("SEARCH_THRUST (Hijau):", self.tune_search_thrust)
    
        self.tune_align_thrust = QDoubleSpinBox()
        self.tune_align_thrust.setRange(0, 1.0); self.tune_align_thrust.setSingleStep(0.05)
        self.tune_align_thrust.setValue(self.nav_thread.config.ALIGN_THRUST)
        self.tuning_layout.addRow("ALIGN_THRUST (Hijau):", self.tune_align_thrust)
        
        self.tune_retreat_thrust = QDoubleSpinBox()
        self.tune_retreat_thrust.setRange(-1.0, 0.0); self.tune_retreat_thrust.setSingleStep(0.05)
        self.tune_retreat_thrust.setValue(self.nav_thread.config.RETREAT_THRUST)
        self.tuning_layout.addRow("RETREAT_THRUST (Hijau):", self.tune_retreat_thrust)
    
        self.tune_retreat_dur = QDoubleSpinBox()
        self.tune_retreat_dur.setRange(0.0, 10.0); self.tune_retreat_dur.setSingleStep(0.1)
        self.tune_retreat_dur.setValue(self.nav_thread.config.RETREAT_DURATION_S)
        self.tuning_layout.addRow("RETREAT_DURATION_S (Hijau):", self.tune_retreat_dur)
    
        self.tune_box_width = QDoubleSpinBox()
        self.tune_box_width.setRange(0.1, 2.0); self.tune_box_width.setSingleStep(0.05)
        self.tune_box_width.setValue(self.nav_thread.config.BOX_WIDTH_METERS)
        self.tuning_layout.addRow("BOX_WIDTH_METERS (Hijau):", self.tune_box_width)
    
        self.tune_box_approach_dist = QDoubleSpinBox()
        self.tune_box_approach_dist.setRange(0.5, 5.0); self.tune_box_approach_dist.setSingleStep(0.1)
        self.tune_box_approach_dist.setValue(self.nav_thread.config.BOX_APPROACH_DISTANCE_M)
        self.tuning_layout.addRow("BOX_APPROACH_DISTANCE_M (Hijau):", self.tune_box_approach_dist)
        
        self.tune_yaw_search_box = QSpinBox()
        self.tune_yaw_search_box.setRange(-180, 180); self.tune_yaw_search_box.setSingleStep(5)
        self.tune_yaw_search_box.setValue(self.nav_thread.config.YAW_SEARCH_BOX)
        self.tuning_layout.addRow("YAW_SEARCH_BOX (Hijau):", self.tune_yaw_search_box)
        
        self.tune_box_lat_thrust = QDoubleSpinBox()
        self.tune_box_lat_thrust.setRange(-1.0, 1.0); self.tune_box_lat_thrust.setSingleStep(0.05)
        self.tune_box_lat_thrust.setValue(self.nav_thread.config.BOX_SEARCH_LATERAL_THRUST)
        self.tuning_layout.addRow("BOX_SEARCH_LATERAL_THRUST:", self.tune_box_lat_thrust)
    
        # === MISI BOX BIRU ===
        add_header("Misi Foto Box Biru")
        self.tune_blue_box_search_thrust = QDoubleSpinBox()
        self.tune_blue_box_search_thrust.setRange(0, 1.0); self.tune_blue_box_search_thrust.setSingleStep(0.05)
        self.tune_blue_box_search_thrust.setValue(self.nav_thread.config.BLUE_BOX_SEARCH_THRUST)
        self.tuning_layout.addRow("SEARCH_THRUST (Biru):", self.tune_blue_box_search_thrust)
    
        self.tune_blue_box_align_thrust = QDoubleSpinBox()
        self.tune_blue_box_align_thrust.setRange(0, 1.0); self.tune_blue_box_align_thrust.setSingleStep(0.05)
        self.tune_blue_box_align_thrust.setValue(self.nav_thread.config.BLUE_BOX_ALIGN_THRUST)
        self.tuning_layout.addRow("ALIGN_THRUST (Biru):", self.tune_blue_box_align_thrust)
    
        self.tune_blue_box_width = QDoubleSpinBox()
        self.tune_blue_box_width.setRange(0.1, 2.0); self.tune_blue_box_width.setSingleStep(0.05)
        self.tune_blue_box_width.setValue(self.nav_thread.config.BLUE_BOX_WIDTH_METERS)
        self.tuning_layout.addRow("BOX_WIDTH_METERS (Biru):", self.tune_blue_box_width)
    
        self.tune_blue_box_approach_dist = QDoubleSpinBox()
        self.tune_blue_box_approach_dist.setRange(0.5, 5.0); self.tune_blue_box_approach_dist.setSingleStep(0.1)
        self.tune_blue_box_approach_dist.setValue(self.nav_thread.config.BLUE_BOX_APPROACH_DISTANCE_M)
        self.tuning_layout.addRow("APPROACH_DISTANCE_M (Biru):", self.tune_blue_box_approach_dist)
    
        self.tune_blue_box_lat_offset = QDoubleSpinBox()
        self.tune_blue_box_lat_offset.setRange(-5.0, 5.0); self.tune_blue_box_lat_offset.setSingleStep(0.1)
        self.tune_blue_box_lat_offset.setValue(self.nav_thread.config.BLUE_BOX_LATERAL_OFFSET_M)
        self.tuning_layout.addRow("LATERAL_OFFSET_M (Biru):", self.tune_blue_box_lat_offset)
        
        self.tune_blue_box_yaw_search = QSpinBox()
        self.tune_blue_box_yaw_search.setRange(-180, 180); self.tune_blue_box_yaw_search.setSingleStep(5)
        self.tune_blue_box_yaw_search.setValue(self.nav_thread.config.BLUE_BOX_YAW_SEARCH)
        self.tuning_layout.addRow("YAW_SEARCH (Biru):", self.tune_blue_box_yaw_search)
    
        # === MISI DOCKING (BOX MERAH) ===
        add_header("Misi Docking (Box Merah)")
        self.tune_dock_align_thrust = QDoubleSpinBox()
        self.tune_dock_align_thrust.setRange(0, 1.0); self.tune_dock_align_thrust.setSingleStep(0.05)
        self.tune_dock_align_thrust.setValue(self.nav_thread.config.DOCK_ALIGN_THRUST)
        self.tuning_layout.addRow("DOCK_ALIGN_THRUST:", self.tune_dock_align_thrust)
        
        self.tune_dock_hold_dur = QDoubleSpinBox()
        self.tune_dock_hold_dur.setRange(0.0, 20.0); self.tune_dock_hold_dur.setSingleStep(0.5)
        self.tune_dock_hold_dur.setValue(self.nav_thread.config.DOCK_HOLD_DURATION_S)
        self.tuning_layout.addRow("DOCK_HOLD_DURATION_S:", self.tune_dock_hold_dur)
    
        self.tune_red_box_width = QDoubleSpinBox()
        self.tune_red_box_width.setRange(0.1, 2.0); self.tune_red_box_width.setSingleStep(0.05)
        self.tune_red_box_width.setValue(self.nav_thread.config.RED_BOX_WIDTH_METERS)
        self.tuning_layout.addRow("BOX_WIDTH_METERS (Merah):", self.tune_red_box_width)
    
        self.tune_red_box_dock_dist = QDoubleSpinBox()
        self.tune_red_box_dock_dist.setRange(0.1, 5.0); self.tune_red_box_dock_dist.setSingleStep(0.1)
        self.tune_red_box_dock_dist.setValue(self.nav_thread.config.RED_BOX_DOCK_DISTANCE_M)
        self.tuning_layout.addRow("DOCK_DISTANCE_M (Merah):", self.tune_red_box_dock_dist)
    
        self.tune_yaw_search_dock = QSpinBox()
        self.tune_yaw_search_dock.setRange(-180, 180); self.tune_yaw_search_dock.setSingleStep(5)
        self.tune_yaw_search_dock.setValue(self.nav_thread.config.YAW_SEARCH_DOCK)
        self.tuning_layout.addRow("YAW_SEARCH_DOCK (Merah):", self.tune_yaw_search_dock)
    
        # === PARAMETER YOLO & VIDEO ===
        add_header("YOLO, Video, & Sesi")
        self.tune_yolo_frame_skip = QSpinBox()
        self.tune_yolo_frame_skip.setRange(0, 10); self.tune_yolo_frame_skip.setSingleStep(1)
        self.tune_yolo_frame_skip.setValue(self.nav_thread.config.YOLO_FRAME_SKIP)
        self.tuning_layout.addRow("YOLO_FRAME_SKIP:", self.tune_yolo_frame_skip)
    
        self.tune_yolo_inf_size = QSpinBox()
        self.tune_yolo_inf_size.setRange(320, 1280); self.tune_yolo_inf_size.setSingleStep(32)
        self.tune_yolo_inf_size.setValue(self.nav_thread.config.YOLO_INFERENCE_SIZE)
        self.tuning_layout.addRow("YOLO_INFERENCE_SIZE:", self.tune_yolo_inf_size)
    
        self.tune_session_video_fps = QDoubleSpinBox()
        self.tune_session_video_fps.setRange(1.0, 30.0); self.tune_session_video_fps.setSingleStep(1.0)
        self.tune_session_video_fps.setValue(self.nav_thread.config.SESSION_VIDEO_FPS)
        self.tuning_layout.addRow("SESSION_VIDEO_FPS:", self.tune_session_video_fps)
    
        
        # === KONEKSI SINYAL (WAJIB DIPERBARUI SEMUA) ===
        scroll_area.setWidget(widget)
        
        # Navigasi Umum
        self.tune_acceptance_radius.valueChanged.connect(self.nav_thread.update_acceptance_radius)
        self.tune_thrust.valueChanged.connect(self.nav_thread.update_thrust)
        self.tune_transition_duration.valueChanged.connect(self.nav_thread.update_transition_duration)
        self.tune_geofence_width.valueChanged.connect(self.nav_thread.update_geofence_width)
    
        # Vision Global
        self.tune_focalpx.valueChanged.connect(self.nav_thread.update_focalpx)
        self.tune_p_gain.valueChanged.connect(self.nav_thread.update_p_gain)
        self.tune_smoothing_alpha.valueChanged.connect(self.nav_thread.update_smoothing_alpha)
        self.tune_roi_cutoff.valueChanged.connect(self.nav_thread.update_roi_cutoff)
    
        # Misi Gate (Buoy)
        self.tune_gate_width.valueChanged.connect(self.nav_thread.update_gate_width)
        self.tune_min_buoy_area.valueChanged.connect(self.nav_thread.update_min_buoy_area)
        self.tune_gate_area_ratio.valueChanged.connect(self.nav_thread.update_gate_area_ratio)
    
        # Misi Box Hijau
        self.tune_search_thrust.valueChanged.connect(self.nav_thread.update_search_thrust)
        self.tune_align_thrust.valueChanged.connect(self.nav_thread.update_align_thrust)
        self.tune_retreat_thrust.valueChanged.connect(self.nav_thread.update_retreat_thrust)
        self.tune_retreat_dur.valueChanged.connect(self.nav_thread.update_retreat_duration)
        self.tune_box_width.valueChanged.connect(self.nav_thread.update_box_width)
        self.tune_box_approach_dist.valueChanged.connect(self.nav_thread.update_box_approach_dist)
        self.tune_yaw_search_box.valueChanged.connect(self.nav_thread.update_yaw_search_box)
        self.tune_box_lat_thrust.valueChanged.connect(self.nav_thread.update_box_lat_thrust)
    
        # Misi Box Biru
        self.tune_blue_box_search_thrust.valueChanged.connect(self.nav_thread.update_blue_box_search_thrust)
        self.tune_blue_box_align_thrust.valueChanged.connect(self.nav_thread.update_blue_box_align_thrust)
        self.tune_blue_box_width.valueChanged.connect(self.nav_thread.update_blue_box_width)
        self.tune_blue_box_approach_dist.valueChanged.connect(self.nav_thread.update_blue_box_approach_dist)
        self.tune_blue_box_lat_offset.valueChanged.connect(self.nav_thread.update_blue_box_lat_offset)
        self.tune_blue_box_yaw_search.valueChanged.connect(self.nav_thread.update_blue_box_yaw_search)
    
        # Misi Docking (Box Merah)
        self.tune_dock_align_thrust.valueChanged.connect(self.nav_thread.update_dock_align_thrust)
        self.tune_dock_hold_dur.valueChanged.connect(self.nav_thread.update_dock_hold_dur)
        self.tune_red_box_width.valueChanged.connect(self.nav_thread.update_red_box_width)
        self.tune_red_box_dock_dist.valueChanged.connect(self.nav_thread.update_red_box_dock_dist)
        self.tune_yaw_search_dock.valueChanged.connect(self.nav_thread.update_yaw_search_dock)
    
        # YOLO & Video
        self.tune_yolo_frame_skip.valueChanged.connect(self.nav_thread.update_yolo_frame_skip)
        self.tune_yolo_inf_size.valueChanged.connect(self.nav_thread.update_yolo_inf_size)
        self.tune_session_video_fps.valueChanged.connect(self.nav_thread.update_session_video_fps)
    
        return scroll_area
    # =======================================================================
    # === AKHIR DARI FUNGSI YANG DIGANTI ===
    # =======================================================================
        
    def _create_recording_widget(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        title = QLabel("Waypoint Recorder")
        title.setStyleSheet("font-size: 14px; font-weight: bold;")
        layout.addWidget(title)
        
        button_layout = QHBoxLayout()
        
        self.record_wp_btn = QPushButton("Record Position")
        self.save_wp_btn = QPushButton("Save & Reload")
        self.clear_wp_btn = QPushButton("Clear New")
        
        button_layout.addWidget(self.record_wp_btn)
        button_layout.addWidget(self.save_wp_btn)
        button_layout.addWidget(self.clear_wp_btn)
        
        layout.addLayout(button_layout)
        
        self.wp_status_label = QLabel("Ready to record.")
        self.wp_status_label.setStyleSheet("font-style: italic;")
        layout.addWidget(self.wp_status_label)
        
        self.record_wp_btn.clicked.connect(self.on_record_waypoint)
        self.save_wp_btn.clicked.connect(self.on_save_waypoints)
        self.clear_wp_btn.clicked.connect(self.on_clear_waypoints)
        
        return widget

    def update_ui(self, data):
        self.current_lat = data['lat']
        self.current_lon = data['lon']
        
        if hasattr(self, 'map_view') and self.map_js_ready:
            self.map_view.page().runJavaScript(
                f"updateVehiclePosition({data['lat']}, {data['lon']}, {data['yaw_deg']});"
            )
        
        frame = data['frame']
        h, w, ch = frame.shape
        if h > 0 and w > 0:
            bytes_per_line = ch * w
            q_img = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            pixmap = QPixmap.fromImage(q_img)
            
            label_w = self.video_label.width()
            label_h = self.video_label.height()
            if label_w > 1 and label_h > 1:
                scaled_pixmap = pixmap.scaled(label_w, label_h, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(scaled_pixmap)

        self.yaw_dial.setValue(int(data['yaw_deg']))
        self.pitch_dial.setValue(int(data['pitch_deg']))
        self.roll_dial.setValue(int(data['roll_deg']))
        
        self.monitor_table.setItem(0, 0, QTableWidgetItem(data['state']))
        self.monitor_table.setItem(1, 0, QTableWidgetItem(f"{data['lat']:.7f}"))
        self.monitor_table.setItem(2, 0, QTableWidgetItem(f"{data['lon']:.7f}"))
        
        target_idx_display_str = "N/A"
        if data['state'] not in ["NO_WAYPOINTS", "NO_TELEM", "WAITING_GPS"]:
            target_idx_display_str = str(data['target_wp_idx'] + 1)
        elif data['state'] == "NO_WAYPOINTS":
            target_idx_display_str = "0"
        
        self.monitor_table.setItem(3, 0, QTableWidgetItem(target_idx_display_str))
        self.monitor_table.setItem(4, 0, QTableWidgetItem(f"{data['dist_to_wp_m']:.2f}"))

    def on_record_waypoint(self):
        if self.current_lat == 0.0 and self.current_lon == 0.0 and self.nav_thread.navigator.current_state != "NO_WAYPOINTS":
            self.wp_status_label.setText("Error: No position data yet.")
            return
            
        lat = self.current_lat
        lon = self.current_lon
        
        wp_data = {'lat': lat, 'lon': lon}
        self.recorded_waypoints.append(wp_data)
        
        wp_index = len(self.recorded_waypoints) - 1
        
        text = f"New WP #{wp_index + 1} ({lat:.6f}, {lon:.6f})"
        
        self.wp_status_label.setText(f"Recorded: {text}")
        
        if hasattr(self, 'map_view') and self.map_js_ready:
            self.map_view.page().runJavaScript(f"addWaypointMarker({lat}, {lon}, '{text}');")

    def on_save_waypoints(self):
        wps_to_save = self.recorded_waypoints

        try:
            with open(WAYPOINT_FILE, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=['lat', 'lon'])
                writer.writeheader()
                writer.writerows(wps_to_save)

            self.wp_status_label.setText(f"Saved {len(wps_to_save)} WPs. Reloading mission...")
            
            self.nav_thread.update_waypoints(wps_to_save)
            
            self.initialize_map_widget()
            
        except Exception as e:
            self.wp_status_label.setText(f"Error saving: {e}")

    def on_clear_waypoints(self):
        self.recorded_waypoints.clear()
        self.wp_status_label.setText("Cleared. Press 'Save' to commit empty list.")
        
        self.initialize_map_widget()
            
    def closeEvent(self, event):
        print("Closing application...")
        self.nav_thread.stop()
        self.nav_thread.wait()
        
        # --- PERUBAHAN DI SINI: MENAMBAHKAN PENYIMPANAN KONFIGURASI ---
        if hasattr(self.nav_thread, 'save_config_to_file'):
            print("Menyimpan parameter tuning terakhir...")
            self.nav_thread.save_config_to_file()
        # --------------------------------------------------------
        
        if os.path.exists(TMP_MAP_FILE):
            os.remove(TMP_MAP_FILE)
            
        event.accept()

if __name__ == "__main__":
    if not WAYPOINTS:
        print("WARNING: 'plan.csv' is empty or not found.")
        print("Application will start with an empty map.")
        print("Use the 'Waypoint Recorder' to add waypoints.")

    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
