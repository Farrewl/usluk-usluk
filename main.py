import sys, os, math, time, folium, csv, cv2, json
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDial, QFormLayout, QTableWidget, QTableWidgetItem, QDoubleSpinBox, QSpinBox, QScrollArea, QSplitter, QFrame, QPushButton, QLineEdit)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtCore import QThread, pyqtSignal, QUrl, QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtCore import pyqtSlot, QObject

# uncomment aja salah satu
from modules.simulation_in_ground import NavigatorThread
# from modules.navigator_backend import NavigatorThread

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

class MapBridge(QObject):
    wpMoved = pyqtSignal(int, float, float)
    @pyqtSlot(int, float, float)
    def update_waypoint_pos(self, index, lat, lon):
        print(f"[MapBridge] Waypoint #{index+1} moved to: {lat}, {lon}")
        self.wpMoved.emit(index, lat, lon)

class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("ASV Live Dashboard (PyQt5 Version)")
        self.setGeometry(100, 100, 1600, 900)
        self.map_js_ready = False
        self.map_bridge = MapBridge()
        self.map_bridge.wpMoved.connect(self.on_waypoint_dragged)
        self.map_channel = QWebChannel()
        self.map_channel.registerObject("bridge", self.map_bridge)
        self.current_lat = 0.0
        self.current_lon = 0.0
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
            self.map_view.page().setWebChannel(self.map_channel)
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
          
        if len(current_wps) > 1:
            for i in range(1, len(current_wps)): 
                wp_prev = current_wps[i-1]
                wp_curr = current_wps[i]
                
                locs = [(wp_prev['lat'], wp_prev['lon']), (wp_curr['lat'], wp_curr['lon'])]
                
                is_vision_leg = i in VISION_ENABLED_LEGS 
                
                if is_vision_leg:
                    folium.PolyLine(locations=locs, color='green', weight=4, opacity=0.8, popup=f"Leg (Menuju WP #{i}) (VISION ON)").add_to(m)
                else:
                    folium.PolyLine(locations=locs, color='gray', weight=3, opacity=0.8, dash_array='5, 10', popup=f"Leg (Menuju WP #{i}) (TRANSIT)").add_to(m)
            
        m.save(TMP_MAP_FILE)
        print(f"Generated {TMP_MAP_FILE} with vision legs")

    def _on_map_loaded(self):
        start_lat = -6.9834
        start_lon = 110.4098
        
        if self.nav_thread.navigator.waypoints:
            start_lat = self.nav_thread.navigator.waypoints[0]['lat']
            start_lon = self.nav_thread.navigator.waypoints[0]['lon']
        
        current_wps_json = json.dumps(self.nav_thread.navigator.waypoints)
        
        js_code = f"""
        (function() {{
            var map = null;
            for (var key in window) {{
                if (window[key] instanceof L.Map) {{
                    map = window[key];
                    break;
                }}
            }}

            if (!map) {{
                console.error("Leaflet Map instance not found!");
                return;
            }}

            if (typeof QWebChannel !== "undefined") {{
                new QWebChannel(qt.webChannelTransport, function(channel) {{
                    window.pyBridge = channel.objects.bridge;
                    console.log("Bridge connected!");
                    initDraggableMarkers({current_wps_json});
                }});
            }} else {{
                console.error("QWebChannel not loaded!");
            }}

            window.markers = [];

            function initDraggableMarkers(waypoints) {{
                // Hapus marker lama jika ada untuk mencegah duplikasi
                if (window.markers) {{
                    window.markers.forEach(m => map.removeLayer(m));
                }}
                window.markers = [];

                waypoints.forEach(function(wp, index) {{
                    // Membuat Marker yang DRAGGABLE
                    var marker = L.marker([wp.lat, wp.lon], {{
                        draggable: true,
                        title: "WP #" + (index + 1)
                    }}).addTo(map); // Pastikan ditambahkan ke 'map' yang sudah ditemukan

                    marker.bindPopup("<b>Waypoint #" + (index + 1) + "</b><br>Drag to move");

                    // Event saat selesai geser -> Kirim ke Python
                    marker.on('dragend', function(e) {{
                        var newPos = e.target.getLatLng();
                        console.log("WP Dragged:", index, newPos.lat, newPos.lng);
                        window.pyBridge.update_waypoint_pos(index, newPos.lat, newPos.lng);
                    }});

                    window.markers.push(marker);
                }});
            }}

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
                if(window.vehicleMarker){{
                    var newLatLng = L.latLng(lat, lon);
                    window.vehicleMarker.setLatLng(newLatLng);
                    var iconDiv = window.vehicleMarker.getElement();
                    if(iconDiv) {{
                        var iTag = iconDiv.querySelector('i');
                        if(iTag) {{
                            iTag.style.transform = 'rotate(' + yaw_deg + 'deg)';
                        }}
                    }}
                }}
            }};

        }})();
        """

        qwebchannel_js = "qrc:///qtwebchannel/qwebchannel.js"

        loader_js = f"""
        var script = document.createElement('script');
        script.src = "{qwebchannel_js}";
        script.onload = function() {{
            {js_code}
        }};
        document.head.appendChild(script);
        """
        
        if hasattr(self, 'map_view'):
            self.map_view.page().runJavaScript(loader_js)
        
        self.map_js_ready = True
        print("Map JavaScript is ready.")

    def on_waypoint_dragged(self, index, new_lat, new_lon):
        if index < len(self.nav_thread.navigator.waypoints):
            self.nav_thread.navigator.waypoints[index]['lat'] = new_lat
            self.nav_thread.navigator.waypoints[index]['lon'] = new_lon
            print(f"[GUI] Updated WP #{index+1} in memory.")

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

    def _add_tuning_list(self, label_text, config_key):
        widget = QLineEdit()
        widget.setPlaceholderText("Contoh: 1, 3, 5")
        current_list = getattr(self.nav_thread.config, config_key, [])
        if isinstance(current_list, list): widget.setText(", ".join(map(str, current_list)))

        def save_list():
            text = widget.text()
            try:
                new_list = [int(x.strip()) for x in text.split(',') if x.strip().isdigit()]
                print(f"[GUI] Updating list {config_key} -> {new_list}")
                self.nav_thread.update_config_param(config_key, new_list)
                widget.setText(", ".join(map(str, new_list)))
            except Exception as e:
                print(f"Error parsing list: {e}")

        widget.editingFinished.connect(save_list)
        self.tuning_layout.addRow(label_text, widget)
        return widget

    def _add_tuning_row(self, label_text, config_key, min_val, max_val, step, is_int=False):
        widget = QSpinBox() if is_int else QDoubleSpinBox()
        widget.setRange(min_val, max_val)
        widget.setSingleStep(step)
        current_val = getattr(self.nav_thread.config, config_key, 0)
        widget.setValue(current_val)
        widget.valueChanged.connect(lambda v, k=config_key: self.nav_thread.update_config_param(k, v))
        self.tuning_layout.addRow(label_text, widget)
        return widget

    def _add_header(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size: 14px; font-weight: bold; margin-top: 10px;")
        self.tuning_layout.addRow(label)

    def _create_tuning_widget(self):
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        widget = QWidget()

        self.tuning_layout = QFormLayout(widget)
        self.tuning_layout.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        self._add_header("Navigasi Umum & GPS")
        self.tune_acceptance_radius = self._add_tuning_row("ACCEPTANCE_RADIUS_M:", "ACCEPTANCE_RADIUS_M", 0.5, 10.0, 0.1)
        self.tune_thrust = self._add_tuning_row("THRUST_VALUE (Transit):", "THRUST_VALUE", 0.0, 1.0, 0.05)
        self.tune_transition_duration = self._add_tuning_row("TRANSITION_DURATION_S:", "TRANSITION_DURATION_S", 0.0, 5.0, 0.1)
        self.tune_geofence_width = self._add_tuning_row("GEOFENCE_WIDTH_METERS:", "GEOFENCE_WIDTH_METERS", 0.5, 10.0, 0.1)

        self._add_header("Parameter Vision (Global)")
        self.tune_focalpx = self._add_tuning_row("FOCAL_LENGTH_PX:", "FOCAL_LENGTH_PX", 100, 2000, 10, is_int=True)
        self.tune_p_gain = self._add_tuning_row("VISION_P_GAIN:", "VISION_P_GAIN", 0.0, 10.0, 0.1)
        self.tune_smoothing_alpha = self._add_tuning_row("VISION_SMOOTHING_ALPHA:", "VISION_SMOOTHING_ALPHA", 0.0, 1.0, 0.05)
        self.tune_roi_cutoff = self._add_tuning_row("ROI_TOP_CUTOFF_PERCENT:", "ROI_TOP_CUTOFF_PERCENT", 0.0, 1.0, 0.05)

        self._add_header("Misi Gate (Buoy)")
        self.tune_gate_width = self._add_tuning_row("GATE_WIDTH_METERS:", "GATE_WIDTH_METERS", 0.5, 5.0, 0.1)
        self.tune_min_buoy_area = self._add_tuning_row("MIN_BUOY_AREA_PX:", "MIN_BUOY_AREA_PX", 0, 5000, 10, is_int=True)
        self.tune_gate_area_ratio = self._add_tuning_row("GATE_AREA_SIMILARITY_RATIO:", "GATE_AREA_SIMILARITY_RATIO", 0.0, 1.0, 0.05)

        self._add_header("Misi Foto Box Hijau")
        self.tune_search_thrust = self._add_tuning_row("SEARCH_THRUST:", "SEARCH_THRUST", 0.0, 1.0, 0.05)
        self.tune_align_thrust = self._add_tuning_row("ALIGN_THRUST:", "ALIGN_THRUST", 0.0, 1.0, 0.05)
        self.tune_retreat_thrust = self._add_tuning_row("RETREAT_THRUST:", "RETREAT_THRUST", -1.0, 0.0, 0.05)
        self.tune_retreat_dur = self._add_tuning_row("RETREAT_DURATION_S:", "RETREAT_DURATION_S", 0.0, 10.0, 0.1)
        self.tune_box_width = self._add_tuning_row("BOX_WIDTH_METERS:", "BOX_WIDTH_METERS", 0.1, 2.0, 0.05)
        self.tune_box_approach_dist = self._add_tuning_row("APPROACH_DISTANCE_M:", "BOX_APPROACH_DISTANCE_M", 0.5, 5.0, 0.1)
        self.tune_yaw_search_box = self._add_tuning_row("YAW_SEARCH_BOX:", "YAW_SEARCH_BOX", -180, 180, 5, is_int=True)
        self.tune_box_lat_thrust = self._add_tuning_row("LATERAL_THRUST:", "BOX_SEARCH_LATERAL_THRUST", -1.0, 1.0, 0.05)

        self._add_header("Misi Foto Box Biru")
        self.tune_blue_box_search_thrust = self._add_tuning_row("SEARCH_THRUST:", "BLUE_BOX_SEARCH_THRUST", 0.0, 1.0, 0.05)
        self.tune_blue_box_align_thrust = self._add_tuning_row("ALIGN_THRUST:", "BLUE_BOX_ALIGN_THRUST", 0.0, 1.0, 0.05)
        self.tune_blue_box_width = self._add_tuning_row("BOX_WIDTH_METERS:", "BLUE_BOX_WIDTH_METERS", 0.1, 2.0, 0.05)
        self.tune_blue_box_approach_dist = self._add_tuning_row("APPROACH_DISTANCE_M:", "BLUE_BOX_APPROACH_DISTANCE_M", 0.5, 5.0, 0.1)
        self.tune_blue_box_lat_offset = self._add_tuning_row("LATERAL_OFFSET_M:", "BLUE_BOX_LATERAL_OFFSET_M", -5.0, 5.0, 0.1)
        self.tune_blue_box_yaw_search = self._add_tuning_row("YAW_SEARCH:", "BLUE_BOX_YAW_SEARCH", -180, 180, 5, is_int=True)

        self._add_header("Misi Docking (Box Merah)")
        self.tune_dock_align_thrust = self._add_tuning_row("ALIGN_THRUST:", "DOCK_ALIGN_THRUST", 0.0, 1.0, 0.05)
        self.tune_dock_hold_dur = self._add_tuning_row("HOLD_DURATION_S:", "DOCK_HOLD_DURATION_S", 0.0, 20.0, 0.5)
        self.tune_red_box_width = self._add_tuning_row("BOX_WIDTH_METERS:", "RED_BOX_WIDTH_METERS", 0.1, 2.0, 0.05)
        self.tune_red_box_dock_dist = self._add_tuning_row("DOCK_DISTANCE_M:", "RED_BOX_DOCK_DISTANCE_M", 0.1, 5.0, 0.1)
        self.tune_yaw_search_dock = self._add_tuning_row("YAW_SEARCH:", "YAW_SEARCH_DOCK", -180, 180, 5, is_int=True)

        self._add_header("YOLO & System")
        self.tune_yolo_frame_skip = self._add_tuning_row("YOLO_FRAME_SKIP:", "YOLO_FRAME_SKIP", 0, 10, 1, is_int=True)
        self.tune_yolo_inf_size = self._add_tuning_row("YOLO_INFERENCE_SIZE:", "YOLO_INFERENCE_SIZE", 320, 1280, 32, is_int=True)
        self.tune_session_video_fps = self._add_tuning_row("SESSION_VIDEO_FPS:", "SESSION_VIDEO_FPS", 1.0, 30.0, 1.0)

        self._add_header("Trigger Misi (Waypoint Index)")
        self._add_tuning_list("VISION_ENABLED_LEGS (Gate):", "VISION_ENABLED_LEGS")
        self._add_tuning_list("PHOTO_BOX_LEGS (Hijau):", "PHOTO_BOX_LEGS")
        self._add_tuning_list("BLUE_BOX_PHOTO_LEGS (Biru):", "BLUE_BOX_PHOTO_LEGS")
        self._add_tuning_list("STOP_AND_PHOTO_AT_WP:", "STOP_AND_PHOTO_AT_WP")

        scroll_area.setWidget(widget)
        return scroll_area

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

        self.nav_thread.navigator.waypoints.append({'lat': lat, 'lon': lon})
        new_index = len(self.nav_thread.navigator.waypoints) - 1

        text = f"Added WP #{new_index + 1} ({lat:.6f}, {lon:.6f})"
        self.wp_status_label.setText(text)

        print("[GUI] Waypoint recorded directly to memory. Refreshing map...")
        self.initialize_map_widget()

    def on_save_waypoints(self):
        wps_to_save = self.nav_thread.navigator.waypoints

        try:
            with open(WAYPOINT_FILE, mode='w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=['lat', 'lon'])
                writer.writeheader()
                writer.writerows(wps_to_save)

            self.wp_status_label.setText(f"Saved {len(wps_to_save)} WPs to {WAYPOINT_FILE}")
            
            self.nav_thread.update_waypoints(wps_to_save)
            
            self.initialize_map_widget()
            
        except Exception as e:
            self.wp_status_label.setText(f"Error saving: {e}")

    def on_clear_waypoints(self):
        self.nav_thread.navigator.waypoints = []
        self.wp_status_label.setText("All waypoints cleared (in memory). Save to commit.")
        
        self.initialize_map_widget()

            
    def closeEvent(self, event):
        print("Closing application...")
        self.nav_thread.stop()
        self.nav_thread.wait()
        
        if hasattr(self.nav_thread, 'save_config_to_file'):
            print("Menyimpan parameter tuning terakhir...")
            self.nav_thread.save_config_to_file()
        
        if os.path.exists(TMP_MAP_FILE): os.remove(TMP_MAP_FILE)
            
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