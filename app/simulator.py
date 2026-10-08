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

from . import settings as cfg
from . import aterkia_core as core

try:
    from pymavlink import mavutil
    MAVLINK_IMPORT_OK = True
except ImportError:  # pragma: no cover - mesin tanpa pymavlink
    mavutil = None
    MAVLINK_IMPORT_OK = False
from .manual_link import ManualLink
from .qgc_offboard import QgcOffboard
from .camera import pace_to_fps

try:
    import numpy as np
    import cv2
    CAMERA_AVAILABLE = True
except ImportError:
    CAMERA_AVAILABLE = False

try:
    from .camera import open_camera, make_fallback_frame, flip_frame_if_needed
    CAMERA_HELPER = True
except ImportError:
    CAMERA_HELPER = False

try:
    from .camera_manager import CameraManager
    CAMERA_MANAGER_OK = True
except ImportError:
    CAMERA_MANAGER_OK = False

from . import mavlink_telemetry as mavlink_mod

# skfuzzy TIDAK dipakai di jalur produksi (fuzzy jalan di C via aterkia_core).
# Blok import dipertahankan agar mesin lama tanpa C tetap bisa jalan dengan
# gain statis — nilai FUZZY di bawah tidak dibaca lagi.
FUZZY_ENABLED = False

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False

try:
    from .detection_validation import validate_buoy, draw_validated_boxes
    VALIDATION_AVAILABLE = True
except ImportError:  # cv2/numpy tidak ada -> fallback ke plot() bawaan YOLO
    VALIDATION_AVAILABLE = False


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
                keep = []
                for b in results[0].boxes:
                    cls = int(b.cls[0])
                    xyxy = b.xyxy[0].tolist()
                    if model is self._gate_model and self._gate_model is not None:
                        if cls in (self.config.RED_BALL_CLASS_ID,
                                   self.config.GREEN_BALL_CLASS_ID):
                            # Filter pasca-YOLO: warna + bentuk + area.
                            # Objek mirip bola tapi bukan buoy (mis. wajah
                            # operator) tidak lolos -> tidak ditampilkan.
                            if not VALIDATION_AVAILABLE or not validate_buoy(
                                    frame, cls, xyxy,
                                    min_area=self.config.MIN_BUOY_AREA_PX,
                                    min_color_fraction=self.config.BUOY_MIN_COLOR_FRACTION,
                                    min_saturation=self.config.BUOY_MIN_SATURATION,
                                    max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION):
                                continue
                    keep.append((cls, float(b.conf[0]), xyxy))
                if VALIDATION_AVAILABLE:
                    annotated = draw_validated_boxes(frame, keep,
                                                     results[0].names)
                else:
                    annotated = results[0].plot()
                detected = len(keep) > 0
                if model is self._gate_model and self._gate_model is not None:
                    detected = len(keep) >= 2  # gate butuh 2 buoy
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
        # Posisi mock: None sampai misi punya WP (tanpa pin palsu ITS).
        self.current_lat = None
        self.current_lon = None
        self.current_yaw_rad = 0.0
        self.current_groundspeed = 0.0
        self.dist_to_wp = 0.0

        # --- KAMERA GANDA (lihat app/camera_manager.py) ---
        # Primary = navigasi (negosiasi resolusi), secondary = bawah air
        # (640x480). Keduanya hidup bareng + reconnect otomatis bila USB
        # dicabut / reset bus saat Pixhawk dicolok. `self.cap` dipertahankan
        # sebagai alias primary agar kode lama tetap jalan.
        print(f"[SIM] Membuka kamera NAV idx={self.config.CAMERA_INDEX} + "
              f"BAWAH idx={self.config.WAYPOINT_PHOTO_CAMERA_INDEX}...")
        self.cam = (CameraManager(self.config)
                    if (CAMERA_HELPER and CAMERA_MANAGER_OK) else None)
        if self.cam is not None:
            st = self.cam.status()["primary"]
            if st["opened"]:
                try:
                    real_w = int(self.cam.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    real_h = int(self.cam.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    self.config.FRAME_WIDTH = real_w
                    self.config.FRAME_HEIGHT = real_h
                    print(f"[CAM] Mode aktif kamera utama: {real_w}x{real_h}.")
                except Exception:
                    pass
            else:
                print("[SIM] Kamera utama TIDAK terbuka — pakai frame "
                      "sintetis + coba lagi otomatis.")
            sec = self.cam.status()["secondary"]
            print(f"[CAM] Kamera bawah: {sec['label']}.")
        else:
            print("[SIM] Helper kamera tak ada — pakai frame sintetis.")
        self.cap = self.cam.cap if self.cam is not None else None

        # --- MODEL YOLO ASLI ---
        if YOLO_AVAILABLE:
            try:
                self.gate_model = YOLO(self.config.MODEL_PATH, task='detect')
                print("[SIM] Model GATE dimuat!")
            except Exception as e:
                print(f"[SIM][ERROR] Gagal memuat model GATE: {e}")
                self.gate_model = None

            # 3 model lain: lazy-load di thread latar
            self.box_model = None
            self.blue_box_model = None
            self.red_dock_model = None
            self._models_loading = False
            self._models_loaded = False
        else:
            self.gate_model = self.box_model = None
            self.blue_box_model = self.red_dock_model = None

        # --- Failsafe otomatis (stale-link + low-batt) — port dari
        # navigator.py lama; dipanggil tiap frame sebelum emit status.
        # Field dibaca test_failsafe.py, jangan ganti nama atribut.
        self.failsafe_active = False
        self.failsafe_reason = ""
        self._lowbatt_since = None
        self._rtl_sent_mono = 0.0
        self._last_telem_mono = 0.0

    def _load_models_async(self):
        """Muat 3 model non-gate di thread latar."""
        if self._models_loading or self._models_loaded or self.gate_model is None:
            return
        self._models_loading = True

        def _loader():
            try:
                print("[SIM] Memuat model BOX HIJAU (lazy)...")
                self.box_model = YOLO(self.config.BOX_MODEL_PATH, task='detect')
            except Exception as e:
                print(f"[SIM][WARN] Gagal memuat model BOX HIJAU: {e}")
            try:
                print("[SIM] Memuat model BOX BIRU (lazy)...")
                self.blue_box_model = YOLO(self.config.BLUE_BOX_MODEL_PATH, task='detect')
            except Exception as e:
                print(f"[SIM][WARN] Gagal memuat model BOX BIRU: {e}")
            try:
                print("[SIM] Memuat model BOX MERAH (lazy)...")
                self.red_dock_model = YOLO(self.config.RED_DOCK_MODEL_PATH, task='detect')
            except Exception as e:
                print(f"[SIM][WARN] Gagal memuat model BOX MERAH: {e}")
            self._models_loaded = True
            self._models_loading = False
            print("[SIM] Lazy-load model selesai.")

        threading.Thread(target=_loader, daemon=True, name="lazy-models").start()

    def _reset_state_awal(self):
        """State awal loop GroundSim (dipanggil usai lazy-loader dibuat)."""
        self.state_timer = time.time()
        self.sim_step = 1
        self._loop_start = time.time()
        self._yolo_subcount = 0
        self._last_detection = {}   # id(model) -> (detected, annotated)
        # Histeresis telemetri mock->real: butuh N frame bagus beruntun
        # sebelum pindah (hindari GUI lompat saat Pixhawk baru dicolok /
        # heartbeat sesaat). Mundur ke mock juga butuh N gagal beruntun.
        self._telem_good_streak = 0
        self._telem_bad_streak = 0
        self._telem_use_real = False
        self._mav_port_logged = None
        self._mav_retry_mono = 0.0
        # Jembatan manual RC/gamepad (hitung di C) — dipasang ke master
        # MAVLink setelah konek; GUI bisa minta manual via flag ini.
        self.manual_link = ManualLink(self.config)
        # Sumber manual QGC (Xbox di laptop -> QGC -> MAVLink MANUAL_CONTROL).
        # Adu dengan sumber lokal via core.arb_manual2 (konflik -> netral).
        self.qgc_link = QgcOffboard(self.config)
        self.manual_gui_request = False
        # KILL dari tombol GUI: masuk jalur C (mode_manager KILL) + kirim
        # DISARM ke Pixhawk (lihat _send_disarm).
        self.kill_request = False
        self._disarm_sent_mono = 0.0
        # Hasil cek param failsafe Pixhawk saat startup (thread daemon,
        # lihat _pixhawk_param_check_worker). Dibaca GUI via data_signal.
        self.pixhawk_check = "menunggu MAVLink"
        threading.Thread(target=self._pixhawk_param_check_worker,
                         daemon=True).start()
        self.manual_last = {"surge": 0.0, "yaw": 0.0, "active": False,
                            "mode": 0, "mode_name": "AUTO", "rc_ok": False,
                            "source": "AUTO"}
        self.qgc_last = dict(self.manual_last)
        self._last_loop_t = time.time()

    def _mav_ensure_connected(self):
        """Connect Pixhawk non-blocking (throttled): dipanggil tiap loop.

        connect() lama bersifat blocking ~5 dtk; di sini hanya dicoba
        tiap MAV_RETRY_INTERVAL_S agar loop 25 Hz tak freeze. Sukses
        sekali -> attach RC + GCS forward, selanjutnya tinggal poll().
        Return True bila sudah connected.
        """
        if getattr(self, "mav", None) is not None and self.mav.connected:
            return True
        now = time.monotonic()
        interval = float(getattr(self.config, "MAV_RETRY_INTERVAL_S", 5.0))
        if now - getattr(self, "_mav_retry_mono", 0.0) < interval:
            return False
        self._mav_retry_mono = now
        try:
            self.mav = mavlink_mod.MavlinkTelemetry(
                port=self.config.SERIAL_PORT, baud=self.config.BAUD_RATE)
        except Exception:
            return False
        if not mavlink_mod.MAVLINK_AVAILABLE:
            return False
        ok = False
        try:
            ok = self.mav.connect(
                timeout_s=float(getattr(self.config,
                                        "MAV_CONNECT_TIMEOUT_S", 2.0)))
        except Exception:
            ok = False
        if ok:
            try:
                self.manual_link.attach_mav(self.mav.master)
                self.qgc_link.attach_mav(self.mav.master)
            except Exception:
                pass
            if os.environ.get("GCS_FORWARD", "0") == "1":
                try:
                    self.mav.enable_gcs_forward(
                        True,
                        ip=os.environ.get("GCS_IP", "127.0.0.1"),
                        port=int(os.environ.get("GCS_PORT", "14550")))
                except Exception:
                    pass
            if self._mav_port_logged != self.mav.port:
                self._mav_port_logged = self.mav.port
                print(f"[MAV] Telemetri AKTIF via {self.mav.port} "
                      "(histeresis: butuh fix stabil dulu).")
        return ok

    def _telem_decide_source(self, mav_has_data):
        """Histeresis mock<->real; return True bila pakai data Pixhawk."""
        need = int(getattr(self.config, "TELEM_HYSTERESIS_FRAMES", 5))
        if mav_has_data:
            self._telem_good_streak += 1
            self._telem_bad_streak = 0
            if not self._telem_use_real and self._telem_good_streak >= need:
                self._telem_use_real = True
                print("[MAV] Sumber telemetri: MOCK -> REAL (stabil).")
        else:
            self._telem_bad_streak += 1
            self._telem_good_streak = 0
            if self._telem_use_real and self._telem_bad_streak >= need:
                self._telem_use_real = False
                print("[MAV] Sumber telemetri: REAL -> MOCK (link putus).")
        return self._telem_use_real

    # ------------------- Geolokasi mock -------------------
    def _update_mock_position(self, target_lat, target_lon, speed_mps=1.5):
        """Geser posisi mock mendekati target; True jika sudah tiba (<2 m).

        Geodesi dihitung di C (nav_math via aterkia_core); app/geo.py hanya
        referensi uji.
        """
        dist, bearing = core.nav_distance_bearing(
            self.current_lat, self.current_lon, target_lat, target_lon)
        self.dist_to_wp = dist
        self.current_yaw_rad = bearing
        if dist > 2.0:
            delta = speed_mps * 0.1
            self.current_lat, self.current_lon = core.nav_destination(
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
            conf = 0.5
            if model is self.gate_model:
                # Model gate rawan mendeteksi objek mirip bola (mis. wajah)
                # sebagai buoy — confidence-nya dinaikkan lewat config
                # (BUOY_CONF_THRESHOLD) + validasi warna/bentuk di worker.
                conf = self.config.BUOY_CONF_THRESHOLD
            self._yolo_worker.submit(model, frame.copy(), conf)
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

    # ------------------- Failsafe otomatis (port dari navigator.py) ---
    # ID mode RTL ArduPilot Rover; jeda antar percobaan kirim RTL.
    _ROVER_RTL_MODE = 11
    _RTL_RETRY_INTERVAL_S = 5.0

    def _battery_fields(self):
        """Baca baterai: dari Pixhawk (mav) bila terhubung, else atribut mock."""
        mav = getattr(self, "mav", None)
        if mav is not None and mav.has_battery:
            return {"battery_pct": mav.battery_pct,
                    "voltage_v": mav.voltage_v,
                    "current_a": mav.current_a}
        return {"battery_pct": getattr(self, "current_battery_pct", None),
                "voltage_v": getattr(self, "current_voltage", None),
                "current_a": getattr(self, "current_current", None)}

    def _request_rtl(self):
        """Minta mode RTL via MAV_CMD_DO_SET_MODE (best-effort, throttled).

        Pixhawk TETAP pemegang failsafe utama (RCIN + geofence bawaan);
        perintah ini hanya usaha tambahan, maks 1x per _RTL_RETRY_INTERVAL_S
        agar tak membanjiri link MAVLink yang sedang bermasalah.
        """
        now = time.monotonic()
        if now - self._rtl_sent_mono < self._RTL_RETRY_INTERVAL_S:
            return
        self._rtl_sent_mono = now
        try:
            master = (getattr(self, "master", None)
                      or (self.mav.master if getattr(self, "mav", None)
                          is not None else None))
            if master is None or mavutil is None:
                return
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                self._ROVER_RTL_MODE, 0, 0, 0, 0, 0)
            print("[FAILSAFE] Perintah RTL dikirim (best-effort).")
        except Exception as e:
            print(f"[FAILSAFE] Gagal kirim RTL: {e}")

    def _send_disarm(self):
        """KILL dari tombol GUI: minta Pixhawk DISARM (best-effort).

        MAV_CMD_COMPONENT_ARM_DISARM param1=0. Throttled 1x per 2 detik agar
        tak membanjiri link. Pixhawk TETAP pemegang keselamatan utama (RCIN +
        failsafe bawaan); perintah ini pelengkap agar ESC/ESC benar mati.
        """
        if not getattr(self, "kill_request", False):
            return
        now = time.monotonic()
        if now - getattr(self, "_disarm_sent_mono", 0.0) < 2.0:
            return
        self._disarm_sent_mono = now
        try:
            master = (getattr(self, "master", None)
                      or (self.mav.master if getattr(self, "mav", None)
                          is not None else None))
            if master is None or mavutil is None:
                return
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
            print("[KILL] Disarm dikirim ke Pixhawk (best-effort).")
        except Exception as e:
            print(f"[KILL] Gagal kirim disarm: {e}")

    def _pixhawk_param_check_worker(self):
        """Cek parameter failsafe wajib sekali saat mav tersambung.

        HANYA MEMBACA (PARAM_REQUEST_READ). Hasil disimpan di
        self.pixhawk_check ('OK' / daftar pelanggaran) + dicetak.
        Tidak memblokir loop utama (thread daemon).
        """
        from .pixhawk_check import load_spec, check_params, summarize
        try:
            spec = load_spec()
        except Exception as e:
            self.pixhawk_check = f"spec gagal dibaca ({e})"
            print(f"[PIXHAWK-CHECK] {self.pixhawk_check}")
            return
        deadline = time.time() + 10.0
        while time.time() < deadline:
            mav = getattr(self, "mav", None)
            master = (getattr(self, "master", None)
                      or (mav.master if mav is not None else None))
            if master is not None and mav is not None \
                    and getattr(mav, "connected", False):
                try:
                    results = check_params(master, spec)
                except Exception as e:
                    self.pixhawk_check = f"gagal ({e})"
                    print(f"[PIXHAWK-CHECK] {self.pixhawk_check}")
                    return
                self.pixhawk_check = summarize(results)
                print(f"[PIXHAWK-CHECK] {self.pixhawk_check}")
                for r in results:
                    if r["status"] != "ok":
                        tag = "KRITIS" if r["critical"] else "advisory"
                        if r["status"] == "unread":
                            print(f"  - {r['name']}: TIDAK DIJAWAB ({tag})")
                        else:
                            print(f"  - {r['name']}={r['value']} "
                                  f"luar [{r['min']}..{r['max']}] ({tag})")
                return
            time.sleep(0.5)
        self.pixhawk_check = "MAVLink tidak tersambung (cek dilewati)"

    def _check_failsafe(self):
        """Evaluasi failsafe tiap frame. Return "" bila aman, atau string
        alasan bila aktif — caller mematikan gerak + menimpa status.

        Pemicu (ambang di Config, tab Failsafe di Settings):
          1. stale-link: tak ada ATTITUDE/GLOBAL_POSITION > timeout_s;
          2. low-batt: persen ATAU tegangan di bawah ambang, DITAHAN selama
             hold agar spike sesaat tidak memicu RTL palsu.
        Pulih otomatis bila kondisi normal kembali.
        """
        if not getattr(self.config, "FAILSAFE_ENABLED", True):
            if self.failsafe_active:
                self.failsafe_active = False
                self.failsafe_reason = ""
            return ""
        now = time.monotonic()
        reason = ""
        timeout = float(getattr(self.config, "FAILSAFE_TELEM_TIMEOUT_S", 2.0))
        last = float(getattr(self, "_last_telem_mono", 0.0) or 0.0)
        if last > 0.0 and (now - last) > timeout:
            reason = f"telemetri basi {now - last:.1f}s"
        if not reason:
            fields = self._battery_fields()
            pct = fields.get("battery_pct")
            volt = fields.get("voltage_v")
            low_pct = float(getattr(self.config, "FAILSAFE_LOW_BATT_PCT", 20.0))
            low_v = float(getattr(self.config, "FAILSAFE_LOW_VOLT_V", 13.2))
            hold = float(getattr(self.config, "FAILSAFE_LOW_BATT_HOLD_S", 3.0))
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
                print(f"[FAILSAFE] AKTIF: {reason} — gerak dihentikan "
                      "dan coba RTL.")
            self.failsafe_active = True
            self.failsafe_reason = reason
            self._request_rtl()
            return reason
        if self.failsafe_active:
            print("[FAILSAFE] Pulih: kondisi normal kembali.")
        self.failsafe_active = False
        self.failsafe_reason = ""
        return ""

    # ------------------- Loop utama -------------------
    def run(self, data_signal):
        print("--- [SIM] GROUND SIMULATOR (kamera + YOLO asli) START ---")
        self.running = True

        # --- Pixhawk: connect non-blocking + throttled (loop tak freeze) ---
        # Sebelumnya connect(timeout 5 dtk) blocking di sini -> GUI freeze
        # saat start; kini _mav_ensure_connected() dipanggil tiap loop.
        self.mav = None
        self._last_mav_log = 0.0
        if not mavlink_mod.MAVLINK_AVAILABLE:
            print("[SIM] pymavlink belum terpasang — attitude dari mock.")

        # Worker YOLO asinkron: video loop jalan 30 fps terlepas dari
        # kecepatan inferensi (deteksi tersedia di saatnya).
        self._yolo_worker = _YoloWorker(self.config)
        self._yolo_worker.bind_gate_model(self.gate_model)
        self._yolo_worker.start()
        self._load_models_async()
        self._reset_state_awal()
        deadline = time.monotonic()

        while self.running:
            # --- Pixhawk reconnect throttled (colok belakangan OK) ---
            try:
                self._mav_ensure_connected()
            except Exception:
                pass
            # --- Baca frame NAV via manajer (reconnect otomatis) ---
            _cam_st = "OK"
            if self.cam is not None:
                _ok, frame, _cam_st = self.cam.read_primary(
                    self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT)
                self.cap = self.cam.cap  # alias kompat kode lama
                if frame is None:
                    _ok = False
            elif self.cap is not None:
                try:
                    _ok, frame = self.cap.read()
                except Exception:
                    _ok, frame = False, None
            else:
                _ok, frame = False, None
            if frame is None or not _ok:
                frame = make_fallback_frame(
                    self.config.FRAME_WIDTH, self.config.FRAME_HEIGHT,
                    text=("KAMERA TERPUTUS — mencoba lagi..."
                          if _cam_st == "RETRY"
                          else "KAMERA BELUM ADA — colok USB / Scan"))
            # --- Baca frame BAWAH (downscale, None bila tak ada) ---
            _sec_ok, _sec_frame = False, None
            if self.cam is not None:
                try:
                    _sec_ok, _sec_frame, _ = self.cam.read_secondary()
                except Exception:
                    _sec_ok, _sec_frame = False, None

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

            # mock gerakan ke waypoint aktif (mulai dari WP1 bila ada WP)
            if self.waypoints:
                if self.current_lat is None:
                    self.current_lat = self.waypoints[0]['lat']
                    self.current_lon = self.waypoints[0]['lon']
                target = self.waypoints[min(self.current_waypoint_index,
                                            len(self.waypoints) - 1)]
                speed = 1.5 if self.sim_step in (1, 3, 5) else 0.2
                if self.failsafe_active:
                    speed = 0.0   # failsafe: mock tidak boleh maju
                self._update_mock_position(target['lat'], target['lon'],
                                           speed_mps=speed)
            else:
                self.current_lat, self.current_lon = 0.0, 0.0

            # --- Telemetri: histeresis mock<->real (GUI tak lompat) ---
            if self.mav is not None:
                try:
                    self.mav.poll()
                except Exception:
                    pass
            _mav_has = (self.mav is not None and self.mav.connected
                        and self.mav.has_attitude)
            # Stamp umur telemetri untuk failsafe stale-link (port navigator).
            if _mav_has:
                self._last_telem_mono = time.monotonic()
            use_real = self._telem_decide_source(_mav_has)
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

            # Kendali manual (hitung di C): KILL > MANUAL > AUTO. Di mode
            # MANUAL, state_txt ditimpa agar operator tahu siapa memegang
            # kapal; field op_mode/manual_* dibaca chip + panel GUI.
            try:
                now_man = time.time()
                dt_man = max(1e-4, now_man - self._last_loop_t)
                self._last_loop_t = now_man
                self.manual_last = self.manual_link.update(
                    dt=dt_man, kill=self.kill_request,
                    gui_manual=self.manual_gui_request)
                # Sumber manual QGC (Xbox via MAVLink) — poll non-blocking.
                self.qgc_last = self.qgc_link.poll(dt=dt_man)
            except Exception:
                pass
            man = getattr(self, "manual_last", {}) or {}
            qgc = getattr(self, "qgc_last", {}) or {}

            # --- Adu dua sumber manual: QGC (Xbox laptop) vs lokal
            # (RC/gamepad kapal). Hitung di C (arb_manual2): keduanya
            # aktif = KONFLIK kendali -> netral + warning, TANPA memilih.
            try:
                arb2 = core.arb_manual2(
                    bool(qgc.get("active")),
                    float(qgc.get("surge", 0.0)), float(qgc.get("yaw", 0.0)),
                    bool(man.get("active")),
                    float(man.get("surge", 0.0)), float(man.get("yaw", 0.0)))
            except Exception:
                arb2 = {"surge": 0.0, "yaw": 0.0, "source": 0,
                        "source_name": "?", "conflict": False}

            op_mode = man.get("mode_name", "AUTO")
            if arb2.get("conflict"):
                # Dua operator memegang bersamaan -> paksa netral dulu.
                man = dict(man, active=True, surge=0.0, yaw=0.0)
                op_mode = "MANUAL"
                status_txt = "KONFLIK manual (QGC + lokal) -> NETRAL"
            elif qgc.get("active"):
                # QGC menang (sumber lokal tidak aktif).
                man = dict(man, active=True, mode_name="MANUAL",
                           surge=float(arb2.get("surge", 0.0)),
                           yaw=float(arb2.get("yaw", 0.0)),
                           rc_ok=bool(qgc.get("rc_ok", False)),
                           source="QGC")
                op_mode = "MANUAL"
                status_txt = (f"MANUAL-QGC (surge {float(arb2.get('surge', 0.0)):+.2f} "
                              f"yaw {float(arb2.get('yaw', 0.0)):+.2f})")
            elif op_mode == "MANUAL" and man.get("active"):
                status_txt = (f"MANUAL (surge {float(man.get('surge', 0.0)):+.2f} "
                              f"yaw {float(man.get('yaw', 0.0)):+.2f})")
            elif op_mode == "KILL":
                status_txt = "KILL (E-stop)"

            # --- Failsafe otomatis (stale-link + low-batt) — prioritas
            # TERTINGGI: menimpa status apa pun sebelum dikirim ke GUI.
            fs_reason = self._check_failsafe()
            if fs_reason:
                status_txt = f"FAILSAFE ({fs_reason})"
                op_mode = "FAILSAFE"
            # KILL GUI: kirim DISARM best-effort (throttled di dalam).
            self._send_disarm()

            # Simpan sementara agar bisa dimodifikasi sebelum emit.
            _pkt = {
                "lat": lat, "lon": lon,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg, "roll_deg": roll_deg,
                "state": self.current_state,
                "target_wp_idx": self.current_waypoint_index,
                "dist_to_wp_m": self.dist_to_wp,
                "groundspeed": (self.mav.groundspeed
                                if (use_real and self.mav is not None)
                                else self.current_groundspeed),
                "mavlink_ok": bool(getattr(self.mav, "connected", False))
                if self.mav else False,
                "telem_source": "REAL" if use_real else "MOCK",
                "gps_fix": (bool(self.mav.lat is not None)
                            if (use_real and self.mav is not None) else True),
                "frame": processed_frame,
                "frame_secondary": _sec_frame if _sec_ok else None,
                "cam_status": _cam_st,
                "op_mode": op_mode,
                "manual_active": bool(man.get("active", False)),
                "manual_surge": float(man.get("surge", 0.0)),
                "manual_yaw": float(man.get("yaw", 0.0)),
                "rc_ok": bool(man.get("rc_ok", False)),
                "manual_source": str(man.get("source", "AUTO")),
                "gcs_forward": bool(getattr(self.mav, "gcs_active", False)),
                "failsafe_active": bool(self.failsafe_active),
                "failsafe_reason": str(self.failsafe_reason),
                "pixhawk_check": str(getattr(self, "pixhawk_check", "N/A")),
            }
            # Baterai dari Pixhawk (SYS_STATUS/BATTERY_STATUS) bila terhubung.
            if self.mav and self.mav.has_battery:
                _pkt["voltage_v"] = self.mav.voltage_v
                _pkt["current_a"] = self.mav.current_a
                _pkt["battery_pct"] = self.mav.battery_pct
            else:
                _pkt["voltage_v"] = None
                _pkt["current_a"] = None
                _pkt["battery_pct"] = None
            data_signal.emit(_pkt)

            if self.dist_to_wp < 3.0 and self.current_waypoint_index < len(self.waypoints) - 1:
                if self.sim_step in (1, 3, 5):
                    self.current_waypoint_index += 1

            # Pacing 25 Hz monotonic (tanpa printf tiap detik —
            # FPS hanya tampil di chip GUI main.py, bukan terminal).
            # Frame kamera lambat (mis. 10 fps) tidak menurunkan display:
            # cap.read() blocking memberi frame terakhir, pacing tetap 25.
            deadline = pace_to_fps(deadline)
            self._last_loop_t = time.time()

    def stop(self):
        self.running = False
        yolo_worker = getattr(self, "_yolo_worker", None)
        if yolo_worker is not None:
            yolo_worker.stop_and_wait()
        if getattr(self, 'mav', None):
            try:
                self.mav.close()
            except Exception:
                pass
        if getattr(self, "cam", None) is not None:
            try:
                self.cam.close()
            except Exception:
                pass
        self.cap = None

    # API GUI: Scan kamera + ganti index saat runtime (plug-and-play).
    def rescan_cameras(self):
        cam = getattr(self, "cam", None)
        if cam is None:
            try:
                from .camera import list_cameras
                return list_cameras()
            except Exception:
                return []
        return cam.rescan()

    def select_camera(self, role, index):
        """Ganti kamera NAV/BAWAH saat runtime. Return True bila terbuka."""
        cam = getattr(self, "cam", None)
        if cam is None:
            return False
        role = str(role).upper()
        if role in ("NAV", "PRIMARY", "UTAMA"):
            ok = cam.select_primary(index)
            self.cap = cam.cap
            return ok
        return cam.select_secondary(index)


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

        self.current_lat = None
        self.current_lon = None
        self.current_yaw_rad = 0.0
        self.current_groundspeed = 5.0

        self.current_state = "INIT"
        self.task_timer = 0.0
        self.last_loop_time = time.time()
        self.last_used_p_gain = config.VISION_P_GAIN
        self.image_center_x = config.FRAME_WIDTH / 2.0
        self.mock_box = None
        self.fuzzy_ctrl = self._create_fuzzy_controller()

    def _create_fuzzy_controller(self):
        # DIHAPUS dari jalur produksi: fuzzy Sugeno jalan di C
        # (core/src/fuzzy.c via aterkia_core.fuzzy_docking_gain).
        # Stub dipertahankan agar atribut lama tidak AttributeError.
        return None

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
        """Koreksi yaw (rad) dari selisih pixel objek vs pusat frame.

        Gain P dinamis dihitung di C (fuzzy docking via aterkia_core);
        skfuzzy tidak dipakai di jalur produksi (hanya referensi lama).
        """
        if not mock_obj:
            return 0.0
        error_px = mock_obj['cx'] - self.image_center_x
        dist_m = mock_obj['dist']
        try:
            error_m = (error_px * dist_m) / self.config.FOCAL_LENGTH_PX
        except ZeroDivisionError:
            return 0.0
        raw_rad = math.atan2(error_m, dist_m)

        try:
            gain = core.fuzzy_docking_gain(min(max(dist_m, 0.0), 10.0),
                                           abs(error_px))
            if gain <= 0.0:
                gain = self.config.VISION_P_GAIN
        except Exception:
            gain = self.config.VISION_P_GAIN
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
        deadline = time.monotonic()
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
                jarak_wp, bear_wp = core.nav_distance_bearing(
                    self.current_lat, self.current_lon, wp['lat'], wp['lon'])

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
                target_yaw = core.nav_normalize_angle(
                    self.current_yaw_rad + correction_rad)
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
                diff = core.nav_normalize_angle(target_yaw - self.current_yaw_rad)
                self.current_yaw_rad += diff * dt * 2.0
                self.current_lat, self.current_lon = core.nav_destination(
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
                "voltage_v": None, "current_a": None, "battery_pct": None,
            })
            deadline = pace_to_fps(deadline)

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