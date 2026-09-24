# Konfigurasi & Parameter Tuning

Satu-satunya sumber kebenaran: `app/settings.py` → disimpan otomatis ke `config/tuning_params.json` saat GUI ditutup.

## Kamera & Orientasi

| Parameter | Default | Arti |
|---|---|---|
| `CAMERA_INDEX` | `0` | Kamera navigasi — preferensi serial, auto-negosiasi pilih resolusi tertinggi |
| `CAMERA_FLIP_MODE` | `0` | `0=normal, 1=h-flip, 2=v-flip, 3=180°` — **tidak mirror** (`jangan reverse`) |
| `CAMERA_TARGET_FPS` | `20` | FPS target pacing (turun dari 30 untuk stabilitas) |
| `CAMERA_MIN_ACCEPT_FPS` | `15` | Mode < ini ditolak negosiasi |
| `CAMERA_MAX_AUTO_WIDTH/HEIGHT` | `1920×1080` | Batas probing resolusi |
| `FRAME_WIDTH / FRAME_HEIGHT` | `1280×720` | Default (ditimpa negosiasi) |

## Model YOLO

| Parameter | Default | Misi |
|---|---|---|
| `MODEL_PATH` | `weights/buoy.pt` | Gate (≥2 pelampung) |
| `BOX_MODEL_PATH` | `weights/box_hijau.pt` | Foto kotak hijau |
| `BLUE_BOX_MODEL_PATH` | `weights/best_blue_dark.pt` | Foto kotak biru (gelap) |
| `RED_DOCK_MODEL_PATH` | `weights/best_red_new.pt` | Docking kotak merah |

## Telemetri & Hardware

| Parameter | Default | Arti |
|---|---|---|
| `SERIAL_PORT` | `COM8` / `/dev/ttyACM0` | Port Pixhawk |
| `BAUD_RATE` | `57600` | Baud MAVLink |
| `OFFBOARD_STREAM_RATE_HZ` | `30` | Rate stream offboard |

## Navigasi Umum

| Parameter | Default | Arti |
|---|---|---|
| `ACCEPTANCE_RADIUS_M` | `2.0` | Radius "sampai" waypoint |
| `THRUST_VALUE` | `0.9` | Thrust transit (0..1) |
| `TRANSITION_DURATION_S` | `0.5` | Jeda antar WP |
| `GEOFENCE_WIDTH_METERS` | `1.0` | Simpangan max dari garis lintasan |

## Vision Global

| Parameter | Default | Arti |
|---|---|---|
| `FOCAL_LENGTH_PX` | `400` | Focal length estimasi jarak |
| `VISION_P_GAIN` | `1.9` | Gain koreksi yaw visi |
| `VISION_SMOOTHING_ALPHA` | `0.6` | EMA smoothing (tinggi=responsif) |
| `ROI_TOP_CUTOFF_PERCENT` | `0.20` | Potong atas frame (hindari langit) |
| `YOLO_FRAME_SKIP` | `1` | Inferensi tiap N frame |
| `YOLO_INFERENCE_SIZE` | `320` | Res input YOLO (kecil=cepat) |
| `SESSION_VIDEO_FPS` | `10` | FPS rekaman video session |

## Filter & Kontroler Galat (baru — dipakai navigator via C core)

| Parameter | Default | Arti |
|---|---|---|
| `PID_GATE_P` | `1.9` | Gain P gate (awal = VISION_P_GAIN) |
| `PID_GATE_I` | `0.0` | Gain I gate |
| `PID_GATE_D` | `0.0` | Gain D gate |
| `PID_DEADBAND` | `0.02` | Zona mati radian (~1.1°) |
| `PID_OUTPUT_LIMIT` | `0.5` | Clamp output PID radian |
| `PID_INTEGRAL_LIMIT` | `0.5` | Anti-windup limit |
| `COMPLEMENTARY_ALPHA` | `0.98` | Alpha filter komplementer |
| `EKF_HEADING_ENABLED` | `true` | Aktifkan EKF heading |
| `EKF_PROCESS_NOISE` | `1e-4` | Noise proses EKF (rad²) |
| `EKF_MEAS_NOISE` | `1e-2` | Noise pengukuran EKF (rad²) |

## Misi Gate (Buoy) — Threshold **longgar** + debug

| Parameter | Default | Arti |
|---|---|---|
| `VISION_ENABLED_LEGS` | `[1,3,5]` | Leg dengan koreksi gate |
| `GATE_WIDTH_METERS` | `1.0` | Lebar gate aktual |
| `MIN_BUOY_AREA_PX` | `40` | Luas minimal deteksi (turun dr 80) |
| `GATE_AREA_SIMILARITY_RATIO` | `0.5` | Rasio kemiripan 2 buoy |
| `BUOY_CONF_THRESHOLD` | `0.35` | Confidence model (turun dr 0.55) |
| `BUOY_MIN_COLOR_FRACTION` | `0.05` | Fraksi warna crop (turun dr 0.12) |
| `BUOY_MIN_SATURATION` | `0.35` | Saturasi min HSV (turun dr 0.55) |
| `BUOY_MAX_ASPECT_DEVIATION` | `0.50` | Batas `|w/h-1|` (naik dr 0.35) |
| `GATE_VERTICAL_ALIGN_PX` | `75` | Ambang vertikal switch gate (px) |
| `GATE_PASS_DISTANCE_M` | `2.5` | Jarak lolos gate → switch cepat |
| `GATE_LOST_TOLERANCE_FRAMES` | `5` | Frame toleransi kehilangan deteksi |
| `DETECTION_DEBUG` | `false` | Print alasan tolak per objek (throttle 1x/detik) |

`validate_buoy(..., debug=True)` → return `(ok, reasons[])` dengan nilai terukur per kriteria.

## Misi Foto Waypoint

| Parameter | Default | Arti |
|---|---|---|
| `STOP_AND_PHOTO_AT_WP` | `[]` | Index WP stop & foto |
| `WAYPOINT_PHOTO_STOP_DURATION_S` | `3.0` | Lama berhenti foto |
| `DETECTION_CONFIRM_DURATION_S` | `0.5` | Deteksi stabil sebelum yakin |

## Misi Kotak Hijau

| Parameter | Default | Arti |
|---|---|---|
| `PHOTO_BOX_LEGS` | `[6]` | Leg memicu misi hijau |
| `SEARCH_THRUST` | `0.4` | Thrust cari kotak |
| `ALIGN_THRUST` | `0.3` | Thrust align |
| `RETREAT_THRUST` | `-0.7` | Thrust mundur (neg=mundur) |
| `RETREAT_DURATION_S` | `0.5` | Durasi mundur |
| `BOX_WIDTH_METERS` | `0.4` | Lebar kotak hijau |
| `BOX_APPROACH_DISTANCE_M` | `2.0` | Jarak stop depan kotak |
| `YAW_SEARCH_BOX` | `-90` | Sudut rotasi cari |
| `BOX_SEARCH_LATERAL_THRUST` | `-0.2` | Thrust lateral cari |

## Misi Kotak Biru

| Parameter | Default | Arti |
|---|---|---|
| `BLUE_BOX_PHOTO_LEGS` | `[8]` | Leg memicu biru |
| `BLUE_BOX_WIDTH_METERS` | `0.6` | Lebar kotak biru |
| `BLUE_BOX_APPROACH_DISTANCE_M` | `1.5` | Jarak stop |
| `BLUE_BOX_LATERAL_OFFSET_M` | `-1.0` | Offset samping foto |
| `BLUE_BOX_ALIGN_THRUST` | `0.3` | Thrust align |
| `BLUE_BOX_SEARCH_THRUST` | `0.2` | Thrust cari |
| `BLUE_BOX_YAW_SEARCH` | `-90` | Sudut rotasi cari |

## Misi Docking (Kotak Merah)

| Parameter | Default | Arti |
|---|---|---|
| `RED_BOX_NAV_AFTER_WP` | `8` | WP mulai docking |
| `RED_BOX_WIDTH_METERS` | `0.6` | Lebar kotak merah |
| `RED_BOX_DOCK_DISTANCE_M` | `1.0` | Jarak "masuk dock" |
| `DOCK_ALIGN_THRUST` | `0.4` | Thrust align |
| `DOCK_HOLD_DURATION_S` | `5.0` | Hold saat dock |
| `YAW_SEARCH_DOCK` | `90` | Sudut rotasi cari |

## Redis / Dashboard (opsional)

| Parameter | Default | Arti |
|---|---|---|
| `REDIS_HOST/PORT` | `localhost:6379` | Broker telemetri |
| `TELEMETRY_CHANNEL` | `asv_telemetry` | Posisi/yaw/state |
| `VISION_CHANNEL` | `asv_vision` | Frame + deteksi |
| `MISSION_CHANNEL` | `asv_mission` | Status misi |

## Tips Tuning Cepat

- **Kapal oleng transit** → kecilkan `THRUST_VALUE` / besarkan `TRANSITION_DURATION_S`.
- **Melenceng garis** → kecilkan `GEOFENCE_WIDTH_METERS` / cek koreksi gate leg 1/3/5.
- **Deteksi tidak konsisten** → naikkan `DETECTION_CONFIRM_DURATION_S`, turunkan `YOLO_INFERENCE_SIZE`.
- **Terlalu dekat kotak** → naikkan `*_APPROACH_DISTANCE_M`.
- **Baterai LiPO 4S**: full 16.8 V, empty 12.8 V → persen linear dari tegangan kalau firmware tak kirim `battery_remaining`.