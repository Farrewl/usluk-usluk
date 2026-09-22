# Konfigurasi & Parameter Tuning

Satu-satunya sumber kebenaran parameter adalah `app/settings.py`
(disimpan otomatis ke `config/tuning_params.json` saat GUI ditutup).

## Model & Kamera

| Parameter | Default | Arti |
|---|---|---|
| `MODEL_PATH` | `weights/buoy.pt` | Model gate: butuh ≥2 deteksi pelampung |
| `BOX_MODEL_PATH` | `weights/box_hijau.pt` | Deteksi kotak hijau (misi foto) |
| `BLUE_BOX_MODEL_PATH` | `weights/best_blue_dark.pt` | Deteksi kotak biru (versi gelap) |
| `RED_DOCK_MODEL_PATH` | `weights/best_red_new.pt` | Deteksi kotak merah (docking) |
| `CAMERA_INDEX` | `0` | Kamera navigasi (depan) |
| `WAYPOINT_PHOTO_CAMERA_INDEX` | `1` | Kamera bawah air untuk foto WP8 |
| `FRAME_WIDTH / FRAME_HEIGHT` | `1280 × 720` | Resolusi capture kamera |

## Hardware / Telemetri

| Parameter | Default | Arti |
|---|---|---|
| `SERIAL_PORT` | `COM8` | Port Pixhawk (Windows). Pi: `/dev/ttyACM0` |
| `BAUD_RATE` | `57600` | Baud MAVLink |
| `OFFBOARD_STREAM_RATE_HZ` | `30` | Kecepatan stream perintah offboard |

## Redis / Dashboard Web

| Parameter | Default | Arti |
|---|---|---|
| `REDIS_HOST / REDIS_PORT` | `localhost:6379` | Broker telemetri |
| `TELEMETRY_CHANNEL` | `asv_telemetry` | Publikasi posisi/yaw/state |
| `VISION_CHANNEL` | `asv_vision` | Publikasi frame + deteksi |
| `MISSION_CHANNEL` | `asv_mission` | Publikasi status misi |
| `NGROK_HOST` | `dashboardaterolas.app` | URL dashboard web |
| `SERVER_UPLOAD_URL` | `https://…/api/images/upload` | Upload foto ke Laravel |

## Navigasi Umum

| Parameter | Default | Arti |
|---|---|---|
| `ACCEPTANCE_RADIUS_M` | `2.0` | Jarak dianggap "sampai" di waypoint |
| `THRUST_VALUE` | `0.9` | Thrust transit antar waypoint (0..1) |
| `TRANSITION_DURATION_S` | `0.5` | Masa tenang setelah ganti WP |
| `GEOFENCE_WIDTH_METERS` | `1.0` | Simpangan maks dari garis lintasan |

## Vision Global

| Parameter | Default | Arti |
|---|---|---|
| `FOCAL_LENGTH_PX` | `400` | Focal length kamera (px) utk estimasi jarak |
| `VISION_P_GAIN` | `1.9` | Gain koreksi yaw berbasis visi |
| `VISION_SMOOTHING_ALPHA` | `0.6` | Smoothing koreksi (EMA): tinggi = responsif, rendah = halus |
| `ROI_TOP_CUTOFF_PERCENT` | `0.20` | Bagian atas frame yang dipotong (hindari horizon/langit) |
| `YOLO_FRAME_SKIP` | `1` | Inferensi tiap N frame |
| `YOLO_INFERENCE_SIZE` | `320` | Resolusi input YOLO (lebih kecil = lebih cepat) |

## Misi Gate (Buoy)

| Parameter | Default | Arti |
|---|---|---|
| `VISION_ENABLED_LEGS` | `[1,3,5]` | Leg waypoint yang mengaktifkan koreksi gate |
| `GATE_WIDTH_METERS` | `1.0` | Lebar gate aktual (estimasi jarak) |
| `MIN_BUOY_AREA_PX` | `80` | Luas minimal deteksi agar dianggap pelampung |
| `GATE_AREA_SIMILARITY_RATIO` | `0.5` | Rasio kemiripan ukuran 2 buoy |

## Misi Foto Waypoint

| Parameter | Default | Arti |
|---|---|---|
| `STOP_AND_PHOTO_AT_WP` | `[]` | Daftar index WP yang berhenti & difoto |
| `WAYPOINT_PHOTO_STOP_DURATION_S` | `3.0` | Lama berhenti ketika fotokan |
| `DETECTION_CONFIRM_DURATION_S` | `0.5` | Deteksi harus stabil selama ini sebelum diyakini |

## Misi Kotak Hijau

| Parameter | Default | Arti |
|---|---|---|
| `PHOTO_BOX_LEGS` | `[6]` | Leg yang memicu misi kotak hijau |
| `SEARCH_THRUST` | `0.4` | Thrust saat memutar mencari kotak |
| `ALIGN_THRUST` | `0.3` | Thrust saat menyelaraskan diri |
| `RETREAT_THRUST` | `-0.7` | Thrust mundur setelah foto (negatif = mundur) |
| `RETREAT_DURATION_S` | `0.5` | Durasi mundur |
| `BOX_WIDTH_METERS` | `0.4` | Lebar kotak hijau aktual |
| `BOX_APPROACH_DISTANCE_M` | `2.0` | Jarak berhenti depan kotak hijau |
| `YAW_SEARCH_BOX` | `-90` | Sudut rotasi saat mencari |
| `BOX_SEARCH_LATERAL_THRUST` | `-0.2` | Thrust lateral saat mencari |

## Misi Kotak Biru (Foto Samping)

| Parameter | Default | Arti |
|---|---|---|
| `BLUE_BOX_PHOTO_LEGS` | `[8]` | Leg yang memicu misi kotak biru |
| `BLUE_BOX_WIDTH_METERS` | `0.6` | Lebar kotak biru aktual |
| `BLUE_BOX_APPROACH_DISTANCE_M` | `1.5` | Jarak berhenti depan kotak biru |
| `BLUE_BOX_LATERAL_OFFSET_M` | `-1.0` | Offset samping saat memotret |
| `BLUE_BOX_ALIGN_THRUST` | `0.3` | Thrust saat align |
| `BLUE_BOX_SEARCH_THRUST` | `0.2` | Thrust saat cari |
| `BLUE_BOX_YAW_SEARCH` | `-90` | Sudut rotasi saat cari |

## Misi Docking (Kotak Merah)

| Parameter | Default | Arti |
|---|---|---|
| `RED_BOX_NAV_AFTER_WP` | `8` | WP setelah ini mulai misi docking (setelah foto biru) |
| `RED_BOX_WIDTH_METERS` | `0.6` | Lebar kotak merah aktual |
| `RED_BOX_DOCK_DISTANCE_M` | `1.0` | Jarak "masuk dock" |
| `DOCK_ALIGN_THRUST` | `0.4` | Thrust saat align docking |
| `DOCK_HOLD_DURATION_S` | `5.0` | Berapa lama hold saat sudah dock |
| `YAW_SEARCH_DOCK` | `90` | Sudut rotasi saat cari dock |

## Tips Tuning Cepat

- **Kapal oleng saat transit** → kecilkan `THRUST_VALUE` atau besarkan `TRANSITION_DURATION_S`.
- **Melenceng dari garis** → kecilkan `GEOFENCE_WIDTH_METERS` / periksa koreksi gate di leg 1/3/5.
- **Deteksi tidak konsisten** → naikkan `DETECTION_CONFIRM_DURATION_S`, turunkan `YOLO_INFERENCE_SIZE` agar FPS naik.
- **Terlalu dekat kotak saat foto** → naikkan `*_APPROACH_DISTANCE_M`.