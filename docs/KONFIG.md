# Konfigurasi & Parameter Tuning

Satu-satunya sumber kebenaran: `app/settings.py` → disimpan otomatis ke
`config/tuning_params.json` saat GUI ditutup. Hanya key di `TUNING_PARAM_KEYS`
yang ikut tersimpan (path model, port serial, index kamera, host tidak ikut,
agar file tetap portable antar mesin).

Nilai default di tabel = default kode saat `tuning_params.json` kosong.

---

## Model YOLO

| Parameter | Default | Misi |
|---|---|---|
| `MODEL_PATH` | `weights/buoy.pt` | Gate (≥2 pelampung) |
| `BOX_MODEL_PATH` | `weights/box_hijau.pt` | Foto kotak hijau |
| `BLUE_BOX_MODEL_PATH` | `weights/best_blue_dark.pt` | Foto kotak biru (gelap) |
| `RED_DOCK_MODEL_PATH` | `weights/best_red_new.pt` | Docking kotak merah |

---

## Kamera & Video

| Parameter | Default | Arti |
|---|---|---|
| `CAMERA_INDEX` | `1` | Kamera navigasi — preferensi; auto-negosiasi pilih resolusi tertinggi |
| `WAYPOINT_PHOTO_CAMERA_INDEX` | `2` | Kamera bawah air (foto WP8) |
| `CAMERA_FLIP_MODE` | `1` | `0=normal, 1=h-flip, 2=v-flip, 3=180°` |
| `FRAME_WIDTH / FRAME_HEIGHT` | `1280×720` | Default (ditimpa hasil negosiasi) |
| `CAMERA_TARGET_FPS` | `30` | FPS yang diusahakan (negosiasi + pacing loop) |
| `CAMERA_MAX_AUTO_WIDTH / HEIGHT` | `1920×1080` | Batas resolusi yang diminta saat probing |
| `CAMERA_MIN_ACCEPT_FPS` | `25` | Mode dengan fps nyata di bawah ini ditolak |
| `SECONDARY_PREVIEW_INTERVAL` | `3` | Baca kamera bawah 1× per N frame (cuma preview GUI) |
| `SESSION_VIDEO_FPS` | `10.0` | FPS rekaman video sesi (bila `ENABLE_SESSION_RECORDING`) |

---

## Telemetri & Hardware

| Parameter | Default | Arti |
|---|---|---|
| `SERIAL_PORT` | `COM8` | Port Pixhawk (Windows). Pi: `/dev/ttyACM0` |
| `BAUD_RATE` | `57600` | Baud MAVLink |
| `OFFBOARD_STREAM_RATE_HZ` | `30` | Rate stream perintah offboard |
| `MAV_CONNECT_TIMEOUT_S` | `2.0` | Batas sekali coba connect (loop tidak freeze) |
| `MAV_RETRY_INTERVAL_S` | `5.0` | Jeda antar percobaan connect |
| `TELEM_HYSTERESIS_FRAMES` | `5` | Frame bagus/gagal beruntun sebelum pindah sumber MOCK↔REAL |

---

## Navigasi Umum

| Parameter | Default | Arti |
|---|---|---|
| `ACCEPTANCE_RADIUS_M` | `2.0` | Jarak dianggap "sampai" waypoint |
| `THRUST_VALUE` | `0.9` | Thrust transit (0..1) |
| `TRANSITION_DURATION_S` | `0.5` | Masa tenang setelah ganti WP |
| `GEOFENCE_WIDTH_METERS` | `1.0` | Simpangan maks dari garis lintasan |

---

## Vision Global

| Parameter | Default | Arti |
|---|---|---|
| `FOCAL_LENGTH_PX` | `400` | Focal length kamera (px) untuk estimasi jarak |
| `VISION_P_GAIN` | `1.9` | Gain koreksi yaw berbasis visi |
| `VISION_SMOOTHING_ALPHA` | `0.6` | EMA smoothing (tinggi=responsif, rendah=halus) |
| `ROI_TOP_CUTOFF_PERCENT` | `0.20` | Potong bagian atas frame (hindari langit) |
| `YOLO_FRAME_SKIP` | `2` | Inferensi tiap N+1 frame |
| `YOLO_INFERENCE_SIZE` | `256` | Resolusi input YOLO (kecil=cepat) |
| `YOLO_RESULT_MAX_AGE_S` | `0.5` | Umur maks hasil async sebelum dianggap basi |

---

## Filter & Kontroler Galat (dipakai navigator via C core)

| Parameter | Default | Arti |
|---|---|---|
| `PID_KP` | `1.0` | Gain P koreksi yaw |
| `PID_KI` | `0.0` | Gain I |
| `PID_KD` | `0.0` | Gain D |
| `PID_DEADBAND` | `0.01` | Zona mati radian (~0.6°) |
| `PID_OUTPUT_LIMIT` | `0.6` | Clamp output PID (radian) |
| `PID_INTEGRAL_LIMIT` | `0.3` | Anti-windup |
| `COMPLEMENTARY_ALPHA` | `0.6` | Alpha filter komplementer (1=gyro, 0=terukur) |
| `EKF_PROCESS_NOISE` | `0.05` | Noise proses EKF heading (rad²) |
| `EKF_MEAS_NOISE` | `0.10` | Noise pengukuran EKF heading (rad²) |

---

## Misi Gate (Buoy)

| Parameter | Default | Arti |
|---|---|---|
| `VISION_ENABLED_LEGS` | `[1,3,5]` | Leg dengan koreksi gate |
| `GATE_WIDTH_METERS` | `1.0` | Lebar gate aktual |
| `MIN_BUOY_AREA_PX` | `16` | Luas minimal deteksi agar dianggap pelampung |
| `GATE_AREA_SIMILARITY_RATIO` | `0.5` | Rasio kemiripan ukuran 2 buoy |
| `GATE_VERTICAL_ALIGN_PX` | `75` | Ambang beda sumbu-Y merah/hijau agar dianggap 1 gate |
| `GATE_PASS_DISTANCE_M` | `1.2` | Jarak lolos gate → switch cepat ke gate berikut |
| `GATE_LOST_TOLERANCE_FRAMES` | `5` | Frame toleransi kehilangan deteksi |

### Filter false-positive buoy (`app/detection_validation.py`)

| Parameter | Default | Arti |
|---|---|---|
| `BUOY_CONF_THRESHOLD` | `0.35` | Confidence dasar model gate |
| `BUOY_MIN_COLOR_FRACTION` | `0.05` | Fraksi piksel crop yang harus cocok warna |
| `BUOY_MIN_SATURATION` | `0.35` | Saturasi HSV minimum |
| `BUOY_MAX_ASPECT_DEVIATION` | `0.5` | Batas `|w/h-1|` (bentuk mirip bola) |
| `BUOY_CONF_SMALL_THRESHOLD` | `0.20` | Conf min untuk box kecil (deteksi jauh) |
| `BUOY_SMALL_AREA_PX` | `80` | Ambang "box kecil" |
| `BUOY_TRACK_MATCH_PX` | `30` | Radius match tracking antar frame |
| `BUOY_TRACK_BOOST` | `1.5` | Faktor boost conf buoy yang sudah dilacak |

### Ambang per-class (hijau/merah)

| Parameter | Default | Arti |
|---|---|---|
| `BUOY_CONF_THRESHOLD_GREEN` | `0.30` | Conf hijau (lebih longgar, tenggelam duluan di gelap) |
| `BUOY_CONF_SMALL_THRESHOLD_GREEN` | `0.15` | Conf kecil hijau |
| `BUOY_MIN_COLOR_FRACTION_GREEN` | `0.04` | Fraksi warna hijau |
| `BUOY_MIN_SATURATION_GREEN` | `0.30` | Saturasi hijau |
| `BUOY_CONF_THRESHOLD_RED` | `0.35` | Conf merah |
| `BUOY_CONF_SMALL_THRESHOLD_RED` | `0.20` | Conf kecil merah |
| `BUOY_MIN_COLOR_FRACTION_RED` | `0.05` | Fraksi warna merah |
| `BUOY_MIN_SATURATION_RED` | `0.35` | Saturasi merah |

### Threshold adaptif gelap

| Parameter | Default | Arti |
|---|---|---|
| `BUOY_ADAPTIVE_ENABLED` | `True` | Aktifkan pelemahan ambang saat frame gelap |
| `BUOY_BRIGHTNESS_THRESHOLD` | `80` | Mean V di bawah ini = "gelap" |
| `BUOY_ADAPTIVE_MIN_SATURATION` | `0.20` | Saturasi min saat gelap |
| `BUOY_ADAPTIVE_MIN_VALUE` | `20` | Value min saat gelap |
| `BUOY_ADAPTIVE_COLOR_FRACTION_MULT` | `0.5` | Pengali fraksi warna saat gelap |
| `DETECTION_DEBUG` | `False` | Cetak alasan tolak per objek (maks 1×/detik) |

---

## Misi Foto Waypoint

| Parameter | Default | Arti |
|---|---|---|
| `STOP_AND_PHOTO_AT_WP` | `[]` | Index WP yang berhenti & difoto |
| `WAYPOINT_PHOTO_STOP_DURATION_S` | `3.0` | Lama berhenti saat fotokan |
| `DETECTION_CONFIRM_DURATION_S` | `0.5` | Deteksi harus stabil selama ini sebelum diyakini |

---

## Misi Kotak Hijau

| Parameter | Default | Arti |
|---|---|---|
| `PHOTO_BOX_LEGS` | `[6]` | Leg yang memicu misi kotak hijau |
| `SEARCH_THRUST` | `0.4` | Thrust saat memutar cari kotak |
| `ALIGN_THRUST` | `0.3` | Thrust saat menyelaraskan |
| `RETREAT_THRUST` | `-0.7` | Thrust mundur setelah foto (negatif = mundur) |
| `RETREAT_DURATION_S` | `0.5` | Durasi mundur |
| `BOX_WIDTH_METERS` | `0.4` | Lebar kotak hijau aktual |
| `BOX_APPROACH_DISTANCE_M` | `2.0` | Jarak berhenti depan kotak hijau |
| `YAW_SEARCH_BOX` | `-90` | Sudut rotasi saat mencari |
| `BOX_SEARCH_LATERAL_THRUST` | `-0.2` | Thrust lateral saat mencari |

---

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

---

## Misi Docking (Kotak Merah)

| Parameter | Default | Arti |
|---|---|---|
| `RED_BOX_NAV_AFTER_WP` | `8` | WP setelah ini mulai docking (setelah foto biru) |
| `RED_BOX_WIDTH_METERS` | `0.6` | Lebar kotak merah aktual |
| `RED_BOX_DOCK_DISTANCE_M` | `1.0` | Jarak "masuk dock" |
| `DOCK_ALIGN_THRUST` | `0.4` | Thrust saat align docking |
| `DOCK_HOLD_DURATION_S` | `5.0` | Lama hold saat sudah dock |
| `YAW_SEARCH_DOCK` | `90` | Sudut rotasi saat cari dock |

---

## Kendali Manual (RC / Gamepad / QGC)

| Parameter | Default | Arti |
|---|---|---|
| `MANUAL_ENABLED` | `True` | Aktifkan kendali manual lokal |
| `MANUAL_MAX_SURGE` | `0.6` | Batas gas manual |
| `MANUAL_MAX_YAW` | `0.7` | Batas belok manual |
| `MANUAL_DEADBAND` | `0.05` | Zona mati stick |
| `MANUAL_EXPO` | `0.3` | Eksponensial stick |
| `MANUAL_RATE_LIMIT` | `2.0` | Batas laju perubahan |
| `RC_TIMEOUT_MS` | `500` | RC dianggap hilang setelah ini |
| `RC_CH_THROTTLE` | `3` | Channel RC throttle (1-based) |
| `RC_CH_YAW` | `4` | Channel RC yaw |
| `RC_CH_MODE` | `5` | Channel RC mode |
| `RC_CH_DEADMAN` | `7` | Channel RC deadman (aktif bila > 1500) |
| `MANUAL_LOST_HOLD_S` | `1.0` | Lama tahan netral setelah RC hilang |
| `QGC_MANUAL_TIMEOUT_S` | `0.5` | Paket MANUAL_CONTROL basi > ini → netral |

---

## Failsafe

| Parameter | Default | Arti |
|---|---|---|
| `FAILSAFE_ENABLED` | `True` | Aktifkan failsafe otomatis |
| `FAILSAFE_TELEM_TIMEOUT_S` | `2.0` | Telemetri stale > ini → thrust 0 + coba RTL |
| `FAILSAFE_LOW_BATT_PCT` | `20.0` | Ambang persen baterai rendah |
| `FAILSAFE_LOW_VOLT_V` | `13.2` | Ambang tegangan baterai rendah |
| `FAILSAFE_LOW_BATT_HOLD_S` | `3.0` | Tahan selama ini sebelum RTL (anti spike sesaat) |

---

## Tips Tuning Cepat

- **Kapal oleng saat transit** → kecilkan `THRUST_VALUE` atau besarkan `TRANSITION_DURATION_S`.
- **Melenceng dari garis** → kecilkan `GEOFENCE_WIDTH_METERS` / periksa koreksi gate leg 1/3/5.
- **Deteksi tidak konsisten** → naikkan `DETECTION_CONFIRM_DURATION_S`, atau turunkan
  `YOLO_INFERENCE_SIZE` agar FPS naik.
- **Terlalu dekat kotak saat foto** → naikkan `*_APPROACH_DISTANCE_M`.
- **FPS kamera turun** → pastikan `CAMERA_TARGET_FPS`/negosiasi cocok; naikkan
  `SECONDARY_PREVIEW_INTERVAL` bila kamera bawah tetap berat.
- **Baterai LiPO 4S**: full 16.8 V, empty 12.8 V → persen linear dari tegangan
  kalau firmware tak kirim `battery_remaining`.
