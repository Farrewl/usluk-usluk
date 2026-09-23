# Arsitektur ASV Backend

## Alur Data (1 loop navigasi)

```
                ┌────────────────────────────────────────────────┐
                │                  Raspberry Pi / PC dev         │
                │                                                │
  Kamera depan ─┴──▶ frame ──▶ [resize + ROI] ──▶ [YOLO deteksi] │
                                                    │ detections │
                                                    ▼            │
  Kamera bawah ─▶ foto WP8 (smart capture) ◀──┐  [State Machine] │
                                               │       │ thrust, │
                                               │       │ target_ │
  GPS/Attitude ◀── Pixhawk ── MAVLink ──────▶ [Telemetri] yaw_rad │
  (pymavlink, busy)                            │       ▼         │
                                               │  [SET_ATTITUDE_TARGET] ──▶ Pixhawk (offboard)
                                               └──────┴────────────┬────────┘
                                                                   │ Redis publish
                                                                    ▼
                                              GUI PyQt5 (folio map, video, tuning)
                                                                    │
                                                                    ▼
                                              app/gateway.py ──WebSocket──▶ Dashboard web
```

## Modul & Tanggung Jawab

| File | Tanggung jawab | Kode lama |
|---|---|---|
| `main.py` | Entry GUI PyQt5: map folium dragable, video, panel tuning, waypoint recorder | `main.py` |
| `app/simulator.py` | 2 simulator darat (kamera asli / mock) — tanpa Pixhawk | `modules/simulation_in_ground.py` + `modules/simulator_backend.py` |
| `app/navigator.py` | Navigasi asli: state machine misi, fuzzy Sugeno, MAVLink offboard, Redis | `modules/navigator_backend.py` |
| `app/settings.py` | Semua parameter + path standar (satu sumber kebenaran) | `modules/config.py` |
| `app/geo.py` | Matematika geodetik murni (haversine, bearing, cross-track) | (duplikat di 3 file lama) |
| `app/gateway.py` | Relay Redis → WebSocket ke dashboard web | `modules/web_gateway.py` |
| `app/scripts/` | Tool dev: `cek_camera`, `tes_exposure`, `tes_conflict` | file di root |

## Komunikasi Antar Proses

- **Telemetri & vision ke web**: Redis pub/sub, channel `asv_telemetry`, `asv_vision`, `asv_mission`.
- **GUI ↔ navigator**: langsung via sinyal PyQt (`newData`) — satu proses, tidak lewat Redis.
- **Frame video**: dikirim sebagai numpy array lewat sinyal; ke web via base64/jpeg (Redis).

## Path & Data

- `config/plan.csv` — waypoint (format `lat,lon`, baris pertama header). **Satu-satunya file plan.**
- `config/tuning_params.json` — parameter tuning tersimpan saat GUI ditutup (`save_params`).
- `weights/*.pt` — model YOLO. **Tidak masuk git** (~6 MB/file).
- `data/` — output runtime (captures, video sesi, map sementara). Di-ignore git.

## Bersih-bersih Git (Tahap 1)

History commit lama masih berisi file berat (38 MB pack): 7× `.pt`, banyak `.jpg`,
`__pycache__`, dan `temp_map.html`. Kami **tidak menulis ulang history** (repo tim
sudah di GitHub). Yang dilakukan:

1. `git rm --cached` semua `*.pt`, `*.jpg`, `*.pyc`, `temp_map.html`.
2. `.gitignore` baru mencegah file itu kembali.
3. Weights diunduh via `scripts/download_weights.sh` (bukan dari git).
4. (Opsional nanti) slim history dengan `git filter-repo` setelah semua stabil.

## Peta Konversi Python → C/C++ (Tahap 2)

Inti navigasi dikonversi bertahap ke `core/`. Matematika & MAVLink murni C,
visi pakai C++ (ONNX Runtime). GPU/UI tetap Python (PyQt5).

| Logika (Python) | Target C/C++ | Bahasa |
|---|---|---|
| `app/geo.py` (haversine, bearing, cross-track, normalize) | `core/src/nav_math.c` | C |
| `_set_attitude_target`, `_update_telemetry`, heartbeat (pymavlink) | `core/src/mavlink_bridge.c` (framing v1 + serial, byte-identik pymavlink) | C |
| Fuzzy Sugeno gate & docking (kini `app/fuzzy.py` — skfuzzy>=0.5 gagal bikin singleton) | `core/src/fuzzy.c` | C |
| State machine misi (`run()` navigator → kini referensi `app/state_machine.py`) | `core/src/state_machine.c` | C |
| YOLO inferensi (ultralytics → ONNX) | `core/src/vision_detector.cpp` (ONNX Runtime) | C++ |
| Kamera, resize, ROI | `core/src/camera.cpp` (OpenCV) | C++ |
| Redis publish | `core/src/redis_pub.c` (hiredis) | C |

GUI `main.py` tetap Python dan berkomunikasi dengan core C/C++ lewat
Redis (telemetri) + ZMQ/shared memory (frame). Detail tahapan di
`core/README.md`. Setiap modul C punya unit test (`core/tests/`) dan
diperbandingkan output-nya terhadap Python sebelum dipakai.

## Kamera & Filter Deteksi Buoy (Tahap 3)

### Negosiasi kamera otomatis (`app/camera.py`)

- `negotiate_camera(index, target_fps, min_fps)` — cari mode (codec ×
  resolusi × fps) dari kandidat `CAMERA_CANDIDATE_SIZES`
  (1920×1080 → 320×240) dengan MJPG dulu, YUYV cadangan. Tiap kandidat
  di-warm-up singkat lalu fps-nya DIUKUR NYATA (`_measure_fps`), bukan
  hanya cek `CAP_PROP_FPS`.
- `negotiate_best_camera(preferred)` — quick-score tiap index kamera
  (`_probe_index_quality`, hanya backend native V4L2/DSHOW + `/dev/videoN`
  cek supaya probe cepat), pilih index dengan luas piksel terbesar yang
  tetap ≥ `CAMERA_MIN_ACCEPT_FPS`, lalu negosiasi detail hanya di index
  itu. `CAMERA_INDEX` tetap dihormati sebagai preferensi (seri menang
  preferen).
- Dipakai kamera navigasi (simulator & navigator) via `open_camera(..., 
  auto_highest=True)`; ukuran hasil negosiasi menimpa
  `FRAME_WIDTH/HEIGHT` agar video sintetis & HUD konsisten.
- Hasil aktual pada mesin dev: `1280×720 MJPG @ 30 fps` (index 1 yang
  di-set di config cuma sanggup 320×240 — otomatis pindah ke index 0).

### Video stabil ~30 fps (`app/simulator.py`)

- `_YoloWorker` (QThread) menjalankan inferensi YOLO ASINKRON: loop video
  menyerahkan frame lalu lanjut, hasil terbaru diambil lewat `latest()`.
  Frame-skip (`YOLO_FRAME_SKIP`) hanya mengatur seberapa sering frame
  dikirim ke worker, bukan menahan loop. Sambungan kamera 30 fps tetap
  mengalir walau inferensi CPU ~30-600 ms.
- Pacing `1/CAMERA_TARGET_FPS` + log `[SIM] FPS aktual` tiap 5 s.
- `main.py` men-downscale frame di sisi numpy (INTER_AREA) sebelum QImage
  agar main-thread GUI tidak terbebani QPixmap 1280×720 per frame.

### Filter pasca-YOLO buoy (`app/detection_validation.py`)

YOLO kadang mengira objek mirip bola (mis. wajah operator saat uji
darat) sebagai `red_ball`/`green_ball`. `validate_buoy()` menolak
false-positive dengan syarat fisik buoy:

1. warna pekat: merah (hue 0..10 ∪ 170..179) / hijau (hue 35..85) pada
   HSV, saturasi ≥ `BUOY_MIN_SATURATION`, value ≥ 40 (bukan hitam);
2. bentuk bola: `|w/h - 1| ≤ BUOY_MAX_ASPECT_DEVIATION`;
3. ukuran: area ≥ `MIN_BUOY_AREA_PX`.

Plus `BUOY_CONF_THRESHOLD` (default 0.55) menaikkan ambang confidence
model gate. Ambang semua bisa di-tuning lewat `config/tuning_params.json`
(keyware masuk `TUNING_PARAM_KEYS`), tanpa row GUI baru.

Wiring: `app/navigator.py::_detect_objects` (buoy yang gagal validasi
tidak masuk `last_detections`), `app/simulator.py::_YoloWorker`
(deteksi ditolak tidak digambar — pakai `draw_validated_boxes()` mengganti
`results[0].plot()`), dan `scripts/test_deteksi.py` (menampilkan
`*ditolak` per deteksi). Validasi unit: `tests/test_detection_validation.py`.