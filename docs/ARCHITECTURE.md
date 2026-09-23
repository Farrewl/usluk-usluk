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