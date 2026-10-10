# Arsitektur Aterkia ASV

Dokumen ini menjelaskan **bagaimana sistem bekerja** (alur data, modul,
keputusan desain). Untuk arti tiap parameter lihat `docs/KONFIG.md`, untuk
perilaku kapal per state lihat `docs/STATES.md`, untuk cara menjalankan
lihat `docs/RUNBOOK.md`.

---

## 1. Alur Data (1 loop navigasi)

```
  Kamera depan ─▶ frame ─▶ [resize + ROI] ─▶ YoloWorker (async, thread)
                                                    │ detections (cap waktu)
                                                    ▼
  Kamera bawah ─▶ preview 10 Hz / foto WP8 ◀─┐  State Machine (C)
                                             │       │ thrust, target_yaw_rad
  GPS/Attitude ◀─ Pixhawk ── MAVLink ──────▶ Telemetri  │
  (pymavlink, non-blocking)                    │       ▼
                                               │  OutputLink.set_attitude_target()
                                               │       │ (SET_ATTITUDE_TARGET)
                                               └───────┴──▶ Pixhawk (OFFBOARD)
                                                                 │
                                                                 ▼
                                                    GUI PyQt5 (video + peta + HUD)
```

- **GUI ↔ navigator**: satu proses, lewat sinyal PyQt (`newData`) — **bukan**
  Redis/WebSocket. Relay web sudah dihapus dari repo.
- **Frame video**: dikirim sebagai numpy array lewat sinyal; tidak lewat jaringan.
- **Pixhawk tetap pemilik ESC/mixer.** Companion hanya mengirim setpoint
  offboard (`SET_ATTITUDE_TARGET` + thrust). Mixer differential drive di C
  hanya untuk HUD/log/uji (`OutputLink.mixer_pwm()`), **tidak** dikirim ke
  Pixhawk sebagai PWM.

---

## 2. Modul & Tanggung Jawab

| File | Tanggung jawab |
|---|---|
| `main.py` | Entry GUI PyQt5: peta folium, video dual (nav + bawah), panel tuning, waypoint recorder, pemilih backend |
| `app/navigator.py` | **Navigator produksi**: loop 30 Hz, state machine C, pipeline visi, offboard ke Pixhawk, foto lokal |
| `app/simulator.py` | Dev darat: `GroundSimNavigator` (kamera+YOLO asli, posisi mock) & `MockSimNavigator` (tanpa hardware) |
| `app/yolo_worker.py` | Worker inferensi YOLO async (thread murni, satu untuk semua model) |
| `app/output_link.py` | Output ke Pixhawk: `SET_ATTITUDE_TARGET`, RTL, disarm |
| `app/safety.py` | Failsafe otomatis (stale-link / low-batt) |
| `app/camera.py` | Buka kamera V4L2/DSHOW, negosiasi resolusi/fps, flip |
| `app/camera_manager.py` | Kelola kamera primer/sekunder, fallback, reconnect |
| `app/mavlink_telemetry.py` | Baca MAVLink non-blocking (attitude, posisi, baterai) |
| `app/manual_link.py` | Kendali manual lokal (RC/gamepad) → (surge, yaw) |
| `app/qgc_offboard.py` | Kendali manual dari QGC (Xbox → MAVLink `MANUAL_CONTROL`) |
| `app/detection_validation.py` | Filter pasca-YOLO: warna/bentuk buoy + debug |
| `app/pixhawk_check.py` | Cek parameter failsafe Pixhawk saat startup |
| `app/settings.py` | Semua parameter + path (satu sumber kebenaran) |
| `app/settings_dialog.py` | Panel tuning GUI (tab) |
| `app/slim_map.py` | Widget peta ringan (tile) |
| `app/gui_theme.py`, `app/logutil.py` | Tema GUI & logger/throttle |
| `app/aterkia_core.py` | **Tipis** ctypes wrapper ke `core/libaterkia.so` (Linux) / `aterkia_core.dll` (Windows), tanpa logika |

Oracle uji (dulu di `app/`, kini pindah ke `tests/_ref/`): `geo.py`,
`gate_sequencer.py`, `filtering.py`, `fuzzy.py`, `state_machine.py`.

---

## 3. Inti C (`core/`)

Komputasi deterministik & kritis-waktu di C agar byte-identik dengan oracle
Python dan bisa diuji otomatis (`tests/test_core_*.py`, ctypes cross-check).

| File | Header | Fungsi |
|---|---|---|
| `nav_math.c` | `nav_math.h` | Geodesi murni (haversine, bearing, CTE, normalize) |
| `gate_vision.c` | `gate_vision.h` | Koleksi pasangan gate, pinhole, geometri buoy, sequencer latch+memori |
| `fuzzy.c` | `fuzzy.h` | Fuzzy Sugeno singleton (gain gate/docking) |
| `state_machine.c` | `state_machine.h` | **18 state** misi + sub-mesin mundur |
| `control_filters.c` | `control_filters.h` | PID+deadband/anti-windup, complementary, EKF heading |
| `thruster_mixer.c` | `thruster_mixer.h` | Differential drive: surge+yaw → PWM 1100–1900 |
| `mavlink_bridge.c` | `mavlink_bridge.h` | Framing MAVLink v1 + serial (byte-identik pymavlink) |
| `ecu_link.c` | `ecu_link.h` | UART STM32 (frame 8 byte + checksum) |
| `arbitrator.c` | `arbitrator.h` | Prioritas KILL > MANUAL > AUTO |
| `manual_control.c` | `manual_control.h` | Skala/limit/expo/rate-limit kendali manual |
| `rc_decode.c` | `rc_decode.h` | Dekode channel RC → (surge, yaw, mode, deadman) |
| `mode_manager.c` | `mode_manager.h` | Mode op: AUTO / MANUAL / KILL + transisi |
| `avoidance.c` | `avoidance.h` | Avoidance sederhana (deflect yaw ≤ 25°) |
| `gamepad_input.cpp` | `gamepad_input.hpp` | Baca gamepad USB (joystick Linux) |

Build lokal menghasilkan `core/libaterkia.so` (Linux) / `core/aterkia_core.dll`
(Windows); **di-ignore git**. Bila belum di-build, `aterkia_core.C_AVAILABLE=False`
dan gagal secara eksplisit (bukan angka palsu).

---

## 4. Kamera & Visi

### Negosiasi kamera otomatis (`app/camera.py`)

- `negotiate_camera(index, target_fps, min_fps)` — cari mode (codec × resolusi × fps)
  dari `CAMERA_CANDIDATE_SIZES` (1920×1080 → 320×240), MJPG dulu lalu YUYV.
  Tiap kandidat di-warm-up singkat lalu fps-nya **diukur nyata** (`_measure_fps`),
  bukan sekadar `CAP_PROP_FPS`.
- `negotiate_best_camera(preferred)` — quick-score tiap index kamera
  (`_probe_index_quality`, hanya backend native V4L2/DSHOW), pilih index dengan
  luas piksel terbesar yang tetap ≥ `CAMERA_MIN_ACCEPT_FPS`, lalu negosiasi
  detail di index itu. `CAMERA_INDEX` dihormati sebagai preferensi.
- Dipakai navigator & simulator via `open_camera(..., auto_highest=True)`;
  ukuran hasil negosiasi menimpa `FRAME_WIDTH/HEIGHT`.
- Default: target **30 fps**, min accept **25 fps**. Hasil umum di dev:
  `1280×720 MJPG @ 30 fps`.
- Kamera bawah (`WAYPOINT_PHOTO_CAMERA_INDEX`, default index 2) hanya
  **preview GUI 10 Hz** (`SECONDARY_PREVIEW_INTERVAL=3`) + foto WP8. Membacanya
  tiap frame akan berebut bandwidth USB dengan kamera navigasi.

### Worker inferensi async (`app/yolo_worker.py`)

- Satu thread + antrean kapasitas 1 (frame terbaru menang). Loop utama hanya
  `submit(model_id, model, frame, conf)` lalu `latest_status()` — tak pernah
  menunggu inferensi.
- Frame-skip (`YOLO_FRAME_SKIP`) mengatur seberapa sering frame dikirim ke worker,
  bukan menahan loop. Video kamera tetap mengalir walau inferensi lambat.
- Hasil tiap model dicap waktu (monotonic); caller menolak hasil basi
  (`YOLO_RESULT_MAX_AGE_S`).
- Dipakai bersama navigator produksi & simulator (satu sumber logika).
- Threading murni (bukan `QThread`) agar bisa diuji tanpa `QApplication`.

### Filter pasca-YOLO buoy (`app/detection_validation.py`)

YOLO kadang mengira objek mirip bola (mis. wajah operator) sebagai
`red_ball`/`green_ball`. `validate_buoy()` menolak false-positive dengan
syarat fisik buoy:

1. warna pekat: merah (hue 0..10 ∪ 170..179) / hijau (hue 35..85), saturasi ≥
   `BUOY_MIN_SATURATION`, value ≥ ambang (bukan hitam);
2. bentuk bola: `|w/h - 1| ≤ BUOY_MAX_ASPECT_DEVIATION`;
3. ukuran: area ≥ `MIN_BUOY_AREA_PX`;
4. ambang adaptif saat frame gelap (`BUOY_ADAPTIVE_*`, dihitung dari mean V
   thumbnail 64×64 — murah).

Ambang per-class (`*_GREEN` / `*_RED`) dan boost tracking (`BUOY_TRACK_*`)
melengkapi deteksi jauh. Semua bisa di-tuning lewat `config/tuning_params.json`
(masuk `TUNING_PARAM_KEYS`) tanpa kode baru. `DETECTION_DEBUG` mencetak alasan
tolak (maks 1×/detik).

---

## 5. Output & Failsafe

### `app/output_link.py` (jalur produksi)

- Thruster = `SET_ATTITUDE_TARGET` via MAVLink:
  - `thrust` = surge `[-1 mundur .. +1 maju]`;
  - `yaw` = quaternion dari heading target;
  - `lateral` = body roll rate (belokan halus, bit 0 `type_mask`).
- Type mask: bit 1 & 2 diabaikan (pakai attitude yaw penuh), bit 0 = roll rate
  (hanya bila ada `lateral_thrust`).
- Throttle stream oleh `OFFBOARD_STREAM_RATE_HZ`; `prepare_offboard()` mengirim
  setpoint awal; `request_rtl()` (`MAV_CMD_DO_SET_MODE`, Rover RTL mode 11) dan
  `send_disarm()` (`MAV_CMD_COMPONENT_ARM_DISARM`) best-effort + throttled.

### `app/safety.py` (failsafe otomatis)

- **Stale-link**: tak ada ATTITUDE/GLOBAL_POSITION selama
  `FAILSAFE_TELEM_TIMEOUT_S` → thrust 0 + coba RTL best-effort.
- **Low-batt**: persen ATAU tegangan di bawah ambang, **ditahan** selama
  `FAILSAFE_LOW_BATT_HOLD_S` (agar spike sesaat tak memicu RTL palsu).
- Pulih otomatis bila kondisi normal. Aksi RTL didelegasikan ke `OutputLink`.
- Dipakai bersama navigator produksi & simulator.

Pixhawk **tetap** pemegang failsafe utama (RCIN + geofence bawaan); companion
hanya lapisan tambahan.

---

## 6. Mode & Kendali Manual

- **AUTO** — misi otonom (state machine).
- **MANUAL** — dua sumber bisa aktif: gamepad/RC lokal (`app/manual_link.py`)
  dan QGC (Xbox → `MANUAL_CONTROL` → `app/qgc_offboard.py`). Arbitrase di C
  (`arb_manual2`): dua-duanya aktif = **konflik → netral + warning** (tidak
  ada rebutan diam-diam). Paket QGC basi > `QGC_MANUAL_TIMEOUT_S` → netral.
- **KILL** — tombol GUI: masuk jalur C (`mode_manager` KILL) + kirim DISARM
  ke Pixhawk via `OutputLink.send_disarm()`.

Output servo/ESC tetap dari Pixhawk — companion memantau & mengarbitrasi,
bukan penggerak PWM.

---

## 7. Hardware & Power

- **LiPO 4S (motor)** → ESC → Thruster; Pixhawk pemegang PWM utama.
- **LiFePO4 4S (PC/NUC)** → NUC + kamera + sensor.
- **E-stop hardware** → STM32 → TC4420 kill line (hardware cut, independen NUC/Pixhawk).
- **STM32 supervisor**: ACS758 (arus), NTC B3950 (suhu ESC), BME280 (ambient),
  divider (tegangan) → frame 8 byte tiap 100 ms ke NUC via UART.
- **Watchdog**: heartbeat NUC putus > 500 ms ATAU E-stop → STM32 potong driver.
- **Baterai (GUI)**: tegangan pack dari SYS_STATUS/BATTERY_STATUS; persen linear
  dari tegangan bila firmware tak kirim `battery_remaining`. Chip: hijau ≥50%,
  kuning 20–50%, merah <20%, abu = no data (LiPO 4S full 16.8 V, empty 12.8 V).

---

## 8. Build & Test

```bash
# Build library C (sekali / tiap ubah core/src/*.c)
make -C core linux        # -> core/libaterkia.so
make -C core check        # -Wall -Wextra -Werror
# Windows (MinGW): mingw32-make -C core windows -> core/aterkia_core.dll

# Cross-check C vs Python (butuh gcc; tanpa gcc otomatis skip)
python3 -m unittest discover -s tests -p 'test_core_*.py'

# GUI headless (uji cepat tanpa display)
QT_QPA_PLATFORM=offscreen python main.py
```

> Catatan: `tests/` kini **hanya** berisi `test_core_*.py` (cross-check C↔Python)
> dan `tests/_ref/` (oracle Python). Test GUI/fitur lama dihapus untuk
> produksi yang ramping; verifikasi perilaku end-to-end lewat smoke test misi
> penuh (lihat `docs/RUNBOOK.md`).

---

## 9. Path & Data

- `config/plan.csv` — waypoint (format `lat,lon`, baris pertama header).
  **Satu-satunya file plan.**
- `config/tuning_params.json` — parameter tuning tersimpan (dari `save_params`).
- `weights/*.pt` — model YOLO. **Tidak masuk git** (~6 MB/file); unduh via
  `scripts/download_weights.sh` / `.ps1`.
- `data/` — output runtime (captures, foto waypoint, video sesi, peta sementara).
  Di-ignore git.
- `core/libaterkia.so` — di-ignore git (build lokal).
