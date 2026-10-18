# Aterkia ASV — Kapal Robot Otonom

Backend navigasi & GUI untuk **Autonomous Surface Vehicle** kompetisi Roboboat.
Kapal menyusuri waypoint, melewati **gate** (2 pelampung), memotret **kotak**,
dan **docking** ke kotak merah — berbasis visi (YOLO) + telemetri MAVLink.

```
Kamera ─▶ YOLO (async, ONNX) ─▶ gate vision + PID/complementary/EKF (C core)
                                    │
                                    ▼
      Thruster mixer (differential) ─▶ MAVLink OFFBOARD ─▶ Pixhawk → ESC → Thruster
```

Pixhawk tetap pemilik ESC/mixer; companion (NUC/Pi) hanya mengirim setpoint
offboard. Repo ini **khusus Linux** (mini-PC kapal). Dashboard web adalah
**repo terpisah** yang membaca `data/monitor/`.

---

## Cepat Mulai

Model YOLO & library inti jalan tanpa langkah manual: cukup 2 langkah.

```bash
# 1. Setup .venv ringan (tanpa torch) + build inti C + verifikasi
./scripts/setup_minimal.sh

# 2. Jalankan GUI (menulis ekspor monitoring ke data/monitor/)
./scripts/run_gui.sh
```

Sudah pernah setup? Langsung `./scripts/run_gui.sh`. Setup idempoten:
`.venv` dan library yang sudah ada **dipakai ulang, tak ditimpa**; install
dilewati bila paket inti sudah bisa di-import.

> **Catatan: JANGAN `source .venv/bin/activate`, pakai script wrapper.**
> Script aktivasi python `venv` menyimpan **path absolut** dan rusak bila
> repo **dipindah/di-rename**, apalagi kalau pathnya mengandung **spasi**
> (mis. `asv usluk-usluk`) — `source` diam-diam gagal, `python` kembali ke
> sistem. Solusi: selalu jalankan lewat `./scripts/run_gui.sh` atau langsung
> `./.venv/bin/python3 -O main.py` — keduanya kebal path & shell. Python versi
> venv tetap bisa dipakai walau aktivasi rusak, karena `bin/python3` relatif
> terhadap `pyvenv.cfg`.

Lintasan & identitas tim diatur di `config/site.json` (+ `config/trackA.json`,
`config/trackB.json`) — bisa juga ganti dari dropdown GUI / env
`ASV_TRACK=A|B`. Model YOLO sudah ada di `weights/` (tak perlu unduh).

**Hasil setup:** `.venv ±540 MB` (tanpa torch — hemat >2×), GUI PyQt, 4 model
YOLO ONNX, MAVLink, plus `core/libaterkia.so`. Rincian di `requirements.txt`.

---

## Struktur Repo

```
usluk-usluk/
├── main.py                 # ENTRY POINT — GUI (kapal produksi)
├── app/                    # Paket Python (GUI, navigasi, visi, telemetri)
├── core/                   # Inti C/C++ (deterministik) + Makefile
│   ├── include/*.h
│   ├── src/*.c             # nav_math, gate_vision, fuzzy, state_machine, ...
│   └── libaterkia.so       # build lokal (di-ignore git)
├── config/
│   ├── site.json           # identitas tim, lintasan, pengaturan monitor
│   ├── trackA.json / trackB.json   # waypoint + leg misi per lintasan
│   ├── tuning_params.json  # Parameter tuning (simpan otomatis)
│   └── pixhawk_required*.txt       # spec cek failsafe (ArduPilot / PX4)
├── weights/                # Model YOLO ONNX (runtime) — IKUT git
├── data/                   # Output runtime (foto + data/monitor) — di-ignore git
├── scripts/                # setup_minimal.sh, run_gui.sh, verify_env.py, monitor_read.py
├── requirements.txt
└── README.md               # (dokumen ini) acuan lengkap
```

---

## Cara Menjalankan (mini-PC + laptop operator)

App kapal jalan di **mini-PC** (kamera, Pixhawk, ESC, misi). Laptop operator
hanya membuka browser ke **dashboard web**, yang merupakan **repo terpisah**
dan membaca data dari `data/monitor/` (lihat "Ekspor Monitoring" di bawah).

| Peran | Mesin | Cara |
|---|---|---|
| **Kapal** (kamera, Pixhawk, ESC, misi) | mini-PC Linux | `./scripts/setup_minimal.sh` sekali → `./scripts/run_gui.sh`. Auto-misi jalan + tulis `data/monitor/`. |
| **Dashboard** (web) | mini-PC yang sama | **repo terpisah** — baca `data/monitor/telemetry.json` + `frame.jpg` + `photos.json`, sajikan ke browser. |
| **Operator** | laptop apa pun | buka browser ke IP mini-PC (port dashboard). Tanpa install apa pun. |

### Menyambungkan laptop operator ke mini-PC (WiFi)

Mini-PC dan laptop **harus di jaringan WiFi yang SAMA** (satu SSID/hotspot).

1. **Cari IP mini-PC**: `hostname -I` atau `ip addr show wlan0` (lihat `inet`).
2. **Di laptop**: buka alamat dashboard, mis. `http://192.168.1.42:8080`
   (port sesuai deploy repo web).
3. **Kalau tak bisa dibuka**:
   - Pastikan SSID sama & tak ada "client isolation"/AP isolation di router.
   - `ping <ip-mini-pc>` lalu cek port dari laptop.
   - Firewall mini-PC: `sudo ufw allow 8080/tcp` (port dashboard).
4. **Tanpa router?** Mini-PC bisa jadi hotspot sendiri:
   ```bash
   nmcli device wifi hotspot ifname wlan0 ssid ASV-Aterkia password aterkia123
   ```
   Lalu laptop connect ke WiFi `ASV-Aterkia`, buka `http://10.42.0.1:8080`.
5. **mDNS** (opsional, kalau `avahi-daemon` aktif): bisa juga
   `http://<hostname-mini-pc>.local:8080` tanpa hafal IP.

### QGroundControl

- **Mudah**: pasang QGC di mini-PC, hubungkan langsung ke Pixhawk via USB.
- **Lewat jaringan** (QGC di laptop operator): jalankan mini-PC dengan env
  ```bash
  GCS_FORWARD=1 GCS_IP=<ip-laptop> GCS_PORT=14550 ./scripts/run_gui.sh
  ```
  lalu di QGC laptop: *Add Link → UDP → Listen port 14550*.

> **Penting: port serial USB cuma boleh dipegang SATU program.** Kalau
> QGroundControl (atau mavlink-router) sudah membuka `/dev/ttyACM*`, app ASV
> tak bisa ikut konek — status jadi
> `Pixhawk: /dev/ttyACM1 (NO-LINK BUSY(QGC?) | ...)`. **Tutup QGC dulu**
> (app akan retry otomatis dalam ≤5 s), lalu pakai pola *GCS forward* di atas
> agar QGC tetap dapat telemetri lewat UDP. Cek pemegang port:
> `lsof /dev/ttyACM*` — kalau muncul `QGroundControl`, itulah biangnya.

### Percepat startup & deteksi

- **Fast-start kamera ON default** — kamera utama dibuka sekali (mode cache /
  1280x720), tanpa sweep resolusi berulang. Negosiasi penuh tetap bisa dipicu
  manual via tombol **Scan** (tab Misi). Matikan: `ASV_CAM_FAST=0`.
- **Deteksi kontinu ON default** — model GATE ikut dijalankan saat pra-misi
  (WAITING_GPS dan waypoint-nav) dan frame ber-kotak deteksi tampil di GUI
  **dan** file bridge `data/monitor/frame.jpg`. Matikan: `ASV_DETECT=0`.
  Model YOLO dimuat di thread (lazy); kotak muncul begitu model siap.
- **Kotak = semua deteksi mentah + confidence** (label `kls 0.87`) — bukan
  hanya yang lulus validasi warna. Validasi HSV tetap dipakai untuk keputusan
  navigasi, tampilan selalu jujur apa adanya.
- **Kamera auto-cari** — kalau index yang dikonfigurasi gagal dibuka/restart,
  app otomatis scan index lain sekali dan memakainya (log `pakai index N`).
  Tidak perlu edit config di lapangan.

---

## Setup Manual (rincian)

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt     # Linux; tanpa torch
.venv/bin/python scripts/verify_env.py        # import inti + inferensi ONNX
```

### Library C (`core/`) — wajib untuk navigator

Komputasi deterministik & kritis-waktu ada di C. Dibangun otomatis oleh
`setup_minimal.sh` (idempoten). Manual:

```bash
make -C core linux                 # Linux / Pi  -> core/libaterkia.so
make -C core check                 # -Wall -Wextra -Werror
```

Tanpa library C, `aterkia_core.C_AVAILABLE=False` dan navigator **gagal
eksplisit** (bukan angka palsu). Modul: `nav_math`, `gate_vision`, `fuzzy`,
`state_machine` (18 state), `control_filters`, `thruster_mixer`,
`mavlink_bridge`, `ecu_link`, `arbitrator`, `manual_control`, `rc_decode`,
`mode_manager`, `avoidance`, `gamepad_input`.

### Model YOLO (ikut di repo)

4 model ada di `weights/`: `buoy.onnx` (gate), `box_hijau.onnx` (kotak hijau),
`best_blue_dark.onnx` (kotak biru), `best_red_new.onnx` (docking merah).
Runtime memakai ONNX via `cv2.dnn` (tanpa torch). **Tak perlu unduh.**

### Waypoint & lintasan

Dua file lintasan: `config/trackA.json` (start dock merah) dan
`config/trackB.json` (start dock hijau). Pilih lintasan di `config/site.json`
(`"TRACK": "A"`), dropdown GUI tab Misi, atau env `ASV_TRACK=A|B`.

---

## Arsitektur

### Alur data (1 loop navigasi)

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

- **GUI ↔ navigator**: satu proses, lewat sinyal PyQt (`newData`) — bukan
  jaringan. Frame video dikirim sebagai numpy array lewat sinyal.
- **Pixhawk tetap pemilik ESC/mixer.** Companion hanya mengirim setpoint
  offboard (`SET_ATTITUDE_TARGET` + thrust). Mixer differential drive di C
  hanya untuk HUD/log/uji (`OutputLink.mixer_pwm()`), **tidak** dikirim ke
  Pixhawk sebagai PWM.

### Modul `app/`

| File | Fungsi |
|---|---|
| `settings.py` | Semua parameter & path (satu sumber kebenaran) |
| `navigator.py` | Navigator produksi: state machine + MAVLink offboard (loop 30 Hz) |
| `yolo_worker.py` | Worker inferensi YOLO async (thread murni, satu untuk semua model) |
| `yolo_onnx.py` | Inferensi YOLOv8 ONNX via `cv2.dnn` (`OnnxDetector`, tanpa torch) |
| `output_link.py` | Output ke Pixhawk: `SET_ATTITUDE_TARGET`, RTL, disarm |
| `safety.py` | Failsafe otomatis (stale-link / low-batt) |
| `camera.py` | Buka kamera V4L2, negosiasi resolusi, flip |
| `camera_manager.py` | Kelola kamera primer/sekunder, fallback, reconnect |
| `mavlink_telemetry.py` | Baca MAVLink non-blocking (attitude, posisi, baterai) |
| `manual_link.py` | Kendali manual lokal (RC/gamepad) → (surge, yaw) |
| `qgc_offboard.py` | Kendali manual dari QGC (Xbox → `MANUAL_CONTROL`) |
| `detection_validation.py` | `validate_buoy()` filter pasca-YOLO + debug |
| `pixhawk_check.py` | Cek parameter failsafe Pixhawk saat startup (spec per firmware) |
| `monitor_export.py` | Ekspor **file bridge** → `data/monitor/` untuk web terpisah |
| `telemetry_snapshot.py` | Rakit snapshot JSON lengkap (identitas…health, semua `params`) |
| `settings_dialog.py` | Panel tuning GUI |
| `slim_map.py` | Widget peta ringan (tile) |
| `gui_theme.py` | Tema/style GUI |
| `logutil.py` | Logger + throttle |
| `aterkia_core.py` | ctypes wrapper ke `core/libaterkia.so`, tanpa logika |
| `geotag.py` | Geo-tag foto (Day/Date/Time, koordinat DD & DM, SOG, COG) |

### Inti C (`core/`)

| File | Fungsi |
|---|---|
| `nav_math.c` | Geodesi murni (haversine, bearing, CTE, normalize) |
| `gate_vision.c` | Koleksi pasangan gate, pinhole, geometri buoy, sequencer latch |
| `fuzzy.c` | Fuzzy Sugeno singleton (gain gate/docking) |
| `state_machine.c` | **18 state** misi + sub-mesin mundur |
| `control_filters.c` | PID+deadband/anti-windup, complementary, EKF heading |
| `thruster_mixer.c` | Differential drive: surge+yaw → PWM 1100–1900 |
| `mavlink_bridge.c` | Framing MAVLink v1 + serial |
| `ecu_link.c` | UART STM32 (frame 8 byte + checksum) |
| `arbitrator.c` | Prioritas KILL > MANUAL > AUTO |
| `manual_control.c` | Skala/limit/expo/rate-limit kendali manual |
| `rc_decode.c` | Dekode channel RC → (surge, yaw, mode, deadman) |
| `mode_manager.c` | Mode op: AUTO / MANUAL / KILL + transisi |
| `avoidance.c` | Avoidance sederhana (deflect yaw ≤ 25°) |
| `gamepad_input.cpp` | Baca gamepad USB (joystick Linux) |

### Kamera & Visi

- **Negosiasi kamera otomatis** (`app/camera.py`): cari mode (codec × resolusi
  × fps), MJPG dulu lalu YUYV; fps **diukur nyata** (`_measure_fps`), bukan
  sekadar `CAP_PROP_FPS`. `negotiate_best_camera(preferred)` menilai tiap index
  dan memakai yang luas pikselnya terbesar yang tetap ≥ `CAMERA_MIN_ACCEPT_FPS`.
  Default: target **30 fps**, min accept **25 fps**; hasil umum 1280×720 MJPG
  @ 30 fps.
- **Kamera bawah** (`WAYPOINT_PHOTO_CAMERA_INDEX`, default index 2) hanya
  **preview GUI 10 Hz** (`SECONDARY_PREVIEW_INTERVAL=3`) + foto WP8 — membaca
  tiap frame akan berebut bandwidth USB dengan kamera navigasi.
- **Worker inferensi async**: satu thread + antrean kapasitas 1 (frame terbaru
  menang). Loop utama hanya `submit(...)` lalu `latest_status()` — tak pernah
  menunggu inferensi. Frame-skip (`YOLO_FRAME_SKIP`) mengatur seberapa sering
  frame dikirim ke worker. Hasil dicap waktu (monotonic); caller menolak hasil
  basi (`YOLO_RESULT_MAX_AGE_S`). Threading murni (bukan `QThread`) agar bisa
  diuji tanpa `QApplication`.
- **Filter pasca-YOLO buoy** (`app/detection_validation.py`): YOLO kadang
  mengira objek mirip bola (mis. wajah operator) sebagai `red_ball`/`green_ball`.
  `validate_buoy()` menolak false-positive dengan syarat fisik buoy:
  1) warna pekat (merah hue 0..10 ∪ 170..179 / hijau hue 35..85, saturasi ≥
  `BUOY_MIN_SATURATION`, value cukup); 2) bentuk bola
  `|w/h - 1| ≤ BUOY_MAX_ASPECT_DEVIATION`; 3) area ≥ `MIN_BUOY_AREA_PX`;
  4) ambang adaptif saat frame gelap (`BUOY_ADAPTIVE_*`). `DETECTION_DEBUG`
  mencetak alasan tolak (maks 1×/detik).

### Output & Failsafe

- **Output** (`app/output_link.py`): thruster = `SET_ATTITUDE_TARGET` via
  MAVLink — `thrust` = surge `[-1 mundur .. +1 maju]`; `yaw` = quaternion dari
  heading target; `lateral` = body roll rate (belokan halus). Throttle stream
  oleh `OFFBOARD_STREAM_RATE_HZ`; `request_rtl()` (`MAV_CMD_DO_SET_MODE`, Rover
  RTL mode 11) dan `send_disarm()` (`MAV_CMD_COMPONENT_ARM_DISARM`) best-effort.
- **Failsafe app** (`app/safety.py`): **stale-link** (tak ada ATTITUDE/
  GLOBAL_POSITION selama `FAILSAFE_TELEM_TIMEOUT_S` → thrust 0 + coba RTL);
  **low-batt** (persen ATAU tegangan di bawah ambang, ditahan selama
  `FAILSAFE_LOW_BATT_HOLD_S` agar spike sesaat tak memicu RTL palsu). Pulih
  otomatis bila normal. Pixhawk **tetap** pemegang failsafe utama.

### Mode & Kendali Manual

- **AUTO** — misi otonom (state machine).
- **MANUAL** — dua sumber bisa aktif: gamepad/RC lokal (`app/manual_link.py`)
  dan QGC (Xbox → `MANUAL_CONTROL` → `app/qgc_offboard.py`). Arbitrase di C
  (`arb_manual2`): dua-duanya aktif = **konflik → netral + warning**. Paket QGC
  basi > `QGC_MANUAL_TIMEOUT_S` → netral.
- **KILL** — tombol GUI: masuk jalur C (`mode_manager` KILL) + kirim DISARM ke
  Pixhawk via `OutputLink.send_disarm()`.

Saat **bukan AUTO**, navigator menahan aksi misi (thrust 0 / target yaw tetap)
dan tidak meminta deteksi baru.

---

## State Machine Misi (18 state)

Keputusan transisi dihitung di **C** (`sm_transition`); Python hanya menerapkan
efek samping (wp++ / reset timer / reset vision). `sm_guard` menangani state
"penjaga" (telemetri / waypoint).

```
        ┌────────────── WAITING_GPS ──────────────┐
        │   (tunggu GPS 3D fix + attitude)        │
        ▼                                         │
  WAYPOINT_NAV ── deteksi gate aktif (leg 1,3,5)  │
        ├─ wp ∈ PHOTO_BOX_LEGS      ──▶ APPROACH_BOX_SEARCH
        ├─ wp ∈ BLUE_BOX_PHOTO_LEGS ──▶ APPROACH_BLUE_BOX_SEARCH
        ├─ wp ∈ STOP_AND_PHOTO_AT_WP ─▶ TAKE_WAYPOINT_PHOTO
        ├─ jarak < ACCEPTANCE_RADIUS ─▶ WAYPOINT_TRANSITION (wp++)
        └─ (pilihan lain)            ── tetap nav + koreksi gate

  Guard (menyela misi): NO_TELEM · NO_WAYPOINTS · MISSION_COMPLETE
```

| # | State | Perilaku utama | Ke state berikutnya |
|---|---|---|---|
| 0 | `WAITING_GPS` | Thrust 0 sambil tunggu GPS fix + attitude | `WAYPOINT_NAV` |
| 1 | `NO_TELEM` | Guard: GPS/attitude belum ada → thrust 0 | `WAYPOINT_NAV` (saat pulih) |
| 2 | `NO_WAYPOINTS` | Guard: lintasan kosong → idle | `WAYPOINT_NAV` |
| 3 | `MISSION_COMPLETE` | Semua WP selesai; thrust 0 | (selesai) |
| 4 | `WAYPOINT_NAV` | Thrust tetap; target yaw = bearing ke WP; koreksi gate bila leg vision aktif | lihat peta |
| 5 | `WAYPOINT_TRANSITION` | Masa tenang setelah ganti WP | `WAYPOINT_NAV` |
| 6 | `TAKE_WAYPOINT_PHOTO` | Berhenti `WAYPOINT_PHOTO_STOP_DURATION_S`, snap foto WP | `APPROACH_RED_BOX_SEARCH` / `WAYPOINT_TRANSITION` |
| 7 | `APPROACH_BOX_SEARCH` | Rotasi cari kotak hijau | `APPROACH_BOX_ALIGN` |
| 8 | `APPROACH_BOX_ALIGN` | Koreksi yaw halus → kotak hijau | `TAKE_PHOTO` |
| 9 | `TAKE_PHOTO` | Berhenti, fotokan kotak hijau | `RETREAT` |
| 10 | `RETREAT` | Brake → netral → mundur | `WAYPOINT_NAV` (wp++) |
| 11 | `APPROACH_BLUE_BOX_SEARCH` | Rotasi cari kotak biru | `APPROACH_BLUE_BOX_ALIGN` |
| 12 | `APPROACH_BLUE_BOX_ALIGN` | Koreksi yaw & offset lateral → kotak biru | `TAKE_BLUE_BOX_PHOTO` |
| 13 | `TAKE_BLUE_BOX_PHOTO` | Stabilisasi, kamera bawah air (smart capture) | `BLUE_BOX_RETREAT` |
| 14 | `BLUE_BOX_RETREAT` | Mundur; lanjut docking bila model merah ada | `APPROACH_RED_BOX_SEARCH` / `WAYPOINT_NAV` |
| 15 | `APPROACH_RED_BOX_SEARCH` | Rotasi cari kotak merah (docking) | `APPROACH_RED_BOX_ALIGN` |
| 16 | `APPROACH_RED_BOX_ALIGN` | Koreksi yaw presisi; hilang → SEARCH | `RED_BOX_DOCKED` |
| 17 | `RED_BOX_DOCKED` | Hold diam `DOCK_HOLD_DURATION_S` | `WAYPOINT_TRANSITION` / `MISSION_COMPLETE` |

Sub-mesin mundur (`RETREAT` / `BLUE_BOX_RETREAT`): `IDLE → START_BRAKE →
GOTO_NEUTRAL → START_REVERSE` (mundur `RETREAT_THRUST` selama
`RETREAT_DURATION_S`).

---

## Parameter

Satu-satunya sumber kebenaran: `app/settings.py` → disimpan otomatis ke
`config/tuning_params.json` saat GUI ditutup. Hanya key di `TUNING_PARAM_KEYS`
yang ikut tersimpan (path model, port serial, index kamera, host tidak ikut,
agar file tetap portable antar mesin). Nilai default = default kode saat
`tuning_params.json` kosong.

### Model YOLO

| Parameter | Default | Misi |
|---|---|---|
| `MODEL_PATH` | `weights/buoy.onnx` | Gate (≥2 pelampung) |
| `BOX_MODEL_PATH` | `weights/box_hijau.onnx` | Foto kotak hijau |
| `BLUE_BOX_MODEL_PATH` | `weights/best_blue_dark.onnx` | Foto kotak biru (gelap) |
| `RED_DOCK_MODEL_PATH` | `weights/best_red_new.onnx` | Docking kotak merah |

### Kamera & Video

| Parameter | Default | Arti |
|---|---|---|
| `CAMERA_INDEX` | `1` | Kamera navigasi — preferensi; auto-negosiasi pilih resolusi tertinggi |
| `WAYPOINT_PHOTO_CAMERA_INDEX` | `2` | Kamera bawah air (foto WP8) |
| `CAMERA_FLIP_MODE` | `1` | `0=normal, 1=h-flip, 2=v-flip, 3=180°` |
| `FRAME_WIDTH / FRAME_HEIGHT` | `1280×720` | Default (ditimpa hasil negosiasi) |
| `CAMERA_TARGET_FPS` | `30` | FPS yang diusahakan |
| `CAMERA_MAX_AUTO_WIDTH / HEIGHT` | `1920×1080` | Batas resolusi saat probing |
| `CAMERA_MIN_ACCEPT_FPS` | `25` | Mode dengan fps nyata di bawah ini ditolak |
| `SECONDARY_PREVIEW_INTERVAL` | `3` | Baca kamera bawah 1× per N frame (preview GUI) |
| `SESSION_VIDEO_FPS` | `10.0` | FPS rekaman video sesi (bila aktif) |

### Telemetri & Hardware

| Parameter | Default | Arti |
|---|---|---|
| `SERIAL_PORT` | `COM8` | Port Pixhawk. Kosong/`COM8` → auto-deteksi (Linux: pilih otomatis dari pyserial, mis. `/dev/ttyACM1`) |
| `BAUD_RATE` | `57600` | Baud MAVLink |
| `OFFBOARD_STREAM_RATE_HZ` | `30` | Rate stream perintah offboard |
| `MAV_CONNECT_TIMEOUT_S` | `2.0` | Batas sekali coba connect |
| `MAV_RETRY_INTERVAL_S` | `5.0` | Jeda antar percobaan connect |

### Navigasi Umum

| Parameter | Default | Arti |
|---|---|---|
| `ACCEPTANCE_RADIUS_M` | `2.0` | Jarak dianggap "sampai" waypoint |
| `THRUST_VALUE` | `0.9` | Thrust transit (0..1) |
| `TRANSITION_DURATION_S` | `0.5` | Masa tenang setelah ganti WP |
| `GEOFENCE_WIDTH_METERS` | `1.0` | Simpangan maks dari garis lintasan |

### Vision Global

| Parameter | Default | Arti |
|---|---|---|
| `FOCAL_LENGTH_PX` | `400` | Focal length kamera (px) untuk estimasi jarak |
| `VISION_P_GAIN` | `1.9` | Gain koreksi yaw berbasis visi |
| `VISION_SMOOTHING_ALPHA` | `0.6` | EMA smoothing |
| `ROI_TOP_CUTOFF_PERCENT` | `0.20` | Potong bagian atas frame (hindari langit) |
| `YOLO_FRAME_SKIP` | `2` | Inferensi tiap N+1 frame |
| `YOLO_INFERENCE_SIZE` | `256` | Resolusi input YOLO (kecil = cepat) |
| `YOLO_RESULT_MAX_AGE_S` | `0.5` | Umur maks hasil async sebelum dianggap basi |

### Filter & Kontroler Galat (via C core)

| Parameter | Default | Arti |
|---|---|---|
| `PID_KP` / `PID_KI` / `PID_KD` | `1.0` / `0.0` / `0.0` | Gain PID koreksi yaw |
| `PID_DEADBAND` | `0.01` | Zona mati radian (~0.6°) |
| `PID_OUTPUT_LIMIT` | `0.6` | Clamp output PID (radian) |
| `PID_INTEGRAL_LIMIT` | `0.3` | Anti-windup |
| `COMPLEMENTARY_ALPHA` | `0.6` | Alpha filter komplementer (1=gyro, 0=terukur) |
| `EKF_PROCESS_NOISE` / `EKF_MEAS_NOISE` | `0.05` / `0.10` | Noise proses/ukur EKF heading (rad²) |

### Misi Gate (Buoy)

| Parameter | Default | Arti |
|---|---|---|
| `VISION_ENABLED_LEGS` | `[1,3,5]` | Leg dengan koreksi gate |
| `GATE_WIDTH_METERS` | `1.0` | Lebar gate aktual |
| `MIN_BUOY_AREA_PX` | `16` | Luas minimal deteksi agar dianggap pelampung |
| `GATE_AREA_SIMILARITY_RATIO` | `0.5` | Rasio kemiripan ukuran 2 buoy |
| `GATE_VERTICAL_ALIGN_PX` | `75` | Ambang beda sumbu-Y merah/hijau agar dianggap 1 gate |
| `GATE_PASS_DISTANCE_M` | `1.2` | Jarak lolos gate → switch cepat ke gate berikut |
| `GATE_LOST_TOLERANCE_FRAMES` | `5` | Frame toleransi kehilangan deteksi |

Filter false-positive buoy (`app/detection_validation.py`): `BUOY_CONF_THRESHOLD`
(0.35), `BUOY_MIN_COLOR_FRACTION` (0.05), `BUOY_MIN_SATURATION` (0.35),
`BUOY_MAX_ASPECT_DEVIATION` (0.5), `BUOY_CONF_SMALL_THRESHOLD` (0.20),
`BUOY_SMALL_AREA_PX` (80), `BUOY_TRACK_MATCH_PX` (30), `BUOY_TRACK_BOOST` (1.5).

Ambang per-class: `BUOY_CONF_THRESHOLD_GREEN` (0.30),
`BUOY_CONF_SMALL_THRESHOLD_GREEN` (0.15), `BUOY_MIN_COLOR_FRACTION_GREEN`
(0.04), `BUOY_MIN_SATURATION_GREEN` (0.30); `BUOY_CONF_THRESHOLD_RED` (0.35),
`BUOY_CONF_SMALL_THRESHOLD_RED` (0.20), `BUOY_MIN_COLOR_FRACTION_RED` (0.05),
`BUOY_MIN_SATURATION_RED` (0.35).

Threshold adaptif gelap: `BUOY_ADAPTIVE_ENABLED` (True),
`BUOY_BRIGHTNESS_THRESHOLD` (80), `BUOY_ADAPTIVE_MIN_SATURATION` (0.20),
`BUOY_ADAPTIVE_MIN_VALUE` (20), `BUOY_ADAPTIVE_COLOR_FRACTION_MULT` (0.5),
`DETECTION_DEBUG` (False).

### Misi Foto Waypoint

| Parameter | Default | Arti |
|---|---|---|
| `STOP_AND_PHOTO_AT_WP` | `[]` | Index WP yang berhenti & difoto |
| `WAYPOINT_PHOTO_STOP_DURATION_S` | `3.0` | Lama berhenti saat fotokan |
| `DETECTION_CONFIRM_DURATION_S` | `0.5` | Deteksi harus stabil selama ini sebelum diyakini |

### Misi Kotak Hijau

| Parameter | Default | Arti |
|---|---|---|
| `PHOTO_BOX_LEGS` | `[6]` | Leg yang memicu misi kotak hijau |
| `SEARCH_THRUST` / `ALIGN_THRUST` | `0.4` / `0.3` | Thrust saat mencari / menyelaraskan |
| `RETREAT_THRUST` / `RETREAT_DURATION_S` | `-0.7` / `0.5` | Thrust & durasi mundur (negatif = mundur) |
| `BOX_WIDTH_METERS` | `0.4` | Lebar kotak hijau aktual |
| `BOX_APPROACH_DISTANCE_M` | `2.0` | Jarak berhenti depan kotak hijau |
| `YAW_SEARCH_BOX` | `-90` | Sudut rotasi saat mencari |
| `BOX_SEARCH_LATERAL_THRUST` | `-0.2` | Thrust lateral saat mencari |

### Misi Kotak Biru (Foto Samping)

| Parameter | Default | Arti |
|---|---|---|
| `BLUE_BOX_PHOTO_LEGS` | `[8]` | Leg yang memicu misi kotak biru |
| `BLUE_BOX_WIDTH_METERS` | `0.6` | Lebar kotak biru aktual |
| `BLUE_BOX_APPROACH_DISTANCE_M` | `1.5` | Jarak berhenti depan kotak biru |
| `BLUE_BOX_LATERAL_OFFSET_M` | `-1.0` | Offset samping saat memotret |
| `BLUE_BOX_ALIGN_THRUST` / `BLUE_BOX_SEARCH_THRUST` | `0.3` / `0.2` | Thrust align / cari |
| `BLUE_BOX_YAW_SEARCH` | `-90` | Sudut rotasi saat cari |

### Misi Docking (Kotak Merah)

| Parameter | Default | Arti |
|---|---|---|
| `RED_BOX_NAV_AFTER_WP` | `8` | WP setelah ini mulai docking (setelah foto biru) |
| `RED_BOX_WIDTH_METERS` | `0.6` | Lebar kotak merah aktual |
| `RED_BOX_DOCK_DISTANCE_M` | `1.0` | Jarak "masuk dock" |
| `DOCK_ALIGN_THRUST` | `0.4` | Thrust saat align docking |
| `DOCK_HOLD_DURATION_S` | `5.0` | Lama hold saat sudah dock |
| `YAW_SEARCH_DOCK` | `90` | Sudut rotasi saat cari dock |

### Kendali Manual (RC / Gamepad / QGC)

| Parameter | Default | Arti |
|---|---|---|
| `MANUAL_ENABLED` | `True` | Aktifkan kendali manual lokal |
| `MANUAL_MAX_SURGE` / `MANUAL_MAX_YAW` | `0.6` / `0.7` | Batas gas / belok manual |
| `MANUAL_DEADBAND` / `MANUAL_EXPO` / `MANUAL_RATE_LIMIT` | `0.05` / `0.3` / `2.0` | Zona mati / eksponensial / batas laju |
| `RC_TIMEOUT_MS` | `500` | RC dianggap hilang setelah ini |
| `RC_CH_THROTTLE` / `RC_CH_YAW` / `RC_CH_MODE` / `RC_CH_DEADMAN` | `3` / `4` / `5` / `7` | Channel RC (1-based; deadman aktif bila > 1500) |
| `MANUAL_LOST_HOLD_S` | `1.0` | Lama tahan netral setelah RC hilang |
| `QGC_MANUAL_TIMEOUT_S` | `0.5` | Paket `MANUAL_CONTROL` basi > ini → netral |

### Failsafe

Pengecekan parameter failsafe **di Pixhawk** (`app/pixhawk_check.py`) memilih
file spec sesuai firmware, dideteksi dari HEARTBEAT (`autopilot`):
- **PX4** (autopilot=12) → `config/pixhawk_required_px4.txt`
  (`NAV_DLL_ACT`, `NAV_RCL_ACT`, `COM_LOW_BAT_ACT`, `BAT_LOW_THR`,
  `BAT_CRIT_THR`, `GF_ACTION`).
- **ArduPilot** (autopilot=3) → `config/pixhawk_required.txt`
  (`FS_GCS_ENABLE`, `FS_THR_ENABLE`, `FS_CRASH_CHECK`, `ARMING_CHECK`, ...).

Hasil cek tampil di GUI sebagai `[PX4] ...` / `[ArduPilot] ...`. HANYA MEMBACA —
ubah lewat QGC bila melanggar.

Failsafe **di sisi app** (independen firmware):

| Parameter | Default | Arti |
|---|---|---|
| `FAILSAFE_ENABLED` | `True` | Aktifkan failsafe otomatis |
| `FAILSAFE_TELEM_TIMEOUT_S` | `2.0` | Telemetri stale > ini → thrust 0 + coba RTL |
| `FAILSAFE_LOW_BATT_PCT` | `20.0` | Ambang persen baterai rendah |
| `FAILSAFE_LOW_VOLT_V` | `13.2` | Ambang tegangan baterai rendah |
| `FAILSAFE_LOW_BATT_HOLD_S` | `3.0` | Tahan selama ini sebelum RTL (anti spike) |

### Tips Tuning Cepat

- **Kapal oleng saat transit** → kecilkan `THRUST_VALUE` atau besarkan
  `TRANSITION_DURATION_S`.
- **Melenceng dari garis** → kecilkan `GEOFENCE_WIDTH_METERS` / periksa koreksi
  gate leg 1/3/5.
- **Deteksi tidak konsisten** → naikkan `DETECTION_CONFIRM_DURATION_S`, atau
  turunkan `YOLO_INFERENCE_SIZE` agar FPS naik.
- **Terlalu dekat kotak saat foto** → naikkan `*_APPROACH_DISTANCE_M`.
- **FPS kamera turun** → pastikan `CAMERA_TARGET_FPS`/negosiasi cocok; naikkan
  `SECONDARY_PREVIEW_INTERVAL` bila kamera bawah tetap berat.
- **Baterai LiPO 4S**: full 16.8 V, empty 12.8 V → persen linear dari tegangan
  kalau firmware tak kirim `battery_remaining`.

---

## Ekspor Monitoring (file bridge)

Repo ini **tidak berisi dashboard web**. App kapal hanya **menulis** kondisi
lengkapnya ke `data/monitor/`, yang dibaca oleh **repo web terpisah** (mini-PC
yang sama) dan disajikan ke browser operator:

| File | Isi | Frekuensi |
|---|---|---|
| `telemetry.json` | snapshot LENGKAP (lihat skema di bawah) | `MONITOR_HZ` (default 5 Hz) |
| `frame.jpg` | frame kamera NAV ter-annotasi YOLO | `MONITOR_FRAME_HZ` (default 10 Hz) |
| `frame_bawah.jpg` | kamera bawah (bila ada) | idem |
| `photos.json` | indeks foto misi + geo-tag | tiap foto |
| `events.jsonl` | log kejadian (append, 1 baris/kejadian) | saat terjadi |

Semua ditulis **atomik** (`<nama>.tmp` → `os.replace`) sehingga pembaca tak
pernah melihat file separuh. Pembaca disarankan menangani `FileNotFoundError`/
JSON parsial: saat app baru mulai file belum ada.

### Skema `telemetry.json`

```jsonc
{
  "schema": 1,              // versi kontrak; naik bila ada perubahan breaking
  "ts": 1734567890.12,      // waktu tulis (epoch detik)
  "identity": {
    "team": "Aterkia", "pt": "Universitas Diponegoro",
    "track": "B",           // lintasan aktif: "A" | "B"
    "track_start": {"lat": -6.9, "lon": 110.4, "color": "red"}
  },
  "pose":   {"lat": -6.9, "lon": 110.4,
             "yaw_deg": 12.0, "pitch_deg": 1.0, "roll_deg": -2.0},
  "motion": {"sog_kn": 2.3, "sog_kmh": 4.3, "cog_deg": 12.0, "groundspeed": 1.2},
  "power":  {"battery_pct": 88.0, "voltage_v": 15.2, "current_a": 3.1},
  "link": {
    "mavlink_ok": true, "port": "/dev/ttyACM1", "reason": "",
    "gcs_forward": false, "qgc_active": false, "rc_ok": true,
    "manual_source": "AUTO", "telem_source": "REAL",
    "pixhawk_check": "[PX4] OK"
  },
  "mission": {
    "state": "NAVIGATING",      // state mesin misi (18 state)
    "op_mode": "AUTO",          // AUTO | MANUAL | HOLD | KILL
    "wp_idx": 2, "wp_total": 5, "dist_to_wp_m": 3.5,
    "failsafe_active": false, "failsafe_reason": "",
    "status": "ok", "avoid_active": false, "avoid_bias": 0.0,
    "flags": {
      "prep": true, "start": true, "balls": 3,
      "surface": false, "under": false, "dock": false
    }
  },
  "vision": {
    "gate_passed": 3,
    "models": {                 // per model YOLO
      "gate":  {"detected": true, "n": 2, "age_s": 0.05},
      "box":   {"detected": false, "n": 0, "age_s": 0.1},
      "blue":  {"detected": false, "n": 0, "age_s": 0.1},
      "red":   {"detected": false, "n": 0, "age_s": 0.1}
    }
  },
  "camera": {"status": { /* status kamera (dict bebas) */ }},
  "last_photo": { /* info foto terakhir, lihat photos.json */ },
  "params": { /* SEMUA parameter config/run-time kapal */ },
  "health": {"uptime_s": 512.4}
}
```

Satuan: `sog_kn` knot, `sog_kmh` km/jam, `groundspeed` m/detik, sudut derajat,
`dist_to_wp_m` meter, `battery_pct` persen (atau `null`). Field bisa `null` saat
data belum ada (mis. GPS belum fix). `params` memuat seluruh atribut `Config`.

### `photos.json` (geo-tag)

Array objek hasil `app/geotag.py`:

```jsonc
[{
  "kind": "surface",                     // surface | under | dock | wp
  "file": "x.jpg",
  "path": "data/captures/x.jpg",         // path relatif root repo kapal
  "day": "Mon", "date": "17/10/2026", "time": "14:03:22",
  "ts": 1734567890.0,                     // epoch detik
  "lat": -6.9, "lon": 110.4,
  "sog_kn": 2.3, "sog_kmh": 4.3, "cog_deg": 12.0,
  "coord_dd": "S 6.90000 E 110.40000",   // Degree Decimal
  "coord_dm": "S 6° 54,0000' E 110° 24,0000'",  // Degree Minute (koma)
  "sidecar": "data/captures/x.json"       // geo-tag di samping JPEG (atau null)
}]
```

Gambar asli di `data/captures/` (kotak hijau) dan `data/waypoint_captures/`
(kotak biru / waypoint); sidecar `.json` ada di samping tiap JPEG.

### `events.jsonl`

Satu objek JSON per baris, append-only:

```jsonc
{"ts": 1734567890.1, "kind": "photo", "photo_kind": "surface", "path": "..."}
```

`kind` contoh: `photo` (bisa ditambah `state`, `failsafe`, `gate`).

### Pengaturan monitor

Di `config/site.json` (atau env `ASV_MONITOR*`):

| Kunci | Default | Arti |
|---|---|---|
| `MONITOR_ENABLED` | `true` | Matikan total ekspor (`ASV_MONITOR=0`) |
| `MONITOR_DIR` | `data/monitor` | Folder keluaran (`ASV_MONITOR_DIR`) |
| `MONITOR_HZ` | `5.0` | Frekuensi tulis `telemetry.json` |
| `MONITOR_FRAME` | `true` | Tulis frame JPEG |
| `MONITOR_FRAME_HZ` | `10.0` | Frekuensi tulis frame |
| `MONITOR_FRAME_WIDTH` | `640` | Lebar frame (skala turun) |
| `MONITOR_JPEG_QUALITY` | `70` | Kualitas JPEG (0–100) |
| `MONITOR_EVENTS` | `true` | Tulis `events.jsonl` |

Cek cepat dari terminal: `./.venv/bin/python3 scripts/monitor_read.py --watch`.

---

## Urutan Bring-up di Lapangan

1. **Daya & kamera**: nyalakan NUC + kamera, cek status di GUI (chip kamera).
2. **Pixhawk**: colok USB ke NUC. Pastikan `SERIAL_PORT` benar (`/dev/ttyACM0`
   di Pi, `/dev/ttyACM1` di NUC). Buka GUI, pilih port dari combobox.
3. **Tunggu chip telemetri LOCK** (attitude + GPS 3D fix). Chip baterai harus
   menunjukkan nilai nyata (bukan abu = no data).
4. **Cek parameter failsafe Pixhawk** — log startup `[PIXHAWK-CHECK]` menandai
   yang KRITIS. Betulkan di QGC sebelum arm.
5. **QGC (opsional)**: hubungkan QGC ke port MAVLink NUC untuk monitor +
   Arm/Mode/E-stop dari laptop. Otonomi tetap di NUC.
6. **Arm** (dari QGC atau RC). State `WAITING_GPS` → misi mulai otomatis saat
   fix GPS + attitude tersedia.
7. **Kill switch**: tombol KILL GUI = thrust 0 + DISARM. E-stop hardware
   memotong driver lewat STM32 (independen NUC/Pixhawk).

> Kapal akan mulai bergerak setelah GPS fix. Pastikan area air aman & RC/QGC
> siap takeover sebelum arm.

---

## Troubleshooting Cepat

| Gejala | Cek |
|---|---|
| `C_AVAILABLE=False` | Library C belum/gagal di-build → `make -C core linux` |
| Chip kamera "BELUM ADA" | Kamera tak terdeteksi / index salah; coba Scan kamera di GUI |
| Chip telemetri NO-LINK | Port salah, Pixhawk belum siap, atau kabel; cek `SERIAL_PORT` |
| Telemetri `NO-LINK BUSY(QGC?)` | Port serial dipegang program lain (QGC/mavlink-router). `lsof /dev/ttyACM*` → tutup QGC, app retry ≤5 s. Atau pakai `GCS_FORWARD=1` agar QGC lewat UDP. |
| Telemetri `NO-LINK NO-PORT` | Pixhawk tak terdeteksi OS. Cek `lsusb`; Linux: user harus di grup `dialout` |
| Kapal tak mulai bergerak | Tunggu GPS 3D fix + attitude (`WAITING_GPS`); cek baterai nyata |
| Deteksi buoy palsu (wajah) | Naikkan `BUOY_CONF_THRESHOLD` / `DETECTION_DEBUG=True` |
| Deteksi tak konsisten | Naikkan `DETECTION_CONFIRM_DURATION_S`, turunkan `YOLO_INFERENCE_SIZE` |
| FPS kamera rendah | Cek `CAMERA_TARGET_FPS`; naikkan `SECONDARY_PREVIEW_INTERVAL` |
| RTL palsu saat akselerasi | Naikkan `FAILSAFE_LOW_BATT_HOLD_S` |
| Log `[PIXHAWK-CHECK] KRITIS` | Aktifkan param failsafe yang disarankan di QGC (nama beda per firmware: PX4 `NAV_DLL_ACT` dll, ArduPilot `FS_*`) |
| Cek param `TIDAK DIJAWAB` semua | Firmware vs spec tak cocok, atau port dipakai QGC. Tutup QGC lalu ulangi |

---

## Verifikasi

```bash
# Cek .venv (import inti + inferensi YOLO ONNX)
./.venv/bin/python3 scripts/verify_env.py

# GUI headless (uji tanpa display)
QT_QPA_PLATFORM=offscreen timeout 10 ./.venv/bin/python3 main.py
```

---

## Hardware Ringkas

| Komponen | Peran |
|---|---|
| **NUC / Raspberry Pi** | Komputasi utama (GUI + YOLO + ctypes ke C core) |
| **Pixhawk** | Autopilot: GPS + IMU + kompas, PWM utama ke ESC |
| **STM32F103** | ECU Supervisor: arus/suhu/tegangan, E-stop hardware, watchdog |
| **ESC 100A ×2 + TC4420/IRF3205** | Differential drive thruster kiri/kanan |
| **LiPO 4S (motor) + LiFePO4 4S (PC)** | Daya terpisah |
| **Kamera USB** | Visi utama (default 1280×720 MJPG @ 30 fps) |

- **E-stop hardware** → STM32 → TC4420 kill line (hardware cut, independen
  NUC/Pixhawk).
- **STM32 supervisor**: ACS758 (arus), NTC B3950 (suhu ESC), BME280 (ambient),
  divider (tegangan) → frame 8 byte tiap 100 ms ke NUC via UART.
- **Watchdog**: heartbeat NUC putus > 500 ms ATAU E-stop → STM32 potong driver.
- **Baterai (GUI)**: tegangan pack dari SYS_STATUS/BATTERY_STATUS; persen
  linear dari tegangan bila firmware tak kirim `battery_remaining`. Chip: hijau
  ≥50%, kuning 20–50%, merah <20%, abu = no data (LiPO 4S full 16.8 V, empty
  12.8 V).

---

## Lisensi

MIT — gunakan, modifikasi, distribusikan bebas.
