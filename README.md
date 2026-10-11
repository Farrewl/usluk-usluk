# Aterkia ASV — Kapal Robot Otonom

Backend navigasi & GUI untuk **Autonomous Surface Vehicle** kompetisi Roboboat.
Kapal menyusuri waypoint, melewati **gate** (2 pelampung), memotret **kotak**,
dan **docking** ke kotak merah — berbasis visi (YOLO) + telemetri MAVLink.

```
Kamera ─▶ YOLO (async) ─▶ gate vision + PID/complementary/EKF (C core)
                              │
                              ▼
     Thruster mixer (differential) ─▶ MAVLink OFFBOARD ─▶ Pixhawk → ESC → Thruster
```

Pixhawk tetap pemilik ESC/mixer; companion (NUC/Pi) hanya mengirim setpoint
offboard.

---

## Cepat Mulai

Model YOLO & library inti **ikut di build**: cukup 2 langkah.

```bash
# 1. Setup .venv minimal (torch CPU) + build inti C + verifikasi
./scripts/setup_minimal.sh

# 2. Jalankan GUI (web dashboard otomatis nyala di :8080)
./scripts/run_gui.sh
```

**Windows** (PowerShell, dari root repo):

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
scripts\run.bat
```

Sudah pernah setup? Langsung `./scripts/run_gui.sh`. Setup idempoten:
`.venv` dan library yang sudah ada **dipakai ulang, tak ditimpa**; install
dilewati bila paket inti sudah bisa di-import.

Lintasan & identitas tim diatur di `config/site.json` (+ `config/trackA.json`,
`config/trackB.json`) — bisa juga ganti dari dropdown GUI / env
`ASV_TRACK=A|B`. Model YOLO sudah ada di `weights/` (tak perlu unduh).

---

## Pilih Backend

Lewat variabel lingkungan `ASV_BACKEND`:

```bash
ASV_BACKEND=nav ./scripts/run_gui.sh    # default — kapal produksi (Pixhawk + MAVLink)
ASV_BACKEND=sim ./scripts/run_gui.sh    # simulator darat (kamera+YOLO asli, posisi mock)
```

| Nilai | Backend |
|---|---|
| `nav` (default) | `app.navigator.NavigatorThread` |
| `sim` / `simulator` / `ground` | `app.simulator.GroundSimNavigatorThread` |

---

## Inti C (`core/`) — wajib di-build

Komputasi deterministik & kritis-waktu ada di C, diverifikasi **bit-per-bit**
vs oracle Python (`tests/test_core_*.py`).

```bash
make -C core linux                 # Linux / Pi  -> core/libaterkia.so
make -C core check                 # -Wall -Wextra -Werror
# Windows (MinGW): mingw32-make -C core windows -> core/aterkia_core.dll
```

Modul: `nav_math`, `gate_vision`, `fuzzy`, `state_machine` (18 state),
`control_filters`, `thruster_mixer`, `mavlink_bridge`, `ecu_link`,
`arbitrator`, `manual_control`, `rc_decode`, `mode_manager`, `avoidance`,
`gamepad_input`.

---

## Dukungan Lintas OS

| Komponen | Linux | Windows | Catatan |
|---|---|---|---|
| GUI PyQt (`main.py`) | ✅ | ✅ | sama persis |
| YOLO 4 model (.pt) | ✅ | ✅ | torch CPU (wheel index PyTorch) |
| Setup `.venv` | ✅ `setup_minimal.sh` | ✅ `setup_minimal.ps1` | idempoten, skip yang sudah ada |
| Model YOLO | ✅ ikut repo | ✅ ikut repo | ter-commit, tanpa unduh |
| Dashboard web (SSE+MJPEG) | ✅ | ✅ | buka `http://<ip>:8080` |
| Telemetri Pixhawk | ✅ `/dev/ttyACM*` | ✅ `COM3–9` | deteksi otomatis |
| Kamera | ✅ UID by-id | ✅ index angka | negosiasi resolusi otomatis |
| Library C (`aterkia`) | ✅ `make -C core linux` | ✅ `make -C core windows` | Windows butuh MinGW; tanpa itu gagal eksplisit |
| Gamepad USB | ✅ `/dev/input/js*` | ⛔ stub | Windows: RC Pixhawk / tombol GUI |
| Cross-check C↔Python | ✅ | ✅ | tanpa `gcc` otomatis skip |

---

## Struktur Repo Ringkas

```
usluk-usluk/
├── main.py                 # ENTRY POINT — GUI (pilih backend via ASV_BACKEND)
├── app/                    # Paket Python
│   ├── navigator.py        # Navigator produksi (Pixhawk + state machine)
│   ├── simulator.py        # Simulator darat (GroundSim + MockSim)
│   ├── yolo_worker.py      # Worker inferensi YOLO async
│   ├── output_link.py      # SET_ATTITUDE_TARGET / RTL / disarm
│   ├── safety.py           # Failsafe (stale-link / low-batt)
│   ├── camera.py / camera_manager.py
│   ├── mavlink_telemetry.py
│   ├── manual_link.py / qgc_offboard.py
│   ├── detection_validation.py
│   ├── settings.py         # Semua parameter (satu sumber)
│   └── aterkia_core.py     # ctypes wrapper ke core/
├── core/                   # Inti C/C++ (deterministik)
│   ├── include/*.h
│   ├── src/*.c             # nav_math, gate_vision, fuzzy, state_machine, ...
│   └── libaterkia.so       # build lokal (di-ignore git)
├── web/                    # Dashboard monitoring (SSE + MJPEG, stdlib)
│   ├── server.py           # HTTP server + endpoint /events /video /captures
│   ├── mock.py             # feeder data palsu (env ASV_MOCK=1)
│   └── static/             # index.html + dashboard.js + grid.js + geo.js
├── config/
│   ├── site.json           # identitas tim, lintasan, host/port web
│   ├── trackA.json / trackB.json  # waypoint + leg misi per lintasan
│   └── tuning_params.json  # Parameter tuning (simpan otomatis)
├── weights/                # Model YOLO (*.pt) — IKUT git
├── data/                   # Output runtime (foto) — di-ignore git
├── scripts/                # setup_minimal.sh/.ps1, run_gui.sh, run.bat, verify_env.py
├── docs/                   # Dokumentasi (lihat bawah)
└── tests/                  # Cross-check C↔Python (test_core_*.py + _ref/)
```

---

## Dashboard Web (monitoring KKI)

Saat GUI jalan, server dashboard otomatis nyala (`config/site.json`:
`WEB_HOST`/`WEB_PORT`, default `0.0.0.0:8080`). Panitia/peserta buka
`http://<ip-laptop>:8080` di jaringan router tim.

- `/events` — Server-Sent Events, snapshot telemetri @ `SSE_HZ` (default 5 Hz)
- `/video` — MJPEG dari kamera NAV @ `VIDEO_HZ` (default 10 Hz)
- `/captures` — daftar foto misi + geo-tag (sidecar JSON)
- `/config` — identitas tim, lintasan, geometri grid & waypoint

Isi: header tim/PT + pemilih lintasan A/B + baterai + video; Position-Log
misi 1–6 (Preparation, Start, Floating Ball Set 0–10, Surface Imaging,
Underwater Imaging, Finish); SOG (knot & km/h), COG, attitude; grid
A1–E5 25×25 m + trajectory realtime + kompas; popup foto geo-tag (Day,
Date `DD/MM/YYYY`, Time, Coordinate DD & DM, SOG, COG).

Demo tanpa kapal: `ASV_MOCK=1 ./scripts/run_gui.sh` (data + frame sintetis).

---

## Test & Verifikasi

```bash
# Cross-check C vs Python (bit-per-bit; butuh gcc)
./.venv/bin/python3 -m unittest discover -s tests -p 'test_core_*.py'

# GUI headless (uji tanpa display)
QT_QPA_PLATFORM=offscreen timeout 10 ./.venv/bin/python3 main.py
```

> `tests/` sengaja hanya berisi cross-check C↔Python + oracle (`tests/_ref/`).
> Perilaku misi end-to-end diverifikasi lewat smoke test (lihat `docs/RUNBOOK.md`).

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

---

## Dokumentasi Lengkap

| File | Isi |
|---|---|
| `docs/ARSITEKTUR.md` | Alur data, modul, inti C, kamera, visi, output, failsafe, hardware |
| `docs/MODUL.md` | Tabel modul per folder |
| `docs/KONFIG.md` | Arti tiap parameter di `tuning_params.json` |
| `docs/STATES.md` | 18 state misi + transisi |
| `docs/RUNBOOK.md` | Cara menjalankan produksi + checklist lapangan |

---

## Lisensi

MIT — gunakan, modifikasi, distribusikan bebas.
