# Aterkia ASV — Kapal Robot Otonom

Backend navigasi & dashboard untuk **Autonomous Surface Vehicle** kompetisi Roboboat.  
Sistem membawa kapal menyusuri waypoint, melewati **gate** (2 pelampung), memotret **kotak**, dan **docking** ke kotak merah — berbasis visi (YOLO) + telemetri MAVLink.

```
Kamera ─▶ YOLO ─▶ GateSequencer (midpoint) ─▶ PID+deadband + Complementary + EKF (C core)
                │
                ▼
     Thruster mixer (differential) ─▶ MAVLink OFFBOARD ─▶ Pixhawk → ESC → Thruster kiri/kanan
```

---

## Cepat Mulai

```bash
# 1. Unduh model YOLO ke weights/ (~40 MB)
./scripts/download_weights.sh

# 2. Isi waypoint di config/plan.csv (format: lat,lon per baris)
#    Contoh: -7.282356,112.794925

# 3. Jalankan GUI (Linux / Raspberry Pi)
./scripts/run_gui.sh
#    Atau manual:
python3 main.py

# 4. Test kamera + deteksi langsung
./scripts/test_deteksi.py --flip 0 --debug --imgsz 640
```

**Windows** (PowerShell, dari root repo):

```powershell
# 1. Bikin .venv minimal + torch CPU + verifikasi inferensi (1 perintah)
powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1

# 2. Ambil model YOLO
$env:WEIGHTS_SOURCE = "\\server\bersama\weights"   # folder atau .tar.gz
.\scripts\download_weights.ps1

# 3. Isi waypoint (config/plan.csv) lalu jalankan GUI
.venv\Scripts\python.exe main.py
```

---

## Dukungan Lintas OS

Kode dirancang jalan di **Linux (x86_64/ARM Raspberry Pi) dan Windows 10/11**:

| Komponen | Linux | Windows | Catatan |
|---|---|---|---|
| GUI PyQt (`main.py`) | ✅ | ✅ | sama persis |
| YOLO 4 model (.pt) | ✅ | ✅ | torch CPU, wheel tersedia di index PyTorch |
| Setup `.venv` | ✅ `setup_minimal.sh` | ✅ `setup_minimal.ps1` | kunci penghematan identik |
| Model YOLO | ✅ `download_weights.sh` | ✅ `download_weights.ps1` | butuh `tar` (bawaan Win 10+) |
| Telemetri Pixhawk | ✅ `/dev/ttyACM*` | ✅ `COM3–9` | deteksi otomatis di `app/mavlink_telemetry.py` |
| Kamera | ✅ UID by-id (`/dev/serial/by-id`) | ✅ index angka | plug-and-play tetap, label Windows = "Kamera N" |
| Library C (`aterkia`) | ✅ `make -C core linux` | ✅ `make -C core windows` (MinGW) | tanpa build: `C_AVAILABLE=False`, gagal eksplisit |
| Gamepad USB | ✅ `/dev/input/js*` | ⛔ stub "tidak ada" | Windows: RC Pixhawk / tombol GUI |
| Unit test (194) | ✅ | ✅ | tanpa `gcc` → tes C otomatis skip |

---

## Pilih Backend (main.py baris 20-21)

```python
from app.simulator import NavigatorThread   # simulator darat (kamera asli, tanpa Pixhawk)
# from app.navigator import NavigatorThread  # kapal asli (Pixhawk + MAVLink)
```

---

## Struktur Repo Ringkas

```
aterkia-asv/
├── main.py                 # ENTRY POINT — jalankan GUI
├── app/                    # Paket Python (GUI + logika navigasi)
│   ├── navigator.py        # Navigator asli (Pixhawk + state machine)
│   ├── simulator.py        # Simulator darat (GroundSim + MockSim)
│   ├── settings.py         # Semua parameter (satu sumber)
│   ├── geo.py              # Matematika geodetik
│   ├── camera.py           # Buka kamera + flip + negosiasi resolusi
│   ├── mavlink_telemetry.py# Baca MAVLink non-blocking (attitude + baterai)
│   ├── detection_validation.py # Fraksi warna HSV (butuh OpenCV); geometri -> C
│   ├── filtering.py        # Referensi Python PID/Complementary/EKF (fallback C)
│   ├── fuzzy.py            # Referensi uji Fuzzy Sugeno (produksi -> C)
│   ├── gate_sequencer.py   # REFERENSI UJI saja (produksi -> gate_vision.c)
│   ├── aterkia_core.py     # ctypes wrapper tipis ke core/ (libaterkia.so | aterkia_core.dll)
│   └── gateway.py          # Relay Redis ↔ WebSocket (opsional)
├── core/                   # Inti C/C++ (komputasi deterministik)
│   ├── include/*.h         # Header publik
│   ├── src/*.c             # Implementasi C murni
│   │   ├── nav_math.c      # Haversine, bearing, CTE, normalize
│   │   ├── fuzzy.c         # Sugeno singleton
│   │   ├── state_machine.c # 16 state misi
│   │   ├── gate_vision.c   # Koleksi pasangan gate, pinhole, geometri buoy,
│   │   │                   #   sequencer latch+memori (port gate_sequencer.py)
│   │   ├── mavlink_bridge.c# Framing MAVLink v1 + serial
│   │   ├── control_filters.c # PID+deadband, complementary, EKF heading
│   │   ├── thruster_mixer.c  # Differential drive surge+yaw→PWM
│   │   ├── ecu_link.c      # UART STM32 8-byte frame + checksum
│   │   └── arbitrator.c    # Prioritas KILL > MANUAL > AUTO
│   └── libaterkia.so       # Build lokal (di-ignore git)
├── config/
│   ├── plan.csv            # Waypoint misi (kanonik)
│   └── tuning_params.json  # Parameter tuning (GUI simpan otomatis)
├── weights/                # Model YOLO (*.pt) — di-ignore git, unduh via script
├── data/                   # Output runtime (foto, peta sementara) — di-ignore git
├── docs/
│   ├── ARSITEKTUR.md       # Alur data, modul C, power, kamera, deteksi, sequencer
│   ├── MODUL.md            # Tabel modul per folder
│   ├── KONFIG.md           # Arti tiap parameter tuning
│   └── STATES.md           # 16 state misi + transisi
├── scripts/
│   ├── test_deteksi.py     # Live kamera + deteksi (--flip --debug --imgsz)
│   ├── download_weights.sh # Unduh model YOLO
│   └── run_gui.sh          # Jalankan GUI Linux
└── tests/                  # Unit test (131 OK, 1 skip)
    ├── test_core_*.py      # ctypes cross-check C vs Python (bit-per-bit)
    ├── test_geo.py
    ├── test_filtering.py
    └── test_gate_sequencer.py
```

---

## Fitur Baru (Implementasi Terbaru)

- **Kamera**: `CAMERA_FLIP_MODE` (0-3), target **30 fps stabil**, negosiasi resolusi otomatis.
- **Deteksi Buoy**: Threshold longgar (`conf=0.35`, `color_frac=0.05`, `sat=0.35`, `aspect=0.50`, `area=40px`) + `DETECTION_DEBUG` cetak alasan tolak.
- **Gate Sequencer**: Kumpulkan semua pasang plausibel → latch target → switch cepat lewat midpoint (`GATE_VERTICAL_ALIGN_PX=75px`).
- **Filter C Core**: PID+deadband/anti-windup, Complementary, EKF 1-D heading — bit-per-bit sama Python.
- **Hardware Abstraction**: `thruster_mixer` (differential drive), `ecu_link` (STM32 UART), `arbitrator` (KILL>MANUAL>AUTO).
- **Manual dari QGC**: Xbox di laptop → QGC → MAVLink `MANUAL_CONTROL` (via router 14550/14540) → `app/qgc_offboard.py` konversi ke (v, w); failsafe paket basi > `QGC_MANUAL_TIMEOUT_S` (0.5 dtk) → netral + HOLD. Arbitrasi QGC vs lokal (RC/gamepad) di C (`arb_manual2`): **dua-duanya aktif = konflik → netral + warning**, tidak ada rebutan diam-diam. Catatan: output servo/ESC tetap dari Pixhawk — companion memantau & mengarbitrasi, bukan penggerak PWM.
- **Baterai GUI**: Chip status bar + panel monitoring — hijau ≥50%, kuning 20-50%, merah <20%, abu=no data (LiPO 4S 16.8→12.8 V).

---

## Test & Verifikasi

```bash
# Unit test lengkap (termasuk cross-check C vs Python)
python3 -m unittest discover -s tests -v
# Target: 209 tests, 1 skipped

# GUI headless (CI / server tanpa display)
QT_QPA_PLATFORM=offscreen timeout 10 python main.py

# Rebuild C library (kalau ubah core/src/*.c)
make -C core linux        # -> core/libaterkia.so
make -C core check        # verifikasi ketat -Wall -Wextra -Werror
# (Windows/MinGW: mingw32-make -C core windows -> aterkia_core.dll)
```

---

## Hardware Ringkas

| Komponen | Peran |
|---|---|
| **NUC / Raspberry Pi** | Komputasi utama (Python GUI + YOLO + ctypes ke C core) |
| **Pixhawk** | Autopilot: GPS + IMU + Kompas, PWM utama ke ESC |
| **STM32F103 (Blue Pill)** | ECU Supervisor: arus (ACS758), suhu (NTC+BME280), tegangan, E-stop hardware, watchdog |
| **ESC 100A ×2 + TC4420/IRF3205** | Differential drive thruster kiri/kanan |
| **LiPO 4S (motor)** + **LiFePO4 4S (PC)** | Daya terpisah, XT60, kill switch, LED status |
| **Kamera USB** | Visi utama (1280×720 MJPG ~20 fps) |

---

## Dokumentasi Lengkap

| File | Isi |
|---|---|
| `docs/ARSITEKTUR.md` | Alur data, modul C, power, kamera, deteksi, sequencer |
| `docs/MODUL.md` | Tabel modul per folder (`app/`, `core/`, `config/`, `data/`, `scripts/`, `tests/`, `weights/`) |
| `docs/KONFIG.md` | Arti tiap parameter di `tuning_params.json` (termasuk filter, gate, baterai) |
| `docs/STATES.md` | 16 state misi + transisi (kenapa kapal bertingkah begitu) |

---

## Lisensi

MIT — gunakan, modifikasi, distribusikan bebas.