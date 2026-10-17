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

# 2. Jalankan GUI (menulis ekspor monitoring ke data/monitor/)
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

> **Catatan: JANGAN `source .venv/bin/activate`, pakai script wrapper.**
> Script aktivasi python `venv` menyimpan **path absolut** dan rusak bila
> repo **dipindah/di-rename**, apalagi kalau pathnya mengandung **spasi**
> (mis. `asv usluk-usluk`) — `source` diam-diam gagal, `python` kembali ke
> sistem. Solusi: selalu jalankan lewat `./scripts/run_gui.sh`,
> `scripts\run.bat`, atau langsung `./.venv/bin/python3 -O main.py` —
> ketiganya kebal path & shell. Python versi venv sendiri tetap bisa dipakai
> walau aktivasi rusak, karena `bin/python3` relatif terhadap `pyvenv.cfg`.

Lintasan & identitas tim diatur di `config/site.json` (+ `config/trackA.json`,
`config/trackB.json`) — bisa juga ganti dari dropdown GUI / env
`ASV_TRACK=A|B`. Model YOLO sudah ada di `weights/` (tak perlu unduh).

---

## Cara Menjalankan (mini-PC + laptop)

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
> QGroundControl (atau mavlink-router) sudah membuka `/dev/ttyACM*` / `COM*`,
> app ASV tak bisa ikut konek — status jadi
> `Pixhawk: /dev/ttyACM1 (NO-LINK BUSY(QGC?) | ...)`. **Tutup QGC dulu**
> (app akan retry otomatis dalam ≤5 s), lalu pakai pola *GCS forward* di atas
> agar QGC tetap dapat telemetri lewat UDP. Cek pemegang port:
> `lsof /dev/ttyACM*` (Linux) — kalau muncul `QGroundControl`, itulah biangnya.

### Percepat startup & deteksi

- **Fast-start kamera ON default** — kamera utama dibuka sekali (mode cache /
  1280x720), tanpa sweep resolusi berulang (di Windows bisa 5–25 s). Negosiasi
  penuh tetap bisa dipicu manual via tombol **Scan** (tab Misi). Matikan:
  `ASV_CAM_FAST=0`.
- **Deteksi kontinu ON default** — model GATE ikut dijalankan saat pra-misi
  (WAITING_GPS dan waypoint-nav) dan frame ber-kotak deteksi tampil di GUI
  **dan** web MJPEG. Matikan: `ASV_DETECT=0`. Model YOLO dimuat di thread
  (lazy); kotak muncul begitu model siap (~10 s pertama).
- **Kotak = semua deteksi mentah + confidence** (label `kls 0.87`) — bukan
  hanya yang lulus validasi warna. Validasi HSV tetap dipakai untuk keputusan
  navigasi, tampilan selalu jujur apa adanya.
- **Kamera auto-cari** — kalau index yang dikonfigurasi gagal dibuka/restart,
  app otomatis scan index lain sekali dan memakainya (log `pakai index N`).
  Tidak perlu edit config di lapangan.

---

## Inti C (`core/`) — wajib di-build

Komputasi deterministik & kritis-waktu ada di C, diverifikasi **bit-per-bit**
vs oracle Python (`tests/test_core_*.py`).

```bash
make -C core linux                 # Linux / Pi  -> core/libaterkia.so
make -C core check                 # -Wall -Wextra -Werror
# Windows: core/aterkia_core.dll SUDAH di-repo (self-contained, UCRT
# bawaan Win10/11). Rebuild HANYA kalau ada perubahan C:
#   mingw32-make -C core windows   (MinGW native)
#   make -C core cross             (cross dari Linux, pakai mingw-w64)
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
| Ekspor monitoring (file bridge) | ✅ | ✅ | tulis `data/monitor/` untuk dashboard web terpisah |
| Telemetri Pixhawk | ✅ `/dev/ttyACM*` | ✅ `COM3–9` | deteksi otomatis |
| Kamera | ✅ UID by-id | ✅ index angka | negosiasi resolusi otomatis |
| Library C (`aterkia`) | ✅ `make -C core linux` | ✅ DLL di-repo | Windows: `core/aterkia_core.dll` self-contained sudah di-commit; rebuild opsional (MinGW) |
| Gamepad USB | ✅ `/dev/input/js*` | ⛔ stub | Windows: RC Pixhawk / tombol GUI |
| Cross-check C↔Python | ✅ | ✅ | tanpa `gcc` otomatis skip |

---

## Struktur Repo Ringkas

```
usluk-usluk/
├── main.py                 # ENTRY POINT — GUI (kapal produksi)
├── app/                    # Paket Python
│   ├── navigator.py        # Navigator produksi (Pixhawk + state machine)
│   ├── yolo_worker.py      # Worker inferensi YOLO async
│   ├── output_link.py      # SET_ATTITUDE_TARGET / RTL / disarm
│   ├── safety.py           # Failsafe (stale-link / low-batt)
│   ├── camera.py / camera_manager.py
│   ├── mavlink_telemetry.py
│   ├── manual_link.py / qgc_offboard.py
│   ├── detection_validation.py
│   ├── monitor_export.py   # Ekspor file bridge -> data/monitor/ (web terpisah)
│   ├── telemetry_snapshot.py  # Rakit snapshot JSON lengkap
│   ├── settings.py         # Semua parameter (satu sumber)
│   └── aterkia_core.py     # ctypes wrapper ke core/
├── core/                   # Inti C/C++ (deterministik)
│   ├── include/*.h
│   ├── src/*.c             # nav_math, gate_vision, fuzzy, state_machine, ...
│   └── libaterkia.so       # build lokal (di-ignore git)
├── config/
│   ├── site.json           # identitas tim, lintasan, pengaturan monitor
│   ├── trackA.json / trackB.json  # waypoint + leg misi per lintasan
│   └── tuning_params.json  # Parameter tuning (simpan otomatis)
├── weights/                # Model YOLO (*.pt) — IKUT git
├── data/                   # Output runtime (foto + data/monitor) — di-ignore git
├── scripts/                # setup_minimal.sh/.ps1, run_gui.sh, run.bat, verify_env.py
├── docs/                   # Dokumentasi (lihat bawah)
└── tests/                  # Cross-check C↔Python (test_core_*.py + _ref/)
```

---

## Ekspor Monitoring (file bridge)

Repo ini **tidak lagi berisi dashboard web**. App kapal hanya **menulis**
kondisi lengkapnya ke `data/monitor/`, yang dibaca oleh **repo web terpisah**
(jalan di mini-PC yang sama):

| File | Isi | Frekuensi |
|---|---|---|
| `telemetry.json` | snapshot LENGKAP (identitas, pose, gerak, baterai, link, misi, visi, kamera, `params` semua parameter, health) | `MONITOR_HZ` (default 5) |
| `frame.jpg` | frame kamera NAV ter-annotasi YOLO | `MONITOR_FRAME_HZ` (default 10) |
| `frame_bawah.jpg` | kamera bawah (bila ada) | idem |
| `photos.json` | indeks foto misi + geo-tag | tiap foto |
| `events.jsonl` | log kejadian (append) | saat terjadi |

Semua ditulis **atomik** (`*.tmp` → `os.replace`) sehingga pembaca tak pernah
melihat file separuh. Pengaturan di `config/site.json`: `MONITOR_ENABLED`,
`MONITOR_DIR`, `MONITOR_HZ`, `MONITOR_FRAME`, `MONITOR_FRAME_HZ`,
`MONITOR_FRAME_WIDTH`, `MONITOR_JPEG_QUALITY`, `MONITOR_EVENTS`.

Skema lengkap JSON ada di **`docs/MONITOR.md`** — itu kontrak untuk repo web.
Foto mentah tetap di `data/captures/` + `data/waypoint_captures/`.

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
