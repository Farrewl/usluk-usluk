# Runbook — Menjalankan & Operasional ASV

Panduan praktis dari nol sampai siap di air. Untuk arsitektur lihat
`docs/ARSITEKTUR.md`; arti parameter lihat `docs/KONFIG.md`.

---

## 1. Setup (sekali per mesin)

### Linux / Raspberry Pi

```bash
./scripts/setup_minimal.sh          # bikin .venv minimal (torch CPU) + verifikasi
```

### Windows (PowerShell, dari root repo)

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
```

Hasil: `.venv` ±1,3 GB dengan GUI PyQt, YOLO 4 model, MAVLink. Rincian di
`requirements.txt`.

**Idempoten — aman dijalankan ulang.** Script memakai ulang `.venv` yang sudah
ada (tidak menimpa) dan melewati instalasi bila paket inti sudah bisa
di-import. Jadi menjalankan lagi hanya butuh beberapa detik. Untuk paksa
bersih, hapus dulu: `rm -rf .venv` (Linux) / `Remove-Item -Recurse -Force .venv`
(Windows). Verifikasi akhir memakai `scripts/verify_env.py` (satu sumber Linux
+ Windows).

### Library C (wajib untuk navigator)

```bash
make -C core linux                  # -> core/libaterkia.so
# Windows (MinGW): mingw32-make -C core windows  -> core/aterkia_core.dll
```

Tanpa library C, `aterkia_core.C_AVAILABLE=False` dan navigator gagal
eksplisit (bukan angka palsu).

### Model YOLO (tidak ada di git)

```bash
WEIGHTS_SOURCE=/path/ke/weights ./scripts/download_weights.sh     # Linux
# Windows: .\scripts\download_weights.ps1  (set $env:WEIGHTS_SOURCE dulu)
```

Butuh 4 model: `buoy.pt`, `box_hijau.pt`, `best_blue_dark.pt`, `best_red_new.pt`.

### Waypoint

Isi `config/plan.csv` (format `lat,lon`, baris pertama header). Satu-satunya
file plan.

---

## 2. Menjalankan GUI

```bash
./scripts/run_gui.sh          # Linux (pakai -O + .venv)
# Windows: scripts\run.bat
# Manual:  ./.venv/bin/python3 -O main.py
```

### Pilih backend (env `ASV_BACKEND`)

| Nilai | Backend | Kapan dipakai |
|---|---|---|
| `nav` (default) | `app.navigator.NavigatorThread` | Kapal produksi (Pixhawk + MAVLink) |
| `sim` / `simulator` / `ground` | `app.simulator.GroundSimNavigatorThread` | Uji darat (kamera+YOLO asli, posisi mock) |

```bash
ASV_BACKEND=sim ./scripts/run_gui.sh     # mode simulator darat
```

---

## 3. Urutan Bring-up di Lapangan

1. **Daya & kamera**: nyalakan NUC + kamera, cek kamera terdeteksi
   (`./scripts/test_deteksi.py --debug` di Linux).
2. **Pixhawk**: colok USB ke NUC. Pastikan `SERIAL_PORT` benar
   (`/dev/ttyACM0` di Pi, `COMx` di Windows). Buka GUI, pilih port dari combobox.
3. **Tunggu chip telemetri LOCK** (attitude + GPS 3D fix). Chip baterai harus
   menunjukkan nilai nyata (bukan abu = no data).
4. **Cek parameter failsafe Pixhawk** — log startup `[PIXHAWK-CHECK]` menandai
   yang KRITIS. Betulkan di QGC/Mission Planner sebelum arm.
5. **QGC (opsional)**: `GCS_IP=192.168.1.50 ./scripts/qgc_forward.sh` untuk
   monitor + Arm/Mode/E-stop dari laptop. Otonomi tetap di NUC.
6. **Arm** (dari QGC atau RC). State `WAITING_GPS` → misi mulai otomatis saat
   fix GPS + attitude tersedia.
7. **Kill switch**: tombol KILL GUI = thrust 0 + DISARM. E-stop hardware
   memotong driver lewat STM32 (independen NUC/Pixhawk).

> Kapal akan mulai bergerak setelah GPS fix. Pastikan area air aman & RC/QGC
> siap takeover sebelum arm.

---

## 4. Kendali Manual (takeover)

- **Gamepad/RC lokal**: aktif selama channel deadman (`RC_CH_DEADMAN`) > 1500.
- **QGC (Xbox)**: lewat `MANUAL_CONTROL` MAVLink; paket basi >
  `QGC_MANUAL_TIMEOUT_S` → netral.
- **Konflik** (lokal + QGC aktif bersamaan) → **netral + warning** (arbitrase C
  `arb_manual2`). Tidak ada rebutan diam-diam.
- Saat MANUAL/KILL, aksi misi ditahan; deteksi baru tidak diminta.

---

## 5. Output & Foto

- Output thruster: `SET_ATTITUDE_TARGET` (offboard) ke Pixhawk.
- Foto misi disimpan **lokal**:
  - kotak hijau: `data/captures/`
  - kotak biru (bawah air) & waypoint: `data/waypoint_captures/`
- Tidak ada upload ke server.

---

## 6. Alat Kalibrasi & Diagnosa

| Alat | Fungsi |
|---|---|
| `scripts/test_deteksi.py --flip 0-3 --debug --imgsz 640` | Live kamera + deteksi + alasan tolak |
| `scripts/calib_hsv.py` | Kalibrasi ambang HSV warna |
| `settings_dialog.py` (GUI) | Ubah parameter saat runtime |

---

## 7. Verifikasi & Test

```bash
# Cross-check C vs Python (butuh gcc; tanpa gcc otomatis skip)
./.venv/bin/python3 -m unittest discover -s tests -p 'test_core_*.py'

# GUI headless (uji cepat tanpa display)
QT_QPA_PLATFORM=offscreen timeout 10 ./.venv/bin/python3 main.py
```

---

## 8. Troubleshooting Cepat

| Gejala | Cek |
|---|---|
| `C_AVAILABLE=False` | Library C belum/ gagal di-build → `make -C core linux` |
| Chip kamera "BELUM ADA" | Kamera tak terdeteksi / index salah; coba Scan kamera di GUI |
| Chip telemetri NO-LINK | Port salah, Pixhawk belum siap, atau kabel; cek `SERIAL_PORT` |
| Kapal tak mulai bergerak | Tunggu GPS 3D fix + attitude (`WAITING_GPS`); cek baterai nyata |
| Deteksi buoy palsu (wajah) | Naikkan `BUOY_CONF_THRESHOLD` / `DETECTION_DEBUG=True` |
| Deteksi tak konsisten | Naikkan `DETECTION_CONFIRM_DURATION_S`, turunkan `YOLO_INFERENCE_SIZE` |
| FPS kamera rendah | Cek `CAMERA_TARGET_FPS`; naikkan `SECONDARY_PREVIEW_INTERVAL` |
| RTL palsu saat akselerasi | Naikkan `FAILSAFE_LOW_BATT_HOLD_S` |
| Log `[PIXHAWK-CHECK] KRITIS` | Aktifkan param failsafe yang disarankan di QGC |
