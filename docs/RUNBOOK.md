# Runbook — Menjalankan & Operasional ASV

Panduan praktis dari nol sampai siap di air. Untuk arsitektur lihat
`docs/ARSITEKTUR.md`; arti parameter lihat `docs/KONFIG.md`.

---

## 1. Setup (sekali per mesin)

### Linux / Raspberry Pi

```bash
./scripts/setup_minimal.sh          # bikin .venv ringan (tanpa torch) + verifikasi
```

### Windows (PowerShell, dari root repo)

```powershell
powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
```

Hasil: `.venv` ±540 MB (tanpa torch — hemat >2×) dengan GUI PyQt, YOLO 4
model (ONNX), MAVLink. Inti C (`core/libaterkia.so`) ikut di-build oleh
script. Rincian di `requirements.txt`.

**Idempoten — aman dijalankan ulang.** Script memakai ulang `.venv` yang sudah
ada (tidak menimpa) dan melewati instalasi bila paket inti sudah bisa
di-import. Jadi menjalankan lagi hanya butuh beberapa detik. Untuk paksa
bersih, hapus dulu: `rm -rf .venv` (Linux) / `Remove-Item -Recurse -Force .venv`
(Windows). Verifikasi akhir memakai `scripts/verify_env.py` (satu sumber Linux
+ Windows).

### Library C (wajib untuk navigator)

Dibangun otomatis oleh `setup_minimal.sh` (idempoten — dilewati bila ada).
Manual:

```bash
make -C core linux                  # -> core/libaterkia.so
# Windows (MinGW): mingw32-make -C core windows  -> core/aterkia_core.dll
```

Tanpa library C, `aterkia_core.C_AVAILABLE=False` dan navigator gagal
eksplisit (bukan angka palsu). Di Windows DLL `core/aterkia_core.dll`
sudah ada di repo (self-contained — tak butuh MinGW untuk run); rebuild
HANYA kalau ada perubahan C: `mingw32-make -C core windows`.

### Model YOLO (ikut di repo)

4 model sudah ada di `weights/` — runtime pakai `*.onnx` (`buoy.onnx`,
`box_hijau.onnx`, `best_blue_dark.onnx`, `best_red_new.onnx`), sumber
training `*.pt` ikut pula. **Tak perlu unduh**.

### Waypoint & lintasan

Dua file lintasan: `config/trackA.json` (start dock merah) dan
`config/trackB.json` (start dock hijau). Pilih lintasan di `config/site.json`
(`"TRACK": "A"`), dropdown GUI tab Misi, atau env `ASV_TRACK=A|B`.
`config/plan.csv` tinggal cadangan.

---

## 2. Menjalankan GUI

```bash
./scripts/run_gui.sh          # Linux (pakai -O + .venv)
# Windows: scripts\run.bat
# Manual:  ./.venv/bin/python3 -O main.py
```

### Backend

Selalu **navigator produksi** (`app.navigator.NavigatorThread`, Pixhawk +
MAVLink). Tidak ada mode mock/simulator — data selalu dari kamera & Pixhawk
nyata.

---

## 3. Urutan Bring-up di Lapangan

1. **Daya & kamera**: nyalakan NUC + kamera, cek status di GUI (chip kamera).
2. **Pixhawk**: colok USB ke NUC. Pastikan `SERIAL_PORT` benar
   (`/dev/ttyACM0` di Pi, `COMx` di Windows). Buka GUI, pilih port dari combobox.
3. **Tunggu chip telemetri LOCK** (attitude + GPS 3D fix). Chip baterai harus
   menunjukkan nilai nyata (bukan abu = no data).
4. **Cek parameter failsafe Pixhawk** — log startup `[PIXHAWK-CHECK]` menandai
   yang KRITIS. Betulkan di QGC/Mission Planner sebelum arm.
5. **QGC (opsional)**: hubungkan QGC ke port MAVLink NUC untuk monitor +
   Arm/Mode/E-stop dari laptop. Otonomi tetap di NUC.
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
- Tidak ada upload ke server. Geo-tag tiap foto (Day/Date/Time, koordinat
  DD & DM, SOG, COG) ditulis sebagai sidecar `.json` di samping gambar dan
  masuk ke `data/monitor/photos.json` (dibaca dashboard web).

### Dashboard web (repo terpisah)

Repo ini **tidak berisi web**. App kapal menulis kondisi lengkapnya ke
`data/monitor/` (lihat `docs/MONITOR.md`). Dashboard web adalah **repo
terpisah** di mini-PC yang sama; ia membaca `telemetry.json`, `frame.jpg`,
`photos.json`, `events.jsonl`. Cek cepat dari terminal:
`./.venv/bin/python3 scripts/monitor_read.py --watch`.

---

## 6. Alat Kalibrasi & Diagnosa

| Alat | Fungsi |
|---|---|
| `settings_dialog.py` (GUI) | Ubah parameter saat runtime |
| `DETECTION_DEBUG=True` (tuning) | Cetak alasan buoy ditolak ke log |

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
| Telemetri `NO-LINK BUSY(QGC?)` | Port serial dipegang program lain (QGC/mavlink-router). `lsof /dev/ttyACM*` → tutup QGC, app retry ≤5 s. Atau pakai `GCS_FORWARD=1` agar QGC lewat UDP. |
| Telemetri `NO-LINK NO-PORT` | Pixhawk tak terdeteksi OS. Cek `lsusb`/Device Manager; Linux: user harus di grup `uucp`/`dialout` |
| Kapal tak mulai bergerak | Tunggu GPS 3D fix + attitude (`WAITING_GPS`); cek baterai nyata |
| Deteksi buoy palsu (wajah) | Naikkan `BUOY_CONF_THRESHOLD` / `DETECTION_DEBUG=True` |
| Deteksi tak konsisten | Naikkan `DETECTION_CONFIRM_DURATION_S`, turunkan `YOLO_INFERENCE_SIZE` |
| FPS kamera rendah | Cek `CAMERA_TARGET_FPS`; naikkan `SECONDARY_PREVIEW_INTERVAL` |
| RTL palsu saat akselerasi | Naikkan `FAILSAFE_LOW_BATT_HOLD_S` |
| Log `[PIXHAWK-CHECK] KRITIS` | Aktifkan param failsafe yang disarankan di QGC (nama param beda per firmware: PX4 `NAV_DLL_ACT` dll, ArduPilot `FS_*`) |
| Cek param tampil `TIDAK DIJAWAB` semua | Firmware vs spec tak cocok, atau port dipakai QGC. Lihat folder GUI `Pixhawk:`; tutup QGC lalu ulangi |
