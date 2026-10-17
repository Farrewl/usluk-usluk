# Daftar Modul per Folder

## `app/` — Paket Python utama (GUI, navigasi, visi, telemetri)

| File | Fungsi | Dipakai oleh |
|---|---|---|
| `settings.py` | Semua parameter & path (satu sumber kebenaran) | Semua modul |
| `navigator.py` | Navigator produksi: state machine + MAVLink offboard (Pixhawk) | `main.py` |
| `yolo_worker.py` | Worker inferensi YOLO async (thread murni, satu untuk semua model) | navigator |
| `yolo_onnx.py` | Inferensi YOLOv8 ONNX via `cv2.dnn` (`OnnxDetector`, tanpa torch); API meniru ultralytics | navigator, yolo_worker |
| `output_link.py` | Output ke Pixhawk: `SET_ATTITUDE_TARGET`, RTL, disarm | navigator |
| `safety.py` | Failsafe otomatis (stale-link / low-batt) | navigator |
| `camera.py` | Buka kamera V4L2/DSHOW, negosiasi resolusi, flip | navigator |
| `camera_manager.py` | Kelola kamera primer/sekunder, fallback, reconnect | navigator |
| `mavlink_telemetry.py` | Baca MAVLink non-blocking (attitude, posisi, baterai) | navigator |
| `manual_link.py` | Kendali manual lokal (RC/gamepad) → (surge, yaw) | navigator |
| `qgc_offboard.py` | Kendali manual dari QGC (Xbox → `MANUAL_CONTROL`) | navigator |
| `detection_validation.py` | `validate_buoy()` filter pasca-YOLO + debug | navigator |
| `pixhawk_check.py` | Cek parameter failsafe Pixhawk saat startup (spec per firmware: PX4/ArduPilot) | navigator |
| `monitor_export.py` | Ekspor **file bridge** → `data/monitor/` (telemetry/frame/photos/events) untuk web terpisah | navigator |
| `telemetry_snapshot.py` | Rakit snapshot JSON lengkap kapal (identitas…health, semua `params`) | `monitor_export` |
| `settings_dialog.py` | Panel tuning GUI | `main.py` |
| `slim_map.py` | Widget peta ringan (tile) | `main.py` |
| `gui_theme.py` | Tema/style GUI | `main.py` |
| `logutil.py` | Logger + throttle | Semua modul |
| `aterkia_core.py` | **Tipis** ctypes wrapper ke `core/libaterkia.so` / `aterkia_core.dll`, tanpa logika | navigator |
| `geotag.py` | Geo-tag foto (Day/Date/Time, koordinat DD & DM, SOG, COG) | navigator |

> Dashboard web **tidak lagi di repo ini** — ia repo terpisah yang membaca
> `data/monitor/` (kontrak: `docs/MONITOR.md`).

## `core/` — Inti C/C++ (komputasi deterministik)

| File | Header | Fungsi |
|---|---|---|
| `src/nav_math.c` | `nav_math.h` | Geodesi murni C |
| `src/gate_vision.c` | `gate_vision.h` | Pasangan gate, pinhole, geometri buoy, sequencer latch |
| `src/fuzzy.c` | `fuzzy.h` | Fuzzy Sugeno singleton |
| `src/state_machine.c` | `state_machine.h` | 18 state misi + sub-mesin mundur |
| `src/control_filters.c` | `control_filters.h` | PID/deadband, complementary, EKF heading |
| `src/thruster_mixer.c` | `thruster_mixer.h` | Differential drive surge+yaw→PWM |
| `src/mavlink_bridge.c` | `mavlink_bridge.h` | Framing MAVLink v1 + serial |
| `src/ecu_link.c` | `ecu_link.h` | UART STM32 8-byte frame + checksum |
| `src/arbitrator.c` | `arbitrator.h` | Prioritas KILL > MANUAL > AUTO |
| `src/manual_control.c` | `manual_control.h` | Skala/limit/expo/rate-limit manual |
| `src/rc_decode.c` | `rc_decode.h` | Dekode channel RC → surge/yaw/mode/deadman |
| `src/mode_manager.c` | `mode_manager.h` | Mode op: AUTO / MANUAL / KILL |
| `src/avoidance.c` | `avoidance.h` | Avoidance sederhana (deflect yaw) |
| `src/gamepad_input.cpp` | `gamepad_input.hpp` | Baca gamepad USB (Linux joystick) |
| `libaterkia.so` / `aterkia_core.dll` | — | Shared library (build lokal, di-ignore git) |

## `config/`

| File | Isi |
|---|---|
| `site.json` | Identitas tim (nama, PT), lintasan aktif, pengaturan ekspor monitor (`MONITOR_*`), grid |
| `trackA.json` / `trackB.json` | Waypoint + leg misi per lintasan (vision/foto box/dock) |
| `tuning_params.json` | Parameter tuning (GUI simpan otomatis; hanya `TUNING_PARAM_KEYS`) |
| `plan.csv` | Waypoint lama (cadangan; kini waypoint dari `track*.json`) |
| `archive/` | Arsip plan lama (tidak dipakai) |

## `data/` — Output runtime (di-ignore git)

- `captures/` — foto kotak hijau
- `waypoint_captures/` — foto waypoint
- `calib/` — gambar kalibrasi HSV
- `tiles/` — tile peta
- `session_videos/` — rekaman sesi (bila diaktifkan)
- `monitor/` — ekspor file bridge untuk dashboard web terpisah (`telemetry.json`, `frame.jpg`, `photos.json`, `events.jsonl`)

## `scripts/` — Helper dev/operasional

| File | Fungsi |
|---|---|
| `setup_minimal.sh` / `.ps1` | Bikin `.venv` ringan (tanpa torch) + build inti C + verifikasi (idempoten) |
| `verify_env.py` | Verifikasi `.venv` (import inti + inferensi YOLO ONNX); dipakai setup Linux & Windows |
| `run_gui.sh` / `run.bat` | Jalankan GUI (Linux / Windows) |
| `monitor_read.py` | Lihat/tail data `data/monitor/` dari terminal (debug / rujukan repo web) |

## `tests/` — Cross-check C vs Python

- `test_core_*.py` — ctypes cross-check bit-per-bit vs C (butuh `gcc`; tanpa
  gcc otomatis di-skip).
- `_ref/` — oracle Python: `geo.py`, `gate_sequencer.py`, `filtering.py`,
  `fuzzy.py`, `state_machine.py`.

## `weights/` — Model YOLO (IKUT git, langsung pakai setelah clone)

| File | Misi |
|---|---|
| `buoy.onnx` | Gate (2 pelampung) |
| `box_hijau.onnx` | Foto kotak hijau |
| `best_blue_dark.onnx` | Foto kotak biru (gelap) |
| `best_red_new.onnx` | Docking kotak merah |

Tiap model punya pasangan `*.pt` (sumber training, dipakai saat ekspor ulang
`.onnx`). Runtime hanya memakai `.onnx` via `cv2.dnn` (tanpa torch). Model
lama/percobaan (`box_biru`, `box_merah`, `best_greendock`) sudah dihapus.

## `docs/`

| File | Isi |
|---|---|
| `ARSITEKTUR.md` | Alur data, modul, inti C, kamera, visi, output, failsafe, hardware |
| `MODUL.md` | (Ini) tabel modul per folder |
| `MONITOR.md` | Kontrak ekspor monitoring (file bridge) untuk repo web terpisah |
| `KONFIG.md` | Arti tiap parameter di `tuning_params.json` |
| `STATES.md` | 18 state misi + transisi (kenapa kapal bertingkah begitu) |
| `RUNBOOK.md` | Cara menjalankan produksi + checklist lapangan |
