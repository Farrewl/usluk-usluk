# Daftar Modul per Folder

## `app/` — Paket Python utama (GUI, navigasi, visi, telemetri)

| File | Fungsi | Dipakai oleh |
|---|---|---|
| `settings.py` | Semua parameter & path (satu sumber kebenaran) | Semua modul |
| `navigator.py` | Navigator produksi: state machine + MAVLink offboard (Pixhawk) | `main.py` (backend `nav`) |
| `simulator.py` | `GroundSimNavigator` (kamera asli) + `MockSimNavigator` (tanpa hardware) | `main.py` (backend `sim`) |
| `yolo_worker.py` | Worker inferensi YOLO async (thread murni, satu untuk semua model) | navigator, simulator |
| `output_link.py` | Output ke Pixhawk: `SET_ATTITUDE_TARGET`, RTL, disarm | navigator, simulator |
| `safety.py` | Failsafe otomatis (stale-link / low-batt) | navigator, simulator |
| `camera.py` | Buka kamera V4L2/DSHOW, negosiasi resolusi, flip | navigator, simulator |
| `camera_manager.py` | Kelola kamera primer/sekunder, fallback, reconnect | navigator, simulator |
| `mavlink_telemetry.py` | Baca MAVLink non-blocking (attitude, posisi, baterai) | navigator, simulator |
| `manual_link.py` | Kendali manual lokal (RC/gamepad) → (surge, yaw) | navigator, simulator |
| `qgc_offboard.py` | Kendali manual dari QGC (Xbox → `MANUAL_CONTROL`) | navigator, simulator |
| `detection_validation.py` | `validate_buoy()` filter pasca-YOLO + debug | navigator, simulator |
| `pixhawk_check.py` | Cek parameter failsafe Pixhawk saat startup | navigator |
| `settings_dialog.py` | Panel tuning GUI | `main.py` |
| `slim_map.py` | Widget peta ringan (tile) | `main.py` |
| `gui_theme.py` | Tema/style GUI | `main.py` |
| `logutil.py` | Logger + throttle | Semua modul |
| `aterkia_core.py` | **Tipis** ctypes wrapper ke `core/libaterkia.so` / `aterkia_core.dll`, tanpa logika | navigator, simulator |
| `state_bus.py` | Bus status terbaru (thread-safe) navigator → web | navigator, `web/` |
| `geotag.py` | Geo-tag foto (Day/Date/Time, koordinat DD & DM, SOG, COG) | navigator |

## `web/` — Dashboard monitoring (SSE + MJPEG)

| File | Fungsi |
|---|---|
| `server.py` | HTTP server stdlib: `/events` (SSE), `/video` (MJPEG), `/captures`, `/config` |
| `mock.py` | Feeder data palsu saat `ASV_MOCK=1` (demo tanpa kapal) |
| `static/index.html` | Layout dashboard (header, position-log, grid, foto) |
| `static/dashboard.js` | Klien SSE + isi DOM + popup geo-tag |
| `static/grid.js` | Peta grid A1–E5 + trajectory + kompas (canvas) |
| `static/geo.js` | Format koordinat/satuan di sisi browser |

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
| `site.json` | Identitas tim (nama, PT), lintasan aktif, host/port web, grid |
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

## `scripts/` — Helper dev/operasional

| File | Fungsi |
|---|---|
| `setup_minimal.sh` / `.ps1` | Bikin `.venv` minimal (torch CPU) + build inti C + verifikasi (idempoten) |
| `verify_env.py` | Verifikasi `.venv` (import inti + YOLO); dipakai setup Linux & Windows |
| `run_gui.sh` / `run.bat` | Jalankan GUI (Linux / Windows); web dashboard ikut nyala |

## `tests/` — Cross-check C vs Python

- `test_core_*.py` — ctypes cross-check bit-per-bit vs C (butuh `gcc`; tanpa
  gcc otomatis di-skip).
- `_ref/` — oracle Python: `geo.py`, `gate_sequencer.py`, `filtering.py`,
  `fuzzy.py`, `state_machine.py`.

## `weights/` — Model YOLO (IKUT git, langsung pakai setelah clone)

| File | Misi |
|---|---|
| `buoy.pt` | Gate (2 pelampung) |
| `box_hijau.pt` | Foto kotak hijau |
| `best_blue_dark.pt` | Foto kotak biru (gelap) |
| `best_red_new.pt` | Docking kotak merah |

Model lama/percobaan (`box_biru.pt`, `box_merah.pt`, `best_greendock.pt`) tidak dipakai.

## `docs/`

| File | Isi |
|---|---|
| `ARSITEKTUR.md` | Alur data, modul, inti C, kamera, visi, output, failsafe, hardware |
| `MODUL.md` | (Ini) tabel modul per folder |
| `KONFIG.md` | Arti tiap parameter di `tuning_params.json` |
| `STATES.md` | 18 state misi + transisi (kenapa kapal bertingkah begitu) |
| `RUNBOOK.md` | Cara menjalankan produksi + checklist lapangan |
