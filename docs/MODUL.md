# Daftar Modul per Folder

## `app/` — Paket Python utama (GUI, navigasi, visi, telemetri)

| File | Fungsi | Dipakai oleh |
|------|--------|--------------|
| `settings.py` | Semua parameter & path (satu sumber kebenaran) | Semua modul |
| `geo.py` | Matematika geodetik (haversine, bearing, CTE, normalize) | navigator, simulator, tests |
| `camera.py` | Buka kamera V4L2/DSHOW, negosiasi resolusi, `flip_frame_if_needed` | simulator, navigator, test_deteksi |
| `mavlink_telemetry.py` | Baca MAVLink non-blocking (attitude, posisi, baterai) | simulator, navigator |
| `navigator.py` | Navigator asli: state machine + MAVLink offboard (Pixhawk) | `main.py` (produksi) |
| `simulator.py` | 2 simulator: GroundSim (kamera asli) & MockSim (tanpa kamera) | `main.py` (dev) |
| `gateway.py` | Relay Redis ↔ WebSocket dashboard web | opsional |
| `detection_validation.py` | `validate_buoy()` filter pasca-YOLO + debug | navigator |
| `filtering.py` | Referensi Python: PID, complementary, EKF (fallback C) | tests, navigator (via ctypes) |
| `fuzzy.py` | Fuzzy Sugeno singleton gate & docking gain | navigator |
| `gate_sequencer.py` | `GateSequencer` + `collect_gate_pairs` (pure, testable) | navigator |
| `state_machine.py` | Referensi Python state machine (fallback C) | tests |
| `aterkia_core.py` | **Tipis** ctypes wrapper ke `core/libaterkia.so` (tanpa logika) | navigator (opsional) |
| `scripts/` | (hapus — tool lama diganti `scripts/test_deteksi.py`) | — |

## `core/` — Inti C/C++ (komputasi deterministik)

| File | Header | Fungsi |
|------|--------|--------|
| `src/nav_math.c` | `nav_math.h` | Geodetik murni C |
| `src/fuzzy.c` | `fuzzy.h` | Fuzzy Sugeno C |
| `src/state_machine.c` | `state_machine.h` | 16 state misi C |
| `src/mavlink_bridge.c` | `mavlink_bridge.h` | Framing MAVLink v1 + serial C |
| `src/control_filters.c` | `control_filters.h` | PID/deadband, complementary, EKF |
| `src/thruster_mixer.c` | `thruster_mixer.h` | Differential drive surge+yaw→PWM |
| `src/ecu_link.c` | `ecu_link.h` | UART STM32 8-byte frame + checksum |
| `src/arbitrator.c` | `arbitrator.h` | Prioritas KILL > MANUAL > AUTO |
| `libaterkia.so` | — | Shared library (build lokal, di-ignore git) |

## `config/`

| File | Isi |
|------|-----|
| `plan.csv` | Waypoint misi (`lat,lon` per baris) — **kanonik** |
| `tuning_params.json` | Semua parameter tuning (GUI simpan otomatis) |
| `archive/lintasan_a_old.csv` | Arsip lama (tidak dipakai) |

## `data/` — Output runtime (di-ignore git)

- `captures/` — foto kotak hijau & smart-capture
- `waypoint_captures/` — foto waypoint (`wp_photo_*.jpg`)
- `temp_map.html` — peta folium sementara GUI
- `bukti_wajah_ditolak.jpg` — bukti reject face (byte-identik)

## `scripts/` — Helper dev

| File | Fungsi |
|------|--------|
| `test_deteksi.py` | Live kamera + deteksi (`--flip 0-3 --debug --imgsz 640`) |
| `download_weights.sh` | Unduh model YOLO ke `weights/` |
| `run_gui.sh` | Jalankan GUI Linux (pakai `.venv`) |
| `run.bat` / `runweb.bat` | Wrapper Windows |

## `tests/` — Unit test (Python)

- `test_geo.py`, `test_fuzzy.py`, `test_state_machine.py` — referensi Python
- `test_core_*.py` — ctypes cross-check vs C (bit-per-bit)
- `test_camera.py`, `test_settings.py`, `test_detection_validation.py`, `test_filtering.py`, `test_gate_sequencer.py`, `test_mavlink_telemetry.py`

## `weights/` — Model YOLO (di-ignore git, ~6 MB/file)

| File | Misi |
|------|------|
| `buoy.pt` | Gate (2 pelampung) |
| `box_hijau.pt` | Foto kotak hijau |
| `best_blue_dark.pt` | Foto kotak biru (gelap) |
| `best_red_new.pt` | Docking kotak merah |

Model lama/percobaan (`box_biru.pt`, `box_merah.pt`, `best_greendock.pt`) aman dihapus.

## `docs/`

| File | Isi |
|------|-----|
| `ARSITEKTUR.md` | Alur data + modul C + power + kamera + deteksi + sequencer |
| `MODUL.md` | (Ini) tabel modul per folder |
| `KONFIG.md` | Arti tiap parameter di `tuning_params.json` |
| `STATES.md` | 16 state misi + transisi (kenapa kapal bertingkah begitu) |