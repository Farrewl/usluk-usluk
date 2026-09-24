# Arsitektur Aterkia ASV — Ringkas

## Ringkasan Alur Data

```
Kamera USB ─▶ YOLO (deteksi) ─▶ GateSequencer (midpoint gate)
                                │
                                ▼
                    koreksi yaw (rad) → PID + deadband
                                │
                                ▼
              Complementary filter + EKF heading (C core)
                                │
                                ▼
               Thruster mixer (differential drive) ─▶ MAVLink OFFBOARD
                                │
                                ▼
                        Pixhawk → ESC → Thruster kiri/kanan
```

## Modul C/C++ Inti (`core/`)

| File | Fungsi | Sumber Python |
|------|--------|---------------|
| `nav_math.c` | Haversine, bearing, CTE, normalize | `app/geo.py` |
| `fuzzy.c` | Sugeno singleton gate/docking gain | `app/fuzzy.py` |
| `state_machine.c` | 16 state misi | `app/state_machine.py` |
| `mavlink_bridge.c` | Framing MAVLink v1 + serial | `pymavlink` |
| `control_filters.c` | PID+deadband, complementary, EKF heading | `app/filtering.py` |
| `thruster_mixer.c` | Differential drive: surge+yaw → PWM 1100–1900 | baru |
| `ecu_link.c` | Protokol UART STM32 (8 byte frame + checksum) | baru |
| `arbitrator.c` | Prioritas: KILL > MANUAL > AUTO | baru |

Semua modul C diverifikasi **bit-per-bit** vs Python via `tests/test_core_*.py` (ctypes cross-check).

## Power & Safety

- **LiPO 4S (motor)** → ESC → Thruster. Pixhawk PWM utama.
- **LiFePO4 4S (PC/NUC)** → NUC + kamera + sensor.
- **E-stop hardware** → STM32 → TC4420 kill line (hardware cut, independen NUC/Pixhawk).
- **STM32 supervisor**: baca ACS758 (arus), NTC B3950 (ESC temp), BME280 (ambient), resistor divider (tegangan) → kirim frame 8 byte tiap 100 ms ke NUC via UART.
- **Watchdog**: NUC heartbeat putus >500 ms ATAU E-stop → STM32 potong driver.

## Baterai (GUI)

- Tegangan pack dari SYS_STATUS/BATTERY_STATUS (mV). LiPO 4S: full 16.8 V, empty 12.8 V.
- Persen dihitung linear dari tegangan kalau firmware tak kirim `battery_remaining`.
- Chip status bar: hijau ≥50%, kuning 20–50%, merah <20%, abu = no data.

## Kamera & FPS

- Target 20 fps stabil (`CAMERA_TARGET_FPS=20`, `CAMERA_MIN_ACCEPT_FPS=15`).
- Resolusi negosiasi otomatis: pilih area terbesar (1280×720 MJPG dipilih).
- Flip opsional: `CAMERA_FLIP_MODE` (0=normal, 1=h-flip, 2=v-flip, 3=180°), default 0.

## Deteksi Buoy

- Threshold longgar (override via `tuning_params.json`):
  - `BUOY_CONF_THRESHOLD=0.35`
  - `BUOY_MIN_COLOR_FRACTION=0.05`
  - `BUOY_MIN_SATURATION=0.35`
  - `BUOY_MAX_ASPECT_DEVIATION=0.50`
  - `MIN_BUOY_AREA_PX=40`
- `validate_buoy(..., debug=True)` return `(ok, reasons[])` — print alasan tolak per kriteria.

## Gate Sequencer

- Kumpulkan semua pasang pelampung plausibel → latch target aktif.
- Switch cepat ke gate berikutnya saat:
  - Jarak < `GATE_PASS_DISTANCE_M` ATAU
  - Midpoint turun lewat tengah frame (vertikal < `GATE_VERTICAL_ALIGN_PX=75px`)
- Latch `last_vision_correction_rad` supaya tidak lompat saat kehilangan deteksi.

## Build & Test

```bash
# Build C library (sekali saja)
gcc -shared -fPIC -Icore/include core/src/*.c -o core/libaterkia.so

# Test Python (termasuk ctypes cross-check C vs Python)
python3 -m unittest discover -s tests -v

# GUI headless
QT_QPA_PLATFORM=offscreen python main.py
```

## Entry Point

- `main.py` — pilih backend: `app.simulator.NavigatorThread` (kamera asli) atau `app.navigator.NavigatorThread` (Pixhawk).
- GUI: PyQt5 + folium map + tuning panel + video + HUD + status bar.