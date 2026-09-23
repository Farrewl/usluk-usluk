# core/ — Inti C/C++ (Tahap 2, dalam pengerjaan)

Konversi bertahap inti navigasi dari Python ke C/C++ agar responsif ke
mesin. Semua matematika & MAVLink **murni C**, visi **C++** (ONNX Runtime).
GUI (`app/`) tetap Python dan berkomunikasi via Redis + ZMQ.

## Rencana Modul

```
core/
├── include/           # header publik
├── src/
│   ├── nav_math.c     # C: haversine, bearing, cross-track, normalize   ← dari app/geo.py
│   ├── fuzzy.c        # C: controller Sugeno gate & docking (trapmf + rule) ← dari app/fuzzy.py
│   ├── state_machine.c   # C: mesin state misi (±16 state) ← dari app/state_machine.py
│   ├── mavlink_bridge.c  # C: libmavlink + serial (heartbeat, SET_ATTITUDE_TARGET)
│   ├── camera.cpp     # C++: OpenCV capture + resize + ROI
│   ├── vision_detector.cpp # C++: ONNX Runtime inference (4 model YOLO)
│   └── main.cpp       # pipeline: thread telemetri + vision + kontrol
├── tests/             # unit test tiap modul (assert-output vs Python)
└── CMakeLists.txt
```

## Status Modul

| Modul | Sumber Python | Status | Verifikasi |
|---|---|---|---|
| `nav_math.c` | `app/geo.py` | selesai | `core/tests/test_nav_math.c` + `tests/test_core_nav_math.py` |
| `fuzzy.c` | `app/fuzzy.py` | selesai | `core/tests/test_fuzzy.c` + `tests/test_core_fuzzy.py` |
| `state_machine.c` | `app/state_machine.py` | selesai | `core/tests/test_state_machine.c` + `tests/test_core_state_machine.py` |
| `mavlink_bridge.c` | `app/mavlink_telemetry.py` | opsional | — |

## Aturan Konversi

1. **Output harus identik** dengan Python sebelum dipakai → setiap modul
   diuji terhadap hasil `app/geo.py` / `app/navigator.py` (lihat `tests/`).
2. C murni untuk: math, MAVLink, fuzzy, state machine (tidak ada STL).
3. C++ hanya untuk: kamera & inferensi ONNX (butuh RAII/STL opencv).
4. Tidak ada magic number: semua konstanta `#define`/`enum` dengan nama jelas.
5. Setiap file punya header docblock (tujuan, alur, dipakai siapa).

## Referensi

- `docs/ARCHITECTURE.md` — peta konversi Python → C/C++ per modul
- `app/geo.py` — spesifikasi matematika yang sudah tervalidasi