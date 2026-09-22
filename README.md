# ASV Backend — Roboboat Aterkia

Backend navigasi & dashboard untuk **Autonomous Surface Vehicle (kapal robot)**.
Sistem membawa kapal menyusuri waypoint, melewati *gate* (2 pelampung),
memotret kotak, dan melakukan *docking* ke kotak merah — semuanya berbasis
visi (YOLO) + telemetri.

```
Kamera ─▶ YOLO (deteksi) ─▶ State Machine misi ─▶ MAVLink ─▶ Pixhawk (offboard)
   │                            │
   └──── frame video ───────────┴──▶ GUI PyQt5 (map folium + tuning)
                                        │
                                        └─▶ Redis ─▶ Web dashboard (opsional)
```

> Repo ini sedang ditata ulang. **Tahap 1** (selesai): struktur & git dibersihkan.
> **Tahap 2** (berjalan): inti navigasi dikonversi ke C/C++ (`core/`).

## Struktur Repo

```
asv_backend/
├── main.py                  # ENTRY: python main.py -> jalankan GUI
├── app/                     # Paket Python (GUI + logika navigasi)
│   ├── simulator.py         #   simulator darat (kamera asli / mock)
│   ├── navigator.py         #   navigator asli (Pixhawk + MAVLink)
│   ├── gateway.py           #   relay Redis <-> WebSocket dashboard
│   ├── settings.py          #   semua parameter & path (satu sumber)
│   ├── geo.py               #   matematika geodetik terpusat
│   └── scripts/             #   tool dev: tes kamera, exposure, konflik
├── core/                    # 🆕 inti C/C++ hasil konversi (Tahap 2)
├── config/                  # plan.csv (waypoint) + tuning_params.json
├── weights/                 # model YOLO (*.pt) — UNDUH, bukan di-git
├── data/                    # output runtime (captures, map sementara)
├── docs/                    # ARSITEKTUR, STATE MACHINE, KONFIGURASI
├── scripts/                 # run_gui.sh, download_weights.sh, run.bat...
└── tests/                   # unit test
```

## Cara Menjalankan

**Windows (dev):**
```bat
python main.py            REM GUI utama
python -m app.gateway     REM relay ke dashboard web (opsional)
```

**Linux / Raspberry Pi:**
```bash
./scripts/run_gui.sh
```

**Sebelum pertama kali:** unduh model YOLO ke `weights/` (lihat
`scripts/download_weights.sh`) dan isi `config/plan.csv` dengan waypoint
(format: `lat,lon`).

## Pilih Backend (main.py)

Di bagian atas `main.py`, pilih satu:

```python
from app.simulator import NavigatorThread   # simulator darat (kamera asli)
# from app.navigator import NavigatorThread  # kapal asli (Pixhawk)
```

## Dokumentasi Lain

| Dokumen | Isi |
|---|---|
| `docs/ARCHITECTURE.md` | Alur data lengkap + peta konversi Python → C/C++ |
| `docs/STATES.md` | Semua state misi + transisi (kenapa kapal bertingkah begitu) |
| `docs/CONFIG.md` | Arti setiap parameter tuning |
| `app/geo.py` | Rumus geodetik (haversine, cross-track) — bahan port ke C |