# State Machine Misi (dari `app/navigator.py`)

Ringkasan ini merekam **apa yang dilakukan kapal di tiap state dan kenapa**.
Kalau perilaku kapal aneh, baca dokumen ini + `docs/CONFIG.md`.

## Peta State

```
        ┌────────────── WAITING_GPS ──────────────┐
        │   (tunggu GPS 3D fix + attitude)        │
        ▼                                         │
  WAYPOINT_NAV ── deteksi gate aktif (leg 1,3,5)  │
        │                                         │
        ├─ wp ∈ PHOTO_BOX_LEGS      ──▶ APPROACH_BOX_SEARCH
        ├─ wp ∈ BLUE_BOX_PHOTO_LEGS ──▶ APPROACH_BLUE_BOX_SEARCH
        ├─ wp ∈ STOP_AND_PHOTO_AT_WP ─▶ TAKE_WAYPOINT_PHOTO
        ├─ jarak < ACCEPTANCE_RADIUS ─▶ WAYPOINT_TRANSITION (wp++)
        └─ (pilihan lain)            ── tetap nav + koreksi gate

  WAYPOINT_TRANSITION ──▶ WAYPOINT_NAV        (setelah TRANSITION_DURATION_S)

  APPROACH_BOX_SEARCH ─deteksi 0.5s─▶ APPROACH_BOX_ALIGN ─jarak≤BOX_APPROACH─▶ TAKE_PHOTO
        (rotate YAW_SEARCH_BOX)          (koreksi yaw halus)          │ hilang>2s: tetap TAKE_PHOTO
                                                                      ▼
  TAKE_PHOTO ─▶ RETREAT (brake 0.2s → neutral 0.4s → mundur) ─▶ WAYPOINT_NAV (wp++)

  APPROACH_BLUE_BOX_SEARCH ─▶ APPROACH_BLUE_BOX_ALIGN ─jarak≤BLUE_BOX_APPROACH─▶ TAKE_BLUE_BOX_PHOTO
                                                                      │
                                       TAKE_BLUE_BOX_PHOTO (smart capture bawah air)
                                                                      ▼
                                        BLUE_BOX_RETREAT ─▶ APPROACH_RED_BOX_SEARCH (jika model ada)
                                                               atau WAYPOINT_NAV (wp++)

  APPROACH_RED_BOX_SEARCH ─found─▶ APPROACH_RED_BOX_ALIGN ─jarak≤DOCK_DIST─▶ RED_BOX_DOCKED
        (rotate YAW_SEARCH_DOCK)      │ hilang → kembali ke SEARCH            │ hold DOCK_HOLD
                                                                              ▼
                                    WAYPOINT_TRANSITION (wp++) / MISSION_COMPLETE
```

## Tabel State

| # | State | Perilaku utama | Ke state berikutnya |
|---|---|---|---|
| 1 | `WAITING_GPS` | Kirim thrust 0 sambil tunggu fix GPS + attitude dari Pixhawk | (loop) |
| 2 | `WAYPOINT_NAV` | Thrust tetap; target yaw = bearing ke WP; koreksi gate bila leg vision aktif | lihat legenda di atas |
| 3 | `WAYPOINT_TRANSITION` | Masa tenang setelah ganti WP (hindari osilasi) | `WAYPOINT_NAV` |
| 4 | `TAKE_WAYPOINT_PHOTO` | Berhenti 0.5s, snap foto waypoint | `APPROACH_RED_BOX_SEARCH` (wp==RED_BOX_NAV_AFTER_WP) / `WAYPOINT_TRANSITION` |
| 5 | `APPROACH_BOX_SEARCH` | Rotasi cari kotak hijau (search thrust) | `APPROACH_BOX_ALIGN` |
| 6 | `APPROACH_BOX_ALIGN` | Koreksi yaw halus menuju kotak hijau | `TAKE_PHOTO` |
| 7 | `TAKE_PHOTO` | Berhenti, fotokan kotak hijau | `RETREAT` |
| 8 | `RETREAT` | Brake → netral → mundur (jauh dari kotak) | `WAYPOINT_NAV` (wp++) |
| 9 | `APPROACH_BLUE_BOX_SEARCH` | Rotasi cari kotak biru | `APPROACH_BLUE_BOX_ALIGN` |
| 10 | `APPROACH_BLUE_BOX_ALIGN` | Koreksi yaw & offset lateral ke kotak biru | `TAKE_BLUE_BOX_PHOTO` |
| 11 | `TAKE_BLUE_BOX_PHOTO` | Stabilisasi 2s, kamera bawah air (smart capture) | `BLUE_BOX_RETREAT` |
| 12 | `BLUE_BOX_RETREAT` | Mundur; lanjut docking bila model merah ada | `APPROACH_RED_BOX_SEARCH` / `WAYPOINT_NAV` |
| 13 | `APPROACH_RED_BOX_SEARCH` | Rotasi cari kotak merah (docking area) | `APPROACH_RED_BOX_ALIGN` |
| 14 | `APPROACH_RED_BOX_ALIGN` | Koreksi yaw presisi; hilang target → kembali SEARCH | `RED_BOX_DOCKED` |
| 15 | `RED_BOX_DOCKED` | Hold diam selama DOCK_HOLD_DURATION_S | `WAYPOINT_TRANSITION` / `MISSION_COMPLETE` |
| 16 | `MISSION_COMPLETE` | Semua WP selesai; thrust 0 | (selesai) |

## State "Jaga-Jaga" (bukan misi)

| State | Kondisi | Aksi |
|---|---|---|
| `NO_TELEM` | GPS/attitude belum dapat di loop utama | thrust 0, tunggu |
| `NO_WAYPOINTS` | `plan.csv` kosong / belum dimuat | idle |
| `MISSION_COMPLETE` | index WP melewati daftar | cruising stop |

## Parameter yang Mempengaruhi Transisi

Hampir semua transisi mengacu ke `config` (lihat `docs/CONFIG.md`):
`ACCEPTANCE_RADIUS_M`, `TRANSITION_DURATION_S`, `PHOTO_BOX_LEGS`,
`BLUE_BOX_PHOTO_LEGS`, `STOP_AND_PHOTO_AT_WP`, `RED_BOX_NAV_AFTER_WP`,
`DETECTION_CONFIRM_DURATION_S`, `*_APPROACH_DISTANCE_M`, `DOCK_HOLD_DURATION_S`,
`RETREAT_DURATION_S`, `YAW_SEARCH_*`, dan parameter thrust (`SEARCH_*`, `ALIGN_*`).