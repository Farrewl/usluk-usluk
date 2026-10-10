# State Machine Misi

Dokumen ini merekam **apa yang dilakukan kapal di tiap state dan kenapa**.
Kalau perilaku kapal aneh, baca dokumen ini + `docs/KONFIG.md`.

Tabel `sm_state_t` ada **18 state** (`core/include/state_machine.h`,
dibungkus `app/aterkia_core.py`). Keputusan transisi dihitung di **C**
(`sm_transition`); Python hanya menerapkan efek samping (wp++ / reset timer /
reset vision). `sm_guard` menangani state "penjaga" (telemetri / waypoint).

---

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
  TAKE_PHOTO ─▶ RETREAT (brake → netral → mundur) ─▶ WAYPOINT_NAV (wp++)

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

  Guard (menyela misi): NO_TELEM · NO_WAYPOINTS · MISSION_COMPLETE
```

---

## Tabel State (18)

| # | State | Perilaku utama | Ke state berikutnya |
|---|---|---|---|
| 0 | `WAITING_GPS` | Thrust 0 sambil tunggu GPS fix + attitude. Setelah fix, navigator memulai misi (promosi ke `WAYPOINT_NAV` saat guard NONE). | `WAYPOINT_NAV` |
| 1 | `NO_TELEM` | Guard: GPS/attitude belum dapat di loop → thrust 0, tunggu | `WAYPOINT_NAV` (saat telemetri pulih) |
| 2 | `NO_WAYPOINTS` | Guard: `plan.csv` kosong / belum dimuat → idle | `WAYPOINT_NAV` (saat WP ada) |
| 3 | `MISSION_COMPLETE` | Semua WP selesai; thrust 0 | (selesai) |
| 4 | `WAYPOINT_NAV` | Thrust tetap; target yaw = bearing ke WP; koreksi gate bila leg vision aktif | lihat peta di atas |
| 5 | `WAYPOINT_TRANSITION` | Masa tenang setelah ganti WP (hindari osilasi) | `WAYPOINT_NAV` |
| 6 | `TAKE_WAYPOINT_PHOTO` | Berhenti `WAYPOINT_PHOTO_STOP_DURATION_S`, snap foto waypoint | `APPROACH_RED_BOX_SEARCH` (wp==`RED_BOX_NAV_AFTER_WP`) / `WAYPOINT_TRANSITION` |
| 7 | `APPROACH_BOX_SEARCH` | Rotasi cari kotak hijau (search thrust) | `APPROACH_BOX_ALIGN` |
| 8 | `APPROACH_BOX_ALIGN` | Koreksi yaw halus menuju kotak hijau | `TAKE_PHOTO` |
| 9 | `TAKE_PHOTO` | Berhenti, fotokan kotak hijau | `RETREAT` |
| 10 | `RETREAT` | Brake → netral → mundur (jauh dari kotak) | `WAYPOINT_NAV` (wp++) |
| 11 | `APPROACH_BLUE_BOX_SEARCH` | Rotasi cari kotak biru | `APPROACH_BLUE_BOX_ALIGN` |
| 12 | `APPROACH_BLUE_BOX_ALIGN` | Koreksi yaw & offset lateral ke kotak biru | `TAKE_BLUE_BOX_PHOTO` |
| 13 | `TAKE_BLUE_BOX_PHOTO` | Stabilisasi, kamera bawah air (smart capture) | `BLUE_BOX_RETREAT` |
| 14 | `BLUE_BOX_RETREAT` | Mundur; lanjut docking bila model merah ada | `APPROACH_RED_BOX_SEARCH` / `WAYPOINT_NAV` |
| 15 | `APPROACH_RED_BOX_SEARCH` | Rotasi cari kotak merah (docking) | `APPROACH_RED_BOX_ALIGN` |
| 16 | `APPROACH_RED_BOX_ALIGN` | Koreksi yaw presisi; hilang target → kembali SEARCH | `RED_BOX_DOCKED` |
| 17 | `RED_BOX_DOCKED` | Hold diam selama `DOCK_HOLD_DURATION_S` | `WAYPOINT_TRANSITION` / `MISSION_COMPLETE` |

---

## Sub-machine Mundur (`RETREAT` / `BLUE_BOX_RETREAT`)

Urutan langkah (`sm_retreat_step_t`, nama via `sm_retreat_step_name`):

| Step | Nama | Aksi |
|---|---|---|
| 0 | `IDLE` | belum mulai |
| 1 | `START_BRAKE` | rem (thrust lawan arah) |
| 2 | `GOTO_NEUTRAL` | lepas gas sebentar |
| 3 | `START_REVERSE` | mundur `RETREAT_THRUST` selama `RETREAT_DURATION_S` |

---

## Parameter yang Mempengaruhi Transisi

Hampir semua transisi mengacu ke `config` (lihat `docs/KONFIG.md`):
`ACCEPTANCE_RADIUS_M`, `TRANSITION_DURATION_S`, `PHOTO_BOX_LEGS`,
`BLUE_BOX_PHOTO_LEGS`, `STOP_AND_PHOTO_AT_WP`, `RED_BOX_NAV_AFTER_WP`,
`DETECTION_CONFIRM_DURATION_S`, `*_APPROACH_DISTANCE_M`,
`RED_BOX_DOCK_DISTANCE_M`, `DOCK_HOLD_DURATION_S`, `RETREAT_DURATION_S`,
`YAW_SEARCH_*`, dan parameter thrust (`SEARCH_*`, `ALIGN_*`, `DOCK_*`).

---

## Mode Operasi (di luar state misi)

Mode op (`core/src/mode_manager.c`) menentukan apakah aksi state dijalankan:

- **AUTO** — misi otonom berjalan (aksi per-state aktif).
- **MANUAL** — kendali dari RC/gamepad lokal atau QGC; aksi misi ditahan
  (thrust dari manual).
- **KILL** — tombol GUI: thrust 0 + DISARM ke Pixhawk.

Saat **bukan AUTO**, navigator menahan aksi misi (thrust 0 / target yaw tetap)
dan tidak meminta deteksi baru.
