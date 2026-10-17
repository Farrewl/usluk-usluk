# MONITOR — Kontrak Ekspor Monitoring (file bridge)

Repo **kapal** (`usluk-usluk`) tidak berisi web. Ia hanya **menulis** kondisi
kapal ke `data/monitor/`. Repo **dashboard web terpisah** membacanya dan
menyajikannya ke browser operator.

```
┌──────────────── mini-PC ────────────────┐
│  app kapal (main.py)                     │
│    └─ app/monitor_export.py  ── tulis ──▶ data/monitor/*.json, *.jpg
│                                          │
│  repo web terpisah (mis. :8080) ── baca ─┘  ──▶ browser laptop
└──────────────────────────────────────────┘
```

## 1. File keluaran

| File | Tipe | Isi | Frekuensi |
|---|---|---|---|
| `telemetry.json` | JSON objek | snapshot lengkap (lihat §2) | `MONITOR_HZ` (default 5 Hz) |
| `frame.jpg` | JPEG | frame kamera NAV ter-annotasi YOLO | `MONITOR_FRAME_HZ` (default 10 Hz) |
| `frame_bawah.jpg` | JPEG | kamera bawah (bila ada) | idem |
| `photos.json` | JSON array | indeks foto misi + geo-tag | saat foto diambil |
| `events.jsonl` | JSON Lines (append 1 baris/kejadian) | log kejadian | saat terjadi |

Semua penulisan **atomik**: ditulis ke `<nama>.tmp` lalu `os.replace()` →
pembaca tidak pernah melihat file separuh. Lokasi folder = `MONITOR_DIR`
(default `data/monitor`).

> Pembaca disarankan menangani `FileNotFoundError`/JSON parsial: saat app
> baru mulai, file belum ada.

## 2. Skema `telemetry.json`

```jsonc
{
  "schema": 1,              // versi kontrak; naik bila ada perubahan breaking
  "ts": 1734567890.12,      // waktu tulis (epoch detik)
  "identity": {
    "team": "Aterkia", "pt": "Universitas Diponegoro",
    "track": "B",           // lintasan aktif: "A" | "B"
    "track_start": {"lat": -6.9, "lon": 110.4, "color": "red"}
  },
  "pose":   {"lat": -6.9, "lon": 110.4,
             "yaw_deg": 12.0, "pitch_deg": 1.0, "roll_deg": -2.0},
  "motion": {"sog_kn": 2.3, "sog_kmh": 4.3, "cog_deg": 12.0, "groundspeed": 1.2},
  "power":  {"battery_pct": 88.0, "voltage_v": 15.2, "current_a": 3.1},
  "link": {
    "mavlink_ok": true, "port": "/dev/ttyACM1", "reason": "",
    "gcs_forward": false, "qgc_active": false, "rc_ok": true,
    "manual_source": "AUTO", "telem_source": "REAL",
    "pixhawk_check": "[PX4] OK"
  },
  "mission": {
    "state": "NAVIGATING",      // state mesin misi (18 state)
    "op_mode": "AUTO",          // AUTO | MANUAL | HOLD | KILL
    "wp_idx": 2, "wp_total": 5, "dist_to_wp_m": 3.5,
    "failsafe_active": false, "failsafe_reason": "",
    "status": "ok", "avoid_active": false, "avoid_bias": 0.0,
    "flags": {                  // progres misi KKI
      "prep": true, "start": true, "balls": 3,
      "surface": false, "under": false, "dock": false
    }
  },
  "vision": {
    "gate_passed": 3,
    "models": {                 // per model YOLO
      "gate":  {"detected": true, "n": 2, "age_s": 0.05},
      "box":   {"detected": false, "n": 0, "age_s": 0.1},
      "blue":  {"detected": false, "n": 0, "age_s": 0.1},
      "red":   {"detected": false, "n": 0, "age_s": 0.1}
    }
  },
  "camera": {"status": { /* status kamera (dict bebas) */ }},
  "last_photo": { /* info foto terakhir, lihat §3 */ },
  "params": { /* SEMUA parameter config/run-time kapal (tuning, ambang, dst) */ },
  "health": {"uptime_s": 512.4}
}
```

Catatan satuan: `sog_kn` knot, `sog_kmh` km/jam, `groundspeed` m/detik, sudut
derajat, `dist_to_wp_m` meter, baterai `battery_pct` persen (atau `null`).

Field bisa bernilai `null` saat data belum tersedia (mis. GPS belum fix).
`params` memuat seluruh atribut `Config` (lihat `app/settings.py`) — dipakai
web untuk panel diagnosa/tuning.

## 3. `photos.json`

Array objek hasil `app/geotag.py` (`build_info`/`finalize_capture`):

```jsonc
[{
  "kind": "surface",                     // surface | under | dock | wp
  "file": "x.jpg",
  "path": "data/captures/x.jpg",         // path relatif root repo kapal
  "day": "Mon", "date": "17/10/2026", "time": "14:03:22",
  "ts": 1734567890.0,                     // epoch detik
  "lat": -6.9, "lon": 110.4,
  "sog_kn": 2.3, "sog_kmh": 4.3, "cog_deg": 12.0,
  "coord_dd": "S 6.90000 E 110.40000",   // Degree Decimal
  "coord_dm": "S 6° 54,0000' E 110° 24,0000'",  // Degree Minute (koma)
  "sidecar": "data/captures/x.json"       // file geo-tag di samping JPEG (atau null)
}]
```

Gambar aslinya ada di `data/captures/` dan `data/waypoint_captures/`;
sidecar `.json` (geo-tag) ada di samping tiap JPEG.

## 4. `events.jsonl`

Satu objek JSON per baris, append-only:

```jsonc
{"ts": 1734567890.1, "kind": "photo", "photo_kind": "surface", "path": "..."}
```

`kind` contoh: `photo`, (bisa ditambah `state`, `failsafe`, `gate`).

## 5. Cara repo web mengonsumsi (mini-PC yang sama)

1. Baca `data/monitor/telemetry.json` tiap ~SSE/refresh (mis. 5 Hz) → render
   telemetri, misi, visi.
2. Sajikan `data/monitor/frame.jpg` sebagai gambar yang di-refresh berkala
   (polling `<img>` lebih tahan jaringan goyah daripada MJPEG panjang).
3. Baca `data/monitor/photos.json` untuk galeri; sajikan JPEG dari
   `data/captures/` + `data/waypoint_captures/`.
4. Opsional: `tail -f data/monitor/events.jsonl` untuk log kejadian.

## 6. Mematikan / mengatur

Di `config/site.json` (atau env `ASV_MONITOR*`):

| Kunci | Default | Arti |
|---|---|---|
| `MONITOR_ENABLED` | `true` | Matikan total ekspor (`ASV_MONITOR=0`) |
| `MONITOR_DIR` | `data/monitor` | Folder keluaran (`ASV_MONITOR_DIR`) |
| `MONITOR_HZ` | `5.0` | Frekuensi tulis `telemetry.json` (`ASV_MONITOR_HZ`) |
| `MONITOR_FRAME` | `true` | Tulis frame JPEG |
| `MONITOR_FRAME_HZ` | `10.0` | Frekuensi tulis frame |
| `MONITOR_FRAME_WIDTH` | `640` | Lebar frame (skala turun) |
| `MONITOR_JPEG_QUALITY` | `70` | Kualitas JPEG (0–100) |
| `MONITOR_EVENTS` | `true` | Tulis `events.jsonl` |
