# app/ — Paket Python ASV

| File | Isi |
|---|---|
| `settings.py` | Semua parameter & path standar (satu sumber) |
| `geo.py` | Matematika geodetik murni (haversine, bearing, cross-track) |
| `camera.py` | Buka kamera sesuai OS (V4L2/DSHOW) + kalibrasi exposure + cari index yang bisa |
| `mavlink_telemetry.py` | Baca telemetri MAVLink Pixhawk (attitude roll/pitch/yaw, posisi) non-blocking |
| `navigator.py` | Navigasi asli: state machine misi + MAVLink offboard (Pixhawk) |
| `simulator.py` | 2 simulator darat: `GroundSimNavigator` (kamera asli) & `MockSimNavigator` (tanpa kamera) |
| `gateway.py` | Relay Redis ↔ WebSocket ke dashboard web (jalan: `python -m app.gateway`) |
| `scripts/` | Tool dev: `cek_camera.py`, `tes_exposure.py`, `tes_conflict.py` |

## Kontrak Thread Navigator

`main.py` memakai kelas `NavigatorThread` (QThread) yang wajib menyediakan:
`newData` (sinyal dict), `update_config_param(key, value)`,
`save_config_to_file()`, `update_waypoints(list)`, `stop()`.
Implementasi ada di `simulator.py` (2x) dan `navigator.py`.