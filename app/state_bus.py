"""app/state_bus.py — bus status terbaru (thread-safe) navigator <-> web.

Navigator `_emit()` memanggil `update(...)`; server web memanggil
`snapshot()` (SSE) dan `get_frame()` (MJPEG). Tanpa dependency PyQt.
"""

import copy
import threading
import time

_lock = threading.Lock()

_DEFAULT_MISSION = {
    "prep": False, "start": False, "balls": 0,
    "surface": False, "under": False, "dock": False,
    "score_surface": "", "score_under": "",
}

_snapshot = {
    "type": "telemetry",
    "team": "", "pt": "", "track": "A",
    "track_start": None,
    "lat": None, "lon": None,
    "yaw_deg": 0.0, "pitch_deg": 0.0, "roll_deg": 0.0,
    "sog_kn": 0.0, "sog_kmh": 0.0, "cog_deg": 0.0, "groundspeed": 0.0,
    "battery_pct": None, "voltage_v": None, "current_a": None,
    "state": "--", "wp_idx": 0, "wp_total": 0,
    "mavlink_ok": False, "gps_fix": False,
    "op_mode": "AUTO", "failsafe_active": False, "status": "--",
    "mission": dict(_DEFAULT_MISSION),
    "last_photo": None,
    "ts": 0.0,
}

_frame = None
_frame_seq = 0
_start_ts = time.time()


def update(patch):
    with _lock:
        _snapshot.update(patch)
        _snapshot["ts"] = time.time()


def update_mission(**kw):
    with _lock:
        mission = dict(_snapshot.get("mission") or _DEFAULT_MISSION)
        mission.update(kw)
        _snapshot["mission"] = mission
        _snapshot["ts"] = time.time()


def set_photo(info):
    with _lock:
        _snapshot["last_photo"] = info
        _snapshot["ts"] = time.time()


def snapshot():
    with _lock:
        return copy.deepcopy(_snapshot)


def set_frame(img):
    global _frame, _frame_seq
    with _lock:
        _frame = img
        _frame_seq += 1


def get_frame():
    with _lock:
        return _frame, _frame_seq


def uptime_s():
    return time.time() - _start_ts
