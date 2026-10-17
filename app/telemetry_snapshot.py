"""app/telemetry_snapshot.py — rakit snapshot JSON LENGKAP kondisi kapal.

Dipakai `app/monitor_export.py` untuk menulis `data/monitor/telemetry.json`
yang dibaca oleh **dashboard web terpisah** (repo lain). Modul ini murni:
tanpa jaringan, tanpa HTTP, tanpa dependency web — hanya membentuk dict
JSON-serializable dari paket telemetri + `config` (parameter kapal).

Skema (versi 1):
    { schema, ts, uptime_s,
      identity{team,pt,track,track_start},
      pose{lat,lon,yaw_deg,pitch_deg,roll_deg},
      motion{sog_kn,sog_kmh,cog_deg,groundspeed},
      power{battery_pct,voltage_v,current_a},
      link{mavlink_ok,port,reason,gcs_forward,qgc_active,rc_ok,manual_source,
           telem_source,pixhawk_check},
      mission{state,op_mode,wp_idx,wp_total,dist_to_wp_m,failsafe_active,
              failsafe_reason,status,avoid_active,avoid_bias,flags{...}},
      vision{...}, camera{...}, last_photo{...},
      params{...SEMUA parameter config...}, health{...} }
"""

import time

SCHEMA = 1

_SKIP_PARAM = frozenset({
    # objek berat / duplikat; biarkan utelemetri & tuning yang penting
    "SITE",
})


def _safe(value, depth=0):
    """Kembalikan nilai JSON-serializable; objek aneh -> None/str pendek."""
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if depth > 6:
        return None
    try:
        import numpy as np
        if isinstance(value, np.bool_):
            return bool(value)
        if isinstance(value, np.integer):
            return int(value)
        if isinstance(value, np.floating):
            return float(value)
        if isinstance(value, np.ndarray):
            return _safe(value.tolist(), depth + 1)
    except Exception:
        pass
    if isinstance(value, dict):
        return {str(k): _safe(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_safe(v, depth + 1) for v in value]
    return None


def dump_params(config):
    """Semua atribut publik `config` (parameter/tuning kapal) yang JSON-able."""
    out = {}
    if config is None:
        return out
    for name, val in vars(config).items():
        if name.startswith("_") or name in _SKIP_PARAM:
            continue
        if callable(val):
            continue
        out[name] = _safe(val)
    # buang yang tak terwakili (None hasil _safe untuk objek kompleks) hanya
    # bila key-nya memang bukan None asli; sisakan None asli.
    return {k: v for k, v in out.items()
            if v is not None or getattr(config, k, None) is None}


def build_snapshot(pkt, config, mission=None, vision=None, camera=None,
                   last_photo=None, extra=None):
    """Bentuk dict snapshot penuh dari satu paket telemetri + config."""
    pkt = pkt or {}
    return {
        "schema": SCHEMA,
        "ts": time.time(),
        "identity": {
            "team": getattr(config, "TEAM_NAME", ""),
            "pt": getattr(config, "TEAM_PT", ""),
            "track": getattr(config, "TRACK", "A"),
            "track_start": _safe(getattr(config, "TRACK_START", None)),
        },
        "pose": {
            "lat": _safe(pkt.get("lat")), "lon": _safe(pkt.get("lon")),
            "yaw_deg": _safe(pkt.get("yaw_deg")),
            "pitch_deg": _safe(pkt.get("pitch_deg")),
            "roll_deg": _safe(pkt.get("roll_deg")),
        },
        "motion": {
            "sog_kn": _safe(pkt.get("sog_kn")),
            "sog_kmh": _safe(pkt.get("sog_kmh")),
            "cog_deg": _safe(pkt.get("cog_deg")),
            "groundspeed": _safe(pkt.get("groundspeed")),
        },
        "power": {
            "battery_pct": _safe(pkt.get("battery_pct")),
            "voltage_v": _safe(pkt.get("voltage_v")),
            "current_a": _safe(pkt.get("current_a")),
        },
        "link": {
            "mavlink_ok": _safe(pkt.get("mavlink_ok")),
            "port": _safe(pkt.get("mav_port")),
            "reason": _safe(pkt.get("mav_reason")),
            "gcs_forward": _safe(pkt.get("gcs_forward")),
            "qgc_active": _safe(pkt.get("qgc_active")),
            "rc_ok": _safe(pkt.get("rc_ok")),
            "manual_source": _safe(pkt.get("manual_source")),
            "telem_source": _safe(pkt.get("telem_source")),
            "pixhawk_check": _safe(pkt.get("pixhawk_check")),
        },
        "mission": {
            "state": _safe(pkt.get("state")),
            "op_mode": _safe(pkt.get("op_mode")),
            "wp_idx": _safe(pkt.get("target_wp_idx")),
            "wp_total": _safe(pkt.get("wp_total")),
            "dist_to_wp_m": _safe(pkt.get("dist_to_wp_m")),
            "failsafe_active": _safe(pkt.get("failsafe_active")),
            "failsafe_reason": _safe(pkt.get("failsafe_reason")),
            "status": _safe(pkt.get("status")),
            "avoid_active": _safe(pkt.get("avoid_active")),
            "avoid_bias": _safe(pkt.get("avoid_bias")),
            "flags": _safe(mission or {}),
        },
        "vision": _safe(vision or {}),
        "camera": _safe(camera or {"status": pkt.get("cam_status")}),
        "last_photo": _safe(last_photo),
        "params": dump_params(config),
        "health": _safe(extra or {}),
    }
