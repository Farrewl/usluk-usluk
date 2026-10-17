"""app/geotag.py — Geo-tag foto misi (Day/Date/Time, koordinat DD & DM, SOG, COG).

Dipakai navigator saat menulis foto kotak hijau/biru. Menulis sidecar JSON
samping file JPEG supaya monitoring bisa menampilkan popup geo-tag.
"""

import datetime
import json
import os
import time

_DAY_EN = ("Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat")


def _hemi(value, is_lat):
    if is_lat:
        return "N" if value >= 0 else "S"
    return "E" if value >= 0 else "W"


def coord_dd(lat, lon):
    """Degree Decimal: S 3.56734 E 104.67235."""
    return f"{_hemi(lat, True)} {abs(lat):.5f} {_hemi(lon, False)} {abs(lon):.5f}"


def _dm(v, is_lat):
    a = abs(v)
    d = int(a)
    mm = f"{(a - d) * 60.0:.4f}".replace(".", ",")
    return f"{_hemi(v, is_lat)} {d}° {mm}'"


def coord_dm(lat, lon):
    """Degree Minute (koma desimal): S 3° 34,0404' E 104° 40,3410'."""
    return f"{_dm(lat, True)} {_dm(lon, False)}"


def day_name(ts):
    return _DAY_EN[(datetime.datetime.fromtimestamp(ts).weekday() + 1) % 7]


def build_info(kind, path, lat, lon, sog_kn, sog_kmh, cog_deg, ts=None):
    """Dict geo-tag lengkap untuk satu capture. `kind` = surface|under|dock|wp."""
    t = ts if ts is not None else time.time()
    dt = datetime.datetime.fromtimestamp(t)
    info = {
        "kind": kind,
        "file": os.path.basename(path),
        "path": path,
        "day": _DAY_EN[(dt.weekday() + 1) % 7],
        "date": dt.strftime("%d/%m/%Y"),
        "time": dt.strftime("%H:%M:%S"),
        "ts": t,
        "lat": None if lat is None else float(lat),
        "lon": None if lon is None else float(lon),
        "sog_kn": float(sog_kn or 0.0),
        "sog_kmh": float(sog_kmh or 0.0),
        "cog_deg": float(cog_deg or 0.0),
    }
    if lat is not None and lon is not None:
        info["coord_dd"] = coord_dd(float(lat), float(lon))
        info["coord_dm"] = coord_dm(float(lat), float(lon))
    else:
        info["coord_dd"] = "--"
        info["coord_dm"] = "--"
    return info


def finalize_capture(kind, jpg_path, lat, lon, sog_kn, sog_kmh, cog_deg):
    """Tulis sidecar + kembalikan info. Return dict (selalu, walau gagal tulis)."""
    info = build_info(kind, jpg_path, lat, lon, sog_kn, sog_kmh, cog_deg)
    side = os.path.splitext(jpg_path)[0] + ".json"
    try:
        with open(side, "w", encoding="utf-8") as f:
            json.dump(info, f, indent=2, ensure_ascii=False)
        info["sidecar"] = side
    except Exception:
        info["sidecar"] = None
    return info
