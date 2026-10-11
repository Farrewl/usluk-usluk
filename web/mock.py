"""web/mock.py — feeder data palsu untuk demo dashboard tanpa kapal.

Aktif bila env ASV_MOCK=1. Menggerakkan posisi sepanjang waypoint lintasan,
menaikkan progres misi, dan membuat frame sintetis supaya MJPEG ada isinya.
"""

import math
import threading
import time

import cv2
import numpy as np

from app import state_bus
from app import settings as cfg


def _frame(t, lat, lon):
    img = np.full((360, 640, 3), 18, dtype=np.uint8)
    cv2.line(img, (0, 180), (640, 180), (40, 60, 40), 2)
    x = int(160 + 160 * math.sin(t * 0.7))
    cv2.circle(img, (x, 210), 16, (40, 200, 80), -1)
    cv2.circle(img, (x + 120, 205), 16, (40, 40, 220), -1)
    cv2.putText(img, "MOCK CAMERA", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (200, 200, 200), 2)
    cv2.putText(img, f"{lat:.5f},{lon:.5f}", (20, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (140, 140, 140), 1)
    return img


def start_mock(cfgobj, hz=20.0):
    """Mulai thread daemon pengisi state_bus. Return thread."""

    def _run():
        wps = cfgobj.WAYPOINTS or []
        if len(wps) < 2:
            wps = [{"lat": -7.9153897, "lon": 112.589158},
                   {"lat": -7.9154700, "lon": 112.5890300}]
        state_bus.update({
            "team": getattr(cfgobj, "TEAM_NAME", ""),
            "pt": getattr(cfgobj, "TEAM_PT", ""),
            "track": getattr(cfgobj, "TRACK", "A"),
            "wp_total": len(wps),
            "telem_source": "MOCK",
        })
        i = 0
        frac = 0.0
        balls = 0
        t0 = time.time()
        surface = False
        under = False
        dock = False
        while True:
            t = time.time() - t0
            a = wps[i % len(wps)]
            b = wps[(i + 1) % len(wps)]
            frac += 0.01
            if frac >= 1.0:
                frac = 0.0
                i += 1
                balls = min(10, balls + (1 if (i % 2 == 0) else 0))
                if i >= 5:
                    surface = True
                if i >= 7:
                    under = True
                if i >= len(wps):
                    dock = True
                    i = 0
            lat = a["lat"] + (b["lat"] - a["lat"]) * frac
            lon = a["lon"] + (b["lon"] - a["lon"]) * frac
            cog = math.degrees(math.atan2(b["lon"] - a["lon"],
                                          b["lat"] - a["lat"])) % 360
            state_bus.update({
                "lat": lat, "lon": lon, "cog_deg": cog, "yaw_deg": cog,
                "sog_kn": 1.6, "sog_kmh": 2.96, "groundspeed": 0.82,
                "battery_pct": max(20.0, 90.0 - t * 0.1),
                "voltage_v": 15.6, "current_a": 4.2,
                "state": "WAYPOINT_NAV", "wp_idx": i,
                "mavlink_ok": True, "gps_fix": True,
                "status": "MOCK",
            })
            state_bus.update_mission(prep=True, start=(t > 2),
                                     balls=balls, surface=surface,
                                     under=under, dock=dock)
            state_bus.set_frame(_frame(t, lat, lon))
            time.sleep(1.0 / hz)

    th = threading.Thread(target=_run, name="MockFeeder", daemon=True)
    th.start()
    return th
