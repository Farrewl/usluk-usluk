#!/usr/bin/env python3
"""
tests/test_core_monitor_export.py — Unit test ekspor monitoring (file bridge).

Mengunci kontrak yang dipakai dashboard web TERPISAH:
  - data/monitor/telemetry.json  (snapshot penuh, valid JSON, ada semua seksi)
  - data/monitor/frame.jpg       (JPEG valid)
  - data/monitor/photos.json     (indeks foto)
  - data/monitor/events.jsonl    (log kejadian)
"""

import json
import os
import sys
import tempfile
import types
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from app import monitor_export as me  # noqa: E402
from app import telemetry_snapshot as ts  # noqa: E402


def _cfg(tmpdir):
    return types.SimpleNamespace(
        TEAM_NAME="Aterkia", TEAM_PT="Universitas Diponegoro", TRACK="B",
        TRACK_START={"lat": -6.9, "lon": 110.4, "color": "red"},
        MONITOR_ENABLED=True, MONITOR_DIR=tmpdir, MONITOR_HZ=50.0,
        MONITOR_FRAME=True, MONITOR_FRAME_HZ=50.0, MONITOR_FRAME_WIDTH=320,
        MONITOR_JPEG_QUALITY=70, MONITOR_EVENTS=True,
        # contoh parameter tuning yang harus ikut ter-dump:
        VISION_P_GAIN=0.42, THRUST_VALUE=0.6, _PRIVATE=object(),
    )


PKT = {
    "lat": -6.9, "lon": 110.4, "yaw_deg": 12.0, "pitch_deg": 1.0,
    "roll_deg": -2.0, "state": "NAVIGATING", "target_wp_idx": 2,
    "wp_total": 5, "dist_to_wp_m": 3.5, "groundspeed": 1.2,
    "mavlink_ok": True, "mav_port": "/dev/ttyACM1", "mav_reason": "",
    "gps_fix": True, "telem_source": "REAL", "op_mode": "AUTO",
    "rc_ok": True, "manual_source": "AUTO", "qgc_active": False,
    "gcs_forward": False, "failsafe_active": False, "failsafe_reason": "",
    "pixhawk_check": "[PX4] OK", "avoid_active": False, "avoid_bias": 0.0,
    "status": "ok", "voltage_v": 15.2, "current_a": 3.1, "battery_pct": 88.0,
    "sog_kn": 2.3, "sog_kmh": 4.3, "cog_deg": 12.0,
}


class TestSnapshot(unittest.TestCase):
    def test_params_dump(self):
        cfg = _cfg("/tmp/does-not-matter")
        params = ts.dump_params(cfg)
        self.assertEqual(params["VISION_P_GAIN"], 0.42)
        self.assertEqual(params["THRUST_VALUE"], 0.6)
        self.assertNotIn("_PRIVATE", params)  # privat dibuang

    def test_build_snapshot_sections_and_json(self):
        cfg = _cfg("/tmp/does-not-matter")
        snap = ts.build_snapshot(PKT, cfg, mission={"prep": True},
                                 vision={"models": {}}, camera={"status": "ok"},
                                 last_photo=None, extra={"uptime_s": 1.0})
        for key in ("schema", "ts", "identity", "pose", "motion", "power",
                    "link", "mission", "vision", "camera", "last_photo",
                    "params", "health"):
            self.assertIn(key, snap)
        self.assertEqual(snap["pose"]["lat"], -6.9)
        self.assertEqual(snap["link"]["mavlink_ok"], True)
        self.assertEqual(snap["mission"]["wp_idx"], 2)
        # wajib JSON-serializable
        json.dumps(snap)


class TestExporter(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="asv-monitor-")
        self.cfg = _cfg(self.tmp)
        me.shutdown()
        self.exp = me.MonitorExporter(self.cfg)

    def tearDown(self):
        me.shutdown()

    def _frame(self, val=30):
        img = np.full((240, 320, 3), val, dtype=np.uint8)
        img[100:140, 100:200] = (0, 200, 0)
        return img

    def test_telemetry_written_and_valid(self):
        self.exp.publish(PKT, mission={"prep": True}, vision={"models": {}})
        path = os.path.join(self.tmp, "telemetry.json")
        self.assertTrue(os.path.exists(path))
        with open(path) as f:
            snap = json.load(f)
        self.assertEqual(snap["schema"], ts.SCHEMA)
        self.assertEqual(snap["identity"]["team"], "Aterkia")
        self.assertEqual(snap["mission"]["flags"]["prep"], True)
        self.assertIn("params", snap)
        self.assertFalse(any(n.endswith(".tmp") for n in os.listdir(self.tmp)))

    def test_frame_jpeg_written(self):
        self.exp.publish(PKT, frame=self._frame())
        # beri kesempatan thread penulis frame
        import time
        for _ in range(50):
            if os.path.exists(os.path.join(self.tmp, "frame.jpg")):
                break
            time.sleep(0.02)
        path = os.path.join(self.tmp, "frame.jpg")
        self.assertTrue(os.path.exists(path))
        with open(path, "rb") as f:
            head = f.read(2)
        self.assertEqual(head, b"\xff\xd8")  # magic JPEG

    def test_photo_index_and_event(self):
        info = {"kind": "surface", "path": "data/captures/a.jpg", "ts": 123.0}
        self.exp.register_photo(info)
        with open(os.path.join(self.tmp, "photos.json")) as f:
            photos = json.load(f)
        self.assertEqual(len(photos), 1)
        self.assertEqual(photos[0]["path"], "data/captures/a.jpg")
        with open(os.path.join(self.tmp, "events.jsonl")) as f:
            lines = [json.loads(x) for x in f if x.strip()]
        self.assertTrue(any(r["kind"] == "photo" for r in lines))

    def test_disabled_no_files(self):
        me.shutdown()
        cfg = _cfg(self.tmp)
        cfg.MONITOR_ENABLED = False
        exp = me.MonitorExporter(cfg)
        exp.publish(PKT, frame=self._frame())
        self.assertFalse(os.path.exists(os.path.join(self.tmp, "telemetry.json")))


if __name__ == "__main__":
    unittest.main()
