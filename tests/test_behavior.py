"""tests/test_behavior.py — perilaku navigasi tanpa waypoint (GATE -> BOX -> DOCK)."""
import unittest

from app.behavior import decide, GATE, BOX, DOCK, pair_gate


class _Cfg:
    RED_BALL_CLASS_ID = 1
    GREEN_BALL_CLASS_ID = 0
    GREEN_BOX_CLASS_ID = 0
    BLUE_BOX_CLASS_ID = 0
    MIN_BUOY_AREA_PX = 16
    SEARCH_THRUST = 0.5
    THRUST_VALUE = 0.8
    ALIGN_THRUST = 0.3
    BLUE_BOX_ALIGN_THRUST = 0.3


class TestBehavior(unittest.TestCase):
    def setUp(self):
        self.cfg = _Cfg()
        self.fw, self.fh = 640, 480

    def test_pair_gate_none_when_empty(self):
        self.assertIsNone(pair_gate([], [], self.fh))

    def test_gate_search_when_no_buoys(self):
        x, t, note = decide(GATE, {}, self.fw, self.fh, self.cfg)
        self.assertIsNone(x)
        self.assertEqual(t, 0.0)
        self.assertEqual(note, "GATE search")

    def test_gate_single_buoy_turns(self):
        det = {1: [{"cx": 320.0, "cy": 240.0, "area": 500}]}
        x, t, note = decide(GATE, det, self.fw, self.fh, self.cfg)
        self.assertIsNone(x)
        self.assertEqual(t, self.cfg.SEARCH_THRUST * 0.5)
        self.assertEqual(note, "GATE single-buoy")

    def test_gate_pair_far_thrusts_search(self):
        det = {1: [{"cx": 100.0, "cy": 240.0, "area": 500}],
               0: [{"cx": 400.0, "cy": 240.0, "area": 500}]}
        x, t, note = decide(GATE, det, self.fw, self.fh, self.cfg)
        self.assertAlmostEqual(x, 250.0)
        self.assertEqual(t, self.cfg.SEARCH_THRUST)
        self.assertEqual(note, "GATE far")

    def test_gate_pair_near_thrusts_full(self):
        det = {1: [{"cx": 300.0, "cy": 240.0, "area": 900}],
               0: [{"cx": 340.0, "cy": 240.0, "area": 900}]}
        x, t, note = decide(GATE, det, self.fw, self.fh, self.cfg)
        self.assertEqual(t, self.cfg.THRUST_VALUE)
        self.assertEqual(note, "GATE pass")

    def test_box_arrived_stops(self):
        det = {0: [{"cx": 320.0, "cy": 240.0, "area": 0.3 * self.fw * self.fh}]}
        x, t, note = decide(BOX, det, self.fw, self.fh, self.cfg)
        self.assertEqual(t, 0.0)
        self.assertEqual(note, "BOX arrived")

    def test_box_align_thrusts(self):
        det = {0: [{"cx": 320.0, "cy": 240.0, "area": 100}]}
        x, t, note = decide(BOX, det, self.fw, self.fh, self.cfg)
        self.assertEqual(t, self.cfg.ALIGN_THRUST)
        self.assertEqual(note, "BOX align")

    def test_dock_arrived_stops(self):
        det = {0: [{"cx": 320.0, "cy": 240.0, "area": 0.4 * self.fw * self.fh}]}
        x, t, note = decide(DOCK, det, self.fw, self.fh, self.cfg)
        self.assertEqual(t, 0.0)
        self.assertEqual(note, "DOCK arrived")


if __name__ == "__main__":
    unittest.main()
