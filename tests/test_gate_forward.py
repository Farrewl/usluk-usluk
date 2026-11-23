"""tests/test_gate_forward.py — Unit test sederhana untuk gate_forward."""
import unittest
from app.gate_forward import check_and_forward


class TestGateForward(unittest.TestCase):
    def test_both_buoys_detected(self):
        dets = {
            1: [{"cx": 200.0, "cy": 240.0, "area": 100}], # Merah di kiri
            0: [{"cx": 440.0, "cy": 240.0, "area": 100}]  # Hijau di kanan (tengah = 320)
        }
        surge, yaw, note = check_and_forward(dets, frame_width=640)
        self.assertEqual(surge, 0.6)
        self.assertAlmostEqual(yaw, 0.0)
        self.assertIn("GATE DETECTED", note)

    def test_missing_one_buoy(self):
        dets = {
            1: [{"cx": 200.0, "cy": 240.0, "area": 100}]
            # Hijau kosong
        }
        surge, yaw, note = check_and_forward(dets, frame_width=640)
        self.assertEqual(surge, 0.0)
        self.assertEqual(yaw, 0.0)
        self.assertEqual(note, "No gate pair")


if __name__ == "__main__":
    unittest.main()
