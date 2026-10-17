#!/usr/bin/env python3
"""
tests/test_core_yolo_onnx.py — Unit test inferensi YOLO via ONNX + cv2.dnn.

Mengunci kontrak jalur baru (tanpa torch/ultralytics):
  - letterbox `auto=True` (rect, stride 32) sesuai ultralytics predict:
    gambar 640x480 + imgsz 256 -> input 192x256 (tanpa pad persegi).
  - OnnxDetector.model(frame, conf=, imgsz=) mengembalikan [res] dengan
    res.boxes (iterable: .cls/.xyxy/.conf list) dan res.names {int: str}.
  - NMS helper memfilter kotak tumpang tindih.

Dilewati otomatis bila cv2 tanpa dnn atau weights/*.onnx belum ada.
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

import cv2  # noqa: E402

from app import yolo_onnx as yo  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ONNX_BUOY = os.path.join(ROOT, "weights", "buoy.onnx")
HAS_DNN = hasattr(cv2, "dnn") and hasattr(cv2.dnn, "readNetFromONNX")


class TestLetterbox(unittest.TestCase):
    def test_rect_no_square_pad(self):
        """640x480 (landscape) + 256 -> (192, 256), bukan 256x256."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        out, ratio, (pl, pt) = yo._letterbox(img, (256, 256))
        self.assertEqual(out.shape[:2], (192, 256))
        self.assertAlmostEqual(ratio, 0.4, places=6)
        self.assertEqual((pl, pt), (0, 0))

    def test_square_image(self):
        img = np.zeros((256, 256, 3), dtype=np.uint8)
        out, ratio, _ = yo._letterbox(img, (256, 256))
        self.assertEqual(out.shape[:2], (256, 256))
        self.assertAlmostEqual(ratio, 1.0, places=6)

    def test_letterbox_dims_multiple_of_stride(self):
        img = np.zeros((300, 700, 3), dtype=np.uint8)
        out, _, _ = yo._letterbox(img, (256, 256))
        h, w = out.shape[:2]
        self.assertEqual(h % 32, 0)
        self.assertEqual(w % 32, 0)


class TestNms(unittest.TestCase):
    def test_suppress_overlap(self):
        boxes = [(0, 0, 10, 10), (1, 1, 11, 11), (100, 100, 110, 110)]
        scores = [0.9, 0.8, 0.7]
        kept = yo._nms(boxes, scores, 0.25, 0.5)
        self.assertIn(0, kept)      # skor tertinggi lolos
        self.assertNotIn(1, kept)   # tumpang tindih -> dibuang
        self.assertIn(2, kept)      # jauh -> lolos


@unittest.skipUnless(HAS_DNN, "cv2 tanpa modul dnn")
@unittest.skipUnless(os.path.exists(ONNX_BUOY), "weights/buoy.onnx tidak ada")
class TestOnnxDetector(unittest.TestCase):
    def test_contract_blank_frame(self):
        det = yo.OnnxDetector(ONNX_BUOY, {0: "green_ball", 1: "red_ball"})
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        res = det(frame, conf=0.25, imgsz=256)
        self.assertEqual(len(res), 1)
        self.assertEqual(res[0].names, {0: "green_ball", 1: "red_ball"})
        self.assertEqual(len(res[0].boxes), 0)
        for b in res[0].boxes:  # tidak seharusnya ada
            self.assertTrue(len(b.cls[0]) >= 0)

    def test_names_from_settings(self):
        from app import settings as cfg
        self.assertIn("buoy.onnx", cfg.YOLO_NAMES)
        self.assertEqual(cfg.YOLO_NAMES["buoy.onnx"][1], "red_ball")


if __name__ == "__main__":
    unittest.main()
