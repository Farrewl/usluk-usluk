"""scripts/verify_env.py — verifikasi .venv ASV (Linux).

Dipanggil di akhir setup_minimal.sh. Keluar dengan
status non-zero bila ada yang salah (import gagal / ONNX tak bisa dibaca).

Catatan:
  - Inferensi YOLO via ONNX + cv2.dnn (tanpa torch/ultralytics) —
    lihat app/yolo_onnx.py. Verifikasi di sini cukup: cv2 punya dnn +
    model .onnx bisa dimuat & dijalankan.
  - Cek ONNX dilewati bila weights/*.onnx belum ada.
"""

import os

import cv2  # noqa: E402
import serial  # noqa: E402,F401
from PyQt5.QtWidgets import QApplication  # noqa: E402,F401
from pymavlink import mavutil  # noqa: E402,F401
import numpy as np  # noqa: E402

assert hasattr(cv2, "dnn") and hasattr(cv2.dnn, "readNetFromONNX"), (
    "cv2 tanpa modul dnn — pakai opencv-python-headless")

print(f"cv2 {cv2.__version__} (dnn OK) | import OK")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
onnx_path = os.path.join(ROOT, "weights", "buoy.onnx")
if not os.path.exists(onnx_path):
    print("ONNX dilewati (weights/buoy.onnx belum ada)")
else:
    net = cv2.dnn.readNetFromONNX(onnx_path)
    blob = cv2.dnn.blobFromImage(
        np.zeros((192, 256, 3), dtype=np.uint8), 1 / 255.0, (256, 256),
        swapRB=True)
    net.setInput(blob)
    out = net.forward()
    assert out is not None and out.size > 0, "ONNX forward gagal"
    print(f"ONNX infer OK (out {tuple(out.shape)})")
