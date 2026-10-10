"""scripts/verify_env.py — verifikasi .venv ASV (dipakai Linux & Windows).

Dipanggil di akhir setup_minimal.sh / setup_minimal.ps1. Keluar dengan
status non-zero bila ada yang salah (torch varian CUDA, import gagal).

Kenapa file .py terpisah (bukan here-string PowerShell / heredoc bash):
here-string `@'...'@ | python -` ditolak Windows PowerShell 5.1 (penutup
harus sendirian sebaris) sehingga setup gagal parse. File .py = satu
sumber, bebas masalah quoting/encoding shell.

Catatan:
  - Cek torch: `torch.version.cuda is None` (wheel CPU-only).
  - YOLO dilewati bila weights/buoy.pt belum ada (dipanggil sebelum
    download_weights).
"""

import numpy as np
import torch

# Wajib varian CPU-only (dari index PyTorch). Kalau bukan -> salah resep.
assert torch.version.cuda is None, (
    f"KECETOT varian CUDA: {torch.__version__} (harus wheel +cpu)")

import cv2  # noqa: E402
import serial  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402,F401
from pymavlink import mavutil  # noqa: E402,F401

print(f"torch {torch.__version__} | cv2 {cv2.__version__} | import OK")

try:
    from ultralytics import YOLO
    r = YOLO("weights/buoy.pt").predict(
        np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
    assert r[0].plot().shape == (480, 640, 3)
    print("YOLO infer OK")
except FileNotFoundError:
    print("YOLO dilewati (weights belum ada)")
