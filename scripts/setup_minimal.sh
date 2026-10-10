#!/usr/bin/env bash
# ============================================================
# scripts/setup_minimal.sh — install .venv minimal ASV (Opsi B).
#
# Hasil: .venv ±1,3 GB (vs ±2,2 GB resep lama) dengan seluruh fungsi
# produksi jalan: GUI PyQt, YOLO 4 model (.pt), MAVLink.
#
# Pakai:
#   ./scripts/setup_minimal.sh
#   .venv/bin/python -m unittest discover -s tests    # verifikasi
#
# KUNCI penghematan (3 pelajaran dari build 2026-10-08):
#  1. torch WAJIB dari index CPU — dari PyPI default ia menarik
#     CUDA+nvidia-*+triton (≈3,6 GB membengkak).
#  2. ultralytics dipasang --no-deps — deps bawaannya menarik
#     matplotlib+polars+opencv-python non-headless (≈600 MB) yang
#     TIDAK PERNAH di-import eager saat inferensi (sudah dibuktikan).
#     Deps runtime aslinya semua tercantum di requirements.txt.
#  3. Folder test/include/bin-test di dalam torch (≈200 MB) aman
#     dibuang — runtime YOLO tidak memakainya.
# ============================================================
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"

if [ -d .venv ]; then
  echo "[!] .venv sudah ada ($(du -sh .venv | cut -f1))."
  read -rp "    Timpa dengan .venv minimal? [y/N] " jawab
  case "$jawab" in
    y|Y) rm -rf .venv ;;
    *) echo "Batal."; exit 1 ;;
  esac
fi

echo "[1/5] Bikin virtualenv..."
"$PY" -m venv .venv
.venv/bin/pip install -q --upgrade pip

echo "[2/5] Install torch+torchvision CPU-ONLY (index PyTorch, bukan PyPI!)..."
.venv/bin/pip install -q --index-url https://download.pytorch.org/whl/cpu \
    torch==2.14.0+cpu torchvision==0.29.0+cpu

echo "[3/5] Install resep minimal + ultralytics tanpa deps borosnya..."
.venv/bin/pip install -q -r requirements.txt
.venv/bin/pip install -q --no-deps ultralytics==8.4.160

echo "[4/5] Pangkas folder test/include/bin-test torch (±200 MB)..."
SP=.venv/lib/python3.12/site-packages
rm -rf "$SP/torch/test" "$SP/torch/include"
rm -f "$SP/torch/lib/libjitbackend_test.so" "$SP/torch/lib/libtorchbind_test.so"
find "$SP/torch/bin" -mindepth 1 \
     ! -name torch_shm_manager \
     ! -name upgrader_models \
     ! -name script_module_v4.ptl \
     -delete

echo "[5/5] Verifikasi import inti + inferensi YOLO..."
.venv/bin/python - <<'EOF'
import numpy as np
import torch
assert torch.__version__.endswith("+cpu"), f"KECETOT varian non-CPU: {torch.__version__}"
import cv2, serial
from PyQt5.QtWidgets import QApplication
from ultralytics import YOLO
from pymavlink import mavutil
r = YOLO("weights/buoy.pt").predict(
    np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
assert r[0].plot().shape == (480, 640, 3)
print(f"torch {torch.__version__} | cv2 {cv2.__version__} | YOLO infer OK")
EOF

echo
echo "Selesai. Total: $(du -sh .venv | cut -f1)"
echo "Jalankan program:  .venv/bin/python main.py"
echo "Jalankan tes:      .venv/bin/python -m unittest discover -s tests"
