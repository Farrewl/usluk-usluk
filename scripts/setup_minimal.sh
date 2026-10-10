#!/usr/bin/env bash
# ============================================================
# scripts/setup_minimal.sh — install .venv minimal ASV (Linux).
#
# Padanan scripts/setup_minimal.ps1 (urut & kunci penghematan SAMA).
# Hasil: .venv ±1,3 GB dengan seluruh fungsi produksi jalan:
# GUI PyQt, YOLO 4 model (.pt), MAVLink.
#
# Pakai:
#   ./scripts/setup_minimal.sh            # idempoten: aman dijalankan ulang
#   .venv/bin/python -m unittest discover -s tests    # verifikasi
#
# IDEMPOTEN (aman diulang):
#   - .venv yang sudah ada DIPAKAI ULANG (tak ditimpa/dihapus).
#   - Bila paket inti sudah bisa di-import, langkah install dilewati.
#   - Langkah pangkas & verifikasi aman diulang.
#   Mau paksa bersih? hapus dulu:  rm -rf .venv
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
FRESH=0

# --- [1/5] Virtualenv (reuse kalau sudah ada) ---
if [ -x .venv/bin/python ]; then
  echo "[i] .venv sudah ada — dipakai ulang (tak ditimpa)."
else
  echo "[1/5] Bikin virtualenv..."
  "$PY" -m venv .venv
  FRESH=1
fi

if [ "$FRESH" = 1 ]; then
  .venv/bin/python -m pip install -q --upgrade pip
fi

# --- [2-3/5] Dependensi (lewati bila sudah lengkap) ---
if .venv/bin/python -c "import torch, torchvision, cv2, serial, pymavlink, ultralytics; from PyQt5 import QtWidgets" 2>/dev/null; then
  echo "[i] Dependensi sudah terpasang — langkah install (2-3) dilewati."
else
  echo "[2/5] Install torch+torchvision CPU-ONLY (index PyTorch, bukan PyPI!)..."
  .venv/bin/pip install -q --index-url https://download.pytorch.org/whl/cpu \
      torch==2.14.0+cpu torchvision==0.29.0+cpu

  echo "[3/5] Install resep minimal + ultralytics tanpa deps borosnya..."
  .venv/bin/pip install -q -r requirements.txt
  .venv/bin/pip install -q --no-deps ultralytics==8.4.160
fi

# --- [4/5] Pangkas folder test/include/bin-test torch (idempoten) ---
echo "[4/5] Pangkas folder test/include/bin-test torch (±200 MB)..."
SP="$(.venv/bin/python -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
rm -rf "$SP/torch/test" "$SP/torch/include"
rm -f "$SP/torch/lib/libjitbackend_test.so" "$SP/torch/lib/libtorchbind_test.so"
if [ -d "$SP/torch/bin" ]; then
  find "$SP/torch/bin" -mindepth 1 \
       ! -name torch_shm_manager \
       ! -name upgrader_models \
       ! -name script_module_v4.ptl \
       -delete
fi

# --- [5/5] Verifikasi (satu sumber dgn Windows: scripts/verify_env.py) ---
echo "[5/5] Verifikasi import inti + inferensi YOLO..."
.venv/bin/python scripts/verify_env.py

echo
echo "Selesai. Total: $(du -sh .venv | cut -f1)"
echo "Jalankan program:  .venv/bin/python main.py"
echo "Jalankan tes:      .venv/bin/python -m unittest discover -s tests"
