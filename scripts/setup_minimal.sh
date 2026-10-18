#!/usr/bin/env bash
# ============================================================
# scripts/setup_minimal.sh — install .venv minimal ASV (Linux).
#
# Hasil: .venv RINGAN (tanpa torch) dengan seluruh fungsi produksi:
# GUI PyQt, YOLO 4 model (ONNX via cv2.dnn), MAVLink.
#
# Pakai:
#   ./scripts/setup_minimal.sh            # idempoten: aman dijalankan ulang
#
# IDEMPOTEN (aman diulang):
#   - .venv yang sudah ada DIPAKAI ULANG (tak ditimpa/dihapus).
#   - Bila paket inti sudah bisa di-import, langkah install dilewati.
#   - Langkah verifikasi aman diulang.
#   Mau paksa bersih? hapus dulu:  rm -rf .venv
#
# KUNCI (build 2026-10-08):
#  - Inferensi YOLO lewat ONNX + cv2.dnn (app/yolo_onnx.py) -> TIDAK perlu
#    torch/torchvision/ultralytics. Ini memangkas RAM (~537->~217 MB),
#    waktu start, dan ukuran .venv secara besar.
#  - Jangan ganti opencv headless -> opencv-python biasa (duplikat Qt).
# ============================================================
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"
FRESH=0

# --- [core] Build library C (libaterkia.so) bila belum ada ---
if [ -f core/libaterkia.so ]; then
  echo "[i] core/libaterkia.so sudah ada — build C dilewati."
else
  echo "[core] Build inti C (butuh gcc + make)..."
  make -C core linux
fi

# --- [1/3] Virtualenv (reuse kalau sudah ada) ---
if [ -x .venv/bin/python ]; then
  echo "[i] .venv sudah ada — dipakai ulang (tak ditimpa)."
else
  echo "[1/3] Bikin virtualenv..."
  "$PY" -m venv .venv
  FRESH=1
fi

if [ "$FRESH" = 1 ]; then
  .venv/bin/python -m pip install -q --upgrade pip
fi

# --- [2/3] Dependensi (lewati bila sudah lengkap) ---
if .venv/bin/python -c "import cv2, serial, pymavlink; from PyQt5 import QtWidgets" 2>/dev/null; then
  echo "[i] Dependensi sudah terpasang — langkah install dilewati."
else
  echo "[2/3] Install resep minimal (tanpa torch)..."
  .venv/bin/pip install -q -r requirements.txt
fi

# --- [3/3] Verifikasi (scripts/verify_env.py) ---
echo "[3/3] Verifikasi import inti + inferensi YOLO (ONNX)..."
.venv/bin/python scripts/verify_env.py

echo
echo "Selesai. Total: $(du -sh .venv | cut -f1)"
echo "Jalankan GUI :  ./scripts/run_gui.sh"
echo "Ekspor monitor: data/monitor/  (lihat README.md)  cek: scripts/monitor_read.py"
