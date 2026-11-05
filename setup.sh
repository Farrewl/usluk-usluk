#!/usr/bin/env bash
# ============================================================
# setup.sh — Setup lingkungan ASV Aterkia untuk Linux (Fresh Clone)
# ============================================================
set -euo pipefail
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
echo "[SETUP] Memulai setup Aterkia ASV..."

# 1. Build library C inti (libaterkia.so)
if [ -f core/libaterkia.so ]; then
  echo "[i] core/libaterkia.so sudah ada."
else
  echo "[core] Kompilasi libaterkia.so..."
  make -C core linux
fi

# 2. Buat virtual environment python
if [ -x .venv/bin/python ]; then
  echo "[i] .venv sudah ada."
else
  echo "[1/2] Membuat virtualenv (.venv)..."
  "$PY" -m venv .venv
  .venv/bin/python -m pip install -q --upgrade pip
fi

# 3. Install dependensi
if .venv/bin/python -c "import cv2, serial, pymavlink; from PyQt5 import QtWidgets" 2>/dev/null; then
  echo "[i] Dependensi Python sudah terpasang."
else
  echo "[2/2] Menginstal requirements.txt..."
  .venv/bin/pip install -r requirements.txt
fi

# 4. Buat folder runtime jika belum ada
mkdir -p data/monitor data/captures data/waypoint_captures data/session_video

echo "[OK] Setup selesai. Jalankan GUI dengan: .venv/bin/python main.py"
