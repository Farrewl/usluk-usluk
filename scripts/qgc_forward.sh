#!/usr/bin/env bash
# scripts/qgc_forward.sh — Jalankan dashboard + forward MAVLink ke QGroundControl.
#
# Arsitektur (Pixhawk 6C hanya 1 link serial ke NUC):
#   Pixhawk --USB--> NUC (main.py, master otonomi) --udp--> QGC laptop
# QGC di laptop lapangan: monitor + Arm/Mode/E-stop. Otonomi tetap NUC.
#
# Pakai:
#   GCS_IP=192.168.1.50 ./scripts/qgc_forward.sh   # QGC di laptop lain
#   ./scripts/qgc_forward.sh                        # QGC di NUC yang sama
# Di QGC laptop: biarkan koneksi UDP default (port 14550) — otomatis nyambung.

set -euo pipefail
cd "$(dirname "$0")/.."

export GCS_FORWARD=1
export GCS_IP="${GCS_IP:-127.0.0.1}"
export GCS_PORT="${GCS_PORT:-14550}"

echo "[QGC] Forward MAVLink -> udp:${GCS_IP}:${GCS_PORT}"
echo "[QGC] Buka QGroundControl di laptop, tunggu status Connected."
echo "[QGC] Arm/mode dari QGC; otonomi dari NUC (KILL > MANUAL > AUTO)."

if [ -x .venv/bin/python ]; then
    exec .venv/bin/python -u main.py
else
    exec python3 -u main.py
fi
