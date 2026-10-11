#!/usr/bin/env bash
# Jalankan GUI dashboard ASV (Linux / Raspberry Pi).
# Pakai .venv kalau ada; kalau belum, jalankan dulu:  ./scripts/setup_minimal.sh
set -euo pipefail
cd "$(dirname "$0")/.."

if [ -x .venv/bin/python ]; then
  exec .venv/bin/python -O -u main.py "$@"
fi

echo "[i] .venv belum ada — jalankan:  ./scripts/setup_minimal.sh" >&2
exec "${PYTHON:-python3}" -O -u main.py "$@"
