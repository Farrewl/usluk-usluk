#!/usr/bin/env bash
# Jalankan GUI dashboard ASV (Linux / Raspberry Pi).
# Gunakan virtualenv kalau belum ada:  python3 -m venv .venv
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"
if [ -d .venv ]; then
  source .venv/bin/activate
fi

exec "$PY" main.py