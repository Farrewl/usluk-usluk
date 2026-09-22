#!/usr/bin/env bash
# Jalankan relay web gateway (Redis -> WebSocket dashboard).
# Opsional: hanya diperlukan kalau dashboard web dipakai.
set -euo pipefail
cd "$(dirname "$0")/.."

PY="${PYTHON:-python3}"
if [ -d .venv ]; then
  source .venv/bin/activate
fi

exec "$PY" -m app.gateway