#!/usr/bin/env bash
# Unduh model YOLO ke folder weights/.
#
# Model (~6 MB/file, total ±25 MB untuk 4 model aktif) sengaja TIDAK
# dimasukkan ke git. Simpan di satu tempat bersama tim, lalu jalankan:
#
#   WEIGHTS_SOURCE=/path/ke/weights.tar.gz  ./scripts/download_weights.sh
#   WEIGHTS_SOURCE=/mnt/usb/weights         ./scripts/download_weights.sh
#
# Atau ekspor langsung variabel di shell sebelum memanggil script.
set -euo pipefail
cd "$(dirname "$0")/.."

SRC="${WEIGHTS_SOURCE:-}"
NEEDED=(buoy.pt box_hijau.pt best_blue_dark.pt best_red_new.pt)

if [ -z "$SRC" ]; then
  echo "Setelah WEIGHTS_SOURCE diisi, file yang dibutuhkan:"
  for f in "${NEEDED[@]}"; do echo "  - weights/$f"; done
  echo
  echo "  WEIGHTS_SOURCE=/path/ke/sumber  ./scripts/download_weights.sh"
  exit 1
fi

mkdir -p weights
if [ -d "$SRC" ]; then
  for f in "${NEEDED[@]}"; do
    if [ -f "$SRC/$f" ]; then
      cp "$SRC/$f" "weights/$f"
      echo "  + weights/$f"
    else
      echo "  ! $f tidak ada di sumber"
    fi
  done
elif [ -f "$SRC" ]; then
  tar -xzf "$SRC" -C weights --strip-components=1
  echo "  + diekstrak dari archive"
fi

echo "Selesai. Model aktif tersimpan di weights/."