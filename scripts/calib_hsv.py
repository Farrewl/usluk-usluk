#!/usr/bin/env python3
"""scripts/calib_hsv.py — Kalibrasi ambang HSV dari foto buoy asli.

Pemakaian:
  .venv/bin/python scripts/calib_hsv.py data/calib/

Membaca SEMUA gambar di folder (nama bebas, tapi disarankan mengandung
`hijau`/`merah` + `jauh`/`sedang`/`dekat`/`remang`/`terang`), lalu untuk
tiap foto melaporkan:
  - mean value (kecerahan 0..255) & saturasi global,
  - distribusi hue di area tengah (tempat buoy biasanya berada),
  - % piksel yang LOLOS gerbang HSV saat ini vs alternatif longgar.

Output diakhiri REKOMENDASI ambang baru (siap salin ke tuning_params.json).

Tidak butuh YOLO/kamera/Pixhawk — murni OpenCV + numpy.
"""

import glob
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

try:
    import cv2
    import numpy as np
except ImportError as e:
    print(f"Butuh opencv + numpy di .venv: {e}")
    sys.exit(1)

from app.detection_validation import (
    RED_HUE_MAX, RED_HUE_MIN2, GREEN_HUE_MIN, GREEN_HUE_MAX, MIN_VALUE,
)


def _stats(path):
    img = cv2.imread(path)
    if img is None:
        return None
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    h, w = hue.shape
    # Area tengah (buoy biasanya di tengah frame saat foto kalibrasi).
    cy, cx = h // 2, w // 2
    crop = (slice(cy - h // 4, cy + h // 4), slice(cx - w // 4, cx + w // 4))
    ch, cs, cv_ = hue[crop], sat[crop], val[crop]
    out = {
        "file": os.path.basename(path),
        "mean_v": float(val.mean()),
        "mean_s": float(sat.mean()) / 255.0,
        "center_v": float(cv_.mean()),
        "center_s": float(cs.mean()) / 255.0,
    }
    # % piksel tengah yang lolos tiap varian gerbang.
    for tag, gmin, gmax in (("hijau_35-85", 35, 85),
                            ("hijau_25-95", 25, 95),
                            ("merah", None, None)):
        if tag == "merah":
            hue_ok = (ch <= RED_HUE_MAX) | (ch >= RED_HUE_MIN2)
        else:
            hue_ok = (ch >= gmin) & (ch <= gmax)
        for smin, vmin in ((0.35, 40), (0.20, 20)):
            mask = hue_ok & (cs >= int(smin * 255)) & (cv_ >= vmin)
            out[f"{tag}_S{smin}_V{vmin}"] = float(mask.mean()) * 100.0
    return out


def main(folder):
    paths = sorted(sum((glob.glob(os.path.join(folder, e))
                        for e in ("*.jpg", "*.jpeg", "*.png")), []))
    if not paths:
        print(f"Tidak ada gambar di {folder}")
        sys.exit(1)
    rows = [r for r in (_stats(p) for p in paths) if r]
    print(f"{'file':34s} {'Vglob':>6s} {'Sglob':>6s} "
          f"{'H35-85 ketat':>12s} {'H35-85 longgar':>14s} "
          f"{'H25-95 longgar':>14s} {'merah longgar':>13s}")
    for r in rows:
        print(f"{r['file']:34s} {r['mean_v']:6.0f} {r['mean_s']:6.2f} "
              f"{r['hijau_35-85_S0.35_V40']:11.1f}% "
              f"{r['hijau_35-85_S0.2_V20']:13.1f}% "
              f"{r['hijau_25-95_S0.2_V20']:13.1f}% "
              f"{r['merah_S0.2_V20']:12.1f}%")
    # Rekomendasi otomatis dari foto paling gelap.
    gelap = min(rows, key=lambda r: r["mean_v"])
    print("\n--- REKOMENDASI ---")
    print(f"Foto tergelap: {gelap['file']} (V global {gelap['mean_v']:.0f})")
    if gelap["mean_v"] < 80:
        print("Frame GELAP -> P4-A aktif saat lomba sore/remang. Saran:")
        print('  "BUOY_ADAPTIVE_ENABLED": true,')
        print('  "BUOY_BRIGHTNESS_THRESHOLD": 80,')
        hijau = gelap["hijau_25-95_S0.2_V20"]
        print(f"  Hijau H25-95 longgar lolos {hijau:.1f}% "
              f"({'CUKUP' if hijau > 15 else 'KURANG — perlu retrain/foto ulang'})")
    else:
        print("Semua foto TERANG (V >= 80) -> jalur normal cukup; "
              "uji lagi dengan foto remang asli bila lomba sore hari.")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "data/calib")
