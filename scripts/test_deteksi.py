#!/usr/bin/env python3
"""
scripts/test_deteksi.py — Uji cepat pipeline deteksi YOLO tanpa GUI.

Pemakaian:
  scripts/test_deteksi.py                 # model buoy.pt, kamera kerja otomatis
  scripts/test_deteksi.py --model weights/buoy.pt --image foto.jpg
  scripts/test_deteksi.py --imgsz 640 --conf 0.3 --frames 30

Yang dilakukan: buka kamera lewat app/camera.py (fallback index 0 bila
CAMERA_INDEX tidak bisa dibuka — jalur yang sama dengan GUI), infer dengan
model (ultralytics YOLO), cetak class+confidence+box per objek, lalu
simpan frame ber-annotasi ke data/deteksi_uji.jpg.

Filter pasca-YOLO (hanya aktif untuk model buoy — nama file mengandung
"buoy"): deteksi class 0/1 yang TIDAK lolos kualifikasi warna/bentuk
buoy (mis. wajah operator) dicetak dengan penanda `*ditolak` dan tidak
digambar pada frame annotasi — persis perilaku navigator/simulator.

Catatan: tidak memakai ultralytics `predict(source=<int>)` karena ia
membuka kamera sendiri tanpa fallback — di mesin ini /dev/video1 tidak
bisa dibuka, jadi jalannya BERHENTI menunggu stream. Kami pakai
capture manual + model(frame) persis seperti navigator.
"""

import argparse
import os
import sys
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from ultralytics import YOLO  # noqa: E402

from app.camera import open_camera  # noqa: E402
from app.detection_validation import (  # noqa: E402
    validate_buoy, draw_validated_boxes,
)
from app import settings as cfg  # noqa: E402

WEIGHTS_DIR = os.path.join(ROOT, "weights")
DEFAULT_MODEL = os.path.join(WEIGHTS_DIR, "buoy.pt")
SAVE_PATH = os.path.join(ROOT, "data", "deteksi_uji.jpg")


def _is_buoy_model(model_path):
    """Validasi warna/bentuk hanya untuk model buoy (bukan box/dock)."""
    return "buoy" in os.path.basename(model_path).lower()


def _print_box(prefix, r, cls, conf, xyxy, rejected=False):
    flag = " *ditolak" if rejected else ""
    print(f"  {prefix} class={r.names[cls]}({cls}) conf={conf:.3f} "
          f"box={[round(v, 1) for v in xyxy]}{flag}")


def _filter_boxes(frame, r, cfgobj, use_validation):
    """Kembali (kept, total, ditolak). kept = list (cls, conf, xyxy)."""
    boxes = r.boxes
    if boxes is None:
        return [], 0, 0
    kept, total, rejected = [], 0, 0
    for j, b in enumerate(boxes):
        cls = int(b.cls[0])
        conf = float(b.conf[0])
        xyxy = b.xyxy[0].tolist()
        total += 1
        if use_validation and cls in (cfgobj.RED_BALL_CLASS_ID,
                                      cfgobj.GREEN_BALL_CLASS_ID):
            lolos = validate_buoy(
                frame, cls, xyxy,
                min_area=cfgobj.MIN_BUOY_AREA_PX,
                min_color_fraction=cfgobj.BUOY_MIN_COLOR_FRACTION,
                min_saturation=cfgobj.BUOY_MIN_SATURATION,
                max_aspect_deviation=cfgobj.BUOY_MAX_ASPECT_DEVIATION)
            if not lolos:
                rejected += 1
                _print_box(f"[{j}]", r, cls, conf, xyxy, rejected=True)
                continue
        kept.append((cls, conf, xyxy))
        _print_box(f"[{j}]", r, cls, conf, xyxy)
    return kept, total, rejected


def _save_annotated(frame, kept, names):
    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
    annotated = (draw_validated_boxes(frame, kept, names) if kept
                 else frame)
    import cv2
    cv2.imwrite(SAVE_PATH, annotated)
    print(f"[DET] Frame ber-annotasi disimpan: {SAVE_PATH}")


def main():
    ap = argparse.ArgumentParser(description="Uji deteksi YOLO (buoy/box) tanpa GUI.")
    ap.add_argument("--model", default=DEFAULT_MODEL, help="path model .pt")
    ap.add_argument("--image", default=None,
                    help="path gambar; default = kamera (fallback otomatis)")
    ap.add_argument("--imgsz", type=int, default=320,
                    help="resolusi inferensi (320 kencang, 640 presisi)")
    ap.add_argument("--conf", type=float, default=0.25,
                    help="ambang confidence 0..1")
    ap.add_argument("--frames", type=int, default=1,
                    help="jumlah frame kamera (1 = cepat; lebih = stabilitas)")
    args = ap.parse_args()

    if not os.path.exists(args.model):
        print(f"ERROR: model tidak ada: {args.model}")
        print("Pulihkan dari git, contoh:")
        print("  git show 43f932b:modules/weights/buoy.pt > weights/buoy.pt")
        return 1

    cfgobj = cfg.Config()
    use_validation = _is_buoy_model(args.model)
    if use_validation:
        # Baris atas menunjukkan nilai filter buoy yang sedang dipakai.
        print(f"[DET] Filter buoy AKTIF (conf>={cfgobj.BUOY_CONF_THRESHOLD}, "
              f"warna>={cfgobj.BUOY_MIN_COLOR_FRACTION}, "
              f"sat>={cfgobj.BUOY_MIN_SATURATION}, "
              f"aspect<=1+{cfgobj.BUOY_MAX_ASPECT_DEVIATION}, "
              f"area>={cfgobj.MIN_BUOY_AREA_PX}px).")

    print(f"[DET] Memuat model: {args.model}")
    model = YOLO(args.model, task="detect")

    if args.image:
        print(f"[DET] Inferensi pada gambar: {args.image} (imgsz={args.imgsz})")
        results = model.predict(source=args.image, imgsz=args.imgsz,
                                conf=args.conf, verbose=False)
        if not results:
            print("[DET] Tidak ada hasil.")
            return 1
        r = results[0]
        kept, total, rejected = _filter_boxes(r.orig_img, r, cfgobj,
                                              use_validation)
        print(f"[DET] Objek terdeteksi: {total} "
              f"(ditolak filter: {rejected}, lolos: {len(kept)})")
        _save_annotated(r.orig_img, kept, r.names)
        return 0

    # --- jalur kamera: sama seperti GUI (app/camera.py) ---
    print("[DET] Membuka kamera (fallback index otomatis)...")
    cap = open_camera(0, target_fps=cfgobj.CAMERA_TARGET_FPS,
                      auto_highest=True)
    if cap is None or not cap.isOpened():
        print("[DET] ERROR: tidak ada kamera yang bisa dibuka.")
        return 1

    total_det = 0
    total_rej = 0
    try:
        for i in range(args.frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                print(f"[DET] Frame {i} gagal dibaca, berhenti.")
                break
            t0 = time.time()
            results = model(frame, imgsz=args.imgsz, conf=args.conf,
                            verbose=False)
            dt_ms = (time.time() - t0) * 1000.0
            r = results[0]
            kept, total, rejected = _filter_boxes(frame, r, cfgobj,
                                                  use_validation)
            total_det += total
            total_rej += rejected
            print(f"[DET] frame {i}: {total} objek "
                  f"(infer {dt_ms:.0f} ms, ditolak {rejected})")
            if kept:
                _save_annotated(frame, kept, r.names)
    finally:
        cap.release()

    print(f"[DET] Selesai — total {total_det} objek terdeteksi, "
          f"{total_rej} ditolak filter buoy.")
    if total_det == 0:
        print("[DET] (0 objek = normal bila tidak ada buoy di depan kamera.)")
    return 0


if __name__ == "__main__":
    sys.exit(main())