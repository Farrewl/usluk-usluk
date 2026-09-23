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

WEIGHTS_DIR = os.path.join(ROOT, "weights")
DEFAULT_MODEL = os.path.join(WEIGHTS_DIR, "buoy.pt")
SAVE_PATH = os.path.join(ROOT, "data", "deteksi_uji.jpg")


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

    print(f"[DET] Memuat model: {args.model}")
    model = YOLO(args.model, task="detect")

    if args.image:
        print(f"[DET] Inferensi pada gambar: {args.image} (imgsz={args.imgsz})")
        results = model.predict(source=args.image, imgsz=args.imgsz,
                                conf=args.conf, verbose=False)
        if not results:
            print("[DET] Tidak ada hasil.")
            return 1
        _tampilkan(results[0])
        return 0

    # --- jalur kamera: sama seperti GUI (app/camera.py) ---
    print("[DET] Membuka kamera (fallback index otomatis)...")
    cap = open_camera(0)
    if cap is None or not cap.isOpened():
        print("[DET] ERROR: tidak ada kamera yang bisa dibuka.")
        return 1

    total_det = 0
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
            boxes = r.boxes
            n = 0 if boxes is None else len(boxes)
            total_det += n
            print(f"[DET] frame {i}: {n} objek (infer {dt_ms:.0f} ms)")
            if boxes is not None and n:
                for j, b in enumerate(boxes):
                    cls = int(b.cls[0])
                    conf = float(b.conf[0])
                    xyxy = [round(v, 1) for v in b.xyxy[0].tolist()]
                    print(f"  [{j}] class={r.names[cls]}({cls}) conf={conf:.3f} "
                          f"box={xyxy}")
            # simpan annotasi dari frame (bukan yang kosong)
            if n:
                os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
                import cv2
                cv2.imwrite(SAVE_PATH, r.plot())
                print(f"[DET] Frame ber-annotasi disimpan: {SAVE_PATH}")
    finally:
        cap.release()

    print(f"[DET] Selesai — total {total_det} objek terdeteksi.")
    if total_det == 0:
        print("[DET] (0 objek = normal bila tidak ada buoy di depan kamera.)")
    return 0


def _tampilkan(r):
    boxes = r.boxes
    n = 0 if boxes is None else len(boxes)
    print(f"[DET] Objek terdeteksi: {n}")
    if boxes is not None and n:
        for i, b in enumerate(boxes):
            cls = int(b.cls[0])
            conf = float(b.conf[0])
            xyxy = [round(v, 1) for v in b.xyxy[0].tolist()]
            print(f"  [{i}] class={r.names[cls]}({cls}) conf={conf:.3f} box={xyxy}")
    os.makedirs(os.path.dirname(SAVE_PATH), exist_ok=True)
    import cv2
    cv2.imwrite(SAVE_PATH, r.plot())
    print(f"[DET] Frame ber-annotasi disimpan: {SAVE_PATH}")


if __name__ == "__main__":
    sys.exit(main())