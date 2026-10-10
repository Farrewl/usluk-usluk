"""app/yolo_worker.py — Worker inferensi YOLO tak-memblokir (satu untuk semua model).

Masalah: `model(frame)` Ultralytics di CPU butuh ratusan ms; bila dipanggil
sinkron di loop 20–30 Hz, capture + kontrol + stream OFFBOARD ikut menunggu.

Solusi: satu thread worker + antrean job berkapasitas 1 (frame terbaru
menang, lama dibuang). Loop utama hanya `submit()` lalu `latest()` — tidak
pernah menunggu inferensi. Hasil tiap model dicap waktu (monotonic) supaya
caller bisa menolak hasil basi.

Threading MURNI (bukan QThread) agar bisa dipakai navigator produksi maupun
simulator darat dan diuji tanpa `QApplication`.

Kontrak:
  - submit(model_id, model, frame, conf, imgsz): antrekan 1 job; bila
    worker sibuk, frame lama dibuang diam-diam. `model=None` -> return False.
  - latest(model_id, max_age_s): (deteksi, umur_s) atau (None, umur) bila
    basi / ({}, inf) bila belum pernah selesai. `deteksi` = dict format
    navigator {cls: [{'cx','cy','box','area','conf'}]} TANPA filter warna
    (filter tetap di caller via validate_buoy agar perilaku identik dengan
    jalur sinkron lama).
  - latest_status(model_id): (detected, annotated, boxes, ts) atau None —
    dipakai simulator darat untuk tampilan + hindar-rintangan.
  - latest_boxes_any(max_age_s): kotak terbaru semua model (hindar-rintangan).
  - bind_gate_model(model): tandai model gate (butuh >=2 buoy utk "detected").
  - start() / stop_and_wait(): lifecycle thread.
"""

import queue
import threading
import time

try:
    from .detection_validation import validate_buoy, draw_validated_boxes
    VALIDATION_AVAILABLE = True
except Exception:  # pragma: no cover - cv2/numpy tidak ada
    validate_buoy = None
    draw_validated_boxes = None
    VALIDATION_AVAILABLE = False

# Batas antrean job: 1 = hanya frame terbaru yang diproses.
_JOB_QUEUE_SIZE = 1
# Timeout ambil job agar thread responsif terhadap stop.
_JOB_GET_TIMEOUT_S = 0.2


class YoloWorker:
    """Satu worker untuk SEMUA model (dibedakan `model_id`)."""

    def __init__(self, config):
        self.config = config
        self._jobs = queue.Queue(maxsize=_JOB_QUEUE_SIZE)
        # model_id -> dict{det, boxes, annotated, detected, ts}
        self._latest = {}
        self._gate_model = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._busy = threading.Event()

    def bind_gate_model(self, model):
        """Tandai model gate (hasil tampilannya minta >= 2 objek = 2 buoy)."""
        self._gate_model = model

    # ------------------- lifecycle -------------------

    def start(self):
        """Jalankan thread worker (idempoten)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="yolo-worker")
        self._thread.start()

    def stop_and_wait(self, timeout_s=3.0):
        """Minta berhenti + tunggu thread selesai (aman dipanggil 2x)."""
        self._stop_event.set()
        thr = self._thread
        if thr is not None and thr.is_alive():
            thr.join(timeout=timeout_s)
        self._thread = None

    @property
    def busy(self):
        """True bila worker sedang menjalankan inferensi."""
        return self._busy.is_set()

    # ------------------- API loop utama -------------------

    def submit(self, model_id, model, frame, conf, imgsz=None):
        """Antrekan 1 job inferensi. Return True bila diterima.

        Bila antrean penuh (worker sibuk + 1 menunggu), job lama dibuang
        dan job baru dimasukkan — frame terbaru selalu menang.
        `model` None -> return False (model belum dimuat / dimatikan).
        """
        if model is None or self._stop_event.is_set():
            return False
        try:
            size = int(imgsz if imgsz is not None
                       else self.config.YOLO_INFERENCE_SIZE)
        except Exception:
            size = 320
        job = (model_id, model, frame, float(conf), size)
        try:
            self._jobs.put_nowait(job)
            return True
        except queue.Full:
            try:
                self._jobs.get_nowait()  # buang job lama
            except queue.Empty:
                pass
            try:
                self._jobs.put_nowait(job)
                return True
            except queue.Full:
                return False

    def latest(self, model_id, max_age_s=float("inf")):
        """Ambil deteksi terbaru: (detections_dict, umur_detik).

        Return ({}, inf) bila belum pernah selesai; (None, umur) bila hasil
        lebih tua dari `max_age_s` (basi — caller boleh pakai cache lama
        atau anggap kosong sesuai kebijakannya).
        """
        with self._lock:
            item = self._latest.get(model_id)
        if item is None:
            return {}, float("inf")
        age = time.monotonic() - item["ts"]
        if age > max_age_s:
            return None, age
        return dict(item["det"]), age

    def latest_status(self, model_id):
        """(detected, annotated, boxes, ts) atau None bila belum pernah.

        Dipakai simulator darat (tampilan + status). Tanpa batas umur —
        caller memakai cache terakhir bila None.
        """
        with self._lock:
            item = self._latest.get(model_id)
        if item is None:
            return None
        return (item["detected"], item["annotated"],
                list(item["boxes"]), item["ts"])

    def latest_boxes_any(self, max_age_s=0.5):
        """Kotak deteksi (cls, conf, xyxy) dari hasil TERBARU semua model.

        Dipakai hindar-rintangan reaktif: hasil lebih tua dari `max_age_s`
        dianggap basi -> []. Return list kosong bila worker belum pernah
        selesai atau semuanya basi (jangan menggerakkan avoid dgn data lama).
        """
        with self._lock:
            now = time.monotonic()
            newest = None
            for item in self._latest.values():
                boxes = item.get("boxes") or []
                if not boxes:
                    continue
                if newest is None or item["ts"] > newest[0]:
                    newest = (item["ts"], boxes)
        if newest is None:
            return []
        ts, boxes = newest
        if now - ts > max_age_s:
            return []
        return list(boxes)

    def drop(self, model_id):
        """Buang hasil tersimpan satu model (mis. ganti misi/leg)."""
        with self._lock:
            self._latest.pop(model_id, None)

    def drop_all(self):
        """Buang semua hasil (reset misi)."""
        with self._lock:
            self._latest.clear()

    # ------------------- internal -------------------

    @staticmethod
    def _parse_results(results):
        """results Ultralytics -> {cls: [{cx,cy,box,area,conf}]} (tanpa filter)."""
        detections = {}
        for r in results:
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for box in boxes:
                cls_t = getattr(box, "cls", None)
                xyxy_t = getattr(box, "xyxy", None)
                if cls_t is None or len(cls_t) == 0:
                    continue
                if xyxy_t is None or len(xyxy_t) == 0:
                    continue
                cls = int(cls_t[0])
                x1, y1, x2, y2 = (int(v) for v in xyxy_t[0])
                area = (x2 - x1) * (y2 - y1)
                try:
                    conf_val = float(box.conf[0])
                except Exception:
                    conf_val = 1.0
                ent = {"cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2,
                       "box": (x1, y1, x2, y2), "area": area, "conf": conf_val}
                detections.setdefault(cls, []).append(ent)
        return detections

    def _gate_keep(self, frame, raw_boxes):
        """Filter tampilan gate: buang kotak yang gagal validate_buoy."""
        keep = []
        for cls, conf, xyxy in raw_boxes:
            if cls not in (self.config.RED_BALL_CLASS_ID,
                           self.config.GREEN_BALL_CLASS_ID):
                keep.append((cls, conf, xyxy))
                continue
            if not VALIDATION_AVAILABLE or not validate_buoy(
                    frame, cls, xyxy,
                    min_area=self.config.MIN_BUOY_AREA_PX,
                    min_color_fraction=self.config.BUOY_MIN_COLOR_FRACTION,
                    min_saturation=self.config.BUOY_MIN_SATURATION,
                    max_aspect_deviation=self.config.BUOY_MAX_ASPECT_DEVIATION):
                continue
            keep.append((cls, conf, xyxy))
        return keep

    def _run(self):
        while not self._stop_event.is_set():
            try:
                model_id, model, frame, conf, imgsz = self._jobs.get(
                    timeout=_JOB_GET_TIMEOUT_S)
            except queue.Empty:
                continue
            self._busy.set()
            try:
                results = model(frame, verbose=False, conf=conf, imgsz=imgsz)
                det = self._parse_results(results)
                raw_boxes = []
                for cls, items in det.items():
                    for b in items:
                        raw_boxes.append((cls, float(b.get("conf", 1.0)),
                                          list(b["box"])))
                is_gate = (model is self._gate_model
                           and self._gate_model is not None)
                if is_gate:
                    keep = self._gate_keep(frame, raw_boxes)
                    detected = len(keep) >= 2   # gate butuh 2 buoy
                else:
                    keep = raw_boxes
                    detected = len(raw_boxes) > 0
                try:
                    if VALIDATION_AVAILABLE:
                        annotated = draw_validated_boxes(
                            frame, keep, results[0].names)
                    else:
                        annotated = results[0].plot()
                except Exception:
                    annotated = frame
                with self._lock:
                    self._latest[model_id] = {
                        "det": det, "boxes": raw_boxes, "annotated": annotated,
                        "detected": bool(detected), "ts": time.monotonic(),
                    }
            except Exception:
                # Model rusak / frame aneh: jangan racuni cache; biarkan
                # hasil lama (caller menilai basi via max_age_s).
                pass
            finally:
                self._busy.clear()
