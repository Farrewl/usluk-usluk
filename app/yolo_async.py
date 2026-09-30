"""app/yolo_async.py — Worker inferensi YOLO tak-memblokir (thread murni).

Masalah: `model(frame)` Ultralytics di CPU butuh ratusan ms; bila dipanggil
sinkron di loop 30 Hz, capture + kontrol + stream OFFBOARD ikut menunggu.

Solusi: satu thread worker + antrean job berkapasitas 1 (frame terbaru
menang, lama dibuang). Loop utama hanya `submit()` lalu `latest()` — tidak
pernah menunggu inferensi. Hasil tiap model dicap waktu (monotonic) supaya
caller bisa menolak hasil basi (`max_age_s`).

Threading murni (bukan QThread) agar bisa dipakai navigator produksi dan
diuji tanpa QApplication.

Kontrak:
  - submit(model_id, model, frame, conf, imgsz, half, device): antrekan 1
    job; bila worker sibuk, frame lama dibuang diam-diam (return False),
    bila diterima return True.
  - latest(model_id, max_age_s): (hasil, umur_s) atau (None, inf) bila
    belum ada / basi. `hasil` = dict deteksi format navigator
    {cls: [{'cx','cy','box','area'}]} — TANPA filter warna (filter tetap
    di caller via validate_buoy agar perilaku identik dengan jalur sinkron).
  - start()/stop_and_wait(): lifecycle thread.
"""

import queue
import threading
import time

# Batas antrean job: 1 = hanya frame terbaru yang diproses.
_JOB_QUEUE_SIZE = 1
# Timeout ambil job agar thread responsif terhadap stop.
_JOB_GET_TIMEOUT_S = 0.2


class YoloAsyncWorker:
    """Satu worker untuk SEMUA model (dibedakan `model_id` string)."""

    def __init__(self):
        self._jobs = queue.Queue(maxsize=_JOB_QUEUE_SIZE)
        self._latest = {}  # model_id -> (detections_dict, monotonic_ts)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._busy = threading.Event()

    # -- lifecycle ------------------------------------------------------
    def start(self):
        """Jalankan thread worker (idempoten)."""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="yolo-async")
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

    # -- API loop utama --------------------------------------------------
    def submit(self, model_id, model, frame, conf, imgsz, half=False,
               device="cpu"):
        """Antrekan 1 job inferensi. Return True bila diterima.

        Bila antrean penuh (worker sibuk + 1 menunggu), job lama dibuang
        dan job baru dimasukkan — frame terbaru selalu menang.
        `model` None -> return False (model belum dimuat / dimatikan).
        """
        if model is None or self._stop_event.is_set():
            return False
        job = (model_id, model, frame, float(conf), int(imgsz),
               bool(half), device)
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
        """Ambil hasil terbaru: (detections_dict, umur_detik).

        Return ({}, inf) bila belum pernah selesai; (None, umur) bila hasil
        lebih tua dari `max_age_s` (basi — caller boleh pakai cache lama
        atau anggap kosong sesuai kebijakannya).
        """
        with self._lock:
            item = self._latest.get(model_id)
        if item is None:
            return {}, float("inf")
        det, ts = item
        age = time.monotonic() - ts
        if age > max_age_s:
            return None, age
        return det, age

    def drop(self, model_id):
        """Buang hasil tersimpan satu model (mis. ganti misi/leg)."""
        with self._lock:
            self._latest.pop(model_id, None)

    def drop_all(self):
        """Buang semua hasil (reset misi)."""
        with self._lock:
            self._latest.clear()

    # -- internal ---------------------------------------------------------
    @staticmethod
    def _parse_results(results):
        """results Ultralytics -> {cls: [{cx,cy,box,area}]} (tanpa filter)."""
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
                x1, y1, x2, y2 = map(int, xyxy_t[0])
                area = (x2 - x1) * (y2 - y1)
                ent = {"cx": (x1 + x2) // 2, "cy": (y1 + y2) // 2,
                       "box": (x1, y1, x2, y2), "area": area}
                try:
                    conf_val = float(box.conf[0])
                except Exception:
                    conf_val = 1.0
                ent["conf"] = conf_val
                detections.setdefault(cls, []).append(ent)
        return detections

    def _run(self):
        while not self._stop_event.is_set():
            try:
                job = self._jobs.get(timeout=_JOB_GET_TIMEOUT_S)
            except queue.Empty:
                continue
            model_id, model, frame, conf, imgsz, half, device = job
            self._busy.set()
            try:
                results = model(frame, verbose=False, imgsz=imgsz,
                                half=half, device=device, conf=conf)
                det = self._parse_results(results)
                with self._lock:
                    self._latest[model_id] = (det, time.monotonic())
            except Exception:
                # Model rusak / frame aneh: jangan racuni cache; biarkan
                # hasil lama (caller menilai basi via max_age_s).
                pass
            finally:
                self._busy.clear()
