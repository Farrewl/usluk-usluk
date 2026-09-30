"""app/uploader.py — Antrean upload foto berkapasitas terbatas (1 worker).

Masalah: tiap foto mem-spawn 1 thread `requests.post(timeout=10)`. Saat
jaringan lemot, thread menumpuk tanpa batas (leak) dan loop misi ikut
terbebani pembuatan thread.

Solusi: 1 thread worker + antrean FIFO berkapasitas `max_queue` (default 3).
Bila penuh, foto TERLAMA dibuang (foto baru lebih penting untuk operator).
Upload gagal (jaringan/server) dicoba ulang maks `max_retries` (default 1x)
lalu dibuang — tidak pernah memblokir caller.

Kontrak:
  - enqueue(path, filename): return True bila masuk antrean.
  - start()/stop_and_wait(): lifecycle thread.
  - pending: jumlah antrean saat ini (observabilitas GUI).
"""

import os
import queue
import threading

try:
    import requests
    _REQUESTS_OK = True
except ImportError:  # pragma: no cover - mesin tanpa requests
    _REQUESTS_OK = False

# Batas antrean upload: 3 foto (~300 KB) — cukup untuk bukti misi.
_DEFAULT_MAX_QUEUE = 3
# Timeout HTTP per percobaan (detik).
_UPLOAD_TIMEOUT_S = 10
# Coba ulang 1x bila gagal (total maks 2 percobaan per foto).
_DEFAULT_MAX_RETRIES = 1
# Timeout join thread saat stop (detik).
_STOP_JOIN_TIMEOUT_S = 3.0


class UploadWorker:
    """Satu worker upload untuk semua foto misi."""

    def __init__(self, upload_url, max_queue=_DEFAULT_MAX_QUEUE,
                 max_retries=_DEFAULT_MAX_RETRIES):
        self._url = upload_url
        self._max_retries = int(max_retries)
        self._jobs = queue.Queue(maxsize=int(max_queue))
        self._stop_event = threading.Event()
        self._thread = None
        self._dropped = 0  # foto dibuang karena antrean penuh (diagnostik)
        self._lock = threading.Lock()

    # -- lifecycle ------------------------------------------------------
    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="uploader")
        self._thread.start()

    def stop_and_wait(self, timeout_s=_STOP_JOIN_TIMEOUT_S):
        self._stop_event.set()
        thr = self._thread
        if thr is not None and thr.is_alive():
            thr.join(timeout=timeout_s)
        self._thread = None

    # -- API --------------------------------------------------------------
    def enqueue(self, path, filename):
        """Masukkan foto ke antrean. Antrean penuh -> foto TERLAMA dibuang.

        Return True bila job masuk antrean.
        """
        if not os.path.exists(path):
            return False
        try:
            self._jobs.put_nowait((path, filename))
            return True
        except queue.Full:
            try:
                self._jobs.get_nowait()  # buang yang paling lama menunggu
                with self._lock:
                    self._dropped += 1
            except queue.Empty:
                pass
            try:
                self._jobs.put_nowait((path, filename))
                return True
            except queue.Full:
                return False

    @property
    def pending(self):
        """Jumlah foto menunggu upload."""
        return self._jobs.qsize()

    @property
    def dropped(self):
        """Jumlah foto dibuang karena antrean penuh."""
        with self._lock:
            return self._dropped

    # -- internal -----------------------------------------------------------
    def _upload_once(self, path, filename):
        with open(path, "rb") as f:
            files = {"file": (filename, f, "image/jpeg")}
            resp = requests.post(self._url, files=files,
                                 timeout=_UPLOAD_TIMEOUT_S)
            return resp.status_code == 200

    def _run(self):
        import time as _time
        while not self._stop_event.is_set():
            try:
                path, filename = self._jobs.get(timeout=0.2)
            except queue.Empty:
                continue
            if not _REQUESTS_OK:
                continue
            ok = False
            for _ in range(self._max_retries + 1):
                try:
                    ok = self._upload_once(path, filename)
                    if ok:
                        break
                except Exception:
                    ok = False
                if self._stop_event.is_set():
                    break
                _time.sleep(0.5)
            # Gagal total -> buang diam-diam (foto tetap ada di disk lokal).
