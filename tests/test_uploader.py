"""Test app/uploader.py — antrean upload berkapasitas terbatas (tanpa server).

`requests.post` di-monkeypatch agar tanpa jaringan: simulasi sukses / gagal
/ lambat. Verifikasi kontrak docblock: enqueue, foto-terlama-dibuang,
retry, pending/dropped, stop bersih.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import os
import tempfile
import time
import unittest

import app.uploader as up_mod
from app.uploader import UploadWorker


class _Resp:
    def __init__(self, code=200):
        self.status_code = code
        self.text = "ok"


def _wait(pred, timeout_s=5.0):
    t0 = time.monotonic()
    while not pred():
        if time.monotonic() - t0 > timeout_s:
            return False
        time.sleep(0.01)
    return True


class _Foto:
    """File JPEG palsu sementara (isi bebas — post di-stub)."""

    def __init__(self, directory):
        fd, self.path = tempfile.mkstemp(suffix=".jpg", dir=directory)
        os.write(fd, b"\xff\xd8\xff\x00fake")
        os.close(fd)

    def hapus(self):
        try:
            os.unlink(self.path)
        except OSError:
            pass


class UploadTestBase(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="upl_")
        self._calls = []
        self._orig_post = up_mod.requests.post if up_mod._REQUESTS_OK else None
        up_mod._REQUESTS_OK = True

    def tearDown(self):
        if self._orig_post is not None:
            up_mod.requests.post = self._orig_post
        for f in os.listdir(self.tmp):
            try:
                os.unlink(os.path.join(self.tmp, f))
            except OSError:
                pass
        try:
            os.rmdir(self.tmp)
        except OSError:
            pass

    def _stub_post(self, fn):
        up_mod.requests.post = fn

    def _foto(self):
        f = _Foto(self.tmp)
        self.addCleanup(f.hapus)
        return f


class TestEnqueueUpload(UploadTestBase):

    def test_sukses_mengosongkan_antrean(self):
        terkirim = []
        self._stub_post(lambda url, files, timeout: terkirim.append(
            files["file"][0]) or _Resp(200))
        w = UploadWorker("http://x/upload", max_queue=3)
        w.start()
        try:
            f = self._foto()
            self.assertTrue(w.enqueue(f.path, "a.jpg"))
            self.assertTrue(_wait(lambda: not w.pending and terkirim))
            self.assertEqual(terkirim, ["a.jpg"])
        finally:
            w.stop_and_wait()

    def test_file_hilang_ditolak(self):
        w = UploadWorker("http://x/upload")
        self.assertFalse(w.enqueue("/tak/ada.jpg", "x.jpg"))
        self.assertEqual(w.pending, 0)

    def test_antrean_penuh_buang_terlama(self):
        # Post pertama diblokir agar antrean sempat penuh.
        lepas = []
        def post_lambat(url, files, timeout):
            if not lepas:
                time.sleep(0.4)
            return _Resp(200)
        self._stub_post(post_lambat)
        w = UploadWorker("http://x/upload", max_queue=1)
        w.start()
        try:
            fotos = [self._foto() for _ in range(3)]
            for i, f in enumerate(fotos):
                w.enqueue(f.path, f"f{i}.jpg")
            lepas.append(True)
            self.assertTrue(_wait(lambda: w.pending == 0, timeout_s=5.0))
            # Minimal 1 foto dibuang (kapasitas 1, 3 foto cepat).
            self.assertGreaterEqual(w.dropped, 1)
        finally:
            w.stop_and_wait()

    def test_gagal_lalu_sukses_retry(self):
        percobaan = []
        def post_flaky(url, files, timeout):
            percobaan.append(files["file"][0])
            return _Resp(500 if len(percobaan) == 1 else 200)
        self._stub_post(post_flaky)
        w = UploadWorker("http://x/upload", max_retries=1)
        w.start()
        try:
            f = self._foto()
            w.enqueue(f.path, "r.jpg")
            self.assertTrue(_wait(lambda: len(percobaan) >= 2))
        finally:
            w.stop_and_wait()

    def test_gagal_total_dibuang_tak_macet(self):
        self._stub_post(lambda url, files, timeout: (_ for _ in ()).throw(
            ConnectionError("putus")))
        w = UploadWorker("http://x/upload", max_retries=1)
        w.start()
        try:
            f = self._foto()
            w.enqueue(f.path, "g.jpg")
            self.assertTrue(_wait(lambda: w.pending == 0, timeout_s=5.0))
        finally:
            w.stop_and_wait()

    def test_stop_idempoten(self):
        w = UploadWorker("http://x/upload")
        w.start()
        w.stop_and_wait()
        w.stop_and_wait()


if __name__ == "__main__":
    unittest.main()
