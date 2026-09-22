"""
app/camera.py — Buka kamera dengan backend yang cocok dengan OS.

Masalah yang diperbaiki: kode lama memakai cv2.CAP_DSHOW (DirectShow
khusus Windows) untuk SEMUA OS -> di Linux kamera tidak pernah terbuka.
Di sini backend dipilih berdasarkan platform:

  Windows : DSHOW (DirectShow), fallback MSMF, fallback ANY
  Linux   : V4L2, fallback ANY (otomatis cari /dev/videoN)

Selalu kembalikan VideoCapture yang SUDAH terbuka, atau None (pemanggil
harus siap menangani kamera None -> fallback frame sintetis).
"""

import platform

import cv2


def open_camera(index, width=None, height=None):
    """Buka kamera `index` dengan backend sesuai OS.

    Argumen:
        index : int — indeks kamera (0 = kamera default/pertama).
        width, height : int|None — resolusi yang diminta; kalau tidak
            didukung kamera, resolusi native tetap dipakai (tidak fatal).

    Return:
        cv2.VideoCapture yang isOpened()==True, atau None bila semua
        backend gagal.
    """
    if platform.system() == "Windows":
        backends = (cv2.CAP_DSHOW, cv2.CAP_MSMF, 0)
    else:
        backends = (cv2.CAP_V4L2, 0)  # 0 = ANY, fallback aman

    last_error = None
    for backend in backends:
        try:
            cap = cv2.VideoCapture(int(index), backend)
        except Exception as exc:  # pragma: no cover - kerasionalan backend
            last_error = exc
            continue
        if cap.isOpened():
            if width and height:
                # Jangan memaksa: kalau gagal, VideoCapture tetap pakai
                # ukuran native kamera (frame dinormalisasi di pemakai).
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            _calibrate_exposure(cap)
            return cap
        cap.release()

    print(f"[CAM] Semua backend gagal membuka kamera index={index} "
          f"(terakhir: {last_error})")
    return None


def _calibrate_exposure(cap, warmup=8):
    """Coba beberapa mode auto-exposure dan pakai yang paling terang.

    Kamera laptop (UVC) sering memberi frame hitam total bila mode
    exposure salah default-nya. Nilai yang dicoba khusus Linux:
      3.0 = auto (constant FPS)  1.0 = auto  0.75 = auto (logika lain)
    Di Windows mode bawaan DSHOW sudah benar, jadi tidak diotak-atik.
    """
    if platform.system() == "Windows":
        return
    # frame jadi hitam terus (brightness ~0) -> coba mode lain
    def _brightness():
        for _ in range(warmup):
            cap.read()
        r, fr = cap.read()
        return float(fr.mean()) if r and fr is not None else -1.0

    best_val, best_brightness = None, float(_brightness())
    if best_brightness < 15.0:  # kandidat gelap -> perlu kalibrasi
        for val in (3.0, 1.0, 0.75):
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, val)
            except Exception:
                continue
            b = _brightness()
            if b > best_brightness:
                best_val, best_brightness = val, b
        if best_val is not None:
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, best_val)
            except Exception:
                pass
            _brightness()  # hangatkan ulang dengan mode terbaik


def find_working_camera(preferred=None, max_index=4, width=None, height=None):
    """Cari kamera yang BISA dibuka.

    Prioritas: `preferred` (bisa None), lalu index 0..max_index. Berguna
    bila config.CAMERA_INDEX menunjuk kamera yang tidak ada di mesin ini
    (mis. di-set 1 untuk laptop lain) — app tetap dapat menampilkan kamera
    yang tersedia sambil log index yang dipakai.

    Return: cv2.VideoCapture terbuka, atau None.
    """
    order = []
    if preferred is not None:
        order.append(int(preferred))
    for i in range(max_index + 1):
        if i not in order:
            order.append(i)

    for idx in order:
        cap = open_camera(idx, width, height)
        if cap is not None:
            if idx != preferred:
                print(f"[CAM] Kamera index {preferred} tidak tersedia, "
                      f"pakai index {idx}.")
            return cap
    return None


def make_fallback_frame(width, height, text="CAMERA ERROR"):
    """Frame sintetis bertuliskan peringatan, untuk saat kamera absent."""
    import numpy as np
    frame = np.zeros((int(height), int(width), 3), dtype=np.uint8)
    cv2.putText(frame, text, (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 0, 255), 2)
    return frame