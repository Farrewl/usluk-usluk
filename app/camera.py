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

import os
import platform
import time

import cv2


# Kandidat resolusi saat negosiasi otomatis (diurutkan dari tertinggi).
# Kamera USB umumnya "meng-clamp" permintaan di atas kemampuan aslinya
# (mis. minta 1920x1080 tapi hanya sanggup 1280x720), jadi urutan ini
# natural menemukan resolusi tertinggi yang benar-benar didukung.
CAMERA_CANDIDATE_SIZES = (
    (1920, 1080), (1600, 1200), (1280, 960), (1280, 720),
    (1024, 768), (800, 600), (640, 480), (320, 240),
)
# Urutan codec: MJPG dulu (bandwidth kecil -> bisa 30 fps di resolusi
# tinggi), YUYV sebagai cadangan (tanpa MJPG, YUYV sering cuma ~7 fps
# di 720p pada USB 2.0).
CAMERA_CODECS = ("MJPG", "YUYV")


def _platform_backends():
    """Backend cv2 sesuai OS (dipakai pembuka kamera & probing mode)."""
    if platform.system() == "Windows":
        return (cv2.CAP_DSHOW, cv2.CAP_MSMF, 0)
    return (cv2.CAP_V4L2, 0)


def _make_capture(index, backend):
    """VideoCapture dengan backend tertentu; None bila gagal total."""
    try:
        return cv2.VideoCapture(int(index), backend)
    except Exception:  # pragma: no cover - variasi build cv2
        return cv2.VideoCapture(int(index))


def _apply_mode(cap, fourcc_val, width, height, target_fps):
    """Terapkan codec + resolusi + fps ke capture yang sudah terbuka."""
    if fourcc_val:
        cap.set(cv2.CAP_PROP_FOURCC, int(fourcc_val))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    if target_fps:
        cap.set(cv2.CAP_PROP_FPS, float(target_fps))


def _measure_fps(cap, seconds=0.6):
    """Ukur fps NYATA dengan membaca frame selama `seconds` detik."""
    t0 = time.time()
    n = 0
    while time.time() - t0 < seconds:
        if cap.read()[0]:
            n += 1
    dt = time.time() - t0
    return n / dt if dt > 0 else 0.0


def _open_capture_mode_for_backend(index, fourcc_val, width, height,
                                   target_fps, backends):
    """Buka kamera dengan mode tertentu via daftar backend eksplisit.

    `backends` berupa tuple backend cv2 yang boleh dicoba (mis. hanya
    V4L2 untuk probe cepat, atau (DSHOW, MSMF, ANY) untuk buka final).
    Return: cv2.VideoCapture terbuka, atau None bila semua gagal.
    """
    for backend in backends:
        cap = _make_capture(index, backend)
        if cap is None or not cap.isOpened():
            if cap is not None:
                cap.release()
            continue
        _apply_mode(cap, fourcc_val, width, height, target_fps)
        return cap
    return None


def _open_capture_mode(index, fourcc_val, width, height, target_fps):
    """Buka kamera dengan mode tertentu; None bila semua backend gagal."""
    return _open_capture_mode_for_backend(index, fourcc_val, width, height,
                                          target_fps, _platform_backends())


def _probe_mode(index, fourcc_val, width, height, target_fps, probe_seconds,
                backends=None):
    """Coba satu kandidat mode: buka, hangatkan, ukur fps nyata.

    Return: (cap, fps_nyata); cap=None berarti mode tidak bisa dibuka.
    `backends` membatasi backend yang dicoba (default: semua platform).
    """
    for backend in (backends or _platform_backends()):
        cap = _open_capture_mode_for_backend(index, fourcc_val, width, height,
                                             target_fps, (backend,))
        if cap is None:
            continue
        for _ in range(6):  # warm-up singkat supaya pengukuran stabil
            if cap.read()[0]:
                break
        fps = _measure_fps(cap, seconds=probe_seconds)
        return cap, fps
    return None, 0.0


def negotiate_camera(index, target_fps=30, max_width=1920, max_height=1080,
                     min_fps=25.0, probe_seconds=0.6):
    """Cari mode terbaik: resolusi tertinggi yang tetap ≥ `min_fps` fps.

    Mencoba kandidat resolusi dari tertinggi, masing-masing dengan codec
    MJPG lalu YUYV, sampai menemukan yang terbuka DAN fps-nya memenuhi.
    Kalau tidak ada yang memenuhi, kembalikan mode pertama yang terbuka
    (`fallback`) atau None bila kamera sama sekali tidak bisa dibuka.

    Return: (fourcc_val, width, height, fps_nyata) atau None.
    """
    sizes = [s for s in CAMERA_CANDIDATE_SIZES
             if s[0] <= int(max_width) and s[1] <= int(max_height)]
    if not sizes:
        sizes = [(640, 480)]

    fallback = None
    for width, height in sizes:
        for codec in CAMERA_CODECS:
            fourcc_val = cv2.VideoWriter_fourcc(*codec)
            cap, fps = _probe_mode(int(index), fourcc_val, width, height,
                                   target_fps, probe_seconds)
            if cap is None:
                continue
            real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            cap.release()
            if fallback is None:
                fallback = (fourcc_val, real_w, real_h, fps)
            if fps >= min_fps:
                return (fourcc_val, real_w, real_h, fps)
    return fallback


def _index_may_exist(idx):
    """Cek cepat apakah indeks kamera kemungkinan ada.

    Di Linux cukup cek node /dev/videoN (menghindari probing memakan waktu
    & spam warning untuk indeks kosong seperti /dev/video2 padahal hanya
    ada 2 kamera). Di Windows probing langsung lebih aman (tidak ada node).
    """
    if platform.system() == "Windows":
        return True
    return os.path.exists(f"/dev/video{int(idx)}")


def _probe_index_quality(index, target_fps):
    """Skor cepat ukuran piksel yang bisa dikeluarkan sebuah index kamera.

    Hanya pakai backend native (V4L2 di Linux / DSHOW di Windows) dengan
    satu mode 1280x720 MJPG — tidak dilakukan eksplorasi panjang. Tujuan:
    membandingkan antar index dengan cepat (~0,5-1 s) sebelum memutuskan
    index terbaik. Return (width, height, fps) hasil clamp kamera, atau
    None bila lewat backend native kamera tak bisa dibuka.
    """
    native = (_platform_backends()[0],)
    fourcc_val = cv2.VideoWriter_fourcc(*"MJPG")
    cap, fps = _probe_mode(int(index), fourcc_val, 1280, 720,
                           target_fps, 0.5, backends=native)
    if cap is None:
        # Kamera tanpa MJPG (mis. YUYV saja): buka default native sekali.
        for backend in native:
            cap2 = _make_capture(int(index), backend)
            if cap2 is not None and cap2.isOpened():
                cap2.read()
                w = int(cap2.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap2.get(cv2.CAP_PROP_FRAME_HEIGHT))
                cap2.release()
                return (w, h, 0.0)
        return None
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cap.release()
    return (w, h, fps)


def negotiate_best_camera(preferred, target_fps=30, max_width=1920,
                          max_height=1080, min_fps=25.0, max_index=4):
    """Pilih kamera terbaik sekaligus mode terbaiknya.

    `preferred` dicoba lebih dulu; selanjutnya index 0..max_index lain
    (yang node-nya ada). Setiap index di-skor cepat (lihat
    `_probe_index_quality`), lalu hanya index dengan luas piksel terbesar
    yang di-negosiasi penuh. Seri diputuskan fps, lalu index (preferen).

    Return: (index_terpilih, fourcc_val, width, height, fps) atau None.
    """
    order = [int(preferred)]
    for i in range(int(max_index) + 1):
        if i not in order and _index_may_exist(i):
            order.append(i)

    scores = {}
    for idx in order:
        s = _probe_index_quality(idx, target_fps)
        if s is not None:
            scores[idx] = s
    if not scores:
        return None

    prefer_pos = {idx: pos for pos, idx in enumerate(order)}

    def _rank(idx):
        w, h, fps = scores[idx]
        return (w * h, fps, -prefer_pos[idx])

    best_idx = max(scores, key=_rank)

    # Negosiasi detail hanya untuk index terbaik: probe pertama pada
    # ukuran terbesar langsung lolos di kamera sehat -> cepat.
    mode = negotiate_camera(best_idx, target_fps=target_fps,
                            max_width=max_width, max_height=max_height,
                            min_fps=min_fps)
    if mode is None:
        w, h, fps = scores[best_idx]
        mode = (cv2.VideoWriter_fourcc(*"MJPG"), w, h, fps)
    if best_idx != int(preferred):
        print(f"[CAM] Kamera {int(preferred)} resolusinya lebih kecil; "
              f"pakai index {best_idx} (area terbesar).")
    return (best_idx, mode[0], mode[1], mode[2], mode[3])


def _open_auto_negotiated(index, target_fps=None):
    """Buka kamera terbaik dengan resolusi tertinggi & fps stabil."""
    from . import settings as cfg

    target = target_fps or cfg.CAMERA_TARGET_FPS
    best = negotiate_best_camera(index,
                                 target_fps=target,
                                 max_width=cfg.CAMERA_MAX_AUTO_WIDTH,
                                 max_height=cfg.CAMERA_MAX_AUTO_HEIGHT,
                                 min_fps=cfg.CAMERA_MIN_ACCEPT_FPS)
    if best is None:
        return None
    chosen_idx, fourcc_val, width, height, fps = best
    codec_name = "MJPG" if fourcc_val == cv2.VideoWriter_fourcc(*"MJPG") else "YUYV"
    print(f"[CAM] Mode negosiasi kamera index={chosen_idx}: {width}x{height} "
          f"{fps:.0f} fps ({codec_name}).")
    cap = _open_capture_mode(int(chosen_idx), fourcc_val, width, height, target)
    if cap is not None:
        _calibrate_exposure(cap)
    return cap


def open_camera(index, width=None, height=None, target_fps=None,
                auto_highest=False):
    """Buka kamera `index` dengan backend sesuai OS.

    Argumen:
        index : int — indeks kamera (0 = kamera default/pertama).
        width, height : int|None — resolusi yang diminta; kalau tidak
            didukung kamera, resolusi native tetap dipakai (tidak fatal).
        target_fps : int|None — fps yang diusahakan (via CAP_PROP_FPS).
        auto_highest : bool — bila True, abaikan width/height dan pakai
            hasil negosiasi kamera terbaik: index dengan resolusi tertinggi
            yang fps-nya stabil (lihat negotiate_best_camera). Dipakai
            kamera navigasi utama.

    Return:
        cv2.VideoCapture yang isOpened()==True, atau None bila semua
        backend gagal.
    """
    if auto_highest:
        cap = _open_auto_negotiated(index, target_fps)
        if cap is not None:
            return cap
        print(f"[CAM] Negosiasi mode gagal untuk index={index}, "
              "lanjut ke mode lama.")

    last_error = None
    for backend in _platform_backends():
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
            if target_fps:
                cap.set(cv2.CAP_PROP_FPS, float(target_fps))
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


# Kode cv2.flip: 1 = horizontal (mirror), 0 = vertikal, -1 = 180 derajat.
_FLIP_MODE_TO_CV2_CODE = {
    0: None,      # tidak diputar sama sekali (gambar sensor asli)
    1: 1,         # mirror kiri-kanan
    2: 0,         # atas-bawah
    3: -1,        # 180 derajat
}


def flip_frame_if_needed(frame, flip_mode=0):
    """Terapkan orientasi kamera sesuai CAMERA_FLIP_MODE.

    `flip_mode` 0 (default) = gambar TIDAK dibalik — persis output sensor,
    jadi "camera jangan reverse" terpenuhi tanpa mengubah apa pun. Mode 1-3
    untuk operator yang memasang kamera dengan orientasi fisik tertentu.
    Mode tidak dikenal diperlakukan sebagai 0 (aman).
    """
    code = _FLIP_MODE_TO_CV2_CODE.get(int(flip_mode))
    if code is None:
        return frame
    return cv2.flip(frame, code)