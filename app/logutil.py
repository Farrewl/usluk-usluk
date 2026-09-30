"""app/logutil.py — Logging ringan dengan throttle (pengganti print jalur panas).

`print()` tiap frame membanjiri konsol & memperlambat loop (I/O). Modul ini
menyediakan:
  - get_logger(): logger standar `logging` bernama "asv".
  - throttled(key, interval_s): True maks 1x per interval per kunci —
    untuk pesan berulang (mis. "frame gagal", "deteksi ditolak").

Pemakaian:
    from .logutil import get_logger, throttled
    log = get_logger()
    log.info("misi mulai")
    if throttled("cam_fail", 5.0):
        log.warning("frame kamera gagal dibaca")
"""

import logging
import threading
import time

_LOGGER_NAME = "asv"
_CONFIGURED = False
_CONFIG_LOCK = threading.Lock()

# Cap waktu terakhir per kunci throttle (monotonic).
_LAST = {}
_LAST_LOCK = threading.Lock()


def get_logger():
    """Logger 'asv' (handler stream, format ringkas). Idempoten."""
    global _CONFIGURED
    log = logging.getLogger(_LOGGER_NAME)
    if _CONFIGURED:
        return log
    with _CONFIG_LOCK:
        if _CONFIGURED:
            return log
        handler = logging.StreamHandler()
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(message)s", "%H:%M:%S"))
        log.addHandler(handler)
        log.setLevel(logging.INFO)
        log.propagate = False
        _CONFIGURED = True
    return log


def throttled(key, interval_s=1.0):
    """True bila `key` boleh dicetak sekarang (maks 1x per interval).

    Thread-safe. Interval 0/nonpositif -> selalu True (tanpa throttle).
    """
    try:
        interval = float(interval_s)
    except (TypeError, ValueError):
        return True
    if interval <= 0:
        return True
    now = time.monotonic()
    with _LAST_LOCK:
        prev = _LAST.get(key, 0.0)
        if now - prev >= interval:
            _LAST[key] = now
            return True
        return False
