"""Lapisan tipis Python -> C (ctypes only, TANPA logika hitung).
Contoh: core.mix_diff_drive(0.5, 0.1) -> (pwm_kiri, pwm_kanan).
Build dulu: gcc -shared -fPIC -Icore/include core/src/thruster_mixer.c core/src/ecu_link.c core/src/arbitrator.c -o core/libaterkia.so
Kalau .so belum ada: C_AVAILABLE=False, pakai acuan Python (app/filtering.py).
"""

import ctypes
import os

APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(APP_DIR)
LIB_PATH = os.path.join(ROOT_DIR, "core", "libaterkia.so")

C_AVAILABLE = False
_lib = None

try:
    if os.path.exists(LIB_PATH):
        _lib = ctypes.CDLL(LIB_PATH)
        C_AVAILABLE = True
except Exception:
    _lib = None
    C_AVAILABLE = False


def _need_lib():
    if not C_AVAILABLE or _lib is None:
        raise RuntimeError(
            "libaterkia.so belum di-build. Jalankan: "
            "gcc -shared -fPIC -Icore/include core/src/thruster_mixer.c "
            "core/src/ecu_link.c core/src/arbitrator.c -o core/libaterkia.so")


# ---------------------------------------------------------------------------
# thruster_mixer.h
# ---------------------------------------------------------------------------

def mix_diff_drive(surge, yaw):
    """Mixer differential drive (C). Return (pwm_kiri, pwm_kanan) µs.

    surge [-1..1] maju, yaw [-1..1] kanan. Output 1100..1900 µs.
    """
    _need_lib()
    _lib.mixer_diff_drive.argtypes = [
        ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
    _lib.mixer_diff_drive.restype = None
    kiri = ctypes.c_int(1500)
    kanan = ctypes.c_int(1500)
    # POINTER (bukan byref) agar cocok argtypes POINTER.
    _lib.mixer_diff_drive(float(surge), float(yaw),
                          ctypes.pointer(kiri), ctypes.pointer(kanan))
    return kiri.value, kanan.value


# ---------------------------------------------------------------------------
# ecu_link.h
# ---------------------------------------------------------------------------

# Urutan field = struct ecu_status_t (C).
class _EcuStatusC(ctypes.Structure):
    _fields_ = [
        ("valid", ctypes.c_int),
        ("kill", ctypes.c_int),
        ("estop", ctypes.c_int),
        ("overtemp", ctypes.c_int),
        ("overcurrent", ctypes.c_int),
        ("current_a", ctypes.c_double),
        ("temp_esc_c", ctypes.c_double),
        ("temp_amb_c", ctypes.c_double),
        ("voltage_v", ctypes.c_double),
    ]


def parse_ecu_frame(raw8):
    """Parse 8 byte frame STM32 (C). Return dict. Lihat ecu_link.h."""
    _need_lib()
    if len(raw8) != 8:
        raise ValueError("frame ECU harus tepat 8 byte")
    _lib.ecu_parse_frame.argtypes = [ctypes.c_char_p,
                                     ctypes.POINTER(_EcuStatusC)]
    _lib.ecu_parse_frame.restype = ctypes.c_int
    buf = ctypes.create_string_buffer(bytes(raw8), 8)
    out = _EcuStatusC()
    ok = _lib.ecu_parse_frame(buf, ctypes.byref(out))
    return {
        "valid": bool(ok and out.valid),
        "kill": bool(out.kill),
        "estop": bool(out.estop),
        "overtemp": bool(out.overtemp),
        "overcurrent": bool(out.overcurrent),
        "current_a": float(out.current_a),
        "temp_esc_c": float(out.temp_esc_c),
        "temp_amb_c": float(out.temp_amb_c),
        "voltage_v": float(out.voltage_v),
    }


def build_ecu_cmd(cmd):
    """Bangun 1 byte perintah NUC->STM32 (C). 0=normal 1=kill 2=clear."""
    _need_lib()
    _lib.ecu_build_cmd.argtypes = [ctypes.c_int]
    _lib.ecu_build_cmd.restype = ctypes.c_ubyte
    return int(_lib.ecu_build_cmd(int(cmd)))


# ---------------------------------------------------------------------------
# arbitrator.h
# ---------------------------------------------------------------------------

class _ArbOutC(ctypes.Structure):
    _fields_ = [
        ("surge", ctypes.c_double),
        ("yaw", ctypes.c_double),
        ("source", ctypes.c_int),
        ("killed", ctypes.c_int),
    ]


# Nama sumber — sama dengan #define ARB_SRC_* di arbitrator.h
SRC_KILLED = 0
SRC_MANUAL = 1
SRC_AUTO = 2
SRC_NAMES = {0: "KILLED", 1: "MANUAL", 2: "AUTO"}


def arbitrate(kill_active, manual_active,
              manual_surge, manual_yaw, auto_surge, auto_yaw):
    """Pilih setpoint thrust (C). Return dict surge/yaw/source/killed."""
    _need_lib()
    _lib.arb_select.argtypes = [
        ctypes.c_int, ctypes.c_int,
        ctypes.c_double, ctypes.c_double,
        ctypes.c_double, ctypes.c_double,
        ctypes.POINTER(_ArbOutC)]
    _lib.arb_select.restype = None
    out = _ArbOutC()
    _lib.arb_select(int(bool(kill_active)), int(bool(manual_active)),
                    float(manual_surge), float(manual_yaw),
                    float(auto_surge), float(auto_yaw),
                    ctypes.byref(out))
    return {
        "surge": float(out.surge),
        "yaw": float(out.yaw),
        "source": int(out.source),
        "source_name": SRC_NAMES.get(int(out.source), "?"),
        "killed": bool(out.killed),
    }
