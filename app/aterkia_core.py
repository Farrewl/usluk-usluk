"""Lapisan tipis Python -> C (ctypes only, TANPA logika hitung).

Semua hitungan (PID, EKF, nav, fuzzy, RC, manual, mixer, arbitrator)
dijalankan fungsi C di core/libaterkia.so. Python hanya meneruskan angka
& struct — tidak ada rumus yang diduplikasi di sini.

Build: gcc -shared -fPIC -O2 -Icore/include core/src/*.c -o core/libaterkia.so -lm
(gamepad_input.cpp hanya dipakai via CMake static lib, bukan .so ini.)

Bila .so belum ada: C_AVAILABLE=False, pemanggil WAJIB gagal eksplisit
(jangan fallback diam-diam ke logika Python ganda).
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
            "gcc -shared -fPIC -O2 -Icore/include core/src/*.c "
            "-o core/libaterkia.so -lm")


_D = ctypes.c_double
_I = ctypes.c_int


def _sig(name, argtypes, restype):
    _need_lib()
    fn = getattr(_lib, name)
    fn.argtypes = argtypes
    fn.restype = restype
    return fn


# ---------------------------------------------------------------------------
# thruster_mixer.h
# ---------------------------------------------------------------------------

def mix_diff_drive(surge, yaw):
    """Mixer differential drive (C). Return (pwm_kiri, pwm_kanan) µs.

    surge [-1..1] maju, yaw [-1..1] kanan. Output 1100..1900 µs.
    """
    fn = _sig("mixer_diff_drive", [_D, _D,
                                   ctypes.POINTER(ctypes.c_int),
                                   ctypes.POINTER(ctypes.c_int)], None)
    kiri = ctypes.c_int(1500)
    kanan = ctypes.c_int(1500)
    # POINTER (bukan byref) agar cocok argtypes POINTER.
    fn(float(surge), float(yaw), ctypes.pointer(kiri), ctypes.pointer(kanan))
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
    if len(raw8) != 8:
        raise ValueError("frame ECU harus tepat 8 byte")
    fn = _sig("ecu_parse_frame", [ctypes.c_char_p,
                                  ctypes.POINTER(_EcuStatusC)], _I)
    buf = ctypes.create_string_buffer(bytes(raw8), 8)
    out = _EcuStatusC()
    ok = fn(buf, ctypes.byref(out))
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
    fn = _sig("ecu_build_cmd", [_I], ctypes.c_ubyte)
    return int(fn(int(cmd)))


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
    fn = _sig("arb_select", [_I, _I, _D, _D, _D, _D,
                             ctypes.POINTER(_ArbOutC)], None)
    out = _ArbOutC()
    fn(int(bool(kill_active)), int(bool(manual_active)),
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


# ---------------------------------------------------------------------------
# control_filters.h — PID + complementary + EKF (pengganti app/filtering.py)
# ---------------------------------------------------------------------------

# Urutan field = struct cf_pid_t (C).
class _CfPidC(ctypes.Structure):
    _fields_ = [
        ("kp", _D), ("ki", _D), ("kd", _D),
        ("deadband", _D),
        ("output_limit", _D),
        ("integral_limit", _D),
        ("integral", _D),
        ("last_error", _D),
    ]


class PidState:
    """State PID di memori Python, hitungan di C (cf_pid_update)."""

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, deadband=0.0,
                 output_limit=0.6, integral_limit=0.3):
        self._s = _CfPidC(float(kp), float(ki), float(kd),
                          float(deadband), float(output_limit),
                          float(integral_limit), 0.0, 0.0)

    @property
    def kp(self):
        return self._s.kp

    @kp.setter
    def kp(self, v):
        self._s.kp = float(v)

    def reset(self):
        _sig("cf_pid_reset", [ctypes.POINTER(_CfPidC)], None)(
            ctypes.byref(self._s))

    def update(self, error, dt=0.05):
        fn = _sig("cf_pid_update",
                  [ctypes.POINTER(_CfPidC), _D, _D], _D)
        return float(fn(ctypes.byref(self._s), float(error), float(dt)))


def complementary_filter(alpha, angle_prev, gyro_rate, dt, angle_measured):
    """Fusi heading komplementer (C). Lihat control_filters.h."""
    fn = _sig("cf_complementary", [_D, _D, _D, _D, _D], _D)
    return float(fn(float(alpha), float(angle_prev), float(gyro_rate),
                    float(dt), float(angle_measured)))


# Urutan field = struct cf_ekf_t (C).
class _CfEkfC(ctypes.Structure):
    _fields_ = [
        ("q_heading", _D), ("q_bias", _D),
        ("r_meas", _D),
        ("heading", _D), ("bias", _D),
        ("p00", _D), ("p01", _D), ("p11", _D),
    ]


class EkfState:
    """State EKF heading di memori Python, hitungan di C."""

    def __init__(self, process_noise=0.05, meas_noise=0.10,
                 init_heading=0.0):
        self._s = _CfEkfC()
        _sig("cf_ekf_init", [ctypes.POINTER(_CfEkfC), _D, _D, _D], None)(
            ctypes.byref(self._s), float(process_noise),
            float(meas_noise), float(init_heading))

    def predict(self, gyro_rate, dt):
        fn = _sig("cf_ekf_predict",
                  [ctypes.POINTER(_CfEkfC), _D, _D], _D)
        return float(fn(ctypes.byref(self._s), float(gyro_rate), float(dt)))

    def update(self, measured_heading):
        fn = _sig("cf_ekf_update", [ctypes.POINTER(_CfEkfC), _D], _D)
        return float(fn(ctypes.byref(self._s), float(measured_heading)))

    @property
    def heading(self):
        fn = _sig("cf_ekf_heading", [ctypes.POINTER(_CfEkfC)], _D)
        return float(fn(ctypes.byref(self._s)))


# ---------------------------------------------------------------------------
# nav_math.h — geodesi (pengganti app/geo.py di jalur produksi)
# ---------------------------------------------------------------------------

def nav_normalize_angle(angle_rad):
    """Bawa sudut ke [-pi, pi] (C)."""
    fn = _sig("nav_normalize_angle", [_D], _D)
    return float(fn(float(angle_rad)))


def nav_distance_bearing(lat1, lon1, lat2, lon2):
    """Jarak (m) & bearing (rad) antar koordinat (C)."""
    fn = _sig("nav_distance_bearing", [_D, _D, _D, _D,
                                       ctypes.POINTER(_D)], _D)
    b = _D(0.0)
    dist = fn(float(lat1), float(lon1), float(lat2), float(lon2),
              ctypes.byref(b))
    return float(dist), float(b.value)


def nav_cross_track_distance(lat_p, lon_p, lat_wp1, lon_wp1,
                             lat_wp2, lon_wp2):
    """Simpangan titik P dari garis WP1->WP2, meter (C)."""
    fn = _sig("nav_cross_track_distance",
              [_D, _D, _D, _D, _D, _D], _D)
    return float(fn(float(lat_p), float(lon_p), float(lat_wp1),
                    float(lon_wp1), float(lat_wp2), float(lon_wp2)))


def nav_destination(lat, lon, distance_m, bearing_rad):
    """Koordinat tujuan setelah jalan distance_m meter arah bearing (C)."""
    fn = _sig("nav_destination", [_D, _D, _D, _D,
                                 ctypes.POINTER(_D),
                                 ctypes.POINTER(_D)], None)
    o_lat = _D(0.0)
    o_lon = _D(0.0)
    fn(float(lat), float(lon), float(distance_m), float(bearing_rad),
       ctypes.byref(o_lat), ctypes.byref(o_lon))
    return float(o_lat.value), float(o_lon.value)


# ---------------------------------------------------------------------------
# fuzzy.h — gain P dinamis Sugeno (pengganti app/fuzzy.py di produksi)
# ---------------------------------------------------------------------------

def fuzzy_gate_gain(jarak_m, error_px):
    """Gain P gate (C). Semesta: 0..2 m, 0..320 px."""
    fn = _sig("fuzzy_gate_p_gain", [_D, _D], _D)
    return float(fn(float(jarak_m), float(error_px)))


def fuzzy_docking_gain(jarak_m, error_px):
    """Gain P docking (C). Semesta: 0..10 m, 0..320 px."""
    fn = _sig("fuzzy_docking_p_gain", [_D, _D], _D)
    return float(fn(float(jarak_m), float(error_px)))


# ---------------------------------------------------------------------------
# rc_decode.h — decode RC_CHANNELS (pengganti logika PWM di Python)
# ---------------------------------------------------------------------------

# Urutan field = struct rc_decoded_t (C).
class _RcDecodedC(ctypes.Structure):
    _fields_ = [
        ("surge", _D), ("yaw", _D),
        ("mode_manual", _I), ("deadman", _I), ("rc_ok", _I),
    ]


def rc_decode(ch_list, ch_throttle=3, ch_yaw=4, ch_mode=5, ch_deadman=7):
    """Decode list PWM (µs) -> dict surge/yaw/mode/deadman/rc_ok (C)."""
    n = len(ch_list)
    arr = (_I * max(n, 1))(*[int(v) for v in ch_list]) if n else None
    fn = _sig("rc_decode", [ctypes.POINTER(_I), _I, _I, _I, _I, _I,
                            ctypes.POINTER(_RcDecodedC)], None)
    out = _RcDecodedC()
    fn(arr, int(n), int(ch_throttle), int(ch_yaw),
       int(ch_mode), int(ch_deadman), ctypes.byref(out))
    return {
        "surge": float(out.surge),
        "yaw": float(out.yaw),
        "mode_manual": bool(out.mode_manual),
        "deadman": bool(out.deadman),
        "rc_ok": bool(out.rc_ok),
    }


# ---------------------------------------------------------------------------
# manual_control.h — kurva stick manual
# ---------------------------------------------------------------------------

# Urutan field = struct manual_state_t (C).
class _ManualStateC(ctypes.Structure):
    _fields_ = [("prev_surge", _D), ("prev_yaw", _D)]


# Urutan field = struct manual_out_t (C).
class _ManualOutC(ctypes.Structure):
    _fields_ = [("surge", _D), ("yaw", _D), ("active", _I)]


class ManualState:
    """Memori rate-limiter manual; hitungan di C (manual_update)."""

    def __init__(self):
        self._s = _ManualStateC(0.0, 0.0)

    def reset(self):
        _sig("manual_reset", [ctypes.POINTER(_ManualStateC)], None)(
            ctypes.byref(self._s))

    def update(self, in_surge, in_yaw, enabled, deadman, rc_ok,
               deadband, expo, rate_limit, dt, max_surge, max_yaw):
        fn = _sig("manual_update",
                  [ctypes.POINTER(_ManualStateC),
                   _D, _D, _I, _I, _I, _D, _D, _D, _D, _D, _D,
                   ctypes.POINTER(_ManualOutC)], None)
        out = _ManualOutC()
        fn(ctypes.byref(self._s), float(in_surge), float(in_yaw),
           int(bool(enabled)), int(bool(deadman)), int(bool(rc_ok)),
           float(deadband), float(expo), float(rate_limit), float(dt),
           float(max_surge), float(max_yaw), ctypes.byref(out))
        return {"surge": float(out.surge), "yaw": float(out.yaw),
                "active": bool(out.active)}


# ---------------------------------------------------------------------------
# mode_manager.h — state AUTO/MANUAL/HOLD/KILL
# ---------------------------------------------------------------------------

OP_AUTO = 0
OP_MANUAL = 1
OP_HOLD = 2
OP_KILL = 3
OP_NAMES = {0: "AUTO", 1: "MANUAL", 2: "HOLD", 3: "KILL"}


class ModeState:
    """State mode manager; hitungan di C (mode_update)."""

    def __init__(self):
        # struct mode_state_t = {mode:int, hold_timer:double}
        class _S(ctypes.Structure):
            _fields_ = [("mode", _I), ("hold_timer_s", _D)]
        self._C = _S
        self._s = _S(OP_AUTO, 0.0)

    def reset(self):
        _sig("mode_reset", [ctypes.POINTER(self._C)], None)(
            ctypes.byref(self._s))

    def update(self, kill, manual_req, rc_ok, dt, lost_hold_s):
        fn = _sig("mode_update",
                  [ctypes.POINTER(self._C), _I, _I, _I, _D, _D], _I)
        m = fn(ctypes.byref(self._s), int(bool(kill)),
               int(bool(manual_req)), int(bool(rc_ok)),
               float(dt), float(lost_hold_s))
        return {"mode": int(m), "name": OP_NAMES.get(int(m), "?")}
