"""Lapisan tipis Python -> C (ctypes only, TANPA logika hitung).

Semua hitungan (PID, EKF, nav, fuzzy, RC, manual, mixer, arbitrator,
gate-vision) dijalankan fungsi C di core/libaterkia.so (Linux) atau
core/aterkia_core.dll (Windows). Python hanya meneruskan angka & struct
— tidak ada rumus yang diduplikasi di sini.

Build (lihat core/Makefile):
  Linux  : make -C core linux
           gcc -shared -fPIC -O2 -Icore/include core/src/*.c \
               -o core/libaterkia.so -lm
  Windows: mingw32-make -C core windows  (MinGW: menghasilkan
           core/aterkia_core.dll dari source yang sama)
(gamepad_input.cpp hanya dipakai via CMake static lib, bukan lib ini.)

Bila library belum ada: C_AVAILABLE=False, pemanggil WAJIB gagal
eksplisit (jangan fallback diam-diam ke logika Python ganda).
"""

import ctypes
import os
import platform

APP_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(APP_DIR)

# Nama library beda per OS: Linux gcc -> libaterkia.so,
# Windows (MinGW) -> aterkia_core.dll. Lihat core/Makefile.
if platform.system() == "Windows":
    _LIB_NAME = "aterkia_core.dll"
    _BUILD_HINT = (
        "aterkia_core.dll belum di-build. Jalankan (MinGW): "
        "mingw32-make -C core windows  — atau lihat core/Makefile.")
else:
    _LIB_NAME = "libaterkia.so"
    _BUILD_HINT = (
        "libaterkia.so belum di-build. Jalankan: "
        "gcc -shared -fPIC -O2 -Icore/include core/src/*.c "
        "-o core/libaterkia.so -lm")
LIB_PATH = os.path.join(ROOT_DIR, "core", _LIB_NAME)

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
        raise RuntimeError(_BUILD_HINT)


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


# Nama sumber manual ganda — sama dengan ARB_MANUAL_* di arbitrator.h.
MANUAL_NONE = 0
MANUAL_QGC = 1
MANUAL_LOCAL = 2
MANUAL_CONFLICT = 3
MANUAL2_NAMES = {0: "NONE", 1: "QGC", 2: "LOKAL", 3: "KONFLIK"}


class _ArbManualC(ctypes.Structure):
    _fields_ = [
        ("surge", ctypes.c_double),
        ("yaw", ctypes.c_double),
        ("source", ctypes.c_int),
        ("conflict", ctypes.c_int),
    ]


def arb_manual2(qgc_active, qgc_surge, qgc_yaw,
                loc_active, loc_surge, loc_yaw):
    """Pilih di antara 2 sumber manual QGC vs lokal (C).

    Return dict surge/yaw/source/source_name/conflict. Dua-duanya aktif
    = konflik kendali -> output netral (0,0) + conflict=True.
    """
    fn = _sig("arb_manual2", [_I, _D, _D, _I, _D, _D,
                              ctypes.POINTER(_ArbManualC)], None)
    out = _ArbManualC()
    fn(int(bool(qgc_active)), float(qgc_surge), float(qgc_yaw),
       int(bool(loc_active)), float(loc_surge), float(loc_yaw),
       ctypes.byref(out))
    return {
        "surge": float(out.surge),
        "yaw": float(out.yaw),
        "source": int(out.source),
        "source_name": MANUAL2_NAMES.get(int(out.source), "?"),
        "conflict": bool(out.conflict),
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


# ---------------------------------------------------------------------------
# avoidance.h — hindar-rintangan reaktif (bias yaw menjauh, hitung di C)
# ---------------------------------------------------------------------------

# Urutan field = struct avoid_state_t (C).
class _AvoidStateC(ctypes.Structure):
    _fields_ = [("yaw_bias", _D), ("memory_s", _D)]


def avoid_update(state, det_cx_norm, det_w_norm, dt_s):
    """Bias yaw menjauh dari rintangan besar di tengah frame (C).

    `state` dict {yaw_bias, memory_s} diupdate IN-PLACE dan dikembalikan.
    Input:
      det_cx_norm: pusat objek dinormalisasi [-1..1]; 0 = tengah frame
      det_w_norm : lebar objek dinormalisasi [0..1];  1 = selebar frame
      dt_s       : delta waktu frame (decay memori)
    Return dict {yaw_bias, memory_s, active} — active = |bias| >= 0.05.
    """
    fn = _sig("avoid_update",
              [ctypes.POINTER(_AvoidStateC), _D, _D, _D], None)
    s = _AvoidStateC(float(state.get("yaw_bias", 0.0)),
                     float(state.get("memory_s", 0.0)))
    fn(ctypes.byref(s), float(det_cx_norm), float(det_w_norm), float(dt_s))
    out = {"yaw_bias": float(s.yaw_bias), "memory_s": float(s.memory_s),
           "active": abs(float(s.yaw_bias)) >= 0.05}
    state.clear()
    state.update(out)
    return out


# ---------------------------------------------------------------------------
# gate_vision.h — koleksi pasangan + jarak pinhole + kriteria geometri buoy
# + sequencer latch (pengganti app/gate_sequencer.py di jalur produksi).
# Warna HSV tetap di Python (butuh citra); modul ini hanya geometri.
# ---------------------------------------------------------------------------

# Batas kompilasi (sama dengan #define GV_* di gate_vision.h).
GV_MAX_DET = 64
GV_MAX_PAIRS = 64
GV_MEM_PER_CLS = 32


# Urutan field = struct gv_ball_t (C).
class _GvBallC(ctypes.Structure):
    _fields_ = [("cx", _D), ("cy", _D), ("area", _D)]


# Urutan field = struct gv_pair_t (C).
class _GvPairC(ctypes.Structure):
    _fields_ = [("red_idx", _I), ("green_idx", _I)]


# Urutan field = struct gv_mem_t (C).
class _GvMemC(ctypes.Structure):
    _fields_ = [("cx", _D), ("cy", _D), ("area", _D), ("unseen", _I)]


# Urutan field = struct gv_seq_t (C) — layout terverifikasi 2160 byte.
class _GvSeqC(ctypes.Structure):
    _fields_ = [
        ("pass_distance_m", _D),
        ("lost_tolerance_frames", _I),
        ("gate_width_m", _D),
        ("focal_length_px", _D),
        ("midpoint_match_px", _D),
        ("track_match_px", _D),
        ("track_boost", _D),
        ("has_active", _I),
        ("active_mid_x", _D),
        ("active_mid_y", _D),
        ("active_dist", _D),
        ("active_red", _I),
        ("active_green", _I),
        ("lost_frames", _I),
        ("is_passed", _I),
        ("mem", _GvMemC * 2 * GV_MEM_PER_CLS),
        ("mem_n", _I * 2),
    ]


def _balls_to_c(balls):
    """List dict {cx, cy, area} -> array C (maks GV_MAX_DET)."""
    n = min(len(balls), GV_MAX_DET)
    arr = (_GvBallC * max(n, 1))()
    for i in range(n):
        arr[i].cx = float(balls[i].get("cx", 0.0))
        arr[i].cy = float(balls[i].get("cy", 0.0))
        arr[i].area = float(balls[i].get("area", 0.0))
    return arr, n


def collect_gate_pairs_c(red_balls, green_balls, vertical_align_px,
                         area_similarity_ratio):
    """Kumpulkan pasangan gate (C). Return list (red_ball, green_ball)."""
    red_arr, n_red = _balls_to_c(red_balls)
    grn_arr, n_grn = _balls_to_c(green_balls)
    out = (_GvPairC * GV_MAX_PAIRS)()
    fn = _sig("gv_collect_pairs",
              [ctypes.POINTER(_GvBallC), _I,
               ctypes.POINTER(_GvBallC), _I, _D, _D,
               ctypes.POINTER(_GvPairC)], _I)
    n = fn(red_arr, n_red, grn_arr, n_grn,
           float(vertical_align_px), float(area_similarity_ratio), out)
    pairs = []
    for i in range(max(0, min(int(n), GV_MAX_PAIRS))):
        ri, gi = int(out[i].red_idx), int(out[i].green_idx)
        if 0 <= ri < len(red_balls) and 0 <= gi < len(green_balls):
            pairs.append((red_balls[ri], green_balls[gi]))
    return pairs


def estimate_gate_distance_c(pixel_width, gate_width_m, focal_length_px):
    """Jarak gate pinhole (C); +inf bila tak reliabel."""
    fn = _sig("gv_estimate_distance", [_D, _D, _D], _D)
    return float(fn(float(pixel_width), float(gate_width_m),
                    float(focal_length_px)))


def buoy_geometry_ok_c(w, h, min_area, max_aspect_deviation):
    """Kriteria geometri buoy (C): 1 = lolos, 0 = tolak."""
    fn = _sig("gv_buoy_ok", [_D, _D, _D, _D], _I)
    return bool(fn(float(w), float(h), float(min_area),
                   float(max_aspect_deviation)))


def buoy_geometry_fail_c(w, h, min_area, max_aspect_deviation):
    """Alasan penolakan geometri buoy (C): 0 = lolos, 1 = terlalu kecil,
    2 = area < min_area, 3 = aspek menyimpang."""
    fn = _sig("gv_buoy_fail", [_D, _D, _D, _D], _I)
    return int(fn(float(w), float(h), float(min_area),
                  float(max_aspect_deviation)))


def yaw_correction_c(target_x_px, image_center_x, distance_m,
                     focal_length_px):
    """Koreksi yaw mentah (rad) dari error titik tengah piksel (C).

    0.0 bila distance_m < 0.1 atau focal <= 0 (tak reliabel).
    """
    fn = _sig("gv_yaw_correction", [_D, _D, _D, _D], _D)
    return float(fn(float(target_x_px), float(image_center_x),
                    float(distance_m), float(focal_length_px)))


class GateSequencerC:
    """Sequencer gate; state di memori Python, hitungan di C (gv_seq_*)."""

    def __init__(self, pass_distance_m=1.2, lost_tolerance_frames=5,
                 gate_width_m=1.0, focal_length_px=400.0,
                 midpoint_match_px=6.0, track_match_px=30.0,
                 track_boost=1.5):
        self._C = _GvSeqC
        self._s = _GvSeqC()
        self._pair_cache = []  # pasangan frame terakhir (untuk active_pair)
        _sig("gv_seq_init",
             [ctypes.POINTER(self._C), _D, _I, _D, _D, _D, _D, _D],
             None)(ctypes.byref(self._s), float(pass_distance_m),
                   int(lost_tolerance_frames), float(gate_width_m),
                   float(focal_length_px), float(midpoint_match_px),
                   float(track_match_px), float(track_boost))

    def reset(self):
        _sig("gv_seq_reset", [ctypes.POINTER(self._C)], None)(
            ctypes.byref(self._s))
        self._pair_cache = []

    @property
    def active_pair(self):
        """Pasangan aktif (seperti GateSequencer Python) atau None."""
        if not self._s.has_active:
            return None
        ri, gi = int(self._s.active_red), int(self._s.active_green)
        reds = getattr(self, "_last_red", [])
        grns = getattr(self, "_last_green", [])
        if 0 <= ri < len(reds) and 0 <= gi < len(grns):
            return (reds[ri], grns[gi])
        for r, g in self._pair_cache:
            if r is not None and g is not None:
                return (r, g)
        return None

    def update(self, red_balls, green_balls, pairs, image_center_y):
        """Perbarui antrean (C). Return (mid_x, mid_y, dist, is_passed).

        `pairs` = list (red_ball, green_ball) dari collect_gate_pairs_c.
        Bila kosong -> (None, None, inf, False).
        """
        self._last_red = list(red_balls)
        self._last_green = list(green_balls)
        self._pair_cache = list(pairs)
        red_arr, n_red = _balls_to_c(red_balls)
        grn_arr, n_grn = _balls_to_c(green_balls)
        # Petakan pasangan ke index array C (cari posisi dict yang sama).
        parr = (_GvPairC * max(len(pairs), 1))()
        for i, (r, g) in enumerate(pairs[:GV_MAX_PAIRS]):
            try:
                ri = list(red_balls).index(r)
            except ValueError:
                ri = -1
            try:
                gi = list(green_balls).index(g)
            except ValueError:
                gi = -1
            if ri < 0 or gi < 0:
                continue
            parr[i].red_idx = ri
            parr[i].green_idx = gi
        fn = _sig("gv_seq_update",
                  [ctypes.POINTER(self._C),
                   ctypes.POINTER(_GvBallC), _I,
                   ctypes.POINTER(_GvBallC), _I,
                   ctypes.POINTER(_GvPairC), _I, _D,
                   ctypes.POINTER(_D), ctypes.POINTER(_D),
                   ctypes.POINTER(_D), ctypes.POINTER(_I)], _I)
        mx, my, dist = _D(0.0), _D(0.0), _D(float("inf"))
        ps = _I(0)
        has = fn(ctypes.byref(self._s), red_arr, n_red, grn_arr, n_grn,
                 parr, len(pairs), float(image_center_y),
                 ctypes.byref(mx), ctypes.byref(my),
                 ctypes.byref(dist), ctypes.byref(ps))
        if not has:
            return None, None, float("inf"), False
        return float(mx.value), float(my.value), float(dist.value), bool(ps.value)
