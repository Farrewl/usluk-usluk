"""Konektor RC/gamepad -> C (tipis, TANPA rumus).

Mengambil PWM mentah RC_CHANNELS dari Pixhawk (pymavlink) + joystick USB
(device Linux), meneruskan ke core C (rc_decode + manual_control +
mode_manager) via app/aterkia_core.py, mengembalikan setpoint manual siap
arbitrasi. Semua ambang & kurva hidup di C — file ini hanya kabel.
"""

import glob
import os
import time

try:
    from . import aterkia_core as core
except Exception:  # pragma: no cover - import parsial saat uji
    core = None

# Cache fd gamepad (dibuka malas, non-blocking, via ctypes C++).
_GAMEPAD_LIB = None
_GAMEPAD_FD = -1
_GAMEPAD_PATH = ""


def _gamepad_lib():
    """Muat lib gamepad C++ (build CMake) bila ada; None bila tidak."""
    global _GAMEPAD_LIB
    if _GAMEPAD_LIB is not None:
        return _GAMEPAD_LIB
    try:
        import ctypes
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for cand in (
                os.path.join(root, "core", "build", "libasv_gamepad_input.a"),
                os.path.join(root, "core", "build", "libasv_gamepad_input.so")):
            if os.path.exists(cand):
                _GAMEPAD_LIB = ctypes.CDLL(cand)
                _GAMEPAD_LIB.gamepad_open.argtypes = [ctypes.c_char_p]
                _GAMEPAD_LIB.gamepad_open.restype = ctypes.c_int
                _GAMEPAD_LIB.gamepad_poll.argtypes = [
                    ctypes.c_int, ctypes.POINTER(ctypes.c_double),
                    ctypes.POINTER(ctypes.c_double),
                    ctypes.POINTER(ctypes.c_int)]
                _GAMEPAD_LIB.gamepad_poll.restype = ctypes.c_int
                _GAMEPAD_LIB.gamepad_close.argtypes = [ctypes.c_int]
                _GAMEPAD_LIB.gamepad_close.restype = None
                return _GAMEPAD_LIB
    except Exception:
        pass
    _GAMEPAD_LIB = None
    return None


class ManualLink:
    """Jembatan manual: baca RC/gamepad, olah di C, kembalikan setpoint.

    Pemakaian (simulator/navigator, tiap loop 20 Hz):
        link = ManualLink(config)
        link.attach_mav(master)          # pymavlink connection (boleh None)
        out = link.update(dt)            # dict surge/yaw/mode/rc_ok/...
    """

    def __init__(self, config):
        self.config = config
        self.master = None
        self._ch = []                    # PWM terakhir (µs), maks 16
        self._last_rc_time = 0.0
        self._manual = None              # core.ManualState (dibuat malas)
        self._mode = None                # core.ModeState (dibuat malas)
        self._gamepad_path = ""
        self._last_out = {"surge": 0.0, "yaw": 0.0, "active": False,
                          "mode": 0, "mode_name": "AUTO",
                          "rc_ok": False, "source": "AUTO"}

    # ------------------- sumber input -------------------

    def attach_mav(self, master):
        """Sambungkan koneksi pymavlink Pixhawk (boleh None = RC mati)."""
        self.master = master

    def set_gamepad(self, path):
        """Pilih joystick USB (mis. /dev/input/js0; '' = mati)."""
        global _GAMEPAD_FD, _GAMEPAD_PATH
        if path == self._gamepad_path:
            return
        lib = _gamepad_lib()
        if _GAMEPAD_FD >= 0 and lib is not None:
            try:
                lib.gamepad_close(_GAMEPAD_FD)
            except Exception:
                pass
        _GAMEPAD_FD = -1
        _GAMEPAD_PATH = path or ""
        self._gamepad_path = path or ""

    @staticmethod
    def list_gamepads():
        """Daftar joystick USB yang ada (/dev/input/js*)."""
        return sorted(glob.glob("/dev/input/js*"))

    def _poll_mav_rc(self):
        """Ambil RC_CHANNELS terbaru (non-blocking); True bila ada frame."""
        if self.master is None:
            return False
        try:
            msg = self.master.recv_match(type="RC_CHANNELS",
                                         blocking=False)
        except Exception:
            return False
        if msg is None:
            return False
        ch = []
        for i in range(1, 17):
            v = getattr(msg, f"chan{i}_raw", 0) or 0
            ch.append(int(v))
        self._ch = ch
        self._last_rc_time = time.time()
        return True

    def _poll_gamepad(self):
        """Baca joystick USB; return (surge, yaw, deadman) atau None."""
        global _GAMEPAD_FD
        if not self._gamepad_path:
            return None
        lib = _gamepad_lib()
        if lib is None:
            return None
        try:
            import ctypes
            if _GAMEPAD_FD < 0:
                _GAMEPAD_FD = lib.gamepad_open(
                    self._gamepad_path.encode())
                if _GAMEPAD_FD < 0:
                    return None
            s = ctypes.c_double(0.0)
            y = ctypes.c_double(0.0)
            d = ctypes.c_int(0)
            rc = lib.gamepad_poll(_GAMEPAD_FD, ctypes.byref(s),
                                  ctypes.byref(y), ctypes.byref(d))
            if rc != 0:
                lib.gamepad_close(_GAMEPAD_FD)
                _GAMEPAD_FD = -1
                return None
            return float(s.value), float(y.value), bool(d.value)
        except Exception:
            return None

    # ------------------- update utama (tiap loop) -------------------

    def update(self, dt=0.05, kill=False, gui_manual=False):
        """Hitung setpoint manual (C). Return dict untuk arbitrator/GUI."""
        if core is None or not core.C_AVAILABLE:
            return dict(self._last_out)
        if self._manual is None:
            self._manual = core.ManualState()
        if self._mode is None:
            self._mode = core.ModeState()

        cfg = self.config
        self._poll_mav_rc()

        # Timeout RC: frame basi > RC_TIMEOUT_MS = hilang.
        timeout_s = max(0, float(cfg.RC_TIMEOUT_MS)) / 1000.0
        rc_fresh = (bool(self._ch) and
                    (time.time() - self._last_rc_time) <= timeout_s)

        # Decode RC (C): PWM -> [-1..1] + switch.
        if self._ch:
            dec = core.rc_decode(
                self._ch, cfg.RC_CH_THROTTLE, cfg.RC_CH_YAW,
                cfg.RC_CH_MODE, cfg.RC_CH_DEADMAN)
        else:
            dec = {"surge": 0.0, "yaw": 0.0, "mode_manual": False,
                   "deadman": False, "rc_ok": False}
        rc_ok = bool(dec["rc_ok"] and rc_fresh)

        # Gamepad (backup darat): timpa stick RC bila ada data.
        gp = self._poll_gamepad()
        in_surge, in_yaw = dec["surge"], dec["yaw"]
        deadman = dec["deadman"]
        if gp is not None:
            in_surge, in_yaw, gp_dead = gp
            deadman = bool(deadman or gp_dead)

        # Kurva stick (C): deadband+expo+rate-limit+batas gas.
        man = self._manual.update(
            in_surge, in_yaw, bool(cfg.MANUAL_ENABLED),
            bool(deadman), bool(rc_ok),
            float(cfg.MANUAL_DEADBAND), float(cfg.MANUAL_EXPO),
            float(cfg.MANUAL_RATE_LIMIT), float(dt),
            float(cfg.MANUAL_MAX_SURGE), float(cfg.MANUAL_MAX_YAW))

        # Mode (C): KILL > MANUAL > HOLD > AUTO.
        manual_req = bool(dec["mode_manual"] or gui_manual or man["active"])
        mode = self._mode.update(bool(kill), manual_req, bool(rc_ok),
                                 float(dt), float(cfg.MANUAL_LOST_HOLD_S))

        out = {"surge": man["surge"], "yaw": man["yaw"],
               "active": man["active"] and mode["mode"] == core.OP_MANUAL,
               "mode": mode["mode"], "mode_name": mode["name"],
               "rc_ok": rc_ok, "rc_channels": list(self._ch),
               "deadman": bool(deadman),
               "source": ("MANUAL" if mode["mode"] == core.OP_MANUAL
                          else ("KILL" if mode["mode"] == core.OP_KILL
                                else "AUTO"))}
        self._last_out = out
        return dict(out)
