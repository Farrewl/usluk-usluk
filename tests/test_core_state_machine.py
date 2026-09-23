"""Test kunci — buktikan bahwa C (state_machine.c) menghasilkan trace yang
SAMA dengan Python (app/state_machine.py) untuk skenario misi nyata.

Cara kerja (sama dengan pola test_core_nav_math / test_core_fuzzy):
  1. Compile core/src/state_machine.c -> libstate_machine.so (gcc, sementara).
  2. Bungkus struct sm_input_t/sm_result_t dengan ctypes (urutan field
     identik — lihat header state_machine.h).
  3. Jalankan SKENARIO yang sama pada kedua implementasi: state + retreat_step
     + wp_idx ditelusuri identik (wp_inc diterapkan sama). Bandingkan setiap
     hasil transisi (next, wp_inc, reset_task_timer, reset_vision,
     retreat_step) baris per baris.

Jalankan:  python3 -m unittest discover -s tests -v
"""

import ctypes
import os
import shutil
import subprocess
import tempfile
import unittest

from app.state_machine import (
    ALL_STATES,
)
from app.state_machine import (
    APPROACH_BLUE_BOX_ALIGN,
    APPROACH_BLUE_BOX_SEARCH,
    APPROACH_BOX_ALIGN,
    APPROACH_BOX_SEARCH,
    APPROACH_RED_BOX_ALIGN,
    APPROACH_RED_BOX_SEARCH,
    BLUE_BOX_RETREAT,
    MISSION_COMPLETE,
    RED_BOX_DOCKED,
    RETREAT,
    SR_GOTO_NEUTRAL,
    SR_IDLE,
    SR_START_BRAKE,
    SR_START_REVERSE,
    TAKE_BLUE_BOX_PHOTO,
    TAKE_PHOTO,
    TAKE_WAYPOINT_PHOTO,
    WAYPOINT_NAV,
    WAYPOINT_TRANSITION,
    SmInput,
    sm_guard,
    sm_transition,
)

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_C = os.path.join(REPO, "core", "src", "state_machine.c")
INC_C = os.path.join(REPO, "core", "include")

_STATES = {name: i for i, name in enumerate(ALL_STATES)}
_STEPS = {"IDLE": 0, "START_BRAKE": 1, "GOTO_NEUTRAL": 2, "START_REVERSE": 3}

_INPUT_FIELDS = [
    "wp_idx", "n_wp", "at_waypoint", "wp_in_photo_box_legs",
    "wp_in_blue_box_legs", "wp_in_stop_and_photo", "wp_is_red_after",
    "green_model", "blue_model", "red_model",
    "box_found", "box_confirmed", "box_aligned", "box_lost_over_2s",
    "blue_found", "blue_confirmed", "blue_aligned", "blue_lost_over_2s",
    "red_found", "red_aligned",
    "transition_over", "wp_photo_over", "blue_photo_over",
    "brake_over", "neutral_over", "retreat_over", "dock_hold_over",
]


class CInput(ctypes.Structure):
    """Bayangan sm_input_t (urutan field harus sama persis)."""
    _fields_ = [(f, ctypes.c_int) for f in _INPUT_FIELDS]


class CResult(ctypes.Structure):
    """Bayangan sm_result_t."""
    _fields_ = [("next", ctypes.c_int),
                ("wp_inc", ctypes.c_int),
                ("reset_task_timer", ctypes.c_int),
                ("reset_vision", ctypes.c_int),
                ("retreat_step", ctypes.c_int)]


class CSm:
    """Wrapper library C dengan API kecil untuk dites."""

    def __init__(self, lib):
        self.lib = lib
        lib.sm_transition.argtypes = [ctypes.c_int, ctypes.c_int,
                                      ctypes.POINTER(CInput)]
        lib.sm_transition.restype = CResult
        lib.sm_guard.argtypes = [ctypes.c_int] * 5
        lib.sm_guard.restype = ctypes.c_int
        lib.sm_state_name.argtypes = [ctypes.c_int]
        lib.sm_state_name.restype = ctypes.c_char_p
        lib.sm_retreat_step_name.argtypes = [ctypes.c_int]
        lib.sm_retreat_step_name.restype = ctypes.c_char_p

    def transition(self, state, retreat_step, inp):
        c_in = CInput(**{f: int(getattr(inp, f, 0) or 0)
                         for f in _INPUT_FIELDS})
        r = self.lib.sm_transition(_STATES[state], _STEPS[retreat_step],
                                   ctypes.byref(c_in))
        return (next_name(r.next), int(r.wp_inc), int(r.reset_task_timer),
                int(r.reset_vision), step_name(r.retreat_step))

    def guard(self, gps, yaw, loaded, wp_idx, n_wp):
        v = self.lib.sm_guard(int(gps), int(yaw), int(loaded),
                              int(wp_idx), int(n_wp))
        return next_name(v) if v >= 0 else None

    def state_name_c(self, enum_value):
        return self.lib.sm_state_name(int(enum_value)).decode()


def next_name(enum_value):
    if enum_value < 0:
        return None
    return ALL_STATES[enum_value]


def step_name(value):
    return {0: SR_IDLE, 1: SR_START_BRAKE, 2: SR_GOTO_NEUTRAL,
            3: SR_START_REVERSE}[value]


def _build_sm_lib():
    cc = shutil.which("gcc") or shutil.which("cc")
    if not cc:
        raise unittest.SkipTest("gcc tidak tersedia di mesin ini")
    tmp = tempfile.mkdtemp(prefix="libsm_")
    so = os.path.join(tmp, "libstate_machine.so")
    subprocess.run([cc, "-O2", "-fPIC", "-shared", "-I", INC_C,
                    SRC_C, "-o", so],
                   check=True, capture_output=True)
    return CSm(ctypes.CDLL(so))


def _py_transition(state, retreat_step, inp):
    r = sm_transition(state, retreat_step, inp)
    return (r.next_state, int(r.wp_inc), int(r.reset_task_timer),
            int(r.reset_vision), r.retreat_step)


class TestCStateMachineVsPython(unittest.TestCase):
    """Cross-check identik pada skenario misi lengkap."""

    @classmethod
    def setUpClass(cls):
        cls.c = _build_sm_lib()

    def _run_trace(self, scenario_steps, start_state=WAYPOINT_NAV):
        """Lacak kedua implementasi dengan state/wp/step identik.

        scenario_steps: list (SmInput, ) saja — runner menelusuri state
        sendiri (sama seperti navigator). Kembalikan list hasil tuple.
        """
        trace = []
        for py, c in ((True, False), (False, True)):
            state = start_state
            step = SR_IDLE
            wp_idx = 0
            hasil = []
            for inp in scenario_steps:
                inp = type(inp)(**{f: getattr(inp, f) for f in _INPUT_FIELDS})
                inp.wp_idx = wp_idx
                if inp.n_wp == 0:
                    inp.n_wp = 10
                if py:
                    nxt, wi, rtt, rv, st = _py_transition(state, step, inp)
                else:
                    nxt, wi, rtt, rv, st = self.c.transition(state, step, inp)
                if nxt == MISSION_COMPLETE:
                    pass
                state = nxt
                step = st
                wp_idx += wi
                hasil.append((nxt, wi, rtt, rv, st, wp_idx))
            trace.append(hasil)
        return trace[0], trace[1]

    def _assert_trace_sama(self, scenario_steps, tag):
        py, c = self._run_trace(scenario_steps)
        self.assertEqual(len(py), len(c), f"{tag}: panjang trace beda")
        for i, (a, b) in enumerate(zip(py, c)):
            self.assertEqual(a, b,
                             f"{tag}: langkah {i}: py={a} vs C={b}")

    # ---------------- skenario ----------------

    def test_skenario_gate_nav(self):
        steps = []
        # 5 waypoint, tiba di tiap wp -> TRANSITION -> NAV
        for _ in range(5):
            steps.append(SmInput(at_waypoint=1))
            steps.append(SmInput(transition_over=0))
            steps.append(SmInput(transition_over=1))
        self._assert_trace_sama(steps, "gate-nav")

    def test_skenario_foto_box_hijau(self):
        steps = [
            SmInput(wp_in_photo_box_legs=1, green_model=1),
            SmInput(box_found=1, box_confirmed=0),
            SmInput(box_found=1, box_confirmed=1),
            SmInput(box_found=1, box_aligned=0),
            SmInput(box_found=1, box_aligned=1),
            SmInput(),
            SmInput(brake_over=0),
            SmInput(brake_over=1),
            SmInput(neutral_over=0),
            SmInput(neutral_over=1),
            SmInput(retreat_over=0),
            SmInput(retreat_over=1),
            SmInput(at_waypoint=1),  # lanjut leg berikutnya
        ]
        self._assert_trace_sama(steps, "foto-box")

    def test_skenario_biru_dan_docking(self):
        steps = [
            SmInput(wp_in_blue_box_legs=1, blue_model=1),
            SmInput(blue_found=1, blue_confirmed=0),
            SmInput(blue_found=1, blue_confirmed=1),
            SmInput(blue_found=1, blue_aligned=0),
            SmInput(blue_found=1, blue_aligned=1),
            SmInput(blue_photo_over=0),
            SmInput(blue_photo_over=1),
            SmInput(brake_over=1),
            SmInput(neutral_over=1),
            SmInput(retreat_over=1, red_model=1),
            SmInput(red_found=0),
            SmInput(red_found=1),
            SmInput(red_found=1, red_aligned=0),
            SmInput(red_found=1, red_aligned=1),
            SmInput(dock_hold_over=0),
            SmInput(dock_hold_over=1),
        ]
        self._assert_trace_sama(steps, "biru-docking")

    def test_skenario_hilang_target(self):
        steps = [
            SmInput(wp_in_photo_box_legs=1, green_model=1),
            SmInput(box_found=1, box_confirmed=1),
            SmInput(box_found=0, box_lost_over_2s=0),   # hilang < 2s
            SmInput(box_found=0, box_lost_over_2s=1),   # hilang > 2s
            SmInput(box_found=1, box_aligned=1),
            SmInput(),                                   # TAKE_PHOTO -> RETREAT
            SmInput(brake_over=1),
            SmInput(neutral_over=1),
            SmInput(retreat_over=1),
        ]
        self._assert_trace_sama(steps, "hilang-target")

    def test_skenario_stop_photo_dan_red_after(self):
        steps = [
            SmInput(at_waypoint=1, wp_in_stop_and_photo=1),
            SmInput(wp_photo_over=0),
            SmInput(wp_photo_over=1, wp_is_red_after=1, red_model=1),
            SmInput(red_found=0),
            SmInput(red_found=1),
            SmInput(red_found=1, red_aligned=1),
            SmInput(dock_hold_over=1),
        ]
        self._assert_trace_sama(steps, "stop-foto-red-after")

    def test_retreat_error_step_sama(self):
        # Step IDLE saat di RETREAT = kesalahan internal -> kembali WP_NAV.
        # Dibandingkan langsung (bukan trace) karena butuh step awal IDLE.
        inp = SmInput()
        self.assertEqual(self.c.transition(RETREAT, SR_IDLE, inp),
                         _py_transition(RETREAT, SR_IDLE, inp))
        # Step tak dikenal juga harus jatuh ke jalur yang sama.
        self.assertEqual(self.c.transition(BLUE_BOX_RETREAT, SR_IDLE, inp),
                         _py_transition(BLUE_BOX_RETREAT, SR_IDLE, inp))

    def test_guard_c_vs_py(self):
        kasus = [(0, 1, 1, 0, 5), (1, 0, 1, 0, 5), (1, 1, 0, 0, 5),
                 (1, 1, 1, 5, 5), (1, 1, 1, 4, 5), (1, 1, 1, 0, 0)]
        for gps, yaw, loaded, wp, n in kasus:
            self.assertEqual(self.c.guard(gps, yaw, loaded, wp, n),
                             sm_guard(gps, yaw, loaded, wp, n),
                             f"guard({gps},{yaw},{loaded},{wp},{n})")

    def test_nama_enum_sama(self):
        for name in ALL_STATES:
            self.assertEqual(self.c.state_name_c(_STATES[name]), name)


if __name__ == "__main__":
    unittest.main()