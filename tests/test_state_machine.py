"""Test app/state_machine.py — referensi murni logika transisi misi.

Memvalidasi keputusan transisi yang diekstrak dari run() di navigator.py.
Skenario meniru perilaku kapal nyata (nilai threshold datang dari
telemetri/vision/timer — bukan konstanta di sini).

Jalankan:  python3 -m unittest discover -s tests -v
"""

import unittest

from app.state_machine import (
    APPROACH_BLUE_BOX_ALIGN,
    APPROACH_BLUE_BOX_SEARCH,
    APPROACH_BOX_ALIGN,
    APPROACH_BOX_SEARCH,
    APPROACH_RED_BOX_ALIGN,
    APPROACH_RED_BOX_SEARCH,
    BLUE_BOX_RETREAT,
    MISSION_COMPLETE,
    NO_TELEM,
    NO_WAYPOINTS,
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


def _nav(wp=0, n=5, **kw):
    return SmInput(wp_idx=wp, n_wp=n, **kw)


class TestGuard(unittest.TestCase):
    def test_tanpa_telemetri(self):
        self.assertEqual(sm_guard(0, 1, 1, 0, 5), NO_TELEM)
        self.assertEqual(sm_guard(1, 0, 1, 0, 5), NO_TELEM)

    def test_tanpa_waypoint(self):
        self.assertEqual(sm_guard(1, 1, 0, 0, 5), NO_WAYPOINTS)

    def test_misi_habis(self):
        self.assertEqual(sm_guard(1, 1, 1, 5, 5), MISSION_COMPLETE)

    def test_boleh_lanjut(self):
        self.assertIsNone(sm_guard(1, 1, 1, 4, 5))


class TestNavGate(unittest.TestCase):
    def test_nav_biasa_tetap(self):
        r = sm_transition(WAYPOINT_NAV, SR_IDLE, _nav(at_waypoint=0))
        self.assertEqual(r.next_state, WAYPOINT_NAV)
        self.assertEqual(r.wp_inc, 0)

    def test_sampai_wp_lanjut_transisi(self):
        r = sm_transition(WAYPOINT_NAV, SR_IDLE, _nav(at_waypoint=1))
        self.assertEqual(r.next_state, WAYPOINT_TRANSITION)
        self.assertEqual(r.wp_inc, 1)
        self.assertEqual(r.reset_task_timer, 1)
        self.assertEqual(r.reset_vision, 1)

    def test_wp_terakhir_selesai(self):
        r = sm_transition(WAYPOINT_NAV, SR_IDLE,
                          _nav(wp=4, at_waypoint=1))
        self.assertEqual(r.next_state, MISSION_COMPLETE)
        self.assertEqual(r.wp_inc, 1)

    def test_transisi_menunggu_waktu(self):
        r = sm_transition(WAYPOINT_TRANSITION, SR_IDLE,
                          SmInput(transition_over=0))
        self.assertEqual(r.next_state, WAYPOINT_TRANSITION)
        r = sm_transition(WAYPOINT_TRANSITION, SR_IDLE,
                          SmInput(transition_over=1))
        self.assertEqual(r.next_state, WAYPOINT_NAV)

    def test_foto_wp_stop(self):
        r = sm_transition(WAYPOINT_NAV, SR_IDLE,
                          _nav(wp=2, at_waypoint=1, wp_in_stop_and_photo=1))
        self.assertEqual(r.next_state, TAKE_WAYPOINT_PHOTO)
        self.assertEqual(r.reset_task_timer, 1)
        self.assertEqual(r.wp_inc, 0)

    def test_foto_wp_lanjut_atau_docking(self):
        r = sm_transition(TAKE_WAYPOINT_PHOTO, SR_IDLE,
                          _nav(wp=3, wp_photo_over=1,
                               wp_is_red_after=1, red_model=1))
        self.assertEqual(r.next_state, APPROACH_RED_BOX_SEARCH)
        self.assertNotEqual(r.next_state, WAYPOINT_TRANSITION)  # docking dulu
        r = sm_transition(TAKE_WAYPOINT_PHOTO, SR_IDLE,
                          _nav(wp=1, wp_photo_over=1))
        self.assertEqual(r.next_state, WAYPOINT_TRANSITION)
        self.assertEqual(r.wp_inc, 1)


class TestMisiFotoBox(unittest.TestCase):
    def test_urutan_full(self):
        steps = [
            _nav(wp=0, wp_in_photo_box_legs=1, green_model=1),
            SmInput(box_found=1, box_confirmed=0),
            SmInput(box_found=1, box_confirmed=1),
            SmInput(box_found=1, box_aligned=0),
            SmInput(box_found=1, box_aligned=1),
            SmInput(),
            SmInput(brake_over=1),
            SmInput(neutral_over=1),
            SmInput(retreat_over=0),
            SmInput(retreat_over=1),
        ]
        # states[i] = state BARU sesudah langkah ke-i (state awal WAYPOINT_NAV
        # tidak ikut dicek — dicek lewat hasil transisi).
        states = [APPROACH_BOX_SEARCH, APPROACH_BOX_SEARCH,
                  APPROACH_BOX_ALIGN, APPROACH_BOX_ALIGN, TAKE_PHOTO,
                  RETREAT, RETREAT, RETREAT, RETREAT, WAYPOINT_NAV]
        retreat = [SR_IDLE, SR_IDLE, SR_IDLE, SR_IDLE, SR_IDLE,
                   SR_START_BRAKE, SR_GOTO_NEUTRAL, SR_START_REVERSE,
                   SR_START_REVERSE, SR_IDLE]
        state = WAYPOINT_NAV
        step = SR_IDLE
        for i, inp in enumerate(steps):
            r = sm_transition(state, step, inp)
            state = r.next_state
            step = r.retreat_step
            self.assertEqual(r.next_state, states[i], f"langkah {i}")
            self.assertEqual(r.retreat_step, retreat[i], f"step {i}")

    def test_box_hilang_2s_ambil_foto(self):
        r = sm_transition(APPROACH_BOX_ALIGN, SR_IDLE,
                          SmInput(box_found=0, box_lost_over_2s=1))
        self.assertEqual(r.next_state, TAKE_PHOTO)


class TestMisiBirudanDocking(unittest.TestCase):
    def test_urutan_full(self):
        steps = [
            _nav(wp=0, n=10, wp_in_blue_box_legs=1, blue_model=1),
            SmInput(blue_found=1, blue_confirmed=1),
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
        states = [APPROACH_BLUE_BOX_SEARCH, APPROACH_BLUE_BOX_ALIGN,
                  TAKE_BLUE_BOX_PHOTO, TAKE_BLUE_BOX_PHOTO,
                  BLUE_BOX_RETREAT, BLUE_BOX_RETREAT, BLUE_BOX_RETREAT,
                  APPROACH_RED_BOX_SEARCH, APPROACH_RED_BOX_SEARCH,
                  APPROACH_RED_BOX_ALIGN, APPROACH_RED_BOX_ALIGN,
                  RED_BOX_DOCKED, RED_BOX_DOCKED, WAYPOINT_TRANSITION]
        state = WAYPOINT_NAV
        step = SR_IDLE
        wp_idx = 0
        for i, inp in enumerate(steps):
            inp.wp_idx = wp_idx  # snapshot wp selalu mengikuti kondisi saat ini
            if inp.n_wp == 0:
                inp.n_wp = 10
            r = sm_transition(state, step, inp)
            state = r.next_state
            step = r.retreat_step
            wp_idx += r.wp_inc
            self.assertEqual(r.next_state, states[i], f"langkah {i}")

    def test_blue_retreat_tanpa_model_merah(self):
        r = sm_transition(BLUE_BOX_RETREAT, SR_START_REVERSE,
                          SmInput(retreat_over=1, red_model=0))
        self.assertEqual(r.next_state, WAYPOINT_NAV)
        self.assertEqual(r.wp_inc, 1)


class TestRedDock(unittest.TestCase):
    def test_red_lost_kembali_search(self):
        r = sm_transition(APPROACH_RED_BOX_ALIGN, SR_IDLE,
                          SmInput(red_found=0))
        self.assertEqual(r.next_state, APPROACH_RED_BOX_SEARCH)

    def test_red_align_docked(self):
        r = sm_transition(APPROACH_RED_BOX_ALIGN, SR_IDLE,
                          SmInput(red_found=1, red_aligned=1))
        self.assertEqual(r.next_state, RED_BOX_DOCKED)
        self.assertEqual(r.reset_task_timer, 1)

    def test_dock_selesai_lanjut(self):
        r = sm_transition(RED_BOX_DOCKED, SR_IDLE,
                          SmInput(wp_idx=0, n_wp=10, dock_hold_over=1))
        self.assertEqual(r.next_state, WAYPOINT_TRANSITION)
        self.assertEqual(r.wp_inc, 1)


class TestStateTanpaTransisi(unittest.TestCase):
    def test_guard_states_tetap(self):
        for state in (MISSION_COMPLETE, "WAITING_GPS", "NO_TELEM",
                      "NO_WAYPOINTS"):
            r = sm_transition(state, SR_IDLE, SmInput())
            self.assertEqual(r.next_state, state)
            self.assertEqual(r.wp_inc, 0)


if __name__ == "__main__":
    unittest.main()