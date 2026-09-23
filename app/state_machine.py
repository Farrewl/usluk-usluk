"""
app/state_machine.py — Referensi MURNI logika transisi state machine misi.

Diekstrak dari `run()` di app/navigator.py (tidak menyentuh thrust/vision/
kamera — murni keputusan *ke state mana berikutnya* + efek samping yang
mengubah pengelolaan misi: increment waypoint, reset timer, reset koreksi
vision, step mundur). Semua nilai threshold masuk lewat `SmInput` (bukan
konstanta global) supaya bisa di-port 1:1 ke C (core/src/state_machine.c)
tanpa "magic number" dan dibuktikan identik via ctypes.

Kontrak `sm_transition`:
  - Murni: tidak mengubah argumen; hasil dikembalikan via `SmResult`.
  - `retreat_step` BARU selalu dikembalikan (caller menimpanya); bukan
    "hanya kalau berubah" — biar mudah diaudit & di-cross-check.
"""

# ---------------------------------------------------------------------------
# Nama state (cocok dengan strings yang dipakai navigator/GUI/Redis)
# ---------------------------------------------------------------------------

WAITING_GPS = "WAITING_GPS"
NO_TELEM = "NO_TELEM"
NO_WAYPOINTS = "NO_WAYPOINTS"
MISSION_COMPLETE = "MISSION_COMPLETE"
WAYPOINT_NAV = "WAYPOINT_NAV"
WAYPOINT_TRANSITION = "WAYPOINT_TRANSITION"
TAKE_WAYPOINT_PHOTO = "TAKE_WAYPOINT_PHOTO"
APPROACH_BOX_SEARCH = "APPROACH_BOX_SEARCH"
APPROACH_BOX_ALIGN = "APPROACH_BOX_ALIGN"
TAKE_PHOTO = "TAKE_PHOTO"
RETREAT = "RETREAT"
APPROACH_BLUE_BOX_SEARCH = "APPROACH_BLUE_BOX_SEARCH"
APPROACH_BLUE_BOX_ALIGN = "APPROACH_BLUE_BOX_ALIGN"
TAKE_BLUE_BOX_PHOTO = "TAKE_BLUE_BOX_PHOTO"
BLUE_BOX_RETREAT = "BLUE_BOX_RETREAT"
APPROACH_RED_BOX_SEARCH = "APPROACH_RED_BOX_SEARCH"
APPROACH_RED_BOX_ALIGN = "APPROACH_RED_BOX_ALIGN"
RED_BOX_DOCKED = "RED_BOX_DOCKED"

# Step mundur (sub-machine RETREAT / BLUE_BOX_RETREAT)
SR_IDLE = "IDLE"
SR_START_BRAKE = "START_BRAKE"
SR_GOTO_NEUTRAL = "GOTO_NEUTRAL"
SR_START_REVERSE = "START_REVERSE"

# Semua state misi (16 + guard) — urutan ini harus SAMA dengan enum di C.
ALL_STATES = (
    WAITING_GPS, NO_TELEM, NO_WAYPOINTS, MISSION_COMPLETE,
    WAYPOINT_NAV, WAYPOINT_TRANSITION, TAKE_WAYPOINT_PHOTO,
    APPROACH_BOX_SEARCH, APPROACH_BOX_ALIGN, TAKE_PHOTO, RETREAT,
    APPROACH_BLUE_BOX_SEARCH, APPROACH_BLUE_BOX_ALIGN, TAKE_BLUE_BOX_PHOTO,
    BLUE_BOX_RETREAT, APPROACH_RED_BOX_SEARCH, APPROACH_RED_BOX_ALIGN,
    RED_BOX_DOCKED,
)


class SmInput:
    """Snapshot keputusan transisi — dihitung caller dari telemetri/vision/timer.

    Semua field int bernilai 0/1 (bool). Konvensi nama mengikuti kode asli:
    `*_over` berarti timer sudah lewat ambang; `*_found/*_aligned` hasil
    deteksi vision.
    """

    def __init__(self, wp_idx=0, n_wp=0, at_waypoint=0,
                 wp_in_photo_box_legs=0, wp_in_blue_box_legs=0,
                 wp_in_stop_and_photo=0, wp_is_red_after=0,
                 green_model=0, blue_model=0, red_model=0,
                 box_found=0, box_confirmed=0, box_aligned=0, box_lost_over_2s=0,
                 blue_found=0, blue_confirmed=0, blue_aligned=0,
                 blue_lost_over_2s=0,
                 red_found=0, red_aligned=0,
                 transition_over=0, wp_photo_over=0, blue_photo_over=0,
                 brake_over=0, neutral_over=0, retreat_over=0,
                 dock_hold_over=0):
        self.wp_idx = wp_idx
        self.n_wp = n_wp
        self.at_waypoint = at_waypoint
        self.wp_in_photo_box_legs = wp_in_photo_box_legs
        self.wp_in_blue_box_legs = wp_in_blue_box_legs
        self.wp_in_stop_and_photo = wp_in_stop_and_photo
        self.wp_is_red_after = wp_is_red_after
        self.green_model = green_model
        self.blue_model = blue_model
        self.red_model = red_model
        self.box_found = box_found
        self.box_confirmed = box_confirmed
        self.box_aligned = box_aligned
        self.box_lost_over_2s = box_lost_over_2s
        self.blue_found = blue_found
        self.blue_confirmed = blue_confirmed
        self.blue_aligned = blue_aligned
        self.blue_lost_over_2s = blue_lost_over_2s
        self.red_found = red_found
        self.red_aligned = red_aligned
        self.transition_over = transition_over
        self.wp_photo_over = wp_photo_over
        self.blue_photo_over = blue_photo_over
        self.brake_over = brake_over
        self.neutral_over = neutral_over
        self.retreat_over = retreat_over
        self.dock_hold_over = dock_hold_over


class SmResult:
    """Hasil keputusan transisi (efek samping yang harus dilakukan caller)."""

    def __init__(self, next_state, wp_inc=0, reset_task_timer=0,
                 reset_vision=0, retreat_step=SR_IDLE):
        self.next_state = next_state
        self.wp_inc = wp_inc
        self.reset_task_timer = reset_task_timer
        self.reset_vision = reset_vision
        self.retreat_step = retreat_step

    def __eq__(self, other):
        if not isinstance(other, SmResult):
            return NotImplemented
        return (self.next_state == other.next_state
                and self.wp_inc == other.wp_inc
                and self.reset_task_timer == other.reset_task_timer
                and self.reset_vision == other.reset_vision
                and self.retreat_step == other.retreat_step)

    def __repr__(self):
        return (f"SmResult(next={self.next_state}, wp_inc={self.wp_inc}, "
                f"reset_task_timer={self.reset_task_timer}, "
                f"reset_vision={self.reset_vision}, "
                f"retreat_step={self.retreat_step})")


# Alias here-doc agar penyingkat di C: state "lanjut sama" = nilai saat itu.
def sm_guard(gps_3d_fix, yaw_radio, waypoints_loaded, wp_idx, n_wp):
    """Guard loop utama run(): NO_TELEM / NO_WAYPOINTS / MISSION_COMPLETE.

    Kembalikan state guard, atau None bila misi boleh melanjutkan.
    """
    if not (gps_3d_fix and yaw_radio):
        return NO_TELEM
    if not waypoints_loaded:
        return NO_WAYPOINTS
    if wp_idx >= n_wp:
        return MISSION_COMPLETE
    return None


def sm_transition(state, retreat_step, inp):
    """Keputusan transisi murni — port 1:1 ke core/src/state_machine.c.

    `state` dan `retreat_step` adalah nilai SAAT INI (belum diubah).
    Hasil: state berikutnya + efek samping (wp_inc, reset timer/vision,
    retreat_step baru). Bila tidak ada transisi, `next_state == state`.
    """
    if state == WAYPOINT_NAV:
        if inp.wp_in_photo_box_legs and inp.green_model:
            return SmResult(APPROACH_BOX_SEARCH, reset_vision=1)
        if inp.wp_in_blue_box_legs and inp.blue_model:
            return SmResult(APPROACH_BLUE_BOX_SEARCH, reset_vision=1)
        if inp.at_waypoint:
            if inp.wp_in_stop_and_photo:
                return SmResult(TAKE_WAYPOINT_PHOTO, reset_task_timer=1)
            # wp++ lalu cek habis
            if inp.wp_idx + 1 >= inp.n_wp:
                return SmResult(MISSION_COMPLETE, wp_inc=1, reset_vision=1)
            return SmResult(WAYPOINT_TRANSITION, wp_inc=1, reset_vision=1,
                            reset_task_timer=1)
        return SmResult(WAYPOINT_NAV)

    if state == WAYPOINT_TRANSITION:
        if inp.transition_over:
            return SmResult(WAYPOINT_NAV)
        return SmResult(WAYPOINT_TRANSITION)

    if state == TAKE_WAYPOINT_PHOTO:
        if inp.wp_photo_over:
            if inp.wp_is_red_after and inp.red_model:
                return SmResult(APPROACH_RED_BOX_SEARCH, reset_vision=1)
            if inp.wp_idx + 1 >= inp.n_wp:
                return SmResult(MISSION_COMPLETE, wp_inc=1, reset_vision=1)
            return SmResult(WAYPOINT_TRANSITION, wp_inc=1, reset_vision=1,
                            reset_task_timer=1)
        return SmResult(TAKE_WAYPOINT_PHOTO)

    if state == APPROACH_BOX_SEARCH:
        if inp.box_found and inp.box_confirmed:
            return SmResult(APPROACH_BOX_ALIGN, reset_vision=1)
        return SmResult(APPROACH_BOX_SEARCH)

    if state == APPROACH_BOX_ALIGN:
        if inp.box_found:
            if inp.box_aligned:
                return SmResult(TAKE_PHOTO, reset_vision=1)
            return SmResult(APPROACH_BOX_ALIGN)
        if inp.box_lost_over_2s:
            return SmResult(TAKE_PHOTO, reset_vision=1)
        return SmResult(APPROACH_BOX_ALIGN)

    if state == TAKE_PHOTO:
        # Fase TAKE_PHOTO selalu lanjut ke RETREAT (mulai brake).
        return SmResult(RETREAT, reset_task_timer=1, retreat_step=SR_START_BRAKE)

    if state == RETREAT:
        return _retreat_decision(state, retreat_step, inp)

    if state == APPROACH_BLUE_BOX_SEARCH:
        if inp.blue_found and inp.blue_confirmed:
            return SmResult(APPROACH_BLUE_BOX_ALIGN, reset_vision=1)
        return SmResult(APPROACH_BLUE_BOX_SEARCH)

    if state == APPROACH_BLUE_BOX_ALIGN:
        if inp.blue_found:
            if inp.blue_aligned:
                return SmResult(TAKE_BLUE_BOX_PHOTO, reset_task_timer=1,
                                reset_vision=1)
            return SmResult(APPROACH_BLUE_BOX_ALIGN)
        if inp.blue_lost_over_2s:
            return SmResult(TAKE_BLUE_BOX_PHOTO, reset_task_timer=1,
                            reset_vision=1)
        return SmResult(APPROACH_BLUE_BOX_ALIGN)

    if state == TAKE_BLUE_BOX_PHOTO:
        if inp.blue_photo_over:
            return SmResult(BLUE_BOX_RETREAT, reset_task_timer=1,
                            retreat_step=SR_START_BRAKE)
        return SmResult(TAKE_BLUE_BOX_PHOTO)

    if state == BLUE_BOX_RETREAT:
        return _retreat_decision(state, retreat_step, inp)

    if state == APPROACH_RED_BOX_SEARCH:
        if inp.red_found:
            return SmResult(APPROACH_RED_BOX_ALIGN, reset_vision=1)
        return SmResult(APPROACH_RED_BOX_SEARCH)

    if state == APPROACH_RED_BOX_ALIGN:
        if not inp.red_found:
            return SmResult(APPROACH_RED_BOX_SEARCH)
        if inp.red_aligned:
            return SmResult(RED_BOX_DOCKED, reset_task_timer=1)
        return SmResult(APPROACH_RED_BOX_ALIGN)

    if state == RED_BOX_DOCKED:
        if inp.dock_hold_over:
            if inp.wp_idx + 1 >= inp.n_wp:
                return SmResult(MISSION_COMPLETE, wp_inc=1, reset_vision=1)
            return SmResult(WAYPOINT_TRANSITION, wp_inc=1, reset_vision=1,
                            reset_task_timer=1)
        return SmResult(RED_BOX_DOCKED)

    # WAITING_GPS / NO_TELEM / NO_WAYPOINTS / MISSION_COMPLETE: tanpa transisi.
    return SmResult(state)


def _retreat_decision(state, retreat_step, inp):
    """Sub-machine mundur (RETREAT / BLUE_BOX_RETREAT) — lihat run() navigator.

    Urutan: START_BRAKE (0.2s) -> GOTO_NEUTRAL (0.4s) -> START_REVERSE
    (RETREAT_DURATION_S) -> selesai. Bedanya hanya aksi setelah selesai
    (BLUE_BOX_RETREAT bisa lanjut docking).
    """
    if retreat_step == SR_START_BRAKE:
        if inp.brake_over:
            return SmResult(state, retreat_step=SR_GOTO_NEUTRAL)
        return SmResult(state, retreat_step=SR_START_BRAKE)

    if retreat_step == SR_GOTO_NEUTRAL:
        if inp.neutral_over:
            return SmResult(state, reset_task_timer=1,
                            retreat_step=SR_START_REVERSE)
        return SmResult(state, retreat_step=SR_GOTO_NEUTRAL)

    if retreat_step == SR_START_REVERSE:
        if inp.retreat_over:
            if state == BLUE_BOX_RETREAT and inp.red_model:
                return SmResult(APPROACH_RED_BOX_SEARCH, reset_vision=1,
                                retreat_step=SR_IDLE)
            # RETREAT selesai (atau BLUE tanpa model merah) -> WAYPOINT_NAV,
            # wp++ (bila masih ada), reset koreksi vision.
            return SmResult(WAYPOINT_NAV, wp_inc=1, reset_vision=1,
                            retreat_step=SR_IDLE)
        return SmResult(state, retreat_step=SR_START_REVERSE)

    # IDLE / nilai tak dikenal -> error state: kembali ke WAYPOINT_NAV.
    return SmResult(WAYPOINT_NAV, retreat_step=SR_IDLE)