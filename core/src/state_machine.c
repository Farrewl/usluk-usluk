/*
 * core/src/state_machine.c — Mesin state misi (port 1:1 app/state_machine.py)
 *
 * Logika transisi setara dengan run() di app/navigator.py:
 *   - 16 state misi + 3 guard (WAITING_GPS/NO_TELEM/NO_WAYPOINTS/MISSION_COMPLETE)
 *   - sub-machine mundur RETREAT & BLUE_BOX_RETREAT (brake->netral->mundur)
 *   - tanpa magic number: semua ambang lewat sm_input_t dari pemanggil
 *   - tidak menyentuh thrust/vision/kamera (itu tetap Python, sesuai arsitektur)
 *
 * Verifikasi identik dengan Python:
 *   - core/tests/test_state_machine.c   (nilai emas skenario)
 *   - tests/test_core_state_machine.py  (cross-check ctypes, trace penuh)
 */
#include "state_machine.h"

#include <string.h>

/* ------------------------------------------------------------------------ */
/* Guard loop utama (sm_guard)                                              */
/* ------------------------------------------------------------------------ */

sm_state_t sm_guard(int gps_3d_fix, int yaw_radio, int waypoints_loaded,
                    int wp_idx, int n_wp) {
    if (!(gps_3d_fix && yaw_radio)) {
        return SM_NO_TELEM;
    }
    if (!waypoints_loaded) {
        return SM_NO_WAYPOINTS;
    }
    if (wp_idx >= n_wp) {
        return SM_MISSION_COMPLETE;
    }
    return SM_NONE;
}

/* ------------------------------------------------------------------------ */
/* Sub-machine mundur (dipakai RETREAT & BLUE_BOX_RETREAT)                  */
/* ------------------------------------------------------------------------ */

static sm_result_t sm_retreat_decision(sm_state_t state,
                                       sm_retreat_step_t retreat_step,
                                       const sm_input_t *inp) {
    sm_result_t r;

    if (retreat_step == SM_STEP_START_BRAKE) {
        r.next = state;
        r.wp_inc = 0;
        r.reset_task_timer = 0;
        r.reset_vision = 0;
        r.retreat_step = inp->brake_over ? SM_STEP_GOTO_NEUTRAL
                                         : SM_STEP_START_BRAKE;
        return r;
    }
    if (retreat_step == SM_STEP_GOTO_NEUTRAL) {
        r.next = state;
        r.wp_inc = 0;
        r.reset_task_timer = inp->neutral_over ? 1 : 0;
        r.reset_vision = 0;
        r.retreat_step = inp->neutral_over ? SM_STEP_START_REVERSE
                                           : SM_STEP_GOTO_NEUTRAL;
        return r;
    }
    if (retreat_step == SM_STEP_START_REVERSE) {
        if (inp->retreat_over) {
            r.wp_inc = 0;
            r.reset_task_timer = 0;
            r.retreat_step = SM_STEP_IDLE;
            if (state == SM_BLUE_BOX_RETREAT && inp->red_model) {
                /* BLUE_BOX_RETREAT selesai + model merah ada -> docking. */
                r.next = SM_APPROACH_RED_BOX_SEARCH;
                r.reset_vision = 1;
            } else {
                /* RETREAT selesai (atau biru tanpa model merah) -> WP_NAV,
                 * wp++ (bila masih ada waypoint, pengecekan di loop utama),
                 * reset koreksi vision. */
                r.next = SM_WAYPOINT_NAV;
                r.wp_inc = 1;
                r.reset_vision = 1;
            }
            return r;
        }
        r.next = state;
        r.wp_inc = 0;
        r.reset_task_timer = 0;
        r.reset_vision = 0;
        r.retreat_step = SM_STEP_START_REVERSE;
        return r;
    }

    /* IDLE / nilai tak dikenal -> error state: kembali ke WAYPOINT_NAV. */
    r.next = SM_WAYPOINT_NAV;
    r.wp_inc = 0;
    r.reset_task_timer = 0;
    r.reset_vision = 0;
    r.retreat_step = SM_STEP_IDLE;
    return r;
}

/* ------------------------------------------------------------------------ */
/* Transisi utama                                                           */
/* ------------------------------------------------------------------------ */

sm_result_t sm_transition(sm_state_t state, sm_retreat_step_t retreat_step,
                          const sm_input_t *inp) {
    sm_result_t r = { SM_WAYPOINT_NAV, 0, 0, 0, SM_STEP_IDLE };

    switch (state) {
    case SM_WAYPOINT_NAV:
        if (inp->wp_in_photo_box_legs && inp->green_model) {
            r.next = SM_APPROACH_BOX_SEARCH;
            r.reset_vision = 1;
        } else if (inp->wp_in_blue_box_legs && inp->blue_model) {
            r.next = SM_APPROACH_BLUE_BOX_SEARCH;
            r.reset_vision = 1;
        } else if (inp->at_waypoint) {
            if (inp->wp_in_stop_and_photo) {
                r.next = SM_TAKE_WAYPOINT_PHOTO;
                r.reset_task_timer = 1;
            } else if (inp->wp_idx + 1 >= inp->n_wp) {
                /* wp++ lalu habis -> misi selesai. */
                r.next = SM_MISSION_COMPLETE;
                r.wp_inc = 1;
                r.reset_vision = 1;
            } else {
                r.next = SM_WAYPOINT_TRANSITION;
                r.wp_inc = 1;
                r.reset_vision = 1;
                r.reset_task_timer = 1;
            }
        } else {
            r.next = SM_WAYPOINT_NAV;
        }
        break;

    case SM_WAYPOINT_TRANSITION:
        r.next = inp->transition_over ? SM_WAYPOINT_NAV
                                      : SM_WAYPOINT_TRANSITION;
        break;

    case SM_TAKE_WAYPOINT_PHOTO:
        if (inp->wp_photo_over) {
            if (inp->wp_is_red_after && inp->red_model) {
                r.next = SM_APPROACH_RED_BOX_SEARCH;
                r.reset_vision = 1;
            } else if (inp->wp_idx + 1 >= inp->n_wp) {
                r.next = SM_MISSION_COMPLETE;
                r.wp_inc = 1;
                r.reset_vision = 1;
            } else {
                r.next = SM_WAYPOINT_TRANSITION;
                r.wp_inc = 1;
                r.reset_vision = 1;
                r.reset_task_timer = 1;
            }
        } else {
            r.next = SM_TAKE_WAYPOINT_PHOTO;
        }
        break;

    case SM_APPROACH_BOX_SEARCH:
        if (inp->box_found && inp->box_confirmed) {
            r.next = SM_APPROACH_BOX_ALIGN;
            r.reset_vision = 1;
        } else {
            r.next = SM_APPROACH_BOX_SEARCH;
        }
        break;

    case SM_APPROACH_BOX_ALIGN:
        if (inp->box_found) {
            if (inp->box_aligned) {
                r.next = SM_TAKE_PHOTO;
                r.reset_vision = 1;
            } else {
                r.next = SM_APPROACH_BOX_ALIGN;
            }
        } else if (inp->box_lost_over_2s) {
            r.next = SM_TAKE_PHOTO;
            r.reset_vision = 1;
        } else {
            r.next = SM_APPROACH_BOX_ALIGN;
        }
        break;

    case SM_TAKE_PHOTO:
        /* Fase TAKE_PHOTO selalu lanjut ke RETREAT (mulai brake). */
        r.next = SM_RETREAT;
        r.reset_task_timer = 1;
        r.retreat_step = SM_STEP_START_BRAKE;
        break;

    case SM_RETREAT:
        r = sm_retreat_decision(state, retreat_step, inp);
        break;

    case SM_APPROACH_BLUE_BOX_SEARCH:
        if (inp->blue_found && inp->blue_confirmed) {
            r.next = SM_APPROACH_BLUE_BOX_ALIGN;
            r.reset_vision = 1;
        } else {
            r.next = SM_APPROACH_BLUE_BOX_SEARCH;
        }
        break;

    case SM_APPROACH_BLUE_BOX_ALIGN:
        if (inp->blue_found) {
            if (inp->blue_aligned) {
                r.next = SM_TAKE_BLUE_BOX_PHOTO;
                r.reset_task_timer = 1;
                r.reset_vision = 1;
            } else {
                r.next = SM_APPROACH_BLUE_BOX_ALIGN;
            }
        } else if (inp->blue_lost_over_2s) {
            r.next = SM_TAKE_BLUE_BOX_PHOTO;
            r.reset_task_timer = 1;
            r.reset_vision = 1;
        } else {
            r.next = SM_APPROACH_BLUE_BOX_ALIGN;
        }
        break;

    case SM_TAKE_BLUE_BOX_PHOTO:
        if (inp->blue_photo_over) {
            r.next = SM_BLUE_BOX_RETREAT;
            r.reset_task_timer = 1;
            r.retreat_step = SM_STEP_START_BRAKE;
        } else {
            r.next = SM_TAKE_BLUE_BOX_PHOTO;
        }
        break;

    case SM_BLUE_BOX_RETREAT:
        r = sm_retreat_decision(state, retreat_step, inp);
        break;

    case SM_APPROACH_RED_BOX_SEARCH:
        if (inp->red_found) {
            r.next = SM_APPROACH_RED_BOX_ALIGN;
            r.reset_vision = 1;
        } else {
            r.next = SM_APPROACH_RED_BOX_SEARCH;
        }
        break;

    case SM_APPROACH_RED_BOX_ALIGN:
        if (!inp->red_found) {
            r.next = SM_APPROACH_RED_BOX_SEARCH;
        } else if (inp->red_aligned) {
            r.next = SM_RED_BOX_DOCKED;
            r.reset_task_timer = 1;
        } else {
            r.next = SM_APPROACH_RED_BOX_ALIGN;
        }
        break;

    case SM_RED_BOX_DOCKED:
        if (inp->dock_hold_over) {
            if (inp->wp_idx + 1 >= inp->n_wp) {
                r.next = SM_MISSION_COMPLETE;
                r.wp_inc = 1;
                r.reset_vision = 1;
            } else {
                r.next = SM_WAYPOINT_TRANSITION;
                r.wp_inc = 1;
                r.reset_vision = 1;
                r.reset_task_timer = 1;
            }
        } else {
            r.next = SM_RED_BOX_DOCKED;
        }
        break;

    default:
        /* WAITING_GPS / NO_TELEM / NO_WAYPOINTS / MISSION_COMPLETE / tak
         * dikenal: tanpa transisi (tetap di state saat ini). */
        r.next = state;
        break;
    }
    return r;
}

/* ------------------------------------------------------------------------ */
/* Konversi nama <-> enum                                                    */
/* ------------------------------------------------------------------------ */

const char *sm_state_name(sm_state_t s) {
    static const char *const names[SM_STATE_COUNT] = {
        "WAITING_GPS", "NO_TELEM", "NO_WAYPOINTS", "MISSION_COMPLETE",
        "WAYPOINT_NAV", "WAYPOINT_TRANSITION", "TAKE_WAYPOINT_PHOTO",
        "APPROACH_BOX_SEARCH", "APPROACH_BOX_ALIGN", "TAKE_PHOTO",
        "RETREAT", "APPROACH_BLUE_BOX_SEARCH", "APPROACH_BLUE_BOX_ALIGN",
        "TAKE_BLUE_BOX_PHOTO", "BLUE_BOX_RETREAT", "APPROACH_RED_BOX_SEARCH",
        "APPROACH_RED_BOX_ALIGN", "RED_BOX_DOCKED"
    };
    if (s < 0 || s >= SM_STATE_COUNT) {
        return "???";
    }
    return names[s];
}

sm_state_t sm_state_from_name(const char *name) {
    int i;
    for (i = 0; i < SM_STATE_COUNT; i++) {
        if (strcmp(sm_state_name((sm_state_t)i), name) == 0) {
            return (sm_state_t)i;
        }
    }
    return SM_NONE;
}

const char *sm_retreat_step_name(sm_retreat_step_t s) {
    static const char *const names[4] = {
        "IDLE", "START_BRAKE", "GOTO_NEUTRAL", "START_REVERSE"
    };
    if (s < 0 || s >= 4) {
        return "???";
    }
    return names[s];
}