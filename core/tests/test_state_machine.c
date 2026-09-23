/*
 * core/tests/test_state_machine.c — Unit test mesin state misi (C)
 *
 * Skenario nilai emas = trace urutan state yang DIHARAPKAN dari perilaku
 * run() di navigator.py (diekstrak ke app/state_machine.py, C wajib sama).
 * Skenario:
 *   A. Gate nav murni: WP_NAV -> TRANSITION -> WP_NAV ... -> MISSION_COMPLETE
 *   B. Foto box hijau penuh + sub-machine mundur (brake->netral->mundur)
 *   C. Foto box biru + docking red box + hold + lanjut misi
 *   D. Jalan buntu: target hilang saat ALIGN, RED lost, retreat error
 *   E. Guard loop utama (NO_TELEM / NO_WAYPOINTS / MISSION_COMPLETE)
 *   F. Stop-and-photo waypoint + red-dock-setelah-WP
 *   G. Konversi nama <-> enum
 *
 * Jalankan:
 *   gcc -I core/include core/tests/test_state_machine.c core/src/state_machine.c \
 *       -o /tmp/opencode/test_sm && /tmp/opencode/test_sm
 */
#include "state_machine.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

/* Cek satu transisi; cetak nama state biar mudah dibaca. */
static void expect(const char *tag, sm_state_t state, sm_retreat_step_t step,
                   const sm_input_t *in,
                   sm_state_t want_next, int want_wp_inc,
                   int want_reset_tt, int want_reset_vis,
                   sm_retreat_step_t want_step) {
    const sm_result_t r = sm_transition(state, step, in);
    int bad = 0;
    if (r.next != want_next) {
        printf("FAIL %s: next=%s harap=%s\n", tag, sm_state_name(r.next),
               sm_state_name(want_next));
        bad = 1;
    }
    if (r.wp_inc != want_wp_inc) {
        printf("FAIL %s: wp_inc=%d harap=%d\n", tag, r.wp_inc, want_wp_inc);
        bad = 1;
    }
    if (r.reset_task_timer != want_reset_tt) {
        printf("FAIL %s: reset_task_timer=%d harap=%d\n", tag,
               r.reset_task_timer, want_reset_tt);
        bad = 1;
    }
    if (r.reset_vision != want_reset_vis) {
        printf("FAIL %s: reset_vision=%d harap=%d\n", tag,
               r.reset_vision, want_reset_vis);
        bad = 1;
    }
    if (r.retreat_step != want_step) {
        printf("FAIL %s: retreat_step=%s harap=%s\n", tag,
               sm_retreat_step_name(r.retreat_step),
               sm_retreat_step_name(want_step));
        bad = 1;
    }
    if (!bad) {
        printf("ok    %-38s %s -> %s\n", tag, sm_state_name(state),
               sm_state_name(r.next));
    } else {
        failures++;
    }
}

static void expect_stay(const char *tag, sm_state_t state,
                        sm_retreat_step_t step, const sm_input_t *in,
                        sm_retreat_step_t want_step) {
    expect(tag, state, step, in, state, 0, 0, 0, want_step);
}

int main(void) {
    /* ---------------------- A. Gate nav murni ---------------------- */
    expect("A1 nav->transition", SM_WAYPOINT_NAV, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 0, .n_wp = 5, .at_waypoint = 1 },
           SM_WAYPOINT_TRANSITION, 1, 1, 1, SM_STEP_IDLE);
    expect_stay("A2 transition menunggu", SM_WAYPOINT_TRANSITION, SM_STEP_IDLE,
                &(sm_input_t){ .transition_over = 0 }, SM_STEP_IDLE);
    expect("A3 transition selesai", SM_WAYPOINT_TRANSITION, SM_STEP_IDLE,
           &(sm_input_t){ .transition_over = 1 },
           SM_WAYPOINT_NAV, 0, 0, 0, SM_STEP_IDLE);
    expect("A4 wp terakhir -> selesai", SM_WAYPOINT_NAV, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 4, .n_wp = 5, .at_waypoint = 1 },
           SM_MISSION_COMPLETE, 1, 0, 1, SM_STEP_IDLE);
    expect_stay("A5 nav biasa", SM_WAYPOINT_NAV, SM_STEP_IDLE,
                &(sm_input_t){ .wp_idx = 2, .n_wp = 5, .at_waypoint = 0 },
                SM_STEP_IDLE);

    /* ----------------- B. Foto box hijau penuh ----------------- */
    expect("B1 leg foto-box", SM_WAYPOINT_NAV, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 0, .n_wp = 5,
                          .wp_in_photo_box_legs = 1, .green_model = 1 },
           SM_APPROACH_BOX_SEARCH, 0, 0, 1, SM_STEP_IDLE);
    expect_stay("B2 search verifikasi", SM_APPROACH_BOX_SEARCH, SM_STEP_IDLE,
                &(sm_input_t){ .box_found = 1, .box_confirmed = 0 },
                SM_STEP_IDLE);
    expect("B3 search terkunci", SM_APPROACH_BOX_SEARCH, SM_STEP_IDLE,
           &(sm_input_t){ .box_found = 1, .box_confirmed = 1 },
           SM_APPROACH_BOX_ALIGN, 0, 0, 1, SM_STEP_IDLE);
    expect_stay("B4 align maju", SM_APPROACH_BOX_ALIGN, SM_STEP_IDLE,
                &(sm_input_t){ .box_found = 1, .box_aligned = 0 },
                SM_STEP_IDLE);
    expect("B5 align sampai", SM_APPROACH_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .box_found = 1, .box_aligned = 1 },
           SM_TAKE_PHOTO, 0, 0, 1, SM_STEP_IDLE);
    expect("B6 take photo -> retreat", SM_TAKE_PHOTO, SM_STEP_IDLE,
           &(sm_input_t){},
           SM_RETREAT, 0, 1, 0, SM_STEP_START_BRAKE);
    expect_stay("B7 brake", SM_RETREAT, SM_STEP_START_BRAKE,
                &(sm_input_t){ .brake_over = 0 }, SM_STEP_START_BRAKE);
    expect("B8 brake selesai", SM_RETREAT, SM_STEP_START_BRAKE,
           &(sm_input_t){ .brake_over = 1 },
           SM_RETREAT, 0, 0, 0, SM_STEP_GOTO_NEUTRAL);
    expect_stay("B9 netral", SM_RETREAT, SM_STEP_GOTO_NEUTRAL,
                &(sm_input_t){ .neutral_over = 0 }, SM_STEP_GOTO_NEUTRAL);
    expect("B10 netral selesai", SM_RETREAT, SM_STEP_GOTO_NEUTRAL,
           &(sm_input_t){ .neutral_over = 1 },
           SM_RETREAT, 0, 1, 0, SM_STEP_START_REVERSE);
    expect_stay("B11 mundur", SM_RETREAT, SM_STEP_START_REVERSE,
                &(sm_input_t){ .retreat_over = 0 }, SM_STEP_START_REVERSE);
    expect("B12 mundur selesai", SM_RETREAT, SM_STEP_START_REVERSE,
           &(sm_input_t){ .retreat_over = 1 },
           SM_WAYPOINT_NAV, 1, 0, 1, SM_STEP_IDLE);

    /* -------------- C. Box biru + docking merah ------------- */
    expect("C1 leg foto-biru", SM_WAYPOINT_NAV, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 0, .n_wp = 10,
                          .wp_in_blue_box_legs = 1, .blue_model = 1 },
           SM_APPROACH_BLUE_BOX_SEARCH, 0, 0, 1, SM_STEP_IDLE);
    expect("C2 biru search terkunci", SM_APPROACH_BLUE_BOX_SEARCH, SM_STEP_IDLE,
           &(sm_input_t){ .blue_found = 1, .blue_confirmed = 1 },
           SM_APPROACH_BLUE_BOX_ALIGN, 0, 0, 1, SM_STEP_IDLE);
    expect("C3 biru align sampai", SM_APPROACH_BLUE_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .blue_found = 1, .blue_aligned = 1 },
           SM_TAKE_BLUE_BOX_PHOTO, 0, 1, 1, SM_STEP_IDLE);
    expect_stay("C4 biru fotokan", SM_TAKE_BLUE_BOX_PHOTO, SM_STEP_IDLE,
                &(sm_input_t){ .blue_photo_over = 0 }, SM_STEP_IDLE);
    expect("C5 biru foto selesai", SM_TAKE_BLUE_BOX_PHOTO, SM_STEP_IDLE,
           &(sm_input_t){ .blue_photo_over = 1 },
           SM_BLUE_BOX_RETREAT, 0, 1, 0, SM_STEP_START_BRAKE);
    expect("C6 biru brake selesai", SM_BLUE_BOX_RETREAT, SM_STEP_START_BRAKE,
           &(sm_input_t){ .brake_over = 1 },
           SM_BLUE_BOX_RETREAT, 0, 0, 0, SM_STEP_GOTO_NEUTRAL);
    expect("C7 biru netral selesai", SM_BLUE_BOX_RETREAT, SM_STEP_GOTO_NEUTRAL,
           &(sm_input_t){ .neutral_over = 1 },
           SM_BLUE_BOX_RETREAT, 0, 1, 0, SM_STEP_START_REVERSE);
    expect("C8 biru mundur -> docking", SM_BLUE_BOX_RETREAT,
           SM_STEP_START_REVERSE,
           &(sm_input_t){ .retreat_over = 1, .red_model = 1 },
           SM_APPROACH_RED_BOX_SEARCH, 0, 0, 1, SM_STEP_IDLE);
    expect_stay("C9 red search rotasi", SM_APPROACH_RED_BOX_SEARCH,
                SM_STEP_IDLE, &(sm_input_t){ .red_found = 0 }, SM_STEP_IDLE);
    expect("C10 red search ketemu", SM_APPROACH_RED_BOX_SEARCH, SM_STEP_IDLE,
           &(sm_input_t){ .red_found = 1 },
           SM_APPROACH_RED_BOX_ALIGN, 0, 0, 1, SM_STEP_IDLE);
    expect_stay("C11 red align maju", SM_APPROACH_RED_BOX_ALIGN, SM_STEP_IDLE,
                &(sm_input_t){ .red_found = 1, .red_aligned = 0 },
                SM_STEP_IDLE);
    expect("C12 red dock", SM_APPROACH_RED_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .red_found = 1, .red_aligned = 1 },
           SM_RED_BOX_DOCKED, 0, 1, 0, SM_STEP_IDLE);
    expect_stay("C13 dock hold", SM_RED_BOX_DOCKED, SM_STEP_IDLE,
                &(sm_input_t){ .dock_hold_over = 0 }, SM_STEP_IDLE);
    expect("C14 dock selesai -> lanjut", SM_RED_BOX_DOCKED, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 0, .n_wp = 10, .dock_hold_over = 1 },
           SM_WAYPOINT_TRANSITION, 1, 1, 1, SM_STEP_IDLE);
    expect("C15 dock selesai -> complete", SM_RED_BOX_DOCKED, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 9, .n_wp = 10, .dock_hold_over = 1 },
           SM_MISSION_COMPLETE, 1, 0, 1, SM_STEP_IDLE);

    /* ----------------- D. Jalan buntu / hilang ----------------- */
    expect_stay("D1 box hilang < 2s", SM_APPROACH_BOX_ALIGN, SM_STEP_IDLE,
                &(sm_input_t){ .box_found = 0, .box_lost_over_2s = 0 },
                SM_STEP_IDLE);
    expect("D2 box hilang > 2s", SM_APPROACH_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .box_found = 0, .box_lost_over_2s = 1 },
           SM_TAKE_PHOTO, 0, 0, 1, SM_STEP_IDLE);
    expect("D3 biru hilang > 2s", SM_APPROACH_BLUE_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .blue_found = 0, .blue_lost_over_2s = 1 },
           SM_TAKE_BLUE_BOX_PHOTO, 0, 1, 1, SM_STEP_IDLE);
    expect("D4 red hilang -> search", SM_APPROACH_RED_BOX_ALIGN, SM_STEP_IDLE,
           &(sm_input_t){ .red_found = 0 },
           SM_APPROACH_RED_BOX_SEARCH, 0, 0, 0, SM_STEP_IDLE);
    expect("D5 biru mundur tanpa model merah", SM_BLUE_BOX_RETREAT,
           SM_STEP_START_REVERSE,
           &(sm_input_t){ .wp_idx = 3, .n_wp = 10,
                          .retreat_over = 1, .red_model = 0 },
           SM_WAYPOINT_NAV, 1, 0, 1, SM_STEP_IDLE);
    expect("D6 retreat error step", SM_RETREAT, SM_STEP_IDLE,
           &(sm_input_t){},
           SM_WAYPOINT_NAV, 0, 0, 0, SM_STEP_IDLE);

    /* ------------------ E. Guard loop utama ------------------ */
    if (sm_guard(0, 1, 1, 0, 5) != SM_NO_TELEM) {
        printf("FAIL E1 guard GPS off\n"); failures++;
    } else { printf("ok    E1 guard no-GPS           = NO_TELEM\n"); }
    if (sm_guard(1, 0, 1, 0, 5) != SM_NO_TELEM) {
        printf("FAIL E2 guard yaw off\n"); failures++;
    } else { printf("ok    E2 guard no-yaw           = NO_TELEM\n"); }
    if (sm_guard(1, 1, 0, 0, 5) != SM_NO_WAYPOINTS) {
        printf("FAIL E3 guard no-waypoints\n"); failures++;
    } else { printf("ok    E3 guard no-waypoints     = NO_WAYPOINTS\n"); }
    if (sm_guard(1, 1, 1, 5, 5) != SM_MISSION_COMPLETE) {
        printf("FAIL E4 guard done\n"); failures++;
    } else { printf("ok    E4 guard all-done         = MISSION_COMPLETE\n"); }
    if (sm_guard(1, 1, 1, 4, 5) != SM_NONE) {
        printf("FAIL E5 guard continue\n"); failures++;
    } else { printf("ok    E5 guard boleh-lanjut      = SM_NONE\n"); }

    /* ------------- F. Stop-and-photo + red setelah WP ------------- */
    expect("F1 stop-photo", SM_WAYPOINT_NAV, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 2, .n_wp = 5, .at_waypoint = 1,
                          .wp_in_stop_and_photo = 1 },
           SM_TAKE_WAYPOINT_PHOTO, 0, 1, 0, SM_STEP_IDLE);
    expect_stay("F2 foto waypoint", SM_TAKE_WAYPOINT_PHOTO, SM_STEP_IDLE,
                &(sm_input_t){ .wp_photo_over = 0 }, SM_STEP_IDLE);
    expect("F3 foto wp -> docking", SM_TAKE_WAYPOINT_PHOTO, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 3, .n_wp = 5, .wp_photo_over = 1,
                          .wp_is_red_after = 1, .red_model = 1 },
           SM_APPROACH_RED_BOX_SEARCH, 0, 0, 1, SM_STEP_IDLE);
    expect("F4 foto wp -> lanjut", SM_TAKE_WAYPOINT_PHOTO, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 1, .n_wp = 5, .wp_photo_over = 1 },
           SM_WAYPOINT_TRANSITION, 1, 1, 1, SM_STEP_IDLE);
    expect("F5 foto wp terakhir", SM_TAKE_WAYPOINT_PHOTO, SM_STEP_IDLE,
           &(sm_input_t){ .wp_idx = 4, .n_wp = 5, .wp_photo_over = 1 },
           SM_MISSION_COMPLETE, 1, 0, 1, SM_STEP_IDLE);

    /* ------------------ G. Konversi nama <-> enum ------------------ */
    if (strcmp(sm_state_name(SM_RED_BOX_DOCKED), "RED_BOX_DOCKED") != 0) {
        printf("FAIL G1 nama state\n"); failures++;
    } else { printf("ok    G1 sm_state_name          = RED_BOX_DOCKED\n"); }
    if (sm_state_from_name("WAYPOINT_NAV") != SM_WAYPOINT_NAV) {
        printf("FAIL G2 from-name\n"); failures++;
    } else { printf("ok    G2 sm_state_from_name      = WAYPOINT_NAV\n"); }
    if (sm_state_from_name("TIDAK_ADA") != SM_NONE) {
        printf("FAIL G3 from-name tak dikenal\n"); failures++;
    } else { printf("ok    G3 nama tak dikenal        = SM_NONE\n"); }
    if (strcmp(sm_retreat_step_name(SM_STEP_GOTO_NEUTRAL),
               "GOTO_NEUTRAL") != 0) {
        printf("FAIL G4 nama step\n"); failures++;
    } else { printf("ok    G4 sm_retreat_step_name     = GOTO_NEUTRAL\n"); }

    if (failures == 0) {
        printf("\nSEMUA skenario state_machine LULUS (C == app/state_machine.py)\n");
        return 0;
    }
    printf("\n%d kasus GAGAL\n", failures);
    return 1;
}