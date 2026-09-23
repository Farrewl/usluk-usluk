/*
 * core/include/state_machine.h — Mesin state misi (16 state + guard)
 *
 * Port 1:1 dari app/state_machine.py (referensi murni yang diekstrak dari
 * run() di app/navigator.py). Hanya memutuskan "ke state mana berikutnya"
 * + efek samping (increment waypoint, reset timer, reset koreksi vision,
 * step mundur) — thrust/vision/kamera TETAP di Python/GUI.
 *
 * Kantrak penting:
 *   - sm_transition MURNI: tidak mengubah input; semua keputusan dari
 *     snapshot `sm_input_t` (field 0/1). Nilai threshold dihitung caller
 *     (telemetri/vision/timer), jadi tidak ada magic number di sini.
 *   - `retreat_step` BARU selalu dikembalikan (caller menimpanya).
 *   - Bila tidak ada transisi: result.next == state (tetap).
 *
 * Pemakaian:  #include "state_machine.h"
 */
#ifndef ASV_STATE_MACHINE_H
#define ASV_STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Urutan enum HARUS sama dengan ALL_STATES di app/state_machine.py. */
typedef enum {
    SM_WAITING_GPS = 0,
    SM_NO_TELEM,
    SM_NO_WAYPOINTS,
    SM_MISSION_COMPLETE,
    SM_WAYPOINT_NAV,
    SM_WAYPOINT_TRANSITION,
    SM_TAKE_WAYPOINT_PHOTO,
    SM_APPROACH_BOX_SEARCH,
    SM_APPROACH_BOX_ALIGN,
    SM_TAKE_PHOTO,
    SM_RETREAT,
    SM_APPROACH_BLUE_BOX_SEARCH,
    SM_APPROACH_BLUE_BOX_ALIGN,
    SM_TAKE_BLUE_BOX_PHOTO,
    SM_BLUE_BOX_RETREAT,
    SM_APPROACH_RED_BOX_SEARCH,
    SM_APPROACH_RED_BOX_ALIGN,
    SM_RED_BOX_DOCKED,
    SM_STATE_COUNT
} sm_state_t;

/* Sentinel: guard tidak menghalangi misi (boleh lanjut). */
#define SM_NONE ((sm_state_t)-1)

/* Sub-machine mundur (RETREAT / BLUE_BOX_RETREAT) — cocok dengan
 * SR_* di app/state_machine.py. */
typedef enum {
    SM_STEP_IDLE = 0,
    SM_STEP_START_BRAKE,
    SM_STEP_GOTO_NEUTRAL,
    SM_STEP_START_REVERSE
} sm_retreat_step_t;

/* Snapshot keputusan transisi. Nama field mengikuti kode navigator asli:
 * akhiran `_over` = timer melewati ambang; `_found`/`_aligned` = hasil vision.
 * URUTAN FIELD harus sama dengan SmInput.__init__ di Python (dipakai
 * untuk membangun struct identik saat cross-check ctypes). */
typedef struct {
    int wp_idx;
    int n_wp;
    int at_waypoint;
    int wp_in_photo_box_legs;
    int wp_in_blue_box_legs;
    int wp_in_stop_and_photo;
    int wp_is_red_after;
    int green_model;
    int blue_model;
    int red_model;
    int box_found;
    int box_confirmed;
    int box_aligned;
    int box_lost_over_2s;
    int blue_found;
    int blue_confirmed;
    int blue_aligned;
    int blue_lost_over_2s;
    int red_found;
    int red_aligned;
    int transition_over;
    int wp_photo_over;
    int blue_photo_over;
    int brake_over;
    int neutral_over;
    int retreat_over;
    int dock_hold_over;
} sm_input_t;

/* Hasil keputusan (efek samping yang harus dilakukan pemanggil). */
typedef struct {
    sm_state_t        next;
    int               wp_inc;          /* 1 = current_waypoint_index++ */
    int               reset_task_timer;/* 1 = task_timer di-set ulang */
    int               reset_vision;    /* 1 = last_vision_correction_rad=0 */
    sm_retreat_step_t retreat_step;    /* nilai BARU (selalu terisi) */
} sm_result_t;

/**
 * sm_guard — guard loop utama: NO_TELEM / NO_WAYPOINTS / MISSION_COMPLETE.
 * @return SM_NONE bila misi boleh lanjut; selain itu state guard.
 */
sm_state_t sm_guard(int gps_3d_fix, int yaw_radio, int waypoints_loaded,
                    int wp_idx, int n_wp);

/**
 * sm_transition — keputusan transisi murni dari state + snapshot.
 * @param state         state SAAT INI (belum diubah)
 * @param retreat_step  step mundur SAAT INI (bila di RETREAT/BLUE_BOX_RETREAT)
 * @param inp           snapshot keputusan (0/1 + wp_idx/n_wp)
 * @return result dengan next state & efek samping.
 */
sm_result_t sm_transition(sm_state_t state, sm_retreat_step_t retreat_step,
                          const sm_input_t *inp);

/* Konversi nama <-> enum untuk observability & test. */
const char *sm_state_name(sm_state_t s);
sm_state_t  sm_state_from_name(const char *name);
const char *sm_retreat_step_name(sm_retreat_step_t s);

#ifdef __cplusplus
}
#endif

#endif /* ASV_STATE_MACHINE_H */