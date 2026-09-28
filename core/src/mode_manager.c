/*
 * core/src/mode_manager.c — State mode kendali AUTO/MANUAL/HOLD/KILL (C murni)
 */

#include "mode_manager.h"

void mode_reset(mode_state_t *s) {
    if (s == 0) {
        return;
    }
    s->mode = OP_MODE_AUTO;
    s->hold_timer_s = 0.0;
}

op_mode_t mode_update(mode_state_t *s, int kill, int manual_req, int rc_ok,
                      double dt, double lost_hold_s) {
    if (s == 0) {
        return OP_MODE_AUTO;
    }
    if (dt < 0.0) {
        dt = 0.0;
    }
    if (lost_hold_s < 0.0) {
        lost_hold_s = 0.0;
    }

    /* 1. KILL selalu menang — langsung, tanpa timer. */
    if (kill) {
        s->mode = OP_MODE_KILL;
        s->hold_timer_s = 0.0;
        return s->mode;
    }

    /* 2. Keluar dari KILL bila kill lepas -> AUTO (aman, bukan MANUAL). */
    if (s->mode == OP_MODE_KILL) {
        s->mode = OP_MODE_AUTO;
        s->hold_timer_s = 0.0;
    }

    /* 3. Minta manual + RC sehat -> MANUAL (timer hold di-nol-kan). */
    if (manual_req && rc_ok) {
        s->mode = OP_MODE_MANUAL;
        s->hold_timer_s = 0.0;
        return s->mode;
    }

    /* 4. RC putus di tengah MANUAL -> HOLD selama lost_hold_s detik. */
    if (s->mode == OP_MODE_MANUAL) {
        s->hold_timer_s += dt;
        if (s->hold_timer_s < lost_hold_s) {
            s->mode = OP_MODE_HOLD;
            return s->mode;
        }
        s->mode = OP_MODE_AUTO;
        s->hold_timer_s = 0.0;
        return s->mode;
    }
    if (s->mode == OP_MODE_HOLD) {
        /* RC pulih saat HOLD + masih minta manual -> kembali MANUAL. */
        if (manual_req && rc_ok) {
            s->mode = OP_MODE_MANUAL;
            s->hold_timer_s = 0.0;
            return s->mode;
        }
        s->hold_timer_s += dt;
        if (s->hold_timer_s >= lost_hold_s) {
            s->mode = OP_MODE_AUTO;
            s->hold_timer_s = 0.0;
        }
        return s->mode;
    }

    /* 5. Default: AUTO. */
    s->mode = OP_MODE_AUTO;
    s->hold_timer_s = 0.0;
    return s->mode;
}

const char *mode_name(op_mode_t m) {
    switch (m) {
    case OP_MODE_AUTO:
        return "AUTO";
    case OP_MODE_MANUAL:
        return "MANUAL";
    case OP_MODE_HOLD:
        return "HOLD";
    case OP_MODE_KILL:
        return "KILL";
    default:
        return "?";
    }
}
