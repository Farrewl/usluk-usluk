/*
 * core/src/arbitrator.c — Implementasi arbitrator setpoint (C murni)
 */

#include "arbitrator.h"

static double arb_clamp1(double v) {
    if (v < -1.0) return -1.0;
    if (v >  1.0) return  1.0;
    return v;
}

void arb_select(int kill_active, int manual_active,
                double manual_surge, double manual_yaw,
                double auto_surge, double auto_yaw,
                arb_out_t *out) {
    if (out == 0) return;

    if (kill_active) {
        out->surge = 0.0;
        out->yaw = 0.0;
        out->source = ARB_SRC_KILLED;
        out->killed = 1;
        return;
    }
    out->killed = 0;
    if (manual_active) {
        out->surge = arb_clamp1(manual_surge);
        out->yaw = arb_clamp1(manual_yaw);
        out->source = ARB_SRC_MANUAL;
        return;
    }
    out->surge = arb_clamp1(auto_surge);
    out->yaw = arb_clamp1(auto_yaw);
    out->source = ARB_SRC_AUTO;
}

void arb_manual2(int qgc_active, double qgc_surge, double qgc_yaw,
                 int loc_active, double loc_surge, double loc_yaw,
                 arb_manual_t *out) {
    if (out == 0) return;

    /* Konflik: dua operator memegang bersamaan -> netral, jangan pilih. */
    if (qgc_active && loc_active) {
        out->surge = 0.0;
        out->yaw = 0.0;
        out->source = ARB_MANUAL_CONFLICT;
        out->conflict = 1;
        return;
    }
    out->conflict = 0;
    if (qgc_active) {
        out->surge = arb_clamp1(qgc_surge);
        out->yaw = arb_clamp1(qgc_yaw);
        out->source = ARB_MANUAL_QGC;
        return;
    }
    if (loc_active) {
        out->surge = arb_clamp1(loc_surge);
        out->yaw = arb_clamp1(loc_yaw);
        out->source = ARB_MANUAL_LOCAL;
        return;
    }
    out->surge = 0.0;
    out->yaw = 0.0;
    out->source = ARB_MANUAL_NONE;
}
