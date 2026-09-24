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
