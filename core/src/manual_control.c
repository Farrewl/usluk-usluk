/*
 * core/src/manual_control.c — Kurva stick manual + rate limiter (C murni)
 */

#include "manual_control.h"

/* dt <= 0 -> pakai 1e-4 dtk agar rate_limit*dt tetap valid. */
#define MANUAL_MIN_DT 1e-4

static double manual_clamp1(double v) {
    if (v < -1.0) {
        return -1.0;
    }
    if (v > 1.0) {
        return 1.0;
    }
    return v;
}

void manual_reset(manual_state_t *s) {
    if (s == 0) {
        return;
    }
    s->prev_surge = 0.0;
    s->prev_yaw = 0.0;
}

double manual_curve(double x, double deadband, double expo) {
    double mag, sign, lin;

    x = manual_clamp1(x);
    if (deadband < 0.0) {
        deadband = 0.0;
    }
    if (deadband >= 1.0) {
        return 0.0;
    }
    if (expo < 0.0) {
        expo = 0.0;
    }
    if (expo > 1.0) {
        expo = 1.0;
    }

    mag = (x < 0.0) ? -x : x;
    if (mag < deadband) {
        return 0.0;
    }
    /* Rescale sisa rentang ke [0..1] supaya ujung stick tetap = 1.0. */
    sign = (x < 0.0) ? -1.0 : 1.0;
    lin = (mag - deadband) / (1.0 - deadband);
    /* Expo: campur linear + kubik (0 = lurus, 1 = kubik penuh). */
    return sign * ((1.0 - expo) * lin + expo * lin * lin * lin);
}

double manual_rate_limit(double target, double prev, double max_step) {
    double delta;

    target = manual_clamp1(target);
    if (max_step < 0.0) {
        max_step = 0.0;
    }
    delta = target - prev;
    if (delta > max_step) {
        return prev + max_step;
    }
    if (delta < -max_step) {
        return prev - max_step;
    }
    return target;
}

void manual_update(manual_state_t *s,
                   double in_surge, double in_yaw,
                   int enabled, int deadman, int rc_ok,
                   double deadband, double expo, double rate_limit,
                   double dt, double max_surge, double max_yaw,
                   manual_out_t *out) {
    double curved_s, curved_y, max_step;

    if (out == 0) {
        return;
    }
    if (s == 0) {
        out->surge = 0.0;
        out->yaw = 0.0;
        out->active = 0;
        return;
    }
    if (dt <= 0.0) {
        dt = MANUAL_MIN_DT;
    }
    /* Syarat aktif: fitur nyala + deadman ditahan + RC sehat. */
    if (!enabled || !deadman || !rc_ok) {
        s->prev_surge = 0.0;
        s->prev_yaw = 0.0;
        out->surge = 0.0;
        out->yaw = 0.0;
        out->active = 0;
        return;
    }

    curved_s = manual_curve(in_surge, deadband, expo);
    curved_y = manual_curve(in_yaw, deadband, expo);

    /* Batas gas manual (aman): skala ke MANUAL_MAX_*. */
    if (max_surge < 0.0) {
        max_surge = 0.0;
    }
    if (max_surge > 1.0) {
        max_surge = 1.0;
    }
    if (max_yaw < 0.0) {
        max_yaw = 0.0;
    }
    if (max_yaw > 1.0) {
        max_yaw = 1.0;
    }
    curved_s *= max_surge;
    curved_y *= max_yaw;

    max_step = (rate_limit < 0.0 ? 0.0 : rate_limit) * dt;
    s->prev_surge = manual_rate_limit(curved_s, s->prev_surge, max_step);
    s->prev_yaw = manual_rate_limit(curved_y, s->prev_yaw, max_step);

    out->surge = s->prev_surge;
    out->yaw = s->prev_yaw;
    out->active = 1;
}
