/*
 * core/src/rc_decode.c — Decode channel RC PWM menjadi [-1..1] (C murni)
 */

#include "rc_decode.h"

int rc_pwm_valid(int pwm_us) {
    return pwm_us >= RC_PWM_VALID_LO_US && pwm_us <= RC_PWM_VALID_HI_US;
}

double rc_pwm_to_norm(int pwm_us) {
    double v;
    if (!rc_pwm_valid(pwm_us)) {
        return 0.0;
    }
    v = ((double)pwm_us - (double)RC_PWM_CENTER_US) / (double)RC_PWM_SPAN_US;
    if (v < -1.0) {
        return -1.0;
    }
    if (v > 1.0) {
        return 1.0;
    }
    return v;
}

void rc_decode(const int *ch_us, int n_ch,
               int ch_throttle, int ch_yaw,
               int ch_mode, int ch_deadman,
               rc_decoded_t *out) {
    int t, y, m, d;

    if (out == 0) {
        return;
    }
    out->surge = 0.0;
    out->yaw = 0.0;
    out->mode_manual = 0;
    out->deadman = 0;
    out->rc_ok = 0;

    if (ch_us == 0 || n_ch <= 0 || n_ch > RC_MAX_CHANNELS) {
        return;
    }
    /* Nomor channel 1-based; di luar jangkauan = konfigurasi salah. */
    if (ch_throttle < 1 || ch_throttle > n_ch ||
        ch_yaw < 1 || ch_yaw > n_ch ||
        ch_mode < 1 || ch_mode > n_ch ||
        ch_deadman < 1 || ch_deadman > n_ch) {
        return;
    }

    t = ch_us[ch_throttle - 1];
    y = ch_us[ch_yaw - 1];
    m = ch_us[ch_mode - 1];
    d = ch_us[ch_deadman - 1];

    /* Stick wajib valid; switch boleh invalid (dianggap non-aktif). */
    if (!rc_pwm_valid(t) || !rc_pwm_valid(y)) {
        return;
    }
    out->surge = rc_pwm_to_norm(t);
    out->yaw = rc_pwm_to_norm(y);
    out->mode_manual = (rc_pwm_valid(m) && m > RC_MODE_THRESHOLD_US) ? 1 : 0;
    out->deadman = (rc_pwm_valid(d) && d > RC_DEADMAN_THRESHOLD_US) ? 1 : 0;
    out->rc_ok = 1;
}
