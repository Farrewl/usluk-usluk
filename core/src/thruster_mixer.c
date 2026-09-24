/* Mixer differential drive — urutan operasi IDENTIK acuan Python (bit-per-bit). */

#include "thruster_mixer.h"

double mixer_clamp(double val, double lo, double hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

void mixer_diff_drive(double surge, double yaw,
                      int *pwm_left, int *pwm_right) {
    double left_us, right_us;

    /* Input di-clamp dulu agar tak merusak ESC. */
    surge = mixer_clamp(surge, -MIXER_INPUT_LIMIT, MIXER_INPUT_LIMIT);
    yaw   = mixer_clamp(yaw,   -MIXER_INPUT_LIMIT, MIXER_INPUT_LIMIT);

    /* input 1.0 = 400 us di atas/di bawah center. */
    double span = (double)(MIXER_MAX_PWM - MIXER_CENTER_PWM); /* 400 µs */
    left_us  = (double)MIXER_CENTER_PWM + surge * span - yaw * span;
    right_us = (double)MIXER_CENTER_PWM + surge * span + yaw * span;

    /* Output 1100..1900 us. */
    left_us  = mixer_clamp(left_us,  (double)MIXER_MIN_PWM, (double)MIXER_MAX_PWM);
    right_us = mixer_clamp(right_us, (double)MIXER_MIN_PWM, (double)MIXER_MAX_PWM);

    if (pwm_left)  *pwm_left  = (int)(left_us  + 0.5);
    if (pwm_right) *pwm_right = (int)(right_us + 0.5);
}
