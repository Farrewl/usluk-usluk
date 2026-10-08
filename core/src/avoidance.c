/* ============================================================
 * core/src/avoidance.c — implementasi hindar-rintangan reaktif.
 *
 * Lihat avoidance.h untuk kontrak. Diracik rentan & deterministik
 * (tanpa alokasi, tanpa state global) agar aman di thread loop.
 * ============================================================ */
#include "avoidance.h"

void avoid_update(avoid_state_t *s, double det_cx_norm, double det_w_norm,
                  double dt_s) {
    int obstacle = (det_w_norm >= AVOID_W_MIN &&
                    det_cx_norm >= -AVOID_CX_LIMIT &&
                    det_cx_norm <= AVOID_CX_LIMIT);

    if (obstacle) {
        /* Kekuatan 0..1: makin lebar (dekat/besar) makin kuat. */
        double strength = (det_w_norm - AVOID_W_MIN) / (1.0 - AVOID_W_MIN);
        double bias;
        if (strength < 0.0) strength = 0.0;
        if (strength > 1.0) strength = 1.0;
        bias = -strength * det_cx_norm;
        if (bias < -1.0) bias = -1.0;
        if (bias > 1.0)  bias = 1.0;
        s->yaw_bias = bias;
        s->memory_s = AVOID_MEMORY_S;
    } else if (s->memory_s > 0.0) {
        /* Rintangan hilang: decay memori; bias tetap sampai 0 (anti-jitter). */
        s->memory_s -= (dt_s > 0.0) ? dt_s : 0.0;
        if (s->memory_s <= 0.0) {
            s->memory_s = 0.0;
            s->yaw_bias = 0.0;
        }
    }
}