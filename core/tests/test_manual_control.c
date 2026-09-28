/*
 * core/tests/test_manual_control.c — Unit test kurva stick manual (C murni)
 *
 * Kontrak (lihat core/include/manual_control.h):
 *   - Tanpa enabled/deadman/rc_ok -> output 0 + memori nol.
 *   - Deadband: |x| < deadband -> 0. Expo 0 = linear pasca-rescale.
 *   - Output <= MANUAL_MAX_*. Langkah <= rate_limit*dt.
 *
 * Jalankan langsung:
 *   gcc -Wall -Wextra -I core/include core/tests/test_manual_control.c \
 *       core/src/manual_control.c -lm -o /tmp/opencode/test_manual \
 *   && /tmp/opencode/test_manual
 */
#include "manual_control.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

static void check_close(const char *name, double got, double expected,
                        double tol) {
    const double diff = fabs(got - expected);
    if (diff > tol) {
        printf("FAIL  %-24s got=%.17g expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("ok    %-24s = %.17g\n", name, got);
    }
}

static void check_int(const char *name, int got, int expected) {
    if (got != expected) {
        printf("FAIL  %-24s got=%d expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("ok    %-24s = %d\n", name, got);
    }
}

int main(void) {
    /* Non-aktif -> nol. */
    {
        manual_state_t s = {0.5, 0.5}; /* isi kotor -> harus di-nol-kan */
        manual_out_t o;
        manual_update(&s, 0.8, 0.5, 1, 0, 1, 0.05, 0.3, 2.0, 0.05,
                      0.6, 0.7, &o);
        check_close("mati surge=0", o.surge, 0.0, 0.0);
        check_int("mati active=0", o.active, 0);
        check_close("mati memori nol", s.prev_surge, 0.0, 0.0);
    }

    /* Aktif tapi dalam deadband -> kurva 0, tetap aktif. */
    {
        manual_state_t s = {0.0, 0.0};
        manual_out_t o;
        manual_update(&s, 0.02, -0.02, 1, 1, 1, 0.05, 0.3, 2.0, 0.05,
                      0.6, 0.7, &o);
        check_close("deadband surge=0", o.surge, 0.0, 0.0);
        check_int("deadband active=1", o.active, 1);
    }

    /* Stick penuh: aktif, terbatas gas & rate-limit. */
    {
        manual_state_t s = {0.0, 0.0};
        manual_out_t o;
        manual_update(&s, 1.0, 1.0, 1, 1, 1, 0.05, 0.3, 2.0, 0.05,
                      0.6, 0.7, &o);
        check_int("penuh active=1", o.active, 1);
        if (fabs(o.surge) > 0.6 + 1e-9 || fabs(o.yaw) > 0.7 + 1e-9) {
            printf("FAIL  batas gas surge=%.4g yaw=%.4g\n", o.surge, o.yaw);
            failures++;
        } else {
            printf("ok    batas gas surge=%.4g yaw=%.4g\n", o.surge, o.yaw);
        }
        if (fabs(o.surge) > 2.0 * 0.05 + 1e-9) {
            printf("FAIL  rate-limit surge=%.4g\n", o.surge);
            failures++;
        } else {
            printf("ok    rate-limit surge=%.4g\n", o.surge);
        }
    }

    if (failures == 0) {
        printf("SEMUA OK (manual_control)\n");
        return 0;
    }
    printf("%d KEGAGALAN (manual_control)\n", failures);
    return 1;
}
