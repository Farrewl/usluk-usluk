/*
 * core/tests/test_rc_decode.c — Unit test decode RC PWM (C murni)
 *
 * Kontrak (lihat core/include/rc_decode.h):
 *   - PWM valid 900..2100; normalisasi (pwm-1500)/500 clamp [-1..1].
 *   - Stick invalid -> rc_ok=0. Channel salah -> rc_ok=0.
 *   - Switch > 1500 = aktif; invalid = non-aktif (bukan gagal).
 *
 * Jalankan langsung:
 *   gcc -Wall -Wextra -I core/include core/tests/test_rc_decode.c \
 *       core/src/rc_decode.c -o /tmp/opencode/test_rc_decode \
 *   && /tmp/opencode/test_rc_decode
 */
#include "rc_decode.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

static void check_int(const char *name, int got, int expected) {
    if (got != expected) {
        printf("FAIL  %-28s got=%d expected=%d\n", name, got, expected);
        failures++;
    } else {
        printf("ok    %-28s = %d\n", name, got);
    }
}

static void check_close(const char *name, double got, double expected,
                        double tol) {
    const double diff = fabs(got - expected);
    if (diff > tol) {
        printf("FAIL  %-28s got=%.17g expected=%.17g\n", name, got, expected);
        failures++;
    } else {
        printf("ok    %-28s = %.17g\n", name, got);
    }
}

int main(void) {
    /* Semua tengah: netral, switch mati, rc_ok. */
    {
        int ch[16] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
        rc_decoded_t o;
        rc_decode(ch, 16, 3, 4, 5, 7, &o);
        check_close("tengah surge=0", o.surge, 0.0, 1e-12);
        check_close("tengah yaw=0", o.yaw, 0.0, 1e-12);
        check_int("tengah mode mati", o.mode_manual, 0);
        check_int("tengah deadman mati", o.deadman, 0);
        check_int("tengah rc_ok", o.rc_ok, 1);
    }

    /* Defleksi penuh + switch aktif. */
    {
        int ch[16] = {1500, 1500, 2000, 1000, 1800, 1500, 1900, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
        rc_decoded_t o;
        rc_decode(ch, 16, 3, 4, 5, 7, &o);
        check_close("penuh surge=+1", o.surge, 1.0, 1e-12);
        check_close("penuh yaw=-1", o.yaw, -1.0, 1e-12);
        check_int("penuh mode aktif", o.mode_manual, 1);
        check_int("penuh deadman aktif", o.deadman, 1);
        check_int("penuh rc_ok", o.rc_ok, 1);
    }

    /* Stick rusak -> failsafe. */
    {
        int ch[16] = {1500, 1500, 500, 1500, 1500, 1500, 1500, 1500,
                      1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
        rc_decoded_t o;
        rc_decode(ch, 16, 3, 4, 5, 7, &o);
        check_int("rusak rc_ok=0", o.rc_ok, 0);
        check_close("rusak surge=0", o.surge, 0.0, 0.0);
    }

    /* Konfigurasi channel salah -> rc_ok=0. */
    {
        int ch[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
        rc_decoded_t o;
        rc_decode(ch, 8, 9, 4, 5, 7, &o);
        check_int("salah rc_ok=0", o.rc_ok, 0);
    }

    if (failures == 0) {
        printf("SEMUA OK (rc_decode)\n");
        return 0;
    }
    printf("%d KEGAGALAN (rc_decode)\n", failures);
    return 1;
}
