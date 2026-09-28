/*
 * core/tests/test_mode_manager.c — Unit test state mode AUTO/MANUAL/HOLD/KILL
 *
 * Kontrak (lihat core/include/mode_manager.h):
 *   KILL > MANUAL > HOLD > AUTO. RC putus saat MANUAL -> HOLD selama
 *   lost_hold_s detik lalu AUTO. Keluar KILL -> AUTO.
 *
 * Jalankan langsung:
 *   gcc -Wall -Wextra -I core/include core/tests/test_mode_manager.c \
 *       core/src/mode_manager.c -o /tmp/opencode/test_mode \
 *   && /tmp/opencode/test_mode
 */
#include "mode_manager.h"

#include <stdio.h>

static int failures = 0;

static void check_int(const char *name, int got, int expected) {
    if (got != expected) {
        printf("FAIL  %-28s got=%d(%s) expected=%d(%s)\n", name, got,
               mode_name((op_mode_t)got), expected,
               mode_name((op_mode_t)expected));
        failures++;
    } else {
        printf("ok    %-28s = %s\n", name, mode_name((op_mode_t)got));
    }
}

int main(void) {
    const double dt = 0.05;
    const double hold = 1.0;

    /* KILL menang; lepas kill -> kembali lewat AUTO. */
    {
        mode_state_t s;
        mode_reset(&s);
        check_int("kill menang", mode_update(&s, 1, 1, 1, dt, hold),
                  OP_MODE_KILL);
        check_int("lepas kill+minta manual", mode_update(&s, 0, 1, 1, dt, hold),
                  OP_MODE_MANUAL);
        mode_update(&s, 1, 0, 0, dt, hold);
        check_int("lepas kill polos", mode_update(&s, 0, 0, 0, dt, hold),
                  OP_MODE_AUTO);
    }

    /* Minta manual tanpa RC sehat -> AUTO. */
    {
        mode_state_t s;
        mode_reset(&s);
        check_int("manual tanpa rc", mode_update(&s, 0, 1, 0, dt, hold),
                  OP_MODE_AUTO);
        check_int("manual+rc", mode_update(&s, 0, 1, 1, dt, hold),
                  OP_MODE_MANUAL);
    }

    /* RC putus saat MANUAL -> HOLD -> AUTO setelah hold detik. */
    {
        mode_state_t s;
        int i, got = OP_MODE_AUTO;
        mode_reset(&s);
        mode_update(&s, 0, 1, 1, dt, hold);
        check_int("rc putus -> hold", mode_update(&s, 0, 1, 0, dt, hold),
                  OP_MODE_HOLD);
        for (i = 0; i < (int)(hold / dt) + 2; i++) {
            got = mode_update(&s, 0, 1, 0, dt, hold);
        }
        check_int("hold habis -> auto", got, OP_MODE_AUTO);
    }

    /* RC pulih saat HOLD -> MANUAL lagi. */
    {
        mode_state_t s;
        mode_reset(&s);
        mode_update(&s, 0, 1, 1, dt, hold);
        mode_update(&s, 0, 1, 0, dt, hold);
        check_int("pulih -> manual", mode_update(&s, 0, 1, 1, dt, hold),
                  OP_MODE_MANUAL);
    }

    if (failures == 0) {
        printf("SEMUA OK (mode_manager)\n");
        return 0;
    }
    printf("%d KEGAGALAN (mode_manager)\n", failures);
    return 1;
}
