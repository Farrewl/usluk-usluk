/*
 * core/tests/test_gate_vision.c — Unit test sequencer gate + validasi (C).
 *
 * Nilai acuan NYATA (bukan asal angka):
 *   - pinhole: (1.0 m * 400 px) / 100 px = 4.0 m
 *   - pixel <= 5 -> +inf (tak reliabel)
 *   - buoy 10x10 lolos; 2x2 / aspek 3:1 / area < min ditolak
 *   - latch: frame kosong < toleransi -> target lama dipertahankan
 *
 * Jalankan:
 *   gcc -Wall -Wextra -I core/include core/tests/test_gate_vision.c \
 *       core/src/gate_vision.c -lm -o /tmp/opencode/test_gv \
 *   && /tmp/opencode/test_gv
 */
#include "gate_vision.h"

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
    double d = fabs(got - expected);
    if (d > tol) {
        printf("FAIL  %-28s got=%.6g expected=%.6g\n", name, got, expected);
        failures++;
    } else {
        printf("ok    %-28s = %.6g\n", name, got);
    }
}

int main(void) {
    /* --- pinhole --- */
    check_close("pinhole 4m", gv_estimate_distance(100.0, 1.0, 400.0),
                4.0, 1e-9);
    check_int("pixel<=5 inf",
              gv_estimate_distance(5.0, 1.0, 400.0) == HUGE_VAL, 1);

    /* --- geometri buoy --- */
    check_int("10x10 lolos", gv_buoy_ok(10, 10, 16, 0.5), 1);
    check_int("2x2 tolak", gv_buoy_ok(2, 2, 16, 0.5), 0);
    check_int("aspek 3:1 tolak", gv_buoy_ok(30, 10, 16, 0.5), 0);
    check_int("area<min tolak", gv_buoy_ok(3, 3, 16, 0.5), 0);

    /* --- kode alasan gagal (dipakai lapisan Python utk pesan debug) --- */
    check_int("fail lolos=0", gv_buoy_fail(10, 10, 16, 0.5), 0);
    check_int("fail kecil=1", gv_buoy_fail(2, 2, 16, 0.5), 1);
    check_int("fail area=2", gv_buoy_fail(3, 3, 16, 0.5), 2);
    check_int("fail aspek=3", gv_buoy_fail(30, 10, 16, 0.5), 3);

    /* --- koleksi pasangan --- */
    {
        gv_ball_t red[2] = {{100, 200, 500}, {300, 400, 100}};
        gv_ball_t grn[2] = {{150, 205, 520}, {500, 100, 500}};
        gv_pair_t out[GV_MAX_PAIRS];
        int n = gv_collect_pairs(red, 2, grn, 2, 75.0, 0.5, out);
        /* (100,200)-(150,205): dy=5 lolos, mirip luas lolos.
         * (100,200)-(500,100): dy=100 tolak. (300,400)-*: dy besar tolak. */
        check_int("1 pasangan", n, 1);
        if (n == 1) {
            check_int("red idx 0", out[0].red_idx, 0);
            check_int("green idx 0", out[0].green_idx, 0);
        }
    }

    /* --- latch sequencer --- */
    {
        gv_seq_t s;
        gv_ball_t red[1] = {{100, 200, 500}};
        gv_ball_t grn[1] = {{150, 205, 520}};
        gv_pair_t pairs[1] = {{0, 0}};
        double mx, my, dist;
        int passed, has;
        gv_seq_init(&s, 1.2, 5, 1.0, 400.0, 6.0, 30.0, 1.5);
        has = gv_seq_update(&s, red, 1, grn, 1, pairs, 1, 360.0,
                            &mx, &my, &dist, &passed);
        check_int("ada target", has, 1);
        check_close("mid_x 125", mx, 125.0, 1e-9);
        check_int("belum lewat", passed, 0);
        /* Frame kosong: tahan target lama (toleransi 5). */
        has = gv_seq_update(&s, red, 0, grn, 0, pairs, 0, 360.0,
                            &mx, &my, &dist, &passed);
        check_int("latch saat hilang", has, 1);
        check_close("mid latch 125", mx, 125.0, 1e-9);
        /* Habiskan toleransi -> kosong. */
        {
            int k;
            for (k = 0; k < 6; k++) {
                has = gv_seq_update(&s, red, 0, grn, 0, pairs, 0,
                                    360.0, &mx, &my, &dist, &passed);
            }
        }
        check_int("habis toleransi kosong", has, 0);
    }

    if (failures == 0) {
        printf("SEMUA OK (gate_vision)\n");
        return 0;
    }
    printf("%d KEGAGALAN (gate_vision)\n", failures);
    return 1;
}
