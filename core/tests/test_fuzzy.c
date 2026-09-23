/*
 * core/tests/test_fuzzy.c — Unit test controller fuzzy (C)
 *
 * Nilai emas di bawah dihitung dari app/fuzzy.py (referensi Python),
 * bukan asal angka — modul C harus identik dengan Python.
 * Kasus dipilih: satu titik per sel aturan 3x3 + titik transisi antar
 * sel + input di luar semesta (harus 0.0, tak ada aturan menyala).
 *
 * Jalankan:
 *   gcc -I core/include core/tests/test_fuzzy.c core/src/fuzzy.c -lm \
 *       -o /tmp/opencode/test_fuzzy && /tmp/opencode/test_fuzzy
 */
#include "fuzzy.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

/* Cek |got - expected| <= tol; tampilkan nama kasus biar mudah dibaca. */
static void check_close(const char *name, double got, double expected,
                        double tol) {
    const double diff = fabs(got - expected);
    if (diff > tol) {
        printf("FAIL  %-22s got=%.12f expected=%.12f diff=%.3g > tol=%.3g\n",
               name, got, expected, diff, tol);
        failures++;
    } else {
        printf("ok    %-22s = %.12f\n", name, got);
    }
}

int main(void) {
    const double g = 1e-9; /* toleransi ketat: C harus SEPERSIS Python */

    /* ---- GATE (koreksi buoy) ---- */
    check_close("gate dekat kecil",   fuzzy_gate_p_gain(0.1, 10),   1.0, g);
    check_close("gate dekat sedang",  fuzzy_gate_p_gain(0.1, 60),   1.9, g);
    check_close("gate dekat besar",   fuzzy_gate_p_gain(0.1, 200),  2.5, g);
    check_close("gate tengah kecil",  fuzzy_gate_p_gain(0.5, 10),   1.0, g);
    check_close("gate tengah sedang", fuzzy_gate_p_gain(0.5, 60),   1.9, g);
    check_close("gate tengah besar",  fuzzy_gate_p_gain(0.5, 200),  2.5, g);
    check_close("gate jauh kecil",    fuzzy_gate_p_gain(0.9, 10),   1.9, g);
    check_close("gate jauh sedang",   fuzzy_gate_p_gain(0.9, 60),   2.5, g);
    check_close("gate jauh besar",    fuzzy_gate_p_gain(0.9, 200),  2.5, g);
    check_close("gate transisi 1",    fuzzy_gate_p_gain(0.3, 35),   1.36, g);
    check_close("gate transisi 2",    fuzzy_gate_p_gain(0.75, 100), 2.2, g);
    check_close("gate luar semesta",  fuzzy_gate_p_gain(5.0, 500),  0.0, g);

    /* ---- DOCKING (koreksi box merah) ---- */
    check_close("dock dekat",         fuzzy_docking_p_gain(0.1, 10),   0.5, g);
    check_close("dock tgh kecil",     fuzzy_docking_p_gain(0.2, 10),   0.5, g);
    check_close("dock tgh sedang",    fuzzy_docking_p_gain(0.2, 60),   1.2, g);
    check_close("dock tgh besar",     fuzzy_docking_p_gain(0.2, 200),  1.9, g);
    check_close("dock jauh kecil",    fuzzy_docking_p_gain(0.5, 10),   1.2, g);
    check_close("dock jauh sedang",   fuzzy_docking_p_gain(0.5, 60),   1.9, g);
    check_close("dock jauh besar",    fuzzy_docking_p_gain(0.5, 200),  1.9, g);
    check_close("dock transisi",      fuzzy_docking_p_gain(0.45, 140), 1.9, g);
    check_close("dock luar semesta",  fuzzy_docking_p_gain(5.0, 500),  0.0, g);

    if (failures == 0) {
        printf("\nSEMUA 22 kasus fuzzy LULUS (C == app/fuzzy.py)\n");
        return 0;
    }
    printf("\n%d kasus GAGAL\n", failures);
    return 1;
}