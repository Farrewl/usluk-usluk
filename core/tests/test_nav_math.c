/*
 * core/tests/test_nav_math.c — Unit test matematika navigasi (C)
 *
 * Menguji nav_math.c dengan nilai acuan NYATA (bukan asal angka):
 *   - 1 derajat lintang  = keliling Bumi / 360 = 111.194,926 m
 *   - bearing utara = 0 rad, timur = +pi/2 rad
 *   - round-trip destination: jalan 1000 m ke utara harus kembali ~1000 m
 *
 * Jalankan langsung:
 *   gcc -I core/include core/tests/test_nav_math.c core/src/nav_math.c -lm -o /tmp/opencode/test_nav_math
 *   /tmp/opencode/test_nav_math
 * (atau via ctest/CMake).
 */
#include "nav_math.h"

#include <math.h>
#include <stdio.h>

static int failures = 0;

/* Cek |got - expected| <= tol; tampilkan nama kasus biar mudah dibaca. */
static void check_close(const char *name, double got, double expected, double tol) {
    const double diff = fabs(got - expected);
    if (diff > tol) {
        printf("FAIL  %-28s got=%.17g expected=%.17g diff=%.3g > tol=%.3g\n",
               name, got, expected, diff, tol);
        failures++;
    } else {
        printf("ok    %-28s = %.17g\n", name, got);
    }
}

int main(void) {
    double bearing = 0.0;
    double dist;
    double lat2, lon2;
    double xtd;
    const double nan = NAN;  /* padanan None di Python */

    /* --- distance & bearing --- */
    dist = nav_distance_bearing(0.0, 0.0, 1.0, 0.0, &bearing);
    check_close("jarak 1 derajat lintang", dist, 111194.92664455873, 1e-6);
    check_close("bearing ke utara", bearing, 0.0, 1e-12);

    dist = nav_distance_bearing(0.0, 0.0, 0.0, 1.0, &bearing);
    check_close("bearing ke timur (+pi/2)", bearing, NAV_PI / 2.0, 1e-12);

    dist = nav_distance_bearing(-7.9153897, 112.589158,
                                -7.9153897, 112.589158, &bearing);
    check_close("jarak titik sama = 0", dist, 0.0, 1e-6);

    /* input invalid (NaN) -> INFINITY, bearing 0 */
    dist = nav_distance_bearing(nan, 0.0, 1.0, 0.0, &bearing);
    check_close("input NaN -> jarak inf", dist, INFINITY, 0.0);
    check_close("input NaN -> bearing 0", bearing, 0.0, 0.0);

    /* --- normalize_angle --- */
    check_close("normalisasi 3*pi/2 -> -pi/2",
                nav_normalize_angle(3.0 * NAV_PI / 2.0), -NAV_PI / 2.0, 1e-12);
    check_close("normalisasi -3*pi -> -pi",
                nav_normalize_angle(-3.0 * NAV_PI), -NAV_PI, 1e-12);
    check_close("normalisasi pi (batas) -> pi",
                nav_normalize_angle(NAV_PI), NAV_PI, 1e-12);

    /* --- destination round-trip --- */
    nav_destination(-7.9153897, 112.589158, 1000.0, 0.0, &lat2, &lon2);
    dist = nav_distance_bearing(-7.9153897, 112.589158, lat2, lon2, &bearing);
    check_close("round-trip 1000 m ke utara", dist, 1000.0, 1e-3);

    /* --- cross-track --- */
    xtd = nav_cross_track_distance(0.0, 0.005, 0.0, 0.0, 0.0, 0.01);
    check_close("titik di garis -> simpangan 0", xtd, 0.0, 1e-6);

    /* garis arah timur; P di UTARA = sisi kiri (port) -> NEGATIF.
       Nilai ini sama-sama dihitung Python & C pada test-cros-check. */
    xtd = nav_cross_track_distance(0.001, 0.005, 0.0, 0.0, 0.0, 0.01);
    check_close("port-side 0.1 deg utara garis", xtd, -111.19492664455875, 1e-6);

    /* P di SELATAN garis = kanan (starboard) -> POSITIF */
    xtd = nav_cross_track_distance(-0.001, 0.005, 0.0, 0.0, 0.0, 0.01);
    check_close("starboard-side 0.1 deg selatan", xtd, 111.19492664455875, 1e-6);

    if (failures == 0) {
        printf("\nSEMUA TEST LULUS (%d kasus)\n", 14);
        return 0;
    }
    printf("\n%d TEST GAGAL\n", failures);
    return 1;
}