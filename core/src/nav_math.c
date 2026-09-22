/*
 * core/src/nav_math.c — Implementasi matematika navigasi (C murni)
 *
 * Port setia app/geo.py. Aturan port:
 *   1. Urutan operasi aritmetika disamakan (sin*sin untuk kuadrat, bukan
 *      pow, supaya presisi & hasil dekat dengan Python).
 *   2. radians  = deg * (pi/180)   == math.radians
 *      degrees  = rad * (180/pi)   == math.degrees
 *   3. Nilai invalid (NaN) mengembalikan INFINITY / 0.0 seperti Python
 *      saat menerima None.
 *
 * Satu file tanpa dependensi internal lain — mudah diuji sendiri dan
 * mudah diport lagi ke MCU kalau nanti mau loncat dari Raspberry Pi.
 */
#include "nav_math.h"

#include <math.h>

double nav_distance_bearing(double lat1, double lon1,
                            double lat2, double lon2,
                            double *bearing_out) {
    /* Nilai aman default: pemanggil yang lupa cek error tetap dapat
     * jarak "tak hingga" sehingga logika navigasi memilih berhati-hati. */
    *bearing_out = 0.0;
    if (isnan(lat1) || isnan(lon1) || isnan(lat2) || isnan(lon2)) {
        return INFINITY;
    }

    /* -- haversine -- */
    const double phi1 = lat1 * (NAV_PI / 180.0);        /* math.radians */
    const double phi2 = lat2 * (NAV_PI / 180.0);
    const double d_phi = (lat2 - lat1) * (NAV_PI / 180.0);
    const double d_lambda = (lon2 - lon1) * (NAV_PI / 180.0);

    /* a = sin^2(dphi/2) + cos(phi1)*cos(phi2)*sin^2(dlambda/2) */
    double a = sin(d_phi / 2.0) * sin(d_phi / 2.0)
             + cos(phi1) * cos(phi2)
               * sin(d_lambda / 2.0) * sin(d_lambda / 2.0);
    /* jepit ke [0,1]: error presisi floating-point kecil bisa membuat a
     * sedikit < 0 atau > 1; atan2 tidak menyukai itu. */
    if (a < 0.0) a = 0.0;
    if (a > 1.0) a = 1.0;

    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    const double distance = NAV_EARTH_RADIUS_M * c;

    /* -- bearing (atan2(y,x)) -- */
    const double y = sin(d_lambda) * cos(phi2);
    const double x = cos(phi1) * sin(phi2)
                   - sin(phi1) * cos(phi2) * cos(d_lambda);
    const double bearing = atan2(y, x);

    *bearing_out = bearing;
    return distance;
}

void nav_destination(double lat, double lon, double distance_m,
                     double bearing_rad, double *lat_out, double *lon_out) {
    const double lat1 = lat * (NAV_PI / 180.0);
    const double lon1 = lon * (NAV_PI / 180.0);
    const double angular = distance_m / NAV_EARTH_RADIUS_M;

    const double lat2 = asin(sin(lat1) * cos(angular)
                           + cos(lat1) * sin(angular) * cos(bearing_rad));
    const double lon2 = lon1 + atan2(sin(bearing_rad) * sin(angular) * cos(lat1),
                                     cos(angular) - sin(lat1) * sin(lat2));

    /* math.degrees: KALI (180/pi), bukan bagi (pi/180), biar 1:1 dengan
     * Python (rounding paling akhir identik di antara dua implementasi). */
    *lat_out = lat2 * (180.0 / NAV_PI);
    *lon_out = lon2 * (180.0 / NAV_PI);
}

double nav_normalize_angle(double angle_rad) {
    while (angle_rad > NAV_PI) {
        angle_rad -= 2.0 * NAV_PI;
    }
    while (angle_rad < -NAV_PI) {
        angle_rad += 2.0 * NAV_PI;
    }
    return angle_rad;
}

double nav_cross_track_distance(double lat_p, double lon_p,
                                double lat_wp1, double lon_wp1,
                                double lat_wp2, double lon_wp2) {
    double bearing_1p = 0.0;
    double bearing_12 = 0.0;

    /* Jarak & bearing dari WP1 ke posisi kapal (P) */
    const double dist_1p = nav_distance_bearing(lat_wp1, lon_wp1,
                                                lat_p, lon_p, &bearing_1p);
    /* Bearing garis lintasan WP1 -> WP2 */
    nav_distance_bearing(lat_wp1, lon_wp1, lat_wp2, lon_wp2, &bearing_12);

    /* P tepat di titik awal / data invalid -> tidak ada simpangan */
    if (isinf(dist_1p) || dist_1p == 0.0) {
        return 0.0;
    }

    /* term = jarak_1p / R, lalu jepit agar asin() di bawah aman */
    double term = dist_1p / NAV_EARTH_RADIUS_M;
    if (term > NAV_PI) term = NAV_PI;
    if (term < -NAV_PI) term = -NAV_PI;

    /* sin(sim) = sin(term) * sin(bearing_1p - bearing_12) */
    double sin_term = sin(term) * sin(bearing_1p - bearing_12);
    if (sin_term > 1.0) sin_term = 1.0;
    if (sin_term < -1.0) sin_term = -1.0;

    return asin(sin_term) * NAV_EARTH_RADIUS_M;
}