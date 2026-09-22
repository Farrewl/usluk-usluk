/*
 * core/include/nav_math.h — API matematika navigasi ASV (C murni)
 *
 * Port dari app/geo.py (Python). Semua rumus & urutan operasi dibuat IDENTIK
 * dengan Python agar hasilnya bisa dibandingkan bit-per-bit di
 * tests/test_core_nav_math.py. Kalau ada selisih, test itu yang menyuarakannya.
 *
 * Dipakai oleh: core/src/state_machine.c (nanti), sisanya di docs/ARCHITECTURE.md
 *
 * Konvensi tanda:
 *   - Bearing  : radian, 0 = utara, positif = searah jarum jam (timur).
 *   - Cross-track: positif = P di KANAN arah perjalanan (starboard).
 *   - Input invalid: koordinat NaN dianggap "tidak ada data" (padanan None di
 *     Python) -> nav_distance_bearing mengembalikan INFINITY, bearing 0.0.
 *
 * Semua koordinat dalam derajat, semua sudut/fungsi internal dalam radian.
 * Alasan model bola: cukup akurat untuk rentang misi Sprint (< 2 km) dan
 * jauh lebih cepat/sederhana daripada model ellipsoid saat di-port ke embedded.
 */
#ifndef ASV_NAV_MATH_H
#define ASV_NAV_MATH_H

/* Jari-jari Bumi rata-rata (meter) — sumber: IUGG. 1 satuan ini tidak
 * boleh "hilang" di tengah kode; selalu pakai konstanta di bawah. */
#define NAV_EARTH_RADIUS_M 6371000.0

/* pi 64-bit (mendekati math.pi milik CPython) */
#define NAV_PI 3.14159265358979323846

#ifdef __cplusplus
extern "C" {
#endif

/* Jarak (m) & bearing (rad) antara dua koordinat GPS.
 * Rumus: haversine (jarak) + atan2(y,x) (bearing) — lihat app/geo.py.
 * Output bearing ditulis ke *bearing_out. Return = jarak. */
double nav_distance_bearing(double lat1, double lon1,
                            double lat2, double lon2,
                            double *bearing_out);

/* Koordinat tujuan setelah berjalan sejauh distance_m dengan arah
 * bearing_rad — balikan dari nav_distance_bearing (dipakai simulator). */
void nav_destination(double lat, double lon, double distance_m,
                     double bearing_rad, double *lat_out, double *lon_out);

/* Bawa sudut (radian) ke rentang [-pi, pi] — untuk membandingkan yaw.
 * Memakai pengulangan (bukan fmod) supaya persis sama dengan Python:
 * fmod harus ekstra hati-hati dengan nilai negatif. */
double nav_normalize_angle(double angle_rad);

/* Simpangan titik P terhadap garis lurus WP1 -> WP2 (meter).
 * Dipakai geofence lintasan: kalau melenceng melebihi
 * GEOFENCE_WIDTH_METERS, kapal dikoreksi. */
double nav_cross_track_distance(double lat_p, double lon_p,
                                double lat_wp1, double lon_wp1,
                                double lat_wp2, double lon_wp2);

#ifdef __cplusplus
}
#endif

#endif /* ASV_NAV_MATH_H */