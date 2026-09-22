"""
app/geo.py — Utilitas geodetik murni (tanpa dependensi eksternal).

Dipakai oleh app/navigator.py dan app/simulator.py agar rumus matematika
navigasi TIDAK terduplikasi di tiga tempat seperti sebelumnya.

Semua fungsi memakai model bola Bumi dengan radius rata-rata 6371 km.
Rumus & langkah tiap fungsi ditulis di komentar -> calon port langsung
ke C pada Tahap 2 (core/nav_math.c) dengan output yang identik.
"""

import math

# Jari-jari Bumi rata-rata (meter). Sumber: IUGG.
EARTH_RADIUS_M = 6371000.0


def distance_bearing(lat1, lon1, lat2, lon2):
    """Jarak (meter) dan bearing (radian, 0 = utara) antara 2 koordinat GPS.

    - Jarak  : rumus haversine (dist = 2R * atan2(sqrt(a), sqrt(1-a))).
    - Bearing: atan2(sin(dLon)*cos(phi2),
                    cos(phi1)*sin(phi2) - sin(phi1)*cos(phi2)*cos(dLon)).
    Saat input invalid -> (float('inf'), 0.0) agar pemanggil aman.
    """
    try:
        if None in (lat1, lon1, lat2, lon2):
            return float('inf'), 0.0
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        d_phi = math.radians(lat2 - lat1)
        d_lambda = math.radians(lon2 - lon1)

        a = (math.sin(d_phi / 2) ** 2
             + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
        a = max(0.0, min(1.0, a))  # jagalah agar atan2 aman dari kesalahan fp
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = EARTH_RADIUS_M * c

        y = math.sin(d_lambda) * math.cos(phi2)
        x = (math.cos(phi1) * math.sin(phi2)
             - math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda))
        bearing = math.atan2(y, x)
        return distance, bearing
    except TypeError:
        return float('inf'), 0.0


def destination(lat, lon, distance_m, bearing_rad):
    """Koordinat tujuan setelah berjalan `distance_m` dengan `bearing_rad`.

    Balikan dari distance_bearing: dipakai simulator untuk menggerakkan
    posisi mock sesuai kecepatan & heading kapal.
    """
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    angular = distance_m / EARTH_RADIUS_M

    lat2 = math.asin(math.sin(lat1) * math.cos(angular)
                     + math.cos(lat1) * math.sin(angular) * math.cos(bearing_rad))
    lon2 = lon1 + math.atan2(math.sin(bearing_rad) * math.sin(angular) * math.cos(lat1),
                             math.cos(angular) - math.sin(lat1) * math.sin(lat2))
    return math.degrees(lat2), math.degrees(lon2)


def normalize_angle(angle_rad):
    """Bawa sudut (radian) ke rentang [-pi, pi] — untuk perbandingan yaw.

    Memakai pengulangan (loop) karena praktis di C tanpa fungsi fmod yang
    harus hati-hati dengan nilai negatif.
    """
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def cross_track_distance(lat_p, lon_p, lat_wp1, lon_wp1, lat_wp2, lon_wp2):
    """Simpangan titik P terhadap garis lurus WP1 -> WP2 (meter).

    Positif berarti P berada di kanan arah perjalanan. Dipakai geofence
    lintasan: kalau melenceng melebihi GEOFENCE_WIDTH_METERS, kapal dikoreksi.
    """
    dist_1p, bearing_1p = distance_bearing(lat_wp1, lon_wp1, lat_p, lon_p)
    _, bearing_12 = distance_bearing(lat_wp1, lon_wp1, lat_wp2, lon_wp2)

    if dist_1p in (0.0, float('inf')):
        return 0.0
    term = dist_1p / EARTH_RADIUS_M
    term = max(-math.pi, min(math.pi, term))
    sin_term = math.sin(term) * math.sin(bearing_1p - bearing_12)
    sin_term = max(-1.0, min(1.0, sin_term))
    return math.asin(sin_term) * EARTH_RADIUS_M