"""Unit test matematika geodetik (app/geo.py).

Jalankan dari root repo:
    python -m unittest discover -s tests -v

Acuan nilai nyata:
  - 1 derajat lintang ≈ (pi/180) × 6371000 m = 111194.93 m  (keliling bumi)
  - bearing utara = 0 rad
"""

import math
import unittest

from app import geo


class TestDistanceBearing(unittest.TestCase):

    def test_one_degree_latitude(self):
        """Jarak 1° lintang harus ≈ keliling Bumi / 360."""
        dist, _ = geo.distance_bearing(0.0, 0.0, 1.0, 0.0)
        expected = math.pi / 180.0 * geo.EARTH_RADIUS_M  # 111194.93 m
        self.assertAlmostEqual(dist, expected, delta=10.0)

    def test_zero_distance(self):
        """Titik sama -> jarak 0, bearing tetap terdefinisi."""
        dist, _ = geo.distance_bearing(-7.9153897, 112.589158, -7.9153897, 112.589158)
        self.assertAlmostEqual(dist, 0.0, delta=1e-3)

    def test_symmetric(self):
        """dist(A,B) == dist(B,A)."""
        a = (-7.9153897, 112.589158)
        b = (-7.9154302, 112.5890491)
        d1, _ = geo.distance_bearing(*a, *b)
        d2, _ = geo.distance_bearing(*b, *a)
        self.assertAlmostEqual(d1, d2, delta=1e-6)

    def test_bearing_north(self):
        """Ke utara (menaikkan lat) -> bearing 0 rad."""
        _, bearing = geo.distance_bearing(0.0, 0.0, 1.0, 0.0)
        self.assertAlmostEqual(bearing, 0.0, delta=1e-9)

    def test_bearing_east(self):
        """Ke timur (menaikkan lon) -> bearing +pi/2."""
        _, bearing = geo.distance_bearing(0.0, 0.0, 0.0, 1.0)
        self.assertAlmostEqual(bearing, math.pi / 2, delta=1e-9)

    def test_invalid_input_safe(self):
        """None masuk -> (inf, 0) bukan exception."""
        dist, bearing = geo.distance_bearing(None, 0.0, 1.0, 0.0)
        self.assertEqual(dist, float('inf'))
        self.assertEqual(bearing, 0.0)


class TestNormalizeAngle(unittest.TestCase):

    def test_keep_in_range(self):
        self.assertAlmostEqual(geo.normalize_angle(0.5), 0.5)
        self.assertAlmostEqual(geo.normalize_angle(math.pi), math.pi)
        self.assertAlmostEqual(geo.normalize_angle(-math.pi), -math.pi, delta=1e-9)

    def test_wrap_large_positive(self):
        """3*pi/2 lebih besar dari pi -> -pi/2."""
        self.assertAlmostEqual(geo.normalize_angle(3 * math.pi / 2), -math.pi / 2, delta=1e-9)

    def test_wrap_large_negative(self):
        self.assertAlmostEqual(geo.normalize_angle(-3 * math.pi), -math.pi, delta=1e-9)


class TestDestination(unittest.TestCase):

    def test_round_trip(self):
        """Jalan ke utara 1000 m lalu ukur kembali jaraknya ≈ 1000 m."""
        lat, lon = -7.9153897, 112.589158
        lat2, lon2 = geo.destination(lat, lon, 1000.0, 0.0)
        dist, _ = geo.distance_bearing(lat, lon, lat2, lon2)
        self.assertAlmostEqual(dist, 1000.0, delta=2.0)

    def test_zero_distance_no_move(self):
        lat, lon = geo.destination(-7.9, 112.5, 0.0, 1.0)
        self.assertAlmostEqual(lat, -7.9, delta=1e-9)
        self.assertAlmostEqual(lon, 112.5, delta=1e-9)


class TestCrossTrack(unittest.TestCase):

    def test_point_on_line(self):
        """Titik di tengah garis WP1-WP2 -> simpangan ~0."""
        lat_wp1, lon_wp1 = 0.0, 0.0
        lat_wp2, lon_wp2 = 0.0, 0.01
        # P tepat di tengah garis (timur-barat)
        xtd = geo.cross_track_distance(0.0, 0.005, lat_wp1, lon_wp1, lat_wp2, lon_wp2)
        self.assertAlmostEqual(xtd, 0.0, delta=1e-6)

    def test_point_off_line_negative_port_side(self):
        """Titik di UTARA garis arah timur = sisi kiri (port) -> simpangan NEGATIF.
        Konvensi: kanan (starboard) = positif, kiri (port) = negatif."""
        lat_wp1, lon_wp1 = 0.0, 0.0
        lat_wp2, lon_wp2 = 0.0, 0.01   # garis mengarah ke timur
        xtd = geo.cross_track_distance(0.001, 0.005, lat_wp1, lon_wp1, lat_wp2, lon_wp2)
        self.assertLess(xtd, 0.0)

    def test_point_off_line_positive_starboard_side(self):
        """Titik di SELATAN garis arah timur = sisi kanan (starboard) -> positif."""
        lat_wp1, lon_wp1 = 0.0, 0.0
        lat_wp2, lon_wp2 = 0.0, 0.01   # garis mengarah ke timur
        xtd = geo.cross_track_distance(-0.001, 0.005, lat_wp1, lon_wp1, lat_wp2, lon_wp2)
        self.assertGreater(xtd, 0.0)


if __name__ == "__main__":
    unittest.main()