"""Test app/fuzzy.py — controller Sugeno singleton (gain P dinamis).

Memvalidasi tabel & perhitungan referensi Python yang dipakai port C
(core/src/fuzzy.c). Kasus bernilai "nyata" berasal dari desain controller:
  - satu titik per sel aturan 3x3 (gate) / 3x3+1 (docking)
  - transisi antar sel (dua aturan menyala)
  - input di luar semesta -> 0.0 (tidak ada aturan menyala)

Jalankan:  python3 -m unittest discover -s tests -v
"""

import random
import unittest

from app.fuzzy import (
    GATE_RULES,
    DOCKING_RULES,
    _trapmf,
    docking_p_gain,
    gate_p_gain,
)


class TestTrapmf(unittest.TestCase):
    """Fungsi keanggotaan trapesium: nilai di kaki/plateau/luar."""

    def test_plateau_satu(self):
        self.assertEqual(_trapmf(0.1, 0.0, 0.0, 0.2, 0.4), 1.0)  # DEKAT gate
        self.assertEqual(_trapmf(0.9, 0.7, 0.8, 1.0, 1.0), 1.0)  # JAUH gate (bahu kanan)

    def test_kaki_linier(self):
        # kanan DEKAT gate [0,0,0.2,0.4]: 0.3 -> (0.4-0.3)/(0.4-0.2) = 0.5
        self.assertAlmostEqual(_trapmf(0.3, 0.0, 0.0, 0.2, 0.4), 0.5)
        # kiri BESAR error [110,150,320,320]: 130 -> (130-110)/(150-110) = 0.5
        self.assertAlmostEqual(_trapmf(130.0, 110.0, 150.0, 320.0, 320.0), 0.5)

    def test_luar_semesta_nol(self):
        self.assertEqual(_trapmf(0.0, 0.0, 0.0, 0.2, 0.4), 0.0)  # x<=a
        self.assertEqual(_trapmf(0.4, 0.0, 0.0, 0.2, 0.4), 0.0)  # x>=d
        self.assertEqual(_trapmf(5.0, 0.0, 0.0, 0.2, 0.4), 0.0)
        self.assertEqual(_trapmf(-1.0, 0.0, 0.0, 0.2, 0.4), 0.0)


class TestGateController(unittest.TestCase):
    """Satu titik per 9 sel aturan GATE (jarak DEKAT/SEDANG/JAUH x err KECIL/SEDANG/BESAR)."""

    def test_tiap_sel_aturan(self):
        cases = [
            (0.1, 10, 1.0),   # DEKAT & KECIL  -> RENDAH
            (0.1, 60, 1.9),   # DEKAT & SEDANG -> SEDANG
            (0.1, 200, 2.5),  # DEKAT & BESAR  -> TINGGI
            (0.5, 10, 1.0),   # SEDANG & KECIL -> RENDAH
            (0.5, 60, 1.9),   # SEDANG & SEDANG-> SEDANG
            (0.5, 200, 2.5),  # SEDANG & BESAR -> TINGGI
            (0.9, 10, 1.9),   # JAUH & KECIL   -> SEDANG
            (0.9, 60, 2.5),   # JAUH & SEDANG  -> TINGGI
            (0.9, 200, 2.5),  # JAUH & BESAR   -> TINGGI
        ]
        for jarak, error, expected in cases:
            with self.subTest(jarak=jarak, error=error):
                self.assertAlmostEqual(gate_p_gain(jarak, error), expected)

    def test_transisi_dua_aturan(self):
        # 0.3 m & 35 px: DEKAT(0.3)=0.5 kanan-trapmf; KECIL(35)=0.25,
        # SEDANG(35)=0.1667. Hanya rule 1 & 2 menyala:
        #   alpha1 = min(0.5, 0.25) = 0.25  -> 1.0
        #   alpha2 = min(0.5, 0.1667) = 0.1667 -> 1.9
        # rata-rata = (0.25*1.0 + 0.1667*1.9) / 0.4167 = 1.36
        self.assertAlmostEqual(gate_p_gain(0.3, 35), 1.36)
        # 0.75 m & 100 px: SEDANG(0.75)=0.5, JAUH(0.75)=0.5, SEDANG(100)=1.0.
        # rule 5 & 8 menyala -> (0.5*1.9 + 0.5*2.5) / 1.0 = 2.2
        self.assertAlmostEqual(gate_p_gain(0.75, 100), 2.2)

    def test_luar_semesta_nol(self):
        self.assertEqual(gate_p_gain(5.0, 500), 0.0)
        self.assertEqual(gate_p_gain(2.0, 320), 0.0)  # tepat di tepi semesta

    def test_jumlah_aturan(self):
        self.assertEqual(len(GATE_RULES), 9)


class TestDockingController(unittest.TestCase):
    """7 aturan DOCKING: DEKAT(murni jarak), SEDANG/JAUH x 3 error."""

    def test_tiap_sel_aturan(self):
        cases = [
            (0.1, 10, 0.5),   # DEKAT (hanya jarak) -> RENDAH
            (0.2, 10, 0.5),   # SEDANG & KECIL -> RENDAH
            (0.2, 60, 1.2),   # SEDANG & SEDANG -> SEDANG
            (0.2, 200, 1.9),  # SEDANG & BESAR -> TINGGI
            (0.5, 10, 1.2),   # JAUH & KECIL -> SEDANG
            (0.5, 60, 1.9),   # JAUH & SEDANG -> TINGGI
            (0.5, 200, 1.9),  # JAUH & BESAR -> TINGGI
        ]
        for jarak, error, expected in cases:
            with self.subTest(jarak=jarak, error=error):
                self.assertAlmostEqual(docking_p_gain(jarak, error), expected)

    def test_dekat_tidak_terpengaruh_error(self):
        # Di zona DEKAT (jarak<0.12) rule 1 menyala penuh -> 0.5 apa pun error.
        for error in (0, 30, 150, 320):
            self.assertAlmostEqual(docking_p_gain(0.08, error), 0.5)

    def test_luar_semesta_nol(self):
        self.assertEqual(docking_p_gain(5.0, 500), 0.0)

    def test_jumlah_aturan(self):
        self.assertEqual(len(DOCKING_RULES), 7)


class TestKonsistensiKontinuitas(unittest.TestCase):
    """Gain tidak boleh melompat gila saat input bergeser sedikit."""

    def test_gate_monoton_error(self):
        # Sweep error di zona 5..300 (hindari tepi semesta: error=0 & 320
        # memberi gain 0 karena trapmf menutup di ujung — itu perilaku desain,
        # bukan pelanggaran monoton di zona aktif).
        prev = gate_p_gain(0.5, 5.0)
        for error in range(10, 301, 5):
            cur = gate_p_gain(0.5, error)
            # gain naik seiring error membesar (dari 1.0 menuju 2.5)
            self.assertLessEqual(prev, cur + 1e-9)
            prev = cur

    def test_random_di_semesta_terhingga(self):
        rng = random.Random(1234)
        for _ in range(400):
            # jarak dijauhkan dari tepi semesta (gate 0..1, docking 0.05..1):
            # tepat di tepi semua trapmf menutup -> gain 0 (perilaku desain).
            jarak_gate = rng.uniform(0.02, 0.98)
            jarak_dock = rng.uniform(0.06, 0.98)
            error = rng.uniform(1.0, 319.0)
            g = gate_p_gain(jarak_gate, error)
            self.assertTrue(1.0 - 1e-9 <= g <= 2.5 + 1e-9, f"gate {g=}")
            d = docking_p_gain(jarak_dock, error)
            self.assertTrue(0.5 - 1e-9 <= d <= 1.9 + 1e-9, f"dock {d=}")


if __name__ == "__main__":
    unittest.main()