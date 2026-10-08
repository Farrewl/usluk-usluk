"""
app/fuzzy.py — Controller Sugeno singleton untuk gain P dinamis.

Menggantikan scikit-fuzzy yang SEJAK 0.5.0 GAGAL membuat singleton output:
    ValueError: New membership function RENDAH must be equivalent in
    length to the universe variable. Expected 31, got 1.
Akibatnya `_create_gate_controller`/`_create_docking_controller` di
navigator.py lama masuk jalur "FATAL" dan fuzzy TIDAK PERNAH aktif.

Di sini formulanya ditulis ulang murni (tanpa numpy, tanpa skfuzzy)
sesuai MAKSUD desain aslinya:

  - Keanggotaan  : trapmf (sama seperti fuzz.trapmf)
  - Anteseden AND: min
  - Output       : Sugeno order-0 (singleton) -> rata-rata tertimbang
                   out = sum(alpha_i * value_i) / sum(alpha_i)

Karena matematika murni + tabel konstanta, modul ini bisa di-port 1:1 ke C
(`core/src/fuzzy.c`) dan dibuktikan identik lewat ctypes
(`tests/test_core_fuzzy.py`).

Catatan semesta: jarak gate 0..2 m, docking 0..10 m, error 0..320 px.
Input di luar semesta -> semua keanggotaan 0 -> output 0.0 (tidak ada
aturan menyala). Pemanggil harus membatasi input seperti navigator asli.
"""

# ---------------------------------------------------------------------------
# Fungsi keanggotaan trapesium
# ---------------------------------------------------------------------------

def _trapmf(x, a, b, c, d):
    """Keanggotaan trapesium 0.0-1.0.

    a..b = kaki kiri (naik), b..c = plateau 1.0, c..d = kaki kanan (turun).
    Aman dari pembagian-nol bila b == a atau d == c (bahu trapesium).
    """
    if x <= a or x >= d:
        return 0.0
    if x < b:
        return (x - a) / (b - a) if b > a else 1.0
    if x <= c:
        return 1.0
    return (d - x) / (d - c) if d > c else 1.0


def _sugeno_singleton(dist, error, dist_mf, error_mf, rules):
    """Evaluasi Sugeno order-0: rata-rata tertimbang kekuatan aturan.

    `rules` = daftar (term_jarak, term_error|None, nilai_singleton).
    """
    total_alpha = 0.0
    total_out = 0.0
    for dist_term, error_term, singleton in rules:
        alpha = _trapmf(dist, *dist_mf[dist_term])
        if error_term is not None:
            alpha = min(alpha, _trapmf(error, *error_mf[error_term]))
        total_alpha += alpha
        total_out += alpha * singleton
    if total_alpha <= 0.0:
        return 0.0
    return total_out / total_alpha


# ---------------------------------------------------------------------------
# Tabel GATE (buoy) — dari _create_gate_controller di navigator.py
# ---------------------------------------------------------------------------

GATE_DIST_MF = {
    "DEKAT":  (0.0, 0.0, 0.2, 0.4),
    "SEDANG": (0.3, 0.5, 0.7, 0.8),
    "JAUH":   (0.7, 0.8, 1.0, 1.0),
}
GATE_ERROR_MF = {
    "KECIL":  (0.0, 0.0, 20.0, 40.0),
    "SEDANG": (30.0, 60.0, 100.0, 130.0),
    "BESAR":  (110.0, 150.0, 320.0, 320.0),
}
GATE_RULES = [
    ("DEKAT", "KECIL", 1.0),
    ("DEKAT", "SEDANG", 1.9),
    ("DEKAT", "BESAR", 2.5),
    ("SEDANG", "KECIL", 1.0),
    ("SEDANG", "SEDANG", 1.9),
    ("SEDANG", "BESAR", 2.5),
    ("JAUH", "KECIL", 1.9),
    ("JAUH", "SEDANG", 2.5),
    ("JAUH", "BESAR", 2.5),
]


def gate_p_gain(jarak_m, error_px):
    """Gain P dinamis untuk koreksi gate (buoy). Semesta: 0..2 m, 0..320 px."""
    return _sugeno_singleton(jarak_m, error_px,
                             GATE_DIST_MF, GATE_ERROR_MF, GATE_RULES)


# ---------------------------------------------------------------------------
# Tabel DOCKING (box merah) — dari _create_docking_controller
# ---------------------------------------------------------------------------

DOCKING_DIST_MF = {
    "DEKAT":  (0.05, 0.08, 0.12, 0.15),
    "SEDANG": (0.13, 0.18, 0.25, 0.35),
    "JAUH":   (0.3, 0.4, 1.0, 1.0),
}
DOCKING_ERROR_MF = {
    "KECIL":  (0.0, 0.0, 15.0, 30.0),
    "SEDANG": (25.0, 50.0, 80.0, 100.0),
    "BESAR":  (90.0, 120.0, 320.0, 320.0),
}
# aturan 1 hanya memakai jarak (error_term=None), persis rule asli:
#   ctrl.Rule(jarak['DEKAT'], p_gain['RENDAH'])
DOCKING_RULES = [
    ("DEKAT", None, 0.5),
    ("SEDANG", "KECIL", 0.5),
    ("SEDANG", "SEDANG", 1.2),
    ("SEDANG", "BESAR", 1.9),
    ("JAUH", "KECIL", 1.2),
    ("JAUH", "SEDANG", 1.9),
    ("JAUH", "BESAR", 1.9),
]


def docking_p_gain(jarak_m, error_px):
    """Gain P dinamis untuk koreksi docking. Semesta: 0..10 m, 0..320 px."""
    return _sugeno_singleton(jarak_m, error_px,
                             DOCKING_DIST_MF, DOCKING_ERROR_MF, DOCKING_RULES)