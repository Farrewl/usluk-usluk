/*
 * core/include/fuzzy.h — Controller fuzzy Sugeno singleton: gain P dinamis
 *
 * Port 1:1 dari app/fuzzy.py (implementasi murni Python yang menggantikan
 * scikit-fuzzy — lihat catatan di app/fuzzy.py). Semantika:
 *   - Keanggotaan  : trapmf
 *   - Anteseden AND: min
 *   - Output       : Sugeno order-0 (singleton) -> rata-rata tertimbang
 *                    out = sum(alpha_i * value_i) / sum(alpha_i)
 *   - Bila tidak ada aturan menyala (semesta = 0) -> 0.0
 *
 * Pemakaian C:  #include "fuzzy.h"
 * Uji:          gcc -I core/include core/tests/test_fuzzy.c core/src/fuzzy.c
 *               -lm -o /tmp/opencode/test_fuzzy && /tmp/opencode/test_fuzzy
 */
#ifndef ASV_FUZZY_H
#define ASV_FUZZY_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * fuzzy_gate_p_gain — gain P dinamis untuk koreksi GATE (buoy).
 * @param jarak_m  jarak kapal ke gerbang (meter), semesta khas 0..2
 * @param error_px |offset target dari tengah kamera| (piksel), semesta 0..320
 * @return gain 0.0..2.5; 0.0 bila input di luar semesta (tidak ada aturan).
 */
double fuzzy_gate_p_gain(double jarak_m, double error_px);

/**
 * fuzzy_docking_p_gain — gain P dinamis untuk koreksi DOCKING (box merah).
 * @param jarak_m  jarak kapal ke box docking (meter), semesta khas 0..10
 * @param error_px |offset target dari tengah kamera| (piksel), semesta 0..320
 * @return gain 0.0..1.9; 0.0 bila input di luar semesta.
 */
double fuzzy_docking_p_gain(double jarak_m, double error_px);

#ifdef __cplusplus
}
#endif

#endif /* ASV_FUZZY_H */