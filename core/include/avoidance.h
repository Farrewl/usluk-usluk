/* ============================================================
 * core/include/avoidance.h — hindar-rintangan reaktif (bias yaw).
 *
 * Kapal melaju AUTONOMOUS: YOLO memberi kotak deteksi tiap frame;
 * objek BESAR & DEKAT SUMBU FRAME adalah rintangan. Modul ini
 * menghitung bias yaw menjauh secara REAKTIF (tanpa peta/planner).
 *
 * Seluruh geometri di C (bukan Python) agar perilaku identik di
 * semua mesin + bisa diuji lintas bahasa (tests/test_core_avoidance.py).
 *
 * Aturan (mirror di tests/test_core_avoidance.py):
 *   - rintangan   = lebar >= 25% lebar frame (AVOID_W_MIN) DAN
 *                   |cx| <= 80% (AVOID_CX_LIMIT)
 *   - bias        = -kekuatan * cx_norm   (objek kanan -> belok kiri)
 *                   kekuatan = (w_norm - W_MIN)/(1 - W_MIN) di-clamp 0..1
 *   - memori      = bias ditahan AVOID_MEMORY_S detik setelah objek
 *                   hilang, lalu decay ke 0 (anti-kelelahan zigzag)
 * ============================================================ */
#ifndef ASV_AVOIDANCE_H
#define ASV_AVOIDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Lebar objek >= 25% lebar frame = rintangan berarti (dekat/besar). */
#define AVOID_W_MIN       0.25
/* Hanya objek dalam 80% kiri/kanan sumbu frame. */
#define AVOID_CX_LIMIT    0.80
/* Bias ditahan 0.8 s setelah rintangan hilang (decay anti-zigzag). */
#define AVOID_MEMORY_S    0.80
/* |bias| di atas ini dianggap aktif (pemakaian di simulator/GUI). */
#define AVOID_BIAS_ACTIVE 0.05

typedef struct {
    double yaw_bias;  /* [-1..1]: + belok kanan, - belok kiri (dari frame) */
    double memory_s;  /* sisa detik bias ditahan setelah rintangan hilang */
} avoid_state_t;

/* det_cx_norm: pusat objek dinormalisasi [-1..1]; 0 = tengah frame.
 * det_w_norm : lebar objek dinormalisasi [0..1]; 1 = selebar frame.
 * dt_s       : delta waktu frame (untuk decay memori), > 0.
 * Update state in-place. Tanpa rintangan (w_norm == 0) -> decay memori. */
void avoid_update(avoid_state_t *s, double det_cx_norm, double det_w_norm,
                  double dt_s);

#ifdef __cplusplus
}
#endif

#endif /* ASV_AVOIDANCE_H */