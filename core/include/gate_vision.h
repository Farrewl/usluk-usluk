/*
 * core/include/gate_vision.h — Sequencer gate + validasi buoy (C).
 *
 * Port 1:1 dari app/gate_sequencer.py (koleksi pasangan, latch target,
 * memori tracking) + kriteria geometri app/detection_validation.py
 * (area, aspek, bukan warna — warna butuh citra, tetap di Python).
 *
 * Kontrak:
 *   - gv_collect_pairs: semua pasangan merah-hijau yang sejajar-Y &
 *     mirip luas. Return jumlah pasangan (tulis ke out, maks GV_MAX_PAIRS).
 *   - gv_estimate_distance: pinhole (lebar_m * focal_px) / pixel_lebar;
 *     pixel <= 5 / focal <= 0 / lebar <= 0 -> +inf.
 *   - gv_buoy_ok: area >= min_area, w/h > 2 px, |w/h-1| <= aspek.
 *   - gv_seq_*: latch target aktif + toleransi hilang + lompat saat lewat.
 */
#ifndef ASV_GATE_VISION_H
#define ASV_GATE_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

/* Batas kompilasi (tanpa malloc): cukup untuk frame penuh deteksi. */
#define GV_MAX_DET 64
#define GV_MAX_PAIRS 64
#define GV_MEM_PER_CLS 32

/* Satu kotak deteksi (koordinat piksel + luas). */
typedef struct {
    double cx;
    double cy;
    double area;
} gv_ball_t;

/* Satu pasangan gate (index ke array merah & hijau caller). */
typedef struct {
    int red_idx;
    int green_idx;
} gv_pair_t;

/* Memori tracking buoy per class (port _buoy_memory Python). */
typedef struct {
    double cx;
    double cy;
    double area;
    int unseen; /* frame sejak terakhir terlihat */
} gv_mem_t;

/* State sequencer (port GateSequencer Python). */
typedef struct {
    double pass_distance_m;
    int lost_tolerance_frames;
    double gate_width_m;
    double focal_length_px;
    double midpoint_match_px;
    double track_match_px;
    double track_boost;
    /* target aktif */
    int has_active;
    double active_mid_x;
    double active_mid_y;
    double active_dist;
    int active_red;
    int active_green;
    int lost_frames;
    int is_passed;
    /* memori per class: 0 = hijau, 1 = merah */
    gv_mem_t mem[2][GV_MEM_PER_CLS];
    int mem_n[2];
} gv_seq_t;

/* Kumpulkan pasangan plausible; return jumlah (0..GV_MAX_PAIRS). */
int gv_collect_pairs(const gv_ball_t *red, int n_red,
                     const gv_ball_t *green, int n_green,
                     double vertical_align_px, double area_similarity_ratio,
                     gv_pair_t *out);

/* Jarak gate (m) dari lebar piksel; +inf bila tak reliabel. */
double gv_estimate_distance(double pixel_width, double gate_width_m,
                            double focal_length_px);

/* Kriteria geometri buoy: 0 = tolak, 1 = lolos. */
int gv_buoy_ok(double w, double h, double min_area,
               double max_aspect_deviation);

/* Alasan penolakan geometri: 0 = lolos, 1 = terlalu kecil,
 * 2 = area < min_area, 3 = aspek menyimpang. */
int gv_buoy_fail(double w, double h, double min_area,
                 double max_aspect_deviation);

/* Inisialisasi sequencer (panggil sekali). */
void gv_seq_init(gv_seq_t *s, double pass_distance_m,
                 int lost_tolerance_frames, double gate_width_m,
                 double focal_length_px, double midpoint_match_px,
                 double track_match_px, double track_boost);

/* Reset target & memori (awal misi / ganti leg). */
void gv_seq_reset(gv_seq_t *s);

/* Perbarui antrean; return 1 bila ada target aktif (mid/dist/passed
 * ditulis ke pointer), 0 bila kosong. */
int gv_seq_update(gv_seq_t *s, const gv_ball_t *red, int n_red,
                  const gv_ball_t *green, int n_green,
                  const gv_pair_t *pairs, int n_pairs,
                  double image_center_y, double *mid_x, double *mid_y,
                  double *dist, int *is_passed);

#ifdef __cplusplus
}
#endif

#endif /* ASV_GATE_VISION_H */
