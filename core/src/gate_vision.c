/*
 * core/src/gate_vision.c — Sequencer gate + validasi buoy (C).
 *
 * Port 1:1 dari app/gate_sequencer.py + kriteria geometri
 * app/detection_validation.py. Tanpa malloc (array statis) agar aman
 * di loop 30 Hz Raspberry Pi. Warna (HSV) tetap di Python karena butuh
 * akses citra OpenCV — modul ini hanya geometri + latch + memori.
 */
#include "gate_vision.h"

#include <math.h>

int gv_collect_pairs(const gv_ball_t *red, int n_red,
                     const gv_ball_t *green, int n_green,
                     double vertical_align_px, double area_similarity_ratio,
                     gv_pair_t *out) {
    int n = 0;
    int i, j;
    if (!red || !green || !out) {
        return 0;
    }
    if (n_red > GV_MAX_DET) {
        n_red = GV_MAX_DET;
    }
    if (n_green > GV_MAX_DET) {
        n_green = GV_MAX_DET;
    }
    for (i = 0; i < n_red; i++) {
        for (j = 0; j < n_green; j++) {
            double dy = red[i].cy - green[j].cy;
            double ar = red[i].area;
            double ag = green[j].area;
            double bigger;
            if (dy < 0.0) {
                dy = -dy;
            }
            if (dy > vertical_align_px) {
                continue;
            }
            if (ar <= 0.0 || ag <= 0.0) {
                continue;
            }
            bigger = ar > ag ? ar : ag;
            if ((ar < ag ? ar : ag) / bigger < area_similarity_ratio) {
                continue;
            }
            if (n < GV_MAX_PAIRS) {
                out[n].red_idx = i;
                out[n].green_idx = j;
                n++;
            }
        }
    }
    return n;
}

double gv_estimate_distance(double pixel_width, double gate_width_m,
                            double focal_length_px) {
    if (pixel_width <= 5.0 || focal_length_px <= 0.0 || gate_width_m <= 0.0) {
        return HUGE_VAL;
    }
    return (gate_width_m * focal_length_px) / pixel_width;
}

int gv_buoy_fail(double w, double h, double min_area,
                 double max_aspect_deviation) {
    double area;
    double aspect;
    /* Kode alasan (0 = lolos) supaya lapisan Python bisa memberi pesan
     * debug tanpa menduplikasi rumus geometri:
     *   1 = kotak terlalu kecil (w <= 2 atau h <= 2 piksel),
     *   2 = luas < min_area,
     *   3 = rasio aspek menyimpang dari 1:1 (bukan bola/bujur sangkar).
     */
    if (w <= 2.0 || h <= 2.0) {
        return 1;
    }
    area = w * h;
    if (area < min_area) {
        return 2;
    }
    aspect = (h > 0.0) ? (w / h) : 0.0;
    if (fabs(aspect - 1.0) > max_aspect_deviation) {
        return 3;
    }
    return 0;
}

int gv_buoy_ok(double w, double h, double min_area,
               double max_aspect_deviation) {
    return gv_buoy_fail(w, h, min_area, max_aspect_deviation) == 0;
}

/* Koreksi yaw mentah (rad) dari error titik tengah piksel.

 * Memindahkan rumus yang sebelumnya diduplikasi 4x di Python
 * (_calculate_yaw_correction_gate/box/blue/red):
 *   error_m = (error_px * distance_m) / focal_length_px
 *   raw     = atan2(error_m, distance_m)
 * Aturan batas: distance_m < 0.1 ATAU focal <= 0 -> 0.0 (tak reliabel).
 */
double gv_yaw_correction(double target_x_px, double image_center_x,
                         double distance_m, double focal_length_px) {
    double error_px;
    double error_m;
    if (distance_m < 0.1 || focal_length_px <= 0.0) {
        return 0.0;
    }
    error_px = target_x_px - image_center_x;
    error_m = (error_px * distance_m) / focal_length_px;
    return atan2(error_m, distance_m);
}

void gv_seq_init(gv_seq_t *s, double pass_distance_m,
                 int lost_tolerance_frames, double gate_width_m,
                 double focal_length_px, double midpoint_match_px,
                 double track_match_px, double track_boost) {
    if (!s) {
        return;
    }
    s->pass_distance_m = pass_distance_m;
    s->lost_tolerance_frames = lost_tolerance_frames;
    s->gate_width_m = gate_width_m;
    s->focal_length_px = focal_length_px;
    s->midpoint_match_px = midpoint_match_px;
    s->track_match_px = track_match_px;
    s->track_boost = track_boost;
    gv_seq_reset(s);
}

void gv_seq_reset(gv_seq_t *s) {
    if (!s) {
        return;
    }
    s->has_active = 0;
    s->active_mid_x = 0.0;
    s->active_mid_y = 0.0;
    s->active_dist = HUGE_VAL;
    s->active_red = -1;
    s->active_green = -1;
    s->lost_frames = 0;
    s->is_passed = 0;
    s->mem_n[0] = 0;
    s->mem_n[1] = 0;
}

/* --- helper internal --- */

static double gv_mid_x(const gv_ball_t *r, const gv_ball_t *g) {
    return (r->cx + g->cx) / 2.0;
}

static double gv_mid_y(const gv_ball_t *r, const gv_ball_t *g) {
    return (r->cy + g->cy) / 2.0;
}

/* Perbarui memori satu class dari posisi terlihat frame ini. */
static void gv_mem_update(gv_seq_t *s, int cls, const gv_ball_t *seen,
                          int n_seen) {
    int i, k;
    double tol2;
    if (cls < 0 || cls > 1) {
        return;
    }
    tol2 = s->track_match_px * s->track_match_px;
    for (i = 0; i < s->mem_n[cls]; i++) {
        s->mem[cls][i].unseen++;
    }
    for (k = 0; k < n_seen; k++) {
        int matched = 0;
        for (i = 0; i < s->mem_n[cls]; i++) {
            double dx = seen[k].cx - s->mem[cls][i].cx;
            double dy = seen[k].cy - s->mem[cls][i].cy;
            if (dx * dx + dy * dy <= tol2) {
                s->mem[cls][i].cx =
                    0.7 * s->mem[cls][i].cx + 0.3 * seen[k].cx;
                s->mem[cls][i].cy =
                    0.7 * s->mem[cls][i].cy + 0.3 * seen[k].cy;
                s->mem[cls][i].area = seen[k].area;
                s->mem[cls][i].unseen = 0;
                matched = 1;
                break;
            }
        }
        if (!matched && s->mem_n[cls] < GV_MEM_PER_CLS) {
            s->mem[cls][s->mem_n[cls]].cx = seen[k].cx;
            s->mem[cls][s->mem_n[cls]].cy = seen[k].cy;
            s->mem[cls][s->mem_n[cls]].area = seen[k].area;
            s->mem[cls][s->mem_n[cls]].unseen = 0;
            s->mem_n[cls]++;
        }
    }
    /* Hapus expired (geser padat). */
    {
        int w = 0;
        for (i = 0; i < s->mem_n[cls]; i++) {
            if (s->mem[cls][i].unseen < s->lost_tolerance_frames) {
                if (w != i) {
                    s->mem[cls][w] = s->mem[cls][i];
                }
                w++;
            }
        }
        s->mem_n[cls] = w;
    }
}

/* Boost memori untuk satu buoy: track_boost bila cocok memori. */
static double gv_boost(const gv_seq_t *s, int cls, double cx, double cy) {
    int i;
    double tol2 = s->track_match_px * s->track_match_px;
    if (cls < 0 || cls > 1) {
        return 1.0;
    }
    for (i = 0; i < s->mem_n[cls]; i++) {
        double dx = cx - s->mem[cls][i].cx;
        double dy = cy - s->mem[cls][i].cy;
        if (dx * dx + dy * dy <= tol2) {
            return s->track_boost;
        }
    }
    return 1.0;
}

int gv_seq_update(gv_seq_t *s, const gv_ball_t *red, int n_red,
                  const gv_ball_t *green, int n_green,
                  const gv_pair_t *pairs, int n_pairs,
                  double image_center_y, double *mid_x, double *mid_y,
                  double *dist, int *is_passed) {
    /* Tabel target terurut: index pasangan + jarak + skor boost. */
    int order[GV_MAX_PAIRS];
    double dists[GV_MAX_PAIRS];
    int i, j;
    int best = -1;

    if (!s || !pairs) {
        return 0;
    }
    if (n_pairs > GV_MAX_PAIRS) {
        n_pairs = GV_MAX_PAIRS;
    }

    if (n_pairs == 0) {
        /* Tahan target lama selama toleransi hilang. */
        if (s->has_active && s->lost_frames < s->lost_tolerance_frames) {
            s->lost_frames++;
            if (mid_x) {
                *mid_x = s->active_mid_x;
            }
            if (mid_y) {
                *mid_y = s->active_mid_y;
            }
            if (dist) {
                *dist = s->active_dist;
            }
            if (is_passed) {
                *is_passed = s->is_passed;
            }
            return 1;
        }
        s->has_active = 0;
        s->active_dist = HUGE_VAL;
        return 0;
    }

    s->lost_frames = 0;

    /* Update memori dari buoy yang terlihat (red=cls 1, green=cls 0). */
    gv_mem_update(s, 1, red, n_red > GV_MAX_DET ? GV_MAX_DET : n_red);
    gv_mem_update(s, 0, green, n_green > GV_MAX_DET ? GV_MAX_DET : n_green);

    /* Hitung jarak tiap pasangan + urutan boost (selection sort). */
    for (i = 0; i < n_pairs; i++) {
        const gv_ball_t *r = &red[pairs[i].red_idx];
        const gv_ball_t *g = &green[pairs[i].green_idx];
        double pw = r->cx - g->cx;
        double b;
        if (pw < 0.0) {
            pw = -pw;
        }
        dists[i] = gv_estimate_distance(pw, s->gate_width_m,
                                        s->focal_length_px);
        b = gv_boost(s, 1, r->cx, r->cy);
        {
            double b2 = gv_boost(s, 0, g->cx, g->cy);
            if (b2 > b) {
                b = b2;
            }
        }
        order[i] = i;
        /* simpan skor sementara di dists? tidak — skor dihitung saat
         * sort di bawah via boost ulang (n kecil, murah). */
        (void)b;
    }
    /* Sort order by (dist/boost) menaik. */
    for (i = 0; i < n_pairs; i++) {
        int bi = i;
        for (j = i + 1; j < n_pairs; j++) {
            const gv_ball_t *rr = &red[pairs[order[j]].red_idx];
            const gv_ball_t *gg = &green[pairs[order[j]].green_idx];
            const gv_ball_t *br = &red[pairs[order[bi]].red_idx];
            const gv_ball_t *bg = &green[pairs[order[bi]].green_idx];
            double bj = gv_boost(s, 1, rr->cx, rr->cy);
            double b2j = gv_boost(s, 0, gg->cx, gg->cy);
            double bb = gv_boost(s, 1, br->cx, br->cy);
            double b2b = gv_boost(s, 0, bg->cx, bg->cy);
            double ej, eb;
            if (b2j > bj) {
                bj = b2j;
            }
            if (b2b > bb) {
                bb = b2b;
            }
            ej = bj > 1.0 ? dists[order[j]] / bj : dists[order[j]];
            eb = bb > 1.0 ? dists[order[bi]] / bb : dists[order[bi]];
            if (ej < eb) {
                bi = j;
            }
        }
        if (bi != i) {
            int t = order[i];
            order[i] = order[bi];
            order[bi] = t;
        }
    }

    /* Latch: target aktif masih terlihat & belum lewat -> pertahankan. */
    if (s->has_active && !s->is_passed) {
        for (i = 0; i < n_pairs; i++) {
            const gv_ball_t *r = &red[pairs[i].red_idx];
            const gv_ball_t *g = &green[pairs[i].green_idx];
            double mx = gv_mid_x(r, g);
            double my = gv_mid_y(r, g);
            double dx = mx - s->active_mid_x;
            double dy = my - s->active_mid_y;
            if (dx < 0.0) {
                dx = -dx;
            }
            if (dy < 0.0) {
                dy = -dy;
            }
            if (dx <= s->midpoint_match_px && dy <= s->midpoint_match_px) {
                s->active_mid_x = mx;
                s->active_mid_y = my;
                s->active_dist = dists[i];
                s->is_passed = (dists[i] < s->pass_distance_m ||
                                my > image_center_y);
                if (mid_x) {
                    *mid_x = mx;
                }
                if (mid_y) {
                    *mid_y = my;
                }
                if (dist) {
                    *dist = dists[i];
                }
                if (is_passed) {
                    *is_passed = s->is_passed;
                }
                return 1;
            }
        }
    }

    /* Pilih target depan terdekat (urutan boost). */
    for (i = 0; i < n_pairs; i++) {
        int pi = order[i];
        const gv_ball_t *r = &red[pairs[pi].red_idx];
        const gv_ball_t *g = &green[pairs[pi].green_idx];
        double mx = gv_mid_x(r, g);
        double my = gv_mid_y(r, g);
        if (dists[pi] < s->pass_distance_m || my > image_center_y) {
            continue;
        }
        s->has_active = 1;
        s->active_mid_x = mx;
        s->active_mid_y = my;
        s->active_dist = dists[pi];
        s->active_red = pairs[pi].red_idx;
        s->active_green = pairs[pi].green_idx;
        s->is_passed = 0;
        if (mid_x) {
            *mid_x = mx;
        }
        if (mid_y) {
            *mid_y = my;
        }
        if (dist) {
            *dist = dists[pi];
        }
        if (is_passed) {
            *is_passed = 0;
        }
        return 1;
    }

    /* Semua di belakang -> tandai lewat (kembalikan terdekat). */
    best = order[0];
    {
        const gv_ball_t *r = &red[pairs[best].red_idx];
        const gv_ball_t *g = &green[pairs[best].green_idx];
        s->has_active = 1;
        s->active_mid_x = gv_mid_x(r, g);
        s->active_mid_y = gv_mid_y(r, g);
        s->active_dist = dists[best];
        s->is_passed = 1;
        if (mid_x) {
            *mid_x = s->active_mid_x;
        }
        if (mid_y) {
            *mid_y = s->active_mid_y;
        }
        if (dist) {
            *dist = s->active_dist;
        }
        if (is_passed) {
            *is_passed = 1;
        }
        return 1;
    }
}
