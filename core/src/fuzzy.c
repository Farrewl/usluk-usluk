/*
 * core/src/fuzzy.c — Controller fuzzy Sugeno singleton: gain P dinamis
 *
 * Port 1:1 dari app/fuzzy.py. Dua controller:
 *   - GATE    (buoy): 9 aturan, singleton 1.0 / 1.9 / 2.5, semesta jarak 0..2
 *   - DOCKING (box):  7 aturan, singleton 0.5 / 1.2 / 1.9, semesta jarak 0..10
 *
 * Struktur tabel sama persis dengan app/fuzzy.py supaya mudah diaudit:
 *   GATE_DIST MF  = GATE_DIST_MF,  GATE_ERR MF  = GATE_ERROR_MF,
 *   GATE_RULES    = GATE_RULES,    dst.
 */
#include "fuzzy.h"

/* ------------------------------------------------------------------------ */
/* Fungsi keanggotaan trapesium (identik dengan app/fuzzy._trapmf)          */
/* ------------------------------------------------------------------------ */

static double fuzzy_trapmf(double x, double a, double b, double c, double d) {
    if (x <= a || x >= d) {
        return 0.0;
    }
    if (x < b) {
        return (b > a) ? (x - a) / (b - a) : 1.0; /* aman b==a (bahu kiri) */
    }
    if (x <= c) {
        return 1.0;
    }
    return (d > c) ? (d - x) / (d - c) : 1.0;     /* aman d==c (bahu kanan) */
}

/* ------------------------------------------------------------------------ */
/* Tabel semesta & aturan                                                    */
/* ------------------------------------------------------------------------ */

typedef enum {
    D_DEKAT = 0,
    D_SEDANG,
    D_JAUH
} dist_term_t;

typedef enum {
    E_KECIL = 0,
    E_SEDANG,
    E_BESAR
} err_term_t;

typedef struct {
    dist_term_t dist;    /* term jarak */
    int         has_err; /* 1 bila aturan juga memakai error (AND) */
    err_term_t  err;     /* term error (abaikan bila has_err == 0) */
    double      singleton;
} sugeno_rule_t;

/* -- GATE (buoy) : GATE_DIST_MF / GATE_ERROR_MF / GATE_RULES -- */
static const double GATE_DIST_MF[3][4] = {
    {0.0, 0.0, 0.2, 0.4},     /* DEKAT  */
    {0.3, 0.5, 0.7, 0.8},     /* SEDANG */
    {0.7, 0.8, 1.0, 1.0},     /* JAUH   */
};
static const double GATE_ERROR_MF[3][4] = {
    {0.0, 0.0, 20.0, 40.0},       /* KECIL  */
    {30.0, 60.0, 100.0, 130.0},   /* SEDANG */
    {110.0, 150.0, 320.0, 320.0}, /* BESAR  */
};
static const sugeno_rule_t GATE_RULES[9] = {
    {D_DEKAT, 1, E_KECIL,  1.0},
    {D_DEKAT, 1, E_SEDANG, 1.9},
    {D_DEKAT, 1, E_BESAR,  2.5},
    {D_SEDANG, 1, E_KECIL, 1.0},
    {D_SEDANG, 1, E_SEDANG, 1.9},
    {D_SEDANG, 1, E_BESAR,  2.5},
    {D_JAUH, 1, E_KECIL,  1.9},
    {D_JAUH, 1, E_SEDANG, 2.5},
    {D_JAUH, 1, E_BESAR,  2.5},
};

/* -- DOCKING (box merah) : DOCKING_DIST_MF / DOCKING_ERROR_MF / rules -- */
static const double DOCKING_DIST_MF[3][4] = {
    {0.05, 0.08, 0.12, 0.15}, /* DEKAT  */
    {0.13, 0.18, 0.25, 0.35}, /* SEDANG */
    {0.3,  0.4,  1.0,  1.0},  /* JAUH   */
};
static const double DOCKING_ERROR_MF[3][4] = {
    {0.0, 0.0, 15.0, 30.0},      /* KECIL  */
    {25.0, 50.0, 80.0, 100.0},   /* SEDANG */
    {90.0, 120.0, 320.0, 320.0}, /* BESAR  */
};
/* Aturan 1 hanya memakai jarak (has_err=0), persis rule asli navigator:
 *   ctrl.Rule(jarak['DEKAT'], p_gain['RENDAH'])                      */
static const sugeno_rule_t DOCKING_RULES[7] = {
    {D_DEKAT, 0, E_KECIL,  0.5},
    {D_SEDANG, 1, E_KECIL, 0.5},
    {D_SEDANG, 1, E_SEDANG, 1.2},
    {D_SEDANG, 1, E_BESAR,  1.9},
    {D_JAUH, 1, E_KECIL,  1.2},
    {D_JAUH, 1, E_SEDANG, 1.9},
    {D_JAUH, 1, E_BESAR,  1.9},
};

/* ------------------------------------------------------------------------ */
/* Evaluasi Sugeno order-0 (identik dengan app/fuzzy._sugeno_singleton)     */
/* ------------------------------------------------------------------------ */

static double sugeno_eval(double dist, double error,
                          const double dist_mf[3][4],
                          const double err_mf[3][4],
                          const sugeno_rule_t *rules, int n_rules) {
    double total_alpha = 0.0;
    double total_out = 0.0;
    int i;

    for (i = 0; i < n_rules; i++) {
        const sugeno_rule_t *r = &rules[i];
        double alpha = fuzzy_trapmf(dist,
                                    dist_mf[r->dist][0], dist_mf[r->dist][1],
                                    dist_mf[r->dist][2], dist_mf[r->dist][3]);
        if (r->has_err) {
            const double alpha_err = fuzzy_trapmf(
                error,
                err_mf[r->err][0], err_mf[r->err][1],
                err_mf[r->err][2], err_mf[r->err][3]);
            alpha = (alpha_err < alpha) ? alpha_err : alpha; /* AND = min */
        }
        total_alpha += alpha;
        total_out += alpha * r->singleton;
    }
    if (total_alpha <= 0.0) {
        return 0.0;
    }
    return total_out / total_alpha;
}

/* ------------------------------------------------------------------------ */
/* API publik                                                                */
/* ------------------------------------------------------------------------ */

double fuzzy_gate_p_gain(double jarak_m, double error_px) {
    return sugeno_eval(jarak_m, error_px, GATE_DIST_MF, GATE_ERROR_MF,
                       GATE_RULES,
                       (int)(sizeof(GATE_RULES) / sizeof(GATE_RULES[0])));
}

double fuzzy_docking_p_gain(double jarak_m, double error_px) {
    return sugeno_eval(jarak_m, error_px, DOCKING_DIST_MF, DOCKING_ERROR_MF,
                       DOCKING_RULES,
                       (int)(sizeof(DOCKING_RULES) / sizeof(DOCKING_RULES[0])));
}