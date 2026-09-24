/*
 * PID + deadband + anti-windup, filter komplementer, EKF heading 1-D.
 * Port 1:1 app/filtering.py — urutan operasi IDENTIK (cross-check bit-per-bit
 * di tests/test_core_control_filters.py). Wrap & CTE pinjam nav_math.h.
 */
#ifndef ASV_CONTROL_FILTERS_H
#define ASV_CONTROL_FILTERS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Konteks PID. Caller inisialisasi nol lalu atur gain/batas & cf_pid_reset. */
typedef struct {
    double kp, ki, kd;          /* gain proporsional, integral, derivatif */
    double deadband;            /* zona mati error masukan (radian) */
    double output_limit;        /* clamp output (radian); 0 = tanpa clamp */
    double integral_limit;      /* clamp anti-windup; 0 = tanpa clamp */
    double integral;            /* akumulator integral (radian*detik) */
    double last_error;          /* error frame sebelumnya (radian) */
} cf_pid_t;

/* Reset state integral & last_error (awal misi / leg baru). */
void cf_pid_reset(cf_pid_t *p);
/* Hitung koreksi PID untuk `error` pada selang `dt` detik. */
double cf_pid_update(cf_pid_t *p, double error, double dt);

/* Fusi heading komplementer:
 *   angle = alpha*(angle_prev + gyro_rate*dt) + (1-alpha)*angle_measured */
double cf_complementary(double alpha, double angle_prev,
                        double gyro_rate, double dt, double angle_measured);

/* Konteks EKF 1-D heading: state [heading, bias], kovarians 2x2 simetris
 * (p00, p01=p10, p11). */
typedef struct {
    double q_heading, q_bias;   /* noise proses (rad^2, rad^2/s) */
    double r_meas;              /* noise pengukuran (rad^2) */
    double heading, bias;       /* state */
    double p00, p01, p11;       /* kovarians (p01 = p10) */
} cf_ekf_t;

/* Inisialisasi EKF: noise proses/ukur + initial heading. */
void cf_ekf_init(cf_ekf_t *e, double process_noise, double meas_noise,
                 double init_heading);
/* Prediksi (bawa state maju) dengan laju gyro (rad/s) selama dt detik. */
double cf_ekf_predict(cf_ekf_t *e, double gyro_rate, double dt);
/* Koreksi dengan pengukuran heading; inovasi di-wrap-around di dalam. */
double cf_ekf_update(cf_ekf_t *e, double measured_heading);
/* Heading estimasi ternormalisasi (radian). */
double cf_ekf_heading(const cf_ekf_t *e);

#ifdef __cplusplus
}
#endif

#endif /* ASV_CONTROL_FILTERS_H */