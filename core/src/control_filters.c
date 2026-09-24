/*
 * Port 1:1 app/filtering.py — urutan operasi IDENTIK. C murni, tanpa alokasi.
 * Dependensi: nav_math.h (nav_normalize_angle).
 */

#include "control_filters.h"
#include "nav_math.h"

/* dt <= 0 -> pakai 1e-4 dtk agar kd/dt tidak meledak (= _MIN_DT Python). */
#define CF_MIN_DT 1e-4

/* ---------------------------------------------------------------------------
 * PID + deadband
 * ------------------------------------------------------------------------- */

void cf_pid_reset(cf_pid_t *p) {
    p->integral = 0.0;
    p->last_error = 0.0;
}

double cf_pid_update(cf_pid_t *p, double error, double dt) {
    double derivative, output, limit;

    if (dt <= 0.0) {
        dt = CF_MIN_DT;
    }

    /* Zona mati: |error| kecil -> tanpa koreksi & tanpa akumulasi. */
    if (error < p->deadband && error > -p->deadband) {
        p->last_error = error;
        return 0.0;
    }

    p->integral += error * dt;
    if (p->integral_limit > 0.0) {
        limit = p->integral_limit;
        if (p->integral > limit) p->integral = limit;
        if (p->integral < -limit) p->integral = -limit;
    }

    derivative = (error - p->last_error) / dt;
    p->last_error = error;

    output = p->kp * error + p->ki * p->integral + p->kd * derivative;

    if (p->output_limit > 0.0) {
        limit = p->output_limit;
        if (output > limit) output = limit;
        if (output < -limit) output = -limit;
    }
    return output;
}

/* ---------------------------------------------------------------------------
 * Complementary filter
 * ------------------------------------------------------------------------- */

double cf_complementary(double alpha, double angle_prev,
                        double gyro_rate, double dt, double angle_measured) {
    if (dt <= 0.0) {
        dt = CF_MIN_DT;
    }
    return alpha * (angle_prev + gyro_rate * dt)
           + (1.0 - alpha) * angle_measured;
}

/* ---------------------------------------------------------------------------
 * EKF 1-D heading
 * ------------------------------------------------------------------------- */

void cf_ekf_init(cf_ekf_t *e, double process_noise, double meas_noise,
                 double init_heading) {
    e->q_heading = process_noise;
    e->q_bias = process_noise * 0.1;   /* drift bias lebih lambat dari heading */
    e->r_meas = meas_noise;
    e->heading = init_heading;
    e->bias = 0.0;
    e->p00 = 1.0;
    e->p01 = 0.0;
    e->p11 = 1.0;
}

double cf_ekf_predict(cf_ekf_t *e, double gyro_rate, double dt) {
    double p00, p01, p11, heading, bias;

    if (dt <= 0.0) {
        dt = CF_MIN_DT;
    }
    heading = e->heading;
    bias = e->bias;
    heading += (gyro_rate - bias) * dt;

    /* P = F P F^T + Q,  F = [[1, -dt], [0, 1]] */
    p00 = e->p00 - 2.0 * dt * e->p01 + dt * dt * e->p11 + e->q_heading * dt;
    p01 = e->p01 - dt * e->p11;
    p11 = e->p11 + e->q_bias * dt;

    e->heading = heading;
    e->bias = bias;
    e->p00 = p00;
    e->p01 = p01;
    e->p11 = p11;
    return heading;
}

double cf_ekf_update(cf_ekf_t *e, double measured_heading) {
    double innovation, s, k0, k1, p00, p01, p11, heading, bias;

    innovation = nav_normalize_angle(measured_heading - e->heading);

    /* S = H P H^T + R, H = [1, 0] -> S = p00 + R. */
    s = e->p00 + e->r_meas;
    if (s <= 0.0) {
        s = CF_MIN_DT;
    }
    k0 = e->p00 / s;
    k1 = e->p01 / s;

    heading = e->heading + k0 * innovation;
    bias = e->bias + k1 * innovation;

    /* P = (I - K H) P,  H = [1, 0] */
    p00 = e->p00 * (1.0 - k0);
    p01 = e->p01 * (1.0 - k0);
    p11 = e->p11 - k1 * e->p01;

    e->heading = heading;
    e->bias = bias;
    e->p00 = p00;
    e->p01 = p01;
    e->p11 = p11;
    return heading;
}

double cf_ekf_heading(const cf_ekf_t *e) {
    return nav_normalize_angle(e->heading);
}