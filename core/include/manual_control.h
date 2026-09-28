/*
 * core/include/manual_control.h — Kurva stick manual + rate limiter (C murni)
 *
 * Semua "rasa" stick (deadband, expo, batas laju, batas gas) dihitung di
 * sini — Python (app/manual_link.py) hanya meneruskan angka mentah & param
 * dari app/settings.py. Tanpa alokasi, cocok untuk loop 20 Hz di NUC.
 */
#ifndef ASV_MANUAL_CONTROL_H
#define ASV_MANUAL_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Memori rate-limiter (diisi caller, di-nol-kan via manual_reset). */
typedef struct {
    double prev_surge;  /* output surge frame lalu */
    double prev_yaw;    /* output yaw frame lalu */
} manual_state_t;

/* Setpoint manual hasil olahan stick. */
typedef struct {
    double surge;   /* -1..1 (sudah dibatasi MANUAL_MAX_SURGE) */
    double yaw;     /* -1..1 (sudah dibatasi MANUAL_MAX_YAW) */
    int    active;  /* 1 = deadman ditahan & RC sehat & enabled */
} manual_out_t;

/* Nol-kan memori rate-limiter (awal mode manual / setelah KILL). */
void manual_reset(manual_state_t *s);

/*
 * Kurva stick: deadband (zona mati) + rescale + expo.
 * x [-1..1] -> [-1..1]. |x| < deadband -> 0. Expo 0 = linear,
 * expo 1 = kubik penuh (halus di tengah, galak di ujung).
 */
double manual_curve(double x, double deadband, double expo);

/* Batasi perubahan: |target - prev| <= max_step (rate_limit*dt). */
double manual_rate_limit(double target, double prev, double max_step);

/*
 * Olah stick mentah menjadi setpoint aman.
 * in_surge/in_yaw: [-1..1] dari rc_decode (sudah ternormalisasi).
 * enabled: MANUAL_ENABLED; deadman/rc_ok: dari rc_decode.
 * deadband/expo/rate_limit/max_surge/max_yaw: dari MANUAL_* settings.
 * dt: detik sejak frame lalu (<=0 = pakai 1e-4, anti-ledak).
 * Bila tidak aktif: output 0,0 + memori di-nol-kan (lepas stick = diam).
 */
void manual_update(manual_state_t *s,
                   double in_surge, double in_yaw,
                   int enabled, int deadman, int rc_ok,
                   double deadband, double expo, double rate_limit,
                   double dt, double max_surge, double max_yaw,
                   manual_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ASV_MANUAL_CONTROL_H */
