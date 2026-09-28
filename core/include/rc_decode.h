/*
 * core/include/rc_decode.h — Decode channel RC PWM menjadi [-1..1] (C murni)
 *
 * Jalur: Radiolink TX ~~> RX --> Pixhawk RCIN --> MAVLink RC_CHANNELS --> NUC.
 * Semua ambang sebagai #define (tanpa magic number di .c). Channel yang
 * dipakai (throttle/yaw/mode/deadman) dipilih caller (1-based, dari
 * RC_CH_* di app/settings.py) supaya mapping TX bisa diubah via GUI.
 */
#ifndef ASV_RC_DECODE_H
#define ASV_RC_DECODE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Rentang PWM standar receiver (mikrodetik). */
#define RC_PWM_MIN_US    1000
#define RC_PWM_CENTER_US 1500
#define RC_PWM_MAX_US    2000
/* Di luar ini = kabel putus / frame rusak -> invalid (failsafe). */
#define RC_PWM_VALID_LO_US 900
#define RC_PWM_VALID_HI_US 2100
/* Setengah rentang: normalisasi (pwm-1500)/500 -> [-1..1]. */
#define RC_PWM_SPAN_US 500
/* Kapasitas array channel (MAVLink RC_CHANNELS kirim s.d. 18; 16 cukup). */
#define RC_MAX_CHANNELS 16
/* Ambang switch: channel mode/deadman > ini = AKTIF. */
#define RC_MODE_THRESHOLD_US    1500
#define RC_DEADMAN_THRESHOLD_US 1500

/* Hasil decode satu frame RC. */
typedef struct {
    double surge;       /* -1..1 (maju) dari channel throttle */
    double yaw;         /* -1..1 (kanan) dari channel yaw */
    int    mode_manual; /* 1 = switch mode minta MANUAL */
    int    deadman;     /* 1 = tombol deadman ditahan */
    int    rc_ok;       /* 1 = semua channel wajib valid */
} rc_decoded_t;

/* 1 bila pwm_us di dalam [VALID_LO..VALID_HI], 0 bila failsafe/rusak. */
int rc_pwm_valid(int pwm_us);

/* PWM valid -> [-1..1] ((pwm-1500)/500, di-clamp). Invalid -> 0.0. */
double rc_pwm_to_norm(int pwm_us);

/*
 * Decode satu frame RC_CHANNELS.
 * ch_us: array PWM mentah (µs), n_ch: isinya (<= RC_MAX_CHANNELS).
 * ch_throttle/ch_yaw/ch_mode/ch_deadman: nomor channel 1-based.
 * Nomor di luar 1..n_ch -> rc_ok=0 (konfigurasi salah = failsafe).
 */
void rc_decode(const int *ch_us, int n_ch,
               int ch_throttle, int ch_yaw,
               int ch_mode, int ch_deadman,
               rc_decoded_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ASV_RC_DECODE_H */
