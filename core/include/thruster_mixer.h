/*
 * Mixer differential drive: surge [-1..1] + yaw [-1..1] -> PWM 1100..1900 us.
 *   pwm_kiri  = 1500 + surge*400 - yaw*400
 *   pwm_kanan = 1500 + surge*400 + yaw*400
 */
#ifndef ASV_THRUSTER_MIXER_H
#define ASV_THRUSTER_MIXER_H

#ifdef __cplusplus
extern "C" {
#endif

/* PWM µs untuk stop (netral ESC) */
#define MIXER_CENTER_PWM 1500

/* Rentang PWM minimum & maksimum */
#define MIXER_MIN_PWM 1100
#define MIXER_MAX_PWM 1900

/* Batas input: nilai |surge| atau |yaw| > 1.0 akan di-clamp */
#define MIXER_INPUT_LIMIT 1.0

/* surge [-1 mundur .. +1 maju], yaw [-1 kiri .. +1 kanan]. Output via pointer. */
void mixer_diff_drive(double surge, double yaw,
                      int *pwm_left, int *pwm_right);

/* Clamp nilai ke [lo..hi]. */
double mixer_clamp(double val, double lo, double hi);

#ifdef __cplusplus
}
#endif

#endif /* ASV_THRUSTER_MIXER_H */
