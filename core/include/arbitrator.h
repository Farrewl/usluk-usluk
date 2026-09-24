/*
 * core/include/arbitrator.h — Pilih sumber setpoint thrust (C murni)
 *
 * Prioritas (dari tertinggi):
 *   1. KILL (STM32/E-stop/software) -> output 0,0 + flag killed=1
 *   2. MANUAL (gamepad/RC aktif)    -> pakai manual_surge/manual_yaw
 *   3. AUTO (waypoint/vision)       -> pakai auto_surge/auto_yaw
 *
 * Semua input [-1.0 .. 1.0]. Output di-clamp ke [-1.0 .. 1.0].
 * Struct kecil, tanpa alokasi — cocok untuk loop 20 Hz di NUC.
 */

#ifndef ASV_ARBITRATOR_H
#define ASV_ARBITRATOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Mode kendali yang dipilih arbitrator (untuk log/GUI). */
#define ARB_SRC_KILLED 0
#define ARB_SRC_MANUAL 1
#define ARB_SRC_AUTO   2

typedef struct {
    double surge;   /* -1.0 .. 1.0 */
    double yaw;     /* -1.0 .. 1.0 */
    int    source;  /* ARB_SRC_* */
    int    killed;  /* 1 = output dipaksa 0 */
} arb_out_t;

/*
 * Pilih setpoint.
 *
 * kill_active  : 1 = STM32 kill / E-stop / software kill
 * manual_active: 1 = operator memegang kendali (gamepad/RC)
 * manual_surge, manual_yaw : setpoint manual
 * auto_surge,   auto_yaw   : setpoint otonom (vision/waypoint)
 */
void arb_select(int kill_active, int manual_active,
                double manual_surge, double manual_yaw,
                double auto_surge, double auto_yaw,
                arb_out_t *out);

#ifdef __cplusplus
}
#endif

#endif /* ASV_ARBITRATOR_H */
