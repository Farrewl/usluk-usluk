/*
 * core/include/mode_manager.h — State mode kendali AUTO/MANUAL/HOLD/KILL (C)
 *
 * Satu-satunya tempat keputusan "kapal dikendalikan siapa". Prioritas:
 *   KILL (E-stop/STM32/software) > MANUAL (RC/gamepad + deadman) >
 *   HOLD (RC putus di tengah MANUAL, tahan 1 detik) > AUTO (waypoint/vision).
 * Hysteresis via HOLD: RC hilang sesaat tidak langsung lempar ke AUTO.
 */
#ifndef ASV_MODE_MANAGER_H
#define ASV_MODE_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Mode kendali (untuk log/GUI — urutan = prioritas tampilan, bukan rank). */
typedef enum {
    OP_MODE_AUTO = 0,
    OP_MODE_MANUAL = 1,
    OP_MODE_HOLD = 2,
    OP_MODE_KILL = 3
} op_mode_t;

/* State manager (diisi caller, di-nol-kan via mode_reset). */
typedef struct {
    op_mode_t mode;          /* mode aktif saat ini */
    double    hold_timer_s;  /* lama dalam HOLD (detik) */
} mode_state_t;

/* Kembali ke AUTO, timer nol (awal program / ganti misi). */
void mode_reset(mode_state_t *s);

/*
 * Perbarui mode. Return mode BARU (juga tersimpan di s->mode).
 * kill: 1 = E-stop/STM32/software kill (paling tinggi, langsung KILL).
 * manual_req: 1 = operator minta manual (switch TX / pilih GUI).
 * rc_ok: 1 = frame RC terakhir valid & belum timeout (lihat RC_TIMEOUT_MS).
 * dt: detik sejak panggilan lalu. lost_hold_s: MANUAL_LOST_HOLD_S.
 * HOLD dipertahankan s.d. lost_hold_s detik, lalu jatuh ke AUTO.
 */
op_mode_t mode_update(mode_state_t *s, int kill, int manual_req, int rc_ok,
                      double dt, double lost_hold_s);

/* Nama mode untuk log/GUI ("AUTO"/"MANUAL"/"HOLD"/"KILL", "?" bila asing). */
const char *mode_name(op_mode_t m);

#ifdef __cplusplus
}
#endif

#endif /* ASV_MODE_MANAGER_H */
