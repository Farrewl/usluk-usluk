/*
 * core/include/ecu_link.h — Protokol UART NUC <-> STM32F103 ECU (C murni)
 *
 * STM32 mengirim frame status biner 8 byte tiap 100 ms:
 *
 *   [0]  0xAA           (header byte 1)
 *   [1]  0x55           (header byte 2)
 *   [2]  status_flags   (bit0=kill, bit1=estop, bit2=overtemp, bit3=overcurrent)
 *   [3]  current_A x10  (0..255 = 0.0..25.5 A, skala ACS758)
 *   [4]  temp_esc_C     (derajat C, offset 0 — suhu NTC B3950)
 *   [5]  temp_amb_C     (derajat C, suhu BME280)
 *   [6]  volt_V x10     (0..255 = 0.0..25.5 V, resistor divider)
 *   [7]  checksum       (jumlah byte [2..6] & 0xFF)
 *
 * NUC mengirim perintah 1 byte ke STM32 bila perlu (kill/clear):
 *   0x00 = normal, 0x01 = minta kill (software E-stop), 0x02 = clear fault.
 */

#ifndef ASV_ECU_LINK_H
#define ASV_ECU_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Ukuran & konstanta frame */
#define ECU_FRAME_LEN 8
#define ECU_HDR_0 0xAA
#define ECU_HDR_1 0x55

/* Bit status_flags */
#define ECU_FLAG_KILL        0x01  /* STM32 memutuskan output driver */
#define ECU_FLAG_ESTOP       0x02  /* tombol E-stop fisik ditekan */
#define ECU_FLAG_OVERTEMP    0x04  /* suhu ESC > ambang */
#define ECU_FLAG_OVERCURRENT 0x08  /* arus motor > ambang */

/* Perintah NUC -> STM32 */
#define ECU_CMD_NORMAL 0x00
#define ECU_CMD_KILL   0x01
#define ECU_CMD_CLEAR  0x02

/* Hasil parse satu frame status (nilai sudah diskala ke satuan fisik). */
typedef struct {
    int   valid;          /* 1 = checksum & header OK */
    int   kill;           /* 1 = output driver diputus */
    int   estop;          /* 1 = E-stop ditekan */
    int   overtemp;       /* 1 = ESC kepanasan */
    int   overcurrent;    /* 1 = arus berlebih */
    double current_a;     /* ampere */
    double temp_esc_c;    /* derajat C */
    double temp_amb_c;    /* derajat C */
    double voltage_v;     /* volt */
} ecu_status_t;

/*
 * Parse 8 byte mentah menjadi ecu_status_t.
 * `raw` harus menunjuk ke buffer minimal ECU_FRAME_LEN byte.
 * Return 1 bila valid, 0 bila header/checksum salah (valid=0).
 */
int ecu_parse_frame(const unsigned char *raw, ecu_status_t *out);

/*
 * Bangun 1 byte perintah NUC -> STM32.
 * cmd salah satu dari ECU_CMD_*; nilai lain dianggap NORMAL.
 */
unsigned char ecu_build_cmd(int cmd);

#ifdef __cplusplus
}
#endif

#endif /* ASV_ECU_LINK_H */
