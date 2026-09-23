/*
 * core/include/mavlink_bridge.h — Bantuan MAVLink serial (framing v1 + pesan)
 *
 * Meneruskan jalur Python (app/mavlink_telemetry.py + navigator.py) ke C:
 *   - framing MAVLink v1 (magic 0xFE) dengan CRC-16/MCRF4XX (X.25)
 *   - payload HEARTBEAT (9 byte) & SET_ATTITUDE_TARGET (39 byte)
 *   - buka/tulis serial termios (baud generik; Pixhawk 6C lewat USB
 *     mengabaikan baud, tapi tabel tetap disediakan untuk UART)
 *
 * Layout & crc_extra diambil DARI pymavlink 2.4.49 yang terpasang
 * (pymavlink/dialects/v10/ardupilotmega.py) dan diverifikasi byte-identik
 * lewat tests/test_core_mavlink_bridge.py. Little-endian wire format
 * (sama seperti MAVLink); untuk endian-besar bangun dulu di buffer LE.
 *
 * Pemakaian:  #include "mavlink_bridge.h"
 */
#ifndef ASV_MAVLINK_BRIDGE_H
#define ASV_MAVLINK_BRIDGE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Pengenal pesan (dari message_definitions, pymavlink v10) --- */
#define MAV_MSGID_HEARTBEAT           0u
#define MAV_MSGID_SET_ATTITUDE_TARGET 82u

/* crc_extra spesifik pesan (pymavlink v10 dialect). */
#define MAV_CRC_EXTRA_HEARTBEAT           50u
#define MAV_CRC_EXTRA_SET_ATTITUDE_TARGET 49u

/* Ukuran payload (byte) sesuai unpacker pymavlink: <IBBBBB dan <I4fffffBBB */
#define MAV_PAYLOAD_SIZE_HEARTBEAT           9u
#define MAV_PAYLOAD_SIZE_SET_ATTITUDE_TARGET 39u

/* Ukuran frame v1 tanpa payload (STX+LEN+SEQ+SYSID+COMPID+MSGID+CRC) */
#define MAV_V1_HEADER_SIZE 6u
#define MAV_V1_CRC_SIZE    2u
#define MAV_MAX_FRAME_SIZE (MAV_V1_HEADER_SIZE + 255u + MAV_V1_CRC_SIZE)

/* Magic byte frame versi 1 */
#define MAV_V1_MAGIC 0xFEu

/* ------------------------------------------------------------------------ */
/* CRC-16/MCRF4XX (X.25) — sama dengan pymavlink.mavutil.x25crc             */
/* ------------------------------------------------------------------------ */

/**
 * mav_crc_accumulate — tambahkan satu byte ke state CRC (bit-reflected).
 * @param crc state CRC saat ini (awali dengan 0xFFFF)
 * @param b   byte berikutnya (STX..payload, lalu crc_extra)
 * @return state CRC baru; langkah terakhir: xor 0xFFFF.
 */
uint16_t mav_crc_accumulate(uint16_t crc, uint8_t b);

/**
 * mav_crc_v1_frame — CRC final untuk satu frame v1.
 * Hitung atas [STX, LEN, SEQ, SYSID, COMPID, MSGID, payload], lalu suapkan
 * crc_extra pesan dan xor 0xFFFF (persis aturan MAVLink).
 */
uint16_t mav_crc_v1_frame(uint8_t msgid, uint8_t sysid, uint8_t compid,
                          uint8_t seq, const uint8_t *payload, uint16_t len,
                          uint8_t crc_extra);

/* ------------------------------------------------------------------------ */
/* Builder payload (LE via memcpy; layout = unpacker pymavlink)             */
/* ------------------------------------------------------------------------ */

/**
 * mav_format_heartbeat — payload HEARTBEAT (9 byte).
 * @return ukuran payload (MAV_PAYLOAD_SIZE_HEARTBEAT).
 */
size_t mav_format_heartbeat(uint8_t type, uint8_t autopilot, uint8_t base_mode,
                            uint32_t custom_mode, uint8_t system_status,
                            uint8_t mavlink_version, uint8_t *payload);

/**
 * mav_format_set_attitude_target — payload SET_ATTITUDE_TARGET (39 byte).
 * @param time_boot_ms  ms sejak boot (MAVLink v1 menamai field ini time_boot_ms)
 * @param q             kuaternion orientasi tujuan [4]
 * @param thrust        dorongan 0..1
 * @return ukuran payload (MAV_PAYLOAD_SIZE_SET_ATTITUDE_TARGET).
 */
size_t mav_format_set_attitude_target(uint32_t time_boot_ms, const float q[4],
                                      float body_roll_rate,
                                      float body_pitch_rate,
                                      float body_yaw_rate, float thrust,
                                      uint8_t target_system,
                                      uint8_t target_component,
                                      uint8_t type_mask, uint8_t *payload);

/* ------------------------------------------------------------------------ */
/* Framing v1                                                                */
/* ------------------------------------------------------------------------ */

/**
 * mav_build_v1_frame — pasang header + payload + CRC jadi satu frame.
 * @param out buffer minimal MAV_MAX_FRAME_SIZE
 * @return panjang total frame (6 + len + 2), 0 bila buffer tidak cukup.
 */
size_t mav_build_v1_frame(uint8_t msgid, uint8_t sysid, uint8_t compid,
                          uint8_t seq, const uint8_t *payload, uint16_t len,
                          uint8_t crc_extra, uint8_t *out, size_t out_cap);

/**
 * mav_build_heartbeat_v1 — satu frame HEARTBEAT siap kirim (17 byte).
 */
size_t mav_build_heartbeat_v1(uint8_t sysid, uint8_t compid, uint8_t seq,
                              uint8_t type, uint8_t autopilot, uint8_t base_mode,
                              uint32_t custom_mode, uint8_t system_status,
                              uint8_t mavlink_version,
                              uint8_t *out, size_t out_cap);

/**
 * mav_build_set_attitude_target_v1 — satu frame SET_ATTITUDE_TARGET (47 byte).
 */
size_t mav_build_set_attitude_target_v1(uint8_t sysid, uint8_t compid,
                                        uint8_t seq, uint32_t time_boot_ms,
                                        const float q[4], float body_roll_rate,
                                        float body_pitch_rate,
                                        float body_yaw_rate, float thrust,
                                        uint8_t target_system,
                                        uint8_t target_component,
                                        uint8_t type_mask,
                                        uint8_t *out, size_t out_cap);

/* ------------------------------------------------------------------------ */
/* Serial (termios; jalur Python: detect_serial_port + baud 57600)          */
/* ------------------------------------------------------------------------ */

/**
 * mav_serial_open — buka port serial 8N1, non-blocking read.
 * @param path mis. "/dev/ttyACM0"; baud umum 57600/115200/921600 dsb.
 * @return fd >= 0 sukses; negatif (kode errno) gagal — TIDAK abort.
 */
int mav_serial_open(const char *path, int baud);

/** mav_serial_send — tulis semua byte; return 0 sukses, -1 gagal. */
int mav_serial_send(int fd, const uint8_t *buf, size_t len);

/** mav_serial_close — tutup fd (abaikan bila fd < 0). */
void mav_serial_close(int fd);

#ifdef __cplusplus
}
#endif

#endif /* ASV_MAVLINK_BRIDGE_H */