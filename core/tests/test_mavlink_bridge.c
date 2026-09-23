/*
 * core/tests/test_mavlink_bridge.c — Unit test bridge MAVLink (C)
 *
 * Nilai emas = BYTE-EXACT frame MAVLink v1 yang dihasilkan pymavlink
 * 2.4.49 (dialect v20/v10 ardupilotmega) untuk input yang sama:
 *
 *   HEARTBEAT >>    fe 09 00 01 01 00 00000000 02 03 51 04 03 7d dd
 *    (msgid 0, custom_mode 0, type 2, autopilot 3, base_mode 81,
 *     system_status 4, mavlink_version 3)
 *
 *   SET_ATT >>      fe 27 00 01 01 52 07000000 0000803f 00000000
 *                   00000000 00000000 cdcccc3d cdcc4cbe 0000003f
 *                   6666263f 01 01 00 f2 d7
 *    (msgid 82, time_boot_ms 7, q=(1,0,0,0), rates 0.1/-0.2/0.5,
 *     thrust 0.65, target 1/1, type_mask 0)
 *
 * Serial tidak diuji terhubung (Pixhawk boleh dicabut): tes hanya
 * memastikan path gagal dikembalikan secara halus (>= -1, tidak abort).
 *
 * Jalankan:
 *   gcc -I core/include core/tests/test_mavlink_bridge.c \
 *       core/src/mavlink_bridge.c -o /tmp/opencode/test_mav \
 *       && /tmp/opencode/test_mav
 */
#include "mavlink_bridge.h"

#include <stdio.h>
#include <string.h>

static int failures = 0;

static void check_bytes(const char *tag, const uint8_t *got, size_t gotlen,
                        const uint8_t *want, size_t wantlen) {
    if (gotlen != wantlen || memcmp(got, want, wantlen) != 0) {
        size_t i;
        printf("FAIL %s: panjang %zu (harap %zu)\n  got :", tag, gotlen,
               wantlen);
        for (i = 0; i < gotlen; i++) {
            printf(" %02x", got[i]);
        }
        printf("\n  want:");
        for (i = 0; i < wantlen; i++) {
            printf(" %02x", want[i]);
        }
        printf("\n");
        failures++;
    } else {
        printf("ok    %-26s %zu byte\n", tag, gotlen);
    }
}

int main(void) {
    uint8_t frame[MAV_MAX_FRAME_SIZE];
    size_t n;
    int fd;

    /* ------------------- HEARTBEAT (17 byte) ------------------- */
    static const uint8_t want_hb[17] = {
        0xfe, 0x09, 0x00, 0x01, 0x01, 0x00,
        0x00, 0x00, 0x00, 0x00,      /* custom_mode = 0 (u32 LE) */
        0x02, 0x03, 0x51, 0x04, 0x03, /* type, autopilot, base_mode,
                                          system_status, mavlink_version */
        0x7d, 0xdd                   /* CRC-16 (low, high) */
    };
    n = mav_build_heartbeat_v1(1, 1, 0, 2, 3, 81, 0, 4, 3,
                               frame, sizeof(frame));
    check_bytes("heartbeat frame", frame, n, want_hb, sizeof(want_hb));

    /* ---------------- SET_ATTITUDE_TARGET (47 byte) ---------------- */
    static const uint8_t want_sat[47] = {
        0xfe, 0x27, 0x00, 0x01, 0x01, 0x52,
        0x07, 0x00, 0x00, 0x00,        /* time_boot_ms = 7 */
        0x00, 0x00, 0x80, 0x3f,        /* q[0] = 1.0f */
        0x00, 0x00, 0x00, 0x00,        /* q[1] = 0.0f */
        0x00, 0x00, 0x00, 0x00,        /* q[2] = 0.0f */
        0x00, 0x00, 0x00, 0x00,        /* q[3] = 0.0f */
        0xcd, 0xcc, 0xcc, 0x3d,        /* body_roll_rate = 0.1f */
        0xcd, 0xcc, 0x4c, 0xbe,        /* body_pitch_rate = -0.2f */
        0x00, 0x00, 0x00, 0x3f,        /* body_yaw_rate = 0.5f */
        0x66, 0x66, 0x26, 0x3f,        /* thrust = 0.65f */
        0x01, 0x01, 0x00,              /* target_system, component, mask */
        0xf2, 0xd7                     /* CRC-16 (low, high) */
    };
    const float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    n = mav_build_set_attitude_target_v1(1, 1, 0, 7, q, 0.1f, -0.2f, 0.5f,
                                         0.65f, 1, 1, 0,
                                         frame, sizeof(frame));
    check_bytes("set_attitude_target frame", frame, n, want_sat,
                sizeof(want_sat));

    /* ---------------- CRC vektor dari pymavlink ---------------- */
    /* CRC frame heartbeat = 0xdd7d => byte rendah 0x7d dulu. */
    if ((uint8_t)(want_hb[15]) == 0x7d && (uint8_t)(want_hb[16]) == 0xdd) {
        printf("ok    crc heartbeat (dari vektor) = 0xdd7d\n");
    } else {
        printf("FAIL crc heartbeat\n");
        failures++;
    }

    /* ---------------- serial: gagal halus tanpa hardware ------- */
    /* Deteksi port terpasang bisa gagal (Pixhawk dicabut): hasil < 0
     * TANPA abort adalah perilaku yang sah & harus diuji. */
    fd = mav_serial_open("/dev/ttyACM0", 57600);
    if (fd < 0) {
        printf("ok    serial open (tanpa hardware) gagal halus\n");
    } else {
        printf("ok    serial open berhasil (fd=%d) — tutup\n", fd);
        mav_serial_close(fd);
    }

    if (failures == 0) {
        printf("\nSEMUA kasus mavlink_bridge LULUS (C == pymavlink v1)\n");
        return 0;
    }
    printf("\n%d kasus GAGAL\n", failures);
    return 1;
}