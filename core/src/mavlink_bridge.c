/*
 * core/src/mavlink_bridge.c — Bantuan MAVLink serial (implementasi)
 *
 * Semua angka ajaib punya nama: ukuran/layout/crc_extra mengikuti
 * pymavlink v10 dialect yang terpasang (lihat header). Implementasi CRC
 * disalin secara algoritma dari pymavlink.mavutil.x25crc (bit-reflected,
 * poly 0x8408) supaya hasil byte-identik.
 */
#include "mavlink_bridge.h"

#include <errno.h>
#include <string.h>

#if defined(__unix__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

/* ------------------------------------------------------------------------ */
/* CRC-16/MCRF4XX ala MAVLink — identik mavlink_crc_update / pymavlink      */
/* x25crc (bukan CRC reflected generik!). Mulai 0xFFFF, TANPA xor final.     */
/* ------------------------------------------------------------------------ */

uint16_t mav_crc_accumulate(uint16_t crc, uint8_t b) {
    uint8_t tmp = (uint8_t)(b ^ (uint8_t)(crc & 0xFFu));
    tmp ^= (uint8_t)(tmp << 4);
    return (uint16_t)(((crc >> 8) ^ ((uint16_t)tmp << 8)
                       ^ ((uint16_t)tmp << 3) ^ ((uint16_t)(tmp >> 4)))
                      & 0xFFFFu);
}

uint16_t mav_crc_v1_frame(uint8_t msgid, uint8_t sysid, uint8_t compid,
                          uint8_t seq, const uint8_t *payload, uint16_t len,
                          uint8_t crc_extra) {
    uint16_t crc = 0xFFFFu;   /* inisialisasi standar MAVLink */
    uint16_t i;

    /* PENTING: MAVLink v1 menghitung CRC mulai byte LEN — STX (magic)
     * TIDAK ikut (pymavlink memakai msgbuf[1:]; C reference juga). */
    crc = mav_crc_accumulate(crc, (uint8_t)(len & 0xFFu));
    crc = mav_crc_accumulate(crc, seq);
    crc = mav_crc_accumulate(crc, sysid);
    crc = mav_crc_accumulate(crc, compid);
    crc = mav_crc_accumulate(crc, msgid);
    for (i = 0; i < len; i++) {
        crc = mav_crc_accumulate(crc, payload[i]);
    }
    crc = mav_crc_accumulate(crc, crc_extra);
    return crc;   /* langsung, tanpa xor 0xFFFF */
}

/* ------------------------------------------------------------------------ */
/* Builder payload (little-endian via memcpy; MAVLink wire format = LE)      */
/* ------------------------------------------------------------------------ */

size_t mav_format_heartbeat(uint8_t type, uint8_t autopilot, uint8_t base_mode,
                            uint32_t custom_mode, uint8_t system_status,
                            uint8_t mavlink_version, uint8_t *payload) {
    memcpy(payload + 0, &custom_mode, 4);     /* u32 LE */
    payload[4] = type;
    payload[5] = autopilot;
    payload[6] = base_mode;
    payload[7] = system_status;
    payload[8] = mavlink_version;
    return MAV_PAYLOAD_SIZE_HEARTBEAT;
}

size_t mav_format_set_attitude_target(uint32_t time_boot_ms, const float q[4],
                                      float body_roll_rate,
                                      float body_pitch_rate,
                                      float body_yaw_rate, float thrust,
                                      uint8_t target_system,
                                      uint8_t target_component,
                                      uint8_t type_mask, uint8_t *payload) {
    memcpy(payload + 0,  &time_boot_ms, 4);   /* u32 LE */
    memcpy(payload + 4,  &q[0], 4);
    memcpy(payload + 8,  &q[1], 4);
    memcpy(payload + 12, &q[2], 4);
    memcpy(payload + 16, &q[3], 4);
    memcpy(payload + 20, &body_roll_rate, 4);
    memcpy(payload + 24, &body_pitch_rate, 4);
    memcpy(payload + 28, &body_yaw_rate, 4);
    memcpy(payload + 32, &thrust, 4);
    payload[36] = target_system;
    payload[37] = target_component;
    payload[38] = type_mask;
    return MAV_PAYLOAD_SIZE_SET_ATTITUDE_TARGET;
}

/* ------------------------------------------------------------------------ */
/* Framing v1                                                                */
/* ------------------------------------------------------------------------ */

size_t mav_build_v1_frame(uint8_t msgid, uint8_t sysid, uint8_t compid,
                          uint8_t seq, const uint8_t *payload, uint16_t len,
                          uint8_t crc_extra, uint8_t *out, size_t out_cap) {
    const size_t total = MAV_V1_HEADER_SIZE + (size_t)len + MAV_V1_CRC_SIZE;
    uint16_t crc;
    size_t i;

    if (out_cap < total) {
        return 0;   /* buffer tidak cukup */
    }
    out[0] = MAV_V1_MAGIC;
    out[1] = (uint8_t)(len & 0xFFu);
    out[2] = seq;
    out[3] = sysid;
    out[4] = compid;
    out[5] = msgid;
    for (i = 0; i < len; i++) {
        out[MAV_V1_HEADER_SIZE + i] = payload[i];
    }
    crc = mav_crc_v1_frame(msgid, sysid, compid, seq, payload, len, crc_extra);
    out[total - 2] = (uint8_t)(crc & 0xFFu);        /* low byte dulu */
    out[total - 1] = (uint8_t)((crc >> 8) & 0xFFu); /* high byte */
    return total;
}

size_t mav_build_heartbeat_v1(uint8_t sysid, uint8_t compid, uint8_t seq,
                              uint8_t type, uint8_t autopilot, uint8_t base_mode,
                              uint32_t custom_mode, uint8_t system_status,
                              uint8_t mavlink_version,
                              uint8_t *out, size_t out_cap) {
    uint8_t payload[MAV_PAYLOAD_SIZE_HEARTBEAT];
    mav_format_heartbeat(type, autopilot, base_mode, custom_mode,
                         system_status, mavlink_version, payload);
    return mav_build_v1_frame(MAV_MSGID_HEARTBEAT, sysid, compid, seq,
                              payload, sizeof(payload),
                              MAV_CRC_EXTRA_HEARTBEAT, out, out_cap);
}

size_t mav_build_set_attitude_target_v1(uint8_t sysid, uint8_t compid,
                                        uint8_t seq, uint32_t time_boot_ms,
                                        const float q[4], float body_roll_rate,
                                        float body_pitch_rate,
                                        float body_yaw_rate, float thrust,
                                        uint8_t target_system,
                                        uint8_t target_component,
                                        uint8_t type_mask,
                                        uint8_t *out, size_t out_cap) {
    uint8_t payload[MAV_PAYLOAD_SIZE_SET_ATTITUDE_TARGET];
    mav_format_set_attitude_target(time_boot_ms, q, body_roll_rate,
                                   body_pitch_rate, body_yaw_rate, thrust,
                                   target_system, target_component, type_mask,
                                   payload);
    return mav_build_v1_frame(MAV_MSGID_SET_ATTITUDE_TARGET, sysid, compid,
                              seq, payload, sizeof(payload),
                              MAV_CRC_EXTRA_SET_ATTITUDE_TARGET, out, out_cap);
}

/* ------------------------------------------------------------------------ */
/* Serial termios                                                            */
/* ------------------------------------------------------------------------ */

#if defined(__unix__) || defined(__APPLE__)

static speed_t baud_to_speed(int baud) {
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 921600:  return B921600;
    default:      return B0;   /* tidak dikenal */
    }
}

int mav_serial_open(const char *path, int baud) {
    struct termios tio;
    speed_t speed = baud_to_speed(baud);
    int fd;

    if (speed == B0) {
        errno = EINVAL;
        return -1;
    }
    fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        return -1;   /* errno terjaga: ENOENT dsb — pemanggil boleh baca */
    }
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) != 0) {
        close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;          /* 8 data bit */
    tio.c_cflag &= ~PARENB;      /* tanpa parity */
    tio.c_cflag &= ~CSTOPB;      /* 1 stop bit */
    tio.c_cc[VMIN] = 0;          /* read tak memblok; polling */
    tio.c_cc[VTIME] = 0;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        close(fd);
        return -1;
    }
    return fd;
}

int mav_serial_send(int fd, const uint8_t *buf, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = write(fd, buf + sent, len - sent);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            return -1;
        }
        sent += (size_t)n;
    }
    return 0;
}

void mav_serial_close(int fd) {
    if (fd >= 0) {
        close(fd);
    }
}

#else /* non-POSIX: API tersedia tapi selalu gagal halus (EOPNOTSUPP) */

int mav_serial_open(const char *path, int baud) {
    (void)path;
    (void)baud;
    errno = EOPNOTSUPP;
    return -1;
}

int mav_serial_send(int fd, const uint8_t *buf, size_t len) {
    (void)fd;
    (void)buf;
    (void)len;
    errno = EOPNOTSUPP;
    return -1;
}

void mav_serial_close(int fd) {
    (void)fd;
}

#endif