/*
 * core/src/gamepad_input.cpp — Baca joystick USB Linux via evdev (C++ tipis)
 *
 * Protokol js (/dev/input/js*): header 8 byte [time(4, LE), value(2, LE
 * signed), type(1), number(1)]. type & 0x7F: 0x01 = tombol, 0x02 = axis.
 * bit 0x80 = event inisialisasi (state awal saat open — tetap dipakai).
 */

#include "gamepad_input.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>

/* Tombol Linux melekat saat ditekan (value=1), lepas (value=0). */
#define JS_TYPE_BUTTON 0x01
#define JS_TYPE_AXIS 0x02
#define JS_TYPE_MASK 0x7F

int gamepad_open(const char *path) {
    int fd;

    if (path == 0 || path[0] == '\0') {
        return -1;
    }
    fd = ::open(path, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        return -1;
    }
    return fd;
}

int gamepad_poll(int fd, double *surge, double *yaw, int *deadman) {
    /* State terakhir: stik tengah, deadman lepas. */
    static double last_surge[GAMEPAD_MAX_DEVICES];
    static double last_yaw[GAMEPAD_MAX_DEVICES];
    static int last_deadman[GAMEPAD_MAX_DEVICES];
    static int seen[GAMEPAD_MAX_DEVICES];
    int slot;
    std::uint8_t buf[8];
    ssize_t n;

    if (fd < 0 || surge == 0 || yaw == 0 || deadman == 0) {
        return -1;
    }
    /* Slot cache dari fd (fd kecil berurutan — cukup untuk 4 device). */
    slot = fd % GAMEPAD_MAX_DEVICES;
    if (!seen[slot]) {
        last_surge[slot] = 0.0;
        last_yaw[slot] = 0.0;
        last_deadman[slot] = 0;
        seen[slot] = 1;
    }

    for (;;) {
        /* Baca satu event utuh (8 byte); EAGAIN = antrean habis. */
        std::size_t got = 0;
        while (got < sizeof(buf)) {
            n = ::read(fd, buf + got, sizeof(buf) - got);
            if (n < 0) {
                if (errno == EINTR) {
                    continue;
                }
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break; /* antrean habis — pakai state terakhir */
                }
                return -1; /* fd rusak/putus */
            }
            if (n == 0) {
                return -1; /* device dicabut */
            }
            got += static_cast<std::size_t>(n);
        }
        if (got == 0) {
            break;
        }
        if (got != sizeof(buf)) {
            break; /* parsial — abaikan, pakai state terakhir */
        }

        {
            std::int16_t value;
            std::uint8_t type;
            std::uint8_t number;
            std::memcpy(&value, buf + 4, 2); /* LE signed */
            type = static_cast<std::uint8_t>(buf[6] & JS_TYPE_MASK);
            number = buf[7];
            if (type == JS_TYPE_AXIS) {
                double norm = static_cast<double>(value) /
                              static_cast<double>(GAMEPAD_AXIS_MAX);
                if (norm < -1.0) {
                    norm = -1.0;
                }
                if (norm > 1.0) {
                    norm = 1.0;
                }
                if (number == GAMEPAD_AXIS_SURGE) {
                    /* Stik atas = nilai negatif -> surge positif (maju). */
                    last_surge[slot] = -norm;
                } else if (number == GAMEPAD_AXIS_YAW) {
                    last_yaw[slot] = norm;
                }
            } else if (type == JS_TYPE_BUTTON) {
                if (number == GAMEPAD_BTN_DEADMAN) {
                    last_deadman[slot] = (value != 0) ? 1 : 0;
                }
            }
        }
    }

    *surge = last_surge[slot];
    *yaw = last_yaw[slot];
    *deadman = last_deadman[slot];
    return 0;
}

void gamepad_close(int fd) {
    if (fd >= 0) {
        ::close(fd);
    }
}
