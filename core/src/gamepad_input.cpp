/*
 * core/src/gamepad_input.cpp — Baca joystick USB Linux via evdev (C++ tipis)
 */
#include "gamepad_input.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#ifndef __linux__
int gamepad_open(const char *) { return -1; }
int gamepad_poll(int, double *, double *, int *) { return 0; }
void gamepad_close(int) {}
#else

#include <linux/joystick.h>

int gamepad_open(const char *path) {
    int fd = open(path, O_RDONLY | O_NONBLOCK);
    return fd;
}

int gamepad_poll(int fd, double *surge, double *yaw, int *deadman) {
    if (fd < 0) return 0;
    struct js_event ev;
    int updated = 0;
    while (read(fd, &ev, sizeof(ev)) > 0) {
        updated = 1;
        if (ev.type & JS_EVENT_AXIS) {
            double v = (double)ev.value / 32767.0;
            if (ev.number == 1) *surge = -v; // Axis Y terbalik
            else if (ev.number == 0 || ev.number == 3) *yaw = v;
        } else if (ev.type & JS_EVENT_BUTTON) {
            if (ev.number == 0) *deadman = ev.value;
        }
    }
    return updated;
}

void gamepad_close(int fd) {
    if (fd >= 0) close(fd);
}

#endif
