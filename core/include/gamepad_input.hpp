/*
 * core/include/gamepad_input.hpp — Baca joystick USB Linux via evdev (C++ tipis)
 *
 * Backup darat bila TX/RX mati: gamepad USB (/dev/input/js0) dibaca langsung
 * lewat evdev tanpa bergantung pada modul kernel joystick lama. Pemetaan tombol
 * yang SAMA dengan RC (manual_control.c) — tidak ada jalur khusus gamepad.
 */
#ifndef ASV_GAMEPAD_INPUT_HPP
#define ASV_GAMEPAD_INPUT_HPP

#ifdef __cplusplus
extern "C" {
#endif

#define GAMEPAD_MAX_FD 4

int gamepad_open(const char *path);
int gamepad_poll(int fd, double *surge, double *yaw, int *deadman);
void gamepad_close(int fd);

#ifdef __cplusplus
}
#endif

#endif /* ASV_GAMEPAD_INPUT_HPP */
