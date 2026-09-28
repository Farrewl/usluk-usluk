/*
 * core/include/gamepad_input.hpp — Baca joystick USB Linux via evdev (C++ tipis)
 *
 * Backup darat bila TX/RX mati: gamepad USB (/dev/input/js0) dibaca langsung
 * di C++ (RAII handle, non-blocking) lalu hasilnya masuk jalur `manual`
 * yang SAMA dengan RC (manual_control.c) — tidak ada jalur khusus gamepad.
 * API extern "C" supaya bisa dipanggil dari C & ctypes Python.
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Axis joystick yang dipakai (indeks js_event.number). */
#define GAMEPAD_AXIS_SURGE 1   /* stik kiri atas-bawah (atas = maju) */
#define GAMEPAD_AXIS_YAW   2   /* stik kanan kiri-kanan (kanan = +) */
#define GAMEPAD_BTN_DEADMAN 4  /* tombol LB — TAHAN untuk jalan */
/* Nilai axis mentah joystick Linux. */
#define GAMEPAD_AXIS_MAX 32767
/* Jumlah fd gamepad yang di-cache (cukup untuk 2 operator + cadangan). */
#define GAMEPAD_MAX_DEVICES 4

/*
 * Buka joystick. Return fd >= 0 (non-blocking) atau -1 bila gagal
 * (device tidak ada / bukan joystick). TIDAK abort — caller fallback RC.
 */
int gamepad_open(const char *path);

/*
 * Baca SEMUA event yang mengantre, kembalikan state stik terakhir.
 * surge/yaw: [-1..1] (surge+ = stik atas). deadman: 1 = LB ditahan.
 * Return 0 sukses, -1 bila fd rusak/putus (caller tutup + tandai hilang).
 */
int gamepad_poll(int fd, double *surge, double *yaw, int *deadman);

/* Tutup fd (abaikan bila fd < 0). */
void gamepad_close(int fd);

#ifdef __cplusplus
}
#endif
