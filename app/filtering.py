"""
app/filtering.py — Filter & kontroler untuk galat navigasi (referensi Python).

Semua "pemikiran untuk galat" dipusatkan di sini (bukan tersebar di
navigator.py) supaya bisa diuji per-modul dan di-port 1:1 ke C di
`core/src/control_filters.c` — dibuktikan identik lewat
`tests/test_core_control_filters.py` (ctypes vs Python), mengikuti pola
`app/geo.py` <-> `core/src/nav_math.c`.

Isi modul (sesuai permintaan):
  1. PID dengan deadband   — kontroler koreksi yaw pengganti gain-P murni.
  2. Complementary filter  — fusi heading terukur dengan laju yaw gyro.
  3. EKF heading (1-dimensi) — estimasi heading + bias gyro yang halus.
  4. Normalisasi wrap-around — delegasi ke app/geo.normalize_angle.
  5. Cross-Track Error (CTE) — delegasi ke app/geo.cross_track_distance.

Semua fungsi murni/tanpa state tersembunyi (kecuali objek kontroler yang
state-nya eksplisit lewat atribut) sehingga mudah diuji deterministik.
"""

import math

from . import geo

# Batas dt yang masuk akal (detik) untuk melindungi pembagian saat loop
# menghasilkan dt=0 (mis. kondisi telemetri kosong).
_MIN_DT = 1e-4


class PidController:
    """PID dengan deadband, anti-windup, dan batas output.

    Dipakai untuk mengubah error (radian) menjadi koreksi yaw (radian).
    Perilaku kunci:
      * Deadband: bila |error| < deadband, output 0 dan integral TIDAK
        ditambah (mencegah osilasi kecil / "jitter" saat sudah nyaris tepat).
      * Anti-windup: integral dibatasi oleh `integral_limit` sehingga
        koreksi yang lama jenuh tidak membuat respons terlambat.
      * Output dibatasi `output_limit` (clamp) — koreksi yaw tak mungkin
        melebihi belokan wajar kapal.
    """

    def __init__(self, kp=1.0, ki=0.0, kd=0.0, deadband=0.0,
                 output_limit=0.6, integral_limit=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband
        self.output_limit = output_limit
        self.integral_limit = integral_limit
        self.reset()

    def reset(self):
        """Nol-kan state integral & error sebelumnya (awal misi / leg baru)."""
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, error, dt=0.05):
        """Hitung koreksi PID untuk `error` pada selang `dt` detik.

        Urutan sesuai buku teks digital PID: hitung term, clamp integral
        (anti-windup), lalu clamp output. Deadband dipakai pada error MASUKAN
        (input), bukan output, agar kapal benar-benar berhenti "mengoreksi"
        saat sudah di tengah target.
        """
        if dt <= 0.0:
            dt = _MIN_DT
        if abs(error) < self.deadband:
            # Di dalam zona mati: jangan koreksi, jangan akumulasi integral.
            self.last_error = error
            return 0.0

        self.integral += error * dt
        if self.integral_limit is not None:
            limit = abs(self.integral_limit)
            self.integral = max(-limit, min(limit, self.integral))

        derivative = (error - self.last_error) / dt
        self.last_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        if self.output_limit is not None:
            limit = abs(self.output_limit)
            output = max(-limit, min(limit, output))
        return output


def complementary_filter(alpha, angle_prev, gyro_rate, dt, angle_measured):
    """Fusi heading komplementer: percaya gyro jangka pendek, ukuran jangka panjang.

    Rumus:
        angle = alpha * (angle_prev + gyro_rate * dt) + (1 - alpha) * angle_measured

    - alpha mendekati 1 -> mengikuti gyro (halus, tapi bisa hanyut).
    - alpha mendekati 0 -> mengikuti pengukuran (responsif, tapi berisik).
    Default alpha = 0.6 (app/settings.py COMPLEMENTARY_ALPHA) sebagai
    kompromi standar. Sudut diharapkan sudah ternormalisasi oleh pemanggil.
    """
    if dt <= 0.0:
        dt = _MIN_DT
    return alpha * (angle_prev + gyro_rate * dt) + (1.0 - alpha) * angle_measured


class HeadingEkf:
    """EKF 1-dimensi untuk estimasi heading + bias gyro.

    State  : x = [heading, bias_gyro]  (radian).
    Model  : heading' = heading + (gyro_rate - bias_gyro) * dt
             bias_gyro' = bias_gyro  (random walk, dimodelkan di Q)
    Observasi : heading hasil visi / GPS / kompas (varians EKF_MEAS_NOISE).

    Inovasi dinormalisasi ke [-pi, pi] (wrap-around) supaya lompatan sudut
    -179° -> +179° tidak dianggap selisih 358°.
    """

    def __init__(self, process_noise=0.05, meas_noise=0.10, init_heading=0.0):
        self.q_heading = process_noise      # noise model heading (rad^2/s)
        self.q_bias = process_noise * 0.1   # drift bias gyro (lebih lambat)
        self.r_meas = meas_noise            # noise pengukuran (rad^2)
        self.state = [float(init_heading), 0.0]  # [heading, bias]
        # Kovarians 2x2 (matriks simetris): 3 nilai unik.
        self.p00 = 1.0
        self.p01 = 0.0
        self.p11 = 1.0

    def predict(self, gyro_rate, dt):
        """Prediksi dengan laju gyro (rad/s) selama `dt` detik."""
        if dt <= 0.0:
            dt = _MIN_DT
        heading, bias = self.state
        heading += (gyro_rate - bias) * dt

        # F = [[1, -dt], [0, 1]] -> perbarui P = F P F^T + Q.
        p00 = self.p00 - 2.0 * dt * self.p01 + dt * dt * self.p11 + self.q_heading * dt
        p01 = self.p01 - dt * self.p11
        p11 = self.p11 + self.q_bias * dt
        self.state = [heading, bias]
        self.p00, self.p01, self.p11 = p00, p01, p11
        return heading

    def update(self, measured_heading):
        """Koreksi dengan pengukuran heading (radian), inovasi wrap-around."""
        heading, bias = self.state
        innovation = geo.normalize_angle(measured_heading - heading)

        # S = H P H^T + R dengan H = [1, 0] -> S = p00 + R.
        s = self.p00 + self.r_meas
        if s <= 0.0:
            s = _MIN_DT
        # Gain Kalman K = P H^T / S -> [k0, k1].
        k0 = self.p00 / s
        k1 = self.p01 / s

        heading += k0 * innovation
        bias += k1 * innovation

        # P = (I - K H) P, mengingat H = [1, 0].
        p00 = self.p00 * (1.0 - k0)
        p01 = self.p01 * (1.0 - k0)
        p11 = self.p11 - k1 * self.p01

        self.state = [heading, bias]
        self.p00, self.p01, self.p11 = p00, p01, p11
        return heading

    @property
    def heading(self):
        """Heading estimasi ternormalisasi (radian)."""
        return geo.normalize_angle(self.state[0])


def normalize_wrap(angle_rad):
    """Alias ringkas normalisasi wrap-around (rentang [-pi, pi])."""
    return geo.normalize_angle(angle_rad)


def cross_track_error(lat_p, lon_p, lat_wp1, lon_wp1, lat_wp2, lon_wp2):
    """Delegasi CTE (Cross-Track Error) ke app/geo — satu sumber kebenaran."""
    return geo.cross_track_distance(lat_p, lon_p, lat_wp1, lon_wp1,
                                    lat_wp2, lon_wp2)