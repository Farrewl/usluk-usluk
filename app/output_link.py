"""app/output_link.py — Kirim perintah gerak ke Pixhawk (jalur produksi).

Output thruster = `SET_ATTITUDE_TARGET` via MAVLink ke Pixhawk. Pixhawk
TETAP pemegang ESC/mixer; companion hanya mengirim setpoint offboard:

    thrust  = surge      [-1 mundur .. +1 maju]  (param 7 pesan)
    yaw     = quaternion dari heading target     (field 0..3)
    lateral = body_roll_rate (belokan halus, BIT 0 type_mask)

Type mask: bit 1 & 2 (pitch rate & yaw rate diabaikan — kita pakai attitude
yaw penuh), bit 0 = roll rate (dipakai hanya bila ada lateral_thrust).

Mixer differential (C, core/src/thruster_mixer.c) juga diekspos via
`mixer_pwm()` untuk HUD/log/uji — TIDAK dikirim ke Pixhawk (Pixhawk yang
memetakan thrust+yaw ke PWM motor).
"""

import math
import time

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:          # pragma: no cover - mesin tanpa pymavlink
    mavutil = None
    MAVLINK_AVAILABLE = False

from . import aterkia_core as core
from .logutil import get_logger, throttled

log = get_logger()

# ID mode RTL ArduPilot Rover (lihat mode_mapping_rover pymavlink).
ROVER_RTL_MODE = 11
# Jeda minimal antar percobaan kirim RTL (detik, monotonic).
RTL_RETRY_INTERVAL_S = 5.0
# Jeda minimal antar kirim DISARM (detik, monotonic).
DISARM_RETRY_INTERVAL_S = 2.0


class OutputLink:
    """Jembatan setpoint → MAVLink. Tipis, tanpa rumus navigasi."""

    def __init__(self, config):
        self.config = config
        self.master = None
        self.last_stream_time = 0.0
        self._rtl_sent_mono = 0.0
        self._disarm_sent_mono = 0.0

    # ------------------- koneksi -------------------

    def attach_mav(self, master):
        """Sambungkan koneksi pymavlink Pixhawk (boleh None)."""
        self.master = master

    @property
    def ready(self):
        return self.master is not None

    # ------------------- geometri / mixer -------------------

    @staticmethod
    def yaw_to_quaternion(yaw_rad):
        """Heading (rad) -> quaternion [w, x, y, z] (rotasi murni sumbu-Z)."""
        if not isinstance(yaw_rad, (int, float)) or math.isnan(yaw_rad):
            yaw_rad = 0.0
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        return [cy, 0.0, 0.0, sy]

    @staticmethod
    def mixer_pwm(surge, yaw):
        """Pratinjau PWM kiri/kanan (µs) dari mixer C — untuk HUD/log."""
        try:
            return core.mix_diff_drive(float(surge), float(yaw))
        except Exception:
            return (1500, 1500)

    # ------------------- kirim setpoint -------------------

    def _set_attitude_target(self, thrust, target_yaw_rad, lateral_thrust=0.0):
        if self.master is None:
            return
        thrust = max(-1.0, min(1.0, float(thrust)))
        lateral = max(-1.0, min(1.0, float(lateral_thrust)))
        # bit 1 (pitch rate) & bit 2 (yaw rate) diabaikan; bit 0 (roll rate)
        # hanya bila tidak ada lateral -> pakai attitude penuh.
        type_mask = (1 << 1) | (1 << 2)
        body_roll_rate = 0.0
        if abs(lateral) < 0.01:
            type_mask |= (1 << 0)
        else:
            body_roll_rate = lateral
        quat = self.yaw_to_quaternion(target_yaw_rad)
        try:
            self.master.mav.set_attitude_target_send(
                0, self.master.target_system, self.master.target_component,
                type_mask, quat, body_roll_rate, 0, 0, thrust)
        except Exception as e:
            if throttled("att_fail", 5.0):
                log.warning("Gagal kirim SET_ATTITUDE_TARGET: %s", e)

    def send(self, thrust, target_yaw_rad, force_send=False, lateral_thrust=0.0):
        """Kirim setpoint, di-throttle ke OFFBOARD_STREAM_RATE_HZ.

        Bila `target_yaw_rad` NaN/aneh -> pakai heading terakhir yang valid
        (dipanggil caller) atau 0.0.
        """
        now = time.time()
        rate = float(getattr(self.config, "OFFBOARD_STREAM_RATE_HZ", 30) or 0)
        interval = (1.0 / rate) if rate > 0 else float("inf")
        if not force_send and (now - self.last_stream_time) < interval:
            return False
        if (not isinstance(target_yaw_rad, (int, float))
                or math.isnan(target_yaw_rad)):
            target_yaw_rad = 0.0
        self._set_attitude_target(thrust, target_yaw_rad, lateral_thrust)
        self.last_stream_time = now
        return True

    def prepare_offboard(self, seconds=3.0, yaw_rad=0.0):
        """Kirim stream awal nol `seconds` detik agar Pixhawk boleh OFFBOARD."""
        if self.master is None:
            return
        log.info("Kirim stream awal (%.0f dtk) — siap ARM + OFFBOARD...",
                 seconds)
        rate = float(getattr(self.config, "OFFBOARD_STREAM_RATE_HZ", 30) or 30)
        sleep_s = max(0.0, (1.0 / rate) - 0.001) if rate > 0 else 0.03
        end = time.time() + float(seconds)
        while time.time() < end:
            self._set_attitude_target(0.0, yaw_rad)
            time.sleep(sleep_s)

    def stop_stream(self, times=10, yaw_rad=0.0):
        """Kirim netral beberapa kali (matikan thruster) sebelum tutup."""
        rate = float(getattr(self.config, "OFFBOARD_STREAM_RATE_HZ", 30) or 30)
        sleep_s = (1.0 / rate + 0.01) if rate > 0 else 0.05
        for _ in range(int(times)):
            self._set_attitude_target(0.0, yaw_rad, 0.0)
            time.sleep(sleep_s)

    # ------------------- failsafe / kill -------------------

    def request_rtl(self):
        """Minta mode RTL via MAV_CMD_DO_SET_MODE (best-effort, throttled).

        Pixhawk TETAP pemegang failsafe utama (RCIN + geofence bawaan);
        perintah ini hanya usaha tambahan, maks 1x per RTL_RETRY_INTERVAL_S
        agar tak membanjiri link MAVLink yang sedang bermasalah.
        """
        if self.master is None or mavutil is None:
            return
        now = time.monotonic()
        if now - self._rtl_sent_mono < RTL_RETRY_INTERVAL_S:
            return
        self._rtl_sent_mono = now
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                ROVER_RTL_MODE, 0, 0, 0, 0, 0)
            log.warning("FAILSAFE: perintah RTL dikirim (best-effort).")
        except Exception as e:
            if throttled("rtl_fail", 10.0):
                log.warning("FAILSAFE: gagal kirim RTL: %s", e)

    def send_disarm(self):
        """KILL: minta Pixhawk DISARM (MAV_CMD_COMPONENT_ARM_DISARM, throttled).

        Pixhawk TETAP pemegang keselamatan utama (RCIN + failsafe bawaan);
        perintah ini pelengkap agar ESC benar-benar mati.
        """
        if self.master is None or mavutil is None:
            return
        now = time.monotonic()
        if now - self._disarm_sent_mono < DISARM_RETRY_INTERVAL_S:
            return
        self._disarm_sent_mono = now
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                0, 0, 0, 0, 0, 0, 0)
            log.warning("KILL: disarm dikirim ke Pixhawk (best-effort).")
        except Exception as e:
            if throttled("disarm_fail", 10.0):
                log.warning("KILL: gagal kirim disarm: %s", e)
