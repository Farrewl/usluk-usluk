"""
app/mavlink_telemetry.py — Baca telemetri MAVLink dari Pixhawk (PX4/ArduPilot).

Tujuan: memberi GUI / simulator data attitude (roll, pitch, yaw) dan posisi
NYATA dari Pixhawk tanpa harus menjalankan navigator penuh (yang butuh
YOLO + model .pt). Dipakai GroundSimNavigator: kalau Pixhawk terhubung,
roll/pitch/yaw mengikuti IMU sungguhan; kalau tidak, fallback mock.

Koneksi (skill asv-mavlink):
  - Port otomatis dideteksi: /dev/ttyACM* (USB Pixhawk) -> /dev/ttyUSB*
    (FTDI) -> COM* (Windows). Pixhawk 6C lewat USB = /dev/ttyACM0 (MAVLink),
    /dev/ttyACM1 = console debug (JANGAN dipakai).
  - Baud 57600 standar; lewat USB baud diabaikan driver CDC.
  - Setelah heartbeat, kita minta stream attitude/posisi 10 Hz via
    REQUEST_DATA_STREAM + MAV_CMD_SET_MESSAGE_INTERVAL (lebih andal di PX4).
"""

import glob
import math
import os
import time

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:          # pragma: no cover - mesin tanpa pymavlink
    mavutil = None
    MAVLINK_AVAILABLE = False


def detect_serial_port(preferred=None):
    """Pilih port serial Pixhawk.

    Prioritas: `preferred` (bila ada di sistem) -> /dev/ttyACM* (USB CDC,
    urut naik — Pixhawk biasanya ACM0) -> /dev/ttyUSB* (FTDI) -> COM* (Win).
    Kalau tidak ada port yang eksis: Windows mengembalikan `preferred`
    (driver membukanya saat koneksi), Linux mengembalikan None.
    """
    candidates = []
    if preferred:
        candidates.append(preferred)
    candidates += sorted(glob.glob("/dev/ttyACM*"))
    candidates += sorted(glob.glob("/dev/ttyUSB*"))
    candidates += [f"COM{n}" for n in range(3, 10)]
    for port in candidates:
        if os.path.exists(port):
            return port
    if os.name == "nt" and preferred:
        return preferred
    return None


class MavlinkTelemetry:
    """Pembaca telemetri non-blocking dari satu koneksi MAVLink.

    Pemakaian:
        mav = MavlinkTelemetry(baud=57600)
        if mav.connect(timeout_s=5):
            ... loop tiap frame ...
            mav.poll()
            yaw = mav.yaw_deg   # menyesuaikan IMU Pixhawk
    """

    def __init__(self, port=None, baud=57600):
        self.port = detect_serial_port(port)
        self.baud = baud
        self.master = None
        self.connected = False
        self.last_heartbeat_time = 0.0
        self._attitude = {}            # {'roll','pitch','yaw'} rad
        self._pos = None               # (lat_deg, lon_deg) atau None (no GPS)
        self._groundspeed = 0.0
        # Baterai dari SYS_STATUS / BATTERY_STATUS (None = belum ada data).
        # voltage_v = tegangan pack (V), current_a = arus (A),
        # battery_pct = sisa 0..100 (%), dihitung dari tegangan bila persen
        # tidak dikirim firmware.
        self._voltage_v = None
        self._current_a = None
        self._battery_pct = None

    # ------------------- koneksi -------------------
    def connect(self, timeout_s=8.0):
        """Buka serial + tunggu heartbeat. True bila Pixhawk merespons."""
        if not MAVLINK_AVAILABLE:
            print("[MAV] pymavlink tidak terpasang — telemetri dimatikan.")
            return False
        if self.port is None:
            print("[MAV] Tidak ada port serial ditemukan.")
            return False

        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
        except Exception as exc:
            print(f"[MAV] Gagal membuka {self.port}: {exc}")
            return False

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="HEARTBEAT",
                                         blocking=True, timeout=1)
            if msg is not None:
                self.connected = True
                self.last_heartbeat_time = time.time()
                self._request_streams()
                print(f"[MAV] Heartbeat diterima dari "
                      f"sysid={self.master.target_system} "
                      f"compid={self.master.target_component} "
                      f"via {self.port} — attitude live.")
                return True
            print(f"[MAV] Menunggu heartbeat {self.port} ...")
        print(f"[MAV] Tidak ada heartbeat dari {self.port} "
              f"dalam {timeout_s:.0f}s — pakai mock attitude.")
        return False

    def close(self):
        if self.master is not None:
            self.master.close()
        self.master = None
        self.connected = False

    # ------------------- request stream -------------------
    def _request_streams(self):
        """Minta attitude/posisi mengalir (10 Hz) — PX4 & ArduPilot."""
        # Konstanta MAV_DATA_STREAM_* beda-beda antar dialect pymavlink;
        # kalau tidak dikenal -> di-skip. SET_MESSAGE_INTERVAL di bawah
        # tetap menjamin rate attitude walau request stream ditolak.
        for stream_name in ("MAV_DATA_STREAM_EXTRA_ATTITUDE",
                            "MAV_DATA_STREAM_POSITION",
                            "MAV_DATA_STREAM_EXTRA3"):
            stream_id = getattr(mavutil.mavlink, stream_name, None)
            if stream_id is None:
                continue
            try:
                self.master.mav.request_data_stream_send(
                    self.master.target_system, self.master.target_component,
                    stream_id, 10, 1)
            except Exception:
                pass  # request ditolak firmware -> abaikan

        # SET_MESSAGE_INTERVAL per pesan — lebih andal di PX4.
        for name in ("ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD",
                     "SYS_STATUS", "BATTERY_STATUS"):
            self._set_message_interval(name, 100_000)  # 10 Hz = 100 ms

    def _set_message_interval(self, name, interval_us):
        msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{name}", None)
        if not msg_id:
            return
        try:
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, interval_us, 0, 0, 0, 0, 0)
        except Exception:
            pass  # kalau firmware belum support, stream default tetap ada

    # ------------------- baca pesan (non-blocking) -------------------
    def poll(self):
        """Baca SEMUA pesan yang mengantre; perbarui state internal."""
        if not self.connected or self.master is None:
            return
        try:
            msg = self.master.recv_match(blocking=False)
            while msg is not None:
                mtype = msg.get_type()
                if mtype == "ATTITUDE":
                    self._attitude = {
                        "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw,
                    }
                elif mtype == "GLOBAL_POSITION_INT":
                    # lat/lon == 0 menandakan belum ada GPS fix (PX4)
                    if msg.lat != 0 or msg.lon != 0:
                        self._pos = (msg.lat / 1e7, msg.lon / 1e7)
                elif mtype == "VFR_HUD":
                    self._groundspeed = msg.groundspeed
                elif mtype == "SYS_STATUS":
                    # voltage_battery: millivolt (0/0xFFFF = tidak ada sensor).
                    # current_battery: 10*mA (-1 = tidak ada sensor).
                    # battery_remaining: persen 0..100 (-1 = tidak dikirim).
                    vbatt = getattr(msg, "voltage_battery", 0) or 0
                    if vbatt not in (0, 0xFFFF, 65535):
                        self._voltage_v = vbatt / 1000.0
                    curr = getattr(msg, "current_battery", -1)
                    if curr is not None and curr >= 0:
                        self._current_a = curr / 100.0
                    rem = getattr(msg, "battery_remaining", -1)
                    if rem is not None and 0 <= rem <= 100:
                        self._battery_pct = float(rem)
                elif mtype == "BATTERY_STATUS":
                    # voltages[0] = tegangan cell-1 (mV); voltage pack =
                    # jumlah semua cell yang valid (< 0xFFFF).
                    volts = getattr(msg, "voltages", None) or []
                    cells_mv = [v for v in volts
                                if v not in (0, 0xFFFF, 65535)]
                    if cells_mv:
                        self._voltage_v = sum(cells_mv) / 1000.0
                    curr = getattr(msg, "current_battery", -1)
                    if curr is not None and curr >= 0:
                        self._current_a = curr / 100.0
                    rem = getattr(msg, "battery_remaining", -1)
                    if rem is not None and 0 <= rem <= 100:
                        self._battery_pct = float(rem)
                elif mtype == "HEARTBEAT":
                    self.last_heartbeat_time = time.time()
                msg = self.master.recv_match(blocking=False)
        except Exception as exc:
            print(f"[MAV] Error poll: {exc}")

    # ------------------- properti hasil -------------------
    @property
    def has_attitude(self):
        return bool(self._attitude)

    @property
    def roll_deg(self):
        return math.degrees(self._attitude.get("roll", 0.0))

    @property
    def pitch_deg(self):
        return math.degrees(self._attitude.get("pitch", 0.0))

    @property
    def yaw_deg(self):
        return math.degrees(self._attitude.get("yaw", 0.0))

    @property
    def lat(self):
        return self._pos[0] if self._pos else None

    @property
    def lon(self):
        return self._pos[1] if self._pos else None

    @property
    def groundspeed(self):
        return self._groundspeed

    # ------------------- baterai -------------------
    # Baterai LiPO 4S penuh = 16.8 V, kosong = ~12.8 V (3.2 V/cell).
    # Baterai LiFePO4 4S penuh = 14.6 V, kosong = ~12.0 V (3.0 V/cell).
    # Persen dihitung linear dari tegangan bila firmware tidak mengirim
    # battery_remaining — cukup akurat untuk indikator GUI.
    LIPO_4S_FULL_V = 16.8
    LIPO_4S_EMPTY_V = 12.8

    @property
    def voltage_v(self):
        """Tegangan pack (V) atau None bila belum ada data."""
        return self._voltage_v

    @property
    def current_a(self):
        """Arus pack (A) atau None bila firmware tidak mengirim."""
        return self._current_a

    @property
    def battery_pct(self):
        """Sisa baterai 0..100 (%) atau None bila belum ada data."""
        if self._battery_pct is not None:
            return self._battery_pct
        if self._voltage_v is None:
            return None
        span = self.LIPO_4S_FULL_V - self.LIPO_4S_EMPTY_V
        pct = (self._voltage_v - self.LIPO_4S_EMPTY_V) / span * 100.0
        return max(0.0, min(100.0, pct))

    @property
    def has_battery(self):
        return self._voltage_v is not None

    @property
    def heartbeat_age_s(self):
        """Umur heartbeat terakhir (detik) — buat watchdog koneksi."""
        if self.last_heartbeat_time == 0.0:
            return float("inf")
        return time.time() - self.last_heartbeat_time