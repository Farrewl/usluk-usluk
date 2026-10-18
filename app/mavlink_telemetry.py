"""
app/mavlink_telemetry.py — Baca telemetri MAVLink dari Pixhawk (PX4/ArduPilot).

Tujuan: memberi GUI data attitude (roll, pitch, yaw) dan posisi NYATA dari
Pixhawk tanpa harus menjalankan navigator penuh (yang butuh YOLO + model .pt).

Koneksi (skill asv-mavlink):
  - Port dipilih dari pyserial (hanya port yang BENAR-BENAR ada). Urutan:
    preferred -> port ber-deskripsi autopilot (Auterion/PX4/Pixhawk/
    ArduPilot) -> /dev/ttyACM* -> /dev/ttyUSB* -> COM* (Windows).
    Index ACM TIDAK bisa diandalkan: Pixhawk 6C di mesin uji bisa muncul
    sebagai /dev/ttyACM1, bukan ACM0 — karena itu deskripsi USB dipakai.
  - Baud 57600 standar; lewat USB baud diabaikan driver CDC.
  - Port serial USB hanya bisa dipegang SATU proses. Bila QGroundControl /
    mavlink-router masih membukanya, connect() melaporkan "SIBUK".
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
except ImportError:
    mavutil = None
    MAVLINK_AVAILABLE = False

try:
    from serial.tools import list_ports as _list_ports
    _LIST_PORTS_OK = True
except ImportError:
    _list_ports = None
    _LIST_PORTS_OK = False







GCS_DEFAULT_IP = "127.0.0.1"
GCS_DEFAULT_PORT = 14550

# Deskripsi/hwid USB yang menandakan autopilot (dipakai mengurutkan port).
_AUTOPILOT_HINTS = (
    "px4", "pixhawk", "auterion", "ardupilot", "3d robotics",
    "holybro", "mro", "cubepilot", "cube",
)
# Port onboard/legacy yang tak relevan untuk Pixhawk (jangan dituju).
_SKIP_PREFIXES = ("/dev/ttyS", "/dev/ttyAMA", "/dev/pts")


def _enumerate():
    """[(device, description)] — port serial USB nyata di mesin ini.

    Pakai pyserial (akurat di Windows: cuma COM yang benar-benar ada,
    bukan COM3..9 tebakan). Saring port onboard (ttyS/ttAMA) yang tak
    pernah jadi Pixhawk. Bila pyserial tak ada, fallback ke glob
    /dev/ttyACM* + /dev/ttyUSB*.
    """
    out = []
    seen = set()
    if _LIST_PORTS_OK:
        for p in _list_ports.comports():
            dev = p.device
            if dev.startswith(_SKIP_PREFIXES) or dev in seen:
                continue
            seen.add(dev)
            desc = " ".join(x for x in (p.description, p.manufacturer,
                                        p.hwid) if x)
            out.append((dev, desc))
        if out:
            return out
    for pat in ("/dev/ttyACM*", "/dev/ttyUSB*"):
        for dev in sorted(glob.glob(pat)):
            if dev not in seen:
                seen.add(dev)
                out.append((dev, ""))
    return out


def _autopilot_score(desc):
    """1 bila deskripsi/hwid mengandung nama autopilot, else 0."""
    d = (desc or "").lower()
    return 1 if any(h in d for h in _AUTOPILOT_HINTS) else 0


def detect_serial_ports(preferred=None):
    """Daftar port serial kandidat (untuk dropdown), terurut prioritas.

    Prioritas: `preferred` (bila ada) -> port ber-deskripsi autopilot
    (Auterion/PX4/Pixhawk/ArduPilot) -> /dev/ttyACM* -> /dev/ttyUSB* ->
    COM* lain. Diambil dari pyserial sehingga Windows hanya menawarkan
    COM yang benar-benar terpasang. Return list[str] (bisa kosong).
    """
    entries = _enumerate()
    devices = [d for d, _ in entries]
    score = {d: _autopilot_score(dc) for d, dc in entries}

    def _key(dev):
        is_acm = 0 if ("ttyACM" in dev or dev.upper().startswith("COM")) else 1
        return (-score.get(dev, 0), is_acm, dev)

    ports = []
    if preferred and (os.path.exists(preferred) or preferred in devices):
        ports.append(preferred)
    for dev in sorted(devices, key=_key):
        if dev not in ports:
            ports.append(dev)
    if not ports and os.name == "nt" and preferred:
        ports.append(preferred)
    return ports


def detect_serial_port(preferred=None):
    """Pilih SATU port serial Pixhawk yang benar-benar ada.

    Urutan: `preferred` (bila ada di sistem) -> port ber-deskripsi autopilot
    -> /dev/ttyACM* -> /dev/ttyUSB* -> COM* (Windows). Bila tidak ada yang
    eksis: Windows mengembalikan `preferred` (driver membukanya saat koneksi),
    Linux mengembalikan None.
    """
    devices = {d for d, _ in _enumerate()}
    for dev in detect_serial_ports(preferred):
        if dev in devices or os.path.exists(dev):
            return dev
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
        self.last_error = ""
        self.busy = False



        self._gcs = None
        self._gcs_addr = (GCS_DEFAULT_IP, GCS_DEFAULT_PORT)
        self._gcs_enabled = False
        self._attitude = {}
        self._pos = None
        self._groundspeed = 0.0




        self._voltage_v = None
        self._current_a = None
        self._battery_pct = None
        self._flight_mode_id = None


    def connect(self, timeout_s=8.0):
        """Buka serial + tunggu heartbeat. True bila Pixhawk merespons."""
        self.last_error = ""
        self.busy = False
        if not MAVLINK_AVAILABLE:
            self.last_error = "no-pymavlink"
            print("[MAV] pymavlink tidak terpasang — telemetri dimatikan.")
            return False
        if self.port is None:
            self.last_error = "no-port"
            print("[MAV] Tidak ada port serial ditemukan.")
            return False

        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
        except Exception as exc:
            self.last_error = str(exc)
            errno = getattr(exc, "errno", None)
            if errno == 16 or "busy" in str(exc).lower():
                self.busy = True
                print(f"[MAV] Port {self.port} SIBUK — kemungkinan "
                      "QGroundControl / mavlink-router masih memakainya. "
                      "Tutup QGC dulu; app akan retry otomatis.")
            else:
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
        self.last_error = "no-heartbeat"
        print(f"[MAV] Tidak ada heartbeat dari {self.port} "
              f"dalam {timeout_s:.0f}s.")
        return False

    def close(self):
        if self.master is not None:
            self.master.close()
        self.master = None
        self.connected = False
        if self._gcs is not None:
            try:
                self._gcs.close()
            except Exception:
                pass
            self._gcs = None
        self._gcs_enabled = False


    def enable_gcs_forward(self, on=True, ip=None, port=None):
        """Nyalakan/matikan forward UDP ke QGC (default 127.0.0.1:14550).

        Dipanggil SETELAH connect() sukses. GCS membaca heartbeat +
        telemetri yang sama (Pixhawk tetap 1 master = NUC). QGC bisa
        Arm/Mode/E-stop; otonomi tetap dari NUC.
        """
        if ip is not None or port is not None:
            self._gcs_addr = (ip or self._gcs_addr[0],
                              int(port or self._gcs_addr[1]))
        self._gcs_enabled = bool(on)
        if not on and self._gcs is not None:
            try:
                self._gcs.close()
            except Exception:
                pass
            self._gcs = None

    @property
    def gcs_active(self):
        """True bila socket GCS terbuka & siap forward."""
        return bool(self._gcs_enabled and self._gcs is not None)

    def _ensure_gcs(self):
        """Buka socket udp-out ke QGC secara malas (sekali saja)."""
        if not self._gcs_enabled or not MAVLINK_AVAILABLE:
            return
        if self._gcs is not None:
            return
        try:
            ip, prt = self._gcs_addr
            self._gcs = mavutil.mavlink_connection(
                f"udpout:{ip}:{prt}", source_system=1,
                source_component=191)
            print(f"[MAV] Forward QGC aktif -> udp:{ip}:{prt}.")
        except Exception as exc:
            print(f"[MAV] Gagal buka forward QGC: {exc}")
            self._gcs = None


    def _request_streams(self):
        """Minta attitude/posisi mengalir (10 Hz) — PX4 & ArduPilot."""



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
                pass



        for name in ("ATTITUDE", "GLOBAL_POSITION_INT", "VFR_HUD",
                     "SYS_STATUS", "BATTERY_STATUS", "RC_CHANNELS"):
            self._set_message_interval(name, 100_000)

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
            pass


    def poll(self):
        """Baca SEMUA pesan yang mengantre; perbarui state internal.

        Bila GCS forward aktif, setiap pesan mentah ikut diteruskan ke
        QGC via UDP (tanpa parsing ulang — murah di CPU).
        """
        if not self.connected or self.master is None:
            return
        if self._gcs_enabled:
            self._ensure_gcs()
        try:
            msg = self.master.recv_match(blocking=False)
            while msg is not None:
                if self._gcs is not None:
                    try:
                        self._gcs.write(msg.get_msgbuf())
                    except Exception:
                        pass
                mtype = msg.get_type()
                if mtype == "ATTITUDE":
                    self._attitude = {
                        "roll": msg.roll, "pitch": msg.pitch, "yaw": msg.yaw,
                    }
                elif mtype == "GLOBAL_POSITION_INT":

                    if msg.lat != 0 or msg.lon != 0:
                        self._pos = (msg.lat / 1e7, msg.lon / 1e7)
                elif mtype == "VFR_HUD":
                    self._groundspeed = msg.groundspeed
                elif mtype == "SYS_STATUS":



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
                    try:
                        self._flight_mode_id = int(
                            getattr(msg, "custom_mode", 0) or 0)
                    except Exception:
                        pass
                    if self._gcs is not None:


                        now_hb = time.time()
                        if now_hb - getattr(self, "_last_gcs_hb", 0.0) >= 1.0:
                            self._last_gcs_hb = now_hb
                            try:
                                self._gcs.mav.heartbeat_send(
                                    mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, 0)
                            except Exception:
                                pass
                msg = self.master.recv_match(blocking=False)
        except Exception as exc:
            print(f"[MAV] Error poll: {exc}")


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






    LIPO_4S_FULL_V = 16.8
    LIPO_4S_EMPTY_V = 12.8

    @property
    def voltage_v(self):
        """Tegangan pack (V) atau None bila belum ada data."""
        return self._voltage_v

    @property
    def flight_mode_id(self):
        """custom_mode ArduRover terakhir dari HEARTBEAT, atau None."""
        return self._flight_mode_id

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