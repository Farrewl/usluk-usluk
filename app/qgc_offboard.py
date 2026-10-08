"""app/qgc_offboard.py — Terima MANUAL_CONTROL dari QGC, konversi ke (v, w).

Alur data (topologi via router):

    Xbox (laptop) -> QGC -> MAVLink (router UDP 14550/14540) -> Pixhawk
        -> companion computer (modul ini membaca koneksi pymavlink yang sama)

QGC meneruskan pesan MAVLink MANUAL_CONTROL ke vehicle. Modul ini:
  1. memantau paket tersebut (bukan pembuat setpoint — Pixhawk yang
     mengonsumsinya langsung saat mode MANUAL, servo output tetap di
     Pixhawk, companion TIDAK memegang ESC);
  2. menerjemahkan ke parameter gerak (v, w) = (surge, yaw) ternormalisasi
     [-1..1] untuk chip status GUI + arbitrasi sumber manual;
  3. failsafe lepas-kendali: tanpa paket segar selama
     config.QGC_MANUAL_TIMEOUT_S (0.5 dtk) -> output dipaksa netral
     (0,0) dan mode jatuh ke HOLD (hysteresis) sebelum AUTO.

Arbitrasi dua sumber manual (QGC vs lokal RC/gamepad) dihitung di C —
lihat core/arbitrator.c `arb_manual2` via aterkia_core.arb_manual2().

SIGINT/Ctrl+C: modul ini tidak memegang resource apa pun (socket dan
PWM dipegang proses lain); poll() non-blocking sehingga caller boleh
dihentikan kapan pun tanpa risiko thruster nyangkut di posisi basi.
"""

import time

try:
    from . import aterkia_core as core
except Exception:  # pragma: no cover - import parsial saat uji
    core = None

from .logutil import get_logger, throttled

log = get_logger()

# --- Rentang protocol MAVLink MANUAL_CONTROL (konstan protokol) ---
# x, y, r : int16 dalam [-127, 127] (rush/pitch, roll, yaw)
# z       : uint16 dalam [0, 1000], netral = tengah rentang [Hz-spek]
# Kapal permukaan memakai z -> surge (maju/mundur) dan r -> yaw (buritan).
Z_MIN = 0.0
Z_MAX = 1000.0
Z_NEUTRAL = (Z_MIN + Z_MAX) / 2.0   # 500 = stik throttle di tengah
R_FULL = 127.0                       # deviasi yaw stik penuh


def clamp1(v):
    """Kunci nilai ke rentang [-1.0, 1.0]."""
    if v < -1.0:
        return -1.0
    if v > 1.0:
        return 1.0
    return v


def convert_manual_control(msg):
    """MAVLink MANUAL_CONTROL -> (surge, yaw) masing-masing [-1, 1].

    surge = throttle (z): 0 mundur penuh, Z_NEUTRAL netral, 1000 maju penuh.
    yaw   = stik kanan (r): -127 kiri penuh, 0 lurus, +127 kanan penuh.
    x (rush) & y (roll) diabaikan — ASV permukaan hanya punya dua DOF.
    """
    try:
        z = float(getattr(msg, "z", Z_NEUTRAL))
        r = float(getattr(msg, "r", 0.0))
    except (TypeError, ValueError):
        return 0.0, 0.0
    if z < Z_MIN or z > Z_MAX:
        # Di luar rentang protocol (driver aneh) -> anggap netral, aman.
        return 0.0, 0.0
    surge = (z - Z_NEUTRAL) / (Z_MAX - Z_NEUTRAL)
    yaw = r / R_FULL
    return clamp1(surge), clamp1(yaw)


class QgcOffboard:
    """Pantau MANUAL_CONTROL dari QGC + hasilkan (v, w) siap arbitrasi.

    Pemakaian (tiap loop 20-25 Hz, pola sama dengan ManualLink):
        qgc = QgcOffboard(config)
        qgc.attach_mav(master)          # pymavlink connection (boleh None)
        out = qgc.poll(dt)              # dict surge/yaw/active/mode/...
        arb = core.arb_manual2(...)     # adu dengan sumber lokal di C
    """

    def __init__(self, config):
        self.config = config
        self.master = None
        self._mode = None                 # core.ModeState (dibuat malas)
        self._last_rx_mono = None         # monotonic waktu paket terakhir
        self._surge = 0.0
        self._yaw = 0.0
        self._clock = time.monotonic      # injeksi jam untuk tes
        self._last_out = {"surge": 0.0, "yaw": 0.0, "active": False,
                          "mode": 0, "mode_name": "AUTO", "rc_ok": False,
                          "source": "AUTO", "qgc_ok": False}

    def attach_mav(self, master):
        """Sambungkan koneksi pymavlink (boleh None = QGC tak terlihat)."""
        self.master = master

    def _drain(self):
        """Ambil semua paket MANUAL_CONTROL tertunda (non-blocking).

        Return pesan TERAKHIR (QGC kirim ~10-50 Hz; yang terbaru yang
        berarti), atau None bila tak ada satupun.
        """
        if self.master is None:
            return None
        latest = None
        try:
            while True:
                msg = self.master.recv_match(type="MANUAL_CONTROL",
                                             blocking=False)
                if msg is None:
                    break
                latest = msg
        except Exception:
            return latest
        return latest

    def poll(self, dt=0.05):
        """Perbarui state; return dict (pola sama dengan ManualLink.update).

        - Paket baru diterima  -> reset waktu + konversi (v, w).
        - Paket basi > timeout -> netral dipaksakan; mode -> HOLD/AUTO.
        - Tanpa koneksi MAVLink -> selalu netral/tidak aktif (aman).
        """
        if core is None or not core.C_AVAILABLE:
            return dict(self._last_out)
        if self._mode is None:
            self._mode = core.ModeState()

        msg = self._drain()
        now = self._clock()
        fresh = False
        if msg is not None:
            self._surge, self._yaw = convert_manual_control(msg)
            self._last_rx_mono = now
            fresh = True
            if throttled("qgc_rx", 5.0):
                log.info("QGC MANUAL_CONTROL diterima "
                         "(surge %+.2f yaw %+.2f)", self._surge, self._yaw)
        elif self._last_rx_mono is not None:
            timeout_s = float(getattr(self.config,
                                      "QGC_MANUAL_TIMEOUT_S", 0.5))
            fresh = (now - self._last_rx_mono) <= timeout_s
            if not fresh and throttled("qgc_lost", 5.0):
                log.warning("QGC putus > %.2f dtk -> netral (HOLD)",
                            timeout_s)

        # Paket basi -> output dipaksa netral; jangan bawa angka lama.
        surge = self._surge if fresh else 0.0
        yaw = self._yaw if fresh else 0.0

        # Mode (C): KILL > MANUAL > HOLD > AUTO — sama dengan ManualLink,
        # sehingga putus sesaat ditahan MANUAL_LOST_HOLD_S dulu (HOLD),
        # bukan langsung lompat ke AUTO yang bisa mengejutkan.
        mode = self._mode.update(
            kill=False, manual_req=fresh, rc_ok=fresh,
            dt=float(dt),
            lost_hold_s=float(self.config.MANUAL_LOST_HOLD_S))
        active = fresh and mode["mode"] == core.OP_MANUAL

        out = {"surge": float(surge), "yaw": float(yaw),
               "active": bool(active),
               "mode": int(mode["mode"]), "mode_name": mode["name"],
               "rc_ok": bool(fresh), "qgc_ok": bool(fresh),
               "source": "QGC" if active else mode["name"]}
        self._last_out = out
        return dict(out)
