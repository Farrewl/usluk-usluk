"""app/safety.py — Failsafe otomatis (stale-link / low-batt) untuk navigator.

Menyatukan logika yang dulu terduplikasi di `navigator.py` + `simulator.py`:
evaluasi kondisi tak-aman tiap frame, lalu (bila aktif) minta thrust nol +
RTL best-effort lewat `OutputLink`. Ambang semua di `app/settings.py`
(tab Failsafe), tanpa magic number di sini.

Pemicu:
  1. stale-link — tak ada ATTITUDE/GLOBAL_POSITION selama
     `FAILSAFE_TELEM_TIMEOUT_S`;
  2. low-batt — persen ATAU tegangan di bawah ambang, DITAHAN selama
     `FAILSAFE_LOW_BATT_HOLD_S` agar spike sesaat (arus dud) tak memicu
     RTL palsu.

Pulih otomatis bila kondisi normal kembali (log 1x).
"""

import time

from .logutil import get_logger

log = get_logger()


class SafetyManager:
    """State failsafe + keputusan; aksi (RTL) didelegasikan ke OutputLink."""

    def __init__(self, config, link=None):
        self.config = config
        self.link = link
        self.failsafe_active = False
        self.failsafe_reason = ""
        self._lowbatt_since = None
        self._last_telem_mono = 0.0

    def attach_link(self, link):
        """Sambungkan OutputLink (dipakai untuk perintah RTL)."""
        self.link = link

    def mark_telem(self):
        """Cap waktu telemetri segar (dipanggil tiap ATTITUDE/POSITION tiba)."""
        self._last_telem_mono = time.monotonic()

    def reset(self):
        self.failsafe_active = False
        self.failsafe_reason = ""
        self._lowbatt_since = None
        self._last_telem_mono = 0.0

    def check(self, battery_pct, voltage_v):
        """Evaluasi failsafe. Return "" bila aman, atau string alasan.

        Caller MENIMPA gerak (thrust 0, yaw tetap) lalu menampilkan status
        "FAILSAFE (alasan)" bila return bukan "".
        """
        if not getattr(self.config, "FAILSAFE_ENABLED", True):
            if self.failsafe_active:
                self.failsafe_active = False
                self.failsafe_reason = ""
            return ""

        now = time.monotonic()
        reason = ""

        # 1. Stale-link.
        timeout = float(getattr(self.config, "FAILSAFE_TELEM_TIMEOUT_S", 2.0))
        last = float(getattr(self, "_last_telem_mono", 0.0) or 0.0)
        if last > 0.0 and (now - last) > timeout:
            reason = f"telemetri basi {now - last:.1f}s"

        # 2. Baterai rendah (ditahan agar spike sesaat tidak memicu).
        if not reason:
            low_pct = float(getattr(self.config, "FAILSAFE_LOW_BATT_PCT", 20.0))
            low_v = float(getattr(self.config, "FAILSAFE_LOW_VOLT_V", 13.2))
            hold = float(getattr(self.config, "FAILSAFE_LOW_BATT_HOLD_S", 3.0))
            low = ((battery_pct is not None and battery_pct < low_pct)
                   or (voltage_v is not None and voltage_v < low_v))
            if low:
                if self._lowbatt_since is None:
                    self._lowbatt_since = now
                elif now - self._lowbatt_since >= hold:
                    detail = (f"{battery_pct:.0f}%" if battery_pct is not None
                              else f"{voltage_v:.1f}V")
                    reason = f"baterai rendah {detail}"
            else:
                self._lowbatt_since = None

        if reason:
            if not self.failsafe_active:
                log.warning("FAILSAFE AKTIF: %s — thrust 0 + coba RTL.", reason)
            self.failsafe_active = True
            self.failsafe_reason = reason
            if self.link is not None:
                self.link.request_rtl()
            return reason

        if self.failsafe_active:
            log.info("FAILSAFE pulih: kondisi normal kembali.")
        self.failsafe_active = False
        self.failsafe_reason = ""
        return ""
