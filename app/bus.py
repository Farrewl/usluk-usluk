"""app/bus.py — slot 1 paket telemetri (pengganti pyqtSignal).

Paket lama ditimpa; tidak ada antrean yang tumbuh saat GUI/websocket lambat.
"""
import threading


class LatestPacket:
    def __init__(self):
        self._cond = threading.Condition()
        self._pkt = None

    def put(self, pkt):
        with self._cond:
            self._pkt = pkt
            self._cond.notify_all()

    def get(self, timeout=None):
        """Blok sampai ada paket baru; None kalau timeout."""
        with self._cond:
            self._cond.wait_for(lambda: self._pkt is not None, timeout)
            pkt, self._pkt = self._pkt, None
            return pkt
