"""ASV entry point — headless otonom, telemetri & kontrol lewat WebSocket.

Tidak ada GUI (PyQt sudah dibuang). Jalur:
  navigator thread (visi + MAVLink + C core)  ->  LatestPacket  ->  WebSocket

Mode navigasi: perilaku visi tanpa waypoint (`app/behavior.py`), jadi kapal
bisa jalan otonom di lokasi mana pun. Kalau `WAYPOINTS` diisi, jalur
waypoint lama tetap dipakai.
"""
import asyncio
import logging
import os
import signal
import sys
import threading

from app.navigator import NavigatorThread

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s %(levelname)s %(message)s")
log = logging.getLogger(__name__)

# Kosong = mode perilaku visi (tanpa path planning).
WAYPOINTS = []
WS_HOST = os.environ.get("ASV_WS_HOST", "0.0.0.0")
WS_PORT = int(os.environ.get("ASV_WS_PORT", "8765"))


def build_runtime(waypoints=None):
    return NavigatorThread(list(WAYPOINTS if waypoints is None else waypoints))


def main():
    worker = build_runtime()
    worker.start()

    stop = threading.Event()

    def _stop(*_a):
        stop.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, _stop)

    from app.ws_server import serve
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    task = loop.create_task(serve(worker.newData, worker, WS_HOST, WS_PORT))

    try:
        while not stop.is_set():
            loop.run_until_complete(asyncio.sleep(0.2))
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        task.cancel()
        worker.stop()
        worker.wait(5000)
        log.info("runtime berhenti bersih")


if __name__ == "__main__":
    sys.exit(main())
