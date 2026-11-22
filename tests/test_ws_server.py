"""tests/test_ws_server.py — Unit test untuk server WebSocket lokal."""
import asyncio
import json
import unittest
import websockets

from app.bus import LatestPacket
from app.ws_server import serve


class FakeWorker:
    def __init__(self):
        self.params = {}
        self.stopped = False

    def update_config_param(self, key, value):
        self.params[key] = value

    def stop(self):
        self.stopped = True


class TestWsServer(unittest.TestCase):
    def test_ws_server_handshake(self):
        async def run_test():
            slot = LatestPacket()
            worker = FakeWorker()

            # Start server di port random/unused (8766)
            server_task = asyncio.create_task(serve(slot, worker, host="127.0.0.1", port=8766))
            await asyncio.sleep(0.1)  # Biarkan server bind

            # Push 1 paket telemetri dummy
            slot.put({"status": "OK", "voltage_v": 12.5, "video_due": False})

            # Connect client
            async with websockets.connect("ws://127.0.0.1:8766") as ws:
                raw = await asyncio.wait_for(ws.recv(), timeout=1.0)
                pkt = json.loads(raw)
                self.assertEqual(pkt.get("status"), "OK")
                self.assertEqual(pkt.get("voltage_v"), 12.5)

                # Kirim perintah set param
                await ws.send(json.dumps({"cmd": "set", "key": "THRUST_VALUE", "value": 0.9}))
                await asyncio.sleep(0.05)
                self.assertEqual(worker.params.get("THRUST_VALUE"), 0.9)

            server_task.cancel()

        asyncio.run(run_test())


if __name__ == "__main__":
    unittest.main()
