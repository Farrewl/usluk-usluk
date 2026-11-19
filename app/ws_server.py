"""app/ws_server.py — telemetri & kontrol lewat WebSocket (tanpa GUI).

Client terhubung ke ws://<ip>:8765 lalu menerima JSON telemetri (5 Hz).
Frame kamera dikirim sebagai JPEG base64 (dibatasi VIDEO_GUI_HZ).
Kirim perintah JSON ke server untuk mengubah parameter / start-stop.
"""
import asyncio
import base64
import json

import cv2

from .logutil import get_logger

log = get_logger()


def _encode_frame(frame, quality=60):
    if frame is None:
        return None
    try:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return base64.b64encode(buf.tobytes()).decode("ascii")
    except Exception:
        return None


async def _client(ws, slot, worker):
    while True:
        pkt = await slot.get(timeout=0.2)
        if pkt is None:
            continue
        out = {k: v for k, v in pkt.items()
               if k not in ("frame", "frame_secondary")}
        frame = pkt.get("frame")
        if frame is not None and pkt.get("video_due"):
            out["frame_jpeg"] = _encode_frame(frame)
        try:
            await asyncio.wait_for(ws.send(json.dumps(out)), timeout=1.0)
        except Exception:
            return


async def _commands(ws, worker):
    """Terima perintah: {"cmd":"set","key":"...","value":...} / start / stop."""
    while True:
        try:
            raw = await ws.recv()
        except Exception:
            return
        try:
            msg = json.loads(raw)
            cmd = msg.get("cmd")
            if cmd == "set":
                worker.update_config_param(msg["key"], msg["value"])
            elif cmd == "stop":
                worker.stop()
            elif cmd == "start":
                worker.navigator.auto_started = True
            else:
                await ws.send(json.dumps({"error": "unknown cmd", "cmd": cmd}))
        except Exception as e:
            await ws.send(json.dumps({"error": str(e)}))


async def serve(slot, worker, host="0.0.0.0", port=8765):
    """Jalankan server WebSocket sampai asyncio loop dihentikan."""
    try:
        import websockets
    except ImportError:
        log.error("websockets belum terpasang: pip install websockets")
        return

    async def handler(ws):
        log.info("WebSocket client: %s", ws.remote_address)
        sender = asyncio.create_task(_client(ws, slot, worker))
        receiver = asyncio.create_task(_commands(ws, worker))
        done, pending = await asyncio.wait(
            [sender, receiver], return_when=asyncio.FIRST_COMPLETED)
        for t in pending:
            t.cancel()

    async with websockets.serve(handler, host, port):
        log.info("WebSocket telemetry listen %s:%s", host, port)
        await asyncio.Future()
