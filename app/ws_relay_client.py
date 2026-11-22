"""app/ws_relay_client.py — Dial-out ke Relay VPS (modem seluler biasanya CGNAT).

Kapal menyambung ke wss://relay.domain.com. Browser juga menyambung ke sana.
Menerapkan protokol envelope JSON + binary media per PROMPT_AGENT_ASV.md 5.2.
"""
import asyncio
import json
import time
import cv2
import base64

from .logutil import get_logger, throttled

log = get_logger()


def _encode_frame(frame, quality=60):
    if frame is None:
        return None
    try:
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        # Media biner per spec 5.2: "Bukan base64". 
        # Tapi untuk tahap awal JSON bridge, kita bisa pakai base64 atau kirim biner terpisah.
        # Kita ikuti spec: biner (JPEG) diawali header.
        return buf.tobytes()
    except Exception:
        return None


async def run_relay_client(slot, worker, relay_url, token):
    """Loop dial-out ke Relay dengan exponential backoff."""
    import websockets

    retry_delay = 1.0
    boat_id = "usluk-usluk-01"

    while True:
        try:
            async with websockets.connect(relay_url) as ws:
                log.info("Terhubung ke Relay: %s", relay_url)
                retry_delay = 1.0
                
                # Hello handshake
                await ws.send(json.dumps({
                    "v": 1, "kind": "hello", "seq": 0, "t_unix": time.time(),
                    "boat_id": boat_id, "role": "publisher", "token": token
                }))

                async def send_telemetry():
                    while True:
                        pkt = await slot.get(timeout=0.2)
                        if pkt is None:
                            continue
                        
                        # 1. Kirim Telemetry (JSON)
                        out = {
                            "v": 1, "kind": "telemetry", "seq": int(time.time()*10),
                            "t_unix": time.time(), "boat_id": boat_id
                        }
                        # Filter out frames from JSON
                        out.update({k: v for k, v in pkt.items() 
                                    if k not in ("frame", "frame_secondary")})
                        await ws.send(json.dumps(out))

                        # 2. Kirim Media (Biner) jika video_due
                        if pkt.get("video_due") and pkt.get("frame") is not None:
                            jpeg = _encode_frame(pkt["frame"])
                            if jpeg:
                                # Header: 1 byte channel (0=nav) + 4 byte seq + 8 byte ts
                                # Untuk kemudahan awal, kita kirim biner murni jika relay mendukung.
                                # Spec 5.2: "Bukan base64".
                                header = b'\x00' + int(time.time()).to_bytes(4, 'big') + int(time.time()*1000).to_bytes(8, 'big')
                                await ws.send(header + jpeg)

                async def receive_commands():
                    async for raw in ws:
                        try:
                            msg = json.loads(raw)
                            if msg.get("kind") == "params_set":
                                # Whitelist & validation logic in task 5.3
                                worker.update_config_param(msg["key"], msg["value"])
                                await ws.send(json.dumps({
                                    "v": 1, "kind": "params_ack", "req_id": msg.get("req_id"),
                                    "ok": True, "applied": msg["value"]
                                }))
                        except Exception as e:
                            log.error("Relay cmd error: %s", e)

                await asyncio.gather(send_telemetry(), receive_commands())

        except Exception as e:
            log.error("Relay connection lost: %s. Retrying in %.1fs", e, retry_delay)
            await asyncio.sleep(retry_delay)
            retry_delay = min(retry_delay * 2, 60.0)
