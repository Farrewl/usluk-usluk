import asyncio
import websockets
import redis.asyncio as redis
import redis.exceptions as redis_exceptions
import json

# === KONFIGURASI ===
NGROK_HOST = "dashboardaterolas.app" # SESUAIKAN INI

TELEMETRY_URI = f"wss://{NGROK_HOST}/ws/telemetry"
VISION_URI = f"wss://{NGROK_HOST}/ws/vision_control" 

REDIS_HOST = "localhost"
REDIS_PORT = 6379

TELEMETRY_CHANNEL = "asv_telemetry"
VISION_CHANNEL = "asv_vision"
MISSION_CHANNEL = "asv_mission"
# ===================

async def relay_redis_to_websocket(redis_channel, websocket_uri):
    """
    Satu fungsi generik untuk mendengarkan satu channel Redis
    dan meneruskannya ke satu endpoint WebSocket.
    """
    print(f"Memulai relay untuk {redis_channel} -> {websocket_uri}")

    while True:
        try:
            # 1. Hubungkan ke Redis
            r = await redis.from_url(f"redis://{REDIS_HOST}:{REDIS_PORT}")
            pubsub = r.pubsub()
            await pubsub.subscribe(redis_channel)
            print(f"Berhasil subscribe ke Redis channel: {redis_channel}")

            # 2. Hubungkan ke WebSocket (ngrok/VPS)
            async with websockets.connect(websocket_uri) as websocket:
                print(f"Berhasil terhubung ke WebSocket: {websocket_uri}")
                
                # --- PERUBAHAN UTAMA DI SINI ---
                # 3. Loop utama: Gunakan 'listen' alih-alih 'get_message'
                # Ini adalah cara yang lebih andal untuk pub/sub asinkron
                print(f"Memasuki loop 'listen' untuk {redis_channel}...")
                async for message in pubsub.listen():
                    
                    # Cek apakah ini pesan data (bukan pesan subscribe)
                    if message['type'] == 'message':
                        # Teruskan data mentah (sudah JSON) ke web
                        await websocket.send(message['data'])
                    elif message['type'] == 'subscribe':
                        print(f"Berhasil subscribe (loop listen) ke {message['channel']}")
                # --- AKHIR PERUBAHAN ---

        # --- BLOK EXCEPT ---
        except redis_exceptions.ConnectionError as e:
            print(f"Koneksi Redis gagal (Apakah server Redis sudah jalan?): {e}. Menyambung ulang...")
        except websockets.ConnectionClosedError as e: 
            print(f"WebSocket terputus ({websocket_uri}): {e}. Menyambung ulang...")
        except ConnectionRefusedError as e:
            print(f"Koneksi WebSocket ditolak ({websocket_uri}): {e}. Menyambung ulang...")
        except Exception as e:
            print(f"Error tidak diketahui di relay ({redis_channel}): {e}")
        
        await asyncio.sleep(3) # Tunggu 3 detik sebelum menyambung ulang

async def main():
    """Menjalankan ketiga tugas relay secara bersamaan."""
    
    task_telemetry = asyncio.create_task(
        relay_redis_to_websocket(TELEMETRY_CHANNEL, TELEMETRY_URI)
    )
    task_vision = asyncio.create_task(
        relay_redis_to_websocket(VISION_CHANNEL, VISION_URI)
    )
    task_mission = asyncio.create_task(
        relay_redis_to_websocket(MISSION_CHANNEL, VISION_URI) 
    )
    
    await asyncio.gather(task_telemetry, task_vision, task_mission)

if __name__ == "__main__":
    print("Memulai Web Gateway (Reporter)...")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nGateway dihentikan.")