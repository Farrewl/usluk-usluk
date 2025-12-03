import cv2
import time
import numpy as np
import os

# Konfigurasi Dummy
IDX_CAM_ASLI = 1  # Index kamera aslimu
SAVE_DIR = "hasil_tes_logika"
if not os.path.exists(SAVE_DIR): os.makedirs(SAVE_DIR)

def get_camera_frame_simulated(cam_obj, attempt_number):
    """
    Fungsi ini memalsukan perilaku kamera.
    - Percobaan 1: Kirim Gambar PUTIH (Simulasi Bug WP8)
    - Percobaan 2: Kirim Gambar HITAM (Simulasi Bug/Error)
    - Percobaan 3+: Kirim Gambar ASLI dari kamera
    """
    if attempt_number == 1:
        print("   [SIMULASI] Kamera mengirim frame PUTIH (Overexposed)...")
        # Buat gambar putih polos (255)
        return True, np.full((480, 640, 3), 255, dtype=np.uint8)
    
    elif attempt_number == 2:
        print("   [SIMULASI] Kamera mengirim frame HITAM (Underexposed)...")
        # Buat gambar hitam polos (0)
        return True, np.zeros((480, 640, 3), dtype=np.uint8)
    
    else:
        print("   [SIMULASI] Kamera mengirim frame ASLI (Normal)...")
        # Baca dari kamera beneran
        return cam_obj.read()

def tes_logika_smart_capture():
    print("\n=== MEMULAI TES VALIDASI LOGIKA SMART CAPTURE ===")
    
    # 1. Buka Kamera (Real)
    cam = cv2.VideoCapture(IDX_CAM_ASLI, cv2.CAP_DSHOW)
    if not cam.isOpened():
        print("Error: Kamera tidak terdeteksi.")
        return

    foto_sukses = False
    
    # 2. Loop Percobaan (Sama persis dengan logika di Navigator)
    for percobaan in range(1, 6):
        print(f"\n--- Percobaan ke-{percobaan} ---")
        
        # PANGGIL FUNGSI SIMULASI (Bukan cam.read() biasa)
        ret, frame = get_camera_frame_simulated(cam, percobaan)
        
        if ret:
            # --- LOGIKA YANG KITA UJI ---
            avg_brightness = np.mean(frame)
            print(f"   Brightness Terukur: {avg_brightness:.2f}")

            # Validasi: Tolak jika > 230 (Putih) atau < 5 (Hitam)
            if 5 < avg_brightness < 230:
                print("   [KEPUTUSAN] GAMBAR DITERIMA! (Valid)")
                filename = f"{SAVE_DIR}/foto_final_valid.jpg"
                cv2.imwrite(filename, frame)
                print(f"   Simpan ke: {filename}")
                foto_sukses = True
                break # Keluar loop
            else:
                print("   [KEPUTUSAN] GAMBAR DITOLAK! (Terlalu Terang/Gelap). Retrying...")
                time.sleep(1.0) # Simulasi jeda retry
        else:
            print("   Gagal baca frame.")

    if foto_sukses:
        print("\n=== HASIL: LOGIKA BERHASIL ===")
        print("Program berhasil melewati gangguan gambar Putih & Hitam,")
        print("dan akhirnya mengambil gambar Asli.")
    else:
        print("\n=== HASIL: LOGIKA GAGAL ===")
    
    cam.release()

if __name__ == "__main__":
    tes_logika_smart_capture()