import cv2
import time
import os
import numpy as np

# ================= KONFIGURASI =================
IDX_CAM_NAV = 0       # Sesuaikan dengan index kamera navigasimu
IDX_CAM_FOTO = 1      # Sesuaikan dengan index kamera bawah airmu
RES_WIDTH = 640
RES_HEIGHT = 480
DIR_OUTPUT = "hasil_tes_kamera"
# ===============================================

if not os.path.exists(DIR_OUTPUT):
    os.makedirs(DIR_OUTPUT)

def analisis_kecerahan(image, label):
    """Cek apakah gambar putih total (overexposed/blank)"""
    if image is None: return
    avg_color_per_row = np.average(image, axis=0)
    avg_color = np.average(avg_color_per_row, axis=0)
    brightness = np.mean(avg_color)
    
    print(f"   [{label}] Rata-rata Brightness: {brightness:.2f} (0=Gelap, 255=Putih Total)")
    if brightness > 250:
        print(f"   [WARNING] Gambar terdeteksi PUTIH TOTAL (Blank White) pada {label}!")
    elif brightness < 5:
        print(f"   [WARNING] Gambar terdeteksi HITAM TOTAL pada {label}!")

def test_scenario_1_conflict():
    print("\n=== SKENARIO 1: KONFLIK (Keduanya Aktif Bersamaan) ===")
    print("1. Membuka Kamera Navigasi...")
    cap_nav = cv2.VideoCapture(IDX_CAM_NAV, cv2.CAP_DSHOW)
    cap_nav.set(cv2.CAP_PROP_FRAME_WIDTH, RES_WIDTH)
    cap_nav.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_HEIGHT)
    
    # Baca beberapa frame kamera navigasi
    for i in range(10):
        ret, frame = cap_nav.read()
    
    if cap_nav.isOpened():
        print("   -> Kamera Navigasi AKTIF (Streaming...)")
    else:
        print("   -> Gagal membuka Kamera Navigasi!")
        return

    print("2. Membuka Kamera Foto (TANPA menutup Navigasi)...")
    try:
        cap_foto = cv2.VideoCapture(IDX_CAM_FOTO, cv2.CAP_DSHOW)
        
        # Coba paksa baca
        print("   -> Mencoba membaca frame dari Kamera Foto...")
        ret_foto, frame_foto = cap_foto.read()
        
        filename = os.path.join(DIR_OUTPUT, "skenario1_konflik.jpg")
        if ret_foto:
            cv2.imwrite(filename, frame_foto)
            print(f"   -> Frame TERSIMPAN: {filename}")
            analisis_kecerahan(frame_foto, "Skenario 1")
        else:
            print("   -> GAGAL membaca frame (Ret=False). Kemungkinan Bandwidth Penuh.")
            
        cap_foto.release()
        
    except Exception as e:
        print(f"   -> ERROR Exception: {e}")

    print("3. Menutup Kamera Navigasi...")
    cap_nav.release()
    print("=== Selesai Skenario 1 ===\n")

def test_scenario_2_interleaved():
    print("\n=== SKENARIO 2: BERSELANG-SELING (Switching) ===")
    
    # --- PHASE 1: NAVIGASI ---
    print("1. [PHASE NAV] Membuka Kamera Navigasi...")
    cap_nav = cv2.VideoCapture(IDX_CAM_NAV, cv2.CAP_DSHOW)
    cap_nav.set(cv2.CAP_PROP_FRAME_WIDTH, RES_WIDTH)
    cap_nav.set(cv2.CAP_PROP_FRAME_HEIGHT, RES_HEIGHT)
    
    for i in range(5): cap_nav.read() # Simulasi jalan
    print("   -> Kamera Navigasi berjalan...")

    # --- PHASE 2: PERSIAPAN FOTO ---
    print("2. [SWITCH] Mematikan Kamera Navigasi...")
    cap_nav.release()
    
    print("3. [WAIT] Istirahat 1 detik (Voltage Stabilizing)...")
    time.sleep(1.0) # Jeda PENTING

    # --- PHASE 3: AMBIL FOTO ---
    print("4. [PHASE FOTO] Membuka Kamera Foto...")
    cap_foto = cv2.VideoCapture(IDX_CAM_FOTO, cv2.CAP_DSHOW)
    
    # Warmup kecil untuk Auto-Exposure kamera foto
    for i in range(5): cap_foto.read() 
    
    ret_foto, frame_foto = cap_foto.read()
    filename = os.path.join(DIR_OUTPUT, "skenario2_switch.jpg")
    
    if ret_foto:
        cv2.imwrite(filename, frame_foto)
        print(f"   -> Frame TERSIMPAN: {filename}")
        analisis_kecerahan(frame_foto, "Skenario 2")
    else:
        print("   -> GAGAL membaca frame foto.")
    
    print("5. [SWITCH] Mematikan Kamera Foto...")
    cap_foto.release()

    # --- PHASE 4: KEMBALI NAVIGASI ---
    print("6. [PHASE NAV] Menyalakan KEMBALI Kamera Navigasi...")
    cap_nav = cv2.VideoCapture(IDX_CAM_NAV, cv2.CAP_DSHOW)
    if cap_nav.isOpened():
        ret, frame = cap_nav.read()
        if ret:
            print("   -> SUKSES: Kamera Navigasi hidup kembali.")
        else:
            print("   -> WARNING: Kamera Navigasi terbuka tapi frame kosong.")
    else:
        print("   -> ERROR: Gagal membuka kembali kamera navigasi.")
    
    cap_nav.release()
    print("=== Selesai Skenario 2 ===\n")

if __name__ == "__main__":
    print("--- ALAT TES DIAGNOSA KAMERA ASV ---")
    print(f"Output akan disimpan di folder: {DIR_OUTPUT}")
    
    while True:
        print("\nPilih Mode Tes:")
        print("1. Tes KONFLIK (Simulasi Bug saat ini)")
        print("2. Tes SWITCHING (Simulasi Solusi)")
        print("3. Keluar")
        pilihan = input("Masukkan pilihan (1/2/3): ")

        if pilihan == '1':
            test_scenario_1_conflict()
        elif pilihan == '2':
            test_scenario_2_interleaved()
        elif pilihan == '3':
            break
        else:
            print("Pilihan tidak valid.")