import cv2
import time
import os

# --- KONFIGURASI ---
INDEX_KAMERA = 1  # Kamera Bawah Air yang gelap
FOLDER_HASIL = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "..", "data", "camera_tests")

if not os.path.exists(FOLDER_HASIL):
    os.makedirs(FOLDER_HASIL)

print(f"--- TUNING KAMERA GELAP (INDEX {INDEX_KAMERA}) ---")

# Buka Kamera dengan DSHOW (karena ini yang terbukti jalan di Windows)
cap = cv2.VideoCapture(INDEX_KAMERA, cv2.CAP_DSHOW)

if not cap.isOpened():
    print("FATAL: Kamera tidak terdeteksi!")
    exit()

# Set Resolusi Aman (Sesuai temuan sebelumnya)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def ambil_foto(nama_file, ket):
    # WARM-UP LOOP (SANGAT PENTING)
    # Kamera butuh waktu untuk menerapkan settingan baru ke sensor fisik
    print(f"   -> Menerapkan: {ket} (Tunggu warm-up...)")
    for _ in range(30): # Buang 30 frame agar sensor adaptasi
        cap.read()
    
    ret, frame = cap.read()
    if ret:
        path = os.path.join(FOLDER_HASIL, nama_file)
        cv2.imwrite(path, frame)
        print(f"   [V] Foto disimpan: {path}")
    else:
        print(f"   [X] Gagal mengambil frame untuk {ket}")

# --- SKENARIO 1: DEFAULT (Gelap) ---
print("\n1. Tes Settingan DEFAULT")
ambil_foto("1_default.jpg", "Tanpa ubah setting")

# --- SKENARIO 2: PAKSA AUTO-EXPOSURE (Mode 3) ---
# Di backend DSHOW, Value 3 biasanya artinya 'Auto Mode' (Aperture Priority)
print("\n2. Tes AUTO EXPOSURE (Mode 3)")
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) 
ambil_foto("2_auto_exp_3.jpg", "Auto Exposure = 3")

# --- SKENARIO 3: PAKSA AUTO-EXPOSURE (Mode 1) ---
# Beberapa kamera drivernya aneh, Auto-nya ada di value 1
print("\n3. Tes AUTO EXPOSURE (Mode 1)")
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) 
ambil_foto("3_auto_exp_1.jpg", "Auto Exposure = 1")

# --- SKENARIO 4: DONGKRAK GAIN & BRIGHTNESS (Manual) ---
# Jika Auto gagal, kita paksa sensitivitas (Gain) ke angka tinggi
print("\n4. Tes MANUAL BOOST (Gain & Brightness Tinggi)")
# Kembalikan ke mode manual dulu (biasanya 0.25 atau 1)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
cap.set(cv2.CAP_PROP_GAIN, 200)       # Range biasanya 0-255
cap.set(cv2.CAP_PROP_BRIGHTNESS, 180) # Range biasanya 0-255
ambil_foto("4_manual_boost.jpg", "Gain 200, Brightness 180")

# --- SKENARIO 5: EXPOSURE TIME MANUAL (Rana Lama) ---
# Membuka rana lebih lama. Nilai -4 s/d -6 logaritmik
print("\n5. Tes MANUAL EXPOSURE (Rana Terbuka)")
cap.set(cv2.CAP_PROP_EXPOSURE, -5) 
ambil_foto("5_manual_exposure.jpg", "Exposure -5")

cap.release()
print("\n--- SELESAI ---")
print(f"Silakan buka folder '{FOLDER_HASIL}' dan beritahu saya foto nomor berapa yang TERANG.")