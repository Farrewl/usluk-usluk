# =================================================================
# SKRIP 1: PEREKAM WAYPOINT INTERAKTIF (MODIFIKASI)
# =================================================================
# Deskripsi:
# Jalankan skrip ini di lapangan. Gerakkan kapal secara manual ke setiap
# titik yang Anda inginkan sebagai waypoint, lalu tekan 's' untuk menyimpan
# posisi saat ini. Setelah selesai, tekan 'q' untuk mencetak daftar waypoint
# yang siap disalin DAN menyimpannya ke plan.csv.
# =================================================================

import time
import keyboard
from pymavlink import mavutil
import os
import csv

# --- Konfigurasi Koneksi (SESUAIKAN) ---
SERIAL_PORT = 'COM8'
BAUD_RATE = 57600

def clear_screen():
    """Membersihkan layar terminal untuk tampilan yang lebih rapi."""
    os.system('cls' if os.name == 'nt' else 'clear')

def main():
    """Fungsi utama untuk merekam waypoint."""
    clear_screen()
    print(f"Menghubungkan ke PX4 di: {SERIAL_PORT}...")
    try:
        # Menghubungkan ke Pixhawk/flight controller
        master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD_RATE, autoreconnect=True)
        master.wait_heartbeat()
        print("Heartbeat diterima. Koneksi berhasil.")
    except Exception as e:
        print(f"FATAL: Gagal terhubung. Error: {e}")
        return

    waypoints = []
    current_lat, current_lon = 0.0, 0.0
    last_lat, last_lon = 0.0, 0.0
    
    # Variabel untuk melacak status penekanan 's'
    s_is_pressed = False 

    print("\n--- Perekam Waypoint Siap ---")
    print("Gerakkan kapal ke posisi yang diinginkan (mode Manual).")
    print("Tekan 's' untuk **SEKALI** MENYIMPAN waypoint posisi saat ini.")
    print("Tekan 'q' untuk SELESAI, mencetak hasil, dan menyimpan ke plan.csv.")

    try:
        while not keyboard.is_pressed('q'):
            # Ambil data GPS
            msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=1)
            
            # Cek apakah 's' sedang ditekan
            s_was_pressed = keyboard.is_pressed('s')

            if msg and msg.fix_type >= 3:
                current_lat = msg.lat / 1e7
                current_lon = msg.lon / 1e7

                # --- LOGIKA PENYIMPANAN WAYPOINT ---
                # Hanya simpan satu waypoint per penekanan 's'
                if s_was_pressed and not s_is_pressed:
                    # Simpan waypoint saat ini
                    waypoints.append({'lat': current_lat, 'lon': current_lon})
                    print(f"\n✅ Waypoint #{len(waypoints)} DISIMPAN! (Lat: {current_lat:.7f}, Lon: {current_lon:.7f})\n")
                    
                # Perbarui status penekanan 's'
                s_is_pressed = s_was_pressed
                
                # Hanya perbarui tampilan jika posisi berubah (untuk mengurangi flicker)
                if abs(current_lat - last_lat) > 1e-7 or abs(current_lon - last_lon) > 1e-7:
                    clear_screen()
                    print("--- Perekam Waypoint Aktif ---")
                    print("Tekan 's' untuk MENYIMPAN | Tekan 'q' untuk SELESAI\n")
                    print(f"Posisi Kapal Saat Ini:")
                    print(f"  Lat: {current_lat:.7f}")
                    print(f"  Lon: {current_lon:.7f}\n")
                    print(f"Waypoint Tersimpan: {len(waypoints)}")
                    for i, wp in enumerate(waypoints):
                        print(f"  {i+1}. Lat: {wp['lat']:.7f}, Lon: {wp['lon']:.7f}")
                    last_lat, last_lon = current_lat, current_lon
                    
            else:
                # Tampilkan pesan jika sinyal GPS buruk atau hilang
                if not keyboard.is_pressed('q'): # Hindari cetak saat hampir selesai
                     print("Menunggu sinyal GPS 3D Fix...", end='\r')
            
            time.sleep(0.1) # Jeda singkat untuk mencegah penggunaan CPU berlebihan

    except KeyboardInterrupt:
        print("\nProgram dihentikan.")
    finally:
        clear_screen()
        print("--- HASIL PEREKAMAN WAYPOINT ---")
        
        if not waypoints:
            print("Tidak ada waypoint yang disimpan.")
        else:
            # 1. Cetak format array Python
            print("Salin dan tempel blok kode di bawah ini ke skrip navigasi utama Anda:\n")
            print("WAYPOINTS = [")
            for i, wp in enumerate(waypoints):
                print(f"    {{'lat': {wp['lat']:.7f}, 'lon': {wp['lon']:.7f}, 'desc': 'Waypoint {i+1}'}},")
            print("]")
            
            print("\n" + "="*40 + "\n")
            
            # 2. Simpan ke plan.csv
            filename = "plan.csv"
            fields = ['lat', 'lon']
            try:
                with open(filename, mode='w', newline='', encoding='utf-8') as f:
                    writer = csv.DictWriter(f, fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(waypoints)
                print(f"✅ Semua waypoint berhasil disimpan ke file **{filename}**.")
            except Exception as e:
                 print(f"❌ Gagal menyimpan ke plan.csv. Error: {e}")
                 
        print("\nProgram perekam selesai.")

if __name__ == "__main__":
    main()