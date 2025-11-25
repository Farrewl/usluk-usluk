import time
import keyboard
import os
import csv
import math
import random

# --- KONFIGURASI SIMULASI (TANPA MAVLINK) ---
OBJECT_FILENAME = "objects_simulation.csv"

# Konfigurasi lintasan FIXED
COURSE_HEADING = 45.0  # ⬅️ ORIENTASI LINTASAN TETAP (derajat)
LANE_WIDTH = 1.5       # Jarak antara buoy merah-hijau
LANE_SPACING = 5.0     # Jarak antara set lintasan

# Posisi simulasi awal (contoh: sekitar Jakarta)
SIMULATION_START_LAT = -6.175392
SIMULATION_START_LON = 106.827153

# Optimisasi
DISPLAY_UPDATE_INTERVAL = 0.2
DEBUG_MODE = True

# ----------------- FUNGSI BANTUAN -----------------

def clear_screen():
    """Membersihkan layar terminal."""
    os.system('cls' if os.name == 'nt' else 'clear')

def calculate_destination_point(lat, lon, bearing, distance):
    """
    Menghitung titik tujuan dari posisi awal menggunakan Haversine.
    """
    EARTH_RADIUS = 6371000.0  # meters
    
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)
    angular_distance = distance / EARTH_RADIUS

    lat_dest_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance) +
        math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad)
    )

    lon_dest_rad = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(lat_dest_rad)
    )

    return math.degrees(lat_dest_rad), math.degrees(lon_dest_rad)

def create_lane_set(center_lat, center_lon, course_heading, lane_width, set_number):
    """
    Buat satu set lintasan (3 pasang buoy) dengan orientasi FIXED
    """
    objects = []
    
    # Arah tegak lurus lintasan (untuk buoy kiri-kanan)
    perpendicular = (course_heading + 90) % 360
    
    # Buat 3 pasang buoy dengan spacing tetap
    for i in range(3):
        # Posisi sepanjang lintasan utama
        along_track_distance = i * LANE_SPACING
        lane_center_lat, lane_center_lon = calculate_destination_point(
            center_lat, center_lon, course_heading, along_track_distance
        )
        
        # Buoy hijau (kanan) - searah perpendicular
        green_lat, green_lon = calculate_destination_point(
            lane_center_lat, lane_center_lon, perpendicular, lane_width / 2
        )
        
        # Buoy merah (kiri) - berlawanan perpendicular
        red_lat, red_lon = calculate_destination_point(
            lane_center_lat, lane_center_lon, (perpendicular + 180) % 360, lane_width / 2
        )
        
        # Tambahkan ke objects
        objects.extend([
            {'lat': green_lat, 'lon': green_lon, 'type': 'green_buoy', 'round': set_number},
            {'lat': red_lat, 'lon': red_lon, 'type': 'red_buoy', 'round': set_number}
        ])
    
    return objects

def read_simulated_gps():
    """
    SIMULASI: Membaca data GPS palsu untuk testing
    """
    # Simulasi posisi dengan sedikit random movement
    base_lat = SIMULATION_START_LAT + random.uniform(-0.0001, 0.0001)
    base_lon = SIMULATION_START_LON + random.uniform(-0.0001, 0.0001)
    heading = random.uniform(0, 360)
    
    return {
        'lat': base_lat,
        'lon': base_lon,
        'heading': heading,
        'gps_valid': True,
        'heading_valid': True,
        'fix_type': 3,
        'satellites': 12
    }

# ----------------- FUNGSI UTAMA (TANPA MAVLINK) -----------------

def main():
    clear_screen()
    print("🚤 PEREKAM OBJEK LINTASAN - MODE SIMULASI 🚤")
    print("=" * 60)
    print("📡 STATUS: SIMULASI GPS (Tidak terkoneksi MAVLink)")
    print(f"🎯 ORIENTASI LINTASAN: {COURSE_HEADING}°")
    print("=" * 60)

    # State variables
    objects = []
    current_set = 1
    current_lat, current_lon = SIMULATION_START_LAT, SIMULATION_START_LON
    current_heading = COURSE_HEADING
    gps_valid, heading_valid = True, True
    
    # Keyboard state tracking
    key_state = {
        'r': False, 'g': False, 'b': False,  # Single buoys
        'p': False,  # Pair
        's': False,  # Full set (3 pairs)
        'n': False,  # Next set
        'up': False, 'down': False, 'left': False, 'right': False  # Navigation
    }
    
    # Display throttling
    last_display_time = 0
    
    print("\n🎮 KONTROL SIMULASI:")
    print("   [R] Merah  [G] Hijau  [B] Kotak  [P] Pair  [S] Full Set")
    print("   [N] Next Set  [↑↓←→] Pindah Posisi  [Q] Quit & Save")
    print("\n⏳ Program berjalan...")

    try:
        loop_count = 0
        
        while not keyboard.is_pressed('q'):
            loop_start = time.time()
            loop_count += 1
            
            # ============================================
            # FASE 1: BACA DATA GPS SIMULASI
            # ============================================
            simulated_data = read_simulated_gps()
            
            # Update state dari simulasi
            current_lat = simulated_data['lat']
            current_lon = simulated_data['lon']
            current_heading = simulated_data['heading']
            gps_valid = simulated_data['gps_valid']
            heading_valid = simulated_data['heading_valid']
            
            # ============================================
            # FASE 2: NAVIGASI MANUAL (SIMULASI GERAKAN)
            # ============================================
            move_distance = 0.0001  # ~11 meter per tekan
            
            if keyboard.is_pressed('up'):
                current_lat, current_lon = calculate_destination_point(
                    current_lat, current_lon, COURSE_HEADING, move_distance
                )
            if keyboard.is_pressed('down'):
                current_lat, current_lon = calculate_destination_point(
                    current_lat, current_lon, (COURSE_HEADING + 180) % 360, move_distance
                )
            if keyboard.is_pressed('left'):
                current_lat, current_lon = calculate_destination_point(
                    current_lat, current_lon, (COURSE_HEADING - 90) % 360, move_distance
                )
            if keyboard.is_pressed('right'):
                current_lat, current_lon = calculate_destination_point(
                    current_lat, current_lon, (COURSE_HEADING + 90) % 360, move_distance
                )
            
            # ============================================
            # FASE 3: PROSES KEYBOARD INPUT (OBJECT PLACEMENT)
            # ============================================
            action_taken = False
            
            for key in ['r', 'g', 'b', 'p', 's', 'n']:
                is_pressed = keyboard.is_pressed(key)
                was_pressed = key_state[key]
                
                # Edge detection - hanya trigger saat pertama ditekan
                if is_pressed and not was_pressed:
                    
                    if key in ['r', 'g', 'b']:  # Single buoy
                        obj_type = {'r': 'red_buoy', 'g': 'green_buoy', 'b': 'box'}[key]
                        obj_icon = {'r': '🔴', 'g': '🟢', 'b': '📦'}[key]
                        
                        objects.append({
                            'lat': current_lat, 
                            'lon': current_lon, 
                            'type': obj_type, 
                            'round': current_set
                        })
                        print(f"\n{obj_icon} {obj_type} | Set {current_set}")
                        action_taken = True
                    
                    elif key == 'p':  # PAIR - Buat sepasang buoy
                        # Buat sepasang buoy dengan orientasi FIXED
                        perpendicular = (COURSE_HEADING + 90) % 360
                        
                        # Buoy hijau (kanan)
                        green_lat, green_lon = calculate_destination_point(
                            current_lat, current_lon, perpendicular, LANE_WIDTH / 2
                        )
                        
                        # Buoy merah (kiri)  
                        red_lat, red_lon = calculate_destination_point(
                            current_lat, current_lon, (perpendicular + 180) % 360, LANE_WIDTH / 2
                        )
                        
                        objects.extend([
                            {'lat': green_lat, 'lon': green_lon, 'type': 'green_buoy', 'round': current_set},
                            {'lat': red_lat, 'lon': red_lon, 'type': 'red_buoy', 'round': current_set}
                        ])
                        
                        print(f"\n✅ PAIR | Orientasi:{COURSE_HEADING}° | Set {current_set}")
                        action_taken = True
                    
                    elif key == 's':  # FULL SET - Buat 3 pasang buoy
                        set_objects = create_lane_set(
                            current_lat, current_lon, COURSE_HEADING, LANE_WIDTH, current_set
                        )
                        objects.extend(set_objects)
                        print(f"\n⭐ SET LENGKAP! | 3 pasang | Orientasi:{COURSE_HEADING}° | Set {current_set}")
                        action_taken = True
                    
                    elif key == 'n':  # NEXT SET
                        current_set += 1
                        print(f"\n🚩 >>> SET {current_set} <<<")
                        action_taken = True
                
                # Update key state
                key_state[key] = is_pressed
            
            # ============================================
            # FASE 4: UPDATE DISPLAY
            # ============================================
            current_time = time.time()
            time_since_update = current_time - last_display_time
            
            if action_taken or time_since_update >= DISPLAY_UPDATE_INTERVAL:
                clear_screen()
                
                print("╔═══════════════════════════════════════════════════════════════╗")
                print("║          🚤 PEREKAM OBJEK LINTASAN - SIMULASI 🚤             ║")
                print("╚═══════════════════════════════════════════════════════════════╝")
                
                print("\n🎮 KONTROL:")
                print("   [R]Merah [G]Hijau [B]Kotak [P]Pair [S]Full Set(3 pairs)")
                print("   [N]Next Set  [↑↓←→]Pindah Posisi  [Q]Quit & Save")
                
                print(f"\n🏁 SET: {current_set}  |  📦 TOTAL OBJEK: {len(objects)}")
                print("─" * 65)
                
                # Status simulasi
                print(f"\n📍 POSISI SIMULASI:")
                print(f"   {current_lat:.7f}°, {current_lon:.7f}°")
                print(f"   Heading: {current_heading:.1f}°")
                print(f"   Orientasi Lintasan: {COURSE_HEADING}°")
                
                # Show last 5 objects
                if objects:
                    print(f"\n📋 OBJEK TERAKHIR (5/{len(objects)}):")
                    for obj in objects[-5:]:
                        icon = "🔴" if obj['type'] == 'red_buoy' else ("🟢" if obj['type'] == 'green_buoy' else "📦")
                        print(f"   {icon} {obj['type']:12s} S{obj['round']} | {obj['lat']:.6f}")
                else:
                    print("\n📋 Belum ada objek")
                
                print("\n" + "─" * 65)
                elapsed = time.time() - loop_start
                hz = 1 / elapsed if elapsed > 0 else 0
                print(f"Loop: {loop_count} | Rate: {hz:.0f} Hz | Mode: SIMULASI")
                
                last_display_time = current_time
            
            # ============================================
            # FASE 5: SLEEP MINIMAL
            # ============================================
            elapsed = time.time() - loop_start
            sleep_time = max(0.05 - elapsed, 0.001)  # 20 Hz max
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n⚠️  Program diinterupsi")
        
    finally:
        # ============================================
        # SAVE & SUMMARY
        # ============================================
        clear_screen()
        print("╔═══════════════════════════════════════════════════════════════╗")
        print("║                    📊 HASIL AKHIR SIMULASI 📊                ║")
        print("╚═══════════════════════════════════════════════════════════════╝\n")
        
        if not objects:
            print("❌ Tidak ada data tersimpan\n")
        else:
            try:
                with open(OBJECT_FILENAME, 'w', newline='', encoding='utf-8') as f:
                    writer = csv.DictWriter(f, fieldnames=['lat', 'lon', 'type', 'round'])
                    writer.writeheader()
                    writer.writerows(objects)
                
                print(f"✅ {len(objects)} objek disimpan ke: {OBJECT_FILENAME}\n")
                
                print("📊 RINGKASAN PER SET:")
                print("─" * 65)
                
                sets_summary = {}
                for obj in objects:
                    s = obj['round']
                    if s not in sets_summary:
                        sets_summary[s] = {'red_buoy': 0, 'green_buoy': 0, 'box': 0}
                    sets_summary[s][obj['type']] += 1
                
                for s in sorted(sets_summary.keys()):
                    data = sets_summary[s]
                    total = data['red_buoy'] + data['green_buoy'] + data['box']
                    print(f"   S{s}: 🔴{data['red_buoy']}  🟢{data['green_buoy']}  📦{data['box']}  (Total: {total})")
                
                print("─" * 65)
                
            except Exception as e:
                print(f"❌ Gagal menyimpan: {e}\n")
        
        print("\n🏁 Simulasi selesai!")
        print("   Untuk versi dengan MAVLink, ganti fungsi read_simulated_gps()")
        print("   dengan read_all_mavlink_messages(master)\n")

if __name__ == "__main__":
    main()
