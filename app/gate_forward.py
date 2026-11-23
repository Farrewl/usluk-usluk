"""app/gate_forward.py — Modul simpel Python untuk mendeteksi buoy merah & hijau bersamaan lalu maju.

Cara pakai:
    from app.gate_forward import check_and_forward
    surge, yaw, note = check_and_forward(detections, frame_width)
"""

def check_and_forward(detections, frame_width=640, red_class_id=1, green_class_id=0):
    """Mendeteksi buoy merah dan hijau secara bersamaan di frame.
    
    Returns:
        (surge, yaw, note): tuple perintah thruster dan keterangan.
    """
    reds = detections.get(red_class_id, [])
    greens = detections.get(green_class_id, [])

    # Jika tidak ada kedua jenis buoy, jangan intervensi (kembalikan None)
    if not reds or not greens:
        return 0.0, 0.0, "No gate pair"

    # Cari buoy merah paling tengah/dekat dan hijau paling tengah/dekat
    center_x = frame_width / 2.0
    
    # Ambil rata-rata posisi X buoy merah dan hijau
    red_x = sum(b["cx"] for b in reds) / len(reds)
    green_x = sum(b["cx"] for b in greens) / len(greens)

    # Titik tengah gerbang (gate midpoint)
    gate_mid_x = (red_x + green_x) / 2.0

    # Hitung error arah dari tengah kamera (normalized -1.0 s.d 1.0)
    error_px = gate_mid_x - center_x
    yaw = max(-1.0, min(1.0, error_px / (frame_width / 2.0) * 0.5))

    # Karena kedua buoy terdeteksi bersamaan (artinya gerbang di depan), maju lurus!
    surge = 0.6  # Maju dengan kecepatan 60%

    return surge, yaw, f"GATE DETECTED (Red & Green) -> Forward surge={surge}, yaw={yaw:.2f}"
