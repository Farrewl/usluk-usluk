"""app/behavior.py — navigasi perilaku TANPA waypoint / path planning.

Kapal menentukan jalannya sendiri dari deteksi visi:
  GATE: cari pasangan buoy (merah+hijau) -> arah tengah gap -> lewati.
  BOX : cari kotak hijau -> arah tengah -> maju sampai dekat -> foto.
  DOCK: cari kotak biru (kamera bawah) -> maju pelan sampai dock.

Tidak ada lintasan, tidak ada GPS: perilaku sama di tempat lomba mana pun.
Modul ini murni logika (tanpa kamera/MAVLink) agar bisa diuji cepat.
"""

GATE, BOX, DOCK = "GATE", "BOX", "DOCK"


def pair_gate(red_balls, green_balls, frame_h, min_gap_px=20):
    """Pasangan buoy merah+hijau: tinggi mirip, gap terdekat."""
    best = None
    for r in red_balls:
        for g in green_balls:
            if abs(r["cy"] - g["cy"]) > 0.5 * max(frame_h, 1):
                continue
            gap = abs(r["cx"] - g["cx"])
            if gap < min_gap_px:
                continue
            score = gap
            if best is None or score < best[0]:
                best = (score, r, g)
    if best is None:
        return None
    return best[1], best[2]


def decide(phase, detections, frame_w, frame_h, cfg):
    """Keputusan satu frame -> (target_x_px, thrust, note).

    target_x_px: titik yang harus dikejar di sumbu X (None = putar mencari).
    thrust     : -1..1 (maju positif).
    """
    cx = frame_w / 2.0

    if phase == GATE:
        red = detections.get(int(cfg.RED_BALL_CLASS_ID), [])
        green = detections.get(int(cfg.GREEN_BALL_CLASS_ID), [])
        red = [b for b in red if b["area"] >= cfg.MIN_BUOY_AREA_PX]
        green = [b for b in green if b["area"] >= cfg.MIN_BUOY_AREA_PX]
        pair = pair_gate(red, green, frame_h)
        if pair is not None:
            r, g = pair
            gap_px = abs(r["cx"] - g["cx"])
            # Gap lebar -> masih jauh; gap menyempit -> makin dekat.
            ratio = gap_px / frame_w
            if ratio > 0.35:
                thrust = cfg.SEARCH_THRUST
                note = "GATE far"
            else:
                thrust = cfg.THRUST_VALUE
                note = "GATE pass"
            return (r["cx"] + g["cx"]) / 2.0, thrust, note
        if red or green:
            # Satu buoy terlihat: putar ke arah buoy sambil maju pelan.
            b = max((red or green), key=lambda x: x["area"])
            return None, cfg.SEARCH_THRUST * 0.5, "GATE single-buoy"
        return None, 0.0, "GATE search"

    if phase == BOX:
        boxes = detections.get(int(getattr(cfg, "GREEN_BOX_CLASS_ID", 0)), [])
        boxes = [b for b in boxes
                 if b["area"] >= getattr(cfg, "MIN_BUOY_AREA_PX", 16)]
        if boxes:
            b = max(boxes, key=lambda x: x["area"])
            # Kotak menutupi sebagian besar frame -> dekat, berhenti.
            if b["area"] >= 0.25 * frame_w * frame_h:
                return b["cx"], 0.0, "BOX arrived"
            return b["cx"], cfg.ALIGN_THRUST, "BOX align"
        return None, 0.0, "BOX search"

    # DOCK
    boxes = detections.get(int(getattr(cfg, "BLUE_BOX_CLASS_ID", 0)), [])
    if boxes:
        b = max(boxes, key=lambda x: x["area"])
        if b["area"] >= 0.35 * frame_w * frame_h:
            return b["cx"], 0.0, "DOCK arrived"
        return b["cx"], cfg.BLUE_BOX_ALIGN_THRUST, "DOCK approach"
    return None, 0.0, "DOCK search"


def self_test():
    """Cek cepat logika inti (tanpa kamera)."""
    fw, fh = 640, 480

    class C:
        RED_BALL_CLASS_ID = 1
        GREEN_BALL_CLASS_ID = 0
        GREEN_BOX_CLASS_ID = 0
        BLUE_BOX_CLASS_ID = 0
        MIN_BUOY_AREA_PX = 16
        SEARCH_THRUST = 0.5
        THRUST_VALUE = 0.8
        ALIGN_THRUST = 0.3
        BLUE_BOX_ALIGN_THRUST = 0.3

    cfg = C()
    # Kosong -> putar di tempat
    x, t, note = decide(GATE, {}, fw, fh, cfg)
    assert x is None and t == 0.0 and note == "GATE search", (x, t, note)
    # Dua buoy kiri tengah -> target di antara, maju
    det = {1: [{"cx": 100.0, "cy": 240.0, "area": 500}],
           0: [{"cx": 400.0, "cy": 240.0, "area": 500}]}
    x, t, note = decide(GATE, det, fw, fh, cfg)
    assert abs(x - 250.0) < 1.0 and t == 0.5, (x, t, note)  # gap 0.47 > 0.35
    # Gap kecil -> THRUST_VALUE
    det = {1: [{"cx": 300.0, "cy": 240.0, "area": 900}],
           0: [{"cx": 340.0, "cy": 240.0, "area": 900}]}
    x, t, note = decide(GATE, det, fw, fh, cfg)
    assert t == cfg.THRUST_VALUE and abs(x - 320.0) < 1.0, (x, t, note)
    # Kotak besar -> berhenti
    det = {0: [{"cx": 320.0, "cy": 240.0, "area": 0.3 * fw * fh}]}
    x, t, note = decide(BOX, det, fw, fh, cfg)
    assert t == 0.0 and note == "BOX arrived", (x, t, note)
    # Satu buoy -> putar (target None), thrust pelan
    det = {1: [{"cx": 320.0, "cy": 240.0, "area": 500}]}
    x, t, note = decide(GATE, det, fw, fh, cfg)
    assert x is None and t == cfg.SEARCH_THRUST * 0.5, (x, t, note)
    return True


if __name__ == "__main__":
    print("behavior self-test:", self_test())
