"""
app/detection_validation.py — Filter pasca-YOLO khusus buoy (bola).

YOLO kadang mendeteksi objek mirip bola (mis. wajah operator di hadapan
kamera uji darat) sebagai buoy merah (class 1) / hijau (class 0) dengan
confidence tinggi. Filter ini memakai sifat fisik buoy untuk menolak
false-positive:

  1. Warna pekat: merah (hue ~[0..10] U [170..179]) atau hijau
     (hue ~[35..85]) pada domain HSV, dengan saturasi >= batas.
  2. Bentuk: rasio aspek dekat bujursangkar (bola) — wajah/objek lonjong
     ditolak.
  3. Ukuran: luas kotak >= area minimum (noise kecil ditolak).

Pertimbangan pemilihan batas (kalibrasi terhadap kamera uji 1280x720):
  - Wajah diambil dari jarak ~0,5 m punya box besar dan warna kulit
    (hue ~10-30, saturasi sedang) -> gagal kategori warna merah ATAU hijau
    selama fraksi warna cocok dihitung per-class dengan baik.
  - Buoy asli (bola mengambang) hampir bujur sangkar serta warnanya pekat
    di hampir seluruh areanya -> lolos.

Semua fungsi di sini murni (tanpa state) supaya mudah diuji lewat unit
test — kasus sintetis ada di tests/test_detection_validation.py.
"""

import cv2
import numpy as np

# Batas hue (OpenCV: 0..179) untuk warna buoy.
RED_HUE_MAX = 10        # merah "rendah": hue 0..10
RED_HUE_MIN2 = 170      # merah "tinggi":  hue 170..179
GREEN_HUE_MIN = 35      # hijau: hue 35..85
GREEN_HUE_MAX = 85
MIN_VALUE = 40          # value HSV di bawah ini = hitam/pudar, tak dihitung


def _is_red_hue(hue):
    """True untuk piksel hue merah (dua pita: rendah & tinggi)."""
    return (hue <= RED_HUE_MAX) | (hue >= RED_HUE_MIN2)


def _is_green_hue(hue):
    """True untuk piksel hue hijau."""
    return (hue >= GREEN_HUE_MIN) & (hue <= GREEN_HUE_MAX)


def _hue_ok_for_class(cls, hue):
    """Mask hue yang cocok dengan class buoy; None bila class tak dikenal.

    Konvensi model gate: class 0 = buoy HIJAU, class 1 = buoy MERAH.
    """
    if cls == 0:
        return _is_green_hue(hue)
    if cls == 1:
        return _is_red_hue(hue)
    return None


def color_fraction(frame, cls, xyxy, min_saturation):
    """Fraksi piksel dalam kotak deteksi yang warnanya cocok dengan `cls`.

    Crop diambil dari kotak, dikonversi ke HSV; piksel hitam (value
    rendah) dikeluarkan agar kotak yang menutupi latar gelap tidak
    terhitung. Return float 0..1 terhadap luas kotak.
    """
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]
    hue_ok = _hue_ok_for_class(cls, hue)
    if hue_ok is None:
        return 0.0
    sat_min = int(min_saturation * 255.0)
    mask = hue_ok & (sat >= sat_min) & (val >= MIN_VALUE)
    return float(mask.mean())


def validate_buoy(frame, cls, xyxy, min_area,
                  min_color_fraction=0.05,
                  min_saturation=0.35,
                  max_aspect_deviation=0.50,
                  debug=False):
    """Loloskan kotak deteksi hanya bila benar-benar mirip buoy.

    Kembalikan True bila SEMUA syarat terpenuhi:
      * luas kotak >= min_area (default 16 px² = 4×4, biarkan buoy jauh lolos),
      * |w/h - 1| <= max_aspect_deviation (bentuk mendekati bujur sangkar),
      * fraksi warna yang cocok >= min_color_fraction.
        Untuk box kecil (< 80 px²) ambang warna dibagi 2 (statistik tidak stabil).
    Class di luar 0/1 selalu ditolak (caller menentukan class mana yang
    menjalani validasi — fungsi ini sendiri aman dipanggil untuk apa pun).

    Default batas sengaja LONGGAR (conf 0.35 / fraksi warna 0.05 / saturasi
    0.35 / aspek 0.50 / area 16) supaya buoy ASLI tetap lolos walau lighting
    buruk; wajah operator tetap tertolak karena hue kulit adalah oranye/kuning,
    bukan merah ATAU hijau.

    saat `debug=True`, kembalikan (ok: bool, reasons: list[str]) dengan
    nilai terukur tiap kriteria agar mudah dicetak di test-live
    (scripts/test_deteksi.py --debug).
    """
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    w = x2 - x1
    h = y2 - y1
    reasons = []

    area = w * h
    if w <= 2 or h <= 2:
        if debug:
            reasons.append(f"kotak terlalu kecil (w={w}, h={h} px) < 2 px")
            return False, reasons
        return False
    if area < int(min_area):
        if debug:
            reasons.append(f"area {area} px < MIN_BUOY_AREA_PX({min_area:.0f})")
            return False, reasons
        return False

    aspect = w / h
    if abs(aspect - 1.0) > max_aspect_deviation:
        if debug:
            reasons.append(
                f"aspek {aspect:.2f} menyimpang > {max_aspect_deviation:.2f} "
                f"(bukan bentuk bola/bujur sangkar)")
            return False, reasons
        return False

    # Box kecil (jauh) → statistik warna tidak stabil → turunkan ambang 2×
    is_small = area < 80
    eff_color_frac = min_color_fraction * (0.5 if is_small else 1.0)

    frac = color_fraction(frame, cls, xyxy, min_saturation)
    if frac < eff_color_frac:
        if debug:
            reasons.append(
                f"fraksi warna cocok {frac:.3f} < {eff_color_frac:.3f} "
                f"(warna bukan {('hijau' if cls == 0 else 'merah')} pekat)"
                f"{' [box kecil]' if is_small else ''}")
            return False, reasons
        return False

    if debug:
        reasons.append(f"LOLOS (area {area} px, aspek {aspect:.2f}, "
                       f"warna {frac:.3f}{' [box kecil]' if is_small else ''})")
        return True, reasons
    return True


def draw_validated_boxes(frame, kept, names):
    """Gambar kotak untuk deteksi yang LULUS validasi (helper tampilan).

    `kept` = list berisi tuple (cls, conf, xyxy). Warna: hijau untuk
    buoy hijau, merah untuk buoy merah, cyan untuk deteksi lain.
    Dipakai menggantikan `results[0].plot()` agar wajah (yang ditolak
    validasi) tidak pernah tampil sebagai buoy di layar.
    """
    annotated = frame.copy()
    for cls, conf, xyxy in kept:
        x1, y1, x2, y2 = [int(v) for v in xyxy]
        if cls == 0:
            bgr = (0, 255, 0)
        elif cls == 1:
            bgr = (0, 0, 255)
        else:
            bgr = (255, 255, 0)
        cv2.rectangle(annotated, (x1, y1), (x2, y2), bgr, 2)
        label = f"{names.get(cls, str(cls))} {conf:.2f}"
        cv2.putText(annotated, label, (x1, max(15, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, bgr, 1, cv2.LINE_AA)
    return annotated