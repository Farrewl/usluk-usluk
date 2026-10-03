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

from . import aterkia_core as core

# Batas hue (OpenCV: 0..179) untuk warna buoy.
RED_HUE_MAX = 10        # merah "rendah": hue 0..10
RED_HUE_MIN2 = 170      # merah "tinggi":  hue 170..179
GREEN_HUE_MIN = 35      # hijau: hue 35..85
GREEN_HUE_MAX = 85
MIN_VALUE = 40          # value HSV di bawah ini = hitam/pudar, tak dihitung

# P4-A: ukuran thumbnail untuk ukur kecerahan global (64x64 = 4096 piksel,
# ~0.5% biaya satu frame 720p — nyaris gratis, dihitung 1x per frame).
BRIGHTNESS_THUMB_PX = 64

# Batas tepi sisi minimum — cerminan ambah di C (gv_buoy_fail). Dipakai HANYA
# untuk menyusun pesan debug, bukan untuk memutuskan.
DEBUG_MIN_SIDE_PX = 2

# Ambang "kotak kecil" (fraksi warna dibagi 2). Ini keputusan NYATA pada
# jalur warna (Python), jadi konstanta biasa — bukan debug.
SMALL_BOX_AREA_PX = 80


def _buoy_geometry_reason(fail, w, h, area, min_area, max_aspect_deviation):
    """Pesan debug untuk penolakan geometri, dibangun dari kode C.

    `fail` = nilai balik core.buoy_geometry_fail_c (1/2/3). Angka turunan
    (area/aspek) hanya untuk ditampilkan, tidak ikut memutuskan apa pun.
    """
    if fail == 1:
        return f"kotak terlalu kecil (w={w}, h={h} px) < {DEBUG_MIN_SIDE_PX} px"
    if fail == 2:
        return f"area {area} px < MIN_BUOY_AREA_PX({min_area:.0f})"
    if fail == 3:
        return (f"aspek {w / h:.2f} menyimpang > {max_aspect_deviation:.2f} "
                f"(bukan bentuk bola/bujur sangkar)")
    return f"geometri ditolak (kode {fail})"


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


def color_fraction(frame, cls, xyxy, min_saturation, min_value=MIN_VALUE):
    """Fraksi piksel dalam kotak deteksi yang warnanya cocok dengan `cls`.

    Crop diambil dari kotak, dikonversi ke HSV; piksel hitam (value
    rendah) dikeluarkan agar kotak yang menutupi latar gelap tidak
    terhitung. Return float 0..1 terhadap luas kotak.

    `min_value` default = MIN_VALUE (40) — kompatibel dengan pemanggil
    lama; P4-A mengoper nilai yang lebih longgar saat frame gelap.
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
    mask = hue_ok & (sat >= sat_min) & (val >= int(min_value))
    return float(mask.mean())


def frame_brightness(frame):
    """Kecerahan global frame: mean channel V (0..255) dari thumbnail.

    Dihitung dari thumbnail 64x64 (1x per frame, ~4096 piksel) — cukup
    untuk memutuskan terang/gelap tanpa biaya berarti. Dipakai P4-A
    (adaptive threshold) dan calon P4-B (preprocessing saat gelap).
    """
    if frame is None or getattr(frame, "size", 0) == 0:
        return 255.0  # frame rusak -> anggap terang (jalur normal, aman)
    small = cv2.resize(frame, (BRIGHTNESS_THUMB_PX, BRIGHTNESS_THUMB_PX),
                       interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    return float(hsv[:, :, 2].mean())


def adaptive_thresholds(brightness, brightness_threshold=80,
                        base_saturation=0.35,
                        dark_saturation=0.20,
                        color_fraction_mult=0.5):
    """Ambang HSV efektif untuk kecerahan `brightness` (P4-A).

    Aturan: terang (>= threshold) -> ambang normal (siang, wajah tetap
    tertolak); gelap (< threshold) -> saturasi turun ke `dark_saturation`
    dan fraksi warna dikali `color_fraction_mult`. Interpolasi linear
    halus di zona gelap supaya transisi sore->malam tak melompat:

      sat_eff  = base - (base - dark) * (1 - brightness/threshold)
      frac_mult = 1 - (1 - mult) * (1 - brightness/threshold)

    Return (sat_eff, frac_mult, is_dark). MIN_VALUE adaptif dihitung
    pemanggil: gelap penuh -> 20, terang -> 40 (linear, sama rumusnya).
    """
    if brightness >= brightness_threshold:
        return base_saturation, 1.0, False
    gelap = 1.0 - (brightness / max(brightness_threshold, 1.0))
    sat_eff = base_saturation - (base_saturation - dark_saturation) * gelap
    frac_mult = 1.0 - (1.0 - color_fraction_mult) * gelap
    return sat_eff, frac_mult, True


def adaptive_min_value(brightness, brightness_threshold=80,
                       base_value=MIN_VALUE, dark_value=20):
    """Batas value HSV efektif (P4-A): 40 saat terang -> 20 saat gelap."""
    if brightness >= brightness_threshold:
        return base_value
    gelap = 1.0 - (brightness / max(brightness_threshold, 1.0))
    return base_value - (base_value - dark_value) * gelap


def validate_buoy(frame, cls, xyxy, min_area,
                  min_color_fraction=0.05,
                  min_saturation=0.35,
                  max_aspect_deviation=0.50,
                  debug=False, min_value=MIN_VALUE, brightness=None,
                  adaptive_enabled=False, brightness_threshold=80,
                  dark_saturation=0.20, color_fraction_mult=0.5):
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

    P4-A (adaptif gelap): bila `adaptive_enabled=True`, kecerahan frame
    diukur 1x (`brightness` = hasil frame_brightness, atau dihitung bila
    None) lalu ambang saturasi/value/fraksi dilonggarkan otomatis saat
    gelap (lihat adaptive_thresholds). Siang hari: perilaku IDENTIK dengan
    jalur lama (is_dark=False -> ambang utuh).

    Catatan arsitektur: kriteria GEOMETRI (w/h, area, rasio aspek) diputuskan
    di C (gv_buoy_fail via app/aterkia_core.py) — rumus tunggal, tanpa
    duplikat Python. Yang tetap di sini hanya nilai yang butuh citra: konversi
    HSV + fraksi warna (butuh OpenCV). Bila .so belum di-build, pemanggil
    gagal eksplisit (tidak ada fallback diam-diam ke rumus Python).

    saat `debug=True`, kembalikan (ok: bool, reasons: list[str]) dengan
    nilai terukur tiap kriteria agar mudah dicetak di test-live
    (scripts/test_deteksi.py --debug).
    """
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    w = x2 - x1
    h = y2 - y1
    reasons = []

    area = w * h
    # Kode penolakan geometri dari C (0 = lolos). Sumber kebenaran tunggal.
    fail = core.buoy_geometry_fail_c(w, h, min_area, max_aspect_deviation)
    if fail != 0:
        if debug:
            reasons.append(_buoy_geometry_reason(fail, w, h, area, min_area,
                                                max_aspect_deviation))
            return False, reasons
        return False

    eff_saturation = min_saturation
    eff_value = min_value
    # Box kecil (jauh) → statistik warna tidak stabil → turunkan ambang 2×
    is_small = area < SMALL_BOX_AREA_PX
    eff_color_frac = min_color_fraction * (0.5 if is_small else 1.0)
    gelap_info = ""
    if adaptive_enabled:
        bright = frame_brightness(frame) if brightness is None else brightness
        sat_eff, frac_mult, is_dark = adaptive_thresholds(
            bright, brightness_threshold, min_saturation, dark_saturation,
            color_fraction_mult)
        if is_dark:
            eff_saturation = sat_eff
            eff_value = adaptive_min_value(bright, brightness_threshold,
                                           min_value, 20)
            eff_color_frac = eff_color_frac * frac_mult
            gelap_info = f" [gelap V={bright:.0f}]"

    frac = color_fraction(frame, cls, xyxy, eff_saturation, eff_value)
    if frac < eff_color_frac:
        if debug:
            reasons.append(
                f"fraksi warna cocok {frac:.3f} < {eff_color_frac:.3f} "
                f"(warna bukan {('hijau' if cls == 0 else 'merah')} pekat)"
                f"{' [box kecil]' if is_small else ''}{gelap_info}")
            return False, reasons
        return False

    if debug:
        reasons.append(f"LOLOS (area {area} px, aspek {w / h:.2f}, "
                       f"warna {frac:.3f}{' [box kecil]' if is_small else ''}"
                       f"{gelap_info})")
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