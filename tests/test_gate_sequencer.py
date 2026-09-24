#!/usr/bin/env python3
"""
tests/test_gate_sequencer.py — Unit test GateSequencer (urutan target gate).

Menguji:
  1. collect_gate_pairs: filter vertikal & area-similarity.
  2. GateSequencer: latch target aktif, perpindahan cepat setelah lewat,
     toleransi kehilangan (lost frames).
  3. Deterministik: urutan gate tidak bergantung frame rate.
"""

import math
import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.gate_sequencer import (
    collect_gate_pairs, estimate_gate_distance,
    GateSequencer,
)


# Buoy sintetis minimal
def _ball(cx, cy, area=400, cls=None):
    b = {'cx': cx, 'cy': cy, 'area': area}
    if cls is not None:
        b['cls'] = cls
    return b


def test_collect_gate_pairs_vertical_align():
    red = [_ball(100, 100, 500)]
    green = [_ball(200, 100, 500)]  # same Y -> OK
    pairs = collect_gate_pairs(red, green, vertical_align_px=50, area_similarity_ratio=0.3)
    assert len(pairs) == 1


def test_collect_gate_pairs_vertical_reject():
    red = [_ball(100, 100, 500)]
    green = [_ball(200, 200, 500)]  # delta Y = 100 > 50 -> reject
    pairs = collect_gate_pairs(red, green, vertical_align_px=50, area_similarity_ratio=0.3)
    assert len(pairs) == 0


def test_collect_gate_pairs_area_similarity():
    red = [_ball(100, 100, 1000)]
    green = [_ball(200, 100, 300)]  # ratio = 0.3 < 0.5 -> reject
    pairs = collect_gate_pairs(red, green, vertical_align_px=50, area_similarity_ratio=0.5)
    assert len(pairs) == 0


def test_estimate_gate_distance_formula():
    # pixel_width = 100, gate_width=1.0m, focal=400px -> dist = 4.0m
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    dist = estimate_gate_distance((r, g), gate_width_m=1.0, focal_length_px=400)
    assert abs(dist - 4.0) < 1e-6


def test_estimate_gate_distance_invalid_small_width():
    r = _ball(100, 100, 500)
    g = _ball(101, 100, 500)  # width = 1 px -> inf
    dist = estimate_gate_distance((r, g), gate_width_m=1.0, focal_length_px=400)
    assert dist == float('inf')


def test_sequencer_basic_single_gate():
    """Satu gate -> langsung dipilih & di-latch."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=3,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    pairs = [(r, g)]

    mid_x, mid_y, dist, passed = seq.update(pairs, image_center_y=180)
    assert mid_x == 150 and mid_y == 100
    assert abs(dist - 4.0) < 1e-6
    assert not passed

    # Frame kedua, gate masih ada -> LATCH (target sama, tidak ganti-ganti)
    mid_x2, mid_y2, dist2, passed2 = seq.update(pairs, image_center_y=180)
    assert (mid_x2, mid_y2) == (mid_x, mid_y)


def test_sequencer_lost_tolerance():
    """Gate hilang < lost_tolerance -> target dipertahankan."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=3,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    pairs = [(r, g)]

    seq.update(pairs, 180)
    mid_x, mid_y, dist, passed = seq.update([], 180)  # lost 1
    assert (mid_x, mid_y) == (150, 100)
    seq.update([], 180)  # lost 2
    mid_x3, mid_y3, dist3, passed3 = seq.update([], 180)  # lost 3 (batas)
    assert (mid_x3, mid_y3) == (150, 100)
    # lost 4 -> target hilang
    mid_x4, mid_y4, dist4, passed4 = seq.update([], 180)
    assert mid_x4 is None


def test_sequencer_pass_gate_advance():
    """Gate lewat (dist < pass) -> langsung ambil gate berikutnya."""
    seq = GateSequencer(pass_distance_m=2.0, lost_tolerance_frames=5,
                        gate_width_m=1.0, focal_length_px=400)

    # Gate A jauh (4m), Gate B dekat (1.5m)
    rA, gA = _ball(50, 50, 800), _ball(150, 50, 800)   # width=100px -> 4.0m
    rB, gB = _ball(100, 100, 800), _ball(300, 100, 800) # width=200px -> 2.0m

    # Frame 1: A & B terlihat -> A terdekat? actually B closer (2.0 < 4.0)
    # urutan by distance: B (2m), A (4m)
    # active = B
    pairs = [(rB, gB), (rA, gA)]
    mid_x, mid_y, dist, passed = seq.update(pairs, 180)
    assert abs(dist - 2.0) < 1e-6  # B active
    assert not passed

    # Frame 2: B sudah dekat < pass_distance (2.0m) -> passed
    # simulated: B width grows -> dist < 2.0
    rB2, gB2 = _ball(90, 100, 900), _ball(310, 100, 900)  # width=220 -> 1.8m
    pairs2 = [(rB2, gB2), (rA, gA)]
    mid_x2, mid_y2, dist2, passed2 = seq.update(pairs2, 180)
    # passed = True karena dist < pass_distance
    assert passed2 is True

    # Frame 3: B hilang (kapal sudah lewat), A masih ada -> A jadi aktif
    pairs3 = [(rA, gA)]
    mid_x3, mid_y3, dist3, passed3 = seq.update(pairs3, 180)
    assert abs(dist3 - 4.0) < 1e-6
    assert not passed3


def test_sequencer_y_behind_center_passed():
    """Midpoint Y di bawah image_center_y -> dianggap lewat."""
    seq = GateSequencer(pass_distance_m=10.0, lost_tolerance_frames=5,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 250, 500)  # midpoint_y = 250 > center_y=180
    g = _ball(200, 250, 500)
    pairs = [(r, g)]

    mid_x, mid_y, dist, passed = seq.update(pairs, image_center_y=180)
    assert passed is True  # Y sudah lewat tengah frame


def test_sequencer_reset_on_leg_change():
    """Reset antrean saat berganti leg (waypoint)."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=5,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    pairs = [(r, g)]

    seq.update(pairs, 180)
    seq.reset()  # simulasi ganti leg
    mid_x, mid_y, dist, passed = seq.update([], 180)
    assert mid_x is None  # reset -> tidak ada target


def test_sequencer_tracking_memory_boost():
    """Buoy terlacak mengecil (jauh) tetap diprioritaskan via memory boost."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=5,
                        gate_width_m=1.0, focal_length_px=400,
                        track_match_px=30, track_boost=1.5)
    # Frame 1: gate besar (dekat) — isi memory
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    seq.update([(r, g)], 180)
    assert len(seq._buoy_memory[0]) == 1  # green tracked
    assert len(seq._buoy_memory[1]) == 1  # red tracked

    # Frame 2: dua gate — satu dekat memory, satu baru jauh
    # gate dekat memory: posisi shift kecil (105,102)
    r2 = _ball(105, 102, 450)
    g2 = _ball(205, 102, 450)
    # gate baru jauh: posisi beda jauh
    r3 = _ball(400, 100, 100)
    g3 = _ball(450, 100, 100)
    mid_x, mid_y, dist, passed = seq.update([(r2, g2), (r3, g3)], 180)
    # Gate terlacak harus dipilih (boost menurunkan eff_dist)
    assert (mid_x, mid_y) == (155, 102), f"got {(mid_x, mid_y)}"


def test_sequencer_memory_expires():
    """Memory expired setelah lost_tolerance_frames tanpa terlihat."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=3,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    seq.update([(r, g)], 180)
    assert len(seq._buoy_memory[0]) == 1
    # 3 frame kosong -> memory habis
    seq.update([], 180)
    seq.update([], 180)
    seq.update([], 180)
    mid_x, _, _, _ = seq.update([], 180)
    assert mid_x is None


def test_sequencer_reset_clears_memory():
    """Reset hapus memory tracking juga."""
    seq = GateSequencer(pass_distance_m=1.0, lost_tolerance_frames=5,
                        gate_width_m=1.0, focal_length_px=400)
    r = _ball(100, 100, 500)
    g = _ball(200, 100, 500)
    seq.update([(r, g)], 180)
    seq.reset()
    assert seq._buoy_memory == {0: [], 1: []}


if __name__ == "__main__":
    import traceback
    tests = [
        test_collect_gate_pairs_vertical_align,
        test_collect_gate_pairs_vertical_reject,
        test_collect_gate_pairs_area_similarity,
        test_estimate_gate_distance_formula,
        test_estimate_gate_distance_invalid_small_width,
        test_sequencer_basic_single_gate,
        test_sequencer_lost_tolerance,
        test_sequencer_pass_gate_advance,
        test_sequencer_y_behind_center_passed,
        test_sequencer_reset_on_leg_change,
        test_sequencer_tracking_memory_boost,
        test_sequencer_memory_expires,
        test_sequencer_reset_clears_memory,
    ]
    passed = 0
    for t in tests:
        try:
            t()
            print(f"[OK] {t.__name__}")
            passed += 1
        except Exception as e:
            print(f"[FAIL] {t.__name__}: {e}")
            traceback.print_exc()
    print(f"\n{passed}/{len(tests)} test passed.")
    sys.exit(0 if passed == len(tests) else 1)