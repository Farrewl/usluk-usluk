"""
app/gate_sequencer.py — Urutan target gate (tengah merah-hijau) yang cepat.

Masalah yang dipecahkan: navigator lama hanya "memikirkan" SATU gate tiap
frame (`_find_best_gate` = pasangan paling dekat). Begitu kapal melewati
tengahnya, ia harus mencari ulang gate berikutnya dari nol — terasa lambat
("mikir kelamaan").

Solusi: sequencer mengumpulkan SEMUA pasangan plausible tiap frame, lalu
mempertahankan target aktif (titik tengah + perkiraan jarak) yang di-latch:
  * Target aktif TIDAK berubah-ubah antar frame selama masih terlihat —
    kapal tidak "gelisah" memilih-milih antar dua gate yang sama jauhnya.
  * Saat target aktif hilang sesaat (frame kosong), arah terakhir tetap
    dipertahankan selama `lost_tolerance_frames` frame — kapal tetap ingat
    ke mana harus melaju (tidak mikir ulang dari nol).
  * Saat target aktif LEWAT (jarak < `pass_distance_m` ATAU titik tengahnya
    sudah turun melewati tengah frame, artinya kapal berada tepat/lewat
    di belakangnya), sequencer LANGSUNG memilih target terdekat berikutnya
    yang sudah dikumpulkan.

Semua fungsi murni & deterministik agar mudah diuji:
  tests/test_gate_sequencer.py. Dipakai oleh app/navigator.py (misi asli)
  dan app/simulator.py (skenario darat).
"""


def collect_gate_pairs(red_balls, green_balls, vertical_align_px,
                       area_similarity_ratio):
    """Kumpulkan semua pasangan (merah, hijau) yang bentuknya cocok jadi gate.

    Setiap pasangan harus:
      * Sumbu-Y hampir sejajar (selisih <= vertical_align_px) — dua buoy
        di hadapan kapal tampak berdiri sejajar di frame.
      * Luas kotak mirip (min/max >= area_similarity_ratio) — dua buoy
        sejenis berukuran hampir sama karena jaraknya berimbang.

    Return list (red_ball, green_ball). Urutan tidak dijamin — callers
    memakai perkiraan jarak untuk menyortir.
    """
    pairs = []
    for r in red_balls:
        for g in green_balls:
            if abs(r['cy'] - g['cy']) > vertical_align_px:
                continue
            area_r = r.get('area', 0.0)
            area_g = g.get('area', 0.0)
            if area_r <= 0 or area_g <= 0:
                continue
            bigger = max(area_r, area_g)
            if min(area_r, area_g) / bigger < area_similarity_ratio:
                continue
            pairs.append((r, g))
    return pairs


def estimate_gate_distance(pair, gate_width_m, focal_length_px):
    """Perkirakan jarak gate (meter) dari lebar piksel antar buoy.

    Rumus pinhole: distance = (gate_width_m * focal_length_px) / pixel_width.
    Lebar piksel terlalu kecil (< 6 px) -> float('inf') (tidak reliabel).
    """
    r_ball, g_ball = pair
    pixel_width = abs(r_ball['cx'] - g_ball['cx'])
    if pixel_width <= 5 or focal_length_px <= 0 or gate_width_m <= 0:
        return float('inf')
    return (gate_width_m * focal_length_px) / pixel_width


def _midpoint(pair):
    """Titik tengah pasangan (cx, cy) di frame."""
    r_ball, g_ball = pair
    return ((r_ball['cx'] + g_ball['cx']) / 2.0,
            (r_ball['cy'] + g_ball['cy']) / 2.0)


class GateSequencer:
    """Antrean target gate dengan latch & perpindahan cepat.

    API utama: `update(pairs, image_center_y)` mengembalikan
    (mid_x, mid_y, distance_m, is_passed) untuk target aktif. Saat tidak
    ada target -> (None, None, float('inf'), False).
    """

    def __init__(self, pass_distance_m=1.2, lost_tolerance_frames=5,
                 gate_width_m=1.0, focal_length_px=400,
                 midpoint_match_px=6.0,
                 track_match_px=30, track_boost=1.5):
        self.pass_distance_m = pass_distance_m
        self.lost_tolerance_frames = lost_tolerance_frames
        self.gate_width_m = gate_width_m
        self.focal_length_px = focal_length_px
        self.midpoint_match_px = midpoint_match_px
        self.track_match_px = track_match_px
        self.track_boost = track_boost
        self.reset()

    def reset(self):
        """Kosongkan target & state latch (awal misi / pergantian leg)."""
        self.active_mid = None      # (mid_x, mid_y) target aktif
        self.active_dist = float('inf')
        self.active_pair = None
        self.lost_frames = 0
        self.is_passed = False
        # Memori tracking buoy individual (tidak hanya pasangan):
        # {cls: [(cx, cy, area, frames_since_seen), ...]}
        self._buoy_memory = {0: [], 1: []}

    def _build_targets(self, pairs):
        """List target terurut jarak: [mid_x, mid_y, dist, pair]."""
        built = []
        for pair in pairs:
            mid_x, mid_y = _midpoint(pair)
            dist = estimate_gate_distance(pair, self.gate_width_m,
                                          self.focal_length_px)
            built.append([mid_x, mid_y, dist, pair])
        built.sort(key=lambda t: t[2])
        return built

    def _find_match(self, targets):
        """Cari target yang titik tengahnya == target aktif (dalam toleransi)."""
        if self.active_mid is None:
            return -1
        ax, ay = self.active_mid
        for idx, (mid_x, mid_y, dist, pair) in enumerate(targets):
            if abs(mid_x - ax) <= self.midpoint_match_px and \
               abs(mid_y - ay) <= self.midpoint_match_px:
                return idx
        return -1

    def _is_passed(self, mid_y, dist, image_center_y):
        """True bila kapal sudah berada tepat/lewat di belakang target."""
        return dist < self.pass_distance_m or mid_y > image_center_y

    def update(self, pairs, image_center_y):
        """Perbarui antrean & kembalikan target aktif.

        Return: (mid_x, mid_y, distance_m, is_passed).
        """
        targets = self._build_targets(pairs)

        if not targets:
            # Tidak ada buoy terlihat: tahan target lama bila masih dalam
            # toleransi kehilangan — kapal tetap ingat arah.
            if self.active_mid is not None and \
               self.lost_frames < self.lost_tolerance_frames:
                self.lost_frames += 1
                return (self.active_mid[0], self.active_mid[1],
                        self.active_dist, self.is_passed)
            self.active_mid = None
            self.active_pair = None
            self.active_dist = float('inf')
            return None, None, float('inf'), False

        self.lost_frames = 0

        # --- Update tracking memory for individual buoys ---
        # Extract individual buoys from pairs
        seen_red = set()
        seen_green = set()
        for pair in pairs:
            r_ball, g_ball = pair
            seen_red.add((r_ball['cx'], r_ball['cy'], r_ball.get('area', 0)))
            seen_green.add((g_ball['cx'], g_ball['cy'], g_ball.get('area', 0)))

        self._update_buoy_memory(1, seen_red)  # red
        self._update_buoy_memory(0, seen_green)  # green

        # --- Rank targets: memory boost hanya pengaruh urutan, jarak asli tetap ---
        ranked = self._rank_targets_from_memory(targets)

        match = self._find_match(targets)
        if match >= 0 and not self.is_passed:
            # Target aktif masih terlihat dan belum lewat -> latch tetap.
            mid_x, mid_y, dist, pair = targets[match]
            self.active_mid = (mid_x, mid_y)
            self.active_dist = dist
            self.active_pair = pair
            self.is_passed = self._is_passed(mid_y, dist, image_center_y)
            return mid_x, mid_y, dist, self.is_passed

        # Target aktif hilang ATAU sudah lewat: pilih target depan terdekat.
        # Target di belakang frame (mid_y > center) tidak pernah dipilih lagi.
        # Iterasi urutan ranked (boost), tapi kembalikan jarak ASLI.
        for idx in ranked:
            mid_x, mid_y, dist, pair = targets[idx]
            if dist < self.pass_distance_m or mid_y > image_center_y:
                continue
            self.active_mid = (mid_x, mid_y)
            self.active_dist = dist
            self.active_pair = pair
            self.is_passed = False
            return mid_x, mid_y, dist, False

        # Semua pasangan terlihat sudah di belakang kapal -> lewat semua.
        # Kembalikan yang terdekat (yang justru baru dilewati) agar caller
        # tahu posisinya, tapi tandai passed sehingga tidak dipilih ulang.
        if targets:
            mid_x, mid_y, dist, pair = targets[ranked[0]]
            self.active_mid = (mid_x, mid_y)
            self.active_dist = dist
            self.active_pair = pair
            self.is_passed = True
            return mid_x, mid_y, dist, True

        return None, None, float('inf'), False

    def _update_buoy_memory(self, cls, seen_positions):
        """Perbarui memori posisi buoy per class.

        `seen_positions`: set of (cx, cy, area) yg terlihat frame ini.
        Entri lama di-increment frames_since_seen; yang tidak terlihat
        > lost_tolerance_frames dihapus.
        """
        mem = self._buoy_memory.setdefault(cls, [])
        # Decrement counter for existing (frame tidak terlihat)
        for entry in mem:
            entry[3] += 1
        # Match yang terlihat: reset counter, update posisi (smooth ringan)
        for cx, cy, area in seen_positions:
            matched = False
            for entry in mem:
                px, py, _, _ = entry
                if (cx - px) ** 2 + (cy - py) ** 2 <= self.track_match_px ** 2:
                    entry[0] = int(0.7 * px + 0.3 * cx)
                    entry[1] = int(0.7 * py + 0.3 * cy)
                    entry[2] = area
                    entry[3] = 0
                    matched = True
                    break
            if not matched:
                mem.append([cx, cy, area, 0])
        # Hapus yang expired
        mem[:] = [e for e in mem if e[3] < self.lost_tolerance_frames]

    def _rank_targets_from_memory(self, targets):
        """Urutan index target: yang cocok memori tracking didahulukan.

        Return list index (bukan target baru) — jarak asli tidak diubah,
        hanya urutan pemilihan. Buoy terlacak yang mengecil (jauh)
        tetap diprioritaskan karena memorinya masih ada.
        """
        scored = []
        for idx, (mid_x, mid_y, dist, pair) in enumerate(targets):
            r_ball, g_ball = pair
            boost = 1.0
            for cls, ball in [(1, r_ball), (0, g_ball)]:
                mem = self._buoy_memory.get(cls, [])
                for px, py, _, _frames_ago in mem:
                    if (ball['cx'] - px) ** 2 + (ball['cy'] - py) ** 2 <= self.track_match_px ** 2:
                        boost = max(boost, self.track_boost)
                        break
            eff = dist / boost if boost > 1.0 else dist
            scored.append((eff, idx))
        scored.sort(key=lambda s: s[0])
        return [idx for _, idx in scored]

    def _boost_targets_from_memory(self, targets):
        """Kompat lawas: kembalikan target terurut (jarak asli, bukan eff).

        Dipakai test tracking — urutan = prioritas boost.
        """
        ranked = self._rank_targets_from_memory(targets)
        return [targets[i] for i in ranked]