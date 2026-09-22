# weights/ — Model YOLO

Model deteksi — **tidak di-versi-git** (masing-masing ±6 MB).
Unduh/salin ke folder ini sebelum menjalankan program:

| File | Dipakai untuk |
|---|---|
| `buoy.pt` | Gate (deteksi 2 pelampung) |
| `box_hijau.pt` | Misi foto kotak hijau |
| `best_blue_dark.pt` | Misi foto kotak biru (versi gelap) |
| `best_red_new.pt` | Misi docking kotak merah |

File lain (`box_biru.pt`, `box_merah.pt`, `best_greendock.pt`) adalah versi
lama/percobaan — aman diarsipkan atau dihapus, tidak dipakai mode aktif.

Cara ambil model: `scripts/download_weights.sh` (atau salin manual dari
penyimpanan tim). Di tahap 2, model akan diekspor ke `.onnx` untuk dijalankan
on-the-fly oleh `core/` (ONNX Runtime).