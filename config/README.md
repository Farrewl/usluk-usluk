# config/ — Data konfigurasi (bukan kode)

| File | Isi |
|---|---|
| `plan.csv` | Daftar waypoint misi (format: `lat,lon`). **Kanonik** — dipakai `main.py` |
| `tuning_params.json` | Parameter tuning tersimpan otomatis saat GUI ditutup |
| `archive/lintasan_a_old.csv` | Versi lama lintasan (hanya arsip, tidak dipakai) |

Jangan menyimpan file besar di sini; output runtime ada di `data/`.