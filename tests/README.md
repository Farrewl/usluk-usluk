# tests/ — Unit Test

Test Python yang membuktikan logika benar sebelum dipakai di kapal.
Jalankan dari root repo:

```bash
python -m unittest discover -s tests -v
```

| File | Apa yang diuji |
|---|---|
| `test_geo.py` | Matematika geodetik: haversine vs nilai bumi nyata, simetri, normalize, cross-track, round-trip destination |

Test C/C++ untuk `core/` akan ditambahkan per modul pada Tahap 2
(lihat `core/README.md`).