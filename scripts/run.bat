@echo off
REM Jalankan GUI dashboard ASV (Windows dev).
REM Path dibuat relatif ke folder ini, jadi aman dipindah-pindah.
REM -O: jalankan mode optimized (strip assert/bookkeeping) untuk produksi.
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" (
    .venv\Scripts\python.exe -O -u main.py
) else (
    python -O -u main.py
)
