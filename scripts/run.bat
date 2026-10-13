@echo off
REM Jalankan GUI dashboard ASV (Windows dev).
REM Path dibuat relatif ke folder ini, jadi aman dipindah-pindah.
REM -O: jalankan mode optimized (strip assert/bookkeeping) untuk produksi.
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" (
    .venv\Scripts\python.exe -O -u main.py
) else (
    echo [!] .venv belum ada. Install dulu sekali:
    echo       powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
    python -O -u main.py
)
