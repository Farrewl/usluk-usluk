@echo off
REM Jalankan GUI dashboard ASV (Windows dev).
REM Path dibuat relatif ke folder ini, jadi aman dipindah-pindah.
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" (
    .venv\Scripts\python.exe -u main.py
) else (
    python -u main.py
)
