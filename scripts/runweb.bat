@echo off
REM Jalankan relay web gateway (Redis -> WebSocket dashboard) di Windows.
cd /d "%~dp0.."
if exist ".venv\Scripts\python.exe" (
    .venv\Scripts\python.exe -u -m app.gateway
) else (
    python -u -m app.gateway
)
