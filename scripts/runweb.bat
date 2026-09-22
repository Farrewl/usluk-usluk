@echo off
REM Jalankan relay web gateway (Redis -> WebSocket dashboard) di Windows.
cd /d "%~dp0.."
python -u -m app.gateway