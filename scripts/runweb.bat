@echo off
REM Jalankan relay web gateway (Redis -> WebSocket dashboard) di Windows.
cd /d "%~dp0.."
python -m app.gateway