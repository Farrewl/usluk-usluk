# ============================================================
# scripts/setup_minimal.ps1 — install .venv minimal ASV di WINDOWS.
#
# Padanan scripts/setup_minimal.sh (urut & kunci penghematan SAMA):
# Hasil: .venv ±1,3 GB dengan seluruh fungsi produksi jalan:
# GUI PyQt, YOLO 4 model (.pt), MAVLink, Redis/upload.
#
# Pakai (PowerShell, dari root repo):
#   powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
#   .venv\Scripts\python.exe -m unittest discover -s tests   # verifikasi
#
# KUNCI penghematan (sama dengan versi Linux, lihat setup_minimal.sh):
#  1. torch WAJIB dari index CPU — dari PyPI default ia menarik
#     CUDA+nvidia-*+triton (≈3,6 GB membengkak). Wheel Windows CPU
#     (win_amd64, bertanda +cpu) sudah diverifikasi tersedia.
#  2. ultralytics dipasang --no-deps — deps bawaannya menarik
#     matplotlib+polars+opencv-python non-headless yang TIDAK PERNAH
#     di-import eager saat inferensi.
#  3. Folder test/include di dalam torch aman dibuang.
#
# Catatan Windows (beda dengan Linux):
#  - Gamepad USB (/dev/input/js*) TIDAK ada — fitur gamepad dinonaktif
#    otomatis; RC Pixhawk / tombol GUI tetap jalan.
#  - Library C: aterkia_core.dll butuh MinGW (lihat core/Makefile,
#    target `windows`). Tanpa MinGW pun program tetap jalan dengan
#    C_AVAILABLE=False (gagal eksplisit, tanpa fallback diam-diam).
#  - Butuh Python 3.12 (dicek di langkah 0).
# ============================================================
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

# Interpreter: hormati env PYTHON, default "python".
$Py = if ($env:PYTHON) { $env:PYTHON } else { "python" }

function Invoke-Step {
    # Jalankan satu proses; keluar otomatis kalau exit code != 0.
    # Argumen dilewat literal ke proses (tanpa parsing parameter).
    param([string[]]$Cmd)
    & ($Cmd[0]) $Cmd[1..($Cmd.Length - 1)]
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Write-Host "[0/5] Cek Python 3.12..."
$pyVer = & $Py -c "import sys; print('%d.%d' % sys.version_info[:2])"
if ($LASTEXITCODE -ne 0) { Write-Host "[!] Interpreter '$Py' tak jalan. Set env PYTHON dulu."; exit 1 }
if ($pyVer -ne "3.12") {
    Write-Host "[!] Python $pyVer terdeteksi, resep ini dipin ke 3.12 (wheel torch+cpu)."
    $jawab = Read-Host "    Tetap lanjut? [y/N]"
    if ($jawab -notin @("y", "Y")) { Write-Host "Batal."; exit 1 }
}

if (Test-Path .venv) {
    Write-Host "[!] .venv sudah ada."
    $jawab = Read-Host "    Timpa dengan .venv minimal? [y/N]"
    if ($jawab -notin @("y", "Y")) { Write-Host "Batal."; exit 1 }
    Remove-Item -Recurse -Force .venv
}

Write-Host "[1/5] Bikin virtualenv..."
Invoke-Step @($Py, "-m", "venv", ".venv")
$Vpy = Join-Path ".venv" "Scripts\python.exe"
Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "--upgrade", "pip")

Write-Host "[2/5] Install torch+torchvision CPU-ONLY (index PyTorch, bukan PyPI!)..."
Invoke-Step @($Vpy, "-m", "pip", "install", "-q",
    "--index-url", "https://download.pytorch.org/whl/cpu",
    "torch==2.14.0+cpu", "torchvision==0.29.0+cpu")

Write-Host "[3/5] Install resep minimal + ultralytics tanpa deps borosnya..."
Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "-r", "requirements.txt")
Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "--no-deps", "ultralytics==8.4.160")

Write-Host "[4/5] Pangkas folder test/include torch..."
# Path site-packages Windows: .venv\Lib\site-packages (bukan lib/pythonX).
$SP = Join-Path ".venv" "Lib\site-packages"
foreach ($bagian in @("torch\test", "torch\include")) {
    $p = Join-Path $SP $bagian
    if (Test-Path $p) { Remove-Item -Recurse -Force $p }
}
# File test di dalam torch\lib (paralel lib*test*.so di Linux; bila ada).
Get-ChildItem -Path (Join-Path $SP "torch\lib") -Filter "*test*" -ErrorAction SilentlyContinue |
    Remove-Item -Force -ErrorAction SilentlyContinue

Write-Host "[5/5] Verifikasi import inti + inferensi YOLO..."
if (-not (Test-Path (Join-Path "weights" "buoy.pt"))) {
    Write-Host "[!] weights/buoy.pt belum ada — verifikasi YOLO dilewati."
    Write-Host "    Isi dulu:  `$env:WEIGHTS_SOURCE='\\server\weights'; .\scripts\download_weights.ps1"
}
@'
import numpy as np
import torch
assert torch.version.cuda is None, f"KECETOT varian CUDA: {torch.__version__}"
import cv2, redis, serial
from PyQt5.QtWidgets import QApplication
from pymavlink import mavutil
print(f"torch {torch.__version__} | cv2 {cv2.__version__} | import OK")
try:
    from ultralytics import YOLO
    r = YOLO("weights/buoy.pt").predict(
        np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
    assert r[0].plot().shape == (480, 640, 3)
    print("YOLO infer OK")
except FileNotFoundError:
    print("YOLO dilewati (weights belum ada)")
'@ | & $Vpy -
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

Write-Host ""
Write-Host "Selesai. Jalankan program:  .venv\Scripts\python.exe main.py"
Write-Host "Jalankan tes:               .venv\Scripts\python.exe -m unittest discover -s tests"
