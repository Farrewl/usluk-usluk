# ============================================================
# scripts/setup_minimal.ps1 - install .venv minimal ASV di WINDOWS.
#
# Padanan scripts/setup_minimal.sh (urut & kunci penghematan SAMA).
# Hasil: .venv ~1,3 GB dengan seluruh fungsi produksi jalan:
# GUI PyQt, YOLO 4 model (.pt), MAVLink.
#
# Pakai (PowerShell, dari root repo):
#   powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
#   .venv\Scripts\python.exe -m unittest discover -s tests   # verifikasi
#
# IDEMPOTEN (aman diulang):
#   - .venv yang sudah ada DIPAKAI ULANG (tak ditimpa/dihapus).
#   - Bila paket inti sudah bisa di-import, langkah install dilewati.
#   - Langkah pangkas & verifikasi aman diulang.
#   Mau paksa bersih? hapus dulu:  Remove-Item -Recurse -Force .venv
#
# KUNCI penghematan (sama dengan versi Linux):
#  1. torch WAJIB dari index CPU - dari PyPI default ia menarik
#     CUDA+nvidia-*+triton (~3,6 GB membengkak). Wheel Windows CPU
#     (win_amd64, bertanda +cpu) sudah diverifikasi tersedia.
#  2. ultralytics dipasang --no-deps - deps bawaannya menarik
#     matplotlib+polars+opencv-python non-headless yang TIDAK PERNAH
#     di-import eager saat inferensi.
#  3. Folder test/include di dalam torch aman dibuang.
#
# Catatan Windows (beda dengan Linux):
#  - Gamepad USB (/dev/input/js*) TIDAK ada - fitur gamepad dinonaktif
#    otomatis; RC Pixhawk / tombol GUI tetap jalan.
#  - Library C: aterkia_core.dll butuh MinGW (lihat core/Makefile,
#    target `windows`). Tanpa MinGW pun program tetap jalan dengan
#    C_AVAILABLE=False (gagal eksplisit, tanpa fallback diam-diam).
#  - Butuh Python 3.12 (dicek di langkah 0).
#  - Verifikasi akhir pakai scripts/verify_env.py (BUKAN here-string)
#    agar kompatibel dengan Windows PowerShell 5.1.
# ============================================================
# JANGAN pakai "Stop": di PS 5.1 stderr program native (python/pip)
# direkam jadi ErrorRecord dan EAP=Stop menjadikannya TERMINATING,
# walau sudah diarahkan 2>$null -> setup mati di tengah. Semua langkah
# native dicek eksplisit via $LASTEXITCODE (Invoke-Step + guard).
$ErrorActionPreference = "Continue"
Set-Location (Join-Path $PSScriptRoot "..")

# Interpreter: hormati env PYTHON, default "python".
$Py = if ($env:PYTHON) { $env:PYTHON } else { "python" }
$Fresh = $false

function Invoke-Step {
    # Jalankan satu proses; keluar otomatis kalau exit code != 0.
    # Argumen dilewat literal ke proses (tanpa parsing parameter).
    param([string[]]$Cmd)
    & ($Cmd[0]) $Cmd[1..($Cmd.Length - 1)]
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}

Write-Host "[0/5] Cek Python (min 3.12)..."
$pyVer = & $Py -c "import sys; print('%d.%d' % sys.version_info[:2])"
if ($LASTEXITCODE -ne 0) {
    if ($Py -ne "py") {
        Write-Host "[i] '$Py' tak jalan; coba launcher 'py'..."
        $Py = "py"
        $pyVer = & $Py -c "import sys; print('%d.%d' % sys.version_info[:2])"
    }
    if ($LASTEXITCODE -ne 0) { Write-Host "[!] Interpreter '$Py' tak jalan. Set env PYTHON dulu."; exit 1 }
}
$pyNum = [double]$pyVer
if ($pyNum -lt 3.12) {
    Write-Host "[!] Python $pyVer terlalu tua; butuh >= 3.12."; exit 1
}
if ($pyVer -ne "3.12") {
    Write-Host "[i] Python $pyVer terdeteksi (resep diuji di 3.12; wheel torch+cpu ada untuk 3.13/3.14)."
}

# --- [core] Build aterkia_core.dll bila belum ada (butuh MinGW) ---
if (Test-Path "core\aterkia_core.dll") {
    Write-Host "[i] core\aterkia_core.dll sudah ada - build C dilewati."
} else {
    $mingw = Get-Command mingw32-make -ErrorAction SilentlyContinue
    if ($mingw) {
        Write-Host "[core] Build inti C (MinGW)..."
        & mingw32-make -C core windows
        if ($LASTEXITCODE -ne 0) {
            Write-Host "[!] Build C gagal (mingw32-make rc=$LASTEXITCODE)."
            exit $LASTEXITCODE
        }
    } else {
        Write-Host "[!] MinGW (mingw32-make) tak ada - core\aterkia_core.dll TIDAK dibangun."
        Write-Host "    Program jalan tapi mesin-state C mati (C_AVAILABLE=False)."
        Write-Host "    Install MinGW-w64 lalu:  mingw32-make -C core windows"
    }
}

$Vpy = Join-Path ".venv" "Scripts\python.exe"

# --- [1/5] Virtualenv (reuse kalau sudah ada) ---
if (Test-Path $Vpy) {
    Write-Host "[i] .venv sudah ada - dipakai ulang (tak ditimpa)."
} else {
    Write-Host "[1/5] Bikin virtualenv..."
    Invoke-Step @($Py, "-m", "venv", ".venv")
    $Fresh = $true
}

if ($Fresh) {
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "--upgrade", "pip")
}

# --- [2-3/5] Dependensi (lewati bila sudah lengkap) ---
& $Vpy -c "import torch, torchvision, cv2, serial, pymavlink, ultralytics; from PyQt5 import QtWidgets" 2>$null
if ($LASTEXITCODE -eq 0) {
    Write-Host "[i] Dependensi sudah terpasang - langkah install (2-3) dilewati."
} else {
    Write-Host "[2/5] Install torch+torchvision CPU-ONLY (index PyTorch, bukan PyPI!)..."
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q",
        "--index-url", "https://download.pytorch.org/whl/cpu",
        "torch==2.14.0+cpu", "torchvision==0.29.0+cpu")

    Write-Host "[3/5] Install resep minimal + ultralytics tanpa deps borosnya..."
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "-r", "requirements.txt")
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "--no-deps", "ultralytics==8.4.160")
}

# --- [4/5] Pangkas folder test/include torch (idempoten) ---
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

# --- [5/5] Verifikasi (satu sumber dgn Linux: scripts/verify_env.py) ---
Write-Host "[5/5] Verifikasi import inti + inferensi YOLO..."
if (-not (Test-Path (Join-Path "weights" "buoy.pt"))) {
    Write-Host "[!] weights/buoy.pt belum ada - verifikasi YOLO dilewati."
    Write-Host "    Model YOLO ikut ter-commit di repo; pastikan clone tak parsial."
}
Invoke-Step @($Vpy, "scripts\verify_env.py")

Write-Host ""
Write-Host "Selesai. Jalankan GUI :  .venv\Scripts\python.exe -O -u main.py"
Write-Host "Ekspor monitor        :  data\monitor\  (lihat docs\MONITOR.md)  cek: scripts\monitor_read.py"
Write-Host "Jalankan tes          :  .venv\Scripts\python.exe -m unittest discover -s tests"
