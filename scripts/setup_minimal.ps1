# ============================================================
# scripts/setup_minimal.ps1 - install .venv minimal ASV di WINDOWS.
#
# Padanan scripts/setup_minimal.sh (urut & kunci SAMA).
# Hasil: .venv RINGAN (tanpa torch) dengan seluruh fungsi produksi:
# GUI PyQt, YOLO 4 model (ONNX via cv2.dnn), MAVLink.
#
# Pakai (PowerShell, dari root repo):
#   powershell -ExecutionPolicy Bypass -File scripts\setup_minimal.ps1
#   .venv\Scripts\python.exe -m unittest discover -s tests   # verifikasi
#
# IDEMPOTEN (aman diulang):
#   - .venv yang sudah ada DIPAKAI ULANG (tak ditimpa/dihapus).
#   - Bila paket inti sudah bisa di-import, langkah install dilewati.
#   - Langkah verifikasi aman diulang.
#   Mau paksa bersih? hapus dulu:  Remove-Item -Recurse -Force .venv
#
# KUNCI (sama dengan versi Linux):
#  1. Inferensi YOLO lewat ONNX + cv2.dnn (app/yolo_onnx.py) -> TIDAK perlu
#     torch/torchvision/ultralytics. Memangkas RAM (~537->~217 MB), waktu
#     start, dan ukuran .venv secara besar.
#  2. Jangan ganti opencv headless -> opencv-python biasa (duplikat Qt).
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

Write-Host "[0/3] Cek Python (min 3.12)..."
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
    Write-Host "[i] Python $pyVer terdeteksi (resep diuji di 3.12)."
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

# --- [1/3] Virtualenv (reuse kalau sudah ada) ---
if (Test-Path $Vpy) {
    Write-Host "[i] .venv sudah ada - dipakai ulang (tak ditimpa)."
} else {
    Write-Host "[1/3] Bikin virtualenv..."
    Invoke-Step @($Py, "-m", "venv", ".venv")
    $Fresh = $true
}

if ($Fresh) {
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "--upgrade", "pip")
}

# --- [2/3] Dependensi (lewati bila sudah lengkap) ---
& $Vpy -c "import cv2, serial, pymavlink; from PyQt5 import QtWidgets" 2>$null
if ($LASTEXITCODE -eq 0) {
    Write-Host "[i] Dependensi sudah terpasang - langkah install dilewati."
} else {
    Write-Host "[2/3] Install resep minimal (tanpa torch)..."
    Invoke-Step @($Vpy, "-m", "pip", "install", "-q", "-r", "requirements.txt")
}

# --- [3/3] Verifikasi (satu sumber dgn Linux: scripts/verify_env.py) ---
Write-Host "[3/3] Verifikasi import inti + inferensi YOLO (ONNX)..."
if (-not (Test-Path (Join-Path "weights" "buoy.onnx"))) {
    Write-Host "[!] weights/buoy.onnx belum ada - verifikasi ONNX dilewati."
    Write-Host "    Model ikut ter-commit di repo; pastikan clone tak parsial."
}
Invoke-Step @($Vpy, "scripts\verify_env.py")

Write-Host ""
Write-Host "Selesai. Jalankan GUI :  .venv\Scripts\python.exe -O -u main.py"
Write-Host "Ekspor monitor        :  data\monitor\  (lihat docs\MONITOR.md)  cek: scripts\monitor_read.py"
Write-Host "Jalankan tes          :  .venv\Scripts\python.exe -m unittest discover -s tests"
