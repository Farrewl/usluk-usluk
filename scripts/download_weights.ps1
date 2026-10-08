# ============================================================
# scripts/download_weights.ps1 — ambil model YOLO ke folder weights/.
# Padanan scripts/download_weights.sh (logika sama, Windows).
#
# Model (~6 MB/file, total ±25 MB untuk 4 model aktif) sengaja TIDAK
# dimasukkan ke git. Simpan di satu tempat bersama tim, lalu jalankan:
#
#   $env:WEIGHTS_SOURCE = "\\server\bersama\weights"
#   .\scripts\download_weights.ps1
#
#   $env:WEIGHTS_SOURCE = "D:\backup\weights.tar.gz"
#   .\scripts\download_weights.ps1
#
# WEIGHTS_SOURCE boleh folder (copy per file) atau archive .tar.gz
# (diekstrak; butuh tar — bawaan Windows 10 1803+).
# ============================================================
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

$Src = $env:WEIGHTS_SOURCE
# File yang dibutuhkan program (lihat app/ — model aktif 4 buah).
$Needed = @("buoy.pt", "box_hijau.pt", "best_blue_dark.pt", "best_red_new.pt")

if (-not $Src) {
    Write-Host "Setelah WEIGHTS_SOURCE diisi, file yang dibutuhkan:"
    foreach ($f in $Needed) { Write-Host "  - weights/$f" }
    Write-Host ""
    Write-Host "  `$env:WEIGHTS_SOURCE = 'D:\path\ke\sumber'; .\scripts\download_weights.ps1"
    exit 1
}

New-Item -ItemType Directory -Force -Path weights | Out-Null

if (Test-Path $Src -PathType Container) {
    foreach ($f in $Needed) {
        $asal = Join-Path $Src $f
        if (Test-Path $asal) {
            Copy-Item $asal (Join-Path "weights" $f) -Force
            Write-Host "  + weights/$f"
        } else {
            Write-Host "  ! $f tidak ada di sumber"
        }
    }
} elseif (Test-Path $Src -PathType Leaf) {
    # Archive .tar.gz — tar.exe bawaan Windows 10 1803+ (bsdtar).
    tar -xzf $Src -C weights --strip-components=1
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    Write-Host "  + diekstrak dari archive"
} else {
    Write-Host "  ! WEIGHTS_SOURCE tidak ditemukan: $Src"
    exit 1
}

Write-Host "Selesai. Model aktif tersimpan di weights/."
