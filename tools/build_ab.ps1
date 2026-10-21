# Build the two A/B images (same source, different link base) for the dual-slot OTA test.
#
#   A: IROM 0x08004000, VECT_TAB_OFFSET=0x4000   (runs from Slot A)
#   B: IROM 0x08038000, VECT_TAB_OFFSET=0x38000  (runs from Slot B)
#
# Output: MDK-ARM\bin\CDC_A_<ver>.bin / CDC_B_<ver>.bin
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\build_ab.ps1 -Version 1.0.6
#
# The project file is restored byte-for-byte when the script finishes.

param(
    [string]$Version = "1.0.6"
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$proj = Join-Path $root 'MDK-ARM\CDC.uvprojx'
$log  = Join-Path $env:TEMP 'gateway_build_ab.log'
$bin  = Join-Path $root 'MDK-ARM\bin'
New-Item -ItemType Directory -Force -Path $bin | Out-Null

$orig = [IO.File]::ReadAllBytes($proj)
$bom  = ($orig.Length -ge 3) -and ($orig[0] -eq 0xEF) -and ($orig[1] -eq 0xBB) -and ($orig[2] -eq 0xBF)
$text = [Text.Encoding]::UTF8.GetString($orig)
if ($bom) { $text = $text.Substring(1) }

$encoding = New-Object System.Text.UTF8Encoding($bom)
$verEsc = 'MAIN_FW_VERSION=\"' + $Version + '\"'

function Build-Variant([string]$name, [string]$startAddr, [string]$vectOff) {
    $t = $text
    $t = $t.Replace('<StartAddress>0x8004000</StartAddress>', "<StartAddress>$startAddr</StartAddress>")
    $t = $t.Replace('<StartAddress>0x8000000</StartAddress>', "<StartAddress>$startAddr</StartAddress>")
    $t = $t.Replace('VECT_TAB_OFFSET=0x4000', "VECT_TAB_OFFSET=$vectOff")
    $t = $t.Replace('VECT_TAB_OFFSET=0x38000', "VECT_TAB_OFFSET=$vectOff")
    $t = $t.Replace('MAIN_OTA_AUTO_REBOOT=1</Define>',
                    'MAIN_OTA_AUTO_REBOOT=1,' + $verEsc + '</Define>')
    [IO.File]::WriteAllText($proj, $t, $encoding)

    if (Test-Path $log) { Remove-Item $log -Force }
    $p = Start-Process -FilePath 'D:\Keil_v5\UV4\UV4.exe' `
                       -ArgumentList @('-j0', '-b', $proj, '-o', $log) `
                       -WorkingDirectory (Split-Path $proj) -Wait -PassThru
    if ($p.ExitCode -ne 0) {
        Get-Content $log -Tail 20
        throw "$name build failed (UV4 exit $($p.ExitCode))"
    }
    $dst = Join-Path $bin "CDC_${name}_$Version.bin"
    Copy-Item (Join-Path $bin 'CDC.bin') $dst -Force
    Write-Host ("{0}: {1} bytes -> {2}" -f $name, (Get-Item $dst).Length, $dst)
}

try {
    Build-Variant 'A' '0x8004000' '0x4000'
    Build-Variant 'B' '0x8038000' '0x38000'
}
finally {
    [IO.File]::WriteAllBytes($proj, $orig)
}
Write-Host "A/B build done (project restored)"
