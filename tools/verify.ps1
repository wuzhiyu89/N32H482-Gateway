# Build + verify the gateway firmware (no board needed).
#
#   1. Keil build  -> must report "0 Error(s)" and the same compile/link is
#      warning-free (UV4 exit code 0)
#   2. _host_test/check_ps1.py -> every .ps1 must be UTF-8 with BOM
#   3. _host_test/run.ps1      -> 13 gcc suites, 1840 assertions
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\verify.ps1
#   powershell -ExecutionPolicy Bypass -File tools\verify.ps1 -SkipBuild
#
# This file is intentionally ASCII-only: it needs no BOM.

param(
    [switch]$SkipBuild,
    [switch]$SkipTest
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

function Fail($msg) {
    Write-Host ""
    Write-Host "FAILED: $msg" -ForegroundColor Red
    exit 1
}

$failed = @()

# ---------------------------------------------------------------- 1. build
if (-not $SkipBuild) {
    $uv4 = $env:KEIL_UV4
    if (-not $uv4) {
        $candidates = @(
            'D:\Keil_v5\UV4\UV4.exe',
            'C:\Keil_v5\UV4\UV4.exe',
            (Join-Path ${env:ProgramFiles} 'Keil_v5\UV4\UV4.exe')
        )
        $uv4 = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
    }
    if (-not $uv4) {
        Fail "UV4.exe not found (set KEIL_UV4 or install Keil MDK)"
    }

    $proj = Join-Path $root 'MDK-ARM\CDC.uvprojx'
    $log = Join-Path $env:TEMP 'gateway_build.log'
    if (Test-Path $log) { Remove-Item $log -Force }

    Write-Host "=== Keil build ==="
    $p = Start-Process -FilePath $uv4 `
                       -ArgumentList @('-j0', '-b', $proj, '-o', $log) `
                       -WorkingDirectory (Split-Path $proj) -Wait -PassThru
    $text = ''
    if (Test-Path $log) { $text = Get-Content -LiteralPath $log -Raw -Encoding Default }

    $size = ($text -split "`n" | Where-Object { $_ -match 'Program Size' } | Select-Object -Last 1)
    if ($size) { Write-Host $size.Trim() }

    if ($p.ExitCode -ne 0 -or $text -notmatch '0 Error\(s\)') {
        Write-Host $text
        $failed += "keil build (exit=$($p.ExitCode))"
    } else {
        Write-Host "build OK (0 Error / 0 Warning)"
    }
}

# ------------------------------------------------------------ 2. ps1 guard
if (-not $SkipTest) {
    Write-Host ""
    Write-Host "=== check_ps1 (UTF-8 BOM) ==="
    & python '_host_test\check_ps1.py'
    if ($LASTEXITCODE -ne 0) { $failed += 'check_ps1 (dangerous .ps1 line endings)' }
}

# ------------------------------------------------------------- 3. tests
if (-not $SkipTest) {
    Write-Host ""
    Write-Host "=== host tests ==="
    & powershell -NoProfile -ExecutionPolicy Bypass -File '_host_test\run.ps1'
    if ($LASTEXITCODE -ne 0) { $failed += 'host tests' }
}

# ------------------------------------------------------------- summary
Write-Host ""
if ($failed.Count -eq 0) {
    Write-Host "ALL CHECKS PASSED"
    exit 0
}
Write-Host ("FAILED: " + ($failed -join ', ')) -ForegroundColor Red
exit 1
