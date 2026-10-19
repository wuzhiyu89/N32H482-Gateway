# Build the bootloader for flash map mode 2 (BL at 0x08000000, 16 KB).
#
# This is deliberately separate from the Keil app project: the BL and the app
# must be two different images (BL at 0x08000000, app at 0x08004000).
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File tools\build_bl.ps1
#
# Output: MDK-ARM\bin\BL.bin  (also BL.axf/BL.map under MDK-ARM\Objects\bl)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

$tools = $env:KEIL_ARMCLANG
if (-not $tools) { $tools = 'D:\Keil_v5\ARM\ARMCLANG\Bin' }
$armclang = Join-Path $tools 'armclang.exe'
$armasm   = Join-Path $tools 'armasm.exe'
$armlink  = Join-Path $tools 'armlink.exe'
$fromelf  = Join-Path $tools 'fromelf.exe'
foreach ($t in @($armclang, $armasm, $armlink, $fromelf)) {
    if (-not (Test-Path $t)) { throw "tool not found: $t" }
}

$inc = @(
    (Join-Path $root 'firmware\CMSIS\core'),
    (Join-Path $root 'firmware\CMSIS\device'),
    (Join-Path $root 'firmware\n32h47x_48x_std_periph_driver\inc'),
    (Join-Path $root 'app\common'),
    (Join-Path $root 'bootloader')
)
$defs = @('N32H482', 'USE_STDPERIPH_DRIVER', 'FM_MAP_MODE=2')

$cflags = @(
    '--target=arm-arm-none-eabi', '-mcpu=cortex-m4', '-mthumb',
    '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
    '-std=c99', '-Os', '-Wno-unused-parameter'
)

$srcs = @(
    'firmware\CMSIS\device\system_n32h47x_48x.c',
    'firmware\n32h47x_48x_std_periph_driver\src\misc.c',
    'firmware\n32h47x_48x_std_periph_driver\src\n32h47x_48x_flash.c',
    'firmware\n32h47x_48x_std_periph_driver\src\n32h47x_48x_rcc.c',
    'firmware\n32h47x_48x_std_periph_driver\src\n32h47x_48x_gpio.c',
    'bootloader\bootloader.c',
    'bootloader\main_bl.c'
)

$objDir = Join-Path $root 'MDK-ARM\Objects\bl'
$binDir = Join-Path $root 'MDK-ARM\bin'
New-Item -ItemType Directory -Force -Path $objDir | Out-Null
New-Item -ItemType Directory -Force -Path $binDir | Out-Null

$defArgs = $defs | ForEach-Object { "-D$_" }
$incArgs = $inc  | ForEach-Object { "-I$_" }

$objs = @()
foreach ($s in $srcs) {
    $obj = Join-Path $objDir ((Split-Path -Leaf $s) -replace '\.c$', '.o')
    & $armclang @cflags @defArgs @incArgs -c (Join-Path $root $s) -o $obj
    if ($LASTEXITCODE -ne 0) { throw "compile failed: $s" }
    $objs += $obj
}

$startObj = Join-Path $objDir 'startup_n32h47x_48x.o'
& $armasm --cpu Cortex-M4 --pd 'N32H482 SETA 1' -o $startObj `
    (Join-Path $root 'firmware\CMSIS\device\startup\startup_n32h47x_48x.s')
if ($LASTEXITCODE -ne 0) { throw 'assemble failed: startup' }
$objs += $startObj

$axf = Join-Path $objDir 'BL.axf'
& $armlink --scatter (Join-Path $root 'bootloader\BL.sct') `
    --entry Reset_Handler --info sizes --map --diag_suppress=L6329 `
    @objs -o $axf
if ($LASTEXITCODE -ne 0) { throw 'link failed' }

& $fromelf --bin --output (Join-Path $binDir 'BL.bin') $axf
if ($LASTEXITCODE -ne 0) { throw 'fromelf failed' }

$len = (Get-Item (Join-Path $binDir 'BL.bin')).Length
Write-Host "BL.bin = $len bytes (limit 16384)"
if ($len -gt 16384) { throw 'BL does not fit in 16 KB' }
Write-Host 'BL build OK'
