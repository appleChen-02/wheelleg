[CmdletBinding(SupportsShouldProcess)]
param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Release',
    [ValidateRange(1, 24000)]
    [int]$Frequency = 1000,
    [ValidatePattern('^[A-Za-z0-9]+$')]
    [string]$SerialNumber
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$relativeFirmware = "build/$Configuration/standard_robot.bin"
$firmwarePath = Join-Path $projectRoot $relativeFirmware
if (-not (Test-Path -LiteralPath $firmwarePath -PathType Leaf)) {
    throw "BIN not found: $firmwarePath. Run the STM32: Build $Configuration task first."
}
$firmware = Get-Item -LiteralPath $firmwarePath
if ($firmware.Length -lt 8 -or $firmware.Length -gt 1048576) {
    throw 'The BIN size is invalid for the STM32F407IG 1 MiB Flash.'
}

$cubeCommand = Get-Command cube -ErrorAction SilentlyContinue
$cubeCli = if ($cubeCommand) { $cubeCommand.Source } else { $null }
if (-not $cubeCli) {
    $extensionsRoot = Join-Path $env:USERPROFILE '.vscode/extensions'
    $cubeCli = Get-ChildItem -Path "$extensionsRoot/stmicroelectronics.stm32cube-ide-core-*/resources/binaries/win32/x86_64/cube.exe" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}
if (-not $cubeCli) { throw 'Cube CLI not found. Install the ST STM32CubeIDE for VS Code extension pack.' }

# The address follows STM32F407IGHX_FLASH.ld (no bootloader offset).
$programmerArgs = @('-c', 'port=SWD', "freq=$Frequency")
if ($SerialNumber) { $programmerArgs += "sn=$SerialNumber" }
$programmerArgs += @('-w', $relativeFirmware, '0x08000000', '-v', '-rst')
Write-Host "BIN: $firmwarePath ($($firmware.Length) bytes)"
if (-not $PSCmdlet.ShouldProcess('STM32 target via ST-LINK/SWD at 0x08000000', "cube programmer $($programmerArgs -join ' ')")) {
    return
}

# Keep the filename relative and ASCII for programmer tools on Windows.
Push-Location -LiteralPath $projectRoot
try {
    $previousErrorPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        & $cubeCli programmer @programmerArgs
        $programmerExitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousErrorPreference }
    if ($programmerExitCode -ne 0) {
        throw "BIN programming failed (exit $programmerExitCode). Check ST-LINK, its USB driver, target power, and SWD wiring."
    }
    Write-Host 'BIN programmed, verified, and MCU reset successfully.'
} finally {
    Pop-Location
}
