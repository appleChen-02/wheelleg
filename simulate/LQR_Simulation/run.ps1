[CmdletBinding()]
param(
    [ValidateSet('bipedal', 'sysid')]
    [string]$Mode = 'bipedal',
    [ValidateRange(0.0, [double]::MaxValue)]
    [double]$Duration = 0,
    [switch]$Headless,
    [ValidateSet('truth', 'mcu')]
    [string]$Feedback = 'truth',
    [string]$OutDir,
    [switch]$BuildOnly,
    [string]$Msys2Root = 'D:/MSYS2'
)

$ErrorActionPreference = 'Stop'
$toolBin = Join-Path $Msys2Root 'ucrt64/bin'
$buildDir = Join-Path $PSScriptRoot 'build/msys2_ucrt64'
foreach ($tool in @('cmake.exe', 'ninja.exe', 'gcc.exe', 'g++.exe')) {
    if (-not (Test-Path -LiteralPath (Join-Path $toolBin $tool))) {
        throw "Missing $tool in $toolBin. Install the MSYS2 UCRT64 GCC, CMake and Ninja packages."
    }
}
if ($OutDir -and $Mode -ne 'sysid') {
    throw '-OutDir requires -Mode sysid.'
}

$previousPath = $env:PATH
$env:PATH = "$toolBin;$previousPath"
Push-Location -LiteralPath $PSScriptRoot
try {
    $configureArgs = @(
        '-S', $PSScriptRoot, '-B', $buildDir, '-G', 'Ninja',
        '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON',
        "-DCMAKE_C_COMPILER=$toolBin/gcc.exe",
        "-DCMAKE_CXX_COMPILER=$toolBin/g++.exe",
        "-DCMAKE_MAKE_PROGRAM=$toolBin/ninja.exe",
        '-DLQR_BUILD_SYSID_TESTS=ON'
    )
    & (Join-Path $toolBin 'cmake.exe') @configureArgs
    if ($LASTEXITCODE -ne 0) { throw "CMake configure failed: $LASTEXITCODE" }
    & (Join-Path $toolBin 'cmake.exe') --build $buildDir --parallel 4
    if ($LASTEXITCODE -ne 0) { throw "Build failed: $LASTEXITCODE" }

    if (-not $BuildOnly) {
        $runArgs = @('--mode', $Mode, '--feedback', $Feedback)
        if ($Duration -gt 0) {
            $runArgs += @('--duration', $Duration.ToString([Globalization.CultureInfo]::InvariantCulture))
        }
        if ($Headless) { $runArgs += '--headless' }
        if ($OutDir) { $runArgs += @('--outdir', $OutDir) }
        & (Join-Path $buildDir 'LQRSimulation.exe') @runArgs
        if ($LASTEXITCODE -ne 0) { throw "Simulation failed: $LASTEXITCODE" }
    }
} finally {
    Pop-Location
    $env:PATH = $previousPath
}
