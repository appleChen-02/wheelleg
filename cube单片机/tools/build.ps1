param(
    [ValidateSet('Debug', 'Release')]
    [string]$Configuration = 'Debug',
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$cubeCommand = Get-Command cube -ErrorAction SilentlyContinue
if ($cubeCommand) {
    $cubeCli = $cubeCommand.Source
} else {
    $extensionsRoot = Join-Path $env:USERPROFILE '.vscode/extensions'
    $cubeCli = Get-ChildItem -Path "$extensionsRoot/stmicroelectronics.stm32cube-ide-core-*/resources/binaries/win32/x86_64/cube.exe" -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1 -ExpandProperty FullName
}

Push-Location -LiteralPath $projectRoot
$savedPath = $env:PATH
try {
    # A bare `cube cmake` loads CMake's bundle, but not the compiler bundle.
    # In an ordinary VS Code terminal, add installed GCC/Ninja tools explicitly.
    $bundleRoot = $env:CUBE_BUNDLE_PATH
    if (-not $bundleRoot) { $bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube/bundles' }
    foreach ($tool in @(
        @{ Name = 'arm-none-eabi-gcc'; Bundle = 'gnu-tools-for-stm32'; Executable = 'arm-none-eabi-gcc.exe' },
        @{ Name = 'ninja'; Bundle = 'ninja'; Executable = 'ninja.exe' }
    )) {
        if (-not (Get-Command $tool.Name -ErrorAction SilentlyContinue)) {
            $installed = Get-ChildItem -Path "$bundleRoot/$($tool.Bundle)/*/bin/$($tool.Executable)" -ErrorAction SilentlyContinue |
                Sort-Object LastWriteTime -Descending | Select-Object -First 1
            if ($installed) { $env:PATH = $installed.DirectoryName + [IO.Path]::PathSeparator + $env:PATH }
        }
    }

    function Invoke-ProjectCMake([string[]]$CMakeArguments) {
        # Windows PowerShell may turn redirected native stderr (including
        # compiler warnings) into ErrorRecords. Use the actual exit status.
        $previousErrorPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            if ($cubeCli) { & $cubeCli cmake @CMakeArguments } else { & cmake @CMakeArguments }
            $cmakeExitCode = $LASTEXITCODE
        } finally { $ErrorActionPreference = $previousErrorPreference }
        if ($cmakeExitCode -ne 0) { throw "CMake failed (exit $cmakeExitCode). Check the build output and STM32Cube GCC/Ninja bundles." }
    }

    if ($cubeCli) {
        $env:PATH = (Split-Path -Parent $cubeCli) + [IO.Path]::PathSeparator + $env:PATH
    }
    Invoke-ProjectCMake -CMakeArguments @('--preset', $Configuration)

    $buildArgs = @('--build', '--preset', $Configuration, '--parallel')
    if ($Clean) { $buildArgs += '--clean-first' }
    Invoke-ProjectCMake $buildArgs
} finally {
    $env:PATH = $savedPath
    Pop-Location
}
