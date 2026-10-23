# Builds core/ as a Cortex-M4F static library with STM32CubeCLT, prints its size
# and checks the FPU code. Board-independent toolchain check, not a firmware image.
$ErrorActionPreference = 'Stop'

foreach ($tool in 'arm-none-eabi-gcc', 'arm-none-eabi-size', 'cmake', 'ninja') {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        Write-Error "'$tool' not found in PATH (STM32CubeCLT expected)"
    }
}

Push-Location (Join-Path $PSScriptRoot '..\..')
try {
    & arm-none-eabi-gcc --version | Select-Object -First 1
    cmake --preset target-check
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
    cmake --build --preset target-check
    if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
}
finally {
    Pop-Location
}
