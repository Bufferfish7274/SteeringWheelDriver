$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$source = Join-Path $PSScriptRoot "test_setpoint_ramp.cpp"
$exe = Join-Path $env:TEMP "setpoint_ramp_host_test.exe"

& g++ `
    -std=c++17 `
    -Wall `
    -Wextra `
    -Werror `
    -fno-exceptions `
    -fno-rtti `
    -I $libPath `
    $source `
    (Join-Path $libPath "SetpointRamp.cpp") `
    -lm `
    -o $exe

if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

try {
    & $exe
    $exitCode = $LASTEXITCODE
}
finally {
    Remove-Item -LiteralPath $exe -Force -ErrorAction SilentlyContinue
}

exit $exitCode
