$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$appPath = Join-Path $projectRoot "App"
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_wheel_debug.cpp"
$exe = Join-Path $env:TEMP "wheel_debug_host_test.exe"

& g++ `
    -std=c++17 `
    -Wall `
    -Wextra `
    -Werror `
    -fno-exceptions `
    -fno-rtti `
    -I $stubPath `
    -I $appPath `
    -I $libPath `
    $source `
    (Join-Path $appPath "WheelDebug.cpp") `
    (Join-Path $libPath "AngleOpt.cpp") `
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
