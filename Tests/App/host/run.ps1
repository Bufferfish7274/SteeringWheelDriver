$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$appPath = Join-Path $projectRoot "App"
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_app_debug.c"
$exe = Join-Path $env:TEMP "app_debug_host_test.exe"

& gcc `
    -std=c99 `
    -Wall `
    -Wextra `
    -Werror `
    -DWHEEL_BUILD_DEBUG=1 `
    -I $stubPath `
    -I $appPath `
    -I $libPath `
    $source `
    (Join-Path $appPath "App.c") `
    (Join-Path $appPath "WheelDebug.c") `
    (Join-Path $libPath "GM6020.c") `
    (Join-Path $libPath "VESC.c") `
    (Join-Path $libPath "pid.c") `
    (Join-Path $libPath "bsp_can.c") `
    (Join-Path $libPath "angle_opt.c") `
    (Join-Path $libPath "setpoint_ramp.c") `
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
