param(
    [ValidateSet("production", "bench")]
    [string]$Variant = "production"
)

$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$appPath = Join-Path $projectRoot "App"
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_application.c"
$exe = Join-Path $env:TEMP "application_${Variant}_host_test.exe"
$boardSource = if ($Variant -eq "bench") {
    Join-Path $appPath "board_app_bench.c"
}
else {
    Join-Path $appPath "board_app_production.c"
}
$arguments = @(
    "-std=c99",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-I", $stubPath,
    "-I", $appPath,
    "-I", $libPath
)

if ($Variant -eq "bench") {
    $arguments += "-DAPP_BENCH=1"
}

$syntaxArguments = @(
    "-std=c99",
    "-Wall",
    "-Wextra",
    "-Werror",
    "-fsyntax-only",
    "-I", $stubPath,
    "-I", $appPath,
    "-I", $libPath
)
if ($Variant -eq "bench") {
    $syntaxArguments += "-DAPP_BENCH=1"
}
$syntaxArguments += $boardSource

& gcc @syntaxArguments
if ($LASTEXITCODE -ne 0) {
    exit $LASTEXITCODE
}

$arguments += @(
    $source,
    (Join-Path $appPath "app_config.c"),
    (Join-Path $appPath "steering_wheel_app.c"),
    (Join-Path $appPath "bench_input.c"),
    (Join-Path $libPath "GM6020.c"),
    (Join-Path $libPath "VESC.c"),
    (Join-Path $libPath "MasterCan.c"),
    (Join-Path $libPath "pid.c"),
    (Join-Path $libPath "angle_opt.c"),
    (Join-Path $libPath "setpoint_ramp.c"),
    "-lm",
    "-o", $exe
)

& gcc @arguments
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
