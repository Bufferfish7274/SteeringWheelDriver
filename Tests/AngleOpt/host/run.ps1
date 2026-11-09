$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$libPath = Join-Path $projectRoot "Lib"
$source = Join-Path $PSScriptRoot "test_angle_opt.cpp"
$exe = Join-Path $env:TEMP "angle_opt_host_test.exe"

& g++ `
    -std=c++17 `
    -Wall `
    -Wextra `
    -Werror `
    -fno-exceptions `
    -fno-rtti `
    -I $libPath `
    $source `
    (Join-Path $libPath "AngleOpt.cpp") `
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
