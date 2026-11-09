$ErrorActionPreference = "Stop"

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$appPath = Join-Path $projectRoot "App"
$libPath = Join-Path $projectRoot "Lib"
$stubPath = Join-Path $PSScriptRoot "stubs"
$source = Join-Path $PSScriptRoot "test_app.cpp"
$sources = @(
    (Join-Path $appPath "WheelApp.cpp")
    (Join-Path $appPath "WheelAppApi.cpp")
    (Join-Path $appPath "WheelDebug.cpp")
    (Join-Path $appPath "DebugController.cpp")
    (Join-Path $appPath "ProductionController.cpp")
    (Join-Path $appPath "Telemetry.cpp")
    (Join-Path $appPath "StatusLeds.cpp")
    (Join-Path $libPath "GM6020.cpp")
    (Join-Path $libPath "VESC.cpp")
    (Join-Path $libPath "MasterCan.cpp")
    (Join-Path $libPath "Pid.cpp")
    (Join-Path $libPath "BspCan.cpp")
    (Join-Path $libPath "AngleOpt.cpp")
    (Join-Path $libPath "SetpointRamp.cpp")
)

function Invoke-AppTest([int]$DebugBuild)
{
    $exe = Join-Path $env:TEMP "app_$($DebugBuild)_host_test.exe"

    & g++ `
        -std=c++17 `
        -Wall `
        -Wextra `
        -Werror `
        -fno-exceptions `
        -fno-rtti `
        "-DWHEEL_BUILD_DEBUG=$DebugBuild" `
        -I $stubPath `
        -I $appPath `
        -I $libPath `
        $source `
        $sources `
        -lm `
        -o $exe

    if ($LASTEXITCODE -ne 0) {
        $script:appTestExitCode = $LASTEXITCODE
        return
    }

    try {
        & $exe
        $script:appTestExitCode = $LASTEXITCODE
    }
    finally {
        Remove-Item -LiteralPath $exe -Force -ErrorAction SilentlyContinue
    }
}

$script:appTestExitCode = 1
Invoke-AppTest 1
if ($script:appTestExitCode -eq 0) {
    Invoke-AppTest 0
}

exit $script:appTestExitCode
