$ErrorActionPreference = "Stop"

$root = $PSScriptRoot
$tests = @(
    "AngleOpt",
    "MasterCan",
    "SetpointRamp",
    "GM6020",
    "VESC"
)

foreach ($test in $tests) {
    Write-Host "Running $test host tests..."
    & powershell -ExecutionPolicy Bypass -File `
        (Join-Path $root "$test\host\run.ps1")
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

foreach ($variant in @("production", "bench")) {
    Write-Host "Running Application host tests ($variant)..."
    & powershell -ExecutionPolicy Bypass -File `
        (Join-Path $root "Application\host\run.ps1") `
        -Variant $variant
    if ($LASTEXITCODE -ne 0) {
        exit $LASTEXITCODE
    }
}

Write-Host "All host tests passed."
