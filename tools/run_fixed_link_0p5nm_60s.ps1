param(
    [ValidateSet("left", "right")]
    [string]$Side = "left",
    [bool]$CalibrateOnStart = $true,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

Write-Host "Fixed-link diagnostic: 0.5 Nm, 60 s, side=$Side"
Write-Host "Do not use your hand as the primary restraint. Use a fixture/strap/limit frame and keep the physical emergency stop reachable."
Write-Host "The report checks RawMotorPos drift, RawMotorVel spikes, and whether the controller reaches Stopping / zero output."

& (Join-Path $PSScriptRoot "run_fixed_link_diagnostic.ps1") `
    -Trial $Side `
    -MaxTorqueNm 0.5 `
    -DurationS 60.0 `
    -CalibrateOnStart $CalibrateOnStart `
    -SkipBuild:$SkipBuild
