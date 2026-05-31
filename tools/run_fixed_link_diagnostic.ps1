param(
    [ValidateSet("left", "right", "stop_leg", "zero_compare")]
    [string]$Trial = "left",
    [double]$MaxTorqueNm = 0.5,
    [double]$DurationS = 20.0,
    [bool]$CalibrateOnStart = $true,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$round = "fixed_link_${Trial}_${MaxTorqueNm}nm"
$prefix = "fixed_link_${Trial}_${MaxTorqueNm}nm"

& (Join-Path $PSScriptRoot "run_hardware_trial.ps1") `
    -MaxTorqueNm $MaxTorqueNm `
    -DurationS $DurationS `
    -RoundName $round `
    -LogPrefix $prefix `
    -BaseGainMinNm 0.2 `
    -BaseGainMaxNm $MaxTorqueNm `
    -RampUpRatePerS 0.25 `
    -RampDownRatePerS 2.0 `
    -WarmupAnchorCount 3 `
    -CalibrateOnStart $CalibrateOnStart `
    -SkipBuild:$SkipBuild

$repoRoot = Split-Path -Parent $PSScriptRoot
$logBase = Join-Path $repoRoot "Data\hardware_trials\$round"
$latestCsv = Get-ChildItem $logBase -Recurse -Filter "$prefix*.csv" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

if ($latestCsv) {
    $side = if ($Trial -eq "right") { "right" } else { "left" }
    $report = Join-Path $latestCsv.DirectoryName "fixed_link_diagnostics.json"
    Write-Host "Diagnostic report:"
    & python3 (Join-Path $repoRoot "tools\fixed_link_diagnostics.py") `
        $latestCsv.FullName `
        --side $side `
        --output $report
    Write-Host "  $report"
}
