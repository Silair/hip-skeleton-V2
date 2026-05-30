param(
    [string]$RoundName = "round_5nm_300s",
    [string]$LogPrefix = "hw_v2_5nm_300s"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$Python = "C:\msys64\mingw64\bin\python3.exe"
$LogBase = Join-Path $RepoRoot "Data\hardware_trials\$RoundName"
$ReportDir = Join-Path $LogBase "report"

if (!(Test-Path $Python)) {
    throw "Python not found: $Python"
}
if (!(Test-Path $LogBase)) {
    throw "Hardware trial log folder not found: $LogBase"
}

$csv = Get-ChildItem $LogBase -Recurse -Filter "$LogPrefix*.csv" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

if (!$csv) {
    throw "No runtime CSV found under $LogBase for prefix $LogPrefix"
}

$env:PYTHONPATH = "C:/Users/admin/Documents/Codex/msys_tempfix"
$env:TEMP = "C:/Users/admin/Documents/Codex/.tmp"
$env:TMP = "C:/Users/admin/Documents/Codex/.tmp"
$env:TMPDIR = "C:/Users/admin/Documents/Codex/.tmp"

New-Item -ItemType Directory -Force $ReportDir | Out-Null

Push-Location $RepoRoot
try {
    & $Python tools/analyze_run.py $csv.FullName --output-dir $ReportDir
    if ($LASTEXITCODE -ne 0) {
        throw "analyze_run.py failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Report:  $ReportDir\index.html"
Write-Host "Metrics: $ReportDir\metrics.json"
