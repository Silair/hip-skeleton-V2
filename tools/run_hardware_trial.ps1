param(
    [double]$MaxTorqueNm = 5.0,
    [double]$DurationS = 300.0,
    [string]$RoundName = "round_5nm_300s",
    [string]$LogPrefix = "hw_v2_5nm_300s",
    [string]$Exe = "Data\offline_validation\hs_exoskeleton_v2_hw.exe",
    [int]$CanChannel = 0,
    [string]$LeftMotorId = "0x0001",
    [string]$RightMotorId = "0x0002",
    [double]$LeftTorqueScale = 1.0,
    [double]$RightTorqueScale = 1.0,
    [double]$BaseGainMinNm = 3.0,
    [double]$BaseGainMaxNm = 6.0,
    [double]$RampUpRatePerS = 0.60,
    [double]$RampDownRatePerS = 1.20,
    [int]$WarmupAnchorCount = 3,
    [double]$AoInitialFrequencyHz = 0.8,
    [double]$AoMaxFrequencyHz = 1.4,
    [double]$AnchorFrequencyMaxHz = 1.6,
    [double]$AnchorFrequencyGain = 0.35,
    [double]$AnchorFrequencyGainRamp = 0.15,
    [double]$StopVelocityThresholdRadS = 0.08,
    [double]$StopEnterHoldSeconds = 0.16,
    [double]$FreezeEnterStopProbability = 0.78,
    [double]$FreezeEnterHoldSeconds = 0.20,
    [bool]$CalibrateOnStart = $true,
    [switch]$V1CompatIgnoreEnableAck,
    [switch]$V1CompatIgnoreZeroAck,
    [switch]$SkipBuild
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$ExePath = Join-Path $RepoRoot $Exe
$LogBase = "Data\hardware_trials\$RoundName"

if (!$SkipBuild -or !(Test-Path $ExePath)) {
    & (Join-Path $PSScriptRoot "build_hardware_trial.ps1") -Output $Exe
}

if (!(Test-Path $ExePath)) {
    throw "Hardware executable not found: $ExePath"
}

New-Item -ItemType Directory -Force (Join-Path $RepoRoot $LogBase) | Out-Null

$env:HSX_RUN_DURATION_S = "$DurationS"
$env:HSX_MAX_TORQUE_NM = "$MaxTorqueNm"
$env:HSX_LOG_BASE_FOLDER = $LogBase
$env:HSX_LOG_PREFIX = $LogPrefix
$env:HSX_SYNC_SESSION_ID = $RoundName
$env:HSX_STREAM_ID = "exo_hs_v2_hw"
$env:HSX_CAN_CHANNEL = "$CanChannel"
$env:HSX_LEFT_MOTOR_ID = $LeftMotorId
$env:HSX_RIGHT_MOTOR_ID = $RightMotorId
$env:HSX_LEFT_TORQUE_SCALE = "$LeftTorqueScale"
$env:HSX_RIGHT_TORQUE_SCALE = "$RightTorqueScale"
$env:HSX_BASE_GAIN_MIN_NM = "$BaseGainMinNm"
$env:HSX_BASE_GAIN_MAX_NM = "$BaseGainMaxNm"
$env:HSX_RAMP_UP_RATE_PER_S = "$RampUpRatePerS"
$env:HSX_RAMP_DOWN_RATE_PER_S = "$RampDownRatePerS"
$env:HSX_WARMUP_ANCHOR_COUNT = "$WarmupAnchorCount"
$env:HSX_AO_INITIAL_FREQUENCY_HZ = "$AoInitialFrequencyHz"
$env:HSX_AO_MAX_FREQUENCY_HZ = "$AoMaxFrequencyHz"
$env:HSX_ANCHOR_FREQUENCY_MAX_HZ = "$AnchorFrequencyMaxHz"
$env:HSX_ANCHOR_FREQUENCY_GAIN = "$AnchorFrequencyGain"
$env:HSX_ANCHOR_FREQUENCY_GAIN_RAMP = "$AnchorFrequencyGainRamp"
$env:HSX_STOP_VELOCITY_THRESHOLD_RAD_S = "$StopVelocityThresholdRadS"
$env:HSX_STOP_ENTER_HOLD_SECONDS = "$StopEnterHoldSeconds"
$env:HSX_FREEZE_ENTER_STOP_PROBABILITY = "$FreezeEnterStopProbability"
$env:HSX_FREEZE_ENTER_HOLD_SECONDS = "$FreezeEnterHoldSeconds"
$env:HSX_CALIBRATE_ON_START = if ($CalibrateOnStart) { "1" } else { "0" }
$env:HSX_IGNORE_MOTOR_ENABLE_RESULT = if ($V1CompatIgnoreEnableAck) { "1" } else { "0" }
$env:HSX_IGNORE_ZERO_RESULT = if ($V1CompatIgnoreZeroAck) { "1" } else { "0" }

Write-Host "Starting hardware trial:"
Write-Host "  torque limit: $MaxTorqueNm Nm"
Write-Host "  duration:     $DurationS s"
Write-Host "  log base:     $LogBase"
Write-Host "  executable:   $ExePath"
Write-Host "  torque scale: left=$LeftTorqueScale, right=$RightTorqueScale"
Write-Host "  gains:        base=$BaseGainMinNm..$BaseGainMaxNm Nm, ramp up/down=$RampUpRatePerS/$RampDownRatePerS"
Write-Host "  frequency:    init=$AoInitialFrequencyHz Hz, max=$AoMaxFrequencyHz Hz, anchor max=$AnchorFrequencyMaxHz Hz"
Write-Host "  stop gate:    vel=$StopVelocityThresholdRadS rad/s hold=$StopEnterHoldSeconds s, freeze p=$FreezeEnterStopProbability hold=$FreezeEnterHoldSeconds s"
Write-Host "  V1 compat:    ignore enable ack=$($V1CompatIgnoreEnableAck.IsPresent), ignore zero ack=$($V1CompatIgnoreZeroAck.IsPresent)"
Write-Host "Use the physical emergency stop / power cutoff as the primary stop."

Push-Location $RepoRoot
try {
    & $ExePath
    $exitCode = $LASTEXITCODE
} finally {
    Pop-Location
}

$latestCsv = Get-ChildItem (Join-Path $RepoRoot $LogBase) -Recurse -Filter "$LogPrefix*.csv" |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

if ($latestCsv) {
    Write-Host "Latest runtime CSV: $($latestCsv.FullName)"
    Write-Host "Analyze with VS Code task: HSX: Analyze latest 5Nm 300s"
}

exit $exitCode
