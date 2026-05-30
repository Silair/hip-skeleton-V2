param(
    [string]$Output = "Data\offline_validation\hs_exoskeleton_v2_hw.exe"
)

$ErrorActionPreference = "Stop"

$RepoRoot = Split-Path -Parent $PSScriptRoot
$OldRoot = "C:\Users\admin\Desktop\hs_exoskeleton\hs_exoskeleton"
$Gxx = "C:\msys64\mingw64\bin\g++.exe"
$KvaserInclude = "D:\kvaser\INC"
$KvaserLib = "D:\kvaser\Lib\x64\canlib32.lib"
$EigenInclude = "D:\eigen-3.4.1\eigen-3.4.1"
$OutputPath = Join-Path $RepoRoot $Output

$required = @(
    $Gxx,
    (Join-Path $OldRoot "include\MultiHarmonicAO.h"),
    (Join-Path $OldRoot "include\Kvaser_H\kvaser.h"),
    (Join-Path $OldRoot "include\Robot_H\selfDevManuipulator.h"),
    (Join-Path $KvaserInclude "canlib.h"),
    $KvaserLib,
    (Join-Path $EigenInclude "Eigen\Core")
)

foreach ($path in $required) {
    if (!(Test-Path $path)) {
        throw "Missing required build input: $path"
    }
}

New-Item -ItemType Directory -Force (Split-Path -Parent $OutputPath) | Out-Null

$args = @(
    "-std=c++17",
    "-fpermissive",
    "-include", "cstdint",
    "-DM_PI=3.14159265358979323846",
    "-I.",
    "-I$OldRoot\include",
    "-I$OldRoot\include\Kvaser_H",
    "-I$OldRoot\include\Robot_H",
    "-I$OldRoot\include\DataManager_H",
    "-I$OldRoot\include\Controller_H",
    "-I$OldRoot\include\Trajectory_H",
    "-I$KvaserInclude",
    "-I$EigenInclude",
    "app/main.cpp",
    "control/ExoController.cpp",
    "control/GaitFeatureExtractor.cpp",
    "control/PhaseEstimator.cpp",
    "control/IntentDetector.cpp",
    "control/FreezeManager.cpp",
    "control/StopDetector.cpp",
    "control/AssistStateMachine.cpp",
    "control/TorqueProfile.cpp",
    "control/StopTorqueLimiter.cpp",
    "hardware/KvaserExoHardware.cpp",
    "hardware/LegacyJointSpaceMapper.cpp",
    "logging/ExoLogger.cpp",
    "logging/Clock.cpp",
    "$OldRoot\src\Kvaser_CPP\KvaserBase.cpp",
    "$OldRoot\src\Kvaser_CPP\kvaser.cpp",
    "$OldRoot\src\Timer.cpp",
    $KvaserLib,
    "-o", $OutputPath
)

Push-Location $RepoRoot
try {
    & $Gxx @args
    if ($LASTEXITCODE -ne 0) {
        throw "g++ failed with exit code $LASTEXITCODE"
    }
} finally {
    Pop-Location
}

Write-Host "Built hardware trial executable: $OutputPath"
