# Hip Encoder Stop Detector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [x]`) syntax for tracking.

**Goal:** Replace stop handling that depends on static gait spread with a hip-encoder-only stop detector that uses sustained low joint velocity to pause phase tracking and ramp assistance down before freeze.

**Architecture:** Add a small `StopDetector` control module fed only by current left/right joint velocities. `ExoController` will combine the detector with the existing `FreezeManager`: encoder stop requests disable phase validity/tracking and tell `AssistStateMachine` to soft-stop via a separate `Stopping` ramp-down path; hard hardware faults still call `emergencyStop()`.

**Tech Stack:** C++17 control modules, existing assert-based `tests/test_control.cpp`, manual `g++` test build because this repo currently has no CMake/Make entrypoint.

---

## Files

- Create: `control/StopDetector.h` — config-driven detector API and output struct.
- Create: `control/StopDetector.cpp` — moving-average absolute joint velocity and hold-time logic.
- Modify: `config/ControlConfig.h` — add `StopConfig` to tune velocity threshold, moving-average time constant, and hold time.
- Modify: `control/ExoController.h` — own a `StopDetector`.
- Modify: `control/ExoController.cpp` — update stop detector after reading state; disable phase tracking/validity and request soft-stop when detector is active.
- Modify: `control/AssistStateMachine.h/.cpp` — split natural stop (`Stopping`, soft ramp-down output allowed) from freeze (`Frozen`, output disabled immediately).
- Modify: `tests/test_control.cpp` — add regression tests for low-speed stop detection and assist soft-stop.

## Task 1: Add encoder-only stop detector tests

- [x] **Step 1: Write failing tests**

Add these tests to `tests/test_control.cpp`:

```cpp
#include "control/StopDetector.h"

void test_stop_detector_ignores_static_large_spread_and_triggers_on_low_velocity() {
    exo::ControlConfig config;
    config.stop.velocity_threshold_rad_s = 0.08;
    config.stop.enter_hold_seconds = 0.16;
    exo::StopDetector detector(config.stop);

    exo::ExoState state{};
    state.left.position_rad = 0.45;
    state.right.position_rad = -0.35;
    state.left.velocity_rad_s = 0.02;
    state.right.velocity_rad_s = -0.03;

    exo::StopDecision decision{};
    for (int i = 0; i < 8; ++i) {
        decision = detector.update(state, 0.02);
    }

    assert(decision.stop_requested);
    assert(!decision.phase_tracking_enabled);
    assert(decision.average_abs_velocity_rad_s < config.stop.velocity_threshold_rad_s);
}

void test_stop_detector_exits_when_joint_velocity_returns() {
    exo::ControlConfig config;
    config.stop.velocity_threshold_rad_s = 0.08;
    config.stop.exit_velocity_threshold_rad_s = 0.16;
    config.stop.enter_hold_seconds = 0.10;
    config.stop.exit_hold_seconds = 0.06;
    exo::StopDetector detector(config.stop);

    exo::ExoState state{};
    state.left.velocity_rad_s = 0.01;
    state.right.velocity_rad_s = 0.01;
    for (int i = 0; i < 6; ++i) {
        detector.update(state, 0.02);
    }

    state.left.velocity_rad_s = 0.30;
    state.right.velocity_rad_s = -0.28;
    exo::StopDecision decision{};
    for (int i = 0; i < 4; ++i) {
        decision = detector.update(state, 0.02);
    }

    assert(!decision.stop_requested);
    assert(decision.phase_tracking_enabled);
}
```

- [x] **Step 2: Run test to verify it fails**

Run:

```bash
cd /home/lin/code/hs_exoskeleton_v2
g++ -std=c++17 -I. tests/test_control.cpp control/AssistStateMachine.cpp control/FreezeManager.cpp control/GaitFeatureExtractor.cpp control/IntentDetector.cpp control/PhaseEstimator.cpp control/TorqueProfile.cpp control/StopDetector.cpp -o /tmp/hs_exo_test_control
```

Expected: compile failure because `control/StopDetector.h` and `control/StopDetector.cpp` do not exist.

## Task 2: Implement `StopDetector`

- [x] **Step 1: Add config**

Add to `config/ControlConfig.h`:

```cpp
struct StopConfig {
    double velocity_threshold_rad_s = 0.08;
    double exit_velocity_threshold_rad_s = 0.16;
    double enter_hold_seconds = 0.16;
    double exit_hold_seconds = 0.08;
    double velocity_filter_alpha = 0.25;
};
```

and add `StopConfig stop{};` to `ControlConfig`.

- [x] **Step 2: Add detector header and implementation**

`StopDetector` output:

```cpp
struct StopDecision {
    bool stop_requested = false;
    bool phase_tracking_enabled = true;
    double average_abs_velocity_rad_s = 0.0;
};
```

Logic:

1. Compute `(abs(left.velocity_rad_s) + abs(right.velocity_rad_s)) / 2`.
2. First-order filter it with `velocity_filter_alpha`.
3. If filtered velocity is below `velocity_threshold_rad_s` for `enter_hold_seconds`, enter stopped state.
4. If stopped and filtered velocity is above `exit_velocity_threshold_rad_s` for `exit_hold_seconds`, exit stopped state.
5. `phase_tracking_enabled = !stop_requested`.

- [x] **Step 3: Run test to verify it passes**

Run the same `g++` command from Task 1 and then:

```bash
/tmp/hs_exo_test_control
```

Expected: exit code 0.

## Task 3: Wire stop detector into the live control loop

- [x] **Step 1: Modify `ExoController` ownership**

Add `#include "control/StopDetector.h"` and a `StopDetector stop_detector_;` member initialized from `config.stop`.

- [x] **Step 2: Modify loop order**

After `hardware_.readState(state)` and before phase estimation:

```cpp
StopDecision stop = stop_detector_.update(state, controller_dt_s);
const bool phase_tracking_enabled = freeze.phase_tracking_enabled && stop.phase_tracking_enabled;
PhaseEstimate phase = phase_estimator_.update(features, controller_dt_s, phase_tracking_enabled);
```

When preparing assist inputs:

```cpp
assist_inputs.phase_valid = phase.valid && !freeze.recovery_active && !stop.stop_requested;
assist_inputs.stop_requested = stop.stop_requested;
assist_inputs.freeze_requested = freeze.freeze_requested;
```

The final motor gate remains:

```cpp
command.allow_output = assist.allow_output && !freeze.freeze_requested && state.healthy;
```

This keeps natural stop on the soft-stop path while still hard-gating real `FreezeManager` freezes and hardware faults.

- [x] **Step 3: Compile and run control tests**

Run:

```bash
cd /home/lin/code/hs_exoskeleton_v2
g++ -std=c++17 -I. tests/test_control.cpp control/AssistStateMachine.cpp control/FreezeManager.cpp control/GaitFeatureExtractor.cpp control/IntentDetector.cpp control/PhaseEstimator.cpp control/TorqueProfile.cpp control/StopDetector.cpp -o /tmp/hs_exo_test_control
/tmp/hs_exo_test_control
```

Expected: exit code 0.

## Task 4: Verify Python regression tests still pass

- [x] **Step 1: Run Python tests**

Run:

```bash
cd /home/lin/code/hs_exoskeleton_v2
python3 -m unittest tests/test_analyze_run.py tests/test_replay_curve.py
```

Expected: all tests pass.

## Self-review

- Spec coverage: implements a hip-motor-encoder-only stop detector, uses angle velocity only for stop decision, avoids static spread causing continued walking confidence, and pauses phase tracking during stop.
- Placeholder scan: no deferred implementation placeholders.
- Type consistency: `StopConfig`, `StopDetector`, `StopDecision`, and `ControlConfig.stop` are defined before use.


## Execution Notes

Completed in this session on 2026-05-29. Verification commands run:

```bash
g++ -std=c++17 -I. -I/home/lin/code/2026.4.14hs_exoskeleton/include tests/test_control.cpp control/AssistStateMachine.cpp control/FreezeManager.cpp control/GaitFeatureExtractor.cpp control/IntentDetector.cpp control/PhaseEstimator.cpp control/TorqueProfile.cpp control/StopDetector.cpp -o /tmp/hs_exo_test_control && /tmp/hs_exo_test_control
g++ -std=c++17 -I. -I/home/lin/code/2026.4.14hs_exoskeleton/include -c control/ExoController.cpp -o /tmp/ExoController.o
python3 -m unittest tests/test_analyze_run.py tests/test_replay_curve.py
```

All commands completed successfully.


## Follow-up Rename Note

Natural encoder stop is now represented separately as `AssistState::Stopping` and `AssistInputs::stop_requested`; `FreezeManager` remains a separate high-level freeze path that disables output immediately.
