// Encoder-only stop detector for hip exoskeleton stop handling.
// Uses only current left/right joint angular velocity and hysteresis timers.

#pragma once

#include "config/ControlConfig.h"

namespace exo {

struct ExoState;

struct StopDecision {
    bool stop_requested = false;
    bool phase_tracking_enabled = true;
    double average_abs_velocity_rad_s = 0.0;
};

class StopDetector {
public:
    explicit StopDetector(const StopConfig& config);

    StopDecision update(const ExoState& state, double dt_s);

private:
    StopConfig config_;
    bool initialized_ = false;
    bool stopped_ = false;
    double filtered_abs_velocity_rad_s_ = 0.0;
    double enter_timer_s_ = 0.0;
    double exit_timer_s_ = 0.0;
};

} // namespace exo
