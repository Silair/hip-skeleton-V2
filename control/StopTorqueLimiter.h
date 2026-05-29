// Stopping-state torque limiter: withdraws existing torque toward zero without
// generating new phase-based peaks during natural stops.

#pragma once

#include "config/ControlConfig.h"
#include "control/TorqueProfile.h"

namespace exo {

class StopTorqueLimiter {
public:
    explicit StopTorqueLimiter(const StopConfig& config);

    TorqueCommand update(const TorqueCommand& previous_torque, double dt_s) const;

private:
    static double moveTowardZero(double value, double max_delta);

    StopConfig config_;
};

} // namespace exo
