#include "control/StopTorqueLimiter.h"

#include <algorithm>
#include <cmath>

namespace exo {

StopTorqueLimiter::StopTorqueLimiter(const StopConfig& config)
    : config_(config) {}

double StopTorqueLimiter::moveTowardZero(double value, double max_delta) {
    const double step = std::max(0.0, max_delta);
    if (std::abs(value) <= step) {
        return 0.0;
    }
    return value > 0.0 ? value - step : value + step;
}

TorqueCommand StopTorqueLimiter::update(const TorqueCommand& previous_torque, double dt_s) const {
    const double max_delta = std::max(0.0, config_.max_stop_torque_rate_nm_s) * std::max(0.0, dt_s);

    TorqueCommand command{};
    command.left_nm = moveTowardZero(previous_torque.left_nm, max_delta);
    command.right_nm = moveTowardZero(previous_torque.right_nm, max_delta);
    return command;
}

} // namespace exo
