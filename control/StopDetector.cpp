#include "control/StopDetector.h"

#include <algorithm>
#include <cmath>

#include "hardware/JointTypes.h"

namespace exo {

StopDetector::StopDetector(const StopConfig& config)
    : config_(config) {}

StopDecision StopDetector::update(const ExoState& state, double dt_s) {
    const double raw_abs_velocity =
        0.5 * (std::abs(state.left.velocity_rad_s) + std::abs(state.right.velocity_rad_s));
    const double alpha = std::clamp(config_.velocity_filter_alpha, 0.0, 1.0);

    if (!initialized_) {
        filtered_abs_velocity_rad_s_ = raw_abs_velocity;
        initialized_ = true;
    } else {
        filtered_abs_velocity_rad_s_ =
            (1.0 - alpha) * filtered_abs_velocity_rad_s_ + alpha * raw_abs_velocity;
    }

    if (!stopped_) {
        if (filtered_abs_velocity_rad_s_ <= config_.velocity_threshold_rad_s) {
            enter_timer_s_ += dt_s;
            if (enter_timer_s_ >= config_.enter_hold_seconds) {
                stopped_ = true;
                enter_timer_s_ = 0.0;
                exit_timer_s_ = 0.0;
            }
        } else {
            enter_timer_s_ = 0.0;
        }
    } else {
        if (filtered_abs_velocity_rad_s_ >= config_.exit_velocity_threshold_rad_s) {
            exit_timer_s_ += dt_s;
            if (exit_timer_s_ >= config_.exit_hold_seconds) {
                stopped_ = false;
                exit_timer_s_ = 0.0;
                enter_timer_s_ = 0.0;
            }
        } else {
            exit_timer_s_ = 0.0;
        }
    }

    StopDecision decision{};
    decision.stop_requested = stopped_;
    decision.phase_tracking_enabled = !stopped_;
    decision.average_abs_velocity_rad_s = filtered_abs_velocity_rad_s_;
    return decision;
}

} // namespace exo
