#include "control/GaitFeatureExtractor.h"

#include <cmath>

#include "control/GaitFeatures.h"
#include "hardware/JointTypes.h"

namespace exo {
namespace {
constexpr double kRadToDeg = 57.2957795130823;
}

GaitFeatureExtractor::GaitFeatureExtractor(const PhaseConfig& config)
    : config_(config) {}

double GaitFeatureExtractor::alpha(double dt_s) const {
    const double tau = 1.0 / (2.0 * M_PI * config_.low_pass_cutoff_hz);
    return dt_s / (tau + dt_s);
}

GaitFeatures GaitFeatureExtractor::update(const ExoState& state, double dt_s) {
    // 用左右关节角之和作为标量步态信号（与旧系统一致的简化假设）。
    const double raw_phase_signal = state.left.position_rad + state.right.position_rad;
    if (!initialized_) {
        filtered_phase_signal_rad_ = raw_phase_signal;
        last_filtered_phase_signal_rad_ = raw_phase_signal;
        initialized_ = true;
    } else {
        last_filtered_phase_signal_rad_ = filtered_phase_signal_rad_;
        filtered_phase_signal_rad_ += alpha(dt_s) * (raw_phase_signal - filtered_phase_signal_rad_);
    }

    GaitFeatures features{};
    features.phase_signal_rad = raw_phase_signal;
    features.filtered_phase_signal_rad = filtered_phase_signal_rad_;
    // spread：滤波后信号绝对值的度数表征，粗略对应双腿开合程度。
    features.spread_deg = std::abs(filtered_phase_signal_rad_) * kRadToDeg;
    if (dt_s > 1e-6) {
        features.signed_phase_velocity_deg_s = (filtered_phase_signal_rad_ - last_filtered_phase_signal_rad_) * kRadToDeg / dt_s;
        features.phase_velocity_deg_s = std::abs(features.signed_phase_velocity_deg_s);
    }
    return features;
}

} // namespace exo
