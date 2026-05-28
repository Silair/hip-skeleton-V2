#include "control/IntentDetector.h"

#include <algorithm>

#include "control/GaitFeatures.h"

namespace exo {
namespace {

double saturate(double value) {
    return std::clamp(value, 0.0, 1.0);
}

// 将 value 线性映射到 [low, high] 之间再饱和到 [0,1]，用于把物理量变成可比分数。
double normalize(double value, double low, double high) {
    if (high <= low) {
        return value >= high ? 1.0 : 0.0;
    }
    return saturate((value - low) / (high - low));
}

} // namespace

IntentDetector::IntentDetector(const IntentConfig& config)
    : config_(config) {}

IntentEstimate IntentDetector::update(const GaitFeatures& features, double /*dt_s*/) {
    const double spread_score = normalize(features.spread_deg,
        config_.low_motion_spread_deg,
        config_.active_motion_spread_deg);
    const double velocity_score = normalize(features.phase_velocity_deg_s,
        config_.low_motion_velocity_deg_s,
        config_.active_motion_velocity_deg_s);
    const double amplitude_score = normalize(features.amplitude_rad,
        config_.low_motion_amplitude_rad,
        config_.active_motion_amplitude_rad);
    const double frequency_score = normalize(features.frequency_hz,
        config_.low_motion_frequency_hz,
        config_.active_motion_frequency_hz);

    const double raw_motion_confidence =
        0.35 * spread_score +
        0.30 * velocity_score +
        0.20 * amplitude_score +
        0.15 * frequency_score;

    if (smoothed_motion_confidence_ == 0.0 && raw_motion_confidence > 0.0) {
        smoothed_motion_confidence_ = raw_motion_confidence;
    } else {
        smoothed_motion_confidence_ =
            (1.0 - config_.smoothing_alpha) * smoothed_motion_confidence_ +
            config_.smoothing_alpha * raw_motion_confidence;
    }

    IntentEstimate estimate{};
    estimate.motion_confidence = saturate(smoothed_motion_confidence_);
    estimate.stop_probability = saturate(1.0 - estimate.motion_confidence);
    return estimate;
}

} // namespace exo
