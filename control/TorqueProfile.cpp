#include "control/TorqueProfile.h"

#include <algorithm>
#include <cmath>

namespace exo {
namespace {

double saturate(double value) {
    return std::clamp(value, 0.0, 1.0);
}

double clampTorque(double value, double limit) {
    return std::clamp(value, -limit, limit);
}

} // namespace

TorqueProfile::TorqueProfile(const TorqueConfig& config)
    : config_(config) {}

TorqueCommand TorqueProfile::compute(double phase_rad, double frequency_hz, double torque_scale, bool allow_output) const {
    TorqueCommand command{};
    if (!allow_output || torque_scale <= 0.0) {
        return command;
    }

    const double denom = std::max(1e-9, config_.gain_max_frequency_hz - config_.gain_min_frequency_hz);
    const double freq_ratio = saturate((frequency_hz - config_.gain_min_frequency_hz) / denom);
    const double base_gain = config_.base_gain_min_nm +
        (config_.base_gain_max_nm - config_.base_gain_min_nm) * freq_ratio;
    const double gain = base_gain * torque_scale;

    // 相位超前：在估计步态相位上叠加 lead_angle，使力矩略早于机械需要。
    const double shifted_phase = phase_rad + config_.lead_angle_rad;
    command.left_nm = gain * std::max(0.0, -std::cos(shifted_phase));
    command.right_nm = gain * std::max(0.0, std::cos(shifted_phase));

    command.left_nm = clampTorque(command.left_nm, config_.max_torque_nm);
    command.right_nm = clampTorque(command.right_nm, config_.max_torque_nm);
    return command;
}

} // namespace exo
