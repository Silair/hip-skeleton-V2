#include "control/PhaseEstimator.h"

#include <algorithm>
#include <cmath>

#include "control/GaitFeatures.h"

namespace exo {

PhaseEstimator::PhaseEstimator(const PhaseConfig& config)
    : config_(config),
      oscillator_(3) {
    oscillator_.init(config_.ao_initial_frequency_hz);
}

double PhaseEstimator::wrapAngle(double angle_rad) {
    double wrapped = std::fmod(angle_rad, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped;
}

PhaseEstimate PhaseEstimator::update(const GaitFeatures& features, double dt_s, bool tracking_enabled) {
    now_s_ += dt_s;

    if (tracking_enabled) {
        // 将本控制周期内的信号变化分段喂给 AO，减小大步长带来的数值误差。
        const double internal_step = std::min(config_.ao_internal_step_s, std::max(1e-4, dt_s));
        double remaining = dt_s;
        double elapsed = 0.0;
        while (remaining > 1e-6) {
            const double step = std::min(internal_step, remaining);
            const double ratio = (elapsed + step) / std::max(dt_s, 1e-6);
            const double input = last_signal_rad_ + (features.filtered_phase_signal_rad - last_signal_rad_) * ratio;
            oscillator_.update(input, step);
            elapsed += step;
            remaining -= step;
        }
        last_signal_rad_ = features.filtered_phase_signal_rad;

        const double freq_hz = oscillator_.omega / (2.0 * M_PI);
        if (freq_hz < config_.ao_min_frequency_hz) {
            oscillator_.omega = config_.ao_min_frequency_hz * 2.0 * M_PI;
        } else if (freq_hz > config_.ao_max_frequency_hz) {
            oscillator_.omega = config_.ao_max_frequency_hz * 2.0 * M_PI;
        }
    }

    // 通过相位速度符号变化检测局部峰/谷，作为步态锚点候选。
    const bool positive_peak = last_phase_velocity_deg_s_ > 0.0 && features.signed_phase_velocity_deg_s <= 0.0;
    const bool negative_peak = last_phase_velocity_deg_s_ <= 0.0 && features.signed_phase_velocity_deg_s > 0.0;
    last_phase_velocity_deg_s_ = features.signed_phase_velocity_deg_s;

    bool anchor_detected = false;
    const double frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    const double period_s = frequency_hz > 1e-3 ? (1.0 / frequency_hz) : 2.0;
    const bool anchor_spacing_ok = (now_s_ - last_anchor_time_s_) > (config_.anchor_min_fraction_of_period * period_s);
    if ((positive_peak || negative_peak) && features.spread_deg > config_.peak_min_spread_deg && anchor_spacing_ok) {
        anchor_detected = true;
        last_anchor_time_s_ = now_s_;
        const double target_phase = positive_peak ? (M_PI / 2.0) : (3.0 * M_PI / 2.0);
        double error = target_phase - oscillator_.phi_GP;
        while (error > M_PI) error -= 2.0 * M_PI;
        while (error < -M_PI) error += 2.0 * M_PI;
        phase_offset_rad_ = 0.5 * phase_offset_rad_ + 0.5 * error;
    }

    if (!tracking_enabled) {
        phase_offset_rad_ *= std::exp(-dt_s * config_.anchor_decay_gain);
    }

    PhaseEstimate estimate{};
    estimate.phase_rad = wrapAngle(oscillator_.phi_GP + phase_offset_rad_);
    estimate.frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    estimate.amplitude_rad = std::abs(oscillator_.alpha[1]);
    estimate.ao_signal_estimate_rad = oscillator_.theta_IL_hat;
    estimate.ao_signal_error_rad = oscillator_.error_F;
    estimate.valid = estimate.frequency_hz >= config_.ao_min_frequency_hz * 0.5;
    estimate.anchor_detected = anchor_detected;
    return estimate;
}

} // namespace exo
