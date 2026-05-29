#include "control/PhaseEstimator.h"

#include <algorithm>
#include <cmath>

#include "control/GaitFeatures.h"

namespace exo {

PhaseEstimator::PhaseEstimator(const PhaseConfig& config)
    : config_(config),
      oscillator_(3) {
    oscillator_.init(config_.ao_initial_frequency_hz);
    omega_target_rad_s_ = config_.ao_initial_frequency_hz * 2.0 * M_PI;
}

double PhaseEstimator::wrapAngle(double angle_rad) {
    double wrapped = std::fmod(angle_rad, 2.0 * M_PI);
    if (wrapped < 0.0) {
        wrapped += 2.0 * M_PI;
    }
    return wrapped;
}

double PhaseEstimator::clamp01(double value) {
    return std::clamp(value, 0.0, 1.0);
}

double PhaseEstimator::moveToward(double current, double target, double max_step) {
    if (target > current) {
        return std::min(target, current + max_step);
    }
    return std::max(target, current - max_step);
}

double PhaseEstimator::anchorFrequencyGainForState(const PhaseConfig& config, AssistState assist_state) {
    switch (assist_state) {
    case AssistState::Active:
        return config.anchor_frequency_gain;
    case AssistState::Ramp:
        return config.anchor_frequency_gain_ramp;
    default:
        return 0.0;
    }
}

void PhaseEstimator::clampOmegaToConfigLimits() {
    const double min_omega_rad_s = config_.ao_min_frequency_hz * 2.0 * M_PI;
    const double max_omega_rad_s = config_.ao_max_frequency_hz * 2.0 * M_PI;
    oscillator_.omega = std::clamp(oscillator_.omega, min_omega_rad_s, max_omega_rad_s);
    omega_target_rad_s_ = std::clamp(omega_target_rad_s_, min_omega_rad_s, max_omega_rad_s);
}

void PhaseEstimator::applyRateLimitedOmega(double dt_s) {
    const double max_omega_delta_rad_s = config_.max_omega_rate_rad_s2 * std::max(dt_s, 1e-6);
    oscillator_.omega = moveToward(oscillator_.omega, omega_target_rad_s_, max_omega_delta_rad_s);
    clampOmegaToConfigLimits();
}

PhaseEstimate PhaseEstimator::update(const GaitFeatures& features,
                                     double dt_s,
                                     bool tracking_enabled,
                                     AssistState assist_state,
                                     double stop_probability) {
    now_s_ += dt_s;
    if (!tracking_enabled && previous_tracking_enabled_) {
        reliable_anchor_count_since_tracking_enable_ = 0;
        has_frequency_anchor_ = false;
        omega_target_tracking_active_ = false;
    }
    previous_tracking_enabled_ = tracking_enabled;

    if (tracking_enabled) {
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
        clampOmegaToConfigLimits();
    }

    const double prev_velocity_deg_s = last_phase_velocity_deg_s_;
    const double curr_velocity_deg_s = features.signed_phase_velocity_deg_s;
    const double previous_velocity_abs_deg_s = std::abs(prev_velocity_deg_s);
    const double current_velocity_abs_deg_s = std::abs(curr_velocity_deg_s);

    const bool positive_peak = prev_velocity_deg_s > 0.0 && curr_velocity_deg_s <= 0.0;
    const bool negative_peak = prev_velocity_deg_s < 0.0 && curr_velocity_deg_s >= 0.0;
    const double peak_velocity_abs_deg_s = std::max(previous_velocity_abs_deg_s, current_velocity_abs_deg_s);
    const bool velocity_magnitude_ok = peak_velocity_abs_deg_s >= config_.anchor_min_velocity_deg_s;
    const bool reliable_peak =
        positive_peak && velocity_magnitude_ok && features.spread_deg > config_.anchor_min_spread_deg;
    const bool reliable_valley =
        negative_peak && velocity_magnitude_ok && features.spread_deg > config_.anchor_min_spread_deg;

    last_phase_velocity_deg_s_ = curr_velocity_deg_s;

    bool anchor_detected = false;
    bool anchor_frequency_updated = false;
    bool anchor_rejected = false;
    double anchor_measured_frequency_hz = 0.0;
    double anchor_confidence = 0.0;
    double omega_correction_hz = 0.0;
    const double frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    const double omega_before_rad_s = oscillator_.omega;

    const bool anchor_spacing_ok = (now_s_ - last_anchor_time_s_) > config_.anchor_refractory_s;
    const bool anchor_reliable = reliable_peak || reliable_valley;
    const bool anchor_signal_ok = anchor_spacing_ok && tracking_enabled;
    const bool stop_intent_blocks_update =
        stop_probability >= config_.anchor_update_disable_stop_probability;
    const bool allow_frequency_update_state =
        !stop_intent_blocks_update &&
        (assist_state == AssistState::Active || assist_state == AssistState::Ramp);

    if (anchor_reliable && anchor_signal_ok) {
        anchor_detected = true;
        last_anchor_time_s_ = now_s_;
        reliable_anchor_count_since_tracking_enable_ += 1;

        const AnchorType current_anchor_type = reliable_peak ? AnchorType::Peak : AnchorType::Valley;
        const double dt_anchor_s = now_s_ - last_frequency_anchor_time_s_;
        const bool warmup_done =
            reliable_anchor_count_since_tracking_enable_ > config_.reacquire_anchor_warmup_count;
        const bool interval_ok =
            dt_anchor_s >= config_.anchor_min_interval_s && dt_anchor_s <= config_.anchor_max_interval_s;
        const bool refractory_ok = dt_anchor_s >= config_.anchor_refractory_s;
        const bool type_ok = !has_frequency_anchor_ || last_frequency_anchor_type_ != current_anchor_type;

        if (config_.enable_anchor_frequency_update && allow_frequency_update_state && tracking_enabled &&
            has_frequency_anchor_ && warmup_done && interval_ok && refractory_ok && type_ok) {
            anchor_measured_frequency_hz = 0.5 / std::max(dt_anchor_s, 1e-6);
            const double predicted_half_period_s = 0.5 / std::max(frequency_hz, 1e-6);
            const double interval_ratio = dt_anchor_s / std::max(predicted_half_period_s, 1e-6);
            const double interval_score = clamp01(1.0 - std::abs(interval_ratio - 1.0) / 0.4);
            const double amplitude_score = clamp01(
                (features.spread_deg - config_.anchor_min_spread_deg) /
                std::max(config_.anchor_spread_margin_deg, 1e-6));
            const double velocity_score = clamp01(
                std::min(previous_velocity_abs_deg_s, current_velocity_abs_deg_s) /
                std::max(config_.anchor_velocity_reference_deg_s, 1e-6));
            anchor_confidence = 0.4 * interval_score + 0.3 * amplitude_score + 0.3 * velocity_score;

            const double state_frequency_gain = anchorFrequencyGainForState(config_, assist_state);
            const double effective_gain = state_frequency_gain * anchor_confidence;

            if (anchor_measured_frequency_hz >= config_.anchor_frequency_min_hz &&
                anchor_measured_frequency_hz <= config_.anchor_frequency_max_hz &&
                anchor_confidence >= config_.anchor_min_confidence && effective_gain > 1e-6) {
                const double current_frequency_hz = std::clamp(
                    frequency_hz, config_.ao_min_frequency_hz, config_.ao_max_frequency_hz);
                double limited_frequency_hz = std::clamp(
                    anchor_measured_frequency_hz,
                    config_.anchor_frequency_min_hz,
                    config_.anchor_frequency_max_hz);
                limited_frequency_hz = std::clamp(
                    limited_frequency_hz,
                    current_frequency_hz / std::max(config_.anchor_max_frequency_ratio, 1.0),
                    current_frequency_hz * std::max(config_.anchor_max_frequency_ratio, 1.0));
                const double corrected_frequency_hz = moveToward(
                    current_frequency_hz, limited_frequency_hz, config_.anchor_max_frequency_step_hz);
                const double target_frequency_hz =
                    (1.0 - effective_gain) * current_frequency_hz + effective_gain * corrected_frequency_hz;
                omega_target_rad_s_ = target_frequency_hz * 2.0 * M_PI;
                omega_target_tracking_active_ = true;
                applyRateLimitedOmega(dt_s);
                omega_correction_hz = (oscillator_.omega - omega_before_rad_s) / (2.0 * M_PI);
                anchor_frequency_updated = std::abs(oscillator_.omega - omega_before_rad_s) > 1e-9;
            } else {
                anchor_rejected = true;
            }
        } else if (config_.enable_anchor_frequency_update && has_frequency_anchor_ && anchor_reliable) {
            anchor_rejected = true;
        }

        last_frequency_anchor_time_s_ = now_s_;
        last_frequency_anchor_type_ = current_anchor_type;
        has_frequency_anchor_ = true;
    } else if (reliable_peak || reliable_valley) {
        anchor_rejected = true;
    }

    if (tracking_enabled && omega_target_tracking_active_) {
        applyRateLimitedOmega(dt_s);
        const double max_omega_delta_rad_s = config_.max_omega_rate_rad_s2 * std::max(dt_s, 1e-6);
        if (std::abs(oscillator_.omega - omega_target_rad_s_) <= max_omega_delta_rad_s) {
            omega_target_tracking_active_ = false;
            omega_target_rad_s_ = oscillator_.omega;
        }
    }

    PhaseEstimate estimate{};
    estimate.phase_rad = wrapAngle(oscillator_.phi_GP);
    estimate.frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    estimate.amplitude_rad = std::abs(oscillator_.alpha[1]);
    estimate.ao_signal_estimate_rad = oscillator_.theta_IL_hat;
    estimate.ao_signal_error_rad = oscillator_.error_F;
    estimate.valid = estimate.frequency_hz >= config_.ao_min_frequency_hz * 0.5;
    estimate.anchor_detected = anchor_detected;
    estimate.anchor_frequency_updated = anchor_frequency_updated;
    estimate.anchor_rejected = anchor_rejected;
    estimate.anchor_measured_frequency_hz = anchor_measured_frequency_hz;
    estimate.anchor_confidence = anchor_confidence;
    estimate.omega_correction_hz = omega_correction_hz;
    return estimate;
}

} // namespace exo
