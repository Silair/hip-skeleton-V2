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

bool PhaseEstimator::confirmPendingAnchor(const PhaseConfig& config,
                                          AnchorType type,
                                          double curr_velocity_deg_s,
                                          double spread_deg) {
    if (spread_deg <= config.anchor_min_spread_deg) {
        return false;
    }
    if (type == AnchorType::Peak) {
        return curr_velocity_deg_s <= -config.anchor_min_velocity_deg_s;
    }
    return curr_velocity_deg_s >= config.anchor_min_velocity_deg_s;
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

PhaseEstimator::AnchorFrequencyMeasurement PhaseEstimator::measureAnchorFrequency(
    const AnchorEventContext& ctx,
    AnchorType anchor_type) const {
    AnchorFrequencyMeasurement measurement{};
    if (!has_frequency_anchor_) {
        return measurement;
    }

    const double frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    const double dt_anchor_s = now_s_ - last_frequency_anchor_time_s_;
    const bool warmup_done =
        reliable_anchor_count_since_tracking_enable_ > config_.reacquire_anchor_warmup_count;
    const bool interval_ok =
        dt_anchor_s >= config_.anchor_min_interval_s && dt_anchor_s <= config_.anchor_max_interval_s;
    const bool refractory_ok = dt_anchor_s >= config_.anchor_refractory_s;
    const bool type_ok = last_frequency_anchor_type_ != anchor_type;

    if (!warmup_done) {
        measurement.rejection = AnchorRejectReason::Warmup;
        return measurement;
    }
    if (!interval_ok) {
        measurement.rejection = AnchorRejectReason::Interval;
        return measurement;
    }
    if (!refractory_ok) {
        measurement.rejection = AnchorRejectReason::Refractory;
        return measurement;
    }
    if (!type_ok) {
        measurement.rejection = AnchorRejectReason::AnchorType;
        return measurement;
    }

    measurement.measured_frequency_hz = 0.5 / std::max(dt_anchor_s, 1e-6);
    const double predicted_half_period_s = 0.5 / std::max(frequency_hz, 1e-6);
    const double interval_ratio = dt_anchor_s / std::max(predicted_half_period_s, 1e-6);
    const double interval_score = clamp01(1.0 - std::abs(interval_ratio - 1.0) / 0.4);
    const double amplitude_score = clamp01(
        (ctx.features.spread_deg - config_.anchor_min_spread_deg) /
        std::max(config_.anchor_spread_margin_deg, 1e-6));
    const double velocity_score = clamp01(
        std::min(ctx.previous_velocity_abs_deg_s, ctx.current_velocity_abs_deg_s) /
        std::max(config_.anchor_velocity_reference_deg_s, 1e-6));
    measurement.confidence = 0.4 * interval_score + 0.3 * amplitude_score + 0.3 * velocity_score;

    if (measurement.measured_frequency_hz < config_.anchor_frequency_min_hz ||
        measurement.measured_frequency_hz > config_.anchor_frequency_max_hz) {
        measurement.rejection = AnchorRejectReason::FrequencyRange;
        return measurement;
    }

    if (measurement.confidence < config_.anchor_min_confidence) {
        measurement.rejection = AnchorRejectReason::LowConfidence;
        return measurement;
    }

    measurement.ready = true;
    return measurement;
}

bool PhaseEstimator::applyAnchorFrequencyCorrection(const AnchorEventContext& ctx,
                                                    PhaseEstimate& estimate,
                                                    double omega_before_rad_s,
                                                    double measured_frequency_hz,
                                                    double confidence) {
    const double frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    const double state_frequency_gain = anchorFrequencyGainForState(config_, ctx.assist_state);
    const double effective_gain = state_frequency_gain * confidence;
    if (effective_gain <= 1e-6) {
        return false;
    }

    estimate.anchor_measured_frequency_hz = measured_frequency_hz;
    estimate.anchor_confidence = confidence;

    const double current_frequency_hz =
        std::clamp(frequency_hz, config_.ao_min_frequency_hz, config_.ao_max_frequency_hz);
    double limited_frequency_hz = std::clamp(
        measured_frequency_hz, config_.anchor_frequency_min_hz, config_.anchor_frequency_max_hz);
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
    applyRateLimitedOmega(ctx.dt_s);
    estimate.omega_correction_hz = (oscillator_.omega - omega_before_rad_s) / (2.0 * M_PI);
    estimate.anchor_frequency_updated = std::abs(oscillator_.omega - omega_before_rad_s) > 1e-9;
    return estimate.anchor_frequency_updated;
}

void PhaseEstimator::tryApplyDeferredFrequencyCorrection(const AnchorEventContext& ctx,
                                                         PhaseEstimate& estimate,
                                                         double omega_before_rad_s) {
    if (!config_.enable_tracking_deferred_frequency || !has_deferred_frequency_correction_) {
        return;
    }
    if (!config_.enable_anchor_frequency_update || !ctx.tracking_enabled) {
        return;
    }
    if (ctx.stop_probability >= config_.anchor_update_disable_stop_probability) {
        has_deferred_frequency_correction_ = false;
        return;
    }
    if (ctx.assist_state != AssistState::Active && ctx.assist_state != AssistState::Ramp) {
        return;
    }

    if (applyAnchorFrequencyCorrection(
            ctx, estimate, omega_before_rad_s, deferred_measured_frequency_hz_, deferred_confidence_)) {
        estimate.anchor_detected = true;
        estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::None);
    }
    has_deferred_frequency_correction_ = false;
}

void PhaseEstimator::processAnchorEvent(const AnchorEventContext& ctx,
                                        AnchorType anchor_type,
                                        PhaseEstimate& estimate,
                                        double omega_before_rad_s) {
    estimate.anchor_detected = true;
    estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::None);
    last_anchor_time_s_ = now_s_;
    reliable_anchor_count_since_tracking_enable_ += 1;

    const bool stop_intent_blocks_update =
        ctx.stop_probability >= config_.anchor_update_disable_stop_probability;
    const bool allow_frequency_update_state =
        !stop_intent_blocks_update &&
        (ctx.assist_state == AssistState::Active || ctx.assist_state == AssistState::Ramp);

    if (!config_.enable_anchor_frequency_update || !ctx.tracking_enabled) {
        last_frequency_anchor_time_s_ = now_s_;
        last_frequency_anchor_type_ = anchor_type;
        has_frequency_anchor_ = true;
        return;
    }

    if (stop_intent_blocks_update) {
        estimate.anchor_rejected = true;
        estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::StopIntent);
        has_deferred_frequency_correction_ = false;
    } else if (!has_frequency_anchor_) {
        // First anchor: establish timing only.
    } else {
        const AnchorFrequencyMeasurement measurement = measureAnchorFrequency(ctx, anchor_type);
        if (!measurement.ready) {
            estimate.anchor_rejected = true;
            estimate.anchor_reject_reason = static_cast<int>(measurement.rejection);
            if (measurement.rejection != AnchorRejectReason::None) {
                estimate.anchor_measured_frequency_hz = measurement.measured_frequency_hz;
                estimate.anchor_confidence = measurement.confidence;
            }
        } else if (!allow_frequency_update_state) {
            estimate.anchor_measured_frequency_hz = measurement.measured_frequency_hz;
            estimate.anchor_confidence = measurement.confidence;
            if (config_.enable_tracking_deferred_frequency &&
                ctx.assist_state == AssistState::Tracking) {
                deferred_measured_frequency_hz_ = measurement.measured_frequency_hz;
                deferred_confidence_ = measurement.confidence;
                has_deferred_frequency_correction_ = true;
            }
            estimate.anchor_rejected = true;
            estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::AssistState);
        } else if (!applyAnchorFrequencyCorrection(
                       ctx,
                       estimate,
                       omega_before_rad_s,
                       measurement.measured_frequency_hz,
                       measurement.confidence)) {
            estimate.anchor_rejected = true;
            estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::LowConfidence);
        }
    }

    last_frequency_anchor_time_s_ = now_s_;
    last_frequency_anchor_type_ = anchor_type;
    has_frequency_anchor_ = true;
}

void PhaseEstimator::resetStartupPriorState() {
    last_motion_confidence_ = 0.0;
    last_spread_deg_ = 0.0;
    same_sign_velocity_frames_ = 0;
    spread_increase_frames_ = 0;
    velocity_sign_ = 0;
    last_zero_cross_time_s_ = -10.0;
    startup_frequency_prior_hz_ = 0.0;
    startup_confidence_ = 0.0;
    has_startup_prior_ = false;
    startup_prior_from_zero_cross_ = false;
    startup_prior_applied_ = false;
    startup_tracking_enter_time_s_ = -10.0;
}

void PhaseEstimator::updateStartupPrior(const GaitFeatures& features,
                                         AssistState assist_state,
                                         double stop_probability,
                                         double motion_confidence,
                                         bool tracking_enabled,
                                         bool positive_peak,
                                         bool negative_peak,
                                         PhaseEstimate& estimate) {
    if (!config_.enable_startup_frequency_prior || !tracking_enabled) {
        return;
    }

    const bool startup_state =
        assist_state == AssistState::Tracking || assist_state == AssistState::Ramp;
    const bool stop_ok = stop_probability < config_.startup_prior_max_stop_probability;
    const bool motion_rising =
        motion_confidence >= last_motion_confidence_ + config_.startup_prior_motion_confidence_rise;
    const bool motion_ok = motion_confidence >= config_.startup_prior_min_motion_confidence;
    const bool motion_buildup_ok = motion_ok && (motion_rising || motion_confidence >= 0.52);

    if (startup_state && tracking_enabled && startup_tracking_enter_time_s_ < -5.0) {
        startup_tracking_enter_time_s_ = now_s_;
    }
    if (!startup_state || !stop_ok || startup_prior_applied_) {
        last_motion_confidence_ = motion_confidence;
        last_spread_deg_ = features.spread_deg;
        return;
    }

    const double curr_velocity_deg_s = features.signed_phase_velocity_deg_s;
    int sign = 0;
    if (curr_velocity_deg_s > config_.startup_prior_min_velocity_deg_s) {
        sign = 1;
    } else if (curr_velocity_deg_s < -config_.startup_prior_min_velocity_deg_s) {
        sign = -1;
    }
    if (sign != 0 && sign == velocity_sign_) {
        same_sign_velocity_frames_ += 1;
    } else if (sign != 0) {
        velocity_sign_ = sign;
        same_sign_velocity_frames_ = 1;
    } else {
        same_sign_velocity_frames_ = 0;
        velocity_sign_ = 0;
    }

    if (features.spread_deg > last_spread_deg_ + 1e-3) {
        spread_increase_frames_ += 1;
    } else if (features.spread_deg < last_spread_deg_ - 1e-3) {
        spread_increase_frames_ = 0;
    }

    if (positive_peak || negative_peak) {
        if (last_zero_cross_time_s_ > -5.0) {
            const double half_period_s = now_s_ - last_zero_cross_time_s_;
            if (half_period_s >= config_.anchor_min_interval_s && half_period_s <= config_.anchor_max_interval_s) {
                startup_frequency_prior_hz_ = std::clamp(
                    0.5 / half_period_s,
                    config_.startup_prior_frequency_min_hz,
                    config_.startup_prior_frequency_max_hz);
                startup_prior_from_zero_cross_ = true;
            }
        }
        last_zero_cross_time_s_ = now_s_;
    }

    if (startup_prior_from_zero_cross_ &&
        startup_frequency_prior_hz_ >= config_.startup_prior_frequency_min_hz) {
        has_startup_prior_ = true;
    }

    const bool buildup_ok = motion_buildup_ok &&
                            same_sign_velocity_frames_ >= config_.startup_prior_same_sign_frames &&
                            spread_increase_frames_ >= config_.startup_prior_spread_increase_frames &&
                            features.spread_deg >= config_.anchor_min_spread_deg;

    if (buildup_ok) {
        estimate.startup_prior_candidate = true;
        const double sign_score = clamp01(
            static_cast<double>(same_sign_velocity_frames_) /
            std::max(config_.startup_prior_same_sign_frames, 1));
        const double spread_score = clamp01(
            static_cast<double>(spread_increase_frames_) /
            std::max(config_.startup_prior_spread_increase_frames, 1));
        const double motion_score = clamp01(
            (motion_confidence - config_.startup_prior_min_motion_confidence) /
            std::max(1.0 - config_.startup_prior_min_motion_confidence, 1e-6));
        startup_confidence_ = 0.35 * sign_score + 0.35 * spread_score + 0.30 * motion_score;

        if (has_startup_prior_) {
            estimate.startup_prior_valid = true;
            estimate.startup_prior_frequency_hz = startup_frequency_prior_hz_;
            estimate.startup_prior_confidence = startup_confidence_;
        }
    }

    last_motion_confidence_ = motion_confidence;
    last_spread_deg_ = features.spread_deg;
}

void PhaseEstimator::tryApplyStartupPrior(const AnchorEventContext& ctx,
                                          PhaseEstimate& estimate,
                                          double omega_before_rad_s) {
    if (!config_.enable_startup_frequency_prior || !has_startup_prior_ || startup_prior_applied_) {
        return;
    }
    if (!ctx.tracking_enabled) {
        return;
    }
    const double apply_confidence =
        startup_prior_from_zero_cross_
            ? std::max(startup_confidence_, config_.startup_prior_min_confidence_to_apply)
            : startup_confidence_;
    if (apply_confidence < config_.startup_prior_min_confidence_to_apply) {
        return;
    }
    if (now_s_ - startup_tracking_enter_time_s_ < config_.startup_prior_min_apply_time_s) {
        return;
    }
    if (ctx.stop_probability >= config_.startup_prior_max_stop_probability) {
        return;
    }
    if (ctx.assist_state == AssistState::Stopping || ctx.assist_state == AssistState::Frozen ||
        ctx.assist_state == AssistState::Fault) {
        return;
    }

    const bool in_ramp_or_active =
        ctx.assist_state == AssistState::Ramp || ctx.assist_state == AssistState::Active;
    const bool in_tracking_apply = config_.startup_prior_apply_during_tracking && ctx.tracking_enabled;
    if (!in_ramp_or_active && !in_tracking_apply) {
        return;
    }
    if (!in_ramp_or_active && !startup_prior_from_zero_cross_) {
        return;
    }

    const double current_frequency_hz =
        std::clamp(std::abs(oscillator_.omega) / (2.0 * M_PI), config_.ao_min_frequency_hz, config_.ao_max_frequency_hz);
    double effective_gain = config_.startup_prior_gain * apply_confidence;
    if (!in_ramp_or_active) {
        effective_gain *= config_.startup_prior_tracking_gain_scale;
    }
    if (effective_gain <= 1e-6) {
        return;
    }

    const double target_frequency_hz = std::clamp(
        startup_frequency_prior_hz_,
        config_.startup_prior_frequency_min_hz,
        config_.startup_prior_frequency_max_hz);
    const double blended_frequency_hz =
        (1.0 - effective_gain) * current_frequency_hz + effective_gain * target_frequency_hz;
    omega_target_rad_s_ = blended_frequency_hz * 2.0 * M_PI;
    omega_target_tracking_active_ = true;
    applyRateLimitedOmega(ctx.dt_s);

    estimate.startup_prior_valid = true;
    estimate.startup_prior_frequency_hz = startup_frequency_prior_hz_;
    estimate.startup_prior_confidence = startup_confidence_;
    estimate.startup_prior_applied = true;
    estimate.omega_correction_hz = (oscillator_.omega - omega_before_rad_s) / (2.0 * M_PI);
    startup_prior_applied_ = true;
}

PhaseEstimate PhaseEstimator::update(const GaitFeatures& features,
                                     double dt_s,
                                     bool tracking_enabled,
                                     AssistState assist_state,
                                     double stop_probability,
                                     double motion_confidence) {
    now_s_ += dt_s;
    if (!tracking_enabled && previous_tracking_enabled_) {
        reliable_anchor_count_since_tracking_enable_ = 0;
        has_frequency_anchor_ = false;
        omega_target_tracking_active_ = false;
        has_pending_anchor_ = false;
        has_deferred_frequency_correction_ = false;
        resetStartupPriorState();
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

    PhaseEstimate estimate{};
    const double omega_before_rad_s = oscillator_.omega;
    const bool anchor_spacing_ok = (now_s_ - last_anchor_time_s_) > config_.anchor_refractory_s;
    const bool anchor_candidate = reliable_peak || reliable_valley;
    const bool anchor_signal_ok = anchor_spacing_ok && tracking_enabled;

    const AnchorEventContext ctx{
        features,
        dt_s,
        assist_state,
        stop_probability,
        prev_velocity_deg_s,
        curr_velocity_deg_s,
        previous_velocity_abs_deg_s,
        current_velocity_abs_deg_s,
        tracking_enabled,
    };

    updateStartupPrior(
        features,
        assist_state,
        stop_probability,
        motion_confidence,
        tracking_enabled,
        positive_peak,
        negative_peak,
        estimate);

    tryApplyDeferredFrequencyCorrection(ctx, estimate, omega_before_rad_s);
    tryApplyStartupPrior(ctx, estimate, omega_before_rad_s);

    if (config_.anchor_confirm_delay_frames > 0 && has_pending_anchor_) {
        if (confirmPendingAnchor(config_, pending_anchor_type_, curr_velocity_deg_s, features.spread_deg)) {
            AnchorEventContext confirm_ctx = ctx;
            confirm_ctx.prev_velocity_deg_s = pending_prev_velocity_deg_s_;
            confirm_ctx.previous_velocity_abs_deg_s = std::abs(pending_prev_velocity_deg_s_);
            confirm_ctx.current_velocity_abs_deg_s = pending_peak_velocity_abs_deg_s_;
            processAnchorEvent(confirm_ctx, pending_anchor_type_, estimate, omega_before_rad_s);
        } else {
            estimate.anchor_rejected = true;
            estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::ConfirmFailed);
        }
        has_pending_anchor_ = false;
    }

    if (!estimate.anchor_detected && config_.anchor_confirm_delay_frames > 0 && anchor_candidate && anchor_signal_ok) {
        estimate.anchor_candidate = true;
        has_pending_anchor_ = true;
        pending_anchor_type_ = reliable_peak ? AnchorType::Peak : AnchorType::Valley;
        pending_prev_velocity_deg_s_ = prev_velocity_deg_s;
        pending_peak_velocity_abs_deg_s_ = peak_velocity_abs_deg_s;
    } else if (!estimate.anchor_detected && anchor_candidate && anchor_signal_ok) {
        estimate.anchor_candidate = true;
        const AnchorType anchor_type = reliable_peak ? AnchorType::Peak : AnchorType::Valley;
        processAnchorEvent(ctx, anchor_type, estimate, omega_before_rad_s);
    } else if (anchor_candidate) {
        estimate.anchor_rejected = true;
        estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::Refractory);
    } else if (positive_peak || negative_peak) {
        estimate.anchor_rejected = true;
        estimate.anchor_reject_reason = static_cast<int>(AnchorRejectReason::UnreliableSignal);
    }

    if (tracking_enabled && omega_target_tracking_active_) {
        applyRateLimitedOmega(dt_s);
        const double max_omega_delta_rad_s = config_.max_omega_rate_rad_s2 * std::max(dt_s, 1e-6);
        if (std::abs(oscillator_.omega - omega_target_rad_s_) <= max_omega_delta_rad_s) {
            omega_target_tracking_active_ = false;
            omega_target_rad_s_ = oscillator_.omega;
        }
    }

    estimate.phase_rad = wrapAngle(oscillator_.phi_GP);
    estimate.frequency_hz = std::abs(oscillator_.omega) / (2.0 * M_PI);
    estimate.amplitude_rad = std::abs(oscillator_.alpha[1]);
    estimate.ao_signal_estimate_rad = oscillator_.theta_IL_hat;
    estimate.ao_signal_error_rad = oscillator_.error_F;
    estimate.valid = estimate.frequency_hz >= config_.ao_min_frequency_hz * 0.5;
    previous_assist_state_ = assist_state;
    return estimate;
}

} // namespace exo
