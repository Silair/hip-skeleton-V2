// 纯控制链回归：意图、力矩对称、助力状态机、冻结滞回、特征提取、相位跟踪（无 CAN/电机）。

#include <cassert>
#include <cmath>

#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatureExtractor.h"
#include "control/GaitFeatures.h"
#include "control/IntentDetector.h"
#include "control/PhaseEstimator.h"
#include "control/StopDetector.h"
#include "control/StopTorqueLimiter.h"
#include "control/TorqueProfile.h"
#include "config/ControlConfig.h"
#include "hardware/JointTypes.h"

namespace {

void fill_sine_features(exo::GaitFeatures& features, double t_s, double frequency_hz) {
    const double phase = 2.0 * M_PI * frequency_hz * t_s;
    features.filtered_phase_signal_rad = std::sin(phase);
    features.spread_deg = std::abs(features.filtered_phase_signal_rad) * 57.2957795130823;
    features.signed_phase_velocity_deg_s =
        2.0 * M_PI * frequency_hz * std::cos(phase) * 57.2957795130823;
    features.phase_velocity_deg_s = std::abs(features.signed_phase_velocity_deg_s);
}

void test_intent_detector_prefers_motion_over_stop() {
    exo::ControlConfig config;
    exo::IntentDetector detector(config.intent);

    exo::GaitFeatures features{};
    features.spread_deg = 32.0;
    features.phase_velocity_deg_s = 85.0;
    features.amplitude_rad = 0.65;
    features.frequency_hz = 0.95;

    exo::IntentEstimate estimate = detector.update(features, 0.02);
    assert(estimate.motion_confidence > 0.7);
    assert(estimate.stop_probability < 0.3);
}

void test_intent_detector_prefers_stop_when_quiet() {
    exo::ControlConfig config;
    exo::IntentDetector detector(config.intent);

    exo::GaitFeatures features{};
    features.spread_deg = 1.0;
    features.phase_velocity_deg_s = 1.0;
    features.amplitude_rad = 0.01;
    features.frequency_hz = 0.05;

    exo::IntentEstimate estimate{};
    for (int i = 0; i < 20; ++i) {
        estimate = detector.update(features, 0.02);
    }

    assert(estimate.motion_confidence < 0.2);
    assert(estimate.stop_probability > 0.8);
}

void test_torque_profile_is_phase_symmetric() {
    exo::ControlConfig config;
    exo::TorqueProfile profile(config.torque);

    exo::TorqueCommand right = profile.compute(0.0, 0.95, 1.0, true);
    exo::TorqueCommand left = profile.compute(M_PI, 0.95, 1.0, true);

    assert(right.right_nm > 0.0);
    assert(right.left_nm == 0.0);
    assert(left.left_nm > 0.0);
    assert(left.right_nm == 0.0);
    assert(std::abs(right.right_nm - left.left_nm) < 1e-9);
}

void test_assist_state_machine_reaches_active_after_warmup() {
    exo::ControlConfig config;
    exo::AssistStateMachine machine(config.assist);

    exo::AssistInputs inputs{};
    inputs.motion_confidence = 0.9;
    inputs.phase_valid = true;
    inputs.freeze_requested = false;
    inputs.faulted = false;
    inputs.anchor_detected = false;

    exo::AssistOutput output{};
    output = machine.update(inputs, 0.02);
    assert(output.state == exo::AssistState::Tracking);

    inputs.anchor_detected = true;
    for (int i = 0; i < 4; ++i) {
        output = machine.update(inputs, 0.02);
        inputs.anchor_detected = false;
        output = machine.update(inputs, 0.02);
        inputs.anchor_detected = true;
    }

    assert(output.state == exo::AssistState::Active || output.state == exo::AssistState::Ramp);
    assert(output.torque_scale > 0.0);
}

void test_freeze_manager_uses_hysteresis() {
    exo::ControlConfig config;
    exo::FreezeManager manager(config.freeze);

    exo::IntentEstimate walk{};
    walk.motion_confidence = 0.85;
    walk.stop_probability = 0.10;
    manager.update(walk, 0.02);

    exo::IntentEstimate stop{};
    stop.motion_confidence = 0.05;
    stop.stop_probability = 0.95;

    exo::FreezeDecision decision{};
    for (int i = 0; i < 20; ++i) {
        decision = manager.update(stop, 0.02);
    }
    assert(decision.freeze_requested);

    for (int i = 0; i < 20; ++i) {
        decision = manager.update(walk, 0.02);
    }
    assert(!decision.freeze_requested);
}

void test_gait_feature_extractor_generates_velocity() {
    exo::ControlConfig config;
    exo::GaitFeatureExtractor extractor(config.phase);

    exo::ExoState state{};
    state.left.position_rad = 0.0;
    state.right.position_rad = 0.0;
    extractor.update(state, 0.02);

    state.left.position_rad = 0.3;
    state.right.position_rad = 0.2;
    exo::GaitFeatures features = extractor.update(state, 0.02);

    assert(features.spread_deg > 0.0);
    assert(features.phase_velocity_deg_s > 0.0);
    assert(features.signed_phase_velocity_deg_s > 0.0);

    state.left.position_rad = -0.3;
    state.right.position_rad = -0.2;
    features = extractor.update(state, 0.02);

    assert(features.phase_velocity_deg_s > 0.0);
    assert(features.signed_phase_velocity_deg_s < 0.0);
}

void test_phase_estimator_tracks_signal() {
    exo::ControlConfig config;
    exo::PhaseEstimator estimator(config.phase);
    exo::PhaseEstimate estimate{};
    exo::GaitFeatures features{};

    for (int i = 0; i < 200; ++i) {
        fill_sine_features(features, i * 0.02, 0.9);
        estimate = estimator.update(features, 0.02, true, exo::AssistState::Active);
    }

    assert(estimate.valid);
    assert(estimate.frequency_hz > 0.5);
}

void test_phase_estimator_applies_anchor_frequency_updates_during_active_walking() {
    exo::ControlConfig config;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    int anchor_update_count = 0;
    for (int i = 0; i < 400; ++i) {
        fill_sine_features(features, i * dt, 0.9);
        const exo::PhaseEstimate estimate =
            estimator.update(features, dt, true, exo::AssistState::Active, 0.0);
        anchor_update_count += estimate.anchor_frequency_updated ? 1 : 0;
    }

    assert(anchor_update_count >= 3);
}

void test_phase_estimator_no_anchor_frequency_update_when_stop_probability_high() {
    exo::ControlConfig config;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};
    exo::PhaseEstimate estimate{};

    const double dt = 0.02;
    for (int i = 0; i < 300; ++i) {
        fill_sine_features(features, i * dt, 0.9);
        estimator.update(features, dt, true, exo::AssistState::Active, 0.0);
    }

    fill_sine_features(features, 300.0 * dt, 0.9);
    estimate = estimator.update(features, dt, true, exo::AssistState::Active, 0.9);
    assert(!estimate.anchor_frequency_updated);
}

void test_phase_estimator_delayed_confirm_rejects_single_frame_spike() {
    exo::ControlConfig config;
    config.phase.anchor_confirm_delay_frames = 1;
    config.phase.anchor_refractory_s = 0.0;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    features.filtered_phase_signal_rad = 0.0;
    features.spread_deg = 20.0;
    features.signed_phase_velocity_deg_s = 20.0;
    estimator.update(features, 0.02, true, exo::AssistState::Active, 0.0);

    features.signed_phase_velocity_deg_s = -20.0;
    const exo::PhaseEstimate pending = estimator.update(features, 0.02, true, exo::AssistState::Active, 0.0);
    assert(!pending.anchor_detected);

    features.signed_phase_velocity_deg_s = 20.0;
    const exo::PhaseEstimate failed = estimator.update(features, 0.02, true, exo::AssistState::Active, 0.0);
    assert(failed.anchor_rejected);
    assert(failed.anchor_reject_reason == static_cast<int>(exo::AnchorRejectReason::ConfirmFailed));
}

void test_phase_estimator_builds_startup_prior_from_zero_cross() {
    exo::ControlConfig config;
    config.phase.enable_startup_frequency_prior = true;
    config.phase.startup_prior_same_sign_frames = 2;
    config.phase.startup_prior_spread_increase_frames = 2;
    config.phase.anchor_confirm_delay_frames = 0;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    bool saw_valid_prior = false;
    for (int i = 0; i < 100; ++i) {
        fill_sine_features(features, i * dt, 0.6);
        features.spread_deg = std::max(features.spread_deg, 10.0 + i * 0.2);
        const exo::PhaseEstimate estimate =
            estimator.update(features, dt, true, exo::AssistState::Tracking, 0.1, 0.52);
        if (estimate.startup_prior_valid && estimate.startup_prior_frequency_hz > 0.5) {
            saw_valid_prior = true;
            break;
        }
    }
    assert(saw_valid_prior);
}

void test_phase_estimator_applies_startup_prior_on_ramp_entry() {
    exo::ControlConfig config;
    config.phase.anchor_confirm_delay_frames = 0;
    config.phase.enable_startup_frequency_prior = true;
    config.phase.startup_prior_same_sign_frames = 3;
    config.phase.startup_prior_spread_increase_frames = 3;
    config.phase.startup_prior_gain = 0.25;
    config.phase.startup_prior_min_apply_time_s = 0.0;
    config.phase.startup_prior_apply_during_tracking = false;
    config.phase.startup_prior_min_confidence_to_apply = 0.0;
    config.phase.anchor_confirm_delay_frames = 0;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    exo::PhaseEstimate last{};
    for (int i = 0; i < 100; ++i) {
        fill_sine_features(features, i * dt, 0.6);
        features.spread_deg = std::max(features.spread_deg, 8.0 + i * 0.2);
        last = estimator.update(features, dt, true, exo::AssistState::Tracking, 0.1, 0.52);
    }
    assert(last.startup_prior_valid);
    assert(last.startup_prior_frequency_hz >= config.phase.startup_prior_frequency_min_hz);

    const exo::PhaseEstimate ramp_estimate =
        estimator.update(features, dt, true, exo::AssistState::Ramp, 0.1, 0.55);
    assert(ramp_estimate.startup_prior_applied || last.startup_prior_applied);
    assert(ramp_estimate.startup_prior_frequency_hz >= config.phase.startup_prior_frequency_min_hz ||
           last.startup_prior_frequency_hz >= config.phase.startup_prior_frequency_min_hz);
}

void test_phase_estimator_startup_prior_skipped_when_stop_probability_high() {
    exo::ControlConfig config;
    config.phase.enable_startup_frequency_prior = true;
    config.phase.startup_prior_same_sign_frames = 2;
    config.phase.startup_prior_spread_increase_frames = 2;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    for (int i = 0; i < 6; ++i) {
        features.spread_deg = 8.0 + i * 4.0;
        features.signed_phase_velocity_deg_s = 40.0;
        estimator.update(features, dt, true, exo::AssistState::Tracking, 0.1, 0.48 + i * 0.03);
    }

    const exo::PhaseEstimate ramp_estimate =
        estimator.update(features, dt, true, exo::AssistState::Ramp, 0.9, 0.55);
    assert(!ramp_estimate.startup_prior_applied);
}

void test_phase_estimator_applies_deferred_frequency_on_ramp_after_tracking() {
    exo::ControlConfig config;
    config.phase.anchor_confirm_delay_frames = 0;
    config.phase.anchor_refractory_s = 0.0;
    config.phase.reacquire_anchor_warmup_count = 0;
    config.phase.enable_tracking_deferred_frequency = true;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    for (int i = 0; i < 120; ++i) {
        fill_sine_features(features, i * dt, 0.6);
        estimator.update(features, dt, true, exo::AssistState::Tracking, 0.0);
    }

    bool saw_tracking_reject = false;
    for (int i = 120; i < 220; ++i) {
        fill_sine_features(features, i * dt, 0.6);
        const exo::PhaseEstimate estimate =
            estimator.update(features, dt, true, exo::AssistState::Tracking, 0.0);
        if (estimate.anchor_rejected &&
            estimate.anchor_reject_reason == static_cast<int>(exo::AnchorRejectReason::AssistState) &&
            estimate.anchor_confidence >= config.phase.anchor_min_confidence) {
            saw_tracking_reject = true;
        }
    }
    assert(saw_tracking_reject);

    bool saw_deferred_update = false;
    for (int i = 220; i < 240; ++i) {
        fill_sine_features(features, i * dt, 0.6);
        const exo::PhaseEstimate estimate =
            estimator.update(features, dt, true, exo::AssistState::Ramp, 0.0);
        if (estimate.anchor_frequency_updated) {
            saw_deferred_update = true;
            break;
        }
    }
    assert(saw_deferred_update);
}

void test_phase_estimator_reports_low_confidence_reject_reason() {
    exo::ControlConfig config;
    config.phase.anchor_confirm_delay_frames = 0;
    config.phase.anchor_refractory_s = 0.0;
    config.phase.reacquire_anchor_warmup_count = 0;
    config.phase.anchor_min_confidence = 0.99;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};

    const double dt = 0.02;
    for (int i = 0; i < 80; ++i) {
        fill_sine_features(features, i * dt, 0.9);
        estimator.update(features, dt, true, exo::AssistState::Active, 0.0);
    }

    bool saw_low_confidence = false;
    for (int i = 80; i < 200; ++i) {
        fill_sine_features(features, i * dt, 0.9);
        const exo::PhaseEstimate estimate =
            estimator.update(features, dt, true, exo::AssistState::Active, 0.0);
        if (estimate.anchor_rejected &&
            estimate.anchor_reject_reason == static_cast<int>(exo::AnchorRejectReason::LowConfidence)) {
            saw_low_confidence = true;
            break;
        }
    }
    assert(saw_low_confidence);
}

void test_phase_estimator_no_anchor_frequency_update_when_stopping() {
    exo::ControlConfig config;
    exo::PhaseEstimator estimator(config.phase);
    exo::GaitFeatures features{};
    exo::PhaseEstimate estimate{};

    const double dt = 0.02;
    for (int i = 0; i < 300; ++i) {
        fill_sine_features(features, i * dt, 0.9);
        estimator.update(features, dt, true, exo::AssistState::Active);
    }

    fill_sine_features(features, 300.0 * dt, 0.9);
    estimate = estimator.update(features, dt, true, exo::AssistState::Stopping);
    assert(!estimate.anchor_frequency_updated);
}

void test_phase_estimator_ramp_uses_reduced_gain() {
    exo::ControlConfig config;
    exo::GaitFeatures features{};

    const double dt = 0.02;
    double max_active_correction_hz = 0.0;
    double max_ramp_correction_hz = 0.0;

    {
        exo::PhaseEstimator active_estimator(config.phase);
        for (int i = 0; i < 400; ++i) {
            fill_sine_features(features, i * dt, 0.9);
            const exo::PhaseEstimate estimate =
                active_estimator.update(features, dt, true, exo::AssistState::Active);
            max_active_correction_hz = std::max(max_active_correction_hz, std::abs(estimate.omega_correction_hz));
        }
    }

    {
        exo::PhaseEstimator ramp_estimator(config.phase);
        for (int i = 0; i < 400; ++i) {
            fill_sine_features(features, i * dt, 0.9);
            const exo::PhaseEstimate estimate =
                ramp_estimator.update(features, dt, true, exo::AssistState::Ramp);
            max_ramp_correction_hz = std::max(max_ramp_correction_hz, std::abs(estimate.omega_correction_hz));
        }
    }

    assert(max_active_correction_hz > 1e-6);
    assert(max_ramp_correction_hz > 1e-6);
    assert(max_ramp_correction_hz < max_active_correction_hz);
}


void test_stop_detector_ignores_static_large_spread_and_triggers_on_low_velocity() {
    exo::ControlConfig config;
    config.stop.velocity_threshold_rad_s = 0.08;
    config.stop.enter_hold_seconds = 0.16;
    config.stop.velocity_filter_alpha = 1.0;
    exo::StopDetector detector(config.stop);

    exo::ExoState state{};
    state.left.position_rad = 0.45;
    state.right.position_rad = -0.35;
    state.left.velocity_rad_s = 0.02;
    state.right.velocity_rad_s = -0.03;

    exo::StopDecision decision{};
    for (int i = 0; i < 8; ++i) {
        decision = detector.update(state, 0.02);
    }

    assert(decision.stop_requested);
    assert(!decision.phase_tracking_enabled);
    assert(decision.average_abs_velocity_rad_s < config.stop.velocity_threshold_rad_s);
}

void test_stop_detector_exits_when_joint_velocity_returns() {
    exo::ControlConfig config;
    config.stop.velocity_threshold_rad_s = 0.08;
    config.stop.exit_velocity_threshold_rad_s = 0.16;
    config.stop.enter_hold_seconds = 0.10;
    config.stop.exit_hold_seconds = 0.06;
    config.stop.velocity_filter_alpha = 1.0;
    exo::StopDetector detector(config.stop);

    exo::ExoState state{};
    state.left.velocity_rad_s = 0.01;
    state.right.velocity_rad_s = 0.01;
    for (int i = 0; i < 6; ++i) {
        detector.update(state, 0.02);
    }

    state.left.velocity_rad_s = 0.30;
    state.right.velocity_rad_s = -0.28;
    exo::StopDecision decision{};
    for (int i = 0; i < 4; ++i) {
        decision = detector.update(state, 0.02);
    }

    assert(!decision.stop_requested);
    assert(decision.phase_tracking_enabled);
}

void test_assist_state_machine_stop_request_uses_stopping_state_with_output_until_scale_reaches_zero() {
    exo::ControlConfig config;
    config.assist.warmup_anchor_count = 1;
    config.assist.ramp_up_rate_per_s = 10.0;
    config.assist.ramp_down_rate_per_s = 2.0;
    exo::AssistStateMachine machine(config.assist);

    exo::AssistInputs inputs{};
    inputs.motion_confidence = 0.9;
    inputs.phase_valid = true;
    inputs.anchor_detected = true;

    exo::AssistOutput output = machine.update(inputs, 0.02);
    output = machine.update(inputs, 0.02);
    inputs.anchor_detected = false;
    output = machine.update(inputs, 0.20);
    assert(output.torque_scale > 0.0);

    inputs.stop_requested = true;
    output = machine.update(inputs, 0.02);

    assert(output.state == exo::AssistState::Stopping);
    assert(output.torque_scale > 0.0);
    assert(output.allow_output);
}


void test_assist_state_machine_freeze_request_disables_output_immediately() {
    exo::ControlConfig config;
    config.assist.warmup_anchor_count = 1;
    config.assist.ramp_up_rate_per_s = 10.0;
    exo::AssistStateMachine machine(config.assist);

    exo::AssistInputs inputs{};
    inputs.motion_confidence = 0.9;
    inputs.phase_valid = true;
    inputs.anchor_detected = true;

    exo::AssistOutput output = machine.update(inputs, 0.02);
    output = machine.update(inputs, 0.02);
    inputs.anchor_detected = false;
    output = machine.update(inputs, 0.20);
    assert(output.torque_scale > 0.0);

    inputs.freeze_requested = true;
    output = machine.update(inputs, 0.02);

    assert(output.state == exo::AssistState::Frozen);
    assert(output.torque_scale == 0.0);
    assert(!output.allow_output);
}



void test_freeze_manager_does_not_freeze_before_prior_motion() {
    exo::ControlConfig config;
    exo::FreezeManager manager(config.freeze);

    exo::IntentEstimate quiet{};
    quiet.motion_confidence = 0.05;
    quiet.stop_probability = 0.95;

    exo::FreezeDecision decision{};
    for (int i = 0; i < 30; ++i) {
        decision = manager.update(quiet, 0.02);
    }

    assert(decision.state == exo::FreezeState::Live);
    assert(!decision.freeze_requested);
    assert(decision.phase_tracking_enabled);
}

void test_freeze_manager_reset_to_live_clears_natural_stop_freeze() {
    exo::ControlConfig config;
    exo::FreezeManager manager(config.freeze);

    exo::IntentEstimate walk{};
    walk.motion_confidence = 0.85;
    walk.stop_probability = 0.10;
    manager.update(walk, 0.02);

    exo::IntentEstimate stop{};
    stop.motion_confidence = 0.05;
    stop.stop_probability = 0.95;

    exo::FreezeDecision decision{};
    for (int i = 0; i < 20; ++i) {
        decision = manager.update(stop, 0.02);
    }
    assert(decision.freeze_requested);

    decision = manager.resetToLive();

    assert(decision.state == exo::FreezeState::Live);
    assert(!decision.freeze_requested);
    assert(decision.phase_tracking_enabled);
    assert(!decision.recovery_active);
}


void test_stop_torque_limiter_moves_torque_toward_zero_without_sign_flip() {
    exo::ControlConfig config;
    config.stop.max_stop_torque_rate_nm_s = 40.0;
    exo::StopTorqueLimiter limiter(config.stop);

    exo::TorqueCommand previous{};
    previous.left_nm = 4.0;
    previous.right_nm = 1.0;

    exo::TorqueCommand limited = limiter.update(previous, 0.02);

    assert(std::abs(limited.left_nm - 3.2) < 1e-9);
    assert(std::abs(limited.right_nm - 0.2) < 1e-9);

    limited = limiter.update(limited, 0.02);
    assert(std::abs(limited.left_nm - 2.4) < 1e-9);
    assert(limited.right_nm == 0.0);
}

void test_stop_torque_limiter_handles_negative_torque_without_overshoot() {
    exo::ControlConfig config;
    config.stop.max_stop_torque_rate_nm_s = 30.0;
    exo::StopTorqueLimiter limiter(config.stop);

    exo::TorqueCommand previous{};
    previous.left_nm = -0.4;
    previous.right_nm = -2.0;

    exo::TorqueCommand limited = limiter.update(previous, 0.02);

    assert(limited.left_nm == 0.0);
    assert(std::abs(limited.right_nm + 1.4) < 1e-9);
}

} // namespace

int main() {
    test_intent_detector_prefers_motion_over_stop();
    test_intent_detector_prefers_stop_when_quiet();
    test_torque_profile_is_phase_symmetric();
    test_stop_torque_limiter_moves_torque_toward_zero_without_sign_flip();
    test_stop_torque_limiter_handles_negative_torque_without_overshoot();
    test_assist_state_machine_reaches_active_after_warmup();
    test_freeze_manager_uses_hysteresis();
    test_freeze_manager_does_not_freeze_before_prior_motion();
    test_freeze_manager_reset_to_live_clears_natural_stop_freeze();
    test_gait_feature_extractor_generates_velocity();
    test_phase_estimator_tracks_signal();
    test_phase_estimator_applies_anchor_frequency_updates_during_active_walking();
    test_phase_estimator_no_anchor_frequency_update_when_stop_probability_high();
    test_phase_estimator_delayed_confirm_rejects_single_frame_spike();
    test_phase_estimator_builds_startup_prior_from_zero_cross();
    test_phase_estimator_applies_startup_prior_on_ramp_entry();
    test_phase_estimator_startup_prior_skipped_when_stop_probability_high();
    test_phase_estimator_applies_deferred_frequency_on_ramp_after_tracking();
    test_phase_estimator_reports_low_confidence_reject_reason();
    test_phase_estimator_no_anchor_frequency_update_when_stopping();
    test_phase_estimator_ramp_uses_reduced_gain();
    test_stop_detector_ignores_static_large_spread_and_triggers_on_low_velocity();
    test_stop_detector_exits_when_joint_velocity_returns();
    test_assist_state_machine_stop_request_uses_stopping_state_with_output_until_scale_reaches_zero();
    test_assist_state_machine_freeze_request_disables_output_immediately();
    return 0;
}
