// 纯控制链回归：意图、力矩对称、助力状态机、冻结滞回、特征提取、相位跟踪（无 CAN/电机）。

#include <cassert>
#include <cmath>

#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatureExtractor.h"
#include "control/GaitFeatures.h"
#include "control/IntentDetector.h"
#include "control/PhaseEstimator.h"
#include "control/TorqueProfile.h"
#include "config/ControlConfig.h"
#include "hardware/JointTypes.h"

namespace {

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

    exo::IntentEstimate stop{};
    stop.motion_confidence = 0.05;
    stop.stop_probability = 0.95;

    exo::FreezeDecision decision{};
    for (int i = 0; i < 20; ++i) {
        decision = manager.update(stop, 0.02);
    }
    assert(decision.freeze_requested);

    exo::IntentEstimate walk{};
    walk.motion_confidence = 0.85;
    walk.stop_probability = 0.10;
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
        const double t = i * 0.02;
        features.filtered_phase_signal_rad = std::sin(t * 2.0 * M_PI * 0.9);
        features.spread_deg = std::abs(features.filtered_phase_signal_rad) * 57.2957795130823;
        features.phase_velocity_deg_s = std::abs(std::cos(t * 2.0 * M_PI * 0.9)) * 57.2957795130823;
        estimate = estimator.update(features, 0.02, true);
    }

    assert(estimate.valid);
    assert(estimate.frequency_hz > 0.5);
}

} // namespace

int main() {
    test_intent_detector_prefers_motion_over_stop();
    test_intent_detector_prefers_stop_when_quiet();
    test_torque_profile_is_phase_symmetric();
    test_assist_state_machine_reaches_active_after_warmup();
    test_freeze_manager_uses_hysteresis();
    test_gait_feature_extractor_generates_velocity();
    test_phase_estimator_tracks_signal();
    return 0;
}
