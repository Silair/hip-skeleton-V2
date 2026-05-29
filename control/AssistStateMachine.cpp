#include "control/AssistStateMachine.h"

#include <algorithm>

namespace exo {

AssistStateMachine::AssistStateMachine(const AssistConfig& config)
    : config_(config) {}

AssistOutput AssistStateMachine::update(const AssistInputs& inputs, double dt_s) {
    if (inputs.faulted) {
        state_ = AssistState::Fault;
        torque_scale_ = 0.0;
    } else if (inputs.freeze_requested) {
        state_ = AssistState::Frozen;
        torque_scale_ = 0.0;
        warmup_anchor_count_ = 0;
    } else if (inputs.stop_requested) {
        state_ = AssistState::Stopping;
        torque_scale_ = std::max(0.0, torque_scale_ - config_.ramp_down_rate_per_s * dt_s);
    } else {
        switch (state_) {
        case AssistState::Transparent:
            torque_scale_ = 0.0;
            warmup_anchor_count_ = 0;
            if (inputs.motion_confidence >= config_.motion_entry_confidence && inputs.phase_valid) {
                state_ = AssistState::Tracking;
            }
            break;
        case AssistState::Tracking:
            torque_scale_ = 0.0;
            if (inputs.motion_confidence < config_.motion_exit_confidence || !inputs.phase_valid) {
                state_ = AssistState::Transparent;
                warmup_anchor_count_ = 0;
            } else if (inputs.anchor_detected) {
                ++warmup_anchor_count_;
                if (warmup_anchor_count_ >= config_.warmup_anchor_count) {
                    state_ = AssistState::Ramp;
                }
            }
            break;
        case AssistState::Ramp:
            if (inputs.motion_confidence < config_.motion_exit_confidence || !inputs.phase_valid) {
                state_ = AssistState::Transparent;
                torque_scale_ = 0.0;
                warmup_anchor_count_ = 0;
            } else {
                torque_scale_ = std::min(1.0, torque_scale_ + config_.ramp_up_rate_per_s * dt_s);
                if (torque_scale_ >= 0.999) {
                    state_ = AssistState::Active;
                }
            }
            break;
        case AssistState::Active:
            if (inputs.motion_confidence < config_.motion_exit_confidence || !inputs.phase_valid) {
                state_ = AssistState::Transparent;
                torque_scale_ = 0.0;
                warmup_anchor_count_ = 0;
            } else {
                torque_scale_ = 1.0;
            }
            break;
        case AssistState::Stopping:
            if (!inputs.stop_requested) {
                state_ = AssistState::Tracking;
                torque_scale_ = 0.0;
                warmup_anchor_count_ = 0;
            }
            break;
        case AssistState::Frozen:
            if (!inputs.freeze_requested) {
                state_ = AssistState::Tracking;
                torque_scale_ = 0.0;
                warmup_anchor_count_ = 0;
            }
            break;
        case AssistState::Fault:
            break;
        }
    }

    AssistOutput output{};
    output.state = state_;
    output.torque_scale = torque_scale_;
    output.allow_output = (state_ == AssistState::Ramp || state_ == AssistState::Active || state_ == AssistState::Stopping) && torque_scale_ > 0.0;
    return output;
}

} // namespace exo
