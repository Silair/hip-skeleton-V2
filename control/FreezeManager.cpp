#include "control/FreezeManager.h"

#include "control/GaitFeatures.h"

namespace exo {

FreezeManager::FreezeManager(const FreezeConfig& config)
    : config_(config) {}

FreezeDecision FreezeManager::update(const IntentEstimate& intent, double dt_s) {
    if (intent.motion_confidence >= config_.resume_motion_confidence) {
        motion_seen_ = true;
    }

    switch (state_) {
    case FreezeState::Live:
        // 持续「像要停」超过 enter_hold 才进入 Frozen，避免短暂犹豫误触发。
        if (motion_seen_ && intent.stop_probability >= config_.enter_stop_probability) {
            enter_timer_s_ += dt_s;
            if (enter_timer_s_ >= config_.enter_hold_seconds) {
                state_ = FreezeState::Frozen;
                enter_timer_s_ = 0.0;
            }
        } else {
            enter_timer_s_ = 0.0;
        }
        break;
    case FreezeState::Frozen:
        // 退出条件更严（更低 stop_prob + 更高 motion_conf），并需持续 resume_hold。
        if (intent.stop_probability <= config_.exit_stop_probability && intent.motion_confidence >= config_.resume_motion_confidence) {
            resume_timer_s_ += dt_s;
            if (resume_timer_s_ >= config_.resume_hold_seconds) {
                state_ = FreezeState::Recovery;
                resume_timer_s_ = 0.0;
            }
        } else {
            resume_timer_s_ = 0.0;
        }
        break;
    case FreezeState::Recovery:
        if (intent.motion_confidence >= config_.resume_motion_confidence) {
            resume_timer_s_ += dt_s;
            if (resume_timer_s_ >= config_.resume_hold_seconds) {
                state_ = FreezeState::Live;
                resume_timer_s_ = 0.0;
            }
        } else {
            state_ = FreezeState::Frozen;
            resume_timer_s_ = 0.0;
        }
        break;
    }

    FreezeDecision decision{};
    decision.state = state_;
    decision.freeze_requested = (state_ == FreezeState::Frozen);
    decision.phase_tracking_enabled = (state_ != FreezeState::Frozen);
    decision.recovery_active = (state_ == FreezeState::Recovery);
    return decision;
}


FreezeDecision FreezeManager::resetToLive() {
    state_ = FreezeState::Live;
    enter_timer_s_ = 0.0;
    resume_timer_s_ = 0.0;
    motion_seen_ = false;

    FreezeDecision decision{};
    decision.state = state_;
    decision.freeze_requested = false;
    decision.phase_tracking_enabled = true;
    decision.recovery_active = false;
    return decision;
}

} // namespace exo
