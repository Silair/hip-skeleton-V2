// 三态滞回：Live（正常跟踪）→ Frozen（认为用户要停）→ Recovery（恢复中）→ Live。
// 输出给助力状态机与相位估计：是否请求冻结、是否允许相位跟踪、是否在恢复窗口。

#pragma once

#include "config/ControlConfig.h"

namespace exo {

struct FreezeDecision;
struct IntentEstimate;

enum class FreezeState {
    Live,
    Frozen,
    Recovery,
};

struct FreezeDecision {
    FreezeState state = FreezeState::Live;
    bool freeze_requested = false;
    bool phase_tracking_enabled = true;
    bool recovery_active = false;
};

class FreezeManager {
public:
    explicit FreezeManager(const FreezeConfig& config);

    FreezeDecision update(const IntentEstimate& intent, double dt_s);
    FreezeDecision resetToLive();

private:
    FreezeConfig config_;
    FreezeState state_ = FreezeState::Live;
    double enter_timer_s_ = 0.0;
    double resume_timer_s_ = 0.0;
    bool motion_seen_ = false;
};

} // namespace exo
