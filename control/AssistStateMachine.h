// 助力输出门控：从透明（无力）→ 跟踪步态 → 累计锚点热身 → 爬升力矩比例 → 全力 → 可因冻结/故障回落。

#pragma once

#include "config/ControlConfig.h"

namespace exo {

enum class AssistState {
    Transparent,
    Tracking,
    Ramp,
    Active,
    Frozen,
    Fault,
};

struct AssistInputs {
    double motion_confidence = 0.0;
    bool phase_valid = false;
    bool anchor_detected = false;
    bool freeze_requested = false;
    bool faulted = false;
};

struct AssistOutput {
    AssistState state = AssistState::Transparent;
    double torque_scale = 0.0;
    bool allow_output = false;
};

class AssistStateMachine {
public:
    explicit AssistStateMachine(const AssistConfig& config);

    AssistOutput update(const AssistInputs& inputs, double dt_s);

private:
    AssistConfig config_;
    AssistState state_ = AssistState::Transparent;
    double torque_scale_ = 0.0;
    int warmup_anchor_count_ = 0;
};

} // namespace exo
