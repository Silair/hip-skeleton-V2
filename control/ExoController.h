// V2 主控制器：固定频率循环里串联「读状态 → 特征 → 相位 → 意图 → 冻结 → 助力门控 → 力矩 → 写指令 → 记日志」。

#pragma once

#include "config/ControlConfig.h"
#include "control/AssistStateMachine.h"
#include "control/FreezeManager.h"
#include "control/GaitFeatureExtractor.h"
#include "control/IntentDetector.h"
#include "control/PhaseEstimator.h"
#include "control/TorqueProfile.h"
#include "hardware/IExoHardware.h"
#include "logging/ExoLogger.h"
#include "logging/Clock.h"

namespace exo {

class ExoController {
public:
    ExoController(const ControlConfig& config, IExoHardware& hardware, ExoLogger& logger);

    bool initialize();
    bool run();
    void shutdown();

private:
    ControlConfig config_;
    IExoHardware& hardware_;
    ExoLogger& logger_;
    GaitFeatureExtractor feature_extractor_;
    PhaseEstimator phase_estimator_;
    IntentDetector intent_detector_;
    FreezeManager freeze_manager_;
    AssistStateMachine assist_state_machine_;
    TorqueProfile torque_profile_;
    Clock clock_;
};

} // namespace exo
