// 根据步态特征的多个归一化指标加权得到「运动置信度」，并做一阶平滑抑制噪声。

#pragma once

#include "config/ControlConfig.h"

namespace exo {

struct GaitFeatures;
struct IntentEstimate;

class IntentDetector {
public:
    explicit IntentDetector(const IntentConfig& config);

    IntentEstimate update(const GaitFeatures& features, double dt_s);

private:
    IntentConfig config_;
    double smoothed_motion_confidence_ = 0.0;
};

} // namespace exo
