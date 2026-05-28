// 从 ExoState 提取简单步态特征：左右髋角和经低通，再算「幅度」与变化率。

#pragma once

#include "config/ControlConfig.h"

namespace exo {

struct ExoState;
struct GaitFeatures;

class GaitFeatureExtractor {
public:
    explicit GaitFeatureExtractor(const PhaseConfig& config);

    GaitFeatures update(const ExoState& state, double dt_s);

private:
    // 一阶低通系数，由截止频率与 dt 推导。
    double alpha(double dt_s) const;

    PhaseConfig config_;
    double filtered_phase_signal_rad_ = 0.0;
    double last_filtered_phase_signal_rad_ = 0.0;
    bool initialized_ = false;
};

} // namespace exo
