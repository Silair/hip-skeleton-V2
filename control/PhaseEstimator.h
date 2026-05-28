// 用 MultiHarmonicAO 跟踪滤波后的步态信号，输出连续相位；在步态峰值处做锚点修正。
// tracking_enabled 为 false（如冻结）时可衰减相位偏移，避免错误漂移。

#pragma once

#include "MultiHarmonicAO.h"
#include "config/ControlConfig.h"

namespace exo {

struct GaitFeatures;
struct PhaseEstimate;

class PhaseEstimator {
public:
    explicit PhaseEstimator(const PhaseConfig& config);

    PhaseEstimate update(const GaitFeatures& features, double dt_s, bool tracking_enabled);

private:
    static double wrapAngle(double angle_rad);

    PhaseConfig config_;
    MultiHarmonicAO oscillator_;
    double last_signal_rad_ = 0.0;
    double last_phase_velocity_deg_s_ = 0.0;
    double last_anchor_time_s_ = -10.0;
    double phase_offset_rad_ = 0.0;
    double now_s_ = 0.0;
};

} // namespace exo
