// 按估计步频映射基础增益，用 cos 相位在左右腿之间分配对称助力，并施加 lead 角与力矩上限。

#pragma once

#include "config/ControlConfig.h"

namespace exo {

struct TorqueCommand {
    double left_nm = 0.0;
    double right_nm = 0.0;
};

class TorqueProfile {
public:
    explicit TorqueProfile(const TorqueConfig& config);

    TorqueCommand compute(double phase_rad, double frequency_hz, double torque_scale, bool allow_output) const;

private:
    TorqueConfig config_;
};

} // namespace exo
