// 控制管道中传递的轻量结构体：步态特征、意图估计、相位估计（与算法模块一一对应）。

#pragma once

namespace exo {

// 由左右髋角合成的标量信号及其滤波、展宽、变化率等，供相位与意图模块使用。
struct GaitFeatures {
    double phase_signal_rad = 0.0;
    double filtered_phase_signal_rad = 0.0;
    double spread_deg = 0.0;
    // 运动强度使用绝对速度；相位锚点检测使用 signed_phase_velocity_deg_s 保留方向。
    double phase_velocity_deg_s = 0.0;
    double signed_phase_velocity_deg_s = 0.0;
    double amplitude_rad = 0.0;
    double frequency_hz = 0.0;
};

// 0~1：多像在周期性行走；stop_probability 为其补（粗略「停住」置信度）。
struct IntentEstimate {
    double motion_confidence = 0.0;
    double stop_probability = 1.0;
};

// 自适应振荡器输出的步态相位、频率、幅值及是否可信；anchor_detected 表示检测到步态锚点。
struct PhaseEstimate {
    double phase_rad = 0.0;
    double frequency_hz = 0.0;
    double amplitude_rad = 0.0;
    double ao_signal_estimate_rad = 0.0;
    double ao_signal_error_rad = 0.0;
    bool valid = false;
    bool anchor_detected = false;
    bool anchor_candidate = false;
    bool anchor_frequency_updated = false;
    bool anchor_rejected = false;
    double anchor_measured_frequency_hz = 0.0;
    double anchor_confidence = 0.0;
    double omega_correction_hz = 0.0;
    int anchor_reject_reason = 0;
};

} // namespace exo
