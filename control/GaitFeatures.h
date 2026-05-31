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

// anchor_type: 0 = none, 1 = peak, 2 = valley.
enum class PhaseAnchorType : int {
    None = 0,
    Peak = 1,
    Valley = 2,
};

// 自适应振荡器输出的步态相位、频率、幅值及是否可信；anchor_detected 表示检测到步态锚点。
struct PhaseEstimate {
    // phase_rad == phi_final_rad (torque-facing phase).
    double phase_rad = 0.0;
    double phi_gp_rad = 0.0;
    double phi_e_rad = 0.0;
    double phi_final_rad = 0.0;
    double frequency_hz = 0.0;
    double amplitude_rad = 0.0;
    double ao_signal_estimate_rad = 0.0;
    double ao_signal_error_rad = 0.0;
    bool valid = false;
    bool phi_e_active = false;
    bool phi_e_latched = false;
    bool phi_e_timed_out = false;
    bool phi_e_gate = false;
    double ce_latch_rad = 0.0;
    double target_phi_rad = 0.0;
    double phi_e_error_rad = 0.0;
    double phi_e_dot_limited_rad_s = 0.0;
    int anchor_type = 0;
    bool anchor_detected = false;
    bool anchor_candidate = false;
    bool anchor_frequency_updated = false;
    bool anchor_rejected = false;
    double anchor_measured_frequency_hz = 0.0;
    double anchor_confidence = 0.0;
    double omega_correction_hz = 0.0;
    int anchor_reject_reason = 0;
    bool startup_prior_valid = false;
    bool startup_prior_candidate = false;
    bool startup_prior_applied = false;
    double startup_prior_frequency_hz = 0.0;
    double startup_prior_confidence = 0.0;
};

} // namespace exo
